#include <acl/acl.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace camera_processing
{

namespace
{

constexpr int kCameraWidth = 640;
constexpr int kCameraHeight = 480;

constexpr int kModelWidth = 640;
constexpr int kModelHeight = 640;
constexpr int kModelChannels = 3;

constexpr int kYoloAttributes = 84;
constexpr int kYoloCandidates = 8400;
constexpr int kYoloClasses = 80;

constexpr int kLetterboxPadX = 0;
constexpr int kLetterboxPadY = 80;
constexpr float kLetterboxScale = 1.0F;

constexpr float kPaddingValue = 114.0F / 255.0F;

constexpr size_t kExpectedInputFloatCount =
    1ULL * kModelChannels * kModelHeight * kModelWidth;

constexpr size_t kExpectedOutputFloatCount =
    1ULL * kYoloAttributes * kYoloCandidates;

constexpr size_t kExpectedInputBytes =
    kExpectedInputFloatCount * sizeof(float);

constexpr size_t kExpectedOutputBytes =
    kExpectedOutputFloatCount * sizeof(float);

void checkAcl(aclError ret, const std::string & operation)
{
  if (ret != ACL_SUCCESS) {
    throw std::runtime_error(
            operation + " failed, aclError=" + std::to_string(ret));
  }
}

std::string aclDataTypeToString(aclDataType type)
{
  if (type == ACL_FLOAT) {
    return "ACL_FLOAT";
  }

  return "aclDataType(" +
         std::to_string(static_cast<int>(type)) +
         ")";
}

std::string dimsToString(const aclmdlIODims & dims)
{
  std::ostringstream oss;
  oss << "[";

  for (size_t i = 0; i < dims.dimCount; ++i) {
    if (i != 0) {
      oss << ", ";
    }
    oss << dims.dims[i];
  }

  oss << "]";
  return oss.str();
}

bool dimsEqual(
  const aclmdlIODims & dims,
  const std::vector<int64_t> & expected)
{
  if (dims.dimCount != expected.size()) {
    return false;
  }

  for (size_t i = 0; i < expected.size(); ++i) {
    if (dims.dims[i] != expected[i]) {
      return false;
    }
  }

  return true;
}

struct Detection
{
  float x1{0.0F};
  float y1{0.0F};
  float x2{0.0F};
  float y2{0.0F};

  float score{0.0F};
  int class_id{-1};
};

float boxArea(const Detection & box)
{
  const float width = std::max(0.0F, box.x2 - box.x1);
  const float height = std::max(0.0F, box.y2 - box.y1);
  return width * height;
}

float intersectionOverUnion(
  const Detection & a,
  const Detection & b)
{
  const float ix1 = std::max(a.x1, b.x1);
  const float iy1 = std::max(a.y1, b.y1);
  const float ix2 = std::min(a.x2, b.x2);
  const float iy2 = std::min(a.y2, b.y2);

  const float iw = std::max(0.0F, ix2 - ix1);
  const float ih = std::max(0.0F, iy2 - iy1);

  const float intersection = iw * ih;
  const float union_area = boxArea(a) + boxArea(b) - intersection;

  if (union_area <= 0.0F) {
    return 0.0F;
  }

  return intersection / union_area;
}

std::vector<Detection> classWiseNms(
  std::vector<Detection> candidates,
  float iou_threshold,
  size_t max_detections)
{
  std::sort(
    candidates.begin(),
    candidates.end(),
    [](const Detection & a, const Detection & b) {
      return a.score > b.score;
    });

  std::vector<Detection> result;
  result.reserve(std::min(max_detections, candidates.size()));

  std::vector<bool> suppressed(candidates.size(), false);

  for (size_t i = 0; i < candidates.size(); ++i) {
    if (suppressed[i]) {
      continue;
    }

    result.push_back(candidates[i]);

    if (result.size() >= max_detections) {
      break;
    }

    for (size_t j = i + 1; j < candidates.size(); ++j) {
      if (suppressed[j]) {
        continue;
      }

      if (candidates[i].class_id != candidates[j].class_id) {
        continue;
      }

      if (intersectionOverUnion(candidates[i], candidates[j]) >
        iou_threshold)
      {
        suppressed[j] = true;
      }
    }
  }

  return result;
}

}  // namespace


class AscendYoloEngine
{
public:
  AscendYoloEngine(
    const std::string & model_path,
    int device_id,
    float confidence_threshold,
    float nms_iou_threshold,
    size_t max_detections,
    rclcpp::Logger logger)
  : model_path_(model_path),
    device_id_(device_id),
    confidence_threshold_(confidence_threshold),
    nms_iou_threshold_(nms_iou_threshold),
    max_detections_(max_detections),
    logger_(logger)
  {
    try {
      initialize();
    } catch (...) {
      cleanup();
      throw;
    }
  }

  ~AscendYoloEngine()
  {
    cleanup();
  }

  AscendYoloEngine(const AscendYoloEngine &) = delete;
  AscendYoloEngine & operator=(const AscendYoloEngine &) = delete;

  std::vector<Detection> infer(const sensor_msgs::msg::Image & image)
  {
    // Important if this node is ever moved to a MultiThreadedExecutor.
    checkAcl(
      aclrtSetCurrentContext(context_),
      "aclrtSetCurrentContext");

    float * preprocess_destination = nullptr;

    if (run_mode_ == ACL_HOST) {
      preprocess_destination =
        static_cast<float *>(input_host_);
    } else {
      // In ACL_DEVICE mode the application can directly populate the
      // device-side model input buffer.
      preprocess_destination =
        static_cast<float *>(input_device_);
    }

    preprocessRgb640x480(image, preprocess_destination);

    if (run_mode_ == ACL_HOST) {
      checkAcl(
        aclrtMemcpy(
          input_device_,
          input_bytes_,
          input_host_,
          input_bytes_,
          ACL_MEMCPY_HOST_TO_DEVICE),
        "aclrtMemcpy HOST_TO_DEVICE");
    }

    checkAcl(
      aclmdlExecuteAsync(
        model_id_,
        input_dataset_,
        output_dataset_,
        stream_),
      "aclmdlExecuteAsync");

    checkAcl(
      aclrtSynchronizeStream(stream_),
      "aclrtSynchronizeStream");

    const float * output = nullptr;

    if (run_mode_ == ACL_HOST) {
      checkAcl(
        aclrtMemcpy(
          output_host_,
          output_bytes_,
          output_device_,
          output_bytes_,
          ACL_MEMCPY_DEVICE_TO_HOST),
        "aclrtMemcpy DEVICE_TO_HOST");

      output = static_cast<const float *>(output_host_);
    } else {
      output = static_cast<const float *>(output_device_);
    }

    return postprocess(output);
  }

private:
  void initialize()
  {
    RCLCPP_INFO(
      logger_,
      "Initializing Ascend YOLO engine: model=%s device=%d",
      model_path_.c_str(),
      device_id_);

    checkAcl(aclInit(nullptr), "aclInit");
    acl_initialized_ = true;

    checkAcl(
      aclrtSetDevice(device_id_),
      "aclrtSetDevice");
    device_set_ = true;

    checkAcl(
      aclrtCreateContext(&context_, device_id_),
      "aclrtCreateContext");

    checkAcl(
      aclrtSetCurrentContext(context_),
      "aclrtSetCurrentContext");

    checkAcl(
      aclrtCreateStream(&stream_),
      "aclrtCreateStream");

    checkAcl(
      aclrtGetRunMode(&run_mode_),
      "aclrtGetRunMode");

    RCLCPP_INFO(
      logger_,
      "Ascend run mode: %s",
      run_mode_ == ACL_HOST ? "ACL_HOST" : "ACL_DEVICE");

    checkAcl(
      aclmdlLoadFromFile(
        model_path_.c_str(),
        &model_id_),
      "aclmdlLoadFromFile");
    model_loaded_ = true;

    model_desc_ = aclmdlCreateDesc();
    if (model_desc_ == nullptr) {
      throw std::runtime_error("aclmdlCreateDesc returned nullptr");
    }

    checkAcl(
      aclmdlGetDesc(model_desc_, model_id_),
      "aclmdlGetDesc");

    inspectAndValidateModel();

    createModelBuffers();

    RCLCPP_INFO(
      logger_,
      "Ascend YOLO engine initialized successfully");
  }

  void inspectAndValidateModel()
  {
    const size_t input_count =
      aclmdlGetNumInputs(model_desc_);

    const size_t output_count =
      aclmdlGetNumOutputs(model_desc_);

    RCLCPP_INFO(
      logger_,
      "OM descriptor: inputs=%zu outputs=%zu",
      input_count,
      output_count);

    if (input_count != 1) {
      throw std::runtime_error(
              "YOLO v1 expects exactly 1 model input, actual=" +
              std::to_string(input_count));
    }

    if (output_count != 1) {
      throw std::runtime_error(
              "YOLO v1 expects exactly 1 model output, actual=" +
              std::to_string(output_count));
    }

    aclmdlIODims input_dims{};
    checkAcl(
      aclmdlGetInputDims(
        model_desc_,
        0,
        &input_dims),
      "aclmdlGetInputDims");

    aclmdlIODims output_dims{};
    checkAcl(
      aclmdlGetOutputDims(
        model_desc_,
        0,
        &output_dims),
      "aclmdlGetOutputDims");

    input_bytes_ =
      aclmdlGetInputSizeByIndex(model_desc_, 0);

    output_bytes_ =
      aclmdlGetOutputSizeByIndex(model_desc_, 0);

    input_dtype_ =
      aclmdlGetInputDataType(model_desc_, 0);

    output_dtype_ =
      aclmdlGetOutputDataType(model_desc_, 0);

    RCLCPP_INFO(
      logger_,
      "OM input[0]: dims=%s dtype=%s bytes=%zu",
      dimsToString(input_dims).c_str(),
      aclDataTypeToString(input_dtype_).c_str(),
      input_bytes_);

    RCLCPP_INFO(
      logger_,
      "OM output[0]: dims=%s dtype=%s bytes=%zu",
      dimsToString(output_dims).c_str(),
      aclDataTypeToString(output_dtype_).c_str(),
      output_bytes_);

    // v1 deliberately supports the model interface that was actually
    // exported and converted in this project. Do not silently guess.
    if (!dimsEqual(
        input_dims,
        {1, 3, 640, 640}))
    {
      throw std::runtime_error(
              "Unexpected OM input dims. Expected [1,3,640,640], actual=" +
              dimsToString(input_dims));
    }

    if (!dimsEqual(
        output_dims,
        {1, 84, 8400}))
    {
      throw std::runtime_error(
              "Unexpected OM output dims. Expected [1,84,8400], actual=" +
              dimsToString(output_dims));
    }

    if (input_dtype_ != ACL_FLOAT) {
      throw std::runtime_error(
              "v1 expects FP32 OM input. Actual dtype=" +
              aclDataTypeToString(input_dtype_));
    }

    if (output_dtype_ != ACL_FLOAT) {
      throw std::runtime_error(
              "v1 expects FP32 OM output. Actual dtype=" +
              aclDataTypeToString(output_dtype_));
    }

    if (input_bytes_ != kExpectedInputBytes) {
      throw std::runtime_error(
              "Unexpected OM input byte size. Expected=" +
              std::to_string(kExpectedInputBytes) +
              " actual=" +
              std::to_string(input_bytes_));
    }

    if (output_bytes_ != kExpectedOutputBytes) {
      throw std::runtime_error(
              "Unexpected OM output byte size. Expected=" +
              std::to_string(kExpectedOutputBytes) +
              " actual=" +
              std::to_string(output_bytes_));
    }
  }

  void createModelBuffers()
  {
    // Device input.
    checkAcl(
      aclrtMalloc(
        &input_device_,
        input_bytes_,
        ACL_MEM_MALLOC_HUGE_FIRST),
      "aclrtMalloc input");

    // Device output.
    checkAcl(
      aclrtMalloc(
        &output_device_,
        output_bytes_,
        ACL_MEM_MALLOC_HUGE_FIRST),
      "aclrtMalloc output");

    // Host staging buffers only exist in ACL_HOST mode.
    if (run_mode_ == ACL_HOST) {
      checkAcl(
        aclrtMallocHost(
          &input_host_,
          input_bytes_),
        "aclrtMallocHost input");

      checkAcl(
        aclrtMallocHost(
          &output_host_,
          output_bytes_),
        "aclrtMallocHost output");
    }

    input_dataset_ = aclmdlCreateDataset();
    if (input_dataset_ == nullptr) {
      throw std::runtime_error(
              "aclmdlCreateDataset(input) returned nullptr");
    }

    output_dataset_ = aclmdlCreateDataset();
    if (output_dataset_ == nullptr) {
      throw std::runtime_error(
              "aclmdlCreateDataset(output) returned nullptr");
    }

    input_data_buffer_ =
      aclCreateDataBuffer(
        input_device_,
        input_bytes_);

    if (input_data_buffer_ == nullptr) {
      throw std::runtime_error(
              "aclCreateDataBuffer(input) returned nullptr");
    }

    output_data_buffer_ =
      aclCreateDataBuffer(
        output_device_,
        output_bytes_);

    if (output_data_buffer_ == nullptr) {
      throw std::runtime_error(
              "aclCreateDataBuffer(output) returned nullptr");
    }

    checkAcl(
      aclmdlAddDatasetBuffer(
        input_dataset_,
        input_data_buffer_),
      "aclmdlAddDatasetBuffer(input)");

    checkAcl(
      aclmdlAddDatasetBuffer(
        output_dataset_,
        output_data_buffer_),
      "aclmdlAddDatasetBuffer(output)");
  }

  void preprocessRgb640x480(
    const sensor_msgs::msg::Image & image,
    float * destination)
  {
    if (destination == nullptr) {
      throw std::runtime_error(
              "preprocess destination is nullptr");
    }

    if (image.encoding != "rgb8") {
      throw std::runtime_error(
              "Expected rgb8, actual encoding=" +
              image.encoding);
    }

    if (
      image.width != kCameraWidth ||
      image.height != kCameraHeight)
    {
      throw std::runtime_error(
              "v1 expects Astra RGB 640x480, actual=" +
              std::to_string(image.width) +
              "x" +
              std::to_string(image.height));
    }

    constexpr size_t minimum_step =
      kCameraWidth * 3ULL;

    if (image.step < minimum_step) {
      throw std::runtime_error(
              "Invalid RGB image step=" +
              std::to_string(image.step));
    }

    const size_t required_data_size =
      static_cast<size_t>(image.step) *
      static_cast<size_t>(image.height);

    if (image.data.size() < required_data_size) {
      throw std::runtime_error(
              "RGB image data smaller than step*height");
    }

    //
    // Current real camera is already 640x480.
    //
    // Ultralytics-style 640x640 letterbox therefore becomes:
    //
    //   640x480 RGB
    //       ↓
    //   no resize
    //       ↓
    //   top    = 80
    //   bottom = 80
    //   left   = 0
    //   right  = 0
    //
    // Padding value: 114 / 255.
    //
    std::fill(
      destination,
      destination + kExpectedInputFloatCount,
      kPaddingValue);

    constexpr size_t plane_size =
      static_cast<size_t>(kModelWidth) *
      static_cast<size_t>(kModelHeight);

    float * r_plane = destination;
    float * g_plane = destination + plane_size;
    float * b_plane = destination + 2 * plane_size;

    for (int y = 0; y < kCameraHeight; ++y) {
      const uint8_t * src_row =
        image.data.data() +
        static_cast<size_t>(y) *
        static_cast<size_t>(image.step);

      const int dst_y = y + kLetterboxPadY;

      for (int x = 0; x < kCameraWidth; ++x) {
        const size_t src_index =
          static_cast<size_t>(x) * 3ULL;

        const uint8_t r = src_row[src_index + 0];
        const uint8_t g = src_row[src_index + 1];
        const uint8_t b = src_row[src_index + 2];

        const size_t dst_index =
          static_cast<size_t>(dst_y) *
          kModelWidth +
          static_cast<size_t>(x);

        r_plane[dst_index] =
          static_cast<float>(r) / 255.0F;

        g_plane[dst_index] =
          static_cast<float>(g) / 255.0F;

        b_plane[dst_index] =
          static_cast<float>(b) / 255.0F;
      }
    }
  }

  std::vector<Detection> postprocess(
    const float * output) const
  {
    if (output == nullptr) {
      throw std::runtime_error(
              "YOLO output pointer is nullptr");
    }

    std::vector<Detection> candidates;
    candidates.reserve(512);

    //
    // YOLOv8 raw tensor:
    //
    // [1, 84, 8400]
    //
    // channel 0: cx
    // channel 1: cy
    // channel 2: w
    // channel 3: h
    // channel 4..83: class scores
    //
    // Memory access:
    //
    // output[channel * 8400 + candidate]
    //
    for (int candidate = 0;
      candidate < kYoloCandidates;
      ++candidate)
    {
      float best_score =
        -std::numeric_limits<float>::infinity();

      int best_class = -1;

      for (int class_id = 0;
        class_id < kYoloClasses;
        ++class_id)
      {
        const float score =
          output[
          static_cast<size_t>(4 + class_id) *
          kYoloCandidates +
          candidate];

        if (score > best_score) {
          best_score = score;
          best_class = class_id;
        }
      }

      // YOLOv8 exported Detect output:
      // confidence is the max class score.
      // There is no separate YOLOv5-style objectness term here.
      if (
        best_class < 0 ||
        !std::isfinite(best_score) ||
        best_score < confidence_threshold_)
      {
        continue;
      }

      const float cx =
        output[0ULL * kYoloCandidates + candidate];

      const float cy =
        output[1ULL * kYoloCandidates + candidate];

      const float width =
        output[2ULL * kYoloCandidates + candidate];

      const float height =
        output[3ULL * kYoloCandidates + candidate];

      if (
        !std::isfinite(cx) ||
        !std::isfinite(cy) ||
        !std::isfinite(width) ||
        !std::isfinite(height) ||
        width <= 0.0F ||
        height <= 0.0F)
      {
        continue;
      }

      float x1 = cx - width * 0.5F;
      float y1 = cy - height * 0.5F;
      float x2 = cx + width * 0.5F;
      float y2 = cy + height * 0.5F;

      //
      // Undo 640x640 letterbox.
      //
      x1 =
        (x1 - static_cast<float>(kLetterboxPadX)) /
        kLetterboxScale;

      x2 =
        (x2 - static_cast<float>(kLetterboxPadX)) /
        kLetterboxScale;

      y1 =
        (y1 - static_cast<float>(kLetterboxPadY)) /
        kLetterboxScale;

      y2 =
        (y2 - static_cast<float>(kLetterboxPadY)) /
        kLetterboxScale;

      x1 = std::clamp(
        x1,
        0.0F,
        static_cast<float>(kCameraWidth - 1));

      x2 = std::clamp(
        x2,
        0.0F,
        static_cast<float>(kCameraWidth - 1));

      y1 = std::clamp(
        y1,
        0.0F,
        static_cast<float>(kCameraHeight - 1));

      y2 = std::clamp(
        y2,
        0.0F,
        static_cast<float>(kCameraHeight - 1));

      if (x2 <= x1 || y2 <= y1) {
        continue;
      }

      Detection detection;
      detection.x1 = x1;
      detection.y1 = y1;
      detection.x2 = x2;
      detection.y2 = y2;
      detection.score = best_score;
      detection.class_id = best_class;

      candidates.push_back(detection);
    }

    return classWiseNms(
      std::move(candidates),
      nms_iou_threshold_,
      max_detections_);
  }

  void cleanup() noexcept
  {
    //
    // aclDataBuffer owns only its descriptor, not the underlying memory.
    //
    if (input_data_buffer_ != nullptr) {
      (void)aclDestroyDataBuffer(input_data_buffer_);
      input_data_buffer_ = nullptr;
    }

    if (output_data_buffer_ != nullptr) {
      (void)aclDestroyDataBuffer(output_data_buffer_);
      output_data_buffer_ = nullptr;
    }

    if (input_dataset_ != nullptr) {
      (void)aclmdlDestroyDataset(input_dataset_);
      input_dataset_ = nullptr;
    }

    if (output_dataset_ != nullptr) {
      (void)aclmdlDestroyDataset(output_dataset_);
      output_dataset_ = nullptr;
    }

    if (input_host_ != nullptr) {
      (void)aclrtFreeHost(input_host_);
      input_host_ = nullptr;
    }

    if (output_host_ != nullptr) {
      (void)aclrtFreeHost(output_host_);
      output_host_ = nullptr;
    }

    if (input_device_ != nullptr) {
      (void)aclrtFree(input_device_);
      input_device_ = nullptr;
    }

    if (output_device_ != nullptr) {
      (void)aclrtFree(output_device_);
      output_device_ = nullptr;
    }

    if (model_desc_ != nullptr) {
      (void)aclmdlDestroyDesc(model_desc_);
      model_desc_ = nullptr;
    }

    if (model_loaded_) {
      (void)aclmdlUnload(model_id_);
      model_loaded_ = false;
    }

    if (stream_ != nullptr) {
      (void)aclrtDestroyStream(stream_);
      stream_ = nullptr;
    }

    if (context_ != nullptr) {
      (void)aclrtDestroyContext(context_);
      context_ = nullptr;
    }

    if (device_set_) {
      (void)aclrtResetDevice(device_id_);
      device_set_ = false;
    }

    if (acl_initialized_) {
      (void)aclFinalize();
      acl_initialized_ = false;
    }
  }

private:
  std::string model_path_;
  int device_id_{0};

  float confidence_threshold_{0.25F};
  float nms_iou_threshold_{0.45F};
  size_t max_detections_{100};

  rclcpp::Logger logger_;

  bool acl_initialized_{false};
  bool device_set_{false};
  bool model_loaded_{false};

  aclrtRunMode run_mode_{ACL_HOST};

  aclrtContext context_{nullptr};
  aclrtStream stream_{nullptr};

  uint32_t model_id_{0};
  aclmdlDesc * model_desc_{nullptr};

  aclDataType input_dtype_{ACL_FLOAT};
  aclDataType output_dtype_{ACL_FLOAT};

  size_t input_bytes_{0};
  size_t output_bytes_{0};

  void * input_device_{nullptr};
  void * output_device_{nullptr};

  void * input_host_{nullptr};
  void * output_host_{nullptr};

  aclmdlDataset * input_dataset_{nullptr};
  aclmdlDataset * output_dataset_{nullptr};

  aclDataBuffer * input_data_buffer_{nullptr};
  aclDataBuffer * output_data_buffer_{nullptr};
};


class RgbdPerceptionNode : public rclcpp::Node
{
public:
  RgbdPerceptionNode()
  : Node("rgbd_perception_node")
  {
    model_path_ =
      declare_parameter<std::string>(
      "model_path",
      "/data/projects/perception_models/yolov8n/yolov8n.om");

    device_id_ =
      declare_parameter<int>(
      "device_id",
      0);

    confidence_threshold_ =
      declare_parameter<double>(
      "confidence_threshold",
      0.25);

    nms_iou_threshold_ =
      declare_parameter<double>(
      "nms_iou_threshold",
      0.45);

    max_detections_ =
      declare_parameter<int>(
      "max_detections",
      100);

    log_every_n_frames_ =
      declare_parameter<int>(
      "log_every_n_frames",
      30);

    if (
      confidence_threshold_ < 0.0 ||
      confidence_threshold_ > 1.0)
    {
      throw std::runtime_error(
              "confidence_threshold must be in [0,1]");
    }

    if (
      nms_iou_threshold_ < 0.0 ||
      nms_iou_threshold_ > 1.0)
    {
      throw std::runtime_error(
              "nms_iou_threshold must be in [0,1]");
    }

    if (max_detections_ <= 0) {
      throw std::runtime_error(
              "max_detections must be > 0");
    }

    engine_ =
      std::make_unique<AscendYoloEngine>(
      model_path_,
      device_id_,
      static_cast<float>(confidence_threshold_),
      static_cast<float>(nms_iou_threshold_),
      static_cast<size_t>(max_detections_),
      get_logger());

    detection_publisher_ =
      create_publisher<vision_msgs::msg::Detection2DArray>(
      "/camera_processing/detections",
      rclcpp::QoS(10));

    auto image_qos =
      rclcpp::SensorDataQoS().keep_last(1);

    image_subscription_ =
      create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw",
      image_qos,
      std::bind(
        &RgbdPerceptionNode::imageCallback,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "rgbd_perception_node ready");

    RCLCPP_INFO(
      get_logger(),
      "Input : /camera/color/image_raw");

    RCLCPP_INFO(
      get_logger(),
      "Output: /camera_processing/detections");

    RCLCPP_INFO(
      get_logger(),
      "Model : %s",
      model_path_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "confidence=%.3f nms_iou=%.3f max_detections=%d",
      confidence_threshold_,
      nms_iou_threshold_,
      max_detections_);
  }

private:
  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    //
    // Engine uses one set of preallocated input/output buffers.
    // Explicit serialization makes this safe even if the executor
    // is changed later.
    //
    std::lock_guard<std::mutex> lock(inference_mutex_);

    try {
      const auto start =
        std::chrono::steady_clock::now();

      const std::vector<Detection> detections =
        engine_->infer(*msg);

      const auto end =
        std::chrono::steady_clock::now();

      publishDetections(
        msg->header,
        detections);

      ++frame_count_;

      if (
        log_every_n_frames_ > 0 &&
        frame_count_ %
        static_cast<uint64_t>(log_every_n_frames_) == 0)
      {
        const double total_ms =
          std::chrono::duration<double, std::milli>(
          end - start).count();

        std::ostringstream summary;

        const size_t shown =
          std::min<size_t>(
          detections.size(),
          5);

        for (size_t i = 0; i < shown; ++i) {
          const auto & det = detections[i];

          summary
            << " [class="
            << det.class_id
            << " score="
            << std::fixed
            << std::setprecision(2)
            << det.score
            << " box=("
            << static_cast<int>(det.x1)
            << ","
            << static_cast<int>(det.y1)
            << ")-("
            << static_cast<int>(det.x2)
            << ","
            << static_cast<int>(det.y2)
            << ")]";
        }

        RCLCPP_INFO(
          get_logger(),
          "frame=%llu total=%.2f ms detections=%zu%s",
          static_cast<unsigned long long>(frame_count_),
          total_ms,
          detections.size(),
          summary.str().c_str());
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Perception callback failed: %s",
        e.what());
    }
  }

  void publishDetections(
    const std_msgs::msg::Header & source_header,
    const std::vector<Detection> & detections)
  {
    vision_msgs::msg::Detection2DArray message;
    message.header = source_header;
    message.detections.reserve(detections.size());

    for (const Detection & det : detections) {
      vision_msgs::msg::Detection2D ros_detection;

      // Preserve camera acquisition timestamp/frame.
      ros_detection.header = source_header;

      const double width =
        static_cast<double>(det.x2 - det.x1);

      const double height =
        static_cast<double>(det.y2 - det.y1);

      ros_detection.bbox.center.position.x =
        static_cast<double>(
        (det.x1 + det.x2) * 0.5F);

      ros_detection.bbox.center.position.y =
        static_cast<double>(
        (det.y1 + det.y2) * 0.5F);

      ros_detection.bbox.center.theta = 0.0;

      ros_detection.bbox.size_x = width;
      ros_detection.bbox.size_y = height;

      vision_msgs::msg::ObjectHypothesisWithPose hypothesis;

      // Standard COCO class index encoded as string.
      // No tracking is implemented in detection v1.
      hypothesis.hypothesis.class_id =
        std::to_string(det.class_id);

      hypothesis.hypothesis.score =
        static_cast<double>(det.score);

      ros_detection.results.push_back(
        std::move(hypothesis));

      ros_detection.id.clear();

      message.detections.push_back(
        std::move(ros_detection));
    }

    detection_publisher_->publish(message);
  }

private:
  std::string model_path_;
  int device_id_{0};

  double confidence_threshold_{0.25};
  double nms_iou_threshold_{0.45};

  int max_detections_{100};
  int log_every_n_frames_{30};

  uint64_t frame_count_{0};

  std::mutex inference_mutex_;

  std::unique_ptr<AscendYoloEngine> engine_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
    image_subscription_;

  rclcpp::Publisher<
    vision_msgs::msg::Detection2DArray>::SharedPtr
    detection_publisher_;
};

}  // namespace camera_processing


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node =
      std::make_shared<
      camera_processing::RgbdPerceptionNode>();

    rclcpp::spin(node);

    node.reset();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("rgbd_perception_node"),
      "Fatal error: %s",
      e.what());

    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}