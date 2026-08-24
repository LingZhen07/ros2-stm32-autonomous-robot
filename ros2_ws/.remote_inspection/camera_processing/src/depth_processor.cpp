#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"

class DepthProcessor : public rclcpp::Node
{
public:
  DepthProcessor()
  : Node("depth_processor")
  {
    // Existing project parameters.
    roi_radius_ = declare_parameter<int>("roi_radius", 2);
    min_valid_pixels_ = declare_parameter<int>("min_valid_pixels", 5);
    log_every_n_frames_ = declare_parameter<int>("log_every_n_frames", 30);

    // Stable-v1 parameters.
    temporal_window_size_ =
      declare_parameter<int>("temporal_window_size", 5);

    invalid_debounce_frames_ =
      declare_parameter<int>("invalid_debounce_frames", 3);

    if (roi_radius_ < 0) {
      throw std::invalid_argument("roi_radius must be >= 0");
    }

    if (min_valid_pixels_ <= 0) {
      throw std::invalid_argument("min_valid_pixels must be > 0");
    }

    if (temporal_window_size_ <= 0) {
      throw std::invalid_argument("temporal_window_size must be > 0");
    }

    if (invalid_debounce_frames_ <= 0) {
      throw std::invalid_argument("invalid_debounce_frames must be > 0");
    }

    if (log_every_n_frames_ <= 0) {
      log_every_n_frames_ = 30;
    }

    point_pub_ =
      create_publisher<geometry_msgs::msg::PointStamped>(
        "/camera_processing/depth/center_point",
        10);

    valid_pub_ =
      create_publisher<std_msgs::msg::Bool>(
        "/camera_processing/depth/valid",
        10);

    depth_sub_ =
      create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth/image_raw",
        rclcpp::SensorDataQoS(),
        std::bind(
          &DepthProcessor::depthCallback,
          this,
          std::placeholders::_1));

    camera_info_sub_ =
      create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/depth/camera_info",
        rclcpp::SensorDataQoS(),
        std::bind(
          &DepthProcessor::cameraInfoCallback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "DepthProcessor stable-v1 started: "
      "ROI radius=%d, min_valid=%d, temporal_window=%d, debounce=%d",
      roi_radius_,
      min_valid_pixels_,
      temporal_window_size_,
      invalid_debounce_frames_);
  }

private:
  struct CameraModel
  {
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};

    uint32_t width{0};
    uint32_t height{0};

    std::string frame_id;
    bool valid{false};
  };

  // --------------------------------------------------------------------------
  // CameraInfo
  // --------------------------------------------------------------------------

  void cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    const double fx = msg->k[0];
    const double fy = msg->k[4];
    const double cx = msg->k[2];
    const double cy = msg->k[5];

    if (!std::isfinite(fx) ||
        !std::isfinite(fy) ||
        !std::isfinite(cx) ||
        !std::isfinite(cy) ||
        fx <= 0.0 ||
        fy <= 0.0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Received invalid Depth CameraInfo intrinsics");

      return;
    }

    std::lock_guard<std::mutex> lock(camera_model_mutex_);

    camera_model_.fx = fx;
    camera_model_.fy = fy;
    camera_model_.cx = cx;
    camera_model_.cy = cy;

    camera_model_.width = msg->width;
    camera_model_.height = msg->height;

    camera_model_.frame_id = msg->header.frame_id;
    camera_model_.valid = true;
  }

  // --------------------------------------------------------------------------
  // Depth image
  // --------------------------------------------------------------------------

  void depthCallback(
    const sensor_msgs::msg::Image::SharedPtr msg)
  {
    ++frame_count_;

    if (!validateDepthImage(*msg)) {
      handleInvalidFrame();
      return;
    }

    CameraModel camera;

    {
      std::lock_guard<std::mutex> lock(camera_model_mutex_);
      camera = camera_model_;
    }

    if (!camera.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Waiting for valid /camera/depth/camera_info");

      handleInvalidFrame();
      return;
    }

    // CameraInfo must correspond to this depth geometry.
    if ((camera.width != 0 && camera.width != msg->width) ||
        (camera.height != 0 && camera.height != msg->height))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Depth Image / CameraInfo size mismatch: image=%ux%u info=%ux%u",
        msg->width,
        msg->height,
        camera.width,
        camera.height);

      handleInvalidFrame();
      return;
    }

    const int center_u = static_cast<int>(msg->width / 2U);
    const int center_v = static_cast<int>(msg->height / 2U);

    int valid_pixel_count = 0;

    const auto spatial_depth_mm =
      computeSpatialMedian(
        *msg,
        center_u,
        center_v,
        valid_pixel_count);

    if (!spatial_depth_mm.has_value()) {
      handleInvalidFrame();

      logFrame(
        valid_pixel_count,
        std::nullopt,
        std::nullopt);

      return;
    }

    // ------------------------------------------------------------------------
    // Fresh measurement exists in THIS frame.
    // Only now may the temporal filter be updated.
    // ------------------------------------------------------------------------

    temporal_depths_mm_.push_back(*spatial_depth_mm);

    while (
      static_cast<int>(temporal_depths_mm_.size()) >
      temporal_window_size_)
    {
      temporal_depths_mm_.pop_front();
    }

    const double filtered_depth_mm =
      median(
        std::vector<double>(
          temporal_depths_mm_.begin(),
          temporal_depths_mm_.end()));

    handleValidFrame();

    // mm -> meter.
    const double z_m = filtered_depth_mm * 0.001;

    geometry_msgs::msg::PointStamped point;

    // IMPORTANT:
    // This timestamp comes from the CURRENT valid depth image.
    // We never manufacture a fresh timestamp from an invalid frame.
    point.header.stamp = msg->header.stamp;

    // Raw depth pixel belongs to the depth optical frame.
    if (!msg->header.frame_id.empty()) {
      point.header.frame_id = msg->header.frame_id;
    } else {
      point.header.frame_id = camera.frame_id;
    }

    point.point.x =
      (static_cast<double>(center_u) - camera.cx) *
      z_m / camera.fx;

    point.point.y =
      (static_cast<double>(center_v) - camera.cy) *
      z_m / camera.fy;

    point.point.z = z_m;

    point_pub_->publish(point);

    logFrame(
      valid_pixel_count,
      spatial_depth_mm,
      filtered_depth_mm);
  }

  // --------------------------------------------------------------------------
  // Input validation
  // --------------------------------------------------------------------------

  bool validateDepthImage(
    const sensor_msgs::msg::Image & msg)
  {
    if (msg.encoding != "16UC1") {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Unsupported depth encoding: '%s', expected 16UC1",
        msg.encoding.c_str());

      return false;
    }

    if (msg.width == 0 || msg.height == 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Invalid zero-sized depth image");

      return false;
    }

    constexpr size_t bytes_per_pixel = sizeof(uint16_t);

    const size_t minimum_row_bytes =
      static_cast<size_t>(msg.width) * bytes_per_pixel;

    if (static_cast<size_t>(msg.step) < minimum_row_bytes) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Invalid depth step=%u, expected at least %zu",
        msg.step,
        minimum_row_bytes);

      return false;
    }

    const size_t required_data_size =
      static_cast<size_t>(msg.step) *
      static_cast<size_t>(msg.height);

    if (msg.data.size() < required_data_size) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Depth buffer too small: data=%zu required=%zu",
        msg.data.size(),
        required_data_size);

      return false;
    }

    return true;
  }

  // --------------------------------------------------------------------------
  // Safe 16UC1 parser
  // --------------------------------------------------------------------------

  uint16_t readDepthMm(
    const sensor_msgs::msg::Image & msg,
    int u,
    int v) const
  {
    const size_t byte_offset =
      static_cast<size_t>(v) *
      static_cast<size_t>(msg.step) +
      static_cast<size_t>(u) * sizeof(uint16_t);

    const uint8_t b0 = msg.data[byte_offset];
    const uint8_t b1 = msg.data[byte_offset + 1U];

    if (msg.is_bigendian) {
      return static_cast<uint16_t>(
        (static_cast<uint16_t>(b0) << 8U) |
        static_cast<uint16_t>(b1));
    }

    return static_cast<uint16_t>(
      static_cast<uint16_t>(b0) |
      (static_cast<uint16_t>(b1) << 8U));
  }

  // --------------------------------------------------------------------------
  // 5x5 spatial median
  // --------------------------------------------------------------------------

  std::optional<double> computeSpatialMedian(
    const sensor_msgs::msg::Image & msg,
    int center_u,
    int center_v,
    int & valid_pixel_count) const
  {
    std::vector<double> depths_mm;

    const int diameter = 2 * roi_radius_ + 1;

    depths_mm.reserve(
      static_cast<size_t>(diameter * diameter));

    for (int dv = -roi_radius_; dv <= roi_radius_; ++dv) {
      const int v = center_v + dv;

      if (v < 0 || v >= static_cast<int>(msg.height)) {
        continue;
      }

      for (int du = -roi_radius_; du <= roi_radius_; ++du) {
        const int u = center_u + du;

        if (u < 0 || u >= static_cast<int>(msg.width)) {
          continue;
        }

        const uint16_t depth_mm =
          readDepthMm(msg, u, v);

        // Astra/OpenNI2 convention:
        // zero means invalid / missing depth.
        if (depth_mm == 0U) {
          continue;
        }

        depths_mm.push_back(
          static_cast<double>(depth_mm));
      }
    }

    valid_pixel_count =
      static_cast<int>(depths_mm.size());

    if (valid_pixel_count < min_valid_pixels_) {
      return std::nullopt;
    }

    return median(std::move(depths_mm));
  }

  // --------------------------------------------------------------------------
  // Median helper
  // --------------------------------------------------------------------------

  static double median(
    std::vector<double> values)
  {
    if (values.empty()) {
      throw std::logic_error(
        "median() called with empty vector");
    }

    const size_t n = values.size();
    const size_t middle = n / 2U;

    std::nth_element(
      values.begin(),
      values.begin() + static_cast<std::ptrdiff_t>(middle),
      values.end());

    const double upper = values[middle];

    if ((n % 2U) != 0U) {
      return upper;
    }

    const auto lower_it =
      std::max_element(
        values.begin(),
        values.begin() + static_cast<std::ptrdiff_t>(middle));

    return (*lower_it + upper) * 0.5;
  }

  // --------------------------------------------------------------------------
  // Debounced validity
  // --------------------------------------------------------------------------

  void handleValidFrame()
  {
    invalid_streak_ = 0;

    if (!debounced_valid_) {
      debounced_valid_ = true;

      RCLCPP_INFO(
        get_logger(),
        "Depth state -> VALID");
    }

    publishValidity();
  }

  void handleInvalidFrame()
  {
    ++invalid_streak_;

    if (
      debounced_valid_ &&
      invalid_streak_ >= invalid_debounce_frames_)
    {
      debounced_valid_ = false;

      // Once validity is genuinely lost, old temporal history is no longer
      // allowed to bias the first measurement after recovery.
      temporal_depths_mm_.clear();

      RCLCPP_WARN(
        get_logger(),
        "Depth state -> INVALID after %d consecutive invalid frames",
        invalid_streak_);
    }

    publishValidity();

    // CRITICAL:
    // Do NOT publish PointStamped here.
    //
    // During debounce, debounced_valid_ may still be true for 1-2 bad frames,
    // but there is no new physical measurement in the current frame.
    //
    // Therefore:
    //   - no old PointStamped is republished
    //   - no old point receives a new timestamp
    //   - downstream freshness semantics remain correct
  }

  void publishValidity()
  {
    std_msgs::msg::Bool msg;
    msg.data = debounced_valid_;
    valid_pub_->publish(msg);
  }

  // --------------------------------------------------------------------------
  // Diagnostic logging
  // --------------------------------------------------------------------------

  void logFrame(
    int valid_pixel_count,
    const std::optional<double> & spatial_mm,
    const std::optional<double> & temporal_mm)
  {
    if ((frame_count_ % static_cast<uint64_t>(log_every_n_frames_)) != 0U) {
      return;
    }

    const int roi_diameter = 2 * roi_radius_ + 1;
    const int roi_total = roi_diameter * roi_diameter;

    if (!spatial_mm.has_value()) {
      RCLCPP_INFO(
        get_logger(),
        "Depth frame: spatial INVALID, valid_pixels=%d/%d, "
        "debounced_valid=%s, invalid_streak=%d",
        valid_pixel_count,
        roi_total,
        debounced_valid_ ? "true" : "false",
        invalid_streak_);

      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Depth frame: valid_pixels=%d/%d, spatial=%.1f mm, "
      "temporal=%.1f mm, window=%zu/%d, state=%s",
      valid_pixel_count,
      roi_total,
      *spatial_mm,
      temporal_mm.value_or(*spatial_mm),
      temporal_depths_mm_.size(),
      temporal_window_size_,
      debounced_valid_ ? "VALID" : "INVALID");
  }

private:
  // Parameters.
  int roi_radius_{2};
  int min_valid_pixels_{5};
  int log_every_n_frames_{30};
  int temporal_window_size_{5};
  int invalid_debounce_frames_{3};

  // Camera model.
  std::mutex camera_model_mutex_;
  CameraModel camera_model_;

  // Temporal depth filter.
  std::deque<double> temporal_depths_mm_;

  // Validity state.
  int invalid_streak_{0};
  bool debounced_valid_{false};

  uint64_t frame_count_{0};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
    depth_sub_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
    camera_info_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
    point_pub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    valid_pub_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<DepthProcessor>());

  rclcpp::shutdown();
  return 0;
}