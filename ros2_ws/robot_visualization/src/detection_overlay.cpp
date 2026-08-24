#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace robot_visualization
{

namespace
{

using Image = sensor_msgs::msg::Image;
using CompressedImage = sensor_msgs::msg::CompressedImage;
using DetectionArray = vision_msgs::msg::Detection2DArray;

// Exact class order recovered from the verified yolov8n.onnx metadata in
// /data/projects/perception_models/yolov8n. The frozen detector publishes
// these zero-based COCO indices as decimal strings.
constexpr std::array<const char *, 80> kCoco80Labels = {
  "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
  "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
  "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
  "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
  "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
  "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
  "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
  "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
  "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
  "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book",
  "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
};

bool sameStamp(
  const builtin_interfaces::msg::Time & lhs,
  const builtin_interfaces::msg::Time & rhs)
{
  return lhs.sec == rhs.sec && lhs.nanosec == rhs.nanosec;
}

std::string labelForClassId(const std::string & class_id)
{
  int numeric_id = -1;
  const char * begin = class_id.data();
  const char * end = begin + class_id.size();
  const auto parsed = std::from_chars(begin, end, numeric_id);

  if (
    parsed.ec == std::errc{} && parsed.ptr == end && numeric_id >= 0 &&
    numeric_id < static_cast<int>(kCoco80Labels.size()))
  {
    return kCoco80Labels[static_cast<size_t>(numeric_id)];
  }

  return class_id.empty() ? std::string("unknown") : class_id;
}

}  // namespace

class DetectionOverlay : public rclcpp::Node
{
public:
  DetectionOverlay()
  : Node("detection_overlay")
  {
    max_cache_size_ = static_cast<size_t>(
      declare_parameter<int>("max_cache_size", 30));
    max_wait_ = std::chrono::milliseconds(
      declare_parameter<int>("max_wait_ms", 500));
    compressed_jpeg_quality_ = declare_parameter<int>("compressed_jpeg_quality", 75);
    compressed_max_rate_hz_ = declare_parameter<double>("compressed_max_rate_hz", 12.0);

    if (max_cache_size_ == 0U) {
      throw std::runtime_error("max_cache_size must be greater than zero");
    }
    if (max_wait_.count() <= 0) {
      throw std::runtime_error("max_wait_ms must be greater than zero");
    }
    if (compressed_jpeg_quality_ < 1 || compressed_jpeg_quality_ > 100) {
      throw std::runtime_error("compressed_jpeg_quality must be in [1, 100]");
    }
    if (compressed_max_rate_hz_ <= 0.0) {
      throw std::runtime_error("compressed_max_rate_hz must be greater than zero");
    }
    compressed_min_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / compressed_max_rate_hz_));

    image_publisher_ = create_publisher<Image>(
      "/camera_processing/detection_image",
      rclcpp::SensorDataQoS().keep_last(1));
    compressed_image_publisher_ = create_publisher<CompressedImage>(
      "/camera_processing/detection_image/compressed",
      rclcpp::SensorDataQoS().keep_last(1));

    image_subscription_ = create_subscription<Image>(
      "/camera/color/image_raw",
      rclcpp::SensorDataQoS().keep_last(5),
      std::bind(&DetectionOverlay::imageCallback, this, std::placeholders::_1));

    detection_subscription_ = create_subscription<DetectionArray>(
      "/camera_processing/detections",
      rclcpp::QoS(10),
      std::bind(&DetectionOverlay::detectionCallback, this, std::placeholders::_1));

    flush_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&DetectionOverlay::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Exact-stamp overlay ready: image=/camera/color/image_raw detections="
      "/camera_processing/detections output=/camera_processing/detection_image "
      "compressed=/camera_processing/detection_image/compressed "
      "jpeg_quality=%d compressed_max_rate_hz=%.1f cache=%zu wait_ms=%lld",
      compressed_jpeg_quality_,
      compressed_max_rate_hz_,
      max_cache_size_,
      static_cast<long long>(max_wait_.count()));
  }

private:
  struct PendingFrame
  {
    Image::ConstSharedPtr image;
    DetectionArray::ConstSharedPtr detections;
    std::chrono::steady_clock::time_point received;
  };

  struct ReadyFrame
  {
    Image::ConstSharedPtr image;
    DetectionArray::ConstSharedPtr detections;
    bool unmatched{false};
  };

  void imageCallback(const Image::ConstSharedPtr image)
  {
    std::vector<ReadyFrame> ready;
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      cache_.push_back(PendingFrame{image, nullptr, std::chrono::steady_clock::now()});

      while (cache_.size() > max_cache_size_) {
        ready.push_back(ReadyFrame{cache_.front().image, nullptr, true});
        cache_.pop_front();
        ++cache_pressure_count_;
      }

      collectReadyLocked(std::chrono::steady_clock::now(), ready);
    }
    publishReady(ready);
  }

  void detectionCallback(const DetectionArray::ConstSharedPtr detections)
  {
    std::vector<ReadyFrame> ready;
    bool matched = false;
    uint64_t missing_count = 0;

    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      const auto match = std::find_if(
        cache_.begin(), cache_.end(),
        [&detections](const PendingFrame & frame) {
          return sameStamp(frame.image->header.stamp, detections->header.stamp);
        });

      if (match != cache_.end()) {
        match->detections = detections;
        matched = true;
      } else {
        ++missing_detection_association_count_;
        missing_count = missing_detection_association_count_;
      }

      collectReadyLocked(std::chrono::steady_clock::now(), ready);
    }

    if (!matched) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No cached image with exact detection stamp; dropping detection "
        "(cumulative=%llu)",
        static_cast<unsigned long long>(missing_count));
    }
    publishReady(ready);
  }

  void timerCallback()
  {
    std::vector<ReadyFrame> ready;
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      collectReadyLocked(std::chrono::steady_clock::now(), ready);
    }
    publishReady(ready);
  }

  void collectReadyLocked(
    const std::chrono::steady_clock::time_point now,
    std::vector<ReadyFrame> & ready)
  {
    // Preserve image order. A frame is released with its exact matching
    // detections, or unannotated after a bounded wait.
    while (!cache_.empty()) {
      const bool timed_out = now - cache_.front().received >= max_wait_;
      if (!cache_.front().detections && !timed_out) {
        break;
      }

      const bool unmatched = !cache_.front().detections;
      ready.push_back(
        ReadyFrame{cache_.front().image, cache_.front().detections, unmatched});
      cache_.pop_front();

      if (unmatched) {
        ++unmatched_image_count_;
      }
    }
  }

  void publishReady(const std::vector<ReadyFrame> & ready)
  {
    if (ready.empty()) {
      return;
    }

    bool had_unmatched = false;
    for (size_t index = 0; index < ready.size(); ++index) {
      const ReadyFrame & frame = ready[index];
      had_unmatched = had_unmatched || frame.unmatched;
      // When several cached frames become ready together, only the newest one
      // is eligible for visualization compression. The raw compatibility topic
      // still receives every frame.
      const bool publish_compressed =
        index + 1U == ready.size() && compressedPublishDue();
      publishFrame(frame, publish_compressed);
    }

    if (had_unmatched) {
      uint64_t unmatched = 0;
      uint64_t pressure = 0;
      {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        unmatched = unmatched_image_count_;
        pressure = cache_pressure_count_;
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Publishing unmatched frames without boxes as designed "
        "(timeout=%llu cache_pressure=%llu)",
        static_cast<unsigned long long>(unmatched),
        static_cast<unsigned long long>(pressure));
    }
  }

  bool compressedPublishDue() const
  {
    return !has_compressed_publish_time_ ||
           std::chrono::steady_clock::now() >= next_compressed_publish_;
  }

  void publishFrame(const ReadyFrame & frame, const bool publish_compressed)
  {
    try {
      cv_bridge::CvImagePtr converted = cv_bridge::toCvCopy(
        *frame.image, sensor_msgs::image_encodings::BGR8);

      if (frame.detections) {
        drawDetections(converted->image, *frame.detections);
      }

      converted->header = frame.image->header;
      converted->encoding = sensor_msgs::image_encodings::BGR8;
      image_publisher_->publish(*converted->toImageMsg());

      if (publish_compressed) {
        CompressedImage compressed;
        compressed.header = converted->header;
        compressed.format = "jpeg";
        const std::vector<int> encode_parameters{
          cv::IMWRITE_JPEG_QUALITY, compressed_jpeg_quality_};
        if (!cv::imencode(
            ".jpg", converted->image, compressed.data, encode_parameters))
        {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000, "JPEG encoding returned no image");
          return;
        }
        compressed_image_publisher_->publish(compressed);
        const auto now = std::chrono::steady_clock::now();
        if (has_compressed_publish_time_) {
          next_compressed_publish_ += compressed_min_period_;
        } else {
          next_compressed_publish_ = now + compressed_min_period_;
          has_compressed_publish_time_ = true;
        }
      }
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Image conversion failed: %s", error.what());
    } catch (const cv::Exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "JPEG encoding failed: %s", error.what());
    }
  }

  static void drawDetections(cv::Mat & image, const DetectionArray & detections)
  {
    const cv::Scalar box_color(40, 220, 40);
    const cv::Scalar text_color(255, 255, 255);

    for (const auto & detection : detections.detections) {
      if (detection.results.empty()) {
        continue;
      }

      const double center_x = detection.bbox.center.position.x;
      const double center_y = detection.bbox.center.position.y;
      const double size_x = detection.bbox.size_x;
      const double size_y = detection.bbox.size_y;

      if (
        !std::isfinite(center_x) || !std::isfinite(center_y) ||
        !std::isfinite(size_x) || !std::isfinite(size_y) ||
        size_x <= 0.0 || size_y <= 0.0)
      {
        continue;
      }

      const int left = std::clamp(
        static_cast<int>(std::lround(center_x - size_x * 0.5)), 0, image.cols - 1);
      const int top = std::clamp(
        static_cast<int>(std::lround(center_y - size_y * 0.5)), 0, image.rows - 1);
      const int right = std::clamp(
        static_cast<int>(std::lround(center_x + size_x * 0.5)), 0, image.cols - 1);
      const int bottom = std::clamp(
        static_cast<int>(std::lround(center_y + size_y * 0.5)), 0, image.rows - 1);

      if (right <= left || bottom <= top) {
        continue;
      }

      const auto & hypothesis = detection.results.front().hypothesis;
      std::ostringstream text;
      text << labelForClassId(hypothesis.class_id) << " " << std::fixed <<
        std::setprecision(2) << hypothesis.score;

      cv::rectangle(image, cv::Point(left, top), cv::Point(right, bottom), box_color, 2);

      int baseline = 0;
      const cv::Size text_size = cv::getTextSize(
        text.str(), cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
      const int label_top = std::max(0, top - text_size.height - baseline - 6);
      const int label_right = std::min(image.cols - 1, left + text_size.width + 8);
      cv::rectangle(
        image,
        cv::Point(left, label_top),
        cv::Point(label_right, top),
        box_color,
        cv::FILLED);
      cv::putText(
        image,
        text.str(),
        cv::Point(left + 4, std::max(text_size.height, top - baseline - 3)),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        text_color,
        1,
        cv::LINE_AA);
    }
  }

  size_t max_cache_size_{30};
  std::chrono::milliseconds max_wait_{500};
  int compressed_jpeg_quality_{75};
  double compressed_max_rate_hz_{12.0};
  std::chrono::steady_clock::duration compressed_min_period_{
    std::chrono::milliseconds(83)};
  std::chrono::steady_clock::time_point next_compressed_publish_{};
  bool has_compressed_publish_time_{false};

  std::mutex cache_mutex_;
  std::deque<PendingFrame> cache_;
  uint64_t unmatched_image_count_{0};
  uint64_t missing_detection_association_count_{0};
  uint64_t cache_pressure_count_{0};

  rclcpp::Publisher<Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<CompressedImage>::SharedPtr compressed_image_publisher_;
  rclcpp::Subscription<Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<DetectionArray>::SharedPtr detection_subscription_;
  rclcpp::TimerBase::SharedPtr flush_timer_;
};

}  // namespace robot_visualization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_visualization::DetectionOverlay>());
  rclcpp::shutdown();
  return 0;
}
