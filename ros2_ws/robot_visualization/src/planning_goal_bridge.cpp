#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>

namespace robot_visualization
{

class PlanningGoalBridge : public rclcpp::Node
{
public:
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

  PlanningGoalBridge()
  : Node("planning_goal_bridge")
  {
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/visualization/planned_path",
      rclcpp::QoS(1).reliable().transient_local());

    action_client_ = rclcpp_action::create_client<ComputePathToPose>(
      this, "/compute_path_to_pose");

    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/visualization/goal",
      rclcpp::QoS(10),
      std::bind(&PlanningGoalBridge::goalCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Non-actuating planning bridge ready: /visualization/goal -> "
      "/compute_path_to_pose -> /visualization/planned_path");
  }

private:
  void goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal)
  {
    if (
      goal->header.frame_id.empty() ||
      !std::isfinite(goal->pose.position.x) ||
      !std::isfinite(goal->pose.position.y))
    {
      RCLCPP_ERROR(get_logger(), "Rejected invalid planning goal");
      return;
    }

    if (goal_in_flight_.exchange(true)) {
      RCLCPP_WARN(get_logger(), "Rejected goal because another plan request is in flight");
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Planning goal received: frame=%s x=%.3f y=%.3f",
      goal->header.frame_id.c_str(), goal->pose.position.x, goal->pose.position.y);

    if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      goal_in_flight_ = false;
      RCLCPP_ERROR(get_logger(), "/compute_path_to_pose action server is unavailable");
      return;
    }

    ComputePathToPose::Goal action_goal;
    action_goal.goal = *goal;
    action_goal.planner_id = "GridBased";
    action_goal.use_start = false;
    request_started_ = std::chrono::steady_clock::now();

    rclcpp_action::Client<ComputePathToPose>::SendGoalOptions options;
    options.goal_response_callback =
      std::bind(&PlanningGoalBridge::goalResponseCallback, this, std::placeholders::_1);
    options.result_callback =
      std::bind(&PlanningGoalBridge::resultCallback, this, std::placeholders::_1);

    try {
      action_client_->async_send_goal(action_goal, options);
    } catch (const std::exception & error) {
      goal_in_flight_ = false;
      RCLCPP_ERROR(get_logger(), "Failed to send planning goal: %s", error.what());
    }
  }

  void goalResponseCallback(GoalHandle::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      goal_in_flight_ = false;
      RCLCPP_ERROR(get_logger(), "ComputePathToPose goal rejected");
      return;
    }

    RCLCPP_INFO(get_logger(), "ComputePathToPose goal accepted");
  }

  void resultCallback(const GoalHandle::WrappedResult & result)
  {
    goal_in_flight_ = false;

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result) {
      RCLCPP_ERROR(
        get_logger(), "ComputePathToPose failed with action result code %d",
        static_cast<int>(result.code));
      return;
    }

    const nav_msgs::msg::Path & path = result.result->path;
    const double path_length = calculatePathLength(path);
    const double planner_seconds =
      static_cast<double>(result.result->planning_time.sec) +
      static_cast<double>(result.result->planning_time.nanosec) * 1.0e-9;
    const double round_trip_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - request_started_).count();

    path_publisher_->publish(path);

    RCLCPP_INFO(
      get_logger(),
      "ComputePathToPose succeeded: frame=%s poses=%zu length=%.3f m "
      "planning_time=%.6f s round_trip=%.2f ms",
      path.header.frame_id.c_str(), path.poses.size(), path_length,
      planner_seconds, round_trip_ms);
  }

  static double calculatePathLength(const nav_msgs::msg::Path & path)
  {
    double length = 0.0;
    for (size_t index = 1; index < path.poses.size(); ++index) {
      const auto & previous = path.poses[index - 1].pose.position;
      const auto & current = path.poses[index].pose.position;
      length += std::hypot(current.x - previous.x, current.y - previous.y);
    }
    return length;
  }

  std::atomic_bool goal_in_flight_{false};
  std::chrono::steady_clock::time_point request_started_{};

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr action_client_;
};

}  // namespace robot_visualization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_visualization::PlanningGoalBridge>());
  rclcpp::shutdown();
  return 0;
}
