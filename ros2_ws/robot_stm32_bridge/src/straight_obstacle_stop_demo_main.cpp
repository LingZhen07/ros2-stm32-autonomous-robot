#include <exception>

#include "rclcpp/rclcpp.hpp"
#include "robot_stm32_bridge/straight_obstacle_stop_demo.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(robot_stm32_bridge::make_straight_obstacle_stop_demo());
  } catch (const std::exception &exception) {
    RCLCPP_FATAL(rclcpp::get_logger("straight_obstacle_stop_demo"), "%s",
                 exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
