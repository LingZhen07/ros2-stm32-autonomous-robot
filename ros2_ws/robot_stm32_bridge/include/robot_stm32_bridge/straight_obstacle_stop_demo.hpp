#pragma once

#include <memory>

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

namespace robot_stm32_bridge {

std::shared_ptr<rclcpp::Node> make_straight_obstacle_stop_demo(
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

} // namespace robot_stm32_bridge
