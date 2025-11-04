#pragma once
#include "rclcpp/rclcpp.hpp"

inline rclcpp::Logger logger() {
    static rclcpp::Logger logger = rclcpp::get_logger("assembler");
    return logger;
}