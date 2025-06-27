#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "trio_interface_ros2/cmd_trio_motion_tcp.hpp"

class CmdTrioMotionNode : public rclcpp::Node
{
public:
    CmdTrioMotionNode();

private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    std::unique_ptr<trio_interface_ros2::CmdTrioMotionTCP> tcp_client_;
};

