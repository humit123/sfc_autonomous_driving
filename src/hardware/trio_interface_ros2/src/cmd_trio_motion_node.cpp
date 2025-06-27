#include "trio_interface_ros2/cmd_trio_motion_node.hpp"

CmdTrioMotionNode::CmdTrioMotionNode() : Node("cmd_trio_motion_node") {
//    tcp_client_ = std::make_unique<trio_interface_ros2::CmdTrioMotionTCP>(shared_from_this());
    
    tcp_client_ = std::make_unique<trio_interface_ros2::CmdTrioMotionTCP>(this->get_logger());
    
    tcp_client_->initServo();

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&CmdTrioMotionNode::twist_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "CMD Trio Motion Node initialized.");
}

void CmdTrioMotionNode::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double vel[2] = { msg->linear.x, msg->linear.y };
    tcp_client_->sendCmd(vel);
}

