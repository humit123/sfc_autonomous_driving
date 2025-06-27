#include "trio_interface_ros2/fsm_trio_motion_node.hpp"

FSMTrioMotionNode::FSMTrioMotionNode() : Node("fsm_trio_motion_node") {
    tcp_server_ = std::make_unique<trio_interface_ros2::FSMTrioMotionTCP>();
    tcp_server_->initComm();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&FSMTrioMotionNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "FSM Trio Motion Node initialized.");
}

void FSMTrioMotionNode::timer_callback() {
    tcp_server_->comm();
}

