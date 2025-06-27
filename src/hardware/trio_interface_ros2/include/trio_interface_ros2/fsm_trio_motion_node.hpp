#pragma once

#include <rclcpp/rclcpp.hpp>
#include "trio_interface_ros2/fsm_trio_motion_tcp.hpp"

class FSMTrioMotionNode : public rclcpp::Node
{
public:
    FSMTrioMotionNode();

private:
    void timer_callback();
    std::unique_ptr<trio_interface_ros2::FSMTrioMotionTCP> tcp_server_;
    rclcpp::TimerBase::SharedPtr timer_;
};
