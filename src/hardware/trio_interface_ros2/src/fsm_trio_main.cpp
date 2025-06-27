#include "rclcpp/rclcpp.hpp"
#include "trio_interface_ros2/fsm_trio_motion_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FSMTrioMotionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
