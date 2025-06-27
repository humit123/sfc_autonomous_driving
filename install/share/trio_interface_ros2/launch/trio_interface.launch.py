from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="trio_interface_ros2",
            executable="cmd_trio_main",
            name="cmd_trio_node"
        ),
        Node(
            package="trio_interface_ros2",
            executable="fsm_trio_main",
            name="fsm_trio_node"
        ),
        Node(
            package="trio_interface_ros2",
            executable="cmd_trio_motion_tcp",
            name="cmd_trio_motion_tcp_node"
        ),
        Node(
            package="trio_interface_ros2",
            executable="fsm_trio_motion_tcp",
            name="fsm_trio_motion_tcp_node"
        )
    ])
