cmake_minimum_required(VERSION 3.8)
project(trio_interface_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# 실행 노드
add_executable(cmd_trio_main
  src/cmd_trio_main.cpp
)
ament_target_dependencies(cmd_trio_main rclcpp geometry_msgs)
target_link_libraries(cmd_trio_main trio_tcp_lib)

add_executable(fsm_trio_main
  src/fsm_trio_main.cpp
)
ament_target_dependencies(fsm_trio_main rclcpp std_msgs)
target_link_libraries(fsm_trio_main trio_tcp_lib)

# 내부 클래스들을 묶은 라이브러리
add_library(trio_tcp_lib
  src/cmd_trio_motion_node.cpp
  src/cmd_trio_motion_tcp.cpp
  src/fsm_trio_motion_node.cpp
  src/fsm_trio_motion_tcp.cpp
)
target_include_directories(trio_tcp_lib PUBLIC include)
ament_target_dependencies(trio_tcp_lib rclcpp geometry_msgs std_msgs)

# 설치 설정
install(TARGETS
  cmd_trio_main
  fsm_trio_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

