#ifndef TRIO_INTERFACE_ROS2_FSM_TRIO_MOTION_TCP_HPP
#define TRIO_INTERFACE_ROS2_FSM_TRIO_MOTION_TCP_HPP

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <sstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

namespace trio_interface_ros2 {

constexpr int MNG_TRIO_PORT = 6000;
constexpr int BUFFER_LEN = 19;

constexpr int SK_MOBILE_ROBOT_IDLE = 0;
constexpr int SK_MOBILE_ROBOT_SMART_FARM_GOTO_TARGET = 42;

class FSMTrioMotionTCP
{
public:
  FSMTrioMotionTCP();
  ~FSMTrioMotionTCP();

  bool initComm();
  void comm();
  int getCmd() const;
  int getTarget() const;
  void setState(const int & state);
  void setPose(const int & pose);

private:
  void parseCommand(const std::string & line);
  bool m_connected;
  int m_server_fd;
  int m_socket;
  int m_opt;
  struct sockaddr_in m_address;
  char m_buffer[BUFFER_LEN];

  int m_cmd;
  int m_target;
  int m_state;
  int m_pose;
};

} // namespace trio_interface_ros2

#endif  // TRIO_INTERFACE_ROS2_FSM_TRIO_MOTION_TCP_HPP

