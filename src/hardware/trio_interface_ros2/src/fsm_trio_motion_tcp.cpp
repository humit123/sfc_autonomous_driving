#include "trio_interface_ros2/fsm_trio_motion_tcp.hpp"
//현재 구조는 노드 1회 생성시 server socket은 열어놓고 client 연결은 1회 후 종료 하는 형태임
//향후 개선 고려

namespace trio_interface_ros2 {
// 모든 구현 내용

FSMTrioMotionTCP::FSMTrioMotionTCP()
: m_connected(false), m_server_fd(-1), m_socket(-1),
  m_cmd(0), m_target(0), m_state(0), m_pose(0)
{
  std::memset(m_buffer, 0, sizeof(m_buffer));
}

FSMTrioMotionTCP::~FSMTrioMotionTCP()
{
  if (m_socket >= 0) {
    close(m_socket);
  }
  if (m_server_fd >= 0) {
    close(m_server_fd);
  }
}

bool FSMTrioMotionTCP::initComm()
{
  rclcpp::Logger logger = rclcpp::get_logger("FSMTrioMotionTCP");
  RCLCPP_INFO(logger, "Initializing communication...");

  m_server_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (m_server_fd < 0) {
    RCLCPP_ERROR(logger, "Failed to create socket.");
    return false;
  }

  m_opt = 1;
  if (setsockopt(m_server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &m_opt, sizeof(m_opt))) {
    RCLCPP_ERROR(logger, "Failed to set socket options.");
    return false;
  }

  m_address.sin_family = AF_INET;
  m_address.sin_addr.s_addr = INADDR_ANY;
  m_address.sin_port = htons(MNG_TRIO_PORT);

  if (bind(m_server_fd, (struct sockaddr *)&m_address, sizeof(m_address)) < 0) {
    RCLCPP_ERROR(logger, "Failed to bind socket.");
    return false;
  }

  if (listen(m_server_fd, 3) < 0) {
    RCLCPP_ERROR(logger, "Listen failed.");
    return false;
  }

  socklen_t addrlen = sizeof(m_address);
  m_socket = accept(m_server_fd, (struct sockaddr *)&m_address, &addrlen);
  if (m_socket < 0) {
    RCLCPP_ERROR(logger, "Accept failed.");
    return false;
  }

  m_connected = true;
  RCLCPP_INFO(logger, "Communication initialized.");
  return true;
}

void FSMTrioMotionTCP::parseCommand(const std::string & line)
{
  rclcpp::Logger logger = rclcpp::get_logger("FSMTrioMotionTCP");

  if (line.length() < 10) return;

  if (line[0] == '7' && line[1] == '7' &&
      line[2] == '6' && line[3] == '6' &&
      line[4] == '1' && line[8] == '5' &&
      line[9] == '5')
  {
    if (!isdigit(line[5]) || !isdigit(line[6]) || !isdigit(line[7])) {
      RCLCPP_WARN(logger, "Invalid digit in command.");
      return;
    }

    if (line[5] == '1') {
      m_cmd = SK_MOBILE_ROBOT_SMART_FARM_GOTO_TARGET;
      m_target = (line[6] - '0') * 10 + (line[7] - '0');
    } else if (line[5] == '0') {
      m_cmd = SK_MOBILE_ROBOT_IDLE;
    } else {
      RCLCPP_WARN(logger, "Unknown command: %c", line[5]);
    }
  } else {
    RCLCPP_WARN(logger, "Invalid header/footer in packet.");
  }
}

void FSMTrioMotionTCP::comm()
{
  if (!m_connected) {
    if (!initComm()) {
      return;
    }
  }

  std::memset(m_buffer, 0, sizeof(m_buffer));
  int len = read(m_socket, m_buffer, sizeof(m_buffer));
  if (len > 0) {
    std::string input(m_buffer, len);
    std::stringstream ss(input);
    std::string line;
    while (std::getline(ss, line, '\n')) {
      parseCommand(line);
    }
  }

  std::memset(m_buffer, 0, sizeof(m_buffer));
  m_buffer[0] = '7';
  m_buffer[1] = '7';
  m_buffer[2] = '6';
  m_buffer[3] = '6';
  m_buffer[4] = '1';
  m_buffer[5] = (m_state == SK_MOBILE_ROBOT_SMART_FARM_GOTO_TARGET) ? '1' :
                (m_state == SK_MOBILE_ROBOT_IDLE) ? '0' : '2';
  m_buffer[6] = (m_pose / 10) % 10 + '0';
  m_buffer[7] = m_pose % 10 + '0';
  m_buffer[8] = '5';
  m_buffer[9] = '5';
  m_buffer[10] = '\n';

  send(m_socket, m_buffer, 11, 0);
  // 클라이언트 연결 종료 및 상태 리셋
  close(m_socket);
  m_socket = -1;
  m_connected = false;
}

int FSMTrioMotionTCP::getCmd() const { return m_cmd; }
int FSMTrioMotionTCP::getTarget() const { return m_target; }
void FSMTrioMotionTCP::setState(const int & state) { m_state = state; }
void FSMTrioMotionTCP::setPose(const int & pose) { m_pose = pose; }

} // namespace trio_interface_ros2
