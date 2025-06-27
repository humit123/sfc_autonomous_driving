#ifndef TRIO_INTERFACE_ROS2_CMD_TRIO_MOTION_TCP_HPP
#define TRIO_INTERFACE_ROS2_CMD_TRIO_MOTION_TCP_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <cmath>

namespace trio_interface_ros2 {

constexpr int TRIO_PORT = 5000;
//constexpr int BUFFER_LEN = 19;
constexpr int BUFFER_LEN = 18;

class CmdTrioMotionTCP
{
public:
    //explicit CmdTrioMotionTCP(rclcpp::Node::SharedPtr node);
    explicit CmdTrioMotionTCP(const rclcpp::Logger & logger);
    ~CmdTrioMotionTCP();

    bool initServo();
    bool sendCmd(const double* vel);
    void setGear(double ratio);

private:
    rclcpp::Logger logger_;
    int vel2rpm(double v);
    void buildBuffer(const double* vel);

    rclcpp::Node::SharedPtr node_;
    int server_fd_;
    int socket_fd_;
    int opt_ = 1;
    char buffer_[BUFFER_LEN];
    sockaddr_in address_;
    bool connected_ = false;
    double gear_ratio_ = 1.0;
};

} // namespace trio_interface_ros2

#endif
