#include "trio_interface_ros2/cmd_trio_motion_tcp.hpp"
#include <arpa/inet.h>
#include <netinet/tcp.h>


using namespace trio_interface_ros2;

//CmdTrioMotionTCP::CmdTrioMotionTCP(rclcpp::Node::SharedPtr node)
//: node_(std::move(node)), server_fd_(-1), socket_fd_(-1)
//{
//}

CmdTrioMotionTCP::CmdTrioMotionTCP(const rclcpp::Logger & logger)
: logger_(logger), server_fd_(-1), socket_fd_(-1)
{}

CmdTrioMotionTCP::~CmdTrioMotionTCP()
{
    double stop[2] = {0.0, 0.0};
    sendCmd(stop);

    if (socket_fd_ >= 0) close(socket_fd_);
    if (server_fd_ >= 0) close(server_fd_);
}

bool CmdTrioMotionTCP::initServo()
{
    //RCLCPP_INFO(node_->get_logger(), "Initializing TRIO Motion TCP server...");
    RCLCPP_INFO(logger_, "TRIO Motion TCP server initialized.");

    if ((server_fd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        //RCLCPP_ERROR(node_->get_logger(), "Socket creation failed.");
        RCLCPP_ERROR(logger_, "Socket creation failed.");
        return false;
    }

    if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt_, sizeof(opt_)) < 0)
    { 
        //RCLCPP_ERROR(node_->get_logger(), "setsockopt failed.");
        RCLCPP_ERROR(logger_, "setsockopt failed.");
        return false;
    }

    address_.sin_family = AF_INET;
    address_.sin_addr.s_addr = INADDR_ANY;
    // 아래 주석처리한 코드는 특정 ip에서만 연결받음
    //inet_pton(AF_INET, "192.168.0.250", &address_.sin_addr); //23
    address_.sin_port = htons(TRIO_PORT);

    if (bind(server_fd_, (struct sockaddr*)&address_, sizeof(address_)) < 0)
    {
        //RCLCPP_ERROR(node_->get_logger(), "bind() failed.");
        RCLCPP_ERROR(logger_, "bind() failed.");
        return false;
    }

    if (listen(server_fd_, 3) < 0)
    {
        //RCLCPP_ERROR(node_->get_logger(), "listen() failed.");
        RCLCPP_ERROR(logger_, "listen() failed.");
        return false;
    }

    int addrlen = sizeof(address_);
    if ((socket_fd_ = accept(server_fd_, (struct sockaddr*)&address_, (socklen_t*)&addrlen)) < 0)
    {
        //RCLCPP_ERROR(node_->get_logger(), "accept() failed.");
        RCLCPP_ERROR(logger_, "accept() failed.");
        return false;
    }
    
    struct sockaddr_in peer_addr;
    socklen_t peer_len = sizeof(peer_addr);
    getpeername(socket_fd_, (struct sockaddr *)&peer_addr, &peer_len);

    char ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &peer_addr.sin_addr, ip_str, sizeof(ip_str));
    printf("Connected from IP: %s\n", ip_str);
    
    ////Nagle 알고리즘 비활성화 //accept 이후에 적용해야함
    int flag = 1;
    setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int));

    struct timeval timeout{0, 20000};  // 20 ms
    setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(timeout));

    memset(buffer_, 0, sizeof(buffer_));
    connected_ = true;
    //RCLCPP_INFO(node_->get_logger(), "TRIO Motion TCP server initialized.");
    RCLCPP_INFO(logger_, "TRIO Motion TCP server initialized.");
    return true;
}

bool CmdTrioMotionTCP::sendCmd(const double* vel)
{
    RCLCPP_INFO(logger_, "SendCMd.");
    if (!connected_ && !initServo())
    {
        //RCLCPP_ERROR(node_->get_logger(), "Connection failed. Skipping command.");
        RCLCPP_ERROR(logger_, "Connection failed. Skipping command.");
        return false;
    }

    buildBuffer(vel);
    memset(buffer_, 0, 18);

    buffer_[0] = '7'; buffer_[1] = '7';
    buffer_[2] = '6'; buffer_[3] = '6';
    buffer_[4] = '1';

    int rpm = vel2rpm(vel[0]);
    rpm = std::clamp(rpm, -9999, 9999);
    buffer_[5] = rpm < 0 ? '-' : '+';
    rpm = std::abs(rpm);
    buffer_[6] = (rpm / 1000) % 10 + '0';
    buffer_[7] = (rpm / 100) % 10 + '0';
    buffer_[8] = (rpm / 10) % 10 + '0';
    buffer_[9] = rpm % 10 + '0';

    rpm = vel2rpm(-vel[1]); // left wheel
    rpm = std::clamp(rpm, -9999, 9999);
    buffer_[10] = rpm < 0 ? '-' : '+';
    rpm = std::abs(rpm);
    buffer_[11] = (rpm / 1000) % 10 + '0';
    buffer_[12] = (rpm / 100) % 10 + '0';
    buffer_[13] = (rpm / 10) % 10 + '0';
    buffer_[14] = rpm % 10 + '0';

    buffer_[15] = '5';
    buffer_[16] = '5';
    buffer_[17] = '\n';

    if (send(socket_fd_, buffer_, sizeof(buffer_), MSG_DONTWAIT) <= 0)
    //if (send(socket_fd_, buffer_, 18, 0) <= 0)
    {
    	RCLCPP_ERROR(logger_, "Connection failed. send stop");
        //RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        //                     "Failed to send velocity command. Sending stop...");
        //RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 1000,
        //                     "Failed to send velocity command. Sending stop...");
        
        double stop[2] = {0.0, 0.0};
        buildBuffer(stop);
        //send(socket_fd_, buffer_, sizeof(buffer_), MSG_DONTWAIT);
        send(socket_fd_, buffer_, sizeof(buffer_), 0);
        usleep(200000);
        return false;
    }
    usleep(1000000);
    
    //연결에 의한 blocking 확인
    //char recv_buf[64] = {0};
    //int n = recv(socket_fd_, recv_buf, sizeof(recv_buf), 0);
    //if (n > 0) {
    //    printf("Received from TRIO: %.*s\n", n, recv_buf);
    //} else {
    //    perror("recv failed or closed");
    //}


    return true;
}

void CmdTrioMotionTCP::buildBuffer(const double* vel)
{
    RCLCPP_INFO(logger_, "buildbuffer.");
    memset(buffer_, 0, sizeof(buffer_));

    buffer_[0] = '7'; buffer_[1] = '7';
    buffer_[2] = '6'; buffer_[3] = '6';
    buffer_[4] = '7';

    int rpm = vel2rpm(vel[0]);
    rpm = std::clamp(rpm, -9999, 9999);
    buffer_[5] = rpm < 0 ? '-' : '+';
    rpm = std::abs(rpm);
    buffer_[6] = (rpm / 1000) % 10 + '0';
    buffer_[7] = (rpm / 100) % 10 + '0';
    buffer_[8] = (rpm / 10) % 10 + '0';
    buffer_[9] = rpm % 10 + '0';

    rpm = vel2rpm(-vel[1]); // left wheel
    rpm = std::clamp(rpm, -9999, 9999);
    buffer_[10] = rpm < 0 ? '-' : '+';
    rpm = std::abs(rpm);
    buffer_[11] = (rpm / 1000) % 10 + '0';
    buffer_[12] = (rpm / 100) % 10 + '0';
    buffer_[13] = (rpm / 10) % 10 + '0';
    buffer_[14] = rpm % 10 + '0';

    buffer_[15] = '5';
    buffer_[16] = '5';
    buffer_[17] = '\n';
}

int CmdTrioMotionTCP::vel2rpm(double v)
{
    return static_cast<int>(std::round(v * 60.0 / (2.0 * M_PI) * gear_ratio_));
}

void CmdTrioMotionTCP::setGear(double ratio)
{
    gear_ratio_ = ratio;
    //RCLCPP_INFO(node_->get_logger(), "Gear ratio set to %.2f", gear_ratio_);
    RCLCPP_INFO(logger_, "Gear ratio set to %.2f", gear_ratio_);
}
