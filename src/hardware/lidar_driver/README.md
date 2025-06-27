# LiDAR Driver (RPLIDAR S3)

이 디렉토리는 RPLIDAR S3를 ROS 2 Humble 환경에서 실행하기 위한 launch 설정을 포함합니다.

## 사용 장비
- **센서**: Slamtec RPLIDAR S3
- **연결 포트**: `/dev/ttyUSB0`
- **Baudrate**: 256000

## 설치 및 실행 방법

### 1. 의존 패키지 설치
```bash
sudo apt install ros-humble-rplidar-ros

또는 workspace에 clone
cd ~/autonomous_driving_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git


2. 빌드 및 소스
cd ~/autonomous_driving_ws
colcon build --packages-select lidar_driver
source install/setup.bash

3. 실행
ros2 launch lidar_driver rplidar.launch.py

4. 확인
ros2 topic echo /scan

참고
serial_port는 실제 연결된 포트명에 따라 /dev/ttyUSB1, /dev/ttyAMA0 등으로 변경
rplidar_composition은 rplidar_ros 패키지에서 제공하는 실행 파일