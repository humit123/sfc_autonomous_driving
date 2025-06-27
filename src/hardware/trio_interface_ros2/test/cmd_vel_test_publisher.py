import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.count = 0
        self.timer_ = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        if self.count < 5:
            # 5초간 계속 속도 명령 
            msg.linear.x = 50.0
            msg.linear.y = 50.0
            self.get_logger().info('Sending velocity command...')
        else:
            # 그 이후는 정지
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            self.get_logger().info('Stopping robot.')

        self.publisher_.publish(msg)
        self.count += 1

        if self.count > 5:
            self.get_logger().info('Test complete. Shutting down.')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

