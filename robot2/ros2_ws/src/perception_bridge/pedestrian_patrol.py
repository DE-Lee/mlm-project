import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class PedestrianPatrol(Node):
    def __init__(self):
        super().__init__('pedestrian_patrol')
        # 보행자 속도 토픽 발행자 생성
        self.publisher_ = self.create_publisher(Twist, '/ped_cmd_vel', 10)
        # 0.1초마다 루프 실행
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.start_time = time.time()
        self.direction = 1  # 1: 전진, -1: 후진
        self.speed = 0.6    # 보행자 속도 (m/s)
        self.duration = 5.0 # 왕복 주기 (초)

    def timer_callback(self):
        msg = Twist()
        current_time = time.time()
        
        # 설정한 주기(duration)가 지나면 방향 전환
        if current_time - self.start_time > self.duration:
            self.direction *= -1
            self.start_time = current_time
            self.get_logger().info(f'방향 전환: {"전진" if self.direction > 0 else "후진"}')

        # 속도 설정
        msg.linear.x = self.speed * self.direction
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PedestrianPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 멈춤 명령
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
