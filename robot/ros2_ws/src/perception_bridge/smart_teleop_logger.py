import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import pygame
import cv2
import numpy as np
from cv_bridge import CvBridge

class SmartTeleop(Node):
    def __init__(self):
        super().__init__('smart_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.log_pub = self.create_publisher(Bool, '/is_recording', 10)
        
        # 카메라 구독 추가
        self.cam_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.latest_image = None

        pygame.init()
        # 영상 크기(640x480)에 맞춰 창 크기 조절
        self.screen = pygame.display.set_mode((640, 480))
        pygame.display.set_caption("My Little Medical Control - FPV")
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.speed = 0.8
        self.steer_angle = 0.5

    def image_callback(self, msg):
        # ROS 이미지를 Pygame에서 사용 가능한 포맷으로 변환
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # OpenCV(BGR) -> Pygame(RGB) 변환
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        # 회전 및 크기 조정이 필요할 수 있음
        self.latest_image = np.rot90(rgb_img)
        self.latest_image = pygame.surfarray.make_surface(self.latest_image)
        self.latest_image = pygame.transform.flip(self.latest_image, True, False)

    def control_loop(self):
        pygame.event.pump()
        keys = pygame.key.get_pressed()
        
        # 1. 화면 렌더링 (카메라 영상 띄우기)
        if self.latest_image is not None:
            self.screen.blit(self.latest_image, (0, 0))
        else:
            self.screen.fill((0, 0, 0)) # 영상 없을 때 검은 화면
        
        # 안내 문구 추가 (로깅 상태 표시)
        if keys[pygame.K_SPACE]:
            pygame.draw.circle(self.screen, (255, 0, 0), (30, 30), 10) # 녹화 중 빨간 불
            
        pygame.display.flip()

        # 2. 로봇 제어 로직
        msg = Twist()
        if keys[pygame.K_UP]:    msg.linear.x = self.speed
        if keys[pygame.K_DOWN]:  msg.linear.x = -self.speed
        if keys[pygame.K_LEFT]:  msg.angular.z = self.steer_angle
        if keys[pygame.K_RIGHT]: msg.angular.z = -self.steer_angle
        
        recording = Bool()
        recording.data = True if keys[pygame.K_SPACE] else False
        
        self.pub.publish(msg)
        self.log_pub.publish(recording)

def main():
    rclpy.init()
    node = SmartTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
