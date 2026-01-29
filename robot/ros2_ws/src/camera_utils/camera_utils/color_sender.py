import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class ColorSender(Node):
    def __init__(self):
        super().__init__('color_sender_node')

        # 파라미터 선언 및 기본값 설정
        # 상대 경로 사용 (네임스페이스 지원을 위해 맨 앞 '/' 제거)
        # USB 카메라: image_raw (usb_cam 노드 출력)
        self.declare_parameter('input_topic', 'image_raw')
        # 중요: 기본값에서 맨 앞 '/' 제거 (네임스페이스 적용을 위해)
        self.declare_parameter('output_topic', 'my_color_cam/image_compressed')
        self.declare_parameter('jpeg_quality', 50)
        self.declare_parameter('skip_frames', 2)
        self.declare_parameter('resize_scale', 0.5)

        # 파라미터 값 읽어오기
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        self.skip_frames = self.get_parameter('skip_frames').get_parameter_value().integer_value
        self.resize_scale = self.get_parameter('resize_scale').get_parameter_value().double_value

        # [안전장치] skip_frames가 1보다 작으면 1로 강제 설정 (0으로 나누기 방지)
        if self.skip_frames < 1:
            self.get_logger().warn(f'skip_frames({self.skip_frames})가 너무 작습니다. 1로 설정합니다.')
            self.skip_frames = 1

        self.frame_count = 0
        self.bridge = CvBridge()

        # QoS 설정 (BEST_EFFORT: 카메라 드라이버 호환)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image, self.input_topic, self.listener_callback, sensor_qos)
        self.publisher_ = self.create_publisher(
            CompressedImage, self.output_topic, sensor_qos)

        self.get_logger().info(f'컬러 중계기 시작! 네임스페이스: {self.get_namespace()}')
        self.get_logger().info(f'설정 - 품질: {self.jpeg_quality}, 스킵: {self.skip_frames}, 비율: {self.resize_scale}')
        self.get_logger().info(f'토픽 - 입력: {self.input_topic} -> 출력: {self.output_topic}')

    def listener_callback(self, msg):
        self.frame_count += 1
        # 프레임 스킵 (CPU 부하 감소)
        if self.frame_count % self.skip_frames != 0:
            return

        try:
            # YUV422 -> BGR 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 해상도 축소 (옵션)
            if self.resize_scale < 1.0:
                cv_image = cv2.resize(cv_image, None,
                    fx=self.resize_scale, fy=self.resize_scale,
                    interpolation=cv2.INTER_NEAREST)  # 가장 빠른 보간법

            # JPEG 압축
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            success, encoded_img = cv2.imencode('.jpg', cv_image, encode_param)

            if success:
                msg_out = CompressedImage()
                msg_out.header = msg.header
                msg_out.format = "jpeg"
                msg_out.data = encoded_img.tobytes()
                self.publisher_.publish(msg_out)

        except Exception as e:
            self.get_logger().error(f'변환 실패: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ColorSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료 요청 받음')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()