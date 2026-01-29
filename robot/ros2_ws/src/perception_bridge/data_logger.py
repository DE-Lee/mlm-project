import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from perception_msgs.msg import ObjectArray
import message_filters
import csv
import numpy as np

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        
        # CSV 파일 설정
        self.file = open('evasion_dataset.csv', mode='w', newline='')
        self.writer = csv.writer(self.file)
        
        # 헤더 생성: lidar_0~359 + obj0~9의 rel_x, rel_y, class + target_v, target_steer
        header = [f'lidar_{i}' for i in range(360)]
        for i in range(10):
            header.extend([f'obj{i}_rel_x', f'obj{i}_rel_y', f'obj{i}_class'])
        header.extend(['target_v', 'target_steer'])
        self.writer.writerow(header)

        # 파라미터 및 상태 변수
        self.threshold_dist = 3.0  # 로깅 시작 거리 (3m)
        self.is_logging_active = False
        self.current_cmd_v = 0.0
        self.current_cmd_w = 0.0

        # /cmd_vel 구독 (헤더 없음, 별도 구독)
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10
        )

        # 헤더가 있는 메시지들 동기화 (LaserScan, ObjectArray)
        scan_sub = message_filters.Subscriber(self, LaserScan, '/scan')
        objects_sub = message_filters.Subscriber(self, ObjectArray, '/perception/objects')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [scan_sub, objects_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)
        
        self.get_logger().info(
            f'데이터 로거 시작: 객체 반경 {self.threshold_dist}m 진입 시 자동 기록'
        )

    def cmd_callback(self, msg):
        """최신 조종 명령 업데이트"""
        self.current_cmd_v = msg.linear.x
        self.current_cmd_w = msg.angular.z

    def sync_callback(self, scan, objects_msg):
        """LiDAR와 ObjectArray 동기화 콜백"""
        
        # 가장 가까운 장애물(class=1) 또는 미션 객체(class=2)의 거리 계산
        # obj0이 가장 가까운 객체 (거리순 정렬되어 있음)
        closest_dist = float('inf')
        for obj in objects_msg.objects:
            if obj.object_class != 0:  # 빈 슬롯이 아니면
                dist = np.sqrt(obj.rel_x**2 + obj.rel_y**2)
                if dist < closest_dist:
                    closest_dist = dist
                break  # 거리순 정렬이므로 첫 번째 유효 객체가 가장 가까움

        # 거리 기반 자동 로깅 로직
        if closest_dist < self.threshold_dist:
            if not self.is_logging_active:
                self.get_logger().info(
                    f'>>> 객체 근접 ({closest_dist:.2f}m): 기록 시작'
                )
                self.is_logging_active = True
            
            # LiDAR 데이터 전처리 (inf -> 12.0)
            lidar = [x if x != float('inf') else 12.0 for x in scan.ranges]

            # 객체 배열 데이터 추출
            objects_data = []
            for obj in objects_msg.objects:
                objects_data.extend([obj.rel_x, obj.rel_y, obj.object_class])

            # CSV 한 행 저장
            row = lidar + objects_data + [self.current_cmd_v, self.current_cmd_w]
            self.writer.writerow(row)
        else:
            if self.is_logging_active:
                self.get_logger().info('<<< 객체 멀어짐: 기록 중단')
                self.is_logging_active = False


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.file.close()
        node.get_logger().info('CSV 파일 저장 완료: evasion_dataset.csv')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
