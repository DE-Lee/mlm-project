"""Perception Bridge 노드

YOLO Detection 결과와 LiDAR 데이터를 융합하여
로봇 주변 객체 정보를 발행하는 노드
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import message_filters
import numpy as np

from perception_msgs.msg import Object, ObjectArray
from .lidar_bbox_fusion import LidarBBoxFusion, CameraParams, FusionConfig
from .utils import build_object_array


class PerceptionBridgeNode(Node):
    """Perception Bridge ROS2 노드"""

    def __init__(self):
        super().__init__('perception_bridge_node')

        # 파라미터 선언 및 로드
        self._declare_parameters()
        self._load_parameters()

        # LiDAR-BBox 융합 모듈 초기화
        self._init_fusion_module()

        # 퍼블리셔 생성
        self.objects_pub = self.create_publisher(
            ObjectArray,
            self.output_topic,
            10
        )

        if self.enable_visualization:
            self.marker_pub = self.create_publisher(
                MarkerArray,
                '/perception/markers',
                10
            )

        # 서브스크라이버 생성 (message_filters 사용)
        self.detection_sub = message_filters.Subscriber(
            self, Detection2DArray, self.detection_topic
        )
        self.lidar_sub = message_filters.Subscriber(
            self, LaserScan, self.lidar_topic
        )

        # ApproximateTimeSynchronizer로 동기화
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.detection_sub, self.lidar_sub],
            queue_size=10,
            slop=0.1  # 100ms 허용
        )
        self.sync.registerCallback(self.sync_callback)

        self.get_logger().info(
            f'Perception Bridge 시작: '
            f'detection={self.detection_topic}, '
            f'lidar={self.lidar_topic}, '
            f'output={self.output_topic}'
        )

    def _declare_parameters(self):
        """ROS2 파라미터 선언"""
        # 토픽
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('lidar_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/perception/objects')
        self.declare_parameter('output_frame', 'base_link')

        # 객체 분류
        self.declare_parameter('obstacle_class_ids', [0, 1, 2, 3])
        self.declare_parameter('mission_class_ids', [5])

        # 융합 파라미터
        self.declare_parameter('max_detection_range_m', 10.0)
        self.declare_parameter('min_confidence', 0.5)

        # 카메라 파라미터
        self.declare_parameter('camera_fx', 554.0)
        self.declare_parameter('camera_fy', 554.0)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('camera_hfov', 1.047)
        self.declare_parameter('camera_lidar_angle_offset', 0.0)

        # 디버깅
        self.declare_parameter('enable_visualization', False)

    def _load_parameters(self):
        """파라미터 로드"""
        self.detection_topic = self.get_parameter('detection_topic').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame = self.get_parameter('output_frame').value

        self.obstacle_class_ids = set(
            self.get_parameter('obstacle_class_ids').value
        )
        self.mission_class_ids = set(
            self.get_parameter('mission_class_ids').value
        )

        self.max_detection_range = self.get_parameter('max_detection_range_m').value
        self.min_confidence = self.get_parameter('min_confidence').value

        self.camera_fx = self.get_parameter('camera_fx').value
        self.camera_fy = self.get_parameter('camera_fy').value
        self.camera_cx = self.get_parameter('camera_cx').value
        self.camera_cy = self.get_parameter('camera_cy').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.camera_hfov = self.get_parameter('camera_hfov').value
        self.camera_lidar_angle_offset = self.get_parameter(
            'camera_lidar_angle_offset'
        ).value

        self.enable_visualization = self.get_parameter('enable_visualization').value

    def _init_fusion_module(self):
        """융합 모듈 초기화"""
        camera_params = CameraParams(
            fx=self.camera_fx,
            fy=self.camera_fy,
            cx=self.camera_cx,
            cy=self.camera_cy,
            width=self.image_width,
            height=self.image_height
        )

        fusion_config = FusionConfig(
            camera_hfov=self.camera_hfov,
            camera_lidar_angle_offset=self.camera_lidar_angle_offset,
            max_range=self.max_detection_range
        )

        self.fusion = LidarBBoxFusion(camera_params, fusion_config)

    def _classify_object(self, yolo_class_id: int) -> int:
        """YOLO class_id를 object_class로 매핑

        Returns:
            Object.CLASS_OBSTACLE (1), Object.CLASS_MISSION (2), 또는 0 (무시)
        """
        if yolo_class_id in self.obstacle_class_ids:
            return Object.CLASS_OBSTACLE
        elif yolo_class_id in self.mission_class_ids:
            return Object.CLASS_MISSION
        else:
            return 0  # 매핑되지 않은 클래스는 무시

    def sync_callback(self, detections_msg: Detection2DArray, scan_msg: LaserScan):
        """Detection과 LiDAR 메시지 동기화 콜백"""
        objects = []

        # LiDAR 데이터 numpy 배열로 변환
        lidar_ranges = np.array(scan_msg.ranges)

        for detection in detections_msg.detections:
            # confidence 체크
            if detection.results:
                result = detection.results[0]
                confidence = result.hypothesis.score
                class_id = int(result.hypothesis.class_id)

                if confidence < self.min_confidence:
                    continue

                # 클래스 매핑
                object_class = self._classify_object(class_id)
                if object_class == 0:
                    continue

                # BBox 정보 추출
                bbox = detection.bbox
                center_x = bbox.center.position.x
                size_x = bbox.size_x

                # LiDAR와 융합하여 위치 추정
                position = self.fusion.fuse(
                    bbox_center_x=center_x,
                    bbox_size_x=size_x,
                    lidar_ranges=lidar_ranges,
                    lidar_angle_min=scan_msg.angle_min,
                    lidar_angle_increment=scan_msg.angle_increment
                )

                if position is not None:
                    rel_x, rel_y = position
                    objects.append((rel_x, rel_y, object_class))

        # ObjectArray 메시지 생성 및 발행
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.output_frame

        msg = build_object_array(objects, header)
        self.objects_pub.publish(msg)

        # 시각화 (옵션)
        if self.enable_visualization:
            self._publish_markers(objects, header)

    def _publish_markers(self, objects: list, header: Header):
        """RViz 시각화용 마커 발행"""
        marker_array = MarkerArray()

        # 이전 마커 삭제
        delete_marker = Marker()
        delete_marker.header = header
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for i, (rel_x, rel_y, obj_class) in enumerate(objects):
            marker = Marker()
            marker.header = header
            marker.ns = 'perception_objects'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = rel_x
            marker.pose.position.y = rel_y
            marker.pose.position.z = 0.5

            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 1.0

            # 클래스별 색상
            if obj_class == Object.CLASS_OBSTACLE:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:  # MISSION
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

            marker.color.a = 0.8
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 500000000  # 0.5초

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
