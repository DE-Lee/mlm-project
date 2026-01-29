"""
ROS2 Sync Node for emergency stop system.

여러 센서 토픽을 구독하고 동기화된 데이터를 발행하는 ROS2 노드.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, LaserScan, Imu
from nav_msgs.msg import Odometry
from mlm_avoid_msgs.msg import SyncedData

from .sensor_synchronizer import SensorSynchronizer


class SyncNode(Node):
    """
    ROS2 node for synchronizing sensor data.

    여러 센서 토픽을 구독하고, 시간 동기화된 데이터를 SyncedData 메시지로 발행한다.
    이미지 도착을 기준으로 동기화를 수행한다.

    Subscribes:
        - /image_raw (sensor_msgs/Image)
        - /scan_raw (sensor_msgs/LaserScan)
        - /odom (nav_msgs/Odometry)
        - /imu (sensor_msgs/Imu)

    Publishes:
        - /mlm_avoid/synced_data (mlm_avoid_msgs/SyncedData)
    """

    def __init__(self):
        """
        Initialize the sync node.

        노드를 초기화하고 파라미터, 구독자, 발행자를 설정한다.
        """
        super().__init__('mlm_avoid_sync_node')

        # 파라미터 선언
        self.declare_parameter('buffer_size', 30)
        # self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('image_topic', '/image')
        self.declare_parameter('scan_topic', '/scan_raw')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('synced_data_topic', '/mlm_avoid/synced_data')

        # 파라미터 가져오기
        buffer_size = self.get_parameter('buffer_size').value

        # 동기화기 생성
        self.synchronizer = SensorSynchronizer(buffer_size=buffer_size)

        # QoS 설정
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 구독자 생성
        image_topic = self.get_parameter('image_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        imu_topic = self.get_parameter('imu_topic').value

        self.sub_image = self.create_subscription(
            Image,
            image_topic,
            self.on_image,
            sensor_qos
        )

        self.sub_scan = self.create_subscription(
            LaserScan,
            scan_topic,
            self.on_scan,
            sensor_qos
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            odom_topic,
            self.on_odom,
            sensor_qos
        )

        self.sub_imu = self.create_subscription(
            Imu,
            imu_topic,
            self.on_imu,
            sensor_qos
        )

        # 발행자 생성 (BEST_EFFORT - 센서 데이터 특성)
        synced_data_topic = self.get_parameter('synced_data_topic').value
        self.pub_synced = self.create_publisher(
            SyncedData,
            synced_data_topic,
            sensor_qos  # BEST_EFFORT QoS 사용
        )

        # 통계
        self.sync_count = 0
        self.fail_count = 0

        self.get_logger().info(f'Sync node initialized (buffer_size={buffer_size})')

    def on_image(self, msg: Image) -> None:
        """
        Handle incoming image message.

        Args:
            msg: Image message from camera.

        이미지 도착 시 동기화를 시도하고, 성공하면 SyncedData를 발행한다.
        """
        synced = self.synchronizer.on_image(msg)

        if synced is not None:
            self.publish_synced_data(synced)
            self.sync_count += 1

            # 첫 성공 시 로그
            if self.sync_count == 1:
                self.get_logger().info('First sync successful! Publishing synced data.')
        else:
            self.fail_count += 1

            # 처음 10번, 그 후 100번마다 로그
            if self.fail_count <= 10 or self.fail_count % 100 == 0:
                missing = [
                    name for name in self.synchronizer.SENSOR_NAMES
                    if self.synchronizer.latest[name] is None
                ]
                if missing:
                    self.get_logger().warn(
                        f'Sync failed: waiting for sensors: {missing}'
                    )

    def on_scan(self, msg: LaserScan) -> None:
        """
        Handle incoming LaserScan message.

        Args:
            msg: LaserScan message from LiDAR.

        LiDAR 데이터를 버퍼에 추가한다.
        """
        self.synchronizer.add_to_buffer('scan', msg)

    def on_odom(self, msg: Odometry) -> None:
        """
        Handle incoming Odometry message.

        Args:
            msg: Odometry message.

        Odometry 데이터를 버퍼에 추가한다.
        """
        self.synchronizer.add_to_buffer('odom', msg)

    def on_imu(self, msg: Imu) -> None:
        """
        Handle incoming IMU message.

        Args:
            msg: IMU message.

        IMU 데이터를 버퍼에 추가한다.
        """
        self.synchronizer.add_to_buffer('imu', msg)

    def publish_synced_data(self, synced: dict) -> None:
        """
        Publish synchronized data.

        Args:
            synced: Dictionary containing synced sensor data.

        동기화된 데이터를 SyncedData 메시지로 발행한다.
        """
        msg = SyncedData()

        # 이미지 타임스탬프를 기준으로 헤더 설정
        msg.header = synced['image'].header

        # 각 센서 데이터 복사
        msg.image = synced['image']
        msg.scan = synced['scan']
        msg.odom = synced['odom']
        msg.imu = synced['imu']

        self.pub_synced.publish(msg)


def main(args=None):
    """
    Main entry point.

    노드를 실행한다.
    """
    rclpy.init(args=args)

    node = SyncNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Shutting down. Sync success: {node.sync_count}, '
            f'fail: {node.fail_count}'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
