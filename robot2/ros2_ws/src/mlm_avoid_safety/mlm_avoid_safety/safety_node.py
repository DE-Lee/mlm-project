"""
ROS2 Safety Node for emergency stop system.

TTC 기반 개입 판단 및 cmd_vel mux 기능을 수행하는 ROS2 노드.
위험 상황에서 주행 명령 대신 긴급 정지를 적용한다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from mlm_avoid_msgs.msg import SyncedData

from .ttc_calculator import TTCCalculator


class SafetyNode(Node):
    """
    ROS2 node for safety monitoring and cmd_vel multiplexing.

    동기화된 센서 데이터를 받아 TTC를 계산하고, 위험 상황에서
    주행 명령 대신 긴급 정지를 적용한다.

    Subscribes:
        - /mlm_avoid/synced_data (mlm_avoid_msgs/SyncedData)
        - /cmd_vel_nav (geometry_msgs/Twist) - 주행 명령 (path_player 등)

    Publishes:
        - /cmd_vel (geometry_msgs/Twist) - 최종 로봇 명령
    """

    def __init__(self):
        """
        Initialize the safety node.

        노드를 초기화하고 파라미터, 구독자, 발행자를 설정한다.
        """
        super().__init__('mlm_avoid_safety_node')

        # 파라미터 선언
        self.declare_parameter('ttc_threshold_start', 2.0)
        self.declare_parameter('ttc_threshold_end', 3.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)  # 명령 타임아웃 (초)

        # 토픽 파라미터 (namespace 지원)
        self.declare_parameter('synced_data_topic', '/mlm_avoid/synced_data')
        self.declare_parameter('cmd_vel_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # 파라미터 가져오기
        ttc_start = self.get_parameter('ttc_threshold_start').value
        ttc_end = self.get_parameter('ttc_threshold_end').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        # TTC 계산기 생성
        self.ttc_calculator = TTCCalculator(
            ttc_threshold_start=ttc_start,
            ttc_threshold_end=ttc_end
        )

        # 상태 변수
        self.intervening = False
        self.cmd_vel_nav: Twist | None = None
        self.last_nav_time = self.get_clock().now()

        # QoS 설정
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 토픽 파라미터 가져오기
        synced_data_topic = self.get_parameter('synced_data_topic').value
        cmd_vel_nav_topic = self.get_parameter('cmd_vel_nav_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # 구독자 생성
        self.sub_synced = self.create_subscription(
            SyncedData,
            synced_data_topic,
            self.on_synced_data,
            sensor_qos
        )

        self.sub_cmd_nav = self.create_subscription(
            Twist,
            cmd_vel_nav_topic,
            self.on_cmd_vel_nav,
            cmd_qos
        )

        # 발행자 생성
        self.pub_cmd_vel = self.create_publisher(
            Twist,
            cmd_vel_topic,
            cmd_qos
        )

        # 주기적 cmd_vel 발행 타이머 (10Hz)
        self.timer = self.create_timer(0.1, self.on_timer)

        # 통계
        self.intervention_count = 0

        self.get_logger().info(
            f'Safety node initialized (TTC thresholds: '
            f'start={ttc_start}s, end={ttc_end}s)'
        )

    def on_synced_data(self, msg: SyncedData) -> None:
        """
        Handle synchronized sensor data.

        Args:
            msg: Synchronized sensor data.

        동기화된 센서 데이터를 받아 TTC를 계산하고 개입 여부를 결정한다.
        """
        # 속도 추출
        vx = msg.odom.twist.twist.linear.x
        vy = msg.odom.twist.twist.linear.y

        # TTC 계산
        min_ttc = self.ttc_calculator.calculate_min_ttc(
            ranges=msg.scan.ranges,
            angle_min=msg.scan.angle_min,
            angle_increment=msg.scan.angle_increment,
            vx=vx,
            vy=vy
        )

        # 개입 여부 결정
        prev_intervening = self.intervening
        self.intervening = self.ttc_calculator.should_intervene(
            ttc=min_ttc,
            currently_intervening=self.intervening
        )

        # 상태 변화 로깅
        if self.intervening and not prev_intervening:
            self.intervention_count += 1
            self.get_logger().warn(
                f'EMERGENCY STOP! TTC={min_ttc:.2f}s '
                f'(count: {self.intervention_count})'
            )
        elif not self.intervening and prev_intervening:
            self.get_logger().info(f'Resuming navigation. TTC={min_ttc:.2f}s')

    def on_cmd_vel_nav(self, msg: Twist) -> None:
        """
        Handle navigation cmd_vel.

        Args:
            msg: Twist message from path_player/motion_controller.

        주행 명령을 저장한다.
        """
        self.cmd_vel_nav = msg
        self.last_nav_time = self.get_clock().now()

    def on_timer(self) -> None:
        """
        Periodic callback to publish cmd_vel.

        주기적으로 cmd_vel을 결정하여 발행한다.
        """
        now = self.get_clock().now()
        cmd = Twist()

        if self.intervening:
            # 개입 중: 긴급 정지
            cmd = Twist()  # 모든 값 0
        else:
            # 정상: 주행 명령 통과
            if self.cmd_vel_nav is not None:
                dt = (now - self.last_nav_time).nanoseconds / 1e9
                if dt < self.cmd_vel_timeout:
                    cmd = self.cmd_vel_nav
                # 타임아웃이면 정지 (cmd는 이미 Twist()로 초기화됨)

        self.pub_cmd_vel.publish(cmd)

    def get_status(self) -> dict:
        """
        Get current status.

        Returns:
            Dictionary with current status.

        현재 상태를 반환한다.
        """
        return {
            'intervening': self.intervening,
            'intervention_count': self.intervention_count,
        }


def main(args=None):
    """
    Main entry point.

    노드를 실행한다.
    """
    rclpy.init(args=args)

    node = SafetyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        status = node.get_status()
        node.get_logger().info(
            f'Shutting down. Interventions: {status["intervention_count"]}'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
