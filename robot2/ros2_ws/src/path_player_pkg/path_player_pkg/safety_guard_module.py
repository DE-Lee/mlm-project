import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from perception_msgs.msg import ObjectArray
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import time

# ==============================================================================
# [1] CONFIGURATION (사용자 설정 구역)
# ==============================================================================

# --- 디버깅 설정 ---
DEBUG_MODE = True  # True: 터미널에 거리 정보 출력, False: 조용함

# --- 토픽 이름 설정 ---
TOPIC_LIDAR_SCAN = '/scan_raw'
TOPIC_YOLO_OBJECTS = '/perception/objects'
TOPIC_CMD_INPUT = '/controller/cmd_vel'  # 조이스틱/텔레옵에서 발행
TOPIC_CMD_OUTPUT = '/cmd_vel'             # 필터링 후 로봇으로 발행
TOPIC_SAFETY_OUTPUT = '/robot/is_emergency_stop'  # 디버깅용 안전 상태 토픽

# --- 거리 제어 파라미터 (단위: m) ---
STOP_DISTANCE = 0.6    # 이 거리 이내면 정지 (예: 60cm)
RESUME_DISTANCE = 1.0  # 이 거리 밖으로 벗어나야 다시 출발 (히스테리시스 적용)

# --- 노이즈 필터 (Debouncing) ---
DETECT_THRESHOLD = 3  # N번 연속으로 위험이 감지되어야 정지 신호 보냄 (노이즈 방지)

# ==============================================================================

class SafetyGuardModule(Node):
    def __init__(self):
        super().__init__('safety_guard_module')
        
        # QoS 설정 (센서 데이터 유실 방지 및 최신 데이터 우선)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # 1. Subscribers
        # LiDAR는 센서 헬스 체크용으로만 구독 (융합은 perception_bridge에서 완료)
        self.create_subscription(LaserScan, TOPIC_LIDAR_SCAN, self.scan_callback, qos_profile)
        self.create_subscription(ObjectArray, TOPIC_YOLO_OBJECTS, self.yolo_callback, qos_profile)

        # cmd_vel 필터링: 조이스틱/텔레옵 입력 구독
        self.create_subscription(Twist, TOPIC_CMD_INPUT, self.cmd_input_callback, qos_profile)

        # 2. Publishers
        self.cmd_pub = self.create_publisher(Twist, TOPIC_CMD_OUTPUT, 10)  # 필터링된 명령
        self.safety_pub = self.create_publisher(Bool, TOPIC_SAFETY_OUTPUT, 10)  # 디버깅용

        # 3. State Variables
        self.is_stopped = False       # 현재 멈춤 상태인지 여부
        self.danger_count = 0         # 위험 감지 연속 횟수 카운터 (디바운싱용)

        # 4. Sensor Timeout Check
        self.last_scan_time = time.time()
        self.create_timer(1.0, self.check_sensor_health) # 1초마다 센서 상태 체크

        self.get_logger().info(f"안전 필터 모듈 시작")
        self.get_logger().info(f"입력: {TOPIC_CMD_INPUT} -> 출력: {TOPIC_CMD_OUTPUT}")
        self.get_logger().info(f"STOP: {STOP_DISTANCE}m | RESUME: {RESUME_DISTANCE}m")

    def scan_callback(self, msg):
        # 센서 헬스 체크용 (실제 데이터는 perception_bridge에서 융합됨)
        self.last_scan_time = time.time()

    def cmd_input_callback(self, cmd_msg):
        """조이스틱/텔레옵에서 오는 명령을 받아서 안전 필터링 후 발행"""
        # 안전 체크: emergency_stop 상태면 속도를 0으로
        if self.is_stopped:
            # 긴급 정지: 모든 속도를 0으로
            safe_cmd = Twist()  # 기본값이 모두 0
            self.cmd_pub.publish(safe_cmd)
            if DEBUG_MODE:
                self.get_logger().debug("긴급정지: 속도 0 발행")
        else:
            # 안전: 원본 명령 그대로 통과
            self.cmd_pub.publish(cmd_msg)

    def yolo_callback(self, msg):
        min_person_dist = 999.0  # 초기화
        person_detected = False

        # --- [A] 사람 탐지 및 최단 거리 계산 ---
        for obj in msg.objects:
            # object_class = 1: OBSTACLE (사람, 보행자, 휠체어 등)
            # object_class = 0: EMPTY (빈 슬롯), 2: MISSION (미션 객체)
            if obj.object_class == 1:  # 1 = OBSTACLE (사람)
                person_detected = True

                # 유클리드 거리 계산 (2D 평면상 직선 거리)
                distance = math.sqrt(obj.rel_x**2 + obj.rel_y**2)

                if distance < min_person_dist:
                    min_person_dist = distance

        # --- [B] 상태 결정 (히스테리시스 + 디바운싱) ---
        
        # 1. 위험 여부 1차 판단
        is_currently_danger = False
        
        if person_detected:
            if self.is_stopped:
                # 이미 멈춰있다면? -> RESUME 거리보다 멀어져야 안전
                if min_person_dist < RESUME_DISTANCE:
                    is_currently_danger = True
            else:
                # 움직이는 중이라면? -> STOP 거리보다 가까워지면 위험
                if min_person_dist < STOP_DISTANCE:
                    is_currently_danger = True
        
        # 2. 노이즈 필터 (Debouncing) 적용
        if is_currently_danger:
            self.danger_count += 1
        else:
            self.danger_count = 0 # 안전하면 카운트 즉시 초기화

        # 3. 최종 상태 업데이트
        # 지정된 횟수(3회) 이상 연속으로 위험해야 진짜 멈춤 신호 발생
        if self.danger_count >= DETECT_THRESHOLD:
            self.is_stopped = True
            final_status = True  # True = STOP
        else:
            self.is_stopped = False
            final_status = False # False = GO

        # --- [C] 결과 전송 및 로깅 ---

        # 디버깅용 안전 상태 토픽 발행 (True: 멈춰, False: 가)
        bool_msg = Bool()
        bool_msg.data = final_status
        self.safety_pub.publish(bool_msg)

        # 디버그 로그 출력
        if DEBUG_MODE and person_detected:
            status_str = "STOP" if final_status else "GO"
            self.get_logger().info(
                f"{status_str} | MinDist: {min_person_dist:.2f}m | Count: {self.danger_count}/{DETECT_THRESHOLD}"
            )

    def check_sensor_health(self):
        """
        [구조화] 센서 타임아웃 체크
        나중에 필요하면 아래 주석 해제하여 사용
        """
        now = time.time()
        lidar_delay = now - self.last_scan_time
        
        if lidar_delay > 2.0: # 2초 이상 라이다 끊김
            if DEBUG_MODE:
                self.get_logger().warn(f"라이다 데이터 끊김! ({lidar_delay:.1f}s)")
            
            # [TODO] 여기서 강제로 self.safety_pub.publish(True)를 보내야 함
            # msg = Bool(); msg.data = True; self.safety_pub.publish(msg)
            pass

def main():
    rclpy.init()
    node = SafetyGuardModule()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()