# Robot2 Packages

MentorPi 로봇2용 ROS2 패키지 (Raspberry Pi 5 + Docker)

## Robot2 전용 설정

| 항목 | 값 | 비고 |
|------|-----|------|
| Robot IP | `172.16.10.173` | WiFi 고정 IP |
| ROS_DOMAIN_ID | `3` | ⚠️ Robot1/PC와 동일 (멀티 로봇 통신 필수) |
| Namespace | `robot2` | 런치 파라미터로 분리 |
| PC IP | `172.16.11.167` | 동일 WiFi 환경 |

## 폴더 구조

```
robot2/
├── ros2_ws/src/
│   ├── bringup/              # 로봇 시작 런치
│   │   └── launch/
│   │       ├── mlm_bringup.launch.py    # MLM 최적화 버전
│   │       └── bringup_ns.launch.py     # namespace wrapper
│   ├── driver/
│   │   ├── controller/       # 모터 제어, 오도메트리, EKF
│   │   ├── ros_robot_controller/        # 하드웨어 SDK
│   │   └── ros_robot_controller_msgs/   # 메시지 정의
│   └── peripherals/          # 센서 런치 (LiDAR, 카메라)
├── third_party_ws/src/
│   ├── ldlidar_stl_ros2/     # LD19 LiDAR 드라이버
│   └── laser_filters/        # LiDAR 필터
└── cyclonedds/
    └── cyclonedds.xml        # DDS 설정 (Domain ID: 3)
```

## 핵심 설정 확인 ⚠️

### 1. CycloneDDS 설정 (Peer-to-Peer Unicast)

`cyclonedds/cyclonedds.xml` 파일 확인:

```xml
<Domain id="3">  <!-- ⚠️ 모든 로봇/PC가 동일한 Domain ID 사용 -->
  <Interfaces>
    <NetworkInterface name="wlan0"/>  <!-- Robot2 네트워크 인터페이스 -->
  </Interfaces>
  <Peers>
    <Peer address="172.16.11.167"/>   <!-- PC IP 주소 -->
  </Peers>
</Domain>
```

### 2. 환경 변수 설정 (~/.zshrc)

```bash
source /opt/ros/humble/setup.zsh
source /home/ubuntu/ros2_ws/install/setup.zsh
source /home/ubuntu/third_party_ws/install/setup.zsh
export ROS_DOMAIN_ID=3  # ⚠️ CRITICAL: 모든 로봇/PC가 3 사용 필수!
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds/cyclonedds.xml
```

**⚠️ CRITICAL**: Robot2는 **반드시 `ROS_DOMAIN_ID=3`** 사용 (Robot1/PC와 동일)
- 다른 Domain ID 사용 시 통신 불가능
- Launch 파일에서 자동 검증 (잘못된 값 사용 시 즉시 에러 발생)

## 실행

```bash
# Robot2 namespace로 실행
ros2 launch bringup bringup_ns.launch.py robot_name:=robot2
```

## Robot1과의 차이점

| 항목 | Robot1 | Robot2 |
|------|--------|--------|
| IP | 172.16.10.172 | 172.16.10.173 |
| ROS_DOMAIN_ID | 3 | 3 (동일) |
| Namespace | robot1 | robot2 |
| CycloneDDS Domain | 3 | 3 (동일) |

**분리 방법**: Domain ID가 아닌 **Namespace**로 로봇 분리
- 모든 토픽/서비스/액션이 namespace prefix 사용
- 예: `/robot1/cmd_vel`, `/robot2/cmd_vel`
- PC의 Nav2도 각각 namespace별로 실행
