# Emergency Stop System Transplant Guide
# GT → DOG (robot1) 이식 가이드

**작성일:** 2026-01-27
**작성자:** Claude Code Assistant
**버전:** 1.0
**핵심 원칙:** 기존 DOG 로봇 코드 수정 금지, 이식 패키지만 수정

---

## 1. 시스템 현황 분석

### 1.1 로봇 환경 비교

| 항목 | DOG (172.16.10.172) | GT (172.16.10.37) |
|------|---------------------|-------------------|
| ROS_DOMAIN_ID | 3 | 77 |
| ROBOT_NAMESPACE | robot1 | (없음) |
| CycloneDDS | 설정됨 | 미설정 |
| Multi-robot 지원 | ✅ | ❌ |

### 1.2 DOG (robot1) 실제 토픽 구조

```
bringup_ns.launch.py (robot_name:=robot1)
└── PushRosNamespace("robot1")
    └── mlm_bringup.launch.py
        ├── controller.launch.py
        │   ├── odom_publisher.launch.py
        │   │   ├── odom_publisher_node
        │   │   │   ├── 발행: odom → /robot1/odom
        │   │   │   └── 구독: cmd_vel → /robot1/cmd_vel ✅
        │   │   │   └── 구독: controller/cmd_vel → /robot1/controller/cmd_vel
        │   │   │   └── 구독: /app/cmd_vel → /app/cmd_vel (절대경로)
        │   │   └── ros_robot_controller_node
        │   │       └── 발행: imu_raw → /robot1/ros_robot_controller/imu_raw
        │   └── imu_filter.launch.py
        │       └── imu_filter_node
        │           └── 발행: imu → /robot1/imu
        ├── usb_cam.launch.py
        │   └── usb_cam_node (name="usb_cam")
        │       └── 발행: image_raw → /robot1/usb_cam/image_raw
        └── lidar.launch.py
            └── lidar_node
                └── 발행: scan_raw → /robot1/scan_raw
```

### 1.3 Emergency Stop 시스템 (GT) 실제 코드 분석

#### sync_node.py 토픽 분석
```
파일: mlm_avoid_sync/mlm_avoid_sync/sync_node.py

구독 토픽 (파라미터화됨):
├── Line 48: self.declare_parameter('image_topic', '/image')
├── Line 49: self.declare_parameter('scan_topic', '/scan_raw')
├── Line 50: self.declare_parameter('odom_topic', '/odom')
└── Line 51: self.declare_parameter('imu_topic', '/imu')

발행 토픽 (하드코딩):
└── Line 93: '/mlm_avoid/synced_data' ⚠️ 하드코딩
```

#### safety_node.py 토픽 분석
```
파일: mlm_avoid_safety/mlm_avoid_safety/safety_node.py

구독 토픽 (하드코딩):
├── Line 76: '/mlm_avoid/synced_data' ⚠️ 하드코딩
└── Line 82: '/cmd_vel_nav' ⚠️ 하드코딩 - 🔴 CRITICAL

발행 토픽 (하드코딩):
└── Line 88: '/cmd_vel' ⚠️ 하드코딩 - 🔴 CRITICAL
```

#### motion_controller.py 토픽 분석 (path_player_pkg)
```
파일: path_player_pkg/path_player_pkg/motion_controller.py

구독 토픽 (하드코딩):
├── Line 34: '/target_point'
├── Line 41: '/target_velocity'
└── Line 49: '/odom' ⚠️ 하드코딩

발행 토픽 (하드코딩):
└── Line 55: '/cmd_vel_nav' ⚠️ 하드코딩
```

---

## 2. 토픽 매핑 테이블

### 2.1 DOG (robot1) 실제 발행 토픽

| 발행 노드 | 토픽명 | 메시지 타입 |
|----------|--------|-------------|
| usb_cam_node | `/robot1/usb_cam/image_raw` | sensor_msgs/Image |
| lidar_node | `/robot1/scan_raw` | sensor_msgs/LaserScan |
| ekf_filter_node | `/robot1/odom` | nav_msgs/Odometry |
| imu_filter_node | `/robot1/imu` | sensor_msgs/Imu |

### 2.2 DOG 로봇 제어기 구독 토픽

| 구독 토픽 | 우선순위 | 설명 |
|----------|----------|------|
| `/robot1/cmd_vel` | 1 | **메인 제어 토픽** |
| `/robot1/controller/cmd_vel` | 2 | 컨트롤러 전용 |
| `/app/cmd_vel` | 3 | 앱 전용 (절대경로) |

### 2.3 Nav2 토픽 흐름 (DOG)

```
Nav2 controller_server
    └── 발행: /robot1/cmd_vel_nav
           │
           ▼
Nav2 velocity_smoother
    ├── 구독: /robot1/cmd_vel_nav
    └── 발행: /robot1/cmd_vel
           │
           ▼
odom_publisher_node
    └── 구독: /robot1/cmd_vel → 로봇 제어
```

### 2.4 Emergency Stop 통합 후 예상 토픽 흐름

```
센서 토픽:
/robot1/usb_cam/image_raw ──┐
/robot1/scan_raw ───────────┼──► sync_node
/robot1/odom ───────────────┤       │
/robot1/imu ────────────────┘       ▼
                           /mlm_avoid/synced_data
                                    │
                                    ▼
Nav2 ──► /robot1/cmd_vel_nav ──► safety_node
                                    │
                                    ▼
                           /robot1/cmd_vel ──► 로봇

⚠️ velocity_smoother는 비활성화 또는 safety_node가 대체
```

---

## 3. 이식 대상 패키지

### 3.1 패키지 목록

| 패키지 | 경로 | 파일 수 | 수정 필요 |
|--------|------|---------|----------|
| mlm_avoid_msgs | src/mlm_avoid_msgs/ | 4 | ❌ |
| mlm_avoid_sync | src/mlm_avoid_sync/ | 8 | ✅ |
| mlm_avoid_safety | src/mlm_avoid_safety/ | 9 | ✅ |
| mlm_avoid_bringup | src/mlm_avoid_bringup/ | 6 | ✅ |
| path_player_pkg | src/path_player_pkg/ | 15 | ✅ (선택) |

### 3.2 각 패키지 구조

```
mlm_avoid_msgs/
├── msg/SyncedData.msg
├── package.xml
├── CMakeLists.txt
└── README.md

mlm_avoid_sync/
├── mlm_avoid_sync/
│   ├── __init__.py
│   ├── sync_node.py          ← 수정 필요
│   └── sensor_synchronizer.py
├── launch/sync.launch.py
├── config/                    ← (비어있음)
├── test/
├── setup.py
└── package.xml

mlm_avoid_safety/
├── mlm_avoid_safety/
│   ├── __init__.py
│   ├── safety_node.py        ← 수정 필요
│   └── ttc_calculator.py
├── launch/                    ← (비어있음)
├── config/                    ← (비어있음)
├── test/
├── setup.py
└── package.xml

mlm_avoid_bringup/
├── config/
│   └── avoid_params.yaml     ← 수정 필요
├── launch/
│   ├── avoid.launch.py       ← 수정 필요
│   ├── safety_only.launch.py
│   └── full_system.launch.py
└── package.xml
```

---

## 4. 상세 코드 변경 사항

### 4.1 mlm_avoid_sync/mlm_avoid_sync/sync_node.py

#### 변경 1: synced_data 토픽 파라미터화

**위치:** Line 51 근처 (파라미터 선언부)

```python
# ========== 변경 전 ==========
        self.declare_parameter('imu_topic', '/imu')

# ========== 변경 후 ==========
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('synced_data_topic', '/mlm_avoid/synced_data')
```

**위치:** Line 93 근처 (발행자 생성부)

```python
# ========== 변경 전 ==========
        # 발행자 생성
        self.pub_synced = self.create_publisher(
            SyncedData,
            '/mlm_avoid/synced_data',
            10
        )

# ========== 변경 후 ==========
        # 발행자 생성
        synced_data_topic = self.get_parameter('synced_data_topic').value
        self.pub_synced = self.create_publisher(
            SyncedData,
            synced_data_topic,
            10
        )
```

**위치:** Line 100 근처 (로그 출력부)

```python
# ========== 변경 전 ==========
        self.get_logger().info(f'Sync node initialized (buffer_size={buffer_size})')

# ========== 변경 후 ==========
        self.get_logger().info(
            f'Sync node initialized\n'
            f'  buffer_size={buffer_size}\n'
            f'  image_topic={image_topic}\n'
            f'  scan_topic={scan_topic}\n'
            f'  odom_topic={odom_topic}\n'
            f'  imu_topic={imu_topic}\n'
            f'  synced_data_topic={synced_data_topic}'
        )
```

---

### 4.2 mlm_avoid_safety/mlm_avoid_safety/safety_node.py

#### 변경 1: 토픽 파라미터 선언 추가

**위치:** Line 49 근처 (파라미터 선언부)

```python
# ========== 변경 전 ==========
        self.declare_parameter('ttc_threshold_start', 2.0)
        self.declare_parameter('ttc_threshold_end', 3.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)  # 명령 타임아웃 (초)

# ========== 변경 후 ==========
        self.declare_parameter('ttc_threshold_start', 2.0)
        self.declare_parameter('ttc_threshold_end', 3.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)  # 명령 타임아웃 (초)

        # 토픽 파라미터 (namespace 지원용)
        self.declare_parameter('synced_data_topic', '/mlm_avoid/synced_data')
        self.declare_parameter('cmd_vel_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
```

#### 변경 2: 토픽 파라미터 읽기

**위치:** Line 55 근처 (파라미터 읽기부)

```python
# ========== 변경 전 ==========
        ttc_start = self.get_parameter('ttc_threshold_start').value
        ttc_end = self.get_parameter('ttc_threshold_end').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

# ========== 변경 후 ==========
        ttc_start = self.get_parameter('ttc_threshold_start').value
        ttc_end = self.get_parameter('ttc_threshold_end').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        # 토픽 파라미터 읽기
        synced_data_topic = self.get_parameter('synced_data_topic').value
        cmd_vel_nav_topic = self.get_parameter('cmd_vel_nav_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
```

#### 변경 3: synced_data 구독 토픽 변경

**위치:** Line 76 근처

```python
# ========== 변경 전 ==========
        self.sub_synced = self.create_subscription(
            SyncedData,
            '/mlm_avoid/synced_data',
            self.on_synced_data,
            sensor_qos
        )

# ========== 변경 후 ==========
        self.sub_synced = self.create_subscription(
            SyncedData,
            synced_data_topic,
            self.on_synced_data,
            sensor_qos
        )
```

#### 변경 4: cmd_vel_nav 구독 토픽 변경

**위치:** Line 82 근처

```python
# ========== 변경 전 ==========
        self.sub_cmd_nav = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.on_cmd_vel_nav,
            cmd_qos
        )

# ========== 변경 후 ==========
        self.sub_cmd_nav = self.create_subscription(
            Twist,
            cmd_vel_nav_topic,
            self.on_cmd_vel_nav,
            cmd_qos
        )
```

#### 변경 5: cmd_vel 발행 토픽 변경

**위치:** Line 88 근처

```python
# ========== 변경 전 ==========
        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            cmd_qos
        )

# ========== 변경 후 ==========
        self.pub_cmd_vel = self.create_publisher(
            Twist,
            cmd_vel_topic,
            cmd_qos
        )
```

#### 변경 6: 로그 출력 개선

**위치:** Line 95 근처

```python
# ========== 변경 전 ==========
        self.get_logger().info(
            f'Safety node initialized (TTC thresholds: '
            f'start={ttc_start}s, end={ttc_end}s)'
        )

# ========== 변경 후 ==========
        self.get_logger().info(
            f'Safety node initialized\n'
            f'  TTC thresholds: start={ttc_start}s, end={ttc_end}s\n'
            f'  synced_data_topic: {synced_data_topic}\n'
            f'  cmd_vel_nav_topic: {cmd_vel_nav_topic}\n'
            f'  cmd_vel_topic: {cmd_vel_topic}'
        )
```

---

### 4.3 mlm_avoid_bringup/config/avoid_params.yaml

**전체 파일 교체:**

```yaml
# ========== 변경 전 (GT 원본) ==========
mlm_avoid_sync_node:
  ros__parameters:
    buffer_size: 30
    image_topic: "/ascamera/camera_publisher/rgb0/image"
    scan_topic: "/scan_raw"
    odom_topic: "/odom"
    imu_topic: "/imu"

mlm_avoid_safety_node:
  ros__parameters:
    ttc_threshold_start: 2.0
    ttc_threshold_end: 3.0
    cmd_vel_timeout: 0.5

# ========== 변경 후 (DOG robot1용) ==========
# MLM Avoid System Parameters for DOG (robot1)
# Emergency Stop System 설정 파일

mlm_avoid_sync_node:
  ros__parameters:
    # 버퍼 크기
    buffer_size: 30

    # 센서 토픽 (robot1 namespace 적용)
    image_topic: "/robot1/usb_cam/image_raw"
    scan_topic: "/robot1/scan_raw"
    odom_topic: "/robot1/odom"
    imu_topic: "/robot1/imu"

    # 내부 통신 토픽
    synced_data_topic: "/mlm_avoid/synced_data"

mlm_avoid_safety_node:
  ros__parameters:
    # TTC 임계값 (초)
    ttc_threshold_start: 2.0  # 개입 시작
    ttc_threshold_end: 3.0    # 개입 종료
    cmd_vel_timeout: 0.5      # 명령 타임아웃

    # 토픽 (robot1 namespace 적용)
    synced_data_topic: "/mlm_avoid/synced_data"
    cmd_vel_nav_topic: "/robot1/cmd_vel_nav"
    cmd_vel_topic: "/robot1/cmd_vel"
```

---

### 4.4 mlm_avoid_bringup/launch/avoid.launch.py

**변경 없음** - 파라미터 파일에서 토픽을 설정하므로 launch 파일 수정 불필요

---

### 4.5 path_player_pkg/path_player_pkg/motion_controller.py (선택)

> ⚠️ path_player_pkg는 Emergency Stop 핵심 기능이 아닌 선택 사항입니다.
> Nav2 사용 시에는 motion_controller가 필요 없습니다.

#### 변경 1: 파라미터 선언 추가

**위치:** Line 20 근처 (__init__ 시작부)

```python
# ========== 변경 전 ==========
        self.declare_parameter('drive_type', 'Ackermann')
        self.declare_parameter('angular_p_gain', 2.0)
        # ... 기존 파라미터들 ...

# ========== 변경 후 ==========
        # 토픽 파라미터 (namespace 지원)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('target_point_topic', '/target_point')
        self.declare_parameter('target_velocity_topic', '/target_velocity')

        self.declare_parameter('drive_type', 'Ackermann')
        self.declare_parameter('angular_p_gain', 2.0)
        # ... 기존 파라미터들 ...
```

#### 변경 2: 토픽 파라미터 읽기 및 적용

**위치:** Line 33 근처 (구독자 생성부)

```python
# ========== 변경 전 ==========
        self.target_sub = self.create_subscription(
            PoseStamped,
            '/target_point',
            self.target_callback,
            10
        )

        self.velocity_sub = self.create_subscription(
            Float64,
            '/target_velocity',
            self.velocity_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

# ========== 변경 후 ==========
        # 토픽 파라미터 읽기
        odom_topic = self.get_parameter('odom_topic').value
        cmd_vel_nav_topic = self.get_parameter('cmd_vel_nav_topic').value
        target_point_topic = self.get_parameter('target_point_topic').value
        target_velocity_topic = self.get_parameter('target_velocity_topic').value

        self.target_sub = self.create_subscription(
            PoseStamped,
            target_point_topic,
            self.target_callback,
            10
        )

        self.velocity_sub = self.create_subscription(
            Float64,
            target_velocity_topic,
            self.velocity_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_nav_topic, 10)
```

---

## 5. 단계별 이식 절차

### Phase 1: 준비 (PC에서)

```bash
# 1. 작업 디렉토리 생성
mkdir -p ~/mlm_ws/emergency_stop_transplant/src
cd ~/mlm_ws/emergency_stop_transplant

# 2. GT 컨테이너에서 패키지 복사
# 먼저 GT 호스트의 tmp 폴더로 복사
ssh pi@172.16.10.37 "docker cp MentorPi:/home/ubuntu/ros2_ws/src/mlm_avoid_msgs /home/pi/docker/tmp/"
ssh pi@172.16.10.37 "docker cp MentorPi:/home/ubuntu/ros2_ws/src/mlm_avoid_sync /home/pi/docker/tmp/"
ssh pi@172.16.10.37 "docker cp MentorPi:/home/ubuntu/ros2_ws/src/mlm_avoid_safety /home/pi/docker/tmp/"
ssh pi@172.16.10.37 "docker cp MentorPi:/home/ubuntu/ros2_ws/src/mlm_avoid_bringup /home/pi/docker/tmp/"

# 3. GT 호스트에서 PC로 복사
scp -r pi@172.16.10.37:/home/pi/docker/tmp/mlm_avoid_msgs ./src/
scp -r pi@172.16.10.37:/home/pi/docker/tmp/mlm_avoid_sync ./src/
scp -r pi@172.16.10.37:/home/pi/docker/tmp/mlm_avoid_safety ./src/
scp -r pi@172.16.10.37:/home/pi/docker/tmp/mlm_avoid_bringup ./src/

# (선택) path_player_pkg
ssh pi@172.16.10.37 "docker cp MentorPi:/home/ubuntu/ros2_ws/src/path_player_pkg /home/pi/docker/tmp/"
scp -r pi@172.16.10.37:/home/pi/docker/tmp/path_player_pkg ./src/
```

### Phase 2: 코드 수정 (PC에서)

이 문서의 **섹션 4** 변경 사항을 순서대로 적용:

1. `src/mlm_avoid_sync/mlm_avoid_sync/sync_node.py` 수정
2. `src/mlm_avoid_safety/mlm_avoid_safety/safety_node.py` 수정
3. `src/mlm_avoid_bringup/config/avoid_params.yaml` 수정
4. (선택) `src/path_player_pkg/path_player_pkg/motion_controller.py` 수정

### Phase 3: DOG로 전송

```bash
# 1. PC에서 DOG 호스트의 tmp 폴더로 복사
scp -r ./src/mlm_avoid_msgs pi@172.16.10.172:/home/pi/docker/tmp/
scp -r ./src/mlm_avoid_sync pi@172.16.10.172:/home/pi/docker/tmp/
scp -r ./src/mlm_avoid_safety pi@172.16.10.172:/home/pi/docker/tmp/
scp -r ./src/mlm_avoid_bringup pi@172.16.10.172:/home/pi/docker/tmp/

# 2. DOG 호스트에서 컨테이너로 복사
ssh pi@172.16.10.172 "docker cp /home/pi/docker/tmp/mlm_avoid_msgs MentorPi:/home/ubuntu/ros2_ws/src/"
ssh pi@172.16.10.172 "docker cp /home/pi/docker/tmp/mlm_avoid_sync MentorPi:/home/ubuntu/ros2_ws/src/"
ssh pi@172.16.10.172 "docker cp /home/pi/docker/tmp/mlm_avoid_safety MentorPi:/home/ubuntu/ros2_ws/src/"
ssh pi@172.16.10.172 "docker cp /home/pi/docker/tmp/mlm_avoid_bringup MentorPi:/home/ubuntu/ros2_ws/src/"
```

### Phase 4: 빌드 (DOG 컨테이너)

```bash
# 1. DOG SSH 접속
ssh pi@172.16.10.172

# 2. 컨테이너 접속
docker exec -it MentorPi bash

# 3. ROS2 환경 설정
source /opt/ros/humble/setup.bash
cd /home/ubuntu/ros2_ws

# 4. 메시지 패키지 먼저 빌드 (의존성)
colcon build --packages-select mlm_avoid_msgs
source install/setup.bash

# 5. 나머지 패키지 빌드
colcon build --packages-select mlm_avoid_sync mlm_avoid_safety mlm_avoid_bringup
source install/setup.bash

# 6. 빌드 확인
ros2 pkg list | grep mlm_avoid
# 예상 출력:
# mlm_avoid_bringup
# mlm_avoid_msgs
# mlm_avoid_safety
# mlm_avoid_sync
```

### Phase 5: 테스트

```bash
# 터미널 1: bringup 실행 (DOG 컨테이너)
ros2 launch bringup bringup_ns.launch.py robot_name:=robot1

# 터미널 2: Emergency Stop 실행 (DOG 컨테이너)
ros2 launch mlm_avoid_bringup avoid.launch.py

# 터미널 3: 토픽 확인 (DOG 컨테이너 또는 PC)
ros2 topic list | grep -E "mlm_avoid|cmd_vel|robot1"

# 예상 출력:
# /mlm_avoid/synced_data
# /robot1/cmd_vel
# /robot1/cmd_vel_nav
# /robot1/odom
# /robot1/scan_raw
# /robot1/imu
# /robot1/usb_cam/image_raw
```

---

## 6. 검증 체크리스트

### 6.1 빌드 검증

- [ ] `colcon build` 성공 (에러 없음)
- [ ] `ros2 pkg list | grep mlm_avoid` 4개 패키지 표시

### 6.2 토픽 검증

```bash
# sync_node 토픽 확인
ros2 topic info /mlm_avoid/synced_data
# 예상: Publisher: mlm_avoid_sync_node, Subscriber: mlm_avoid_safety_node

# safety_node 입력 확인
ros2 topic echo /robot1/cmd_vel_nav --once
# 예상: Nav2 또는 motion_controller의 Twist 메시지

# safety_node 출력 확인
ros2 topic echo /robot1/cmd_vel --once
# 예상: safety_node의 Twist 메시지
```

### 6.3 동기화 검증

```bash
# synced_data 발행 확인
ros2 topic hz /mlm_avoid/synced_data
# 예상: 카메라 FPS (약 15-30Hz)

# 동기화 실패 시 로그 확인
# "Sync failed: waiting for sensors: ['image']" 등
```

### 6.4 긴급 정지 검증

```bash
# 1. 로봇 앞에 장애물 배치 (약 1m 거리)
# 2. 주행 명령 전송
ros2 topic pub /robot1/cmd_vel_nav geometry_msgs/Twist "{linear: {x: 0.3}}" --once

# 3. safety_node 로그 확인
# 예상: "[WARN] EMERGENCY STOP! TTC=X.XXs"

# 4. 실제 /robot1/cmd_vel 확인
ros2 topic echo /robot1/cmd_vel
# 예상: linear.x = 0.0 (정지)
```

---

## 7. 롤백 절차

문제 발생 시 원상복구:

```bash
# DOG 컨테이너에서
cd /home/ubuntu/ros2_ws/src
rm -rf mlm_avoid_msgs mlm_avoid_sync mlm_avoid_safety mlm_avoid_bringup

# 재빌드
cd /home/ubuntu/ros2_ws
colcon build
source install/setup.bash
```

---

## 8. 주의사항

### 8.1 velocity_smoother와의 관계

Nav2의 `velocity_smoother`는 `/robot1/cmd_vel_nav` → `/robot1/cmd_vel` 경로를 사용합니다.
Emergency Stop의 `safety_node`도 동일한 경로를 사용하므로 **충돌이 발생**합니다.

**해결 방안:**
1. velocity_smoother 비활성화
2. 또는 safety_node의 출력 토픽을 `/robot1/cmd_vel_safe`로 변경하고, velocity_smoother가 이를 구독하도록 수정

### 8.2 기존 코드 수정 금지

이 문서의 모든 수정 사항은 **이식하는 Emergency Stop 패키지**에만 적용됩니다.
DOG 로봇의 기존 코드는 절대 수정하지 않습니다.

---

## 9. 변경 이력

| 날짜 | 버전 | 변경 내용 |
|------|------|----------|
| 2026-01-27 | 1.0 | 초기 문서 작성 - 정밀 코드 분석 기반 |

---

## 10. 첨부: 수정 파일 전체 코드

### 10.1 sync_node.py (수정 후 전체)

```python
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
    """

    def __init__(self):
        super().__init__('mlm_avoid_sync_node')

        # 파라미터 선언
        self.declare_parameter('buffer_size', 30)
        self.declare_parameter('image_topic', '/image')
        self.declare_parameter('scan_topic', '/scan_raw')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('synced_data_topic', '/mlm_avoid/synced_data')

        # 파라미터 가져오기
        buffer_size = self.get_parameter('buffer_size').value
        image_topic = self.get_parameter('image_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        synced_data_topic = self.get_parameter('synced_data_topic').value

        # 동기화기 생성
        self.synchronizer = SensorSynchronizer(buffer_size=buffer_size)

        # QoS 설정
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 구독자 생성
        self.sub_image = self.create_subscription(
            Image, image_topic, self.on_image, sensor_qos)
        self.sub_scan = self.create_subscription(
            LaserScan, scan_topic, self.on_scan, sensor_qos)
        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self.on_odom, sensor_qos)
        self.sub_imu = self.create_subscription(
            Imu, imu_topic, self.on_imu, sensor_qos)

        # 발행자 생성
        self.pub_synced = self.create_publisher(
            SyncedData, synced_data_topic, 10)

        # 통계
        self.sync_count = 0
        self.fail_count = 0

        self.get_logger().info(
            f'Sync node initialized\n'
            f'  buffer_size={buffer_size}\n'
            f'  image_topic={image_topic}\n'
            f'  scan_topic={scan_topic}\n'
            f'  odom_topic={odom_topic}\n'
            f'  imu_topic={imu_topic}\n'
            f'  synced_data_topic={synced_data_topic}'
        )

    def on_image(self, msg: Image) -> None:
        synced = self.synchronizer.on_image(msg)
        if synced is not None:
            self.publish_synced_data(synced)
            self.sync_count += 1
            if self.sync_count == 1:
                self.get_logger().info('First sync successful!')
        else:
            self.fail_count += 1
            if self.fail_count <= 10 or self.fail_count % 100 == 0:
                missing = [
                    name for name in self.synchronizer.SENSOR_NAMES
                    if self.synchronizer.latest[name] is None
                ]
                if missing:
                    self.get_logger().warn(f'Sync failed: waiting for sensors: {missing}')

    def on_scan(self, msg: LaserScan) -> None:
        self.synchronizer.add_to_buffer('scan', msg)

    def on_odom(self, msg: Odometry) -> None:
        self.synchronizer.add_to_buffer('odom', msg)

    def on_imu(self, msg: Imu) -> None:
        self.synchronizer.add_to_buffer('imu', msg)

    def publish_synced_data(self, synced: dict) -> None:
        msg = SyncedData()
        msg.header = synced['image'].header
        msg.image = synced['image']
        msg.scan = synced['scan']
        msg.odom = synced['odom']
        msg.imu = synced['imu']
        self.pub_synced.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Shutting down. Sync success: {node.sync_count}, fail: {node.fail_count}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 10.2 safety_node.py (수정 후 전체)

```python
"""
ROS2 Safety Node for emergency stop system.

TTC 기반 개입 판단 및 cmd_vel mux 기능을 수행하는 ROS2 노드.
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
    """

    def __init__(self):
        super().__init__('mlm_avoid_safety_node')

        # 파라미터 선언
        self.declare_parameter('ttc_threshold_start', 2.0)
        self.declare_parameter('ttc_threshold_end', 3.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)

        # 토픽 파라미터 (namespace 지원용)
        self.declare_parameter('synced_data_topic', '/mlm_avoid/synced_data')
        self.declare_parameter('cmd_vel_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # 파라미터 가져오기
        ttc_start = self.get_parameter('ttc_threshold_start').value
        ttc_end = self.get_parameter('ttc_threshold_end').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        synced_data_topic = self.get_parameter('synced_data_topic').value
        cmd_vel_nav_topic = self.get_parameter('cmd_vel_nav_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

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

        # 구독자 생성
        self.sub_synced = self.create_subscription(
            SyncedData, synced_data_topic, self.on_synced_data, sensor_qos)
        self.sub_cmd_nav = self.create_subscription(
            Twist, cmd_vel_nav_topic, self.on_cmd_vel_nav, cmd_qos)

        # 발행자 생성
        self.pub_cmd_vel = self.create_publisher(Twist, cmd_vel_topic, cmd_qos)

        # 주기적 cmd_vel 발행 타이머 (10Hz)
        self.timer = self.create_timer(0.1, self.on_timer)

        # 통계
        self.intervention_count = 0

        self.get_logger().info(
            f'Safety node initialized\n'
            f'  TTC thresholds: start={ttc_start}s, end={ttc_end}s\n'
            f'  synced_data_topic: {synced_data_topic}\n'
            f'  cmd_vel_nav_topic: {cmd_vel_nav_topic}\n'
            f'  cmd_vel_topic: {cmd_vel_topic}'
        )

    def on_synced_data(self, msg: SyncedData) -> None:
        vx = msg.odom.twist.twist.linear.x
        vy = msg.odom.twist.twist.linear.y

        min_ttc = self.ttc_calculator.calculate_min_ttc(
            ranges=msg.scan.ranges,
            angle_min=msg.scan.angle_min,
            angle_increment=msg.scan.angle_increment,
            vx=vx, vy=vy
        )

        prev_intervening = self.intervening
        self.intervening = self.ttc_calculator.should_intervene(
            ttc=min_ttc, currently_intervening=self.intervening)

        if self.intervening and not prev_intervening:
            self.intervention_count += 1
            self.get_logger().warn(
                f'EMERGENCY STOP! TTC={min_ttc:.2f}s (count: {self.intervention_count})')
        elif not self.intervening and prev_intervening:
            self.get_logger().info(f'Resuming navigation. TTC={min_ttc:.2f}s')

    def on_cmd_vel_nav(self, msg: Twist) -> None:
        self.cmd_vel_nav = msg
        self.last_nav_time = self.get_clock().now()

    def on_timer(self) -> None:
        now = self.get_clock().now()
        cmd = Twist()

        if self.intervening:
            cmd = Twist()  # 긴급 정지
        else:
            if self.cmd_vel_nav is not None:
                dt = (now - self.last_nav_time).nanoseconds / 1e9
                if dt < self.cmd_vel_timeout:
                    cmd = self.cmd_vel_nav

        self.pub_cmd_vel.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'Shutting down. Interventions: {node.intervention_count}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 10.3 avoid_params.yaml (수정 후 전체)

```yaml
# MLM Avoid System Parameters for DOG (robot1)
# Emergency Stop System 설정 파일
#
# 이 파일은 DOG 로봇(robot1 namespace)에 맞게 설정되었습니다.
# GT 로봇에서 사용하려면 토픽 prefix를 제거하세요.

mlm_avoid_sync_node:
  ros__parameters:
    # 센서 데이터 버퍼 크기
    buffer_size: 30

    # 센서 토픽 (robot1 namespace 적용)
    image_topic: "/robot1/usb_cam/image_raw"
    scan_topic: "/robot1/scan_raw"
    odom_topic: "/robot1/odom"
    imu_topic: "/robot1/imu"

    # 동기화 데이터 발행 토픽 (내부 통신용)
    synced_data_topic: "/mlm_avoid/synced_data"

mlm_avoid_safety_node:
  ros__parameters:
    # TTC 임계값 설정
    # ttc_threshold_start: 이 값 미만이면 긴급 정지 시작
    # ttc_threshold_end: 이 값 초과하면 긴급 정지 해제
    # 히스테리시스로 경계값 근처 떨림 방지
    ttc_threshold_start: 2.0  # 초
    ttc_threshold_end: 3.0    # 초

    # 명령 타임아웃 (이 시간 동안 명령이 없으면 정지)
    cmd_vel_timeout: 0.5  # 초

    # 토픽 설정 (robot1 namespace 적용)
    synced_data_topic: "/mlm_avoid/synced_data"
    cmd_vel_nav_topic: "/robot1/cmd_vel_nav"
    cmd_vel_topic: "/robot1/cmd_vel"
```
