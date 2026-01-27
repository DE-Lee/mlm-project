# Emergency Stop System Implementation Log
# DOG Robot (robot1) 이식 작업 기록

**작업일:** 2026-01-27
**작업자:** Claude Code Assistant
**버전:** 1.3
**상태:** bringup 통합 완료, Nav2 통합 테스트 대기

---

## 1. 작업 개요

### 1.1 목적
GT 로봇의 Emergency Stop 시스템을 DOG 로봇(robot1)에 이식하여 Nav2 자율주행 시 TTC 기반 긴급정지 기능 제공

### 1.2 적용 범위
- Nav2 자율주행 경로만 Emergency Stop 적용
- 수동제어(Web Dashboard, 조이스틱)는 기존 방식 유지 (운전자 책임)

### 1.3 설계 원칙
- Nav2 TTC 비활성화, Emergency Stop TTC로 단일화
- 안전 판단은 로봇 로컬에서 수행 (Edge Safety)
- Fail-Safe: 모든 장애 상황에서 로봇 정지

---

## 2. 변경 전 시스템 상태 (Baseline)

### 2.1 토픽 흐름 (변경 전)
```
PC Nav2:
  controller_server ──▶ /robot1/cmd_vel ──▶ odom_publisher ──▶ Motors
  behavior_server ────▶ /robot1/cmd_vel ──┘

Robot 수동제어:
  Web/조이스틱 ──▶ /robot1/controller/cmd_vel ──▶ odom_publisher
```

### 2.2 Nav2 TTC 설정 (변경 전)
**파일:** `pc/ros2_ws/src/navigation/config/nav2_params_ackermann.yaml`
```yaml
FollowPath:
  use_collision_detection: true
  max_allowed_time_to_collision_up_to_carrot: 1.0
```

### 2.3 Nav2 remapping (변경 전)
**파일:** `pc/ros2_ws/src/navigation/launch/navigation_pc.launch.py`
**Line 116:**
```python
remappings=costmap_remappings + [('cmd_vel', 'cmd_vel')],
```

---

## 3. 변경 사항 목록

### 3.1 PC 측 변경

| # | 파일 | 변경 내용 | 상태 |
|---|------|----------|------|
| 1 | navigation_pc.launch.py | cmd_vel → cmd_vel_nav remapping | [x] 완료 |
| 2 | nav2_params_ackermann.yaml | use_collision_detection: false | [x] 완료 |

### 3.2 Robot 측 변경

| # | 파일 | 변경 내용 | 상태 |
|---|------|----------|------|
| 3 | mlm_avoid_msgs/ | 패키지 추가 (수정 없음) | [x] 완료 |
| 4 | mlm_avoid_sync/sync_node.py | synced_data_topic 파라미터화 | [x] 완료 |
| 5 | mlm_avoid_safety/safety_node.py | 토픽 파라미터화 | [x] 완료 |
| 6 | mlm_avoid_bringup/config/avoid_params.yaml | robot1 토픽 설정 | [x] 완료 |
| 7 | mlm_avoid_bringup/package.xml | path_player_pkg 의존성 제거 | [x] 완료 |
| 8 | bringup/launch/mlm_bringup.launch.py | Emergency Stop 통합 | [x] 완료 |
| 9 | peripherals/config/usb_cam_param.yaml | av_device_format 제거 (카메라 크래시 수정) | [x] 완료 |

### 3.3 CycloneDDS 설정 변경 (PC IP 변경 대응)

| # | 파일 | 변경 내용 | 상태 |
|---|------|----------|------|
| 10 | pc/cyclonedds/cyclonedds.xml | PC IP 172.16.11.203 → 172.16.11.222 | [x] 완료 |
| 11 | robot/cyclonedds/cyclonedds.xml | PC IP 172.16.11.203 → 172.16.11.222 | [x] 완료 |
| 12 | robot/cyclonedds/cyclonedds_robot2.xml | PC IP 172.16.11.203 → 172.16.11.222 | [x] 완료 |
| 13 | robot2/cyclonedds/cyclonedds.xml | PC IP 172.16.11.203 → 172.16.11.222 | [x] 완료 |
| 14 | robot2/cyclonedds/cyclonedds_robot2.xml | PC IP 172.16.11.203 → 172.16.11.222 | [x] 완료 |
| 15 | DOG /home/ubuntu/cyclonedds.xml | PC IP 172.16.11.222로 전송 완료 | [x] 완료 |

---

## 4. 상세 변경 기록

### 4.1 [PC] navigation_pc.launch.py

**파일 경로:** `/home/lee/mlm_ws/pc/ros2_ws/src/navigation/launch/navigation_pc.launch.py`

**변경 위치:** Line 116

**변경 전:**
```python
remappings=costmap_remappings + [('cmd_vel', 'cmd_vel')],
```

**변경 후:**
```python
remappings=costmap_remappings + [('cmd_vel', 'cmd_vel_nav')],
```

**변경 이유:**
- Nav2 출력을 cmd_vel_nav로 분리하여 safety_node가 반드시 거쳐가도록 강제
- odom_publisher가 Nav2 명령을 직접 받는 것을 방지

**영향 범위:**
- controller_server 출력 토픽 변경
- behavior_server 출력 토픽 변경

**롤백 방법:**
```python
remappings=costmap_remappings + [('cmd_vel', 'cmd_vel')],
```

---

### 4.2 [PC] nav2_params_ackermann.yaml

**파일 경로:** `/home/lee/mlm_ws/pc/ros2_ws/src/navigation/config/nav2_params_ackermann.yaml`

**변경 위치:** Line 289-290

**변경 전:**
```yaml
FollowPath:
  # ... 기존 설정 ...
  use_collision_detection: true
  max_allowed_time_to_collision_up_to_carrot: 1.0
```

**변경 후:**
```yaml
FollowPath:
  # ... 기존 설정 ...
  use_collision_detection: false  # Emergency Stop이 TTC 담당
  # max_allowed_time_to_collision_up_to_carrot: 1.0  # 불필요
```

**변경 이유:**
- Nav2 TTC와 Emergency Stop TTC 중복 제거
- 단일 TTC 시스템으로 예측 가능한 동작 보장

**롤백 방법:**
```yaml
use_collision_detection: true
max_allowed_time_to_collision_up_to_carrot: 1.0
```

---

### 4.3 [Robot] mlm_avoid_sync/sync_node.py

**파일 경로:** `mlm_avoid_sync/mlm_avoid_sync/sync_node.py`

**변경 내용:** synced_data_topic 파라미터 추가

**변경 전 (Line 51 근처):**
```python
self.declare_parameter('imu_topic', '/imu')
```

**변경 후:**
```python
self.declare_parameter('imu_topic', '/imu')
self.declare_parameter('synced_data_topic', '/mlm_avoid/synced_data')
```

**변경 전 (Line 93 근처):**
```python
self.pub_synced = self.create_publisher(
    SyncedData,
    '/mlm_avoid/synced_data',
    10
)
```

**변경 후:**
```python
synced_data_topic = self.get_parameter('synced_data_topic').value
self.pub_synced = self.create_publisher(
    SyncedData,
    synced_data_topic,
    10
)
```

**변경 이유:**
- 하드코딩된 토픽명을 파라미터로 변경하여 namespace 지원

---

### 4.4 [Robot] mlm_avoid_safety/safety_node.py

**파일 경로:** `mlm_avoid_safety/mlm_avoid_safety/safety_node.py`

**변경 내용:** 토픽 파라미터화

**변경 전 (Line 49 근처):**
```python
self.declare_parameter('ttc_threshold_start', 2.0)
self.declare_parameter('ttc_threshold_end', 3.0)
self.declare_parameter('cmd_vel_timeout', 0.5)
```

**변경 후:**
```python
self.declare_parameter('ttc_threshold_start', 2.0)
self.declare_parameter('ttc_threshold_end', 3.0)
self.declare_parameter('cmd_vel_timeout', 0.5)

# 토픽 파라미터 (namespace 지원)
self.declare_parameter('synced_data_topic', '/mlm_avoid/synced_data')
self.declare_parameter('cmd_vel_nav_topic', '/cmd_vel_nav')
self.declare_parameter('cmd_vel_topic', '/cmd_vel')
```

**변경 전 (Line 76, 82, 88 - 토픽 생성부):**
```python
# 하드코딩된 토픽
'/mlm_avoid/synced_data'
'/cmd_vel_nav'
'/cmd_vel'
```

**변경 후:**
```python
# 파라미터에서 읽은 토픽
synced_data_topic = self.get_parameter('synced_data_topic').value
cmd_vel_nav_topic = self.get_parameter('cmd_vel_nav_topic').value
cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
```

---

### 4.5 [Robot] mlm_avoid_bringup/config/avoid_params.yaml

**파일 경로:** `mlm_avoid_bringup/config/avoid_params.yaml`

**전체 교체:**

**변경 전 (GT 원본):**
```yaml
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
```

**변경 후 (DOG robot1용):**
```yaml
# Emergency Stop System Configuration for DOG (robot1)
# 작성일: 2026-01-27

mlm_avoid_sync_node:
  ros__parameters:
    buffer_size: 30

    # 센서 토픽 (robot1 namespace)
    # ※ DOG는 /robot1/image_raw로 발행 (GT의 /ascamera/...와 다름)
    image_topic: "/robot1/image_raw"
    scan_topic: "/robot1/scan_raw"
    odom_topic: "/robot1/odom"
    imu_topic: "/robot1/imu"

    # 내부 통신 토픽
    synced_data_topic: "/mlm_avoid/synced_data"

mlm_avoid_safety_node:
  ros__parameters:
    # TTC 임계값
    ttc_threshold_start: 2.0   # 이 값 미만 → 긴급정지 시작
    ttc_threshold_end: 3.0     # 이 값 초과 → 긴급정지 해제
    cmd_vel_timeout: 0.5       # 명령 타임아웃

    # 토픽 설정 (robot1 namespace)
    synced_data_topic: "/mlm_avoid/synced_data"
    cmd_vel_nav_topic: "/robot1/cmd_vel_nav"
    cmd_vel_topic: "/robot1/cmd_vel"
```

---

### 4.6 [Robot] mlm_avoid_bringup/package.xml

**파일 경로:** `mlm_avoid_bringup/package.xml`

**변경 내용:** path_player_pkg 의존성 제거

**변경 전:**
```xml
<exec_depend>mlm_avoid_safety</exec_depend>
<exec_depend>path_player_pkg</exec_depend>
```

**변경 후:**
```xml
<exec_depend>mlm_avoid_safety</exec_depend>
<!-- path_player_pkg 제거: DOG robot은 Nav2 사용, GT 전용 full_system.launch.py는 미사용 -->
```

**변경 이유:**
- DOG 로봇은 Nav2를 사용하므로 path_player_pkg 불필요
- GT 전용 full_system.launch.py를 사용하지 않음

---

### 4.7 [Robot] bringup/launch/mlm_bringup.launch.py

**파일 경로:** `/home/ubuntu/ros2_ws/src/bringup/launch/mlm_bringup.launch.py` (DOG 로봇 내부)

**변경 내용:** Emergency Stop 런치 통합

**변경 전:**
```python
if compiled == "True":
    controller_package_path = get_package_share_directory("controller")
    peripherals_package_path = get_package_share_directory("peripherals")
else:
    controller_package_path = "/home/ubuntu/ros2_ws/src/driver/controller"
    peripherals_package_path = "/home/ubuntu/ros2_ws/src/peripherals"
```

**변경 후:**
```python
if compiled == "True":
    controller_package_path = get_package_share_directory("controller")
    peripherals_package_path = get_package_share_directory("peripherals")
    avoid_package_path = get_package_share_directory("mlm_avoid_bringup")
else:
    controller_package_path = "/home/ubuntu/ros2_ws/src/driver/controller"
    peripherals_package_path = "/home/ubuntu/ros2_ws/src/peripherals"
    avoid_package_path = "/home/ubuntu/ros2_ws/src/mlm_avoid_bringup"
```

**추가된 런치:**
```python
# Emergency Stop 런치 (Nav2 자율주행용 TTC 기반 안전 시스템)
avoid_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(avoid_package_path, "launch/avoid.launch.py")),
)

return [
    # ... 기존 노드들 ...
    avoid_launch,  # Emergency Stop (Nav2 자율주행용)
]
```

**변경 이유:**
- bringup.sh 실행 시 Emergency Stop이 자동으로 함께 시작
- .zshenv 환경 변수가 자동 로드됨 (ROS_DOMAIN_ID, CYCLONEDDS_URI 등)

**롤백 방법:**
- avoid_package_path 관련 코드 삭제
- return 리스트에서 avoid_launch 제거

---

### 4.8 [전체] CycloneDDS IP 주소 변경

**변경 이유:** PC IP 주소 변경 (172.16.11.203 → 172.16.11.222)

**변경된 파일들:**

| 파일 | 변경 전 | 변경 후 |
|------|---------|---------|
| pc/cyclonedds/cyclonedds.xml | 172.16.11.203 | 172.16.11.222 |
| robot/cyclonedds/cyclonedds.xml | 172.16.11.203 | 172.16.11.222 |
| robot/cyclonedds/cyclonedds_robot2.xml | 172.16.11.203 | 172.16.11.222 |
| robot2/cyclonedds/cyclonedds.xml | 172.16.11.203 | 172.16.11.222 |
| robot2/cyclonedds/cyclonedds_robot2.xml | 172.16.11.203 | 172.16.11.222 |
| DOG /home/ubuntu/cyclonedds.xml | - | 172.16.11.222 (신규 전송) |

**롤백 방법:**
- IP 주소가 다시 변경되면 해당 주소로 업데이트

---

### 4.9 [Robot] peripherals/config/usb_cam_param.yaml

**파일 경로:** `/home/ubuntu/ros2_ws/src/peripherals/config/usb_cam_param.yaml` (DOG 로봇 내부)

**변경 내용:** av_device_format 설정 제거

**문제 증상:**
```
[usb_cam_node_exe-7] [swscaler @ 0x76680aed70] No accelerated colorspace conversion found from yuv422p to rgb24.
[usb_cam_node_exe-7] terminate called after throwing an instance of 'char*'
```

**원인 분석:**
- DOG 로봇 카메라는 YUYV 포맷만 지원 (v4l2-ctl --list-formats-ext 확인)
- av_device_format: "YUV422P" 설정이 swscaler 변환을 강제
- swscaler가 가속 변환기 없이 소프트웨어 변환 시도 후 크래시

**변경 전:**
```yaml
/**:
    ros__parameters:
      video_device: "/dev/video0"
      framerate: 30.0
      io_method: "mmap"
      frame_id: "camera"
      pixel_format: "yuyv"
      av_device_format: "YUV422P"  # 문제 원인
      image_width: 640
      image_height: 480
      camera_name: "usb_cam"
      camera_info_url: "package://peripherals/config/camera_info.yaml"
```

**변경 후:**
```yaml
/**:
    ros__parameters:
      video_device: "/dev/video0"
      framerate: 30.0
      io_method: "mmap"
      frame_id: "camera"
      pixel_format: "yuyv"
      # av_device_format 제거 - usb_cam이 자동 변환 처리
      image_width: 640
      image_height: 480
      camera_name: "usb_cam"
      camera_info_url: "package://peripherals/config/camera_info.yaml"
```

**변경 이유:**
- usb_cam 노드가 pixel_format만 보고 자동으로 적절한 변환 수행
- 강제 포맷 지정 시 변환 실패 가능성

**결과:**
- 카메라 노드 정상 시작 (크래시 해결)
- 경고는 남지만 정상 동작: `[swscaler @ ...] No accelerated colorspace conversion found from yuv422p to rgb24.`

**롤백 방법:**
```yaml
av_device_format: "YUV422P"
```
(단, 크래시 발생하므로 권장하지 않음)

---

## 5. 변경 후 시스템 상태

### 5.1 토픽 흐름 (변경 후)
```
PC Nav2:
  controller_server ──▶ /robot1/cmd_vel_nav ──┐
  behavior_server ────▶ /robot1/cmd_vel_nav ──┼──▶ safety_node ──▶ /robot1/cmd_vel ──▶ odom_publisher
                                              │         │
                                              │    TTC 검사
                                              │         │
Robot 센서:                                   │         │
  scan_raw, odom, imu, image ──▶ sync_node ──▶ synced_data ──┘

Robot 수동제어 (변경 없음):
  Web/조이스틱 ──▶ /robot1/controller/cmd_vel ──▶ odom_publisher
```

---

## 6. 잠재적 문제 및 대응 방안

### 6.1 이식 과정 문제

| 문제 | 증상 | 확인 방법 | 해결 방법 |
|------|------|----------|----------|
| msgs 빌드 실패 | colcon build 에러 | 에러 로그 확인 | 의존성 설치 |
| sync_node 센서 미수신 | synced_data 미발행 | `ros2 topic hz` | params.yaml 토픽명 확인 |
| safety_node cmd_vel_nav 미수신 | cmd_vel 미발행 | `ros2 topic echo` | Nav2 remapping 확인 |
| 로봇 움직임 없음 | 모터 무반응 | `ros2 topic echo /robot1/cmd_vel` | 전체 파이프라인 점검 |

### 6.2 운영 중 문제

| 문제 | 증상 | 자동 대응 | 수동 대응 |
|------|------|----------|----------|
| safety_node 크래시 | cmd_vel 중단 | 로봇 정지 (Fail-Safe) | 노드 재시작 |
| sync_node 크래시 | synced_data 중단 | 로봇 정지 | 노드 재시작 |
| 네트워크 단절 | cmd_vel_nav 미수신 | 0.5초 후 정지 | 네트워크 복구 |
| LiDAR 장애 | scan_raw 중단 | TTC 계산 불가 → 정지 | 센서 점검 |

---

## 7. 롤백 절차

### 7.1 긴급 롤백 (Emergency Stop 비활성화)

```bash
# Robot: Emergency Stop 노드 중지
# (bringup은 유지, avoid.launch.py만 중지)
Ctrl+C (avoid.launch.py 터미널)

# PC: Nav2 remapping 원복 필요
# 아래 7.2 참조
```

### 7.2 완전 롤백 (원상복구)

**Step 1: PC navigation_pc.launch.py 원복**
```python
# Line 116
remappings=costmap_remappings + [('cmd_vel', 'cmd_vel')],
```

**Step 2: PC nav2_params_ackermann.yaml 원복**
```yaml
# Line 289-290
use_collision_detection: true
max_allowed_time_to_collision_up_to_carrot: 1.0
```

**Step 3: Robot Emergency Stop 패키지 제거 (선택)**
```bash
ssh pi@172.16.10.172
docker exec -it MentorPi bash
cd /home/ubuntu/ros2_ws/src
rm -rf mlm_avoid_msgs mlm_avoid_sync mlm_avoid_safety mlm_avoid_bringup
cd /home/ubuntu/ros2_ws
colcon build
```

**Step 4: Nav2 재시작**
```bash
# PC
ros2 launch navigation navigation_pc.launch.py map:=mlm_slam.yaml robot_name:=robot1
```

---

## 8. 검증 체크리스트

### 8.1 빌드 검증
- [x] mlm_avoid_msgs 빌드 성공
- [x] mlm_avoid_sync 빌드 성공
- [x] mlm_avoid_safety 빌드 성공
- [x] mlm_avoid_bringup 빌드 성공

### 8.2 토픽 검증
- [ ] /robot1/cmd_vel_nav 발행 확인 (Nav2)
- [x] /mlm_avoid/synced_data 발행 확인 (sync_node) ✓ "First sync successful!"
- [ ] /robot1/cmd_vel 발행 확인 (safety_node)
- [ ] odom_publisher가 /robot1/cmd_vel 수신 확인

### 8.3 기능 검증
- [ ] 정상 주행: Nav2 목표점 도달
- [ ] 긴급정지: 장애물 앞 정지
- [ ] 복구: 장애물 제거 후 주행 재개
- [ ] 수동제어: Web Dashboard 조이스틱 동작

### 8.4 bringup 통합 검증
- [x] mlm_bringup.launch.py에 avoid_launch 추가
- [x] DOG 로봇 컨테이너에 파일 전송
- [x] bringup.sh 실행 시 Emergency Stop 노드 자동 시작 확인 ✓
  - sync_node 시작: "Sync node initialized (buffer_size=30)"
  - safety_node 시작: "Safety node initialized (TTC thresholds: start=2.0s, end=3.0s)"
  - synced_data 발행: "First sync successful!"

### 8.5 카메라 드라이버 검증
- [x] usb_cam_param.yaml 수정 (av_device_format 제거)
- [x] usb_cam_node 정상 시작 (크래시 해결)
- [x] 이미지 발행 확인 (Timer triggering every 33 ms)

---

## 9. 작업 로그

| 시간 | 작업 | 결과 | 비고 |
|------|------|------|------|
| 2026-01-27 | Step 1: 문서 작성 | 완료 | 초기 문서 작성 |
| 2026-01-27 | Step 2: GT 패키지 복사 | 완료 | /home/lee/mlm_ws/emergency_stop_transplant/src/ |
| 2026-01-27 | Step 3: 코드 수정 | 완료 | sync_node, safety_node, avoid_params 수정 |
| 2026-01-27 | Step 4: DOG 전송 및 빌드 | 완료 | 4개 패키지 빌드 성공 |
| 2026-01-27 | Step 5: PC Nav2 수정 | 완료 | cmd_vel remapping, TTC 비활성화 |
| 2026-01-27 | Step 5.1: DOG 문제 해결 | 완료 | mentorpi_description 복원, image_topic 수정 |
| 2026-01-27 | Step 5.2: bringup 통합 | 완료 | mlm_bringup.launch.py에 avoid_launch 추가 |
| 2026-01-27 | Step 5.3: CycloneDDS IP 변경 | 완료 | PC IP 172.16.11.203 → 172.16.11.222 |
| 2026-01-27 | Step 5.4: package.xml 수정 | 완료 | path_player_pkg 의존성 제거 |
| 2026-01-27 | Step 5.5: usb_cam 수정 | 완료 | av_device_format 제거로 카메라 크래시 해결 |
| 2026-01-27 | Step 6: bringup 통합 테스트 | 완료 | sync_node, safety_node 정상 시작 확인 |
| - | Step 7: Nav2 통합 테스트 | 대기 중 | PC Nav2 실행 후 cmd_vel_nav 수신 테스트 필요 |
| - | Step 8: 기능 검증 | 대기 중 | 정상 주행, 긴급정지, 복구, 수동제어 테스트 |

---

## 10. 변경 이력

| 날짜 | 버전 | 변경 내용 |
|------|------|----------|
| 2026-01-27 | 1.0 | 초기 문서 작성 |
| 2026-01-27 | 1.1 | DOG 문제 해결 (mentorpi_description, image_topic) |
| 2026-01-27 | 1.2 | bringup 통합, CycloneDDS IP 변경, package.xml 수정 |
| 2026-01-27 | 1.3 | usb_cam 크래시 해결, bringup 통합 테스트 완료 |

