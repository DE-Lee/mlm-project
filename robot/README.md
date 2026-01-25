# Robot Packages

MentorPi 로봇용 ROS2 패키지 (Raspberry Pi 5 + Docker)

## 폴더 구조

```
robot/
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
    └── cyclonedds.xml        # DDS 설정
```

## 핵심 설정 확인 ⚠️

### 1. CycloneDDS 설정 (Peer-to-Peer Unicast)

`cyclonedds/cyclonedds.xml` 파일에서 다음을 확인:

```xml
<NetworkInterface name="wlan0"/>  <!-- 로봇 네트워크 인터페이스 (유선이면 eth0) -->
<Peer address="172.16.11.167"/>   <!-- PC IP 주소 -->
```

네트워크 인터페이스 확인:
```bash
ip addr show | grep -E "^[0-9]+:" | awk -F: '{print $2}'
```

CycloneDDS 설정을 Docker 컨테이너로 복사:
```bash
mkdir -p ~/cyclonedds
cp cyclonedds/cyclonedds.xml ~/cyclonedds/
```

### 2. 토픽 이름 통일

- LiDAR 토픽: **`scan_raw`** (PC Navigation과 Web 통일)
- 기본값이 `scan_raw`로 설정되어 있음 (`peripherals/launch/lidar.launch.py`)

### 3. rosbridge 서버

- **로봇에서는 rosbridge를 실행하지 않음** (PC에서 실행)
- `mlm_bringup.launch.py`에서 rosbridge 부분이 주석 처리됨
- 로봇 자원 절약 목적

## 설치

### 1. Docker 컨테이너에 파일 복사

```bash
# PC에서 로봇으로 전송
scp -r ros2_ws third_party_ws pi@<ROBOT_IP>:/tmp/

# 로봇에서 Docker로 복사
ssh pi@<ROBOT_IP>
docker cp /tmp/ros2_ws MentorPi:/home/ubuntu/
docker cp /tmp/third_party_ws MentorPi:/home/ubuntu/
```

### 2. Docker 내부에서 빌드

```bash
docker exec -it MentorPi bash

# 메인 워크스페이스 빌드
cd /home/ubuntu/ros2_ws
colcon build --symlink-install

# 서드파티 워크스페이스 빌드
cd /home/ubuntu/third_party_ws
colcon build --symlink-install
```

### 3. CycloneDDS 설정 복사

```bash
# Docker 컨테이너 내부에서
mkdir -p ~/cyclonedds
cp /home/ubuntu/cyclonedds/cyclonedds.xml ~/cyclonedds/
```

### 4. 환경 변수 설정 (~/.zshrc)

```bash
source /opt/ros/humble/setup.zsh
source /home/ubuntu/ros2_ws/install/setup.zsh
source /home/ubuntu/third_party_ws/install/setup.zsh
export ROS_DOMAIN_ID=3
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds/cyclonedds.xml
```

## 실행

```bash
ros2 launch bringup bringup_ns.launch.py robot_name:=robot1
```

## 필수 apt 패키지

```bash
sudo apt install \
  ros-humble-ros-base \
  ros-humble-usb-cam \
  ros-humble-robot-localization \
  ros-humble-imu-filter-madgwick \
  ros-humble-rmw-cyclonedds-cpp
```

**참고**: `ros-humble-rosbridge-suite`는 PC에서만 설치 (로봇에서는 불필요)

## 주요 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/robot1/odom` | nav_msgs/Odometry | 오도메트리 (EKF 융합) |
| `/robot1/scan_raw` | sensor_msgs/LaserScan | LiDAR 스캔 (PC/Web 통일) |
| `/robot1/cmd_vel` | geometry_msgs/Twist | 속도 명령 (구독) |
| `/tf` | tf2_msgs/TFMessage | 좌표 변환 (global) |

## 문제 해결

### PC와 통신이 안 됨

```bash
# 1. CycloneDDS 설정 확인
echo $CYCLONEDDS_URI
cat ~/cyclonedds/cyclonedds.xml

# 2. 네트워크 인터페이스 확인
ip addr show

# 3. PC로 ping 테스트
ping 172.16.11.167

# 4. ROS_DOMAIN_ID 확인
echo $ROS_DOMAIN_ID  # 반드시 3이어야 함
```

### LiDAR 토픽이 발행 안 됨

```bash
# 1. LiDAR 장치 확인
ls -l /dev/ldlidar

# 2. 토픽 확인
ros2 topic list | grep scan
# 예상 출력: /robot1/scan_raw

# 3. 토픽 echo
ros2 topic echo /robot1/scan_raw
```
