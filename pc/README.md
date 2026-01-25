# PC Packages

Navigation PC용 ROS2 패키지 (Ubuntu 22.04)

## 폴더 구조

```
pc/
├── ros2_ws/src/
│   ├── navigation/           # Nav2 Navigation
│   │   ├── config/
│   │   │   ├── nav2_params.yaml
│   │   │   └── nav2_params_ackermann.yaml
│   │   ├── launch/
│   │   │   └── navigation_pc.launch.py
│   │   └── rviz/
│   └── slam/                 # SLAM & 맵 관리
│       ├── config/
│       ├── launch/
│       └── maps/
│           ├── mlm_map.pgm
│           └── mlm_map.yaml
├── cyclonedds/
│   └── cyclonedds.xml        # DDS 설정 (Peer-to-Peer Unicast)
└── convert_map_to_png.sh     # 맵 PNG 변환 스크립트 (웹용)
```

## 핵심 설정 확인 ⚠️

### 1. CycloneDDS 설정 (Peer-to-Peer Unicast)

`cyclonedds/cyclonedds.xml` 파일에서 다음을 확인:

```xml
<NetworkInterface name="wlp0s20f3"/>  <!-- PC 네트워크 인터페이스 -->
<Peer address="172.16.10.172"/>       <!-- 로봇 IP -->
<Peer address="172.16.11.167"/>       <!-- PC IP (자신) -->
```

네트워크 인터페이스 확인:
```bash
ip addr show | grep -E "^[0-9]+:" | awk -F: '{print $2}'
```

### 2. 토픽 이름 통일

- **AMCL scan_topic**: `scan_raw` (Robot LiDAR, Web과 통일)
- `navigation_pc.launch.py`에서 `scan_topic: 'scan_raw'` 설정됨

### 3. rosbridge 서버 (필수!)

- **PC에서 rosbridge 실행** (Web Dashboard 연결용)
- 로봇 자원 절약을 위해 PC에서만 실행
- 포트: 9090 (WebSocket)

## 설치

### 1. setup.sh 실행

```bash
chmod +x setup.sh
./setup.sh
```

### 2. 수동 설치

```bash
# ROS2 의존성
sudo apt install \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rosbridge-suite \
  ros-humble-tf-transformations

# 워크스페이스 빌드
cd ros2_ws
colcon build --symlink-install

# CycloneDDS 설정 복사
mkdir -p ~/cyclonedds
cp cyclonedds/cyclonedds.xml ~/cyclonedds/
# cyclonedds.xml에서 PC IP, 로봇 IP, 네트워크 인터페이스 수정
```

### 3. 환경 변수 (~/.bashrc)

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=3
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds/cyclonedds.xml
```

## CycloneDDS 설정

`cyclonedds/cyclonedds.xml` 파일에서 다음을 수정:

```xml
<NetworkInterface name="wlp0s20f3"/>  <!-- PC 네트워크 인터페이스 -->
<Peer address="172.16.10.172"/>       <!-- 로봇 IP -->
<Peer address="172.16.11.167"/>       <!-- PC IP -->
```

네트워크 인터페이스 확인:
```bash
ip addr show | grep -E "^[0-9]+:" | awk -F: '{print $2}'
```

## 실행

### 1. rosbridge 서버 실행 (필수!)

```bash
# 터미널 1: rosbridge 서버 (Web Dashboard 연결용)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 2. Navigation 실행

```bash
# 터미널 2: Navigation 스택
ros2 launch navigation navigation_pc.launch.py map:=mlm_map.yaml robot_name:=robot1

# 또는 RViz만 실행 (별도 터미널)
ros2 launch navigation rviz_navigation.launch.py
```

## 새 맵 생성

```bash
# SLAM 실행
ros2 launch slam slam.launch.py robot_name:=robot1

# 맵 저장
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/slam/maps/new_map

# 맵 PNG 변환 (웹 대시보드용)
./convert_map_to_png.sh new_map
```

## 맵 PNG 변환 (웹 대시보드용)

웹 브라우저는 PGM 포맷을 렌더링할 수 없으므로 PNG로 변환 필요:

```bash
# 기본 맵 변환
./convert_map_to_png.sh mlm_map

# 또는 다른 맵 변환
./convert_map_to_png.sh your_map_name
```

스크립트가 자동으로:
1. PGM → PNG 변환 (ImageMagick 사용)
2. `../web/public/maps/`로 복사

## 문제 해결

### Robot과 통신 안 됨

```bash
# 1. CycloneDDS 설정 확인
echo $CYCLONEDDS_URI
cat ~/cyclonedds/cyclonedds.xml

# 2. Robot으로 ping 테스트
ping 172.16.10.172

# 3. ROS 토픽 확인
ros2 topic list | grep robot1

# 4. ROS_DOMAIN_ID 확인
echo $ROS_DOMAIN_ID  # 반드시 3이어야 함
```

### rosbridge 연결 안 됨

```bash
# 1. rosbridge 실행 확인
ps aux | grep rosbridge

# 2. 포트 확인
netstat -tuln | grep 9090

# 3. rosbridge 재시작
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Navigation이 LiDAR를 못 받음

```bash
# 1. scan_raw 토픽 확인
ros2 topic list | grep scan_raw
# 예상 출력: /robot1/scan_raw

# 2. scan_raw echo
ros2 topic echo /robot1/scan_raw

# 3. AMCL 파라미터 확인
ros2 param get /robot1/amcl scan_topic
# 예상 출력: scan_raw
```
