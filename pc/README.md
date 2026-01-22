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
└── cyclonedds/
    └── cyclonedds.xml        # DDS 설정 (IP 수정 필요)
```

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

```bash
# Navigation 실행
ros2 launch navigation navigation_pc.launch.py map:=mlm_map.yaml robot_name:=robot1

# RViz만 실행 (별도 터미널)
ros2 launch navigation rviz_navigation.launch.py
```

## 새 맵 생성

```bash
# SLAM 실행
ros2 launch slam slam.launch.py robot_name:=robot1

# 맵 저장
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/slam/maps/new_map
```
