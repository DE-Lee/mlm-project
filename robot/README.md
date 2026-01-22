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

### 3. 환경 변수 설정 (~/.zshrc)

```bash
source /opt/ros/humble/setup.zsh
source /home/ubuntu/ros2_ws/install/setup.zsh
source /home/ubuntu/third_party_ws/install/setup.zsh
export ROS_DOMAIN_ID=3
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
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
  ros-humble-rosbridge-suite \
  ros-humble-rmw-cyclonedds-cpp
```

## 주요 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/robot1/odom` | nav_msgs/Odometry | 오도메트리 |
| `/robot1/scan` | sensor_msgs/LaserScan | LiDAR 스캔 |
| `/robot1/cmd_vel` | geometry_msgs/Twist | 속도 명령 (구독) |
| `/tf` | tf2_msgs/TFMessage | 좌표 변환 |
