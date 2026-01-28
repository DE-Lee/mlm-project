# Multi-Robot Learning System for MentorPi

MentorPi 로봇 기반 멀티로봇 학습 시스템 - Robot, PC Navigation, Web Dashboard 통합 프로젝트

## 시스템 구성

```
┌─────────────────┐                 ┌─────────────────┐                 ┌─────────────┐
│ Robot1          │  CycloneDDS     │       PC        │   rosbridge     │    Web      │
│ (172.16.10.172) │◄──Peer-to-Peer─►│ (172.16.11.222) │◄───WebSocket───►│  Dashboard  │
├─────────────────┤    Unicast      │                 │                 │             │
│ Robot2          │◄──Peer-to-Peer─►│                 │                 │             │
│ (172.16.10.37)  │                 └─────────────────┘                 └─────────────┘
└─────────────────┘                   • Nav2 Navigation                  • 모니터링/제어
  • 센서/액추에이터                     • AMCL Localization               • 목표점 설정
  • Odometry/EKF                      • rosbridge 서버                   • 멀티로봇 지원
  • LiDAR (LD19)                      • Multi-robot 지원
```

## 프로젝트 구조

```
mlm_ws/
├── robot/          # Robot1 제어 패키지 (172.16.10.172)
├── robot2/         # Robot2 제어 패키지 (172.16.10.37)
├── pc/             # Navigation 패키지 (PC - 172.16.11.222)
└── web/            # 웹 대시보드 (React + TypeScript)
```

## 핵심 설정 (반드시 확인)

### 네트워크 설정

| 항목 | 값 | 설명 |
|------|-----|------|
| Robot1 IP | `172.16.10.172` | MentorPi #1 |
| Robot2 IP | `172.16.10.37` | MentorPi #2 |
| PC IP | `172.16.11.222` | PC (Navigation 서버) |
| WiFi | 동일 네트워크 | Robot, PC, Web 모두 같은 WiFi |
| ROS_DOMAIN_ID | `3` | 모든 기기 동일하게 설정 |

### DDS 통신 방식

- **CycloneDDS** Peer-to-Peer Unicast
- Robot → PC 직접 통신 (멀티캐스트 최소화)
- WiFi 환경 최적화

### 토픽 통일

| 토픽 | 타입 | 발행자 | 구독자 |
|------|------|--------|--------|
| `/{robot_name}/scan_raw` | sensor_msgs/LaserScan | Robot LiDAR | PC AMCL, Web |
| `/{robot_name}/odom` | nav_msgs/Odometry | Robot EKF | PC Nav2, Web |
| `/{robot_name}/cmd_vel` | geometry_msgs/Twist | PC Nav2, Web | Robot Controller |
| `/{robot_name}/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | PC AMCL | Web |
| `/{robot_name}/goal_pose` | geometry_msgs/PoseStamped | Web | PC BT Navigator |

### rosbridge 서버

- **PC에서만 실행** (로봇 자원 절약)
- Web Dashboard가 PC의 rosbridge에 연결
- 포트: `9090` (WebSocket)

## 빠른 시작

### 1. Robot1 설정

```bash
# Robot1 (172.16.10.172) - Raspberry Pi 5 Docker 내부
cd /home/ubuntu/ros2_ws
colcon build --symlink-install
source install/setup.zsh

# CycloneDDS 설정 확인 (PC IP가 올바른지 확인)
cat ~/cyclonedds/cyclonedds.xml

# Robot1 실행
ros2 launch bringup bringup_ns.launch.py robot_name:=robot1
```

### 2. Robot2 설정

```bash
# Robot2 (172.16.10.37) - Raspberry Pi 5 Docker 내부
cd /home/ubuntu/ros2_ws
colcon build --symlink-install
source install/setup.zsh

# CycloneDDS 설정 확인 (PC IP가 올바른지 확인)
cat ~/cyclonedds/cyclonedds.xml

# Robot2 실행
ros2 launch bringup bringup_ns.launch.py robot_name:=robot2
```

### 3. PC 설정 (Multi-Robot Navigation)

```bash
# PC (Ubuntu 22.04 - 172.16.11.222)
cd ~/mlm_ws/pc/ros2_ws
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=3

# rosbridge 서버 실행 (필수! - 별도 터미널)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Robot1 Navigation 실행 (별도 터미널)
ros2 launch navigation navigation_pc.launch.py map:=mlm_slam robot_name:=robot1

# Robot2 Navigation 실행 (별도 터미널)
ros2 launch navigation navigation_pc.launch.py map:=mlm_slam robot_name:=robot2

# 맵 PNG 변환 (웹 대시보드용, 최초 1회)
cd ~/mlm_ws/pc
./convert_map_to_png.sh mlm_slam
```

### 4. Web 설정

```bash
# Web Dashboard
cd ~/mlm_ws/web

# .env 파일 생성 및 PC IP 설정
cp .env.example .env
# VITE_ROS_BRIDGE_URL=ws://172.16.11.222:9090 확인

# 의존성 설치
npm install

# 개발 서버 실행
npm run dev
```

## Multi-Robot 운영 가이드

### 전체 시스템 실행 순서

1. **Robot1 & Robot2 실행** (각 로봇에서)
   ```bash
   # Robot1
   ros2 launch bringup bringup_ns.launch.py robot_name:=robot1

   # Robot2
   ros2 launch bringup bringup_ns.launch.py robot_name:=robot2
   ```

2. **PC Navigation 실행** (PC에서 3개 터미널)
   ```bash
   # 터미널 1: rosbridge
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml

   # 터미널 2: Robot1 Navigation
   ros2 launch navigation navigation_pc.launch.py map:=mlm_slam robot_name:=robot1

   # 터미널 3: Robot2 Navigation
   ros2 launch navigation navigation_pc.launch.py map:=mlm_slam robot_name:=robot2
   ```

3. **Web Dashboard 실행**
   ```bash
   cd ~/mlm_ws/web && npm run dev
   ```

### Multi-Robot 토픽 구조

```
/robot1/scan_raw        # Robot1 LiDAR
/robot1/odom            # Robot1 Odometry
/robot1/cmd_vel         # Robot1 속도 명령
/robot1/amcl_pose       # Robot1 AMCL 위치
/robot1/goal_pose       # Robot1 목표점

/robot2/scan_raw        # Robot2 LiDAR
/robot2/odom            # Robot2 Odometry
/robot2/cmd_vel         # Robot2 속도 명령
/robot2/amcl_pose       # Robot2 AMCL 위치
/robot2/goal_pose       # Robot2 목표점

/tf, /tf_static         # Global TF (모든 로봇 공유)
```

### Multi-Robot 확인 명령어

```bash
# 모든 토픽 확인
ros2 topic list | grep -E "robot[12]"

# TF 프레임 확인
ros2 run tf2_tools view_frames

# 각 로봇 위치 확인
ros2 topic echo /robot1/amcl_pose
ros2 topic echo /robot2/amcl_pose

# 통신 상태 확인
ros2 node list
```

## 주요 변경사항 (정합성 수정)

### ✅ 수정 완료

1. **rosbridge 실행 위치**
   - Robot: rosbridge 제거 (주석 처리)
   - PC: rosbridge 실행
   - Web: PC IP로 연결

2. **토픽 통일**
   - Robot LiDAR: `scan_raw` 발행
   - PC AMCL: `scan_raw` 구독
   - Web: `scan_raw` 구독

3. **DDS 설정**
   - Robot: Peer-to-Peer Unicast 설정 추가
   - PC: 기존 Peer 설정 유지

4. **맵 포맷**
   - PC: PGM 원본 유지
   - Web: PNG 변환 스크립트 추가

## 문제 해결

### Robot과 PC 통신 안 됨

```bash
# 1. 네트워크 인터페이스 확인
ip addr show | grep -E "^[0-9]+:" | awk -F: '{print $2}'

# 2. CycloneDDS 설정 수정
# robot/cyclonedds/cyclonedds.xml에서 <NetworkInterface name="wlan0"/> 확인
# pc/cyclonedds/cyclonedds.xml에서 <NetworkInterface name="wlp0s20f3"/> 확인

# 3. Ping 테스트
ping 172.16.11.222  # Robot에서 PC로
ping 172.16.10.172  # PC에서 Robot1으로
ping 172.16.10.37   # PC에서 Robot2로

# 4. ROS_DOMAIN_ID 확인
echo $ROS_DOMAIN_ID  # Robot과 PC 모두 3이어야 함
```

### Web Dashboard 연결 안 됨

```bash
# 1. PC에서 rosbridge 실행 중인지 확인
ps aux | grep rosbridge

# 2. rosbridge 포트 확인
ss -tuln | grep 9090

# 3. .env 파일 확인
cat ~/mlm_ws/web/.env
# VITE_ROS_BRIDGE_URL=ws://172.16.11.222:9090 확인

# 4. 브라우저 콘솔 확인 (F12)
```

### LiDAR 토픽이 안 보임

```bash
# 1. Robot에서 토픽 확인
ros2 topic list | grep scan

# 예상 출력: /robot1/scan_raw

# 2. PC에서 토픽 확인 (CycloneDDS 통신 확인)
ros2 topic list | grep scan

# 3. 토픽 echo 테스트
ros2 topic echo /robot1/scan_raw
```

## 상세 문서

- [Robot 패키지 상세](robot/README.md)
- [PC Navigation 상세](pc/README.md)
- [Web Dashboard 상세](web/README.md)

## 라이센스

MIT License
