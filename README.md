# MLM Multi-Robot Navigation System

MentorPi 멀티로봇 Navigation 시스템 - ROS2 Humble 기반 웹 제어 대시보드

## 시스템 구성

```
┌─────────────────────────────────────────────────────────────────┐
│  [Robot - Raspberry Pi 5]       [PC - Ubuntu 22.04]            │
│  ├─ bringup (센서 드라이버)     ├─ Navigation (Nav2)           │
│  ├─ controller (모터 제어)      ├─ SLAM                        │
│  ├─ peripherals (LiDAR/카메라)  └─ RViz2                       │
│  └─ rosbridge_websocket                                        │
│           │                            │                       │
│           └────── DDS (DOMAIN_ID=3) ───┘                       │
│                          │                                     │
│                          ↓                                     │
│              [Web Dashboard - React]                           │
│              └─ roslib.js (WebSocket:9090)                     │
└─────────────────────────────────────────────────────────────────┘
```

## 폴더 구조

```
mlm-project/
├── robot/           # 로봇용 패키지 (Raspberry Pi + Docker)
│   ├── ros2_ws/src/
│   │   ├── bringup/
│   │   ├── driver/
│   │   └── peripherals/
│   ├── third_party_ws/src/
│   │   ├── ldlidar_stl_ros2/
│   │   └── laser_filters/
│   └── cyclonedds/
├── pc/              # PC용 패키지 (Ubuntu 22.04)
│   ├── ros2_ws/src/
│   │   ├── navigation/
│   │   └── slam/
│   └── cyclonedds/
├── web/             # 웹 대시보드 (React + TypeScript)
│   ├── src/
│   └── public/
└── docs/            # 문서
```

## 빠른 시작

### 1. 로봇 설정
```bash
cd robot
./setup.sh
```

### 2. PC 설정
```bash
cd pc
./setup.sh
```

### 3. 웹 대시보드
```bash
cd web
npm install
npm run dev
```

## 실행 순서

### Step 1: 로봇 Bringup
```bash
# SSH 접속 → Docker 진입
ssh pi@<ROBOT_IP>
docker exec -it MentorPi bash

# 실행
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=3
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch bringup bringup_ns.launch.py robot_name:=robot1
```

### Step 2: PC Navigation
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=3
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds/cyclonedds.xml

ros2 launch navigation navigation_pc.launch.py map:=mlm_map.yaml robot_name:=robot1
```

### Step 3: 웹 대시보드
```bash
cd web
npm run dev
# http://localhost:3000 접속
```

## 환경 설정

### 네트워크 설정 변경 시
1. `pc/cyclonedds/cyclonedds.xml` - PC/로봇 IP 수정
2. `web/.env` - `VITE_ROS_BRIDGE_URL` 수정

### 로봇 추가 시
1. `web/src/config/robots.ts` - ROBOTS 배열에 추가
2. 각 로봇에서 다른 `robot_name` 인자로 bringup 실행

## 라이선스

MIT License
