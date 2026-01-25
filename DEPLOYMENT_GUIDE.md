# Multi-Robot System - 배포 및 실행 가이드

**최종 업데이트**: 2026-01-25
**시스템 버전**: v2.1 (Critical Fixes Applied - Production Ready)

---

## 📋 목차

1. [시스템 개요](#시스템-개요)
2. [사전 요구사항](#사전-요구사항)
3. [초기 설정](#초기-설정)
4. [실행 순서](#실행-순서)
5. [동작 확인](#동작-확인)
6. [문제 해결](#문제-해결)
7. [고급 기능](#고급-기능)

---

## 시스템 개요

### 아키텍처

```
┌─────────────────┐   DDS Domain 3    ┌─────────────────┐   WebSocket     ┌──────────────┐
│  Robot1 (Pi5)   │◄─────────────────►│   PC (Ubuntu)   │◄───────────────►│ Web Browser  │
│ 172.16.10.172   │  Peer-to-Peer     │ 172.16.11.167   │   rosbridge     │              │
│ namespace:      │                   │                 │                 │ • Dashboard  │
│   robot1        │                   │ • Nav2 (robot1) │                 │ • Control    │
└─────────────────┘                   │ • Nav2 (robot2) │                 │ • Monitoring │
                                      │ • rosbridge     │                 └──────────────┘
┌─────────────────┐   DDS Domain 3    │ • Map Server    │
│  Robot2 (Pi5)   │◄─────────────────►│                 │
│ 172.16.10.173   │  Peer-to-Peer     │                 │
│ namespace:      │                   │                 │
│   robot2        │                   │                 │
└─────────────────┘                   └─────────────────┘
```

### 주요 개선 사항 (v2.1)

#### v2.0 기능
✅ **Action-based Navigation**: `navigate_to_pose` action 사용
✅ **Control Mode Management**: AUTONOMOUS / MANUAL / IDLE 모드 분리
✅ **Proper Cancel**: Navigation 취소 기능 완벽 구현
✅ **Emergency Stop**: 모든 로봇 즉시 정지 + Navigation 취소
✅ **Domain ID 통일**: 모든 노드 Domain 3 사용
✅ **환경 변수 설정**: .env 파일 자동 생성

#### v2.1 Critical Fixes
🔧 **FIXED**: Robot2 scan topic - namespace 자동 remapping 적용
🔧 **FIXED**: Action server 준비 상태 체크 - UI 버튼 비활성화
🔧 **FIXED**: ROS_DOMAIN_ID 자동 검증 - 잘못된 domain 사용 방지
🔧 **FIXED**: Bond timeout 30초 확장 - cold boot 안정성 향상
🔧 **REMOVED**: Dead code (useNavigateAction, useGoalPublisher)

---

## 사전 요구사항

### 하드웨어
- **Robot1**: Raspberry Pi 5, WiFi 연결 (172.16.10.172)
- **Robot2**: Raspberry Pi 5, WiFi 연결 (172.16.10.173)
- **PC**: Ubuntu 22.04, WiFi 연결 (172.16.11.167)
- **Web Client**: 최신 Chrome/Firefox 브라우저

### 소프트웨어
- **Robots**: ROS2 Humble + Docker
- **PC**: ROS2 Humble + Nav2
- **Web**: Node.js 20.x

### 네트워크
- 모든 장치가 **동일한 WiFi 네트워크**에 연결
- IP 주소가 **고정**되어 있어야 함
- 방화벽이 **9090 포트(rosbridge)** 허용

---

## 초기 설정

### 1. Robot1 설정 (Pi5)

```bash
# 1.1 CycloneDDS 설정 복사
cd /path/to/workspaces/robot
mkdir -p ~/cyclonedds
cp cyclonedds/cyclonedds.xml ~/cyclonedds/

# 1.2 환경 변수 설정 (영구)
echo 'export ROS_DOMAIN_ID=3' >> ~/.bashrc
echo 'export CYCLONEDDS_URI=file://$HOME/cyclonedds/cyclonedds.xml' >> ~/.bashrc
source ~/.bashrc

# 1.3 ROS2 워크스페이스 빌드
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash  # 또는 setup.zsh

# 1.4 확인
echo $ROS_DOMAIN_ID  # 출력: 3
```

### 2. Robot2 설정 (Pi5)

```bash
# 2.1 CycloneDDS 설정 복사
cd /path/to/workspaces/robot2
mkdir -p ~/cyclonedds
cp cyclonedds/cyclonedds.xml ~/cyclonedds/

# 2.2 환경 변수 설정 (영구)
echo 'export ROS_DOMAIN_ID=3' >> ~/.bashrc
echo 'export CYCLONEDDS_URI=file://$HOME/cyclonedds/cyclonedds.xml' >> ~/.bashrc
source ~/.bashrc

# 2.3 ROS2 워크스페이스 빌드
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 2.4 확인
echo $ROS_DOMAIN_ID  # 출력: 3
```

### 3. PC 설정 (Ubuntu 22.04)

```bash
# 3.1 CycloneDDS 설정 복사
cd /path/to/workspaces/pc
mkdir -p ~/cyclonedds
cp cyclonedds/cyclonedds.xml ~/cyclonedds/

# 3.2 환경 변수 설정 (영구)
echo 'export ROS_DOMAIN_ID=3' >> ~/.bashrc
echo 'export CYCLONEDDS_URI=file://$HOME/cyclonedds/cyclonedds.xml' >> ~/.bashrc
source ~/.bashrc

# 3.3 ROS2 워크스페이스 빌드
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 3.4 rosbridge 설치 (없으면)
sudo apt install ros-humble-rosbridge-suite

# 3.5 확인
echo $ROS_DOMAIN_ID  # 출력: 3
```

### 4. Web 설정

```bash
cd /path/to/workspaces/web

# 4.1 의존성 설치
npm install

# 4.2 .env 파일 확인 (이미 생성됨)
cat .env
# VITE_ROS_BRIDGE_URL=ws://172.16.11.167:9090
# VITE_USE_GAZEBO=false

# 4.3 개발 서버 빌드 (프로덕션용, 선택사항)
npm run build
```

---

## 실행 순서

### 🚀 표준 시작 절차 (권장)

#### 1단계: PC - rosbridge 실행 (필수, 가장 먼저!)

```bash
# 터미널 1
export ROS_DOMAIN_ID=3
export CYCLONEDDS_URI=file://$HOME/cyclonedds/cyclonedds.xml
source ~/ros2_ws/install/setup.bash

ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**대기**: `[INFO] Rosbridge WebSocket server started on port 9090` 확인

---

#### 2단계: PC - Navigation 실행 (Robot1)

```bash
# 터미널 2
export ROS_DOMAIN_ID=3
export CYCLONEDDS_URI=file://$HOME/cyclonedds/cyclonedds.xml
source ~/ros2_ws/install/setup.bash

ros2 launch navigation navigation_pc.launch.py \
  robot_name:=robot1 \
  map:=mlm_map.yaml
```

**대기**: `[lifecycle_manager]: Activating map_server` 확인

---

#### 3단계: PC - Navigation 실행 (Robot2)

```bash
# 터미널 3
export ROS_DOMAIN_ID=3
export CYCLONEDDS_URI=file://$HOME/cyclonedds/cyclonedds.xml
source ~/ros2_ws/install/setup.bash

ros2 launch navigation navigation_pc.launch.py \
  robot_name:=robot2 \
  map:=mlm_map.yaml
```

**대기**: `[lifecycle_manager]: Activating map_server` 확인

---

#### 4단계: Robot1 - 로봇 제어 실행

```bash
# Robot1 Pi5
export ROS_DOMAIN_ID=3
export CYCLONEDDS_URI=file://$HOME/cyclonedds/cyclonedds.xml
source ~/ros2_ws/install/setup.bash

ros2 launch bringup bringup_ns.launch.py robot_name:=robot1
```

**대기**: LiDAR 데이터 확인 - `ros2 topic hz /robot1/scan_raw`

---

#### 5단계: Robot2 - 로봇 제어 실행

```bash
# Robot2 Pi5
export ROS_DOMAIN_ID=3
export CYCLONEDDS_URI=file://$HOME/cyclonedds/cyclonedds.xml
source ~/ros2_ws/install/setup.bash

ros2 launch bringup bringup_ns.launch.py robot_name:=robot2
```

**대기**: LiDAR 데이터 확인 - `ros2 topic hz /robot2/scan_raw`

---

#### 6단계: Web - Dashboard 실행

```bash
cd /path/to/workspaces/web
npm run dev
```

**브라우저 열기**: http://localhost:3000

---

## 동작 확인

### ✅ 체크리스트

#### 1. 네트워크 연결 확인

```bash
# PC에서 실행
ping 172.16.10.172  # Robot1
ping 172.16.10.173  # Robot2

# Robot1에서 실행
ping 172.16.11.167  # PC

# Robot2에서 실행
ping 172.16.11.167  # PC
```

#### 2. ROS 토픽 통신 확인

```bash
# PC에서 실행
export ROS_DOMAIN_ID=3
source ~/ros2_ws/install/setup.bash

# Robot1 토픽 확인
ros2 topic list | grep robot1
# /robot1/scan_raw
# /robot1/odom
# /robot1/cmd_vel
# /robot1/amcl_pose

# Robot2 토픽 확인
ros2 topic list | grep robot2
# /robot2/scan_raw
# /robot2/odom
# /robot2/cmd_vel
# /robot2/amcl_pose

# 토픽 데이터 흐름 확인
ros2 topic hz /robot1/scan_raw  # ~20 Hz
ros2 topic hz /robot1/odom      # ~50 Hz
ros2 topic hz /robot2/scan_raw  # ~20 Hz
ros2 topic hz /robot2/odom      # ~50 Hz
```

#### 3. Web Dashboard 연결 확인

1. **브라우저 열기**: http://localhost:3000
2. **연결 상태**: 우측 상단 "Online" 확인 (초록색)
3. **로봇 표시**: MentorPi #1, MentorPi #2 패널 확인
4. **토픽 수신**: 각 로봇의 위치/속도 데이터 업데이트 확인

#### 4. 기능 테스트

**4.1 초기 위치 설정**
1. Robot1 패널 열기
2. "초기 위치 설정" 버튼 클릭
3. 맵에서 로봇의 실제 위치 클릭+드래그
4. AMCL 위치 업데이트 확인

**4.2 목표점 설정 (Action Navigation)**
1. Robot1 패널에서 "목표점 설정" 클릭
2. 맵에서 목표 위치 클릭+드래그
3. 콘솔 확인: `[robot1] Navigate to: (x, y, θ)`
4. 로봇 이동 시작 확인
5. Mode 표시: `Mode: AUTONOMOUS`

**4.3 Cancel Navigation**
1. 로봇 이동 중 "🛑 Cancel Navigation" 버튼 클릭
2. 로봇 즉시 정지 확인
3. Mode 변경: `Mode: IDLE`

**4.4 Manual Control**
1. "⌨️ 비활성화" 버튼 클릭 → `⌨️ MANUAL`
2. 자동으로 Navigation 취소됨
3. WASD 또는 방향 버튼으로 조작
4. Mode 표시: `Mode: MANUAL`

**4.5 Emergency Stop**
1. 두 로봇 모두 주행 중
2. 상단 "🛑 STOP ALL" 버튼 클릭 또는 ESC 키
3. 모든 로봇 즉시 정지 + Navigation 취소
4. 모든 로봇 Mode: `IDLE`

---

## 문제 해결

### 🔴 문제: Web에서 "Offline" 표시

**원인**: rosbridge 연결 실패

**해결**:
```bash
# 1. PC에서 rosbridge 실행 확인
ps aux | grep rosbridge

# 2. 포트 확인
netstat -tuln | grep 9090

# 3. 방화벽 확인
sudo ufw status
sudo ufw allow 9090/tcp

# 4. .env 파일 확인
cat /workspaces/web/.env
# VITE_ROS_BRIDGE_URL=ws://172.16.11.167:9090

# 5. 브라우저 콘솔 확인 (F12)
# WebSocket connection error 확인
```

---

### 🔴 문제: Robot2 토픽이 안 보임

**원인**: Domain ID 불일치

**해결**:
```bash
# Robot2에서 확인
echo $ROS_DOMAIN_ID  # 3이어야 함

# PC에서 확인
export ROS_DOMAIN_ID=3
ros2 topic list | grep robot2

# 안 보이면 Robot2 재시작
# (Pi5에서) Ctrl+C → 다시 launch
```

---

### 🔴 문제: 목표점 설정해도 로봇이 안 움직임

**가능한 원인**:

**1. Navigation이 실행되지 않음**
```bash
# PC에서 확인
ros2 node list | grep robot1
# /robot1/bt_navigator
# /robot1/controller_server
# /robot1/planner_server
# ...

# 없으면 터미널 2 확인 후 재실행
```

**2. Action Server가 준비되지 않음**
```bash
# PC에서 확인
ros2 action list | grep robot1
# /robot1/navigate_to_pose

# 브라우저 콘솔 확인
# "Action client not available" 또는
# "Action server not available"
```

**3. 로봇이 cmd_vel을 받지 못함**
```bash
# PC에서 확인
ros2 topic echo /robot1/cmd_vel
# (이동 중이면 속도 값이 출력되어야 함)

# Robot1에서 확인
ros2 topic hz /robot1/cmd_vel
```

---

### 🟡 문제: Manual Control이 작동 안 함

**확인**:
1. "⌨️ MANUAL" 활성화 되었는지 확인
2. Mode가 `MANUAL`인지 확인
3. AUTONOMOUS 모드면 자동으로 Navigation이 우선됨
   → Manual 활성화 시 자동 취소됨

---

### 🟡 문제: 두 로봇이 동시에 움직임

**원인**: Namespace 분리 실패

**확인**:
```bash
# 토픽 확인
ros2 topic list | grep cmd_vel
# /robot1/cmd_vel
# /robot2/cmd_vel
# (분리되어 있어야 함)

# 같은 /cmd_vel을 쓰고 있다면 launch 재확인
```

---

## 고급 기능

### 🔧 로봇 추가 (Robot3)

```bash
# 1. robot3 디렉토리 생성
cp -r /workspaces/robot /workspaces/robot3

# 2. robots.ts 수정
# /workspaces/web/src/config/robots.ts
export const ROBOTS: RobotConfig[] = [
  { namespace: 'robot1', name: 'MentorPi #1', color: '#22c55e' },
  { namespace: 'robot2', name: 'MentorPi #2', color: '#3b82f6' },
  { namespace: 'robot3', name: 'MentorPi #3', color: '#f59e0b' },  // 추가
];

# 3. Robot3 실행
export ROS_DOMAIN_ID=3
ros2 launch bringup bringup_ns.launch.py robot_name:=robot3

# 4. PC에서 Navigation 추가 (터미널 4)
ros2 launch navigation navigation_pc.launch.py robot_name:=robot3

# 5. Web Dashboard 새로고침 → Robot3 자동 표시
```

---

### 🎯 Fleet Management (고급)

여러 로봇을 중앙에서 관리:

```python
# fleet_manager.py
from nav2_simple_commander import BasicNavigator

navigators = {
    'robot1': BasicNavigator(namespace='robot1'),
    'robot2': BasicNavigator(namespace='robot2'),
}

# 동시에 서로 다른 목표 설정
navigators['robot1'].goToPose(goal_pose_1)
navigators['robot2'].goToPose(goal_pose_2)

# 모두 완료될 때까지 대기
while not all(nav.isTaskComplete() for nav in navigators.values()):
    time.sleep(0.1)
```

---

## 시스템 종료

**권장 종료 순서** (시작의 역순):

1. Web Dashboard: Ctrl+C
2. Robot2: Ctrl+C
3. Robot1: Ctrl+C
4. PC Navigation (robot2): Ctrl+C
5. PC Navigation (robot1): Ctrl+C
6. PC rosbridge: Ctrl+C

---

## 지원 및 문의

- **이슈 보고**: GitHub Issues
- **문서**: `/workspaces/README.md`
- **전문가 점검 보고서**: `/tmp/mlm_system_audit_report.md`

---

**Happy Multi-Robot Navigation! 🤖🤖**
