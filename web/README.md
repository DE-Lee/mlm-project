# MLM Dashboard

Multi-robot Learning for MentorPi - 웹 기반 로봇 제어 대시보드

## 기능

- 실시간 로봇 위치 시각화 (2D 맵)
- 클릭으로 목표점 설정 → Nav2 자동 주행
- 키보드/버튼으로 수동 조작
- 멀티로봇 지원 (최대 3대)
- 로봇 상태 모니터링

## 요구사항

- Node.js 18+
- ROS2 Humble + rosbridge (PC에서 실행)
- Nav2 Stack (PC)

## 핵심 설정 확인 ⚠️

### 1. rosbridge 연결 (PC)

`.env` 파일에서 **PC IP**로 설정:

```bash
VITE_ROS_BRIDGE_URL=ws://172.16.11.167:9090
```

**중요**: rosbridge는 **PC에서 실행**되어야 함 (Robot 아님)

### 2. 토픽 이름 통일

- LiDAR: **`/scan_raw`** (Robot, PC와 통일)
- `src/config/robots.ts`에서 `scan: '/scan_raw'` 설정됨

### 3. 맵 포맷

- **PNG 포맷** 필요 (브라우저가 PGM을 렌더링 못함)
- PC에서 `convert_map_to_png.sh` 실행하여 변환

## 설치

```bash
# 의존성 설치
npm install

# 환경 변수 설정
cp .env.example .env
# .env 파일에서 VITE_ROS_BRIDGE_URL 확인 (PC IP)
```

## 실행

```bash
# 개발 서버 시작 (http://localhost:3000)
npm run dev

# 프로덕션 빌드
npm run build
```

## ROS2 연동 (PC에서 실행)

### 1. rosbridge 서버 실행 (필수!)

```bash
# PC에서 실행 (로봇 자원 절약)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 2. Navigation 실행

```bash
# PC에서 실행
ros2 launch navigation navigation_pc.launch.py map:=mlm_slam.yaml robot_name:=robot1
```

### 3. Robot 실행

```bash
# Robot (Raspberry Pi Docker 내부)
ros2 launch bringup bringup_ns.launch.py robot_name:=robot1
```

### 4. 맵 파일 변환 및 배치

```bash
# PC에서 맵 PNG 변환 (최초 1회)
cd /path/to/pc
./convert_map_to_png.sh mlm_map
# 자동으로 web/public/maps/에 복사됨
```

## 로봇 추가 (멀티로봇)

`src/config/robots.ts` 파일에서 로봇 추가:

```typescript
export const ROBOTS: RobotConfig[] = [
  {
    namespace: 'robot1',
    name: 'MentorPi #1',
    color: '#22c55e',
  },
  // 아래 주석 해제하여 로봇 추가
  {
    namespace: 'robot2',
    name: 'MentorPi #2',
    color: '#3b82f6',
  },
  {
    namespace: 'robot3',
    name: 'MentorPi #3',
    color: '#f59e0b',
  },
];
```

## 맵 설정

`src/config/robots.ts` 에서 맵 파라미터 수정 (현재 mlm_map 설정됨):

```typescript
export const MAP_CONFIG: MapConfig = {
  image: '/maps/mlm_map.png',  // PNG 포맷 (PGM 아님!)
  resolution: 0.05,            // 5cm/픽셀 (mlm_map.yaml과 동일)
  origin: {
    x: -35.075,  // 맵 좌하단 X (mlm_map.yaml origin[0])
    y: -25.65,   // 맵 좌하단 Y (mlm_map.yaml origin[1])
    theta: 0,
  },
  width: 1403,    // 맵 이미지 너비 (픽셀)
  height: 1026,   // 맵 이미지 높이 (픽셀)
};
```

**중요**: 맵 파라미터는 `pc/ros2_ws/src/slam/maps/mlm_map.yaml`과 일치해야 함

## 키보드 단축키

| 키 | 동작 |
|----|------|
| W / ↑ | 전진 |
| S / ↓ | 후진 |
| A / ← | 좌회전 |
| D / → | 우회전 |
| Q | 전진 + 좌회전 |
| E | 전진 + 우회전 |
| Space | 정지 |

## 토픽 구조

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/{namespace}/odom` | nav_msgs/Odometry | Sub | 로봇 위치/속도 |
| `/{namespace}/scan_raw` | sensor_msgs/LaserScan | Sub | LiDAR 스캔 (Robot/PC 통일) |
| `/{namespace}/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | Sub | AMCL 위치 추정 |
| `/{namespace}/cmd_vel` | geometry_msgs/Twist | Pub | 속도 명령 |
| `/{namespace}/goal_pose` | geometry_msgs/PoseStamped | Pub | 목표점 |

## 프로젝트 구조

```
mlm-dashboard/
├── src/
│   ├── components/      # React 컴포넌트
│   │   ├── Dashboard.tsx
│   │   ├── MapView.tsx
│   │   ├── ControlPanel.tsx
│   │   └── ...
│   ├── hooks/           # 커스텀 훅 (ROS 통신)
│   │   ├── useRosBridge.ts
│   │   ├── useRobotState.ts
│   │   └── useGoalPublisher.ts
│   ├── stores/          # Zustand 상태 관리
│   │   └── robotStore.ts
│   ├── config/          # 설정
│   │   └── robots.ts
│   └── types/           # TypeScript 타입
│       └── ros.ts
├── public/
│   └── maps/            # 맵 이미지 (.png)
└── ...
```

## 문제 해결

### rosbridge 연결 안 됨

```bash
# 1. .env 파일 확인
cat .env
# VITE_ROS_BRIDGE_URL=ws://172.16.11.167:9090 (PC IP)

# 2. PC에서 rosbridge 실행 확인
# PC 터미널에서:
ps aux | grep rosbridge

# 3. 브라우저 콘솔 확인 (F12)
# WebSocket 연결 오류 메시지 확인
```

### 로봇 위치가 안 보임

```bash
# 1. rosbridge 연결 확인 (위 참조)

# 2. PC에서 토픽 확인
ros2 topic list | grep robot1
# /robot1/odom, /robot1/amcl_pose 있어야 함

# 3. 토픽 echo
ros2 topic echo /robot1/odom
ros2 topic echo /robot1/amcl_pose

# 4. 브라우저 콘솔 확인
# useRobotState 구독 메시지 확인
```

### 맵이 안 보임

```bash
# 1. 맵 파일 확인
ls -l public/maps/mlm_map.png

# 2. 맵 없으면 변환
cd ../pc
./convert_map_to_png.sh mlm_map

# 3. src/config/robots.ts 확인
# image: '/maps/mlm_map.png' 확인
```

### LiDAR 스캔이 안 보임

```bash
# 1. scan_raw 토픽 확인
# PC 터미널에서:
ros2 topic list | grep scan_raw
# /robot1/scan_raw 있어야 함

# 2. src/config/robots.ts 확인
# scan: '/scan_raw' 확인
```
