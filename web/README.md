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
- ROS2 Humble (rosbridge 실행 중)
- Nav2 Stack

## 설치

```bash
# 의존성 설치
npm install

# 환경 변수 설정
cp .env.example .env
# .env 파일에서 VITE_ROS_BRIDGE_URL 수정
```

## 실행

```bash
# 개발 서버 시작 (http://localhost:3000)
npm run dev

# 프로덕션 빌드
npm run build
```

## ROS2 설정

### 1. rosbridge 실행

```bash
# 로봇의 자원 확보를 위해 pc에서 수행
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 2. Nav2 실행

```bash
# 로봇 bringup 실행
ros2 ros2 launch bringup bringup_ns.launch.py robot_name:=robot1
```

### 3. 맵 파일 배치

```bash
# 변환된 맵 파일을 public/maps/ 에 복사
cp /path/to/map.pgm public/maps/mlm_map.pgm
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

`src/config/robots.ts` 에서 맵 파라미터 수정:

```typescript
export const MAP_CONFIG: MapConfig = {
  image: '/maps/mlm_map.pgm',
  resolution: 0.05,        // 미터/픽셀
  origin: {
    x: -10,  // 맵 좌하단 X (맵 YAML과 동일하게)
    y: -10,  // 맵 좌하단 Y
    theta: 0,
  },
  width: 400,   // 이미지 너비 (픽셀)
  height: 400,  // 이미지 높이 (픽셀)
};
```

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
│   └── maps/            # 맵 이미지 (.pgm)
└── ...
```
