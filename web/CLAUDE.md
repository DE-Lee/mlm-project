# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 프로젝트 개요

MLM Dashboard는 ROS2 기반 MentorPi 로봇을 위한 웹 제어 대시보드이다. rosbridge를 통해 WebSocket으로 ROS2와 통신하며, 최대 3대의 로봇을 동시에 모니터링하고 제어할 수 있다.

## 개발 명령어

```bash
npm install          # 의존성 설치
npm run dev          # 개발 서버 (http://localhost:3000)
npm run build        # 프로덕션 빌드 (tsc && vite build)
npm run preview      # 빌드 결과 미리보기
```

## 아키텍처

### 상태 관리 흐름
```
ROS2 Topics → rosbridge WebSocket → useRosBridge (연결 관리)
                                  → useRobotState (토픽 구독/파싱)
                                  → robotStore (Zustand 전역 상태)
                                  → React 컴포넌트
```

### 핵심 모듈

- **`src/hooks/useRosBridge.ts`**: rosbridge WebSocket 연결 싱글톤 관리. 자동 재연결 로직 포함.
- **`src/hooks/useRobotState.ts`**: 로봇별 odom/amcl_pose 토픽 구독. Quaternion→Yaw 변환, Gazebo offset 적용.
- **`src/hooks/useGoalPublisher.ts`**: goal_pose (Nav2 목표점), cmd_vel (수동 제어) 발행.
- **`src/stores/robotStore.ts`**: Zustand 스토어. 로봇 pose/velocity/status/goal 상태 관리.
- **`src/config/robots.ts`**: 로봇 목록, ROS 연결 설정, 맵 파라미터. **로봇 추가/맵 변경 시 이 파일 수정.**

### 좌표 변환 (MapView.tsx)
- `pixelToWorld`: 캔버스 클릭 → ROS 월드 좌표
- `worldToPixel`: ROS 월드 좌표 → 캔버스 렌더링 위치
- MAP_CONFIG의 resolution, origin, gazeboOffset 값 사용

## ROS2 연동

### 필수 실행
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml  # WebSocket 브릿지
ros2 launch nav2_bringup bringup_launch.py map:=/path/to/map.yaml  # Nav2 스택
```

### 토픽 네이밍
`/{namespace}/{topic}` 형식. 예: `/robot1/odom`, `/robot1/cmd_vel`

구독: odom, amcl_pose
발행: cmd_vel, goal_pose

## 설정 변경

### 로봇 추가
`src/config/robots.ts`의 ROBOTS 배열에 추가:
```typescript
{ namespace: 'robot2', name: 'MentorPi #2', color: '#3b82f6' }
```

### 맵 변경
1. 맵 이미지를 `public/maps/`에 복사 (PNG 권장)
2. `src/config/robots.ts`의 MAP_CONFIG 수정 (resolution, origin, width, height)
3. Gazebo 시뮬레이터 사용 시 gazeboOffset 설정

### ROS 브릿지 URL
`.env` 파일의 `VITE_ROS_BRIDGE_URL` 또는 `src/config/robots.ts`의 ROS_CONFIG.url

## 경로 별칭

`@/` → `src/` (vite.config.ts, tsconfig.json에 설정됨)
