import { RobotConfig, MapConfig } from '@/types/ros';

// ============================================================
// 로봇 설정 - 여기에 로봇을 추가하면 대시보드에 자동 반영
// ============================================================

export const ROBOTS: RobotConfig[] = [
  {
    namespace: 'robot1',
    name: 'MentorPi #1',
    color: '#22c55e',  // green
  },
  {
    namespace: 'robot2',
    name: 'MentorPi #2',
    color: '#3b82f6',  // blue
  },
  // 로봇 추가 시 아래 주석 해제
  // {
  //   namespace: 'robot3',
  //   name: 'MentorPi #3',
  //   color: '#f59e0b',  // amber
  // },
];

// ============================================================
// ROS 연결 설정
// ============================================================

export const ROS_CONFIG = {
  // rosbridge WebSocket URL
  // .env 파일에서 VITE_ROS_BRIDGE_URL 설정
  // 예: ws://172.16.10.172:9090
  url: import.meta.env.VITE_ROS_BRIDGE_URL || 'ws://localhost:9090',

  // 재연결 설정
  reconnectInterval: 3000,  // ms
  maxReconnectAttempts: 10,
};

// ============================================================
// 맵 설정
// ============================================================

export const MAP_CONFIG: MapConfig = {
  image: '/maps/mlm_slam.png',  // public/maps/mlm_slam.png (PNG for browser compatibility)
  resolution: 0.05,        // 5cm per pixel (mlm_slam.yaml)
  origin: {
    x: -17.4,     // 맵 좌하단 x 좌표 (meters) - mlm_slam.yaml origin[0]
    y: -7.48,     // 맵 좌하단 y 좌표 (meters) - mlm_slam.yaml origin[1]
    theta: 0,
  },
  width: 1454,    // 맵 이미지 너비 (pixels)
  height: 501,    // 맵 이미지 높이 (pixels)
  // Gazebo 시뮬레이터 사용 여부 (환경변수로 설정 가능)
  // true: Gazebo offset 적용, false: 실제 로봇
  useGazeboOffset: import.meta.env.VITE_USE_GAZEBO === 'true',
  // Gazebo final_map 모델의 world 좌표 offset
  gazeboOffset: {
    x: 39.51,
    y: 29.43,
  },
};

// ============================================================
// 토픽 이름 생성 헬퍼
// ============================================================

export const getTopicName = (namespace: string, topic: string): string => {
  // namespace가 비어있으면 토픽 이름만 반환
  if (!namespace) {
    return topic;
  }
  return `/${namespace}${topic}`;
};

// 로봇별 토픽
export const TOPICS = {
  // 구독 (Subscribe)
  odom: '/odom',
  scan: '/scan_raw',  // LiDAR raw 스캔 데이터 (Robot과 PC Navigation 통일)
  amclPose: '/amcl_pose',
  battery: '/ros_robot_controller/battery',  // std_msgs/UInt16 (millivolts)

  // 발행 (Publish)
  cmdVel: '/cmd_vel',
  cmdVelManual: '/controller/cmd_vel',  // 수동 제어용 (Nav2와 충돌 방지)
  goalPose: '/goal_pose',
  initialPose: '/initialpose',
};
