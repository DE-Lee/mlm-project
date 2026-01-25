// ROS 메시지 타입 정의

export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface Point {
  x: number;
  y: number;
  z: number;
}

export interface Pose {
  position: Point;
  orientation: Quaternion;
}

export interface PoseWithCovariance {
  pose: Pose;
  covariance: number[];
}

export interface Header {
  stamp: { sec: number; nanosec: number };
  frame_id: string;
}

export interface Twist {
  linear: Vector3;
  angular: Vector3;
}

export interface TwistWithCovariance {
  twist: Twist;
  covariance: number[];
}

// nav_msgs/Odometry
export interface Odometry {
  header: Header;
  child_frame_id: string;
  pose: PoseWithCovariance;
  twist: TwistWithCovariance;
}

// geometry_msgs/PoseStamped
export interface PoseStamped {
  header: Header;
  pose: Pose;
}

// geometry_msgs/PoseWithCovarianceStamped
export interface PoseWithCovarianceStamped {
  header: Header;
  pose: PoseWithCovariance;
}

// sensor_msgs/LaserScan
export interface LaserScan {
  header: Header;
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  time_increment: number;
  scan_time: number;
  range_min: number;
  range_max: number;
  ranges: number[];
  intensities: number[];
}

// sensor_msgs/BatteryState
export interface BatteryState {
  header: Header;
  voltage: number;
  percentage: number;
  power_supply_status: number;
}

// 로봇 상태 (앱 내부용)
export interface RobotState {
  namespace: string;
  connected: boolean;
  pose: {
    x: number;
    y: number;
    theta: number;  // yaw in radians
  } | null;  // null이면 아직 위치를 모름
  velocity: {
    linear: number;
    angular: number;
  };
  battery: number;
  status: 'idle' | 'navigating' | 'error';
  goal: {
    x: number;
    y: number;
  } | null;
}

// 맵 설정
export interface MapConfig {
  image: string;
  resolution: number;  // meters per pixel
  origin: {
    x: number;
    y: number;
    theta: number;
  };
  width: number;   // pixels
  height: number;  // pixels
  // Gazebo 시뮬레이터 사용 여부
  useGazeboOffset?: boolean;
  // Gazebo 시뮬레이터의 맵 모델 world 좌표 offset
  gazeboOffset?: {
    x: number;
    y: number;
  };
}

// 로봇 설정
export interface RobotConfig {
  namespace: string;
  name: string;
  color: string;
}
