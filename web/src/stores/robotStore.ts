import { create } from 'zustand';
import { RobotState } from '@/types/ros';
import { ROBOTS } from '@/config/robots';

interface RobotStore {
  // 연결 상태
  rosConnected: boolean;
  setRosConnected: (connected: boolean) => void;
  
  // 로봇 상태 (namespace로 인덱싱)
  robots: Record<string, RobotState>;
  
  // 로봇 상태 업데이트
  updateRobotPose: (namespace: string, x: number, y: number, theta: number) => void;
  updateRobotVelocity: (namespace: string, linear: number, angular: number) => void;
  updateRobotBattery: (namespace: string, percentage: number) => void;
  updateRobotStatus: (namespace: string, status: RobotState['status']) => void;
  updateRobotGoal: (namespace: string, goal: { x: number; y: number } | null) => void;
  setRobotConnected: (namespace: string, connected: boolean) => void;
  
  // 선택된 로봇
  selectedRobot: string | null;
  setSelectedRobot: (namespace: string | null) => void;
}

// 초기 로봇 상태 생성
const createInitialRobotState = (namespace: string): RobotState => ({
  namespace,
  connected: false,
  pose: null,  // 초기 위치는 null - 첫 odom/amcl 메시지가 올 때까지 위치 모름
  velocity: { linear: 0, angular: 0 },
  battery: 100,
  status: 'idle',
  goal: null,
});

// 초기 상태
const initialRobots: Record<string, RobotState> = {};
ROBOTS.forEach((robot) => {
  initialRobots[robot.namespace] = createInitialRobotState(robot.namespace);
});

export const useRobotStore = create<RobotStore>((set) => ({
  // 연결 상태
  rosConnected: false,
  setRosConnected: (connected) => set({ rosConnected: connected }),
  
  // 로봇 상태
  robots: initialRobots,
  
  // 로봇 상태 업데이트 함수들
  updateRobotPose: (namespace, x, y, theta) =>
    set((state) => ({
      robots: {
        ...state.robots,
        [namespace]: {
          ...state.robots[namespace],
          pose: { x, y, theta },
        },
      },
    })),
    
  updateRobotVelocity: (namespace, linear, angular) =>
    set((state) => ({
      robots: {
        ...state.robots,
        [namespace]: {
          ...state.robots[namespace],
          velocity: { linear, angular },
        },
      },
    })),
    
  updateRobotBattery: (namespace, percentage) =>
    set((state) => ({
      robots: {
        ...state.robots,
        [namespace]: {
          ...state.robots[namespace],
          battery: percentage,
        },
      },
    })),
    
  updateRobotStatus: (namespace, status) =>
    set((state) => ({
      robots: {
        ...state.robots,
        [namespace]: {
          ...state.robots[namespace],
          status,
        },
      },
    })),
    
  updateRobotGoal: (namespace, goal) =>
    set((state) => ({
      robots: {
        ...state.robots,
        [namespace]: {
          ...state.robots[namespace],
          goal,
          status: goal ? 'navigating' : 'idle',
        },
      },
    })),
    
  setRobotConnected: (namespace, connected) =>
    set((state) => ({
      robots: {
        ...state.robots,
        [namespace]: {
          ...state.robots[namespace],
          connected,
        },
      },
    })),
  
  // 선택된 로봇
  selectedRobot: ROBOTS[0]?.namespace || null,
  setSelectedRobot: (namespace) => set({ selectedRobot: namespace }),
}));
