import { create } from 'zustand';

export enum ControlMode {
  AUTONOMOUS = 'autonomous',  // Nav2가 제어
  MANUAL = 'manual',          // 사용자가 제어
  IDLE = 'idle',              // 정지 상태
}

interface ControlModeStore {
  // 각 로봇의 제어 모드
  modes: Record<string, ControlMode>;

  // 모드 변경
  setMode: (namespace: string, mode: ControlMode) => void;

  // 모드 확인
  getMode: (namespace: string) => ControlMode;

  // Manual 모드로 전환 (안전하게)
  switchToManual: (namespace: string, cancelGoalFn: () => void) => void;

  // Autonomous 모드로 전환
  switchToAutonomous: (namespace: string, stopManualFn: () => void) => void;
}

export const useControlModeStore = create<ControlModeStore>((set, get) => ({
  modes: {},

  setMode: (namespace, mode) =>
    set((state) => ({
      modes: { ...state.modes, [namespace]: mode },
    })),

  getMode: (namespace) => {
    return get().modes[namespace] || ControlMode.IDLE;
  },

  switchToManual: (namespace, cancelGoalFn) => {
    console.log(`[${namespace}] Switching to MANUAL mode`);

    // 1. Nav2 취소
    cancelGoalFn();

    // 2. 모드 전환
    set((state) => ({
      modes: { ...state.modes, [namespace]: ControlMode.MANUAL },
    }));
  },

  switchToAutonomous: (namespace, stopManualFn) => {
    console.log(`[${namespace}] Switching to AUTONOMOUS mode`);

    // 1. Manual 정지
    stopManualFn();

    // 2. 모드 전환
    set((state) => ({
      modes: { ...state.modes, [namespace]: ControlMode.AUTONOMOUS },
    }));
  },
}));

export const useControlMode = (namespace: string) => {
  const { modes, setMode, getMode, switchToManual, switchToAutonomous } = useControlModeStore();

  return {
    currentMode: modes[namespace] || ControlMode.IDLE,
    setMode: (mode: ControlMode) => setMode(namespace, mode),
    getMode: () => getMode(namespace),
    switchToManual: (cancelGoalFn: () => void) => switchToManual(namespace, cancelGoalFn),
    switchToAutonomous: (stopManualFn: () => void) => switchToAutonomous(namespace, stopManualFn),
  };
};
