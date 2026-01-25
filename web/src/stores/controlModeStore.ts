import { create } from 'zustand';

export enum ControlMode {
  AUTONOMOUS = 'autonomous',  // Nav2가 제어
  MANUAL = 'manual',          // 사용자가 제어
  IDLE = 'idle',              // 정지 상태
}

interface ControlModeStore {
  // 각 로봇의 제어 모드
  modes: Record<string, ControlMode>;

  // 모드 설정
  setMode: (namespace: string, mode: ControlMode) => void;

  // 모드 조회
  getMode: (namespace: string) => ControlMode;

  // 모든 로봇 IDLE로 전환
  setAllIdle: () => void;
}

export const useControlModeStore = create<ControlModeStore>((set, get) => ({
  modes: {},

  setMode: (namespace, mode) => {
    console.log(`[ControlMode] ${namespace}: ${mode}`);
    set((state) => ({
      modes: { ...state.modes, [namespace]: mode },
    }));
  },

  getMode: (namespace) => {
    return get().modes[namespace] || ControlMode.IDLE;
  },

  setAllIdle: () => {
    const currentModes = get().modes;
    const newModes: Record<string, ControlMode> = {};
    Object.keys(currentModes).forEach((ns) => {
      newModes[ns] = ControlMode.IDLE;
    });
    set({ modes: newModes });
  },
}));
