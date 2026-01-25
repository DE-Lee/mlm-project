import { create } from 'zustand';

type MapMode = 'none' | 'goal' | 'initialPose';

interface MapInteractionStore {
  mapMode: MapMode;
  activeRobot: string | null;
  setMapMode: (mode: MapMode, robot: string | null) => void;
}

export const useMapInteractionStore = create<MapInteractionStore>((set) => ({
  mapMode: 'none',
  activeRobot: null,
  setMapMode: (mode, robot) =>
    set({
      mapMode: mode,
      activeRobot: robot,
    }),
}));

export const useMapInteraction = () => {
  const { mapMode, activeRobot, setMapMode } = useMapInteractionStore();

  return {
    mapMode,
    activeRobot,
    setMapMode,
  };
};
