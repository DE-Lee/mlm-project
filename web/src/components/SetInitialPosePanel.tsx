import { useMapInteraction } from '@/hooks/useMapInteraction';

interface SetInitialPosePanelProps {
  namespace: string;
}

export const SetInitialPosePanel = ({ namespace }: SetInitialPosePanelProps) => {
  const { mapMode, setMapMode, activeRobot } = useMapInteraction();

  const isActive = mapMode === 'initialPose' && activeRobot === namespace;

  const handleClick = () => {
    if (isActive) {
      setMapMode('none', null);
    } else {
      setMapMode('initialPose', namespace);
    }
  };

  return (
    <button
      onClick={handleClick}
      className={`flex-1 px-3 py-2 rounded-lg font-medium text-sm transition-colors ${
        isActive
          ? 'bg-amber-600 text-white'
          : 'bg-gray-700 text-gray-300 hover:bg-gray-600'
      }`}
    >
      {isActive ? '위치 설정 중...' : '초기 위치 설정'}
    </button>
  );
};
