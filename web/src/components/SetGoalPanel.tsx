import { useMapInteraction } from '@/hooks/useMapInteraction';
import { useNavigation } from '@/hooks/useNavigation';

interface SetGoalPanelProps {
  namespace: string;
}

export const SetGoalPanel = ({ namespace }: SetGoalPanelProps) => {
  const { mapMode, setMapMode, activeRobot } = useMapInteraction();
  const { isActionServerReady } = useNavigation(namespace);

  const isActive = mapMode === 'goal' && activeRobot === namespace;

  const handleClick = () => {
    if (isActive) {
      setMapMode('none', null);
    } else {
      setMapMode('goal', namespace);
    }
  };

  return (
    <button
      onClick={handleClick}
      disabled={!isActionServerReady}
      className={`flex-1 px-3 py-2 rounded-lg font-medium text-sm transition-colors ${
        isActive
          ? 'bg-green-600 text-white'
          : !isActionServerReady
          ? 'bg-gray-800 text-gray-500 cursor-not-allowed'
          : 'bg-gray-700 text-gray-300 hover:bg-gray-600'
      }`}
      title={!isActionServerReady ? 'Navigation 서버 준비 중...' : ''}
    >
      {isActive ? '목표점 설정 중...' : !isActionServerReady ? '⏳ 준비 중...' : '목표점 설정'}
    </button>
  );
};
