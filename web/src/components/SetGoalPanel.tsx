import { useMapInteraction } from '@/hooks/useMapInteraction';
import { useRobotStore } from '@/stores/robotStore';

interface SetGoalPanelProps {
  namespace: string;
}

export const SetGoalPanel = ({ namespace }: SetGoalPanelProps) => {
  const { mapMode, setMapMode, activeRobot } = useMapInteraction();
  const { rosConnected } = useRobotStore();

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
      disabled={!rosConnected}
      className={`flex-1 px-3 py-2 rounded-lg font-medium text-sm transition-colors ${
        isActive
          ? 'bg-green-600 text-white'
          : !rosConnected
          ? 'bg-gray-800 text-gray-500 cursor-not-allowed'
          : 'bg-gray-700 text-gray-300 hover:bg-gray-600'
      }`}
      title={!rosConnected ? 'ROS 연결 대기 중...' : ''}
    >
      {isActive ? '목표점 설정 중...' : !rosConnected ? '⏳ 연결 중...' : '목표점 설정'}
    </button>
  );
};
