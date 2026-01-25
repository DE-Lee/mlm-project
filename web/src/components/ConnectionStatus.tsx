import { useRobotStore } from '@/stores/robotStore';
import { ROS_CONFIG } from '@/config/robots';

export const ConnectionStatus = () => {
  const { rosConnected } = useRobotStore();

  return (
    <div className="flex items-center gap-2 px-3 py-1.5 bg-gray-800 rounded-lg">
      <div
        className={`w-2.5 h-2.5 rounded-full ${
          rosConnected 
            ? 'bg-green-500 shadow-lg shadow-green-500/50' 
            : 'bg-red-500 animate-pulse'
        }`}
      />
      <span className="text-sm text-gray-300">
        {rosConnected ? 'ROS Connected' : 'Disconnected'}
      </span>
      {!rosConnected && (
        <span className="text-xs text-gray-500 ml-1">
          ({ROS_CONFIG.url})
        </span>
      )}
    </div>
  );
};
