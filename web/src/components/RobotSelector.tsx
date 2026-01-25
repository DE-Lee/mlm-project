import { useRobotStore } from '@/stores/robotStore';
import { ROBOTS } from '@/config/robots';

export const RobotSelector = () => {
  const { robots, selectedRobot, setSelectedRobot } = useRobotStore();

  return (
    <div className="flex gap-2">
      {ROBOTS.map((config) => {
        const robot = robots[config.namespace];
        const isSelected = selectedRobot === config.namespace;
        
        return (
          <button
            key={config.namespace || 'default'}
            onClick={() => setSelectedRobot(config.namespace)}
            className={`flex items-center gap-2 px-4 py-2 rounded-lg transition-all ${
              isSelected
                ? 'bg-gray-700 ring-2 ring-offset-2 ring-offset-gray-900'
                : 'bg-gray-800 hover:bg-gray-700'
            }`}
            style={
              isSelected
                ? ({ '--tw-ring-color': config.color } as React.CSSProperties)
                : undefined
            }
          >
            {/* 상태 인디케이터 */}
            <div className="relative">
              <div
                className="w-3 h-3 rounded-full"
                style={{ backgroundColor: config.color }}
              />
              {robot?.connected && (
                <div
                  className="absolute inset-0 rounded-full animate-ping opacity-75"
                  style={{ backgroundColor: config.color }}
                />
              )}
            </div>
            
            {/* 로봇 이름 */}
            <span className={`text-sm ${isSelected ? 'text-white' : 'text-gray-400'}`}>
              {config.name}
            </span>
            
            {/* 연결 상태 */}
            <span className={`text-xs ${robot?.connected ? 'text-green-400' : 'text-red-400'}`}>
              {robot?.connected ? '●' : '○'}
            </span>
          </button>
        );
      })}
    </div>
  );
};
