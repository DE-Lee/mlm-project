import { useRobotStore } from '@/stores/robotStore';
import { ROBOTS } from '@/config/robots';

interface RobotStatusPanelProps {
  namespace: string;
}

export const RobotStatusPanel = ({ namespace }: RobotStatusPanelProps) => {
  const { robots } = useRobotStore();
  const robot = robots[namespace];
  const config = ROBOTS.find((r) => r.namespace === namespace);

  if (!robot || !config) return null;

  const statusColors = {
    idle: 'bg-gray-500',
    navigating: 'bg-blue-500 animate-pulse',
    error: 'bg-red-500',
  };

  const radToDeg = (rad: number) => ((rad * 180) / Math.PI).toFixed(1);

  return (
    <div className="bg-gray-800 rounded-xl p-4 border border-gray-700">
      {/* 헤더 */}
      <div className="flex items-center justify-between mb-4">
        <div className="flex items-center gap-2">
          <div
            className="w-4 h-4 rounded-full"
            style={{ backgroundColor: config.color }}
          />
          <h3 className="font-semibold text-white">{config.name}</h3>
        </div>
        <div className="flex items-center gap-2">
          <div
            className={`w-2 h-2 rounded-full ${
              robot.connected ? 'bg-green-500' : 'bg-red-500'
            }`}
          />
          <span className="text-xs text-gray-400">
            {robot.connected ? 'Online' : 'Offline'}
          </span>
        </div>
      </div>

      {/* 상태 */}
      <div className="grid grid-cols-2 gap-3 text-sm">
        {/* 위치 */}
        <div className="bg-gray-900 rounded-lg p-3">
          <div className="text-gray-400 text-xs mb-1">Position</div>
          <div className="text-white font-mono">
            {robot.pose ? (
              <>
                <div>X: {robot.pose.x.toFixed(2)} m</div>
                <div>Y: {robot.pose.y.toFixed(2)} m</div>
                <div>θ: {radToDeg(robot.pose.theta)}°</div>
              </>
            ) : (
              <div className="text-gray-500">위치 수신 대기 중...</div>
            )}
          </div>
        </div>

        {/* 속도 */}
        <div className="bg-gray-900 rounded-lg p-3">
          <div className="text-gray-400 text-xs mb-1">Velocity</div>
          <div className="text-white font-mono">
            <div>Linear: {robot.velocity.linear.toFixed(2)} m/s</div>
            <div>Angular: {robot.velocity.angular.toFixed(2)} rad/s</div>
          </div>
        </div>

        {/* 배터리 */}
        <div className="bg-gray-900 rounded-lg p-3">
          <div className="text-gray-400 text-xs mb-1">Battery</div>
          <div className="flex items-center gap-2">
            <div className="flex-1 h-2 bg-gray-700 rounded-full overflow-hidden">
              <div
                className={`h-full transition-all ${
                  robot.battery > 50
                    ? 'bg-green-500'
                    : robot.battery > 20
                    ? 'bg-yellow-500'
                    : 'bg-red-500'
                }`}
                style={{ width: `${robot.battery}%` }}
              />
            </div>
            <span className="text-white font-mono text-xs">
              {robot.battery.toFixed(0)}%
            </span>
          </div>
        </div>

        {/* 상태 */}
        <div className="bg-gray-900 rounded-lg p-3">
          <div className="text-gray-400 text-xs mb-1">Status</div>
          <div className="flex items-center gap-2">
            <div className={`w-2 h-2 rounded-full ${statusColors[robot.status]}`} />
            <span className="text-white capitalize">{robot.status}</span>
          </div>
          {robot.goal && (
            <div className="text-xs text-gray-400 mt-1">
              Goal: ({robot.goal.x.toFixed(1)}, {robot.goal.y.toFixed(1)})
            </div>
          )}
        </div>
      </div>
    </div>
  );
};
