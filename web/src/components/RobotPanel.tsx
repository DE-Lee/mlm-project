import { useState } from 'react';
import { useRobotStore } from '@/stores/robotStore';
import { ROBOTS } from '@/config/robots';
import { RobotStatusPanel } from './RobotStatusPanel';
import { ControlPanel } from './ControlPanel';
import { SetInitialPosePanel } from './SetInitialPosePanel';
import { SetGoalPanel } from './SetGoalPanel';

interface RobotPanelProps {
  namespace: string;
}

export const RobotPanel = ({ namespace }: RobotPanelProps) => {
  const [isOpen, setIsOpen] = useState(true);
  const { robots } = useRobotStore();
  const robot = robots[namespace];
  const config = ROBOTS.find((r) => r.namespace === namespace);

  if (!robot || !config) return null;

  return (
    <div className="bg-gray-800 rounded-xl border border-gray-700 overflow-hidden">
      {/* 헤더 - 클릭하여 펼치기/접기 */}
      <button
        onClick={() => setIsOpen(!isOpen)}
        className="w-full px-4 py-3 flex items-center justify-between hover:bg-gray-750 transition-colors"
      >
        <div className="flex items-center gap-3">
          <div
            className="w-4 h-4 rounded-full"
            style={{ backgroundColor: config.color }}
          />
          <h3 className="font-semibold text-white">{config.name}</h3>
        </div>

        <div className="flex items-center gap-3">
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

          <svg
            className={`w-5 h-5 text-gray-400 transition-transform ${
              isOpen ? 'rotate-180' : ''
            }`}
            fill="none"
            stroke="currentColor"
            viewBox="0 0 24 24"
          >
            <path
              strokeLinecap="round"
              strokeLinejoin="round"
              strokeWidth={2}
              d="M19 9l-7 7-7-7"
            />
          </svg>
        </div>
      </button>

      {/* 내용 - 펼쳐진 상태에서만 표시 */}
      {isOpen && (
        <div className="p-4 space-y-4 border-t border-gray-700">
          {/* 위치/목표점 설정 버튼 */}
          <div className="flex gap-2">
            <SetInitialPosePanel namespace={namespace} />
            <SetGoalPanel namespace={namespace} />
          </div>

          {/* 로봇 상태 */}
          <RobotStatusPanel namespace={namespace} />

          {/* 수동 조작 */}
          <ControlPanel namespace={namespace} />
        </div>
      )}
    </div>
  );
};
