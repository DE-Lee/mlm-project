import { useEffect, useCallback } from 'react';
import { useRobotStore } from '@/stores/robotStore';
import { useRosBridge } from '@/hooks/useRosBridge';
import { useRobotState } from '@/hooks/useRobotState';
import { useEmergencyStop } from '@/hooks/useEmergencyStop';
import { ROBOTS } from '@/config/robots';
import { ConnectionStatus } from './ConnectionStatus';
import { RobotSelector } from './RobotSelector';
import { RobotStatusPanel } from './RobotStatusPanel';
import { MapView } from './MapView';
import { ControlPanel } from './ControlPanel';

// 각 로봇의 상태를 구독하는 래퍼 컴포넌트
const RobotStateSubscriber = ({ namespace }: { namespace: string }) => {
  useRobotState(namespace);
  return null;
};

export const Dashboard = () => {
  // ROS 연결
  useRosBridge();

  const { selectedRobot, rosConnected } = useRobotStore();
  const { emergencyStopAll } = useEmergencyStop();

  // ESC 또는 X 키로 긴급 정지
  const handleKeyDown = useCallback(
    (e: KeyboardEvent) => {
      if (e.key === 'Escape' || e.key === 'x' || e.key === 'X') {
        e.preventDefault();
        emergencyStopAll();
      }
    },
    [emergencyStopAll]
  );

  useEffect(() => {
    window.addEventListener('keydown', handleKeyDown);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, [handleKeyDown]);

  return (
    <div className="min-h-screen bg-gray-900 text-white">
      {/* 각 로봇 상태 구독 */}
      {ROBOTS.map((robot) => (
        <RobotStateSubscriber key={robot.namespace} namespace={robot.namespace} />
      ))}

      {/* 헤더 */}
      <header className="bg-gray-800 border-b border-gray-700 px-6 py-4">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-4">
            <h1 className="text-xl font-bold">
              <span className="text-green-400">MLM</span> Dashboard
            </h1>
            <span className="text-gray-500 text-sm">
              Multi-robot Learning for MentorPi
            </span>
          </div>

          <div className="flex items-center gap-4">
            {/* 긴급 정지 버튼 */}
            <button
              onClick={emergencyStopAll}
              className="px-6 py-3 bg-red-600 hover:bg-red-700 active:bg-red-800 text-white font-bold rounded-lg shadow-lg transition-all transform hover:scale-105 active:scale-95 border-2 border-red-500"
              title="긴급 정지 (ESC)"
            >
              STOP
            </button>

            <RobotSelector />
            <ConnectionStatus />
          </div>
        </div>
      </header>

      {/* 메인 컨텐츠 */}
      <main className="p-6">
        <div className="grid grid-cols-12 gap-6">
          {/* 맵 뷰 (8/12) */}
          <div className="col-span-8">
            <div className="bg-gray-800 rounded-xl p-4 border border-gray-700">
              <h2 className="text-lg font-semibold mb-4">Map View</h2>
              <MapView width={700} height={500} />
              
              {!rosConnected && (
                <div className="mt-4 p-4 bg-yellow-900/50 border border-yellow-600 rounded-lg">
                  <p className="text-yellow-400 text-sm">
                    ⚠️ ROS bridge에 연결되지 않았습니다. 
                    rosbridge가 실행 중인지 확인하세요.
                  </p>
                  <code className="text-xs text-gray-400 mt-2 block">
                    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
                  </code>
                </div>
              )}
            </div>
          </div>

          {/* 사이드바 (4/12) */}
          <div className="col-span-4 space-y-4">
            {/* 선택된 로봇 상태 */}
            {selectedRobot && (
              <>
                <RobotStatusPanel namespace={selectedRobot} />
                <ControlPanel namespace={selectedRobot} />
              </>
            )}

            {/* 모든 로봇 요약 (멀티로봇 시) */}
            {ROBOTS.length > 1 && (
              <div className="bg-gray-800 rounded-xl p-4 border border-gray-700">
                <h3 className="text-white font-semibold mb-3">All Robots</h3>
                <div className="space-y-2">
                  {ROBOTS.map((robot) => (
                    <RobotMiniCard key={robot.namespace} namespace={robot.namespace} />
                  ))}
                </div>
              </div>
            )}

            {/* 도움말 */}
            <div className="bg-gray-800 rounded-xl p-4 border border-gray-700">
              <h3 className="text-white font-semibold mb-2">Quick Help</h3>
              <ul className="text-sm text-gray-400 space-y-1">
                <li>• <strong>초기 위치</strong>: 버튼 클릭 → 맵에서 클릭+드래그</li>
                <li>• <strong>목표점 설정</strong>: 버튼 클릭 후 맵에서 위치 클릭</li>
                <li>• <strong>수동 조작</strong>: WASD 또는 방향키</li>
                <li>• <strong>긴급 정지</strong>: ESC / X / STOP 버튼</li>
                <li>• <strong>로봇 전환</strong>: 상단 로봇 탭 클릭</li>
              </ul>
            </div>
          </div>
        </div>
      </main>
    </div>
  );
};

// 로봇 미니 카드 (멀티로봇 목록용)
const RobotMiniCard = ({ namespace }: { namespace: string }) => {
  const { robots, selectedRobot, setSelectedRobot } = useRobotStore();
  const robot = robots[namespace];
  const config = ROBOTS.find((r) => r.namespace === namespace);

  if (!robot || !config) return null;

  return (
    <button
      onClick={() => setSelectedRobot(namespace)}
      className={`w-full flex items-center gap-3 p-2 rounded-lg transition-colors ${
        selectedRobot === namespace
          ? 'bg-gray-700'
          : 'bg-gray-900 hover:bg-gray-700'
      }`}
    >
      <div
        className="w-3 h-3 rounded-full"
        style={{ backgroundColor: config.color }}
      />
      <span className="text-sm text-white flex-1 text-left">{config.name}</span>
      <span className={`text-xs ${robot.connected ? 'text-green-400' : 'text-gray-500'}`}>
        {robot.connected ? 'Online' : 'Offline'}
      </span>
    </button>
  );
};
