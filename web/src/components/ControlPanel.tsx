import { useState, useCallback, useRef, useEffect } from 'react';
import { useManualControl } from '@/hooks/useNavigation';
import { useNavigation } from '@/hooks/useNavigation';
import { useRobotStore } from '@/stores/robotStore';
import { useControlModeStore, ControlMode } from '@/stores/controlModeStore';

interface ControlPanelProps {
  namespace: string;
}

export const ControlPanel = ({ namespace }: ControlPanelProps) => {
  const { publishVelocity, stop } = useManualControl(namespace);
  const { cancelNavigation } = useNavigation(namespace);
  const { robots, keyboardEnabledRobots, toggleKeyboardEnabled } = useRobotStore();
  const { modes } = useControlModeStore();

  const robot = robots[namespace];
  const currentMode = modes[namespace] || ControlMode.IDLE;
  const isKeyboardEnabled = keyboardEnabledRobots[namespace] || false;
  const [activeKey, setActiveKey] = useState<string | null>(null);
  const intervalRef = useRef<NodeJS.Timeout | null>(null);

  // 속도 설정
  const maxLinear = 0.3;  // m/s
  const maxAngular = 0.5; // rad/s

  // 키보드 컨트롤
  const handleKeyDown = useCallback(
    (key: string) => {
      if (intervalRef.current) return; // 이미 누르고 있음
      
      setActiveKey(key);
      
      const sendCommand = () => {
        switch (key) {
          case 'w':
          case 'ArrowUp':
            publishVelocity(maxLinear, 0);
            break;
          case 's':
          case 'ArrowDown':
            publishVelocity(-maxLinear, 0);
            break;
          case 'a':
          case 'ArrowLeft':
            publishVelocity(0.05, maxAngular);  // 약간 전진하며 좌회전 (Ackermann)
            break;
          case 'd':
          case 'ArrowRight':
            publishVelocity(0.05, -maxAngular);  // 약간 전진하며 우회전 (Ackermann)
            break;
          case 'q':
            publishVelocity(maxLinear, maxAngular);
            break;
          case 'e':
            publishVelocity(maxLinear, -maxAngular);
            break;
        }
      };
      
      sendCommand();
      intervalRef.current = setInterval(sendCommand, 100);
    },
    [publishVelocity, maxLinear, maxAngular]
  );

  const handleKeyUp = useCallback(() => {
    setActiveKey(null);
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
    stop();
  }, [stop]);

  // 전역 키보드 이벤트 (키보드 활성화 시만)
  useEffect(() => {
    // 키보드 비활성화 시 무시
    if (!isKeyboardEnabled) {
      return;
    }

    const handleGlobalKeyDown = (e: KeyboardEvent) => {
      if (['w', 'a', 's', 'd', 'q', 'e', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.key)) {
        e.preventDefault();
        handleKeyDown(e.key);
      }
      if (e.key === ' ') {
        e.preventDefault();
        stop();
      }
    };

    const handleGlobalKeyUp = (e: KeyboardEvent) => {
      if (['w', 'a', 's', 'd', 'q', 'e', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.key)) {
        handleKeyUp();
      }
    };

    window.addEventListener('keydown', handleGlobalKeyDown);
    window.addEventListener('keyup', handleGlobalKeyUp);

    return () => {
      window.removeEventListener('keydown', handleGlobalKeyDown);
      window.removeEventListener('keyup', handleGlobalKeyUp);
    };
  }, [handleKeyDown, handleKeyUp, stop, isKeyboardEnabled]);

  const buttonClass = (key: string) =>
    `w-12 h-12 rounded-lg font-bold text-lg transition-all ${
      !isKeyboardEnabled
        ? 'bg-gray-800 text-gray-600 cursor-not-allowed'
        : activeKey === key
        ? 'bg-green-600 text-white scale-95'
        : 'bg-gray-700 text-gray-300 hover:bg-gray-600'
    }`;

  return (
    <div className="bg-gray-900 rounded-lg p-3">
      <div className="flex items-center justify-between mb-4">
        <h3 className="text-white font-semibold">Manual Control</h3>

        {/* 키보드 활성화 토글 */}
        <button
          onClick={() => toggleKeyboardEnabled(namespace)}
          className={`px-3 py-1 rounded text-xs font-medium transition-colors ${
            isKeyboardEnabled
              ? 'bg-green-600 text-white'
              : 'bg-gray-700 text-gray-400 hover:bg-gray-600'
          }`}
        >
          {isKeyboardEnabled ? '⌨️ 키보드 ON' : '⌨️ 키보드 OFF'}
        </button>
      </div>

      {/* 방향 버튼 */}
      <div className="flex flex-col items-center gap-2 mb-4">
        <div className="flex gap-2">
          <button
            className={buttonClass('q')}
            disabled={!isKeyboardEnabled}
            onMouseDown={() => isKeyboardEnabled && handleKeyDown('q')}
            onMouseUp={handleKeyUp}
            onMouseLeave={handleKeyUp}
          >
            ↖
          </button>
          <button
            className={buttonClass('w')}
            disabled={!isKeyboardEnabled}
            onMouseDown={() => isKeyboardEnabled && handleKeyDown('w')}
            onMouseUp={handleKeyUp}
            onMouseLeave={handleKeyUp}
          >
            ↑
          </button>
          <button
            className={buttonClass('e')}
            disabled={!isKeyboardEnabled}
            onMouseDown={() => isKeyboardEnabled && handleKeyDown('e')}
            onMouseUp={handleKeyUp}
            onMouseLeave={handleKeyUp}
          >
            ↗
          </button>
        </div>

        <div className="flex gap-2">
          <button
            className={buttonClass('a')}
            disabled={!isKeyboardEnabled}
            onMouseDown={() => isKeyboardEnabled && handleKeyDown('a')}
            onMouseUp={handleKeyUp}
            onMouseLeave={handleKeyUp}
          >
            ←
          </button>
          <button
            className={`w-12 h-12 rounded-lg font-bold ${
              !isKeyboardEnabled
                ? 'bg-gray-800 text-gray-600 cursor-not-allowed'
                : 'bg-red-600 text-white hover:bg-red-700'
            }`}
            disabled={!isKeyboardEnabled}
            onClick={() => isKeyboardEnabled && stop()}
          >
            ■
          </button>
          <button
            className={buttonClass('d')}
            disabled={!isKeyboardEnabled}
            onMouseDown={() => isKeyboardEnabled && handleKeyDown('d')}
            onMouseUp={handleKeyUp}
            onMouseLeave={handleKeyUp}
          >
            →
          </button>
        </div>

        <div className="flex gap-2">
          <div className="w-12 h-12" />
          <button
            className={buttonClass('s')}
            disabled={!isKeyboardEnabled}
            onMouseDown={() => isKeyboardEnabled && handleKeyDown('s')}
            onMouseUp={handleKeyUp}
            onMouseLeave={handleKeyUp}
          >
            ↓
          </button>
          <div className="w-12 h-12" />
        </div>
      </div>

      {/* 키보드 안내 */}
      <div className="text-xs text-gray-500 text-center mb-4">
        {isKeyboardEnabled
          ? 'WASD / Arrow Keys to move, Space to stop'
          : '키보드 버튼을 활성화하여 조작하세요'}
      </div>

      {/* 목표 취소 버튼 - AUTONOMOUS 모드일 때만 */}
      {currentMode === ControlMode.AUTONOMOUS && robot?.goal && (
        <button
          onClick={cancelNavigation}
          className="w-full py-2 rounded-lg bg-orange-600 text-white hover:bg-orange-700 transition-colors"
        >
          🛑 Cancel Navigation
        </button>
      )}

      {/* 모드 표시 */}
      <div className="mt-2 text-xs text-center">
        <span
          className={`inline-block px-2 py-1 rounded ${
            currentMode === ControlMode.AUTONOMOUS
              ? 'bg-blue-900 text-blue-300'
              : currentMode === ControlMode.MANUAL
              ? 'bg-green-900 text-green-300'
              : 'bg-gray-700 text-gray-400'
          }`}
        >
          Mode: {currentMode.toUpperCase()}
        </span>
      </div>
    </div>
  );
};
