import { useCallback, useRef } from 'react';
import ROSLIB from 'roslib';
import { getRos } from './useRosBridge';
import { useRobotStore } from '@/stores/robotStore';
import { useControlModeStore, ControlMode } from '@/stores/controlModeStore';
import { getTopicName, TOPICS, ROBOTS } from '@/config/robots';

export const useEmergencyStop = () => {
  const cmdVelPublishersRef = useRef<Map<string, ROSLIB.Topic>>(new Map());
  const { rosConnected, updateRobotGoal, updateRobotStatus } = useRobotStore();
  const { setMode, setAllIdle } = useControlModeStore();

  // cmd_vel 퍼블리셔 가져오기 (lazy 생성)
  const getCmdVelPublisher = useCallback(
    (namespace: string) => {
      const ros = getRos();
      if (!ros) return null;

      let publisher = cmdVelPublishersRef.current.get(namespace);
      if (!publisher) {
        publisher = new ROSLIB.Topic({
          ros,
          name: getTopicName(namespace, TOPICS.cmdVel),
          messageType: 'geometry_msgs/Twist',
        });
        cmdVelPublishersRef.current.set(namespace, publisher);
      }
      return publisher;
    },
    []
  );

  // 단일 로봇 정지
  const stopRobot = useCallback(
    (namespace: string) => {
      const publisher = getCmdVelPublisher(namespace);
      if (!publisher) return;

      const stopMessage = new ROSLIB.Message({
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      });

      // 여러 번 발행하여 확실히 정지
      for (let i = 0; i < 3; i++) {
        publisher.publish(stopMessage);
      }

      console.log(`[${namespace}] Emergency stop - cmd_vel = 0`);
    },
    [getCmdVelPublisher]
  );

  // Nav2 네비게이션 취소
  // NOTE: 서비스 직접 호출 방식 사용 (useNavigation hook 재사용 불가 - hook rules of react)
  // navigate_to_pose action의 cancel_goal 서비스를 직접 호출
  const cancelNavigation = useCallback(
    (namespace: string) => {
      const ros = getRos();
      if (!ros || !rosConnected) return;

      const actionName = `/${namespace}/navigate_to_pose`;

      const cancelService = new ROSLIB.Service({
        ros,
        name: `${actionName}/_action/cancel_goal`,
        serviceType: 'action_msgs/srv/CancelGoal',
      });

      // UUID [0,0,...,0]으로 해당 action의 모든 goal 취소
      const request = new ROSLIB.ServiceRequest({
        goal_info: {
          goal_id: {
            uuid: new Array(16).fill(0),
          },
          stamp: {
            sec: 0,
            nanosec: 0,
          },
        },
      });

      cancelService.callService(
        request,
        (response) => {
          console.log(`[${namespace}] Navigation cancelled ✓`, response);
        },
        (error) => {
          console.warn(`[${namespace}] Navigation cancel failed:`, error);
        }
      );

      console.log(`[${namespace}] Navigation cancel requested`);

      // 상태 업데이트
      updateRobotGoal(namespace, null);
      updateRobotStatus(namespace, 'idle');
      setMode(namespace, ControlMode.IDLE);
    },
    [rosConnected, updateRobotGoal, updateRobotStatus, setMode]
  );

  // 긴급 정지 (모든 로봇)
  const emergencyStopAll = useCallback(() => {
    if (!rosConnected) {
      console.warn('[EmergencyStop] ROS not connected');
      return;
    }

    console.log('[EmergencyStop] 🛑 Stopping all robots!');

    ROBOTS.forEach((robot) => {
      stopRobot(robot.namespace);
      cancelNavigation(robot.namespace);
    });

    // 모든 로봇 IDLE 모드로
    setAllIdle();
  }, [rosConnected, stopRobot, cancelNavigation, setAllIdle]);

  // 특정 로봇 긴급 정지
  const emergencyStop = useCallback(
    (namespace: string) => {
      if (!rosConnected) {
        console.warn('[EmergencyStop] ROS not connected');
        return;
      }

      console.log(`[EmergencyStop] Stopping robot: ${namespace}`);
      stopRobot(namespace);
      cancelNavigation(namespace);
    },
    [rosConnected, stopRobot, cancelNavigation]
  );

  return {
    emergencyStop,
    emergencyStopAll,
    stopRobot,
    cancelNavigation,
  };
};
