import { useCallback, useRef } from 'react';
import ROSLIB from 'roslib';
import { getRos } from './useRosBridge';
import { useRobotStore } from '@/stores/robotStore';
import { getTopicName, TOPICS, ROBOTS } from '@/config/robots';

export const useEmergencyStop = () => {
  const cmdVelPublishersRef = useRef<Map<string, ROSLIB.Topic>>(new Map());
  const { rosConnected, updateRobotGoal, updateRobotStatus } = useRobotStore();

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

  // Nav2 네비게이션 취소 (서비스 호출 방식)
  const cancelNavigation = useCallback(
    (namespace: string) => {
      const ros = getRos();
      if (!ros || !rosConnected) return;

      // navigate_to_pose action cancel 서비스
      // ROS2 Humble에서는 CancelGoal이 action_msgs/srv/CancelGoal 서비스 타입
      const actionName = namespace
        ? `/${namespace}/navigate_to_pose`
        : '/navigate_to_pose';

      const cancelService = new ROSLIB.Service({
        ros,
        name: `${actionName}/_action/cancel_goal`,
        serviceType: 'action_msgs/srv/CancelGoal',
      });

      // 빈 goal_info로 모든 목표 취소
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
          console.log(`[${namespace}] Navigation cancel response:`, response);
        },
        (error) => {
          console.warn(`[${namespace}] Navigation cancel failed:`, error);
        }
      );

      console.log(`[${namespace}] Navigation cancel requested`);

      // 상태 업데이트
      updateRobotGoal(namespace, null);
      updateRobotStatus(namespace, 'idle');
    },
    [rosConnected, updateRobotGoal, updateRobotStatus]
  );

  // 긴급 정지 (모든 로봇)
  const emergencyStopAll = useCallback(() => {
    if (!rosConnected) {
      console.warn('[EmergencyStop] ROS not connected');
      return;
    }

    console.log('[EmergencyStop] Stopping all robots!');

    ROBOTS.forEach((robot) => {
      stopRobot(robot.namespace);
      cancelNavigation(robot.namespace);
    });
  }, [rosConnected, stopRobot, cancelNavigation]);

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
