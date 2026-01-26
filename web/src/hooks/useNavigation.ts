import { useCallback, useRef, useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { getRos } from './useRosBridge';
import { useRobotStore } from '@/stores/robotStore';
import { useControlModeStore, ControlMode } from '@/stores/controlModeStore';

// Yaw to Quaternion 변환
const yawToQuaternion = (yaw: number) => ({
  x: 0,
  y: 0,
  z: Math.sin(yaw / 2),
  w: Math.cos(yaw / 2),
});

/**
 * Nav2 Action-based Navigation Hook
 * navigate_to_pose action을 사용하여 목표점 설정 및 취소
 */
export const useNavigation = (namespace: string) => {
  const actionClientRef = useRef<ROSLIB.ActionClient | null>(null);
  const currentGoalRef = useRef<ROSLIB.Goal | null>(null);
  const [isActionServerReady, setIsActionServerReady] = useState(false);

  const { rosConnected, updateRobotGoal, updateRobotStatus } = useRobotStore();
  const { setMode } = useControlModeStore();

  // Action Client 초기화 및 서버 가용성 체크
  useEffect(() => {
    const ros = getRos();
    if (!ros || !rosConnected) {
      setIsActionServerReady(false);
      return;
    }

    if (!actionClientRef.current) {
      actionClientRef.current = new ROSLIB.ActionClient({
        ros,
        serverName: `/${namespace}/navigate_to_pose`,
        actionName: 'nav2_msgs/action/NavigateToPose',
      });
      console.log(`[${namespace}] NavigateToPose action client created`);
    }

    // Action server 가용성 체크 - status 토픽 구독으로 확인
    const statusTopic = new ROSLIB.Topic({
      ros,
      name: `/${namespace}/navigate_to_pose/_action/status`,
      messageType: 'action_msgs/msg/GoalStatusArray',
    });

    let receivedStatus = false;
    const statusCallback = () => {
      if (!receivedStatus) {
        receivedStatus = true;
        setIsActionServerReady(true);
        console.log(`[${namespace}] Navigation action server is ready ✓`);
      }
    };
    statusTopic.subscribe(statusCallback);

    // 3초 후에도 상태를 받지 못하면 bt_navigator 토픽으로 재확인
    const fallbackTimeout = setTimeout(() => {
      if (!receivedStatus) {
        // bt_navigator가 active 상태면 action server도 준비된 것으로 간주
        const btTopic = new ROSLIB.Topic({
          ros,
          name: `/${namespace}/bt_navigator/transition_event`,
          messageType: 'lifecycle_msgs/msg/TransitionEvent',
        });
        btTopic.subscribe(() => {
          if (!receivedStatus) {
            receivedStatus = true;
            setIsActionServerReady(true);
            console.log(`[${namespace}] Navigation action server is ready (via bt_navigator) ✓`);
          }
          btTopic.unsubscribe();
        });

        // 추가 3초 후에도 없으면 그냥 활성화 (이미 bt_navigator 노드가 존재하므로)
        setTimeout(() => {
          if (!receivedStatus) {
            setIsActionServerReady(true);
            console.log(`[${namespace}] Navigation action server assumed ready`);
          }
        }, 3000);
      }
    }, 3000);

    return () => {
      statusTopic.unsubscribe();
      clearTimeout(fallbackTimeout);
      if (currentGoalRef.current) {
        currentGoalRef.current.cancel();
        currentGoalRef.current = null;
      }
    };
  }, [namespace, rosConnected, isActionServerReady]);

  /**
   * 목표점으로 이동 (Action 사용)
   */
  const navigateToPose = useCallback(
    (x: number, y: number, theta: number = 0) => {
      const actionClient = actionClientRef.current;

      if (!actionClient || !rosConnected) {
        console.warn(`[${namespace}] Action client not available`);
        return false;
      }

      if (!isActionServerReady) {
        console.warn(`[${namespace}] Navigation action server not ready yet. Please wait...`);
        return false;
      }

      // 이전 목표가 있으면 취소
      if (currentGoalRef.current) {
        console.log(`[${namespace}] Cancelling previous goal...`);
        currentGoalRef.current.cancel();
      }

      // 새 목표 생성
      const goal = new ROSLIB.Goal({
        actionClient,
        goalMessage: {
          pose: {
            header: {
              stamp: {
                sec: Math.floor(Date.now() / 1000),
                nanosec: (Date.now() % 1000) * 1e6,
              },
              frame_id: 'map',
            },
            pose: {
              position: { x, y, z: 0 },
              orientation: yawToQuaternion(theta),
            },
          },
        },
      });

      // 피드백 핸들러
      goal.on('feedback', (feedback: any) => {
        if (feedback.distance_remaining !== undefined) {
          console.log(
            `[${namespace}] Distance remaining: ${feedback.distance_remaining.toFixed(2)}m`
          );
        }
        updateRobotStatus(namespace, 'navigating');
      });

      // 결과 핸들러
      goal.on('result', (result: any) => {
        console.log(`[${namespace}] Navigation result:`, result);
        updateRobotGoal(namespace, null);
        updateRobotStatus(namespace, 'idle');
        setMode(namespace, ControlMode.IDLE);
      });

      // 목표 전송
      goal.send();
      currentGoalRef.current = goal;

      // 상태 업데이트
      updateRobotGoal(namespace, { x, y });
      updateRobotStatus(namespace, 'navigating');
      setMode(namespace, ControlMode.AUTONOMOUS);

      console.log(
        `[${namespace}] Navigate to: (${x.toFixed(2)}, ${y.toFixed(2)}, θ=${(
          (theta * 180) /
          Math.PI
        ).toFixed(1)}°)`
      );

      return true;
    },
    [namespace, rosConnected, isActionServerReady, updateRobotGoal, updateRobotStatus, setMode]
  );

  /**
   * Navigation 취소
   */
  const cancelNavigation = useCallback(() => {
    if (currentGoalRef.current) {
      console.log(`[${namespace}] Cancelling navigation...`);
      currentGoalRef.current.cancel();
      currentGoalRef.current = null;

      updateRobotGoal(namespace, null);
      updateRobotStatus(namespace, 'idle');
      setMode(namespace, ControlMode.IDLE);

      return true;
    }
    return false;
  }, [namespace, updateRobotGoal, updateRobotStatus, setMode]);

  /**
   * Navigation 진행 중 여부
   */
  const isNavigating = useCallback(() => {
    return currentGoalRef.current !== null;
  }, []);

  return {
    navigateToPose,
    cancelNavigation,
    isNavigating,
    isActionServerReady,
  };
};

/**
 * 수동 제어 Hook (기존 유지, Control Mode 통합)
 */
export const useManualControl = (namespace: string) => {
  const publisherRef = useRef<ROSLIB.Topic | null>(null);
  const { rosConnected } = useRobotStore();
  const { setMode } = useControlModeStore();

  const publishVelocity = useCallback(
    (linear: number, angular: number) => {
      const ros = getRos();

      if (!ros || !rosConnected) {
        return;
      }

      if (!publisherRef.current) {
        publisherRef.current = new ROSLIB.Topic({
          ros,
          name: `/${namespace}/controller/cmd_vel`,
          messageType: 'geometry_msgs/Twist',
        });
      }

      const twistMessage = new ROSLIB.Message({
        linear: { x: linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular },
      });

      publisherRef.current.publish(twistMessage);
    },
    [namespace, rosConnected]
  );

  const stop = useCallback(() => {
    publishVelocity(0, 0);
    setMode(namespace, ControlMode.IDLE);
  }, [publishVelocity, namespace, setMode]);

  return {
    publishVelocity,
    stop,
  };
};
