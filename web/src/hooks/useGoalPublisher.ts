import { useCallback, useRef } from 'react';
import ROSLIB from 'roslib';
import { getRos } from './useRosBridge';
import { useRobotStore } from '@/stores/robotStore';
import { getTopicName, TOPICS } from '@/config/robots';

// Yaw to Quaternion 변환
const yawToQuaternion = (yaw: number) => ({
  x: 0,
  y: 0,
  z: Math.sin(yaw / 2),
  w: Math.cos(yaw / 2),
});

export const useGoalPublisher = (namespace: string) => {
  const publisherRef = useRef<ROSLIB.Topic | null>(null);
  const { rosConnected, updateRobotGoal } = useRobotStore();

  // Goal Pose 발행
  const publishGoal = useCallback(
    (x: number, y: number, theta: number = 0) => {
      const ros = getRos();
      
      if (!ros || !rosConnected) {
        console.warn('[Goal] ROS not connected');
        return false;
      }

      // Publisher 생성 (재사용)
      if (!publisherRef.current) {
        publisherRef.current = new ROSLIB.Topic({
          ros,
          name: getTopicName(namespace, TOPICS.goalPose),
          messageType: 'geometry_msgs/PoseStamped',
        });
      }

      const goalMessage = new ROSLIB.Message({
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
      });

      console.log(`[${namespace}] Publishing goal: (${x.toFixed(2)}, ${y.toFixed(2)})`);
      publisherRef.current.publish(goalMessage);
      
      // 스토어 업데이트
      updateRobotGoal(namespace, { x, y });
      
      return true;
    },
    [namespace, rosConnected, updateRobotGoal]
  );

  // 목표 취소
  const cancelGoal = useCallback(() => {
    // Nav2 cancel action 호출 필요 (간단히 상태만 업데이트)
    updateRobotGoal(namespace, null);
    console.log(`[${namespace}] Goal cancelled`);
  }, [namespace, updateRobotGoal]);

  return {
    publishGoal,
    cancelGoal,
  };
};

// 수동 제어 (cmd_vel 발행)
export const useManualControl = (namespace: string) => {
  const publisherRef = useRef<ROSLIB.Topic | null>(null);
  const { rosConnected } = useRobotStore();

  const publishVelocity = useCallback(
    (linear: number, angular: number) => {
      const ros = getRos();
      
      if (!ros || !rosConnected) {
        return;
      }

      if (!publisherRef.current) {
        publisherRef.current = new ROSLIB.Topic({
          ros,
          name: getTopicName(namespace, TOPICS.cmdVel),
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
  }, [publishVelocity]);

  return {
    publishVelocity,
    stop,
  };
};
