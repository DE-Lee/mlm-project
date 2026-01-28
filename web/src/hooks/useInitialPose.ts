import { useCallback, useRef } from 'react';
import ROSLIB from 'roslib';
import { getRos } from './useRosBridge';
import { useRobotStore } from '@/stores/robotStore';
import { getTopicName, TOPICS } from '@/config/robots';
import { setOdomToMapOffset } from './useRobotState';

// Yaw to Quaternion 변환
const yawToQuaternion = (yaw: number) => ({
  x: 0,
  y: 0,
  z: Math.sin(yaw / 2),
  w: Math.cos(yaw / 2),
});

export const useInitialPose = (namespace: string) => {
  const publisherRef = useRef<ROSLIB.Topic | null>(null);
  const lastNamespaceRef = useRef<string>('');
  const { rosConnected, updateRobotPose } = useRobotStore();

  // Initial Pose 발행 (AMCL에 로봇 초기 위치 알림)
  const publishInitialPose = useCallback(
    (x: number, y: number, theta: number = 0) => {
      const ros = getRos();

      if (!ros || !rosConnected) {
        console.warn('[InitialPose] ROS not connected');
        return false;
      }

      // Publisher 생성 (namespace 변경 시 재생성)
      if (!publisherRef.current || lastNamespaceRef.current !== namespace) {
        publisherRef.current = new ROSLIB.Topic({
          ros,
          name: getTopicName(namespace, TOPICS.initialPose),
          messageType: 'geometry_msgs/PoseWithCovarianceStamped',
        });
        lastNamespaceRef.current = namespace;
        console.log(`[InitialPose] Created publisher for ${namespace}`);
      }

      // 공분산 행렬 (6x6 = 36개, 대각선만 설정)
      // [x, y, z, roll, pitch, yaw] 순서
      const covariance = new Array(36).fill(0);
      covariance[0] = 0.25;  // x variance
      covariance[7] = 0.25;  // y variance
      covariance[35] = 0.06; // yaw variance

      const initialPoseMessage = new ROSLIB.Message({
        header: {
          stamp: {
            sec: Math.floor(Date.now() / 1000),
            nanosec: (Date.now() % 1000) * 1e6,
          },
          frame_id: 'map',
        },
        pose: {
          pose: {
            position: { x, y, z: 0 },
            orientation: yawToQuaternion(theta),
          },
          covariance,
        },
      });

      console.log(`[${namespace}] Publishing initial pose: (${x.toFixed(2)}, ${y.toFixed(2)}, θ=${(theta * 180 / Math.PI).toFixed(1)}°)`);
      publisherRef.current.publish(initialPoseMessage);

      // odom→map 오프셋 설정 (Nav2 없이도 odom으로 위치 표시 가능하게)
      setOdomToMapOffset(namespace, { x, y, theta });

      // 로컬 상태도 업데이트
      updateRobotPose(namespace, x, y, theta);

      return true;
    },
    [namespace, rosConnected, updateRobotPose]
  );

  return {
    publishInitialPose,
  };
};
