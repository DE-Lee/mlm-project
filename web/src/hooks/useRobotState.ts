import { useEffect, useRef } from 'react';
import ROSLIB from 'roslib';
import { getRos } from './useRosBridge';
import { useRobotStore } from '@/stores/robotStore';
import { getTopicName, TOPICS, MAP_CONFIG } from '@/config/robots';
import { Odometry, PoseWithCovarianceStamped } from '@/types/ros';

// 배터리 전압 → 퍼센트 변환 (2S LiPo 기준)
const voltageToPercent = (millivolts: number): number => {
  const voltage = millivolts / 1000;
  const minVoltage = 6.0;  // 0% (3.0V x 2 cells)
  const maxVoltage = 8.4;  // 100% (4.2V x 2 cells)
  const percent = ((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100;
  return Math.max(0, Math.min(100, percent));
};

// Quaternion to Yaw 변환
const quaternionToYaw = (q: { x: number; y: number; z: number; w: number }): number => {
  const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  return Math.atan2(siny_cosp, cosy_cosp);
};

// odom → map 변환을 위한 오프셋 (초기 위치 설정 시 계산됨)
// namespace별로 관리
const odomToMapOffset: Record<string, { x: number; y: number; theta: number } | null> = {};

// 초기 위치 설정 시 odom→map 오프셋 설정 (외부에서 호출)
export const setOdomToMapOffset = (
  namespace: string,
  mapPose: { x: number; y: number; theta: number }
) => {
  // 현재 odom 위치를 모르면 단순히 map 위치를 오프셋으로 사용
  // (odom이 0,0,0이라고 가정)
  odomToMapOffset[namespace] = {
    x: mapPose.x,
    y: mapPose.y,
    theta: mapPose.theta,
  };
  console.log(`[${namespace}] Set odom→map offset:`, odomToMapOffset[namespace]);
};

export const useRobotState = (namespace: string) => {
  const subscribersRef = useRef<ROSLIB.Topic[]>([]);

  const {
    rosConnected,
    updateRobotPose,
    updateRobotVelocity,
    updateRobotBattery,
    setRobotConnected,
  } = useRobotStore();

  useEffect(() => {
    const ros = getRos();

    if (!ros || !rosConnected) {
      return;
    }

    console.log(`[${namespace}] Subscribing to topics...`);

    // 마지막 odom 메시지의 timestamp (AMCL 메시지 필터링 기준)
    let lastOdomTimeSec = 0;
    let odomReceived = false;
    // 마지막 odom 위치 (odom→map 변환 계산용)
    let lastOdomPose = { x: 0, y: 0, theta: 0 };
    // AMCL이 최근에 업데이트 되었는지 (AMCL 우선)
    let amclActive = false;
    let lastAmclTime = 0;
    const AMCL_TIMEOUT = 2000; // 2초 동안 AMCL 없으면 odom 사용

    // Odometry 구독 (위치 + 속도)
    const odomTopic = new ROSLIB.Topic({
      ros,
      name: getTopicName(namespace, TOPICS.odom),
      messageType: 'nav_msgs/Odometry',
    });

    odomTopic.subscribe((message: unknown) => {
      const msg = message as Odometry;
      const { position, orientation } = msg.pose.pose;
      const yaw = quaternionToYaw(orientation);

      // odom timestamp 기록 (AMCL 필터링 기준)
      lastOdomTimeSec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9;
      odomReceived = true;
      lastOdomPose = { x: position.x, y: position.y, theta: yaw };

      // 속도 업데이트
      const { linear, angular } = msg.twist.twist;
      updateRobotVelocity(namespace, linear.x, angular.z);

      // AMCL이 최근에 없으면 odom으로 위치 업데이트
      const now = Date.now();
      amclActive = (now - lastAmclTime) < AMCL_TIMEOUT;

      if (!amclActive && odomToMapOffset[namespace]) {
        // odom → map 변환 적용
        const offset = odomToMapOffset[namespace]!;
        // 회전 변환: map_x = offset_x + odom_x*cos(offset_theta) - odom_y*sin(offset_theta)
        const cosTheta = Math.cos(offset.theta);
        const sinTheta = Math.sin(offset.theta);
        const mapX = offset.x + position.x * cosTheta - position.y * sinTheta;
        const mapY = offset.y + position.x * sinTheta + position.y * cosTheta;
        const mapTheta = yaw + offset.theta;

        updateRobotPose(namespace, mapX, mapY, mapTheta);
      }

      setRobotConnected(namespace, true);
    });

    subscribersRef.current.push(odomTopic);

    // AMCL Pose 구독 (더 정확한 위치 추정)
    const amclTopic = new ROSLIB.Topic({
      ros,
      name: getTopicName(namespace, TOPICS.amclPose),
      messageType: 'geometry_msgs/PoseWithCovarianceStamped',
    });

    amclTopic.subscribe((message: unknown) => {
      const msg = message as PoseWithCovarianceStamped;

      // 메시지 timestamp 확인 (초 단위)
      const msgTimeSec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9;

      // odom을 아직 안 받았거나, AMCL이 odom보다 오래되었으면 무시
      // (TRANSIENT_LOCAL로 인한 캐시된 오래된 메시지 필터링)
      if (!odomReceived || msgTimeSec < lastOdomTimeSec - 1) {
        console.log(`[${namespace}] Ignoring stale AMCL pose (msgTime: ${msgTimeSec.toFixed(1)}, odomTime: ${lastOdomTimeSec.toFixed(1)})`);
        return;
      }

      const { position, orientation } = msg.pose.pose;
      const yaw = quaternionToYaw(orientation);

      // AMCL 수신 시간 기록
      lastAmclTime = Date.now();

      // Gazebo offset 적용 (useGazeboOffset이 true일 때만)
      const offsetX = MAP_CONFIG.useGazeboOffset ? (MAP_CONFIG.gazeboOffset?.x || 0) : 0;
      const offsetY = MAP_CONFIG.useGazeboOffset ? (MAP_CONFIG.gazeboOffset?.y || 0) : 0;

      const mapX = position.x + offsetX;
      const mapY = position.y + offsetY;

      // odom → map 오프셋 계산 (AMCL이 올 때마다 갱신)
      // map_pose = offset + R(offset_theta) * odom_pose
      // 역산: offset = map_pose - R(offset_theta) * odom_pose
      // 간단히: offset_theta = map_theta - odom_theta
      const offsetTheta = yaw - lastOdomPose.theta;
      const cosTheta = Math.cos(offsetTheta);
      const sinTheta = Math.sin(offsetTheta);
      odomToMapOffset[namespace] = {
        x: mapX - (lastOdomPose.x * cosTheta - lastOdomPose.y * sinTheta),
        y: mapY - (lastOdomPose.x * sinTheta + lastOdomPose.y * cosTheta),
        theta: offsetTheta,
      };

      // AMCL이 더 정확한 위치이므로 항상 사용
      updateRobotPose(namespace, mapX, mapY, yaw);
    });

    subscribersRef.current.push(amclTopic);

    // Battery 구독 (std_msgs/UInt16 - millivolts)
    const batteryTopic = new ROSLIB.Topic({
      ros,
      name: getTopicName(namespace, TOPICS.battery),
      messageType: 'std_msgs/UInt16',
    });

    batteryTopic.subscribe((message: unknown) => {
      const msg = message as { data: number };
      const percentage = voltageToPercent(msg.data);
      updateRobotBattery(namespace, percentage);
    });

    subscribersRef.current.push(batteryTopic);

    // 연결 확인 타이머 (토픽이 오지 않으면 연결 끊김으로 처리)
    let lastMessageTime = Date.now();
    const connectionCheckInterval = setInterval(() => {
      if (Date.now() - lastMessageTime > 5000) {
        setRobotConnected(namespace, false);
      }
    }, 2000);

    // odom 메시지 수신 시 시간 업데이트 (연결 확인용)
    const originalOdomSubscribe = odomTopic.subscribe.bind(odomTopic);
    odomTopic.subscribe = (callback) => {
      return originalOdomSubscribe((msg: unknown) => {
        lastMessageTime = Date.now();
        callback(msg);
      });
    };

    return () => {
      console.log(`[${namespace}] Unsubscribing from topics...`);
      subscribersRef.current.forEach((topic) => {
        topic.unsubscribe();
      });
      subscribersRef.current = [];
      clearInterval(connectionCheckInterval);
    };
  }, [namespace, rosConnected, updateRobotPose, updateRobotVelocity, setRobotConnected]);
};
