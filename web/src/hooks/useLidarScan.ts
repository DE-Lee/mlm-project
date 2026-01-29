import { useEffect, useState, useRef } from 'react';
import ROSLIB from 'roslib';
import { getRos } from './useRosBridge';
import { useRobotStore } from '@/stores/robotStore';

export interface LaserScanMessage {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  time_increment: number;
  scan_time: number;
  range_min: number;
  range_max: number;
  ranges: number[];
  intensities: number[];
}

export interface LidarPoint {
  angle: number;  // radians
  distance: number;  // meters
  x: number;
  y: number;
}

export const useLidarScan = (namespace: string) => {
  const [scanData, setScanData] = useState<LidarPoint[]>([]);
  const [isReceiving, setIsReceiving] = useState(false);
  const [rangeMax, setRangeMax] = useState(5);
  const subscriberRef = useRef<ROSLIB.Topic | null>(null);
  const { rosConnected } = useRobotStore();

  useEffect(() => {
    const ros = getRos();
    if (!ros || !rosConnected) return;

    // Topic: /{namespace}/scan_raw
    const topicName = `/${namespace}/scan_raw`;

    subscriberRef.current = new ROSLIB.Topic({
      ros,
      name: topicName,
      messageType: 'sensor_msgs/LaserScan',
    });

    subscriberRef.current.subscribe((msg: unknown) => {
      const message = msg as LaserScanMessage;
      setIsReceiving(true);
      // 시각화용 최대 거리 제한 (5m)
      setRangeMax(Math.min(message.range_max, 5));

      // Convert ranges to cartesian points
      const points: LidarPoint[] = [];
      const { angle_min, angle_increment, ranges, range_min, range_max } = message;

      for (let i = 0; i < ranges.length; i++) {
        const distance = ranges[i];

        // Skip invalid readings
        if (distance < range_min || distance > range_max || !isFinite(distance)) {
          continue;
        }

        const angle = angle_min + i * angle_increment;

        // Convert to cartesian (robot frame: x forward, y left)
        const x = distance * Math.cos(angle);
        const y = distance * Math.sin(angle);

        points.push({ angle, distance, x, y });
      }

      setScanData(points);
    });

    console.log(`[LiDAR] Subscribed to ${topicName}`);

    return () => {
      if (subscriberRef.current) {
        subscriberRef.current.unsubscribe();
        console.log(`[LiDAR] Unsubscribed from ${topicName}`);
      }
      setIsReceiving(false);
      setScanData([]);
    };
  }, [namespace, rosConnected]);

  return { scanData, isReceiving, rangeMax };
};
