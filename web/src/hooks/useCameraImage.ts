import { useEffect, useState, useRef } from 'react';
import ROSLIB from 'roslib';
import { getRos } from './useRosBridge';
import { useRobotStore } from '@/stores/robotStore';

interface CompressedImageMessage {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  format: string;
  data: string; // base64 encoded
}

export const useCameraImage = (namespace: string) => {
  const [imageData, setImageData] = useState<string | null>(null);
  const [isReceiving, setIsReceiving] = useState(false);
  const [fps, setFps] = useState(0);
  const subscriberRef = useRef<ROSLIB.Topic | null>(null);
  const frameCountRef = useRef(0);
  const lastTimeRef = useRef(Date.now());
  const { rosConnected } = useRobotStore();

  useEffect(() => {
    const ros = getRos();
    if (!ros || !rosConnected) return;

    // Topic: /{namespace}/my_color_cam/image_compressed
    const topicName = `/${namespace}/my_color_cam/image_compressed`;

    subscriberRef.current = new ROSLIB.Topic({
      ros,
      name: topicName,
      messageType: 'sensor_msgs/CompressedImage',
    });

    subscriberRef.current.subscribe((msg: unknown) => {
      const message = msg as CompressedImageMessage;
      setIsReceiving(true);

      // Convert base64 data to image URL
      const imageUrl = `data:image/${message.format};base64,${message.data}`;
      setImageData(imageUrl);

      // Calculate FPS
      frameCountRef.current++;
      const now = Date.now();
      const elapsed = now - lastTimeRef.current;
      if (elapsed >= 1000) {
        setFps(Math.round((frameCountRef.current * 1000) / elapsed));
        frameCountRef.current = 0;
        lastTimeRef.current = now;
      }
    });

    console.log(`[Camera] Subscribed to ${topicName}`);

    return () => {
      if (subscriberRef.current) {
        subscriberRef.current.unsubscribe();
        console.log(`[Camera] Unsubscribed from ${topicName}`);
      }
      setIsReceiving(false);
      setImageData(null);
    };
  }, [namespace, rosConnected]);

  return { imageData, isReceiving, fps };
};
