import { useEffect, useRef, useCallback } from 'react';
import ROSLIB from 'roslib';
import { useRobotStore } from '@/stores/robotStore';
import { ROS_CONFIG } from '@/config/robots';

// 싱글톤 ROS 연결
let rosInstance: ROSLIB.Ros | null = null;

export const getRos = (): ROSLIB.Ros | null => rosInstance;

export const useRosBridge = () => {
  const reconnectAttempts = useRef(0);
  const reconnectTimeout = useRef<NodeJS.Timeout | null>(null);
  
  const { rosConnected, setRosConnected } = useRobotStore();

  const connect = useCallback(() => {
    if (rosInstance?.isConnected) {
      return;
    }

    console.log('[ROS] Connecting to:', ROS_CONFIG.url);
    
    rosInstance = new ROSLIB.Ros({
      url: ROS_CONFIG.url,
    });

    rosInstance.on('connection', () => {
      console.log('[ROS] Connected!');
      setRosConnected(true);
      reconnectAttempts.current = 0;
    });

    rosInstance.on('error', (error) => {
      console.error('[ROS] Error:', error);
    });

    rosInstance.on('close', () => {
      console.log('[ROS] Connection closed');
      setRosConnected(false);
      
      // 재연결 시도
      if (reconnectAttempts.current < ROS_CONFIG.maxReconnectAttempts) {
        reconnectAttempts.current++;
        console.log(`[ROS] Reconnecting... (${reconnectAttempts.current}/${ROS_CONFIG.maxReconnectAttempts})`);
        
        reconnectTimeout.current = setTimeout(() => {
          connect();
        }, ROS_CONFIG.reconnectInterval);
      }
    });
  }, [setRosConnected]);

  const disconnect = useCallback(() => {
    if (reconnectTimeout.current) {
      clearTimeout(reconnectTimeout.current);
    }
    if (rosInstance) {
      rosInstance.close();
      rosInstance = null;
    }
    setRosConnected(false);
  }, [setRosConnected]);

  useEffect(() => {
    connect();
    
    return () => {
      disconnect();
    };
  }, [connect, disconnect]);

  return {
    ros: rosInstance,
    connected: rosConnected,
    reconnect: connect,
    disconnect,
  };
};
