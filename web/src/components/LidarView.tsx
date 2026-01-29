import { useEffect, useRef } from 'react';
import { useLidarScan } from '@/hooks/useLidarScan';

interface LidarViewProps {
  namespace: string;
}

export const LidarView = ({ namespace }: LidarViewProps) => {
  const { scanData, isReceiving, rangeMax } = useLidarScan(namespace);
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Get actual canvas size
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    ctx.scale(dpr, dpr);

    const width = rect.width;
    const height = rect.height;
    const centerX = width / 2;
    const centerY = height / 2;
    const radius = Math.min(width, height) / 2 - 10;

    // Clear canvas
    ctx.fillStyle = '#000000';
    ctx.fillRect(0, 0, width, height);

    // Draw radar circles (1m intervals: 1m, 2m, 3m, 4m, 5m)
    ctx.strokeStyle = '#1f2937';
    ctx.lineWidth = 1;
    for (let i = 1; i <= 5; i++) {
      ctx.beginPath();
      ctx.arc(centerX, centerY, (radius * i) / 5, 0, Math.PI * 2);
      ctx.stroke();
    }

    // Draw cross lines
    ctx.beginPath();
    ctx.moveTo(centerX, centerY - radius);
    ctx.lineTo(centerX, centerY + radius);
    ctx.moveTo(centerX - radius, centerY);
    ctx.lineTo(centerX + radius, centerY);
    ctx.stroke();

    // Draw diagonal lines
    ctx.strokeStyle = '#111827';
    ctx.beginPath();
    const diag = radius * Math.SQRT1_2;
    ctx.moveTo(centerX - diag, centerY - diag);
    ctx.lineTo(centerX + diag, centerY + diag);
    ctx.moveTo(centerX + diag, centerY - diag);
    ctx.lineTo(centerX - diag, centerY + diag);
    ctx.stroke();

    // Draw robot indicator (triangle pointing up = forward)
    ctx.fillStyle = '#3b82f6';
    ctx.beginPath();
    ctx.moveTo(centerX, centerY - 8);
    ctx.lineTo(centerX - 5, centerY + 5);
    ctx.lineTo(centerX + 5, centerY + 5);
    ctx.closePath();
    ctx.fill();

    // Draw LiDAR points
    if (scanData.length > 0) {
      const scale = radius / rangeMax;

      // Draw points with gradient based on distance
      scanData.forEach((point) => {
        // Robot frame: x forward (up), y left (left)
        // Canvas: x right, y down
        // Transform: canvas_x = center - y * scale, canvas_y = center - x * scale
        const canvasX = centerX - point.y * scale;
        const canvasY = centerY - point.x * scale;

        // Color based on distance (green=close, yellow=medium, red=far)
        const ratio = point.distance / rangeMax;
        let color;
        if (ratio < 0.3) {
          color = '#ef4444'; // Red - close obstacle
        } else if (ratio < 0.6) {
          color = '#f59e0b'; // Yellow - medium distance
        } else {
          color = '#22c55e'; // Green - far
        }

        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.arc(canvasX, canvasY, 2, 0, Math.PI * 2);
        ctx.fill();
      });
    }

    // Draw range labels (1m, 2m, 3m, 4m)
    ctx.fillStyle = '#4b5563';
    ctx.font = '10px monospace';
    ctx.textAlign = 'center';
    for (let i = 1; i <= 4; i++) {
      ctx.fillText(`${i}m`, centerX, centerY - (radius * i) / 5 - 3);
    }

  }, [scanData, rangeMax]);

  return (
    <div className="bg-[rgb(17,24,39)] rounded-xl overflow-hidden h-full flex flex-col">
      {/* Header */}
      <div className="px-3 py-2 flex items-center justify-between border-b border-gray-700">
        <div className="flex items-center gap-2">
          <svg className="w-4 h-4 text-green-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 19v-6a2 2 0 00-2-2H5a2 2 0 00-2 2v6a2 2 0 002 2h2a2 2 0 002-2zm0 0V9a2 2 0 012-2h2a2 2 0 012 2v10m-6 0a2 2 0 002 2h2a2 2 0 002-2m0 0V5a2 2 0 012-2h2a2 2 0 012 2v14a2 2 0 01-2 2h-2a2 2 0 01-2-2z" />
          </svg>
          <span className="text-xs text-gray-300 font-medium">LiDAR</span>
        </div>
        <div className="flex items-center gap-2">
          <div className={`w-2 h-2 rounded-full ${isReceiving ? 'bg-green-500' : 'bg-gray-500'}`} />
          {isReceiving && (
            <span className="text-xs text-gray-400">{scanData.length} pts</span>
          )}
        </div>
      </div>

      {/* Radar Display */}
      <div className="flex-1 flex items-center justify-center p-2 min-h-[180px]">
        <canvas
          ref={canvasRef}
          className="w-full h-full"
          style={{ aspectRatio: '1/1', maxHeight: '100%' }}
        />
      </div>
    </div>
  );
};
