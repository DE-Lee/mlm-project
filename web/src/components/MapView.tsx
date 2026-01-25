import { useRef, useEffect, useState, useCallback } from 'react';
import { useRobotStore } from '@/stores/robotStore';
import { useNavigation } from '@/hooks/useNavigation';
import { useInitialPose } from '@/hooks/useInitialPose';
import { useMapInteraction } from '@/hooks/useMapInteraction';
import { ROBOTS, MAP_CONFIG } from '@/config/robots';

interface DragState {
  startPos: { x: number; y: number } | null;  // 월드 좌표
  currentPos: { x: number; y: number } | null;  // 월드 좌표
  isDragging: boolean;
  mode: 'none' | 'goal' | 'initialPose';  // 드래그 시작한 모드
  robot: string | null;  // 어느 로봇을 위한 드래그인지
}

interface MapViewProps {
  width?: number;
  height?: number;
}

export const MapView = ({ width = 600, height = 600 }: MapViewProps) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [mapImage, setMapImage] = useState<HTMLImageElement | null>(null);
  const [hoverPos, setHoverPos] = useState<{ x: number; y: number } | null>(null);
  const [dragState, setDragState] = useState<DragState>({
    startPos: null,
    currentPos: null,
    isDragging: false,
    mode: 'none',
    robot: null,
  });

  const { robots } = useRobotStore();
  const { mapMode, activeRobot, setMapMode } = useMapInteraction();

  // 활성 로봇에 대한 navigation
  const { navigateToPose } = useNavigation(activeRobot || 'robot1');
  const { publishInitialPose } = useInitialPose(activeRobot || 'robot1');

  // 맵 이미지 로드
  useEffect(() => {
    const img = new Image();
    img.onload = () => {
      setMapImage(img);
      console.log('[Map] Image loaded:', img.width, 'x', img.height);
    };
    img.onerror = () => {
      console.warn('[Map] Failed to load map image, using placeholder');
      // 맵 이미지 없으면 빈 캔버스 사용
    };
    img.src = MAP_CONFIG.image;
  }, []);

  // 픽셀 좌표 → 월드 좌표 변환
  const pixelToWorld = useCallback(
    (px: number, py: number) => {
      const scale = Math.min(
        width / MAP_CONFIG.width,
        height / MAP_CONFIG.height
      );
      
      // 캔버스 중심으로부터의 오프셋
      const offsetX = (width - MAP_CONFIG.width * scale) / 2;
      const offsetY = (height - MAP_CONFIG.height * scale) / 2;
      
      // 픽셀 → 맵 픽셀
      const mapPx = (px - offsetX) / scale;
      const mapPy = (py - offsetY) / scale;
      
      // 맵 픽셀 → 월드 좌표 (Y축 반전)
      const worldX = MAP_CONFIG.origin.x + mapPx * MAP_CONFIG.resolution;
      const worldY = MAP_CONFIG.origin.y + (MAP_CONFIG.height - mapPy) * MAP_CONFIG.resolution;
      
      return { x: worldX, y: worldY };
    },
    [width, height]
  );

  // 월드 좌표 → 픽셀 좌표 변환
  const worldToPixel = useCallback(
    (wx: number, wy: number) => {
      const scale = Math.min(
        width / MAP_CONFIG.width,
        height / MAP_CONFIG.height
      );

      const offsetX = (width - MAP_CONFIG.width * scale) / 2;
      const offsetY = (height - MAP_CONFIG.height * scale) / 2;

      // 월드 → 맵 픽셀
      const mapPx = (wx - MAP_CONFIG.origin.x) / MAP_CONFIG.resolution;
      const mapPy = MAP_CONFIG.height - (wy - MAP_CONFIG.origin.y) / MAP_CONFIG.resolution;

      // 맵 픽셀 → 캔버스 픽셀
      const px = offsetX + mapPx * scale;
      const py = offsetY + mapPy * scale;

      return { x: px, y: py };
    },
    [width, height]
  );

  // 마우스 이벤트에서 캔버스 좌표 얻기 (CSS 크기와 캔버스 크기 차이 보정)
  const getCanvasCoords = useCallback(
    (e: React.MouseEvent<HTMLCanvasElement>) => {
      const canvas = canvasRef.current;
      if (!canvas) return null;

      const rect = canvas.getBoundingClientRect();
      // CSS 크기와 캔버스 실제 크기의 비율 계산
      const scaleX = canvas.width / rect.width;
      const scaleY = canvas.height / rect.height;

      // 마우스 좌표를 캔버스 좌표로 변환
      const canvasX = (e.clientX - rect.left) * scaleX;
      const canvasY = (e.clientY - rect.top) * scaleY;

      return { x: canvasX, y: canvasY };
    },
    []
  );

  // 캔버스 렌더링
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // 배경
    ctx.fillStyle = '#1f2937';
    ctx.fillRect(0, 0, width, height);

    // 맵 이미지 그리기
    if (mapImage) {
      const scale = Math.min(
        width / MAP_CONFIG.width,
        height / MAP_CONFIG.height
      );
      const offsetX = (width - MAP_CONFIG.width * scale) / 2;
      const offsetY = (height - MAP_CONFIG.height * scale) / 2;
      
      ctx.drawImage(
        mapImage,
        offsetX,
        offsetY,
        MAP_CONFIG.width * scale,
        MAP_CONFIG.height * scale
      );
    } else {
      // 맵 없을 때 그리드
      ctx.strokeStyle = '#374151';
      ctx.lineWidth = 1;
      const gridSize = 50;
      for (let x = 0; x <= width; x += gridSize) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, height);
        ctx.stroke();
      }
      for (let y = 0; y <= height; y += gridSize) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
        ctx.stroke();
      }
    }

    // 로봇 그리기
    ROBOTS.forEach((robotConfig) => {
      const robot = robots[robotConfig.namespace];
      if (!robot || !robot.pose) return;  // pose가 null이면 아직 위치를 모르므로 그리지 않음

      const pos = worldToPixel(robot.pose.x, robot.pose.y);
      const robotRadius = 15;

      // 로봇 몸체
      ctx.beginPath();
      ctx.fillStyle = robotConfig.color;
      ctx.arc(pos.x, pos.y, robotRadius, 0, Math.PI * 2);
      ctx.fill();

      // 활성화된 로봇 강조 (맵 상호작용 중일 때)
      if (activeRobot === robotConfig.namespace) {
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.stroke();
      }

      // 방향 표시 (화살표)
      // ROS: theta=0은 +X방향(오른쪽), 캔버스: Y축 반전
      const arrowLength = robotRadius + 10;
      const arrowX = pos.x + Math.cos(robot.pose.theta) * arrowLength;
      const arrowY = pos.y - Math.sin(robot.pose.theta) * arrowLength;
      
      ctx.beginPath();
      ctx.strokeStyle = robotConfig.color;
      ctx.lineWidth = 3;
      ctx.moveTo(pos.x, pos.y);
      ctx.lineTo(arrowX, arrowY);
      ctx.stroke();

      // 로봇 이름 (로봇 색상으로 표시)
      ctx.fillStyle = robotConfig.color;
      ctx.font = 'bold 13px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(robotConfig.name, pos.x, pos.y + robotRadius + 18);

      // 목표점 그리기
      if (robot.goal) {
        const goalPos = worldToPixel(robot.goal.x, robot.goal.y);
        
        // 목표점 마커
        ctx.beginPath();
        ctx.strokeStyle = robotConfig.color;
        ctx.lineWidth = 2;
        ctx.setLineDash([5, 5]);
        
        // X 표시
        const markerSize = 10;
        ctx.moveTo(goalPos.x - markerSize, goalPos.y - markerSize);
        ctx.lineTo(goalPos.x + markerSize, goalPos.y + markerSize);
        ctx.moveTo(goalPos.x + markerSize, goalPos.y - markerSize);
        ctx.lineTo(goalPos.x - markerSize, goalPos.y + markerSize);
        ctx.stroke();
        ctx.setLineDash([]);

        // 로봇 → 목표 라인
        ctx.beginPath();
        ctx.strokeStyle = robotConfig.color + '80';
        ctx.setLineDash([5, 5]);
        ctx.moveTo(pos.x, pos.y);
        ctx.lineTo(goalPos.x, goalPos.y);
        ctx.stroke();
        ctx.setLineDash([]);
      }
    });

    // 목표점 설정 모드
    if (mapMode === 'goal') {
      if (dragState.startPos && dragState.mode === 'goal') {
        const startPixel = worldToPixel(dragState.startPos.x, dragState.startPos.y);

        // 시작점 원
        ctx.beginPath();
        ctx.fillStyle = '#22c55e';
        ctx.arc(startPixel.x, startPixel.y, 15, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.stroke();

        // 드래그 중이면 방향 화살표 표시
        if (dragState.isDragging && dragState.currentPos) {
          const endPixel = worldToPixel(dragState.currentPos.x, dragState.currentPos.y);

          // 방향 라인
          ctx.beginPath();
          ctx.strokeStyle = '#22c55e';
          ctx.lineWidth = 3;
          ctx.moveTo(startPixel.x, startPixel.y);
          ctx.lineTo(endPixel.x, endPixel.y);
          ctx.stroke();

          // 화살표 머리
          const angle = Math.atan2(endPixel.y - startPixel.y, endPixel.x - startPixel.x);
          const arrowSize = 15;
          ctx.beginPath();
          ctx.moveTo(endPixel.x, endPixel.y);
          ctx.lineTo(
            endPixel.x - arrowSize * Math.cos(angle - Math.PI / 6),
            endPixel.y - arrowSize * Math.sin(angle - Math.PI / 6)
          );
          ctx.moveTo(endPixel.x, endPixel.y);
          ctx.lineTo(
            endPixel.x - arrowSize * Math.cos(angle + Math.PI / 6),
            endPixel.y - arrowSize * Math.sin(angle + Math.PI / 6)
          );
          ctx.stroke();

          // 각도 표시
          const theta = Math.atan2(
            dragState.currentPos.y - dragState.startPos.y,
            dragState.currentPos.x - dragState.startPos.x
          );
          ctx.fillStyle = '#22c55e';
          ctx.font = '12px sans-serif';
          ctx.textAlign = 'center';
          ctx.fillText(
            `(${dragState.startPos.x.toFixed(2)}, ${dragState.startPos.y.toFixed(2)}) θ=${(theta * 180 / Math.PI).toFixed(1)}°`,
            startPixel.x,
            startPixel.y - 25
          );
        } else {
          // 클릭만 한 상태 - 드래그 안내
          ctx.fillStyle = '#22c55e';
          ctx.font = '12px sans-serif';
          ctx.textAlign = 'center';
          ctx.fillText('드래그하여 방향 설정', startPixel.x, startPixel.y - 25);
        }
      } else if (hoverPos) {
        // 호버 상태 - 클릭 안내
        const pos = worldToPixel(hoverPos.x, hoverPos.y);
        ctx.beginPath();
        ctx.strokeStyle = '#22c55e';
        ctx.lineWidth = 2;
        ctx.setLineDash([5, 5]);
        ctx.arc(pos.x, pos.y, 20, 0, Math.PI * 2);
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.fillStyle = '#22c55e';
        ctx.font = '12px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText(
          `(${hoverPos.x.toFixed(2)}, ${hoverPos.y.toFixed(2)})`,
          pos.x,
          pos.y - 25
        );
      }
    }

    // 초기 위치 설정 모드 - 드래그 중 화살표 표시
    if (mapMode === 'initialPose') {
      if (dragState.startPos) {
        const startPixel = worldToPixel(dragState.startPos.x, dragState.startPos.y);

        // 시작점 원
        ctx.beginPath();
        ctx.fillStyle = '#f59e0b';
        ctx.arc(startPixel.x, startPixel.y, 15, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.stroke();

        // 드래그 중이면 방향 화살표 표시
        if (dragState.isDragging && dragState.currentPos) {
          const endPixel = worldToPixel(dragState.currentPos.x, dragState.currentPos.y);

          // 방향 라인
          ctx.beginPath();
          ctx.strokeStyle = '#f59e0b';
          ctx.lineWidth = 3;
          ctx.moveTo(startPixel.x, startPixel.y);
          ctx.lineTo(endPixel.x, endPixel.y);
          ctx.stroke();

          // 화살표 머리
          const angle = Math.atan2(endPixel.y - startPixel.y, endPixel.x - startPixel.x);
          const arrowSize = 15;
          ctx.beginPath();
          ctx.moveTo(endPixel.x, endPixel.y);
          ctx.lineTo(
            endPixel.x - arrowSize * Math.cos(angle - Math.PI / 6),
            endPixel.y - arrowSize * Math.sin(angle - Math.PI / 6)
          );
          ctx.moveTo(endPixel.x, endPixel.y);
          ctx.lineTo(
            endPixel.x - arrowSize * Math.cos(angle + Math.PI / 6),
            endPixel.y - arrowSize * Math.sin(angle + Math.PI / 6)
          );
          ctx.stroke();

          // 각도 표시
          const theta = Math.atan2(
            dragState.currentPos.y - dragState.startPos.y,
            dragState.currentPos.x - dragState.startPos.x
          );
          ctx.fillStyle = '#f59e0b';
          ctx.font = '12px sans-serif';
          ctx.textAlign = 'center';
          ctx.fillText(
            `(${dragState.startPos.x.toFixed(2)}, ${dragState.startPos.y.toFixed(2)}) θ=${(theta * 180 / Math.PI).toFixed(1)}°`,
            startPixel.x,
            startPixel.y - 25
          );
        } else {
          // 클릭만 한 상태 - 드래그 안내
          ctx.fillStyle = '#f59e0b';
          ctx.font = '12px sans-serif';
          ctx.textAlign = 'center';
          ctx.fillText('드래그하여 방향 설정', startPixel.x, startPixel.y - 25);
        }
      } else if (hoverPos) {
        // 호버 상태 - 클릭 안내
        const pos = worldToPixel(hoverPos.x, hoverPos.y);
        ctx.beginPath();
        ctx.strokeStyle = '#f59e0b';
        ctx.lineWidth = 2;
        ctx.setLineDash([5, 5]);
        ctx.arc(pos.x, pos.y, 20, 0, Math.PI * 2);
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.fillStyle = '#f59e0b';
        ctx.font = '12px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText(
          `(${hoverPos.x.toFixed(2)}, ${hoverPos.y.toFixed(2)})`,
          pos.x,
          pos.y - 25
        );
      }
    }
  }, [mapImage, robots, activeRobot, worldToPixel, mapMode, hoverPos, dragState, width, height]);

  // 마우스 이벤트
  const handleMouseMove = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (mapMode === 'none') return;

    const coords = getCanvasCoords(e);
    if (!coords) return;

    const world = pixelToWorld(coords.x, coords.y);

    if (dragState.isDragging) {
      // 드래그 중이면 현재 위치 업데이트
      setDragState((prev) => ({ ...prev, currentPos: world }));
    } else {
      // 호버 위치 업데이트
      setHoverPos(world);
    }
  };

  const handleMouseDown = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if ((mapMode !== 'initialPose' && mapMode !== 'goal') || !activeRobot) return;

    const coords = getCanvasCoords(e);
    if (!coords) return;

    const world = pixelToWorld(coords.x, coords.y);

    setDragState({
      startPos: world,
      currentPos: world,
      isDragging: true,
      mode: mapMode,
      robot: activeRobot,
    });
    setHoverPos(null);
  };

  const handleMouseUp = () => {
    if (!dragState.startPos || !dragState.isDragging) return;

    // 드래그 완료 - 각도 계산
    let theta = 0;
    if (dragState.currentPos && dragState.startPos) {
      const dx = dragState.currentPos.x - dragState.startPos.x;
      const dy = dragState.currentPos.y - dragState.startPos.y;
      // 최소 거리 이상 드래그했을 때만 각도 계산
      if (Math.sqrt(dx * dx + dy * dy) > 0.1) {
        theta = Math.atan2(dy, dx);
      }
    }

    // 모드에 따라 다른 동작
    if (dragState.mode === 'initialPose') {
      publishInitialPose(dragState.startPos.x, dragState.startPos.y, theta);
    } else if (dragState.mode === 'goal') {
      navigateToPose(dragState.startPos.x, dragState.startPos.y, theta);
    }

    // 상태 초기화
    setDragState({ startPos: null, currentPos: null, isDragging: false, mode: 'none', robot: null });
    setMapMode('none', null);
  };

  const handleClick = () => {
    // 드래그로 처리하므로 클릭은 사용하지 않음
  };

  const cancelMode = () => {
    setMapMode('none', null);
    setHoverPos(null);
    setDragState({ startPos: null, currentPos: null, isDragging: false, mode: 'none', robot: null });
  };

  return (
    <div className="flex flex-col gap-3">
      {/* 모드 안내 (활성화된 경우만) */}
      {mapMode === 'initialPose' && activeRobot && (
        <div className="text-sm text-amber-400 bg-amber-900/30 border border-amber-600/50 rounded-lg px-4 py-2">
          <strong>{ROBOTS.find(r => r.namespace === activeRobot)?.name}</strong>의 초기 위치를 맵에서 클릭+드래그하여 설정하세요
          <button
            onClick={cancelMode}
            className="ml-3 text-xs px-2 py-1 bg-red-600 hover:bg-red-700 rounded"
          >
            취소
          </button>
        </div>
      )}
      {mapMode === 'goal' && activeRobot && (
        <div className="text-sm text-green-400 bg-green-900/30 border border-green-600/50 rounded-lg px-4 py-2">
          <strong>{ROBOTS.find(r => r.namespace === activeRobot)?.name}</strong>의 목표점을 맵에서 클릭+드래그하여 설정하세요
          <button
            onClick={cancelMode}
            className="ml-3 text-xs px-2 py-1 bg-red-600 hover:bg-red-700 rounded"
          >
            취소
          </button>
        </div>
      )}

      {/* 맵 캔버스 */}
      <canvas
        ref={canvasRef}
        width={width}
        height={height}
        onMouseMove={handleMouseMove}
        onMouseDown={handleMouseDown}
        onMouseUp={handleMouseUp}
        onMouseLeave={() => {
          setHoverPos(null);
          if (dragState.isDragging) {
            handleMouseUp();
          }
        }}
        onClick={handleClick}
        className={`rounded-xl border-2 ${
          mapMode === 'goal'
            ? 'border-green-500 cursor-crosshair'
            : mapMode === 'initialPose'
            ? 'border-amber-500 cursor-crosshair'
            : 'border-gray-700'
        }`}
      />

      {/* 범례 */}
      <div className="flex gap-4 text-sm text-gray-400">
        {ROBOTS.map((robot) => (
          <div key={robot.namespace || 'default'} className="flex items-center gap-2">
            <div
              className="w-3 h-3 rounded-full"
              style={{ backgroundColor: robot.color }}
            />
            <span>{robot.name}</span>
          </div>
        ))}
      </div>
    </div>
  );
};
