"""LiDAR-BBox 융합 모듈

카메라 BBox를 LiDAR 데이터와 융합하여 객체의 3D 위치를 추정합니다.
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple, List


@dataclass
class CameraParams:
    """카메라 파라미터"""
    fx: float  # focal length x
    fy: float  # focal length y
    cx: float  # principal point x
    cy: float  # principal point y
    width: int  # 이미지 너비
    height: int  # 이미지 높이


@dataclass
class FusionConfig:
    """융합 설정"""
    camera_hfov: float  # 카메라 수평 FOV (라디안)
    camera_lidar_angle_offset: float  # 카메라-LiDAR 각도 오프셋
    max_range: float  # 최대 감지 거리


class LidarBBoxFusion:
    """LiDAR-BBox 융합 클래스"""

    def __init__(self, camera_params: CameraParams, config: FusionConfig):
        self.camera = camera_params
        self.config = config

    def pixel_to_angle(self, u: float) -> float:
        """이미지 x좌표(픽셀)를 카메라 기준 수평 각도로 변환

        Args:
            u: 이미지 x좌표 (픽셀)

        Returns:
            카메라 기준 수평 각도 (라디안)
            양수 = 왼쪽, 음수 = 오른쪽 (ROS 좌표계 기준)
        """
        # 이미지 중심 기준 오프셋
        # 이미지 좌표: 왼쪽이 0, 오른쪽이 width
        # 카메라 좌표: 왼쪽이 양수, 오른쪽이 음수
        offset = self.camera.cx - u
        angle = np.arctan2(offset, self.camera.fx)
        return angle

    def bbox_to_angle_range(
        self, center_x: float, size_x: float
    ) -> Tuple[float, float]:
        """BBox를 LiDAR 각도 범위로 변환

        Args:
            center_x: BBox 중심 x좌표 (픽셀)
            size_x: BBox 너비 (픽셀)

        Returns:
            (min_angle, max_angle) LiDAR 각도 범위 (라디안)
        """
        half_width = size_x / 2.0
        left_u = center_x - half_width
        right_u = center_x + half_width

        # 픽셀 -> 카메라 각도
        left_angle = self.pixel_to_angle(left_u)
        right_angle = self.pixel_to_angle(right_u)

        # 카메라 -> LiDAR 각도 (오프셋 적용)
        left_angle += self.config.camera_lidar_angle_offset
        right_angle += self.config.camera_lidar_angle_offset

        # min/max 정렬
        min_angle = min(left_angle, right_angle)
        max_angle = max(left_angle, right_angle)

        return min_angle, max_angle

    def get_lidar_points_in_range(
        self,
        ranges: np.ndarray,
        angle_min: float,
        angle_increment: float,
        target_min_angle: float,
        target_max_angle: float
    ) -> List[Tuple[float, float]]:
        """지정된 각도 범위 내의 LiDAR 포인트들을 추출

        Args:
            ranges: LiDAR range 배열
            angle_min: LiDAR 시작 각도 (라디안)
            angle_increment: LiDAR 각도 증분 (라디안)
            target_min_angle: 추출할 최소 각도
            target_max_angle: 추출할 최대 각도

        Returns:
            (distance, angle) 튜플의 리스트
        """
        points = []
        num_ranges = len(ranges)

        for i in range(num_ranges):
            angle = angle_min + i * angle_increment
            distance = ranges[i]

            # 각도 범위 체크
            if target_min_angle <= angle <= target_max_angle:
                # 유효한 거리인지 체크 (inf, nan, 0 제외)
                if np.isfinite(distance) and distance > 0.01:
                    if distance <= self.config.max_range:
                        points.append((distance, angle))

        return points

    def estimate_position(
        self,
        points: List[Tuple[float, float]]
    ) -> Optional[Tuple[float, float]]:
        """LiDAR 포인트들로부터 객체 위치 추정

        최근접점 방식: 가장 가까운 포인트를 객체 위치로 사용

        Args:
            points: (distance, angle) 튜플의 리스트

        Returns:
            (rel_x, rel_y) 로봇 기준 상대 좌표, 포인트 없으면 None
        """
        if not points:
            return None

        # 가장 가까운 포인트 찾기
        min_distance = float('inf')
        min_angle = 0.0

        for distance, angle in points:
            if distance < min_distance:
                min_distance = distance
                min_angle = angle

        # 극좌표 -> 직교좌표 변환
        # ROS 좌표계: x=전방, y=좌측
        rel_x = min_distance * np.cos(min_angle)
        rel_y = min_distance * np.sin(min_angle)

        return rel_x, rel_y

    def fuse(
        self,
        bbox_center_x: float,
        bbox_size_x: float,
        lidar_ranges: np.ndarray,
        lidar_angle_min: float,
        lidar_angle_increment: float
    ) -> Optional[Tuple[float, float]]:
        """BBox와 LiDAR 데이터를 융합하여 객체 위치 추정

        Args:
            bbox_center_x: BBox 중심 x좌표 (픽셀)
            bbox_size_x: BBox 너비 (픽셀)
            lidar_ranges: LiDAR range 배열
            lidar_angle_min: LiDAR 시작 각도
            lidar_angle_increment: LiDAR 각도 증분

        Returns:
            (rel_x, rel_y) 로봇 기준 상대 좌표, 실패 시 None
        """
        # BBox -> 각도 범위
        min_angle, max_angle = self.bbox_to_angle_range(bbox_center_x, bbox_size_x)

        # 해당 범위의 LiDAR 포인트 추출
        points = self.get_lidar_points_in_range(
            lidar_ranges,
            lidar_angle_min,
            lidar_angle_increment,
            min_angle,
            max_angle
        )

        # 위치 추정
        return self.estimate_position(points)
