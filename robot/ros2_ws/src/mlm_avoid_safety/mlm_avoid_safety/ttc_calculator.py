"""
TTC (Time To Collision) calculator for emergency avoidance system.

LiDAR와 Odometry 데이터를 사용하여 충돌까지 남은 시간을 계산하는 모듈.
360도 전체 스캔에서 가장 작은 TTC를 찾아 위험도를 판단한다.
"""

import math
from typing import Sequence


class TTCCalculator:
    """
    Time To Collision calculator using LiDAR and velocity data.

    LiDAR 스캔 데이터와 로봇 속도를 사용하여 각 방향의 TTC를 계산하고,
    가장 작은 TTC를 기반으로 개입 여부를 결정한다.

    Args:
        ttc_threshold_start: TTC below this triggers intervention.
        ttc_threshold_end: TTC above this ends intervention.
        min_approach_speed: Minimum speed to consider as approaching.

    히스테리시스를 통해 경계값 근처에서 왔다갔다 하는 것을 방지한다.
    """

    def __init__(
        self,
        ttc_threshold_start: float = 2.0,
        ttc_threshold_end: float = 3.0,
        min_approach_speed: float = 0.01
    ):
        """
        Initialize the TTC calculator.

        Args:
            ttc_threshold_start: TTC threshold to start intervention (seconds).
            ttc_threshold_end: TTC threshold to end intervention (seconds).
            min_approach_speed: Minimum approach speed to consider (m/s).

        TTC 계산기를 초기화한다.
        """
        self.ttc_threshold_start = ttc_threshold_start
        self.ttc_threshold_end = ttc_threshold_end
        self.min_approach_speed = min_approach_speed

    def calculate_approach_speed(
        self,
        vx: float,
        vy: float,
        angle: float
    ) -> float:
        """
        Calculate approach speed in a specific direction.

        Args:
            vx: Robot velocity in x direction (forward) in m/s.
            vy: Robot velocity in y direction (left) in m/s.
            angle: Direction angle in radians (0 = forward, pi/2 = left).

        Returns:
            Approach speed in that direction. Positive means approaching.

        특정 방향으로의 접근 속도를 계산한다.
        속도 벡터와 방향 벡터의 내적으로 계산한다.
        """
        # 해당 방향의 단위 벡터
        dx = math.cos(angle)
        dy = math.sin(angle)

        # 내적: 해당 방향으로의 속도 성분
        approach_speed = vx * dx + vy * dy

        return approach_speed

    def calculate_ttc(self, distance: float, approach_speed: float) -> float:
        """
        Calculate time to collision.

        Args:
            distance: Distance to obstacle in meters.
            approach_speed: Approach speed toward obstacle in m/s.

        Returns:
            Time to collision in seconds. Infinity if not approaching.

        충돌까지 남은 시간을 계산한다.
        접근하지 않는 경우 무한대를 반환한다.
        """
        if approach_speed <= self.min_approach_speed:
            return float('inf')

        return distance / approach_speed

    def calculate_min_ttc(
        self,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        vx: float,
        vy: float
    ) -> float:
        """
        Calculate minimum TTC across all LiDAR directions.

        Args:
            ranges: LiDAR range readings in meters.
            angle_min: Starting angle of scan in radians.
            angle_increment: Angle between consecutive readings in radians.
            vx: Robot velocity in x direction (forward) in m/s.
            vy: Robot velocity in y direction (left) in m/s.

        Returns:
            Minimum TTC across all directions in seconds.

        모든 LiDAR 방향에서 최소 TTC를 계산한다.
        """
        min_ttc = float('inf')

        # gt 수정 (1) - 정지 상태에서 TTC가 무한대가 되어 재출발하는 문제(Zero Velocity Paradox)를 해결하기 위해 가상 예측 속도 변수 추가
        current_speed = math.sqrt(vx**2 + vy**2)
        
        # 실제 속도 변수 (기본값)
        calc_vx = vx
        calc_vy = vy

        # 정지 상태(0.05m/s 미만)라면 가상 속도 주입 (전방 0.2m/s 가정)하여 정지 유지
        if current_speed < 0.05:
            calc_vx = 0.2
            calc_vy = 0.0

        for i, distance in enumerate(ranges):
            # 유효하지 않은 거리 무시
            if distance <= 0.05 or distance == float('inf') or math.isnan(distance):
                continue

            # 이 방향의 각도
            angle = angle_min + i * angle_increment

            # gt 수정 (2) - 기존 vx, vy 대신 보정된 속도(calc_vx, calc_vy)를 사용하여 계산하도록 변경
            # approach_speed = self.calculate_approach_speed(vx, vy, angle)
            approach_speed = self.calculate_approach_speed(calc_vx, calc_vy, angle)

            # TTC 계산
            ttc = self.calculate_ttc(distance, approach_speed)

            if ttc < min_ttc:
                min_ttc = ttc

        return min_ttc

    def should_intervene(self, ttc: float, currently_intervening: bool) -> bool:
        """
        Decide whether to intervene based on TTC.

        Args:
            ttc: Current minimum TTC in seconds.
            currently_intervening: Whether currently in intervention mode.

        Returns:
            True if should intervene, False otherwise.

        TTC를 기반으로 개입 여부를 결정한다.
        히스테리시스를 적용하여 경계값 근처에서 흔들리지 않도록 한다.
        """
        if ttc < self.ttc_threshold_start:
            # TTC가 시작 임계값 미만이면 개입
            return True
        elif ttc > self.ttc_threshold_end:
            # TTC가 종료 임계값 초과면 개입 종료
            return False
        else:
            # 중간 영역: 현재 상태 유지 (히스테리시스)
            return currently_intervening