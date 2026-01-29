"""
TTC (Time To Collision) 계산기 테스트

Tests for TTC Calculator that computes time to collision from sensor data.
"""

import pytest
import math
from mlm_avoid_safety.ttc_calculator import TTCCalculator


class TestTTCBasic:
    """기본 TTC 계산 테스트

    Basic TTC calculation tests.
    """

    def test_calculate_approach_speed_forward(self):
        """calculate_approach_speed: 전방 이동 시 전방 접근 속도

        Forward approach speed when moving forward.
        """
        calc = TTCCalculator()

        # 전방으로 1 m/s 이동
        vx, vy = 1.0, 0.0
        # 전방 방향 (각도 0)
        angle = 0.0

        approach_speed = calc.calculate_approach_speed(vx, vy, angle)

        assert approach_speed == pytest.approx(1.0, rel=1e-6)

    def test_calculate_approach_speed_backward(self):
        """calculate_approach_speed: 전방 이동 시 후방 접근 속도

        Backward direction has negative approach speed when moving forward.
        """
        calc = TTCCalculator()

        vx, vy = 1.0, 0.0
        angle = math.pi  # 후방 (180도)

        approach_speed = calc.calculate_approach_speed(vx, vy, angle)

        # 후방으로 멀어지므로 음수
        assert approach_speed == pytest.approx(-1.0, rel=1e-6)

    def test_calculate_approach_speed_perpendicular(self):
        """calculate_approach_speed: 직각 방향 접근 속도 0

        Perpendicular direction has zero approach speed.
        """
        calc = TTCCalculator()

        vx, vy = 1.0, 0.0
        angle = math.pi / 2  # 측면 (90도)

        approach_speed = calc.calculate_approach_speed(vx, vy, angle)

        assert approach_speed == pytest.approx(0.0, abs=1e-6)

    def test_calculate_ttc_simple(self):
        """calculate_ttc: 단순 TTC 계산

        Simple TTC calculation with distance and approach speed.
        """
        calc = TTCCalculator()

        distance = 2.0  # 2m
        approach_speed = 1.0  # 1 m/s

        ttc = calc.calculate_ttc(distance, approach_speed)

        assert ttc == pytest.approx(2.0, rel=1e-6)

    def test_calculate_ttc_not_approaching(self):
        """calculate_ttc: 접근하지 않는 경우 무한대

        TTC is infinity when not approaching.
        """
        calc = TTCCalculator()

        distance = 2.0
        approach_speed = -1.0  # 멀어지는 중

        ttc = calc.calculate_ttc(distance, approach_speed)

        assert ttc == float('inf')

    def test_calculate_ttc_stationary(self):
        """calculate_ttc: 정지 상태 무한대

        TTC is infinity when stationary.
        """
        calc = TTCCalculator()

        distance = 2.0
        approach_speed = 0.0

        ttc = calc.calculate_ttc(distance, approach_speed)

        assert ttc == float('inf')


class TestMinTTC:
    """최소 TTC 계산 테스트

    Tests for finding minimum TTC across all directions.
    """

    def test_calculate_min_ttc_uniform(self, uniform_scan, moving_forward_odom):
        """calculate_min_ttc: 균일한 스캔에서 최소 TTC

        Minimum TTC from uniform scan when moving forward.
        """
        calc = TTCCalculator()

        min_ttc = calc.calculate_min_ttc(
            uniform_scan.ranges,
            uniform_scan.angle_min,
            uniform_scan.angle_increment,
            moving_forward_odom.twist.twist.linear.x,
            moving_forward_odom.twist.twist.linear.y
        )

        # 1m / 0.5 m/s = 2초
        assert min_ttc == pytest.approx(2.0, rel=1e-2)

    def test_calculate_min_ttc_obstacle_front(self, obstacle_front_scan, moving_forward_odom):
        """calculate_min_ttc: 전방 장애물, 전진 시

        Minimum TTC with front obstacle when moving forward.
        """
        calc = TTCCalculator()

        min_ttc = calc.calculate_min_ttc(
            obstacle_front_scan.ranges,
            obstacle_front_scan.angle_min,
            obstacle_front_scan.angle_increment,
            moving_forward_odom.twist.twist.linear.x,
            moving_forward_odom.twist.twist.linear.y
        )

        # 0.5m / 0.5 m/s = 1초
        assert min_ttc == pytest.approx(1.0, rel=1e-2)

    def test_calculate_min_ttc_obstacle_rear_forward(self, obstacle_rear_scan, moving_forward_odom):
        """calculate_min_ttc: 후방 장애물, 전진 시 (안전)

        Rear obstacle is not dangerous when moving forward.
        """
        calc = TTCCalculator()

        min_ttc = calc.calculate_min_ttc(
            obstacle_rear_scan.ranges,
            obstacle_rear_scan.angle_min,
            obstacle_rear_scan.angle_increment,
            moving_forward_odom.twist.twist.linear.x,
            moving_forward_odom.twist.twist.linear.y
        )

        # 후방 장애물은 멀어지므로 TTC가 큼 (전방 5m가 최소)
        # 5m / 0.5 m/s = 10초
        assert min_ttc == pytest.approx(10.0, rel=1e-2)

    def test_calculate_min_ttc_obstacle_rear_backward(self, obstacle_rear_scan, moving_backward_odom):
        """calculate_min_ttc: 후방 장애물, 후진 시 (위험)

        Rear obstacle is dangerous when moving backward.
        """
        calc = TTCCalculator()

        min_ttc = calc.calculate_min_ttc(
            obstacle_rear_scan.ranges,
            obstacle_rear_scan.angle_min,
            obstacle_rear_scan.angle_increment,
            moving_backward_odom.twist.twist.linear.x,
            moving_backward_odom.twist.twist.linear.y
        )

        # 0.5m / 0.5 m/s = 1초
        assert min_ttc == pytest.approx(1.0, rel=1e-2)

    def test_calculate_min_ttc_stationary(self, obstacle_front_scan, stationary_odom):
        """calculate_min_ttc: 정지 상태

        TTC is infinity when stationary.
        """
        calc = TTCCalculator()

        min_ttc = calc.calculate_min_ttc(
            obstacle_front_scan.ranges,
            obstacle_front_scan.angle_min,
            obstacle_front_scan.angle_increment,
            stationary_odom.twist.twist.linear.x,
            stationary_odom.twist.twist.linear.y
        )

        assert min_ttc == float('inf')


class TestInterventionDecision:
    """개입 판단 테스트

    Tests for intervention decision based on TTC.
    """

    def test_should_intervene_below_threshold(self):
        """should_intervene: TTC가 임계값 미만이면 개입

        Should intervene when TTC is below threshold.
        """
        calc = TTCCalculator(ttc_threshold_start=2.0, ttc_threshold_end=3.0)

        # TTC 1초 < 임계값 2초
        result = calc.should_intervene(ttc=1.0, currently_intervening=False)

        assert result is True

    def test_should_not_intervene_above_threshold(self):
        """should_intervene: TTC가 임계값 이상이면 개입 안 함

        Should not intervene when TTC is above threshold.
        """
        calc = TTCCalculator(ttc_threshold_start=2.0, ttc_threshold_end=3.0)

        # TTC 5초 > 임계값 3초
        result = calc.should_intervene(ttc=5.0, currently_intervening=False)

        assert result is False

    def test_hysteresis_maintain_intervention(self):
        """should_intervene: 히스테리시스 - 개입 중이면 유지

        Hysteresis: maintain intervention until end threshold.
        """
        calc = TTCCalculator(ttc_threshold_start=2.0, ttc_threshold_end=3.0)

        # TTC 2.5초: start(2초)와 end(3초) 사이
        # 현재 개입 중이면 계속 개입
        result = calc.should_intervene(ttc=2.5, currently_intervening=True)

        assert result is True

    def test_hysteresis_no_intervention(self):
        """should_intervene: 히스테리시스 - 개입 안 하던 중이면 유지

        Hysteresis: don't start intervention in middle zone.
        """
        calc = TTCCalculator(ttc_threshold_start=2.0, ttc_threshold_end=3.0)

        # TTC 2.5초: start(2초)와 end(3초) 사이
        # 현재 개입 안 하던 중이면 그대로
        result = calc.should_intervene(ttc=2.5, currently_intervening=False)

        assert result is False

    def test_end_intervention_above_end_threshold(self):
        """should_intervene: end 임계값 초과 시 개입 종료

        End intervention when TTC exceeds end threshold.
        """
        calc = TTCCalculator(ttc_threshold_start=2.0, ttc_threshold_end=3.0)

        # TTC 3.5초 > end 임계값 3초
        result = calc.should_intervene(ttc=3.5, currently_intervening=True)

        assert result is False
