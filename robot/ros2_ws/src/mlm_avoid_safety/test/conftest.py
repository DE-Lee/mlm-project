"""
MLM Avoid Safety 테스트를 위한 공통 fixtures

Fixtures for testing MLM Avoid Safety package.
"""

import pytest
import math
from typing import Any


# ============================================================================
# Mock ROS2 Message Classes
# ============================================================================

class MockStamp:
    """Mock for builtin_interfaces/Time"""

    def __init__(self, sec: int = 0, nanosec: int = 0):
        self.sec = sec
        self.nanosec = nanosec


class MockHeader:
    """Mock for std_msgs/Header"""

    def __init__(self, sec: int = 0, nanosec: int = 0, frame_id: str = ''):
        self.stamp = MockStamp(sec, nanosec)
        self.frame_id = frame_id


class MockLaserScan:
    """Mock for sensor_msgs/LaserScan"""

    def __init__(
        self,
        ranges: list[float] | None = None,
        angle_min: float = 0.0,
        angle_max: float = 2 * math.pi,
        angle_increment: float | None = None
    ):
        self.header = MockHeader(frame_id='laser_link')
        self.angle_min = angle_min
        self.angle_max = angle_max

        if ranges is None:
            ranges = [1.0] * 360

        n = len(ranges)
        if angle_increment is None:
            angle_increment = (angle_max - angle_min) / n

        self.angle_increment = angle_increment
        self.ranges = ranges
        self.range_min = 0.05
        self.range_max = 12.0


class MockTwist:
    """Mock for geometry_msgs/Twist"""

    def __init__(self, linear_x: float = 0.0, linear_y: float = 0.0, angular_z: float = 0.0):
        self.linear = type('Linear', (), {'x': linear_x, 'y': linear_y, 'z': 0.0})()
        self.angular = type('Angular', (), {'x': 0.0, 'y': 0.0, 'z': angular_z})()


class MockTwistWithCovariance:
    """Mock for geometry_msgs/TwistWithCovariance"""

    def __init__(self, linear_x: float = 0.0, linear_y: float = 0.0, angular_z: float = 0.0):
        self.twist = MockTwist(linear_x, linear_y, angular_z)
        self.covariance = [0.0] * 36


class MockOdometry:
    """Mock for nav_msgs/Odometry"""

    def __init__(self, linear_x: float = 0.0, linear_y: float = 0.0, angular_z: float = 0.0):
        self.header = MockHeader(frame_id='odom')
        self.twist = MockTwistWithCovariance(linear_x, linear_y, angular_z)


# ============================================================================
# Fixtures
# ============================================================================

@pytest.fixture
def mock_scan_factory():
    """MockLaserScan 생성 팩토리

    Factory for creating MockLaserScan instances.
    """
    def create(ranges: list[float] | None = None) -> MockLaserScan:
        return MockLaserScan(ranges=ranges)
    return create


@pytest.fixture
def mock_odom_factory():
    """MockOdometry 생성 팩토리

    Factory for creating MockOdometry instances.
    """
    def create(
        linear_x: float = 0.0,
        linear_y: float = 0.0,
        angular_z: float = 0.0
    ) -> MockOdometry:
        return MockOdometry(linear_x=linear_x, linear_y=linear_y, angular_z=angular_z)
    return create


@pytest.fixture
def uniform_scan():
    """모든 방향 1m 거리의 LiDAR 스캔

    Uniform scan with 1m distance in all directions.
    """
    return MockLaserScan(ranges=[1.0] * 360)


@pytest.fixture
def obstacle_front_scan():
    """전방에 장애물이 있는 LiDAR 스캔

    Scan with obstacle in front (index 0 = forward).
    """
    ranges = [5.0] * 360
    # 전방 30도 (±15도)에 0.5m 장애물
    for i in list(range(0, 15)) + list(range(345, 360)):
        ranges[i] = 0.5
    return MockLaserScan(ranges=ranges)


@pytest.fixture
def obstacle_rear_scan():
    """후방에 장애물이 있는 LiDAR 스캔

    Scan with obstacle in rear (index 180 = backward).
    """
    ranges = [5.0] * 360
    # 후방 30도 (165~195도)에 0.5m 장애물
    for i in range(165, 195):
        ranges[i] = 0.5
    return MockLaserScan(ranges=ranges)


@pytest.fixture
def moving_forward_odom():
    """전진 중인 오도메트리

    Odometry with forward velocity.
    """
    return MockOdometry(linear_x=0.5)  # 0.5 m/s forward


@pytest.fixture
def moving_backward_odom():
    """후진 중인 오도메트리

    Odometry with backward velocity.
    """
    return MockOdometry(linear_x=-0.5)  # 0.5 m/s backward


@pytest.fixture
def stationary_odom():
    """정지 중인 오도메트리

    Stationary odometry.
    """
    return MockOdometry(linear_x=0.0, linear_y=0.0)
