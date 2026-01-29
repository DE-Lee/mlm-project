"""
MLM Avoid Sync 테스트를 위한 공통 fixtures

Fixtures for testing MLM Avoid Sync package.
"""

import pytest


# ============================================================================
# Mock ROS2 Message Classes
# ROS2 의존성 없이 테스트하기 위한 mock 클래스들
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


class MockImage:
    """Mock for sensor_msgs/Image"""

    def __init__(
        self,
        sec: int = 0,
        nanosec: int = 0,
        width: int = 640,
        height: int = 480,
        encoding: str = 'rgb8'
    ):
        self.header = MockHeader(sec, nanosec, 'camera_link')
        self.width = width
        self.height = height
        self.encoding = encoding
        self.step = width * 3
        self.data = bytes(width * height * 3)


class MockLaserScan:
    """Mock for sensor_msgs/LaserScan"""

    def __init__(
        self,
        sec: int = 0,
        nanosec: int = 0,
        ranges: list[float] | None = None,
        angle_min: float = 0.0,
        angle_max: float = 6.28,
        angle_increment: float = 0.0174533  # ~1 degree
    ):
        self.header = MockHeader(sec, nanosec, 'laser_link')
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = angle_increment
        self.time_increment = 0.0
        self.scan_time = 0.1
        self.range_min = 0.05
        self.range_max = 12.0
        self.ranges = ranges if ranges is not None else [1.0] * 360
        self.intensities = []


class MockTwist:
    """Mock for geometry_msgs/Twist"""

    def __init__(self, linear_x: float = 0.0, angular_z: float = 0.0):
        self.linear = type('Linear', (), {'x': linear_x, 'y': 0.0, 'z': 0.0})()
        self.angular = type('Angular', (), {'x': 0.0, 'y': 0.0, 'z': angular_z})()


class MockPose:
    """Mock for geometry_msgs/Pose"""

    def __init__(self):
        self.position = type('Position', (), {'x': 0.0, 'y': 0.0, 'z': 0.0})()
        self.orientation = type('Orientation', (), {
            'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0
        })()


class MockPoseWithCovariance:
    """Mock for geometry_msgs/PoseWithCovariance"""

    def __init__(self):
        self.pose = MockPose()
        self.covariance = [0.0] * 36


class MockTwistWithCovariance:
    """Mock for geometry_msgs/TwistWithCovariance"""

    def __init__(self, linear_x: float = 0.0, angular_z: float = 0.0):
        self.twist = MockTwist(linear_x, angular_z)
        self.covariance = [0.0] * 36


class MockOdometry:
    """Mock for nav_msgs/Odometry"""

    def __init__(
        self,
        sec: int = 0,
        nanosec: int = 0,
        linear_x: float = 0.0,
        angular_z: float = 0.0
    ):
        self.header = MockHeader(sec, nanosec, 'odom')
        self.child_frame_id = 'base_link'
        self.pose = MockPoseWithCovariance()
        self.twist = MockTwistWithCovariance(linear_x, angular_z)


class MockVector3:
    """Mock for geometry_msgs/Vector3"""

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z


class MockQuaternion:
    """Mock for geometry_msgs/Quaternion"""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class MockImu:
    """Mock for sensor_msgs/Imu"""

    def __init__(
        self,
        sec: int = 0,
        nanosec: int = 0,
        linear_accel_x: float = 0.0,
        angular_vel_z: float = 0.0
    ):
        self.header = MockHeader(sec, nanosec, 'imu_link')
        self.orientation = MockQuaternion()
        self.orientation_covariance = [0.0] * 9
        self.angular_velocity = MockVector3(0.0, 0.0, angular_vel_z)
        self.angular_velocity_covariance = [0.0] * 9
        self.linear_acceleration = MockVector3(linear_accel_x, 0.0, 9.81)
        self.linear_acceleration_covariance = [0.0] * 9


# ============================================================================
# Fixtures
# ============================================================================

@pytest.fixture
def mock_image_factory():
    """MockImage 생성 팩토리

    Factory for creating MockImage instances.
    """
    def create(sec: int = 0, nanosec: int = 0) -> MockImage:
        return MockImage(sec=sec, nanosec=nanosec)
    return create


@pytest.fixture
def mock_scan_factory():
    """MockLaserScan 생성 팩토리

    Factory for creating MockLaserScan instances.
    """
    def create(
        sec: int = 0,
        nanosec: int = 0,
        ranges: list[float] | None = None
    ) -> MockLaserScan:
        return MockLaserScan(sec=sec, nanosec=nanosec, ranges=ranges)
    return create


@pytest.fixture
def mock_odom_factory():
    """MockOdometry 생성 팩토리

    Factory for creating MockOdometry instances.
    """
    def create(
        sec: int = 0,
        nanosec: int = 0,
        linear_x: float = 0.0,
        angular_z: float = 0.0
    ) -> MockOdometry:
        return MockOdometry(
            sec=sec,
            nanosec=nanosec,
            linear_x=linear_x,
            angular_z=angular_z
        )
    return create


@pytest.fixture
def mock_imu_factory():
    """MockImu 생성 팩토리

    Factory for creating MockImu instances.
    """
    def create(
        sec: int = 0,
        nanosec: int = 0,
        linear_accel_x: float = 0.0,
        angular_vel_z: float = 0.0
    ) -> MockImu:
        return MockImu(
            sec=sec,
            nanosec=nanosec,
            linear_accel_x=linear_accel_x,
            angular_vel_z=angular_vel_z
        )
    return create


@pytest.fixture
def synced_sensor_data(
    mock_image_factory,
    mock_scan_factory,
    mock_odom_factory,
    mock_imu_factory
):
    """동기화된 센서 데이터 세트

    A set of sensor data with the same timestamp.
    """
    sec = 1000
    nanosec = 500000000  # 0.5초

    return {
        'image': mock_image_factory(sec=sec, nanosec=nanosec),
        'scan': mock_scan_factory(sec=sec, nanosec=nanosec),
        'odom': mock_odom_factory(sec=sec, nanosec=nanosec, linear_x=0.5),
        'imu': mock_imu_factory(sec=sec, nanosec=nanosec),
    }


@pytest.fixture
def async_sensor_data(
    mock_image_factory,
    mock_scan_factory,
    mock_odom_factory,
    mock_imu_factory
):
    """비동기 센서 데이터 세트 (약간의 시간 차이)

    A set of sensor data with slightly different timestamps.
    """
    base_sec = 1000

    return {
        'image': mock_image_factory(sec=base_sec, nanosec=0),
        'scan': mock_scan_factory(sec=base_sec, nanosec=50000000),   # +50ms
        'odom': mock_odom_factory(sec=base_sec, nanosec=20000000),   # +20ms
        'imu': mock_imu_factory(sec=base_sec, nanosec=10000000),     # +10ms
    }
