"""
Sensor data synchronizer for emergency stop system.

여러 센서 데이터를 시간 기반으로 동기화하는 모듈.
카메라, LiDAR, Odometry, IMU 데이터를 동기화하여 하나의 묶음으로 제공한다.

동기화 방식 (Latest):
    - 각 센서의 가장 최신 데이터를 사용
    - 느린 주기의 센서는 새 데이터가 올 때까지 이전 값 재사용
    - 필수 센서(image, scan, odom, imu)는 한 번도 받지 못하면 실패
"""

from typing import Any


class SensorSynchronizer:
    """
    Sensor data synchronizer using latest-value matching.

    여러 센서의 데이터를 동기화한다.
    각 센서의 가장 최신 데이터를 사용하며, 느린 주기의 센서는
    새 데이터가 올 때까지 이전 값을 재사용한다.

    필수 센서 (한 번도 안 받으면 실패):
        - image, scan, odom, imu

    Args:
        buffer_size: Maximum number of items to keep in each buffer.

    버퍼 크기를 설정하여 메모리 사용을 조절한다.
    """

    SENSOR_NAMES = ['image', 'scan', 'odom', 'imu']

    def __init__(self, buffer_size: int = 10):
        """
        Initialize the synchronizer.

        Args:
            buffer_size: Maximum buffer size per sensor.

        동기화기를 초기화한다.
        """
        self.buffer_size = buffer_size
        self.buffers: dict[str, list[Any]] = {name: [] for name in self.SENSOR_NAMES}
        # 각 센서의 최신 데이터 저장
        self.latest: dict[str, Any] = {name: None for name in self.SENSOR_NAMES}

    def stamp_to_float(self, sec: int, nanosec: int) -> float:
        """
        Convert ROS2 timestamp to float seconds.

        Args:
            sec: Seconds part of timestamp.
            nanosec: Nanoseconds part of timestamp.

        Returns:
            Timestamp as float seconds.

        ROS2 타임스탬프를 float 초 단위로 변환한다.
        """
        return float(sec) + float(nanosec) / 1e9

    def add_to_buffer(self, sensor_name: str, msg: Any) -> None:
        """
        Add sensor message to buffer and update latest.

        Args:
            sensor_name: Name of sensor ('image', 'scan', 'odom', 'imu').
            msg: ROS2 message with header.stamp.

        센서 메시지를 버퍼에 추가하고 최신값을 업데이트한다.
        버퍼가 가득 차면 가장 오래된 데이터를 제거한다.
        """
        buffer = self.buffers[sensor_name]
        buffer.append(msg)

        # 최신값 업데이트
        self.latest[sensor_name] = msg

        # 버퍼 오버플로우 처리
        while len(buffer) > self.buffer_size:
            buffer.pop(0)

    def clear_buffer(self, sensor_name: str) -> None:
        """
        Clear buffer for a sensor.

        Args:
            sensor_name: Name of sensor to clear.

        특정 센서의 버퍼를 비운다.
        """
        self.buffers[sensor_name] = []

    def synchronize(self) -> dict[str, Any] | None:
        """
        Synchronize all sensor data using latest values.

        Returns:
            Dictionary of synced sensor data, or None if sync fails.

        각 센서의 최신 데이터를 사용하여 동기화한다.
        필수 센서가 한 번도 받지 못하면 None을 반환한다.
        """
        synced_data: dict[str, Any] = {}

        # 모든 센서: 한 번도 안 받으면 실패
        for sensor_name in self.SENSOR_NAMES:
            msg = self.latest[sensor_name]
            if msg is None:
                return None  # 필수 센서가 없음
            synced_data[sensor_name] = msg

        return synced_data

    def on_image(self, image_msg: Any) -> dict[str, Any] | None:
        """
        Handle image arrival and attempt synchronization.

        Args:
            image_msg: Incoming image message.

        Returns:
            Synchronized data dict if successful, None otherwise.

        이미지 도착 시 호출되어 동기화를 시도한다.
        이미지를 기준으로 다른 센서 데이터와 동기화한다.
        """
        # 이미지를 버퍼에 추가
        self.add_to_buffer('image', image_msg)

        # 동기화 시도
        return self.synchronize()
