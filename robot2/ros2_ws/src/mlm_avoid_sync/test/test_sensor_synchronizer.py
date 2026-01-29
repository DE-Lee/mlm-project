"""
SensorSynchronizer 클래스 테스트

Tests for SensorSynchronizer class that handles sensor data synchronization.
"""

import pytest
from mlm_avoid_sync.sensor_synchronizer import SensorSynchronizer


class TestTimestampConversion:
    """타임스탬프 변환 테스트

    Tests for timestamp conversion utilities.
    """

    def test_stamp_to_float_zero(self):
        """stamp_to_float: 0초 반환

        Zero timestamp should return 0.0.
        """
        sync = SensorSynchronizer(buffer_size=10)
        result = sync.stamp_to_float(sec=0, nanosec=0)

        assert result == 0.0

    def test_stamp_to_float_whole_seconds(self):
        """stamp_to_float: 정수 초만 있는 경우

        Whole seconds only should return float seconds.
        """
        sync = SensorSynchronizer(buffer_size=10)
        result = sync.stamp_to_float(sec=100, nanosec=0)

        assert result == 100.0

    def test_stamp_to_float_with_nanoseconds(self):
        """stamp_to_float: 나노초 포함

        Timestamp with nanoseconds should be correctly converted.
        """
        sync = SensorSynchronizer(buffer_size=10)
        result = sync.stamp_to_float(sec=100, nanosec=500000000)  # 0.5초

        assert result == 100.5

    def test_stamp_to_float_precision(self):
        """stamp_to_float: 정밀도 검증

        Nanosecond precision should be maintained.
        """
        sync = SensorSynchronizer(buffer_size=10)
        result = sync.stamp_to_float(sec=0, nanosec=123456789)

        assert abs(result - 0.123456789) < 1e-9


class TestBufferManagement:
    """버퍼 관리 테스트

    Tests for buffer management functionality.
    """

    def test_add_to_buffer_single_item(self, mock_scan_factory):
        """add_to_buffer: 단일 항목 추가

        Single item should be added to buffer.
        """
        sync = SensorSynchronizer(buffer_size=10)
        scan = mock_scan_factory(sec=100, nanosec=0)

        sync.add_to_buffer('scan', scan)

        assert len(sync.buffers['scan']) == 1

    def test_add_to_buffer_multiple_items(self, mock_scan_factory):
        """add_to_buffer: 여러 항목 추가

        Multiple items should be added in order.
        """
        sync = SensorSynchronizer(buffer_size=10)

        for i in range(5):
            scan = mock_scan_factory(sec=100 + i, nanosec=0)
            sync.add_to_buffer('scan', scan)

        assert len(sync.buffers['scan']) == 5

    def test_add_to_buffer_overflow(self, mock_scan_factory):
        """add_to_buffer: 버퍼 오버플로우 시 오래된 데이터 제거

        Old data should be removed when buffer overflows.
        """
        buffer_size = 5
        sync = SensorSynchronizer(buffer_size=buffer_size)

        # buffer_size보다 많이 추가
        for i in range(10):
            scan = mock_scan_factory(sec=100 + i, nanosec=0)
            sync.add_to_buffer('scan', scan)

        assert len(sync.buffers['scan']) == buffer_size
        # 가장 오래된 것이 제거되었는지 확인
        oldest = sync.buffers['scan'][0]
        assert oldest.header.stamp.sec == 105  # 100+5

    def test_clear_buffer(self, mock_scan_factory):
        """clear_buffer: 버퍼 비우기

        Buffer should be cleared.
        """
        sync = SensorSynchronizer(buffer_size=10)

        for i in range(5):
            scan = mock_scan_factory(sec=100 + i, nanosec=0)
            sync.add_to_buffer('scan', scan)

        sync.clear_buffer('scan')

        assert len(sync.buffers['scan']) == 0


class TestSynchronize:
    """동기화 테스트

    Tests for synchronization of multiple sensor data.
    """

    def test_synchronize_all_available(
        self,
        mock_image_factory,
        mock_scan_factory,
        mock_odom_factory,
        mock_imu_factory
    ):
        """synchronize: 모든 데이터 있는 경우

        Should return synced data when all sensors have latest data.
        """
        sync = SensorSynchronizer(buffer_size=10)

        # 모든 센서 데이터 추가
        sec = 100
        sync.add_to_buffer('image', mock_image_factory(sec=sec, nanosec=0))
        sync.add_to_buffer('scan', mock_scan_factory(sec=sec, nanosec=0))
        sync.add_to_buffer('odom', mock_odom_factory(sec=sec, nanosec=0))
        sync.add_to_buffer('imu', mock_imu_factory(sec=sec, nanosec=0))

        result = sync.synchronize()

        assert result is not None
        assert 'image' in result
        assert 'scan' in result
        assert 'odom' in result
        assert 'imu' in result

    def test_synchronize_missing_sensor(
        self,
        mock_image_factory,
        mock_scan_factory,
        mock_odom_factory
    ):
        """synchronize: 센서가 없는 경우

        Should return None when required sensors have never received data.
        """
        sync = SensorSynchronizer(buffer_size=10)

        sec = 100
        sync.add_to_buffer('image', mock_image_factory(sec=sec, nanosec=0))
        sync.add_to_buffer('scan', mock_scan_factory(sec=sec, nanosec=0))
        sync.add_to_buffer('odom', mock_odom_factory(sec=sec, nanosec=0))
        # imu 한 번도 안 받음

        result = sync.synchronize()

        assert result is None

    def test_synchronize_uses_latest_value(
        self,
        mock_image_factory,
        mock_scan_factory,
        mock_odom_factory,
        mock_imu_factory
    ):
        """synchronize: 최신값 사용 (느린 센서 재사용)

        Should use latest value for slow sensors.
        """
        sync = SensorSynchronizer(buffer_size=10)

        # 느린 센서 (scan) - 한 번만 받음
        sync.add_to_buffer('scan', mock_scan_factory(sec=100, nanosec=0))

        # 빠른 센서들 - 여러 번 받음
        for i in range(5):
            sync.add_to_buffer('image', mock_image_factory(sec=100 + i, nanosec=0))
            sync.add_to_buffer('odom', mock_odom_factory(sec=100 + i, nanosec=0))
            sync.add_to_buffer('imu', mock_imu_factory(sec=100 + i, nanosec=0))

        # 동기화 성공 (scan의 이전 값 재사용)
        result = sync.synchronize()

        assert result is not None
        # scan은 sec=100의 데이터 (latest)
        assert result['scan'].header.stamp.sec == 100
        # image는 sec=104의 데이터 (latest)
        assert result['image'].header.stamp.sec == 104


class TestOnImageCallback:
    """이미지 콜백 기반 동기화 테스트

    Tests for image-triggered synchronization.
    """

    def test_on_image_triggers_sync(
        self,
        mock_image_factory,
        mock_scan_factory,
        mock_odom_factory,
        mock_imu_factory
    ):
        """on_image: 이미지 도착 시 동기화 시도

        Image arrival should trigger synchronization attempt.
        """
        sync = SensorSynchronizer(buffer_size=10)

        # 다른 센서 데이터 미리 추가
        sec = 100
        sync.add_to_buffer('scan', mock_scan_factory(sec=sec, nanosec=0))
        sync.add_to_buffer('odom', mock_odom_factory(sec=sec, nanosec=0))
        sync.add_to_buffer('imu', mock_imu_factory(sec=sec, nanosec=0))

        # 이미지 도착
        image = mock_image_factory(sec=sec, nanosec=0)
        result = sync.on_image(image)

        assert result is not None
        assert result['image'] is image

    def test_on_image_fails_without_other_sensors(
        self,
        mock_image_factory
    ):
        """on_image: 다른 센서 없으면 실패

        Image arrival should fail if other sensors haven't arrived.
        """
        sync = SensorSynchronizer(buffer_size=10)

        # 이미지만 도착
        image = mock_image_factory(sec=100, nanosec=0)
        result = sync.on_image(image)

        assert result is None
