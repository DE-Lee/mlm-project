# mlm_avoid_sync

센서 데이터 동기화 노드 패키지.

## 개요

카메라, LiDAR, Odometry, IMU 데이터를 동기화하여 `SyncedData` 메시지로 발행한다.
**이미지 도착을 기준**으로 각 센서의 최신 데이터를 묶어서 발행한다.

## 노드

### sync_node

센서 데이터 동기화 노드.

#### 구독 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/image_raw` | sensor_msgs/Image | 카메라 이미지 |
| `/scan_raw` | sensor_msgs/LaserScan | LiDAR 스캔 |
| `/odom` | nav_msgs/Odometry | 오도메트리 |
| `/imu` | sensor_msgs/Imu | IMU 데이터 |

#### 발행 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/mlm_avoid/synced_data` | mlm_avoid_msgs/SyncedData | 동기화된 센서 데이터 |

#### 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `buffer_size` | int | 30 | 센서별 버퍼 크기 |
| `image_topic` | string | `/image_raw` | 이미지 토픽 이름 |
| `scan_topic` | string | `/scan_raw` | LiDAR 토픽 이름 |
| `odom_topic` | string | `/odom` | Odometry 토픽 이름 |
| `imu_topic` | string | `/imu` | IMU 토픽 이름 |

## 핵심 클래스

### SensorSynchronizer

센서 데이터 동기화 로직을 담당하는 클래스.

```python
from mlm_avoid_sync.sensor_synchronizer import SensorSynchronizer

sync = SensorSynchronizer(buffer_size=30)
sync.add_to_buffer('scan', scan_msg)
sync.add_to_buffer('odom', odom_msg)
sync.add_to_buffer('imu', imu_msg)

# 이미지 도착 시 동기화 시도
result = sync.on_image(image_msg)
if result is not None:
    # result['image'], result['scan'], result['odom'], result['imu']
```

## 동기화 방식 (Latest Value)

**이미지 기준 동기화**: 이미지가 도착할 때만 SyncedData 발행

```
image 30Hz  ──────●─────●─────●─────●─────  (기준, 이 주기로 발행)
scan 10Hz   ──────────●───────────●───────  (느림 → 최신값 재사용)
imu 100Hz   ────●●●●●●●●●●●●●●●●●●●●●●●●──  (빠름 → 최신값만 사용)
                  ↓     ↓     ↓     ↓
SyncedData       ●     ●     ●     ●        (30Hz)
```

### 동작 원리

1. 각 센서 콜백에서 최신값(`latest`)을 업데이트
2. **이미지 도착 시** 모든 센서의 `latest` 값을 모아서 SyncedData 발행
3. 모든 센서가 한 번 이상 데이터를 받아야 발행 시작

## 실행

```bash
ros2 run mlm_avoid_sync sync_node
```

## 테스트

```bash
cd /workspaces/ros2
python3 -m pytest src/mlm_avoid_sync/test/ -v
```
