# mlm_avoid_msgs

긴급 정지 시스템용 커스텀 ROS2 메시지 정의 패키지.

## 개요

여러 센서 데이터를 시간 동기화하여 하나의 메시지로 묶어 전달하기 위한 커스텀 메시지 타입을 정의한다.

## 메시지 타입

### SyncedData.msg

동기화된 센서 데이터를 담는 메시지.

```
std_msgs/Header header    # 동기화된 타임스탬프
sensor_msgs/Image image   # 카메라 이미지 (640x480 RGB)
sensor_msgs/LaserScan scan # LiDAR 스캔 (360도)
nav_msgs/Odometry odom    # 위치 및 속도
sensor_msgs/Imu imu       # 가속도 및 각속도
```

## 의존성

- std_msgs
- sensor_msgs
- nav_msgs

## 빌드

```bash
colcon build --packages-select mlm_avoid_msgs
```

## 사용 예시

### Python

```python
from mlm_avoid_msgs.msg import SyncedData

msg = SyncedData()
msg.header.stamp = self.get_clock().now().to_msg()
msg.image = image_msg
msg.scan = scan_msg
msg.odom = odom_msg
msg.imu = imu_msg
```

### 토픽 확인

```bash
ros2 interface show mlm_avoid_msgs/msg/SyncedData
```
