# Emergency Stop System - 파일 변경 기록

**작업일:** 2026-01-27
**목적:** GT 로봇 Emergency Stop 시스템을 DOG 로봇(robot1)에 이식

---

## 변경 파일 목록

| # | 파일 위치 | 변경 유형 |
|---|-----------|----------|
| 1 | pc/ros2_ws/src/navigation/launch/navigation_pc.launch.py | 수정 |
| 2 | pc/ros2_ws/src/navigation/config/nav2_params_ackermann.yaml | 수정 |
| 3 | robot/ros2_ws/src/bringup/launch/mlm_bringup.launch.py | 수정 |
| 4 | robot/ros2_ws/src/mlm_avoid_sync/mlm_avoid_sync/sync_node.py | 신규 (GT에서 수정) |
| 5 | robot/ros2_ws/src/mlm_avoid_safety/mlm_avoid_safety/safety_node.py | 신규 (GT에서 수정) |
| 6 | robot/ros2_ws/src/mlm_avoid_bringup/config/avoid_params.yaml | 신규 (GT에서 수정) |
| 7 | robot/ros2_ws/src/mlm_avoid_bringup/launch/avoid.launch.py | 신규 (GT에서 수정) |
| 8 | robot/ros2_ws/src/peripherals/config/usb_cam_param.yaml | 수정 |
| 9 | pc/cyclonedds/cyclonedds.xml | 수정 |
| 10 | robot/cyclonedds/cyclonedds.xml | 수정 |

---

## 1. navigation_pc.launch.py

**파일:** `pc/ros2_ws/src/navigation/launch/navigation_pc.launch.py`

### 변경 전 (Line 116)
```python
        # Controller Server (local_costmap 포함)
        Node(
            package='nav2_controller',
            executable='controller_server',
            ...
            remappings=costmap_remappings,
        ),
```

### 변경 후
```python
        # Controller Server (local_costmap 포함)
        Node(
            package='nav2_controller',
            executable='controller_server',
            ...
            remappings=costmap_remappings + [('cmd_vel', 'cmd_vel_nav')],  # Emergency Stop: safety_node 경유
        ),
```

### 변경 전 (Line 136)
```python
        # Behavior Server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            ...
            remappings=remappings,
        ),
```

### 변경 후
```python
        # Behavior Server (cmd_vel → cmd_vel_nav: Emergency Stop 경유)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            ...
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
```

---

## 2. nav2_params_ackermann.yaml

**파일:** `pc/ros2_ws/src/navigation/config/nav2_params_ackermann.yaml`

### 변경 전 (Line 289-290)
```yaml
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
  ...
  use_collision_detection: true
  max_allowed_time_to_collision_up_to_carrot: 1.0
```

### 변경 후
```yaml
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
  ...
  use_collision_detection: false  # Emergency Stop이 TTC 담당
  # max_allowed_time_to_collision_up_to_carrot: 1.0  # 불필요
```

---

## 3. mlm_bringup.launch.py

**파일:** `robot/ros2_ws/src/bringup/launch/mlm_bringup.launch.py`

### 변경 전
```python
if compiled == "True":
    controller_package_path = get_package_share_directory("controller")
    peripherals_package_path = get_package_share_directory("peripherals")
else:
    controller_package_path = "/home/ubuntu/ros2_ws/src/driver/controller"
    peripherals_package_path = "/home/ubuntu/ros2_ws/src/peripherals"

# ... 중간 생략 ...

return [
    robot_desc_launch,
    controller_node,
    peripherals_launch,
    imu_node,
    odom_node,
]
```

### 변경 후
```python
if compiled == "True":
    controller_package_path = get_package_share_directory("controller")
    peripherals_package_path = get_package_share_directory("peripherals")
    avoid_package_path = get_package_share_directory("mlm_avoid_bringup")
else:
    controller_package_path = "/home/ubuntu/ros2_ws/src/driver/controller"
    peripherals_package_path = "/home/ubuntu/ros2_ws/src/peripherals"
    avoid_package_path = "/home/ubuntu/ros2_ws/src/mlm_avoid_bringup"

# ... 중간 생략 ...

# Emergency Stop 런치 추가
avoid_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(avoid_package_path, "launch/avoid.launch.py")),
)

return [
    robot_desc_launch,
    controller_node,
    peripherals_launch,
    imu_node,
    odom_node,
    avoid_launch,  # Emergency Stop (Nav2 자율주행용)
]
```

---

## 4. sync_node.py (GT 원본에서 수정)

**파일:** `robot/ros2_ws/src/mlm_avoid_sync/mlm_avoid_sync/sync_node.py`

### GT 원본
```python
class SyncNode(Node):
    def __init__(self):
        super().__init__('mlm_avoid_sync_node')

        self.declare_parameter('buffer_size', 30)
        self.declare_parameter('image_topic', '/ascamera/camera_publisher/rgb0/image')
        self.declare_parameter('scan_topic', '/scan_raw')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu')

        # ... 중간 생략 ...

        # Publisher (하드코딩)
        self.pub_synced = self.create_publisher(
            SyncedData,
            '/mlm_avoid/synced_data',
            10
        )
```

### DOG용 수정
```python
class SyncNode(Node):
    def __init__(self):
        super().__init__('mlm_avoid_sync_node')

        self.declare_parameter('buffer_size', 30)
        self.declare_parameter('image_topic', '/ascamera/camera_publisher/rgb0/image')
        self.declare_parameter('scan_topic', '/scan_raw')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('synced_data_topic', '/mlm_avoid/synced_data')  # 추가

        # ... 중간 생략 ...

        # Publisher (파라미터화)
        synced_data_topic = self.get_parameter('synced_data_topic').value
        self.pub_synced = self.create_publisher(
            SyncedData,
            synced_data_topic,
            qos_profile  # QoS: BEST_EFFORT로 변경
        )
```

### QoS 변경
```python
# GT 원본: 기본 QoS (RELIABLE)
self.pub_synced = self.create_publisher(SyncedData, '/mlm_avoid/synced_data', 10)

# DOG용: BEST_EFFORT QoS
from rclpy.qos import QoSProfile, ReliabilityPolicy
qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
self.pub_synced = self.create_publisher(SyncedData, synced_data_topic, qos_profile)
```

---

## 5. safety_node.py (GT 원본에서 수정)

**파일:** `robot/ros2_ws/src/mlm_avoid_safety/mlm_avoid_safety/safety_node.py`

### GT 원본
```python
class SafetyNode(Node):
    def __init__(self):
        super().__init__('mlm_avoid_safety_node')

        self.declare_parameter('ttc_threshold_start', 2.0)
        self.declare_parameter('ttc_threshold_end', 3.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)

        # 토픽 (하드코딩)
        self.sub_synced = self.create_subscription(
            SyncedData, '/mlm_avoid/synced_data', ...)
        self.sub_cmd_vel_nav = self.create_subscription(
            Twist, '/cmd_vel_nav', ...)
        self.pub_cmd_vel = self.create_publisher(
            Twist, '/cmd_vel', ...)
```

### DOG용 수정
```python
class SafetyNode(Node):
    def __init__(self):
        super().__init__('mlm_avoid_safety_node')

        self.declare_parameter('ttc_threshold_start', 2.0)
        self.declare_parameter('ttc_threshold_end', 3.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)

        # 토픽 파라미터 추가 (namespace 지원)
        self.declare_parameter('synced_data_topic', '/mlm_avoid/synced_data')
        self.declare_parameter('cmd_vel_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # 토픽 (파라미터에서 읽기)
        synced_data_topic = self.get_parameter('synced_data_topic').value
        cmd_vel_nav_topic = self.get_parameter('cmd_vel_nav_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.sub_synced = self.create_subscription(
            SyncedData, synced_data_topic, ...)
        self.sub_cmd_vel_nav = self.create_subscription(
            Twist, cmd_vel_nav_topic, ...)
        self.pub_cmd_vel = self.create_publisher(
            Twist, cmd_vel_topic, ...)
```

---

## 6. avoid_params.yaml (GT 원본에서 수정)

**파일:** `robot/ros2_ws/src/mlm_avoid_bringup/config/avoid_params.yaml`

### GT 원본
```yaml
mlm_avoid_sync_node:
  ros__parameters:
    buffer_size: 30
    image_topic: "/ascamera/camera_publisher/rgb0/image"
    scan_topic: "/scan_raw"
    odom_topic: "/odom"
    imu_topic: "/imu"

mlm_avoid_safety_node:
  ros__parameters:
    ttc_threshold_start: 2.0
    ttc_threshold_end: 3.0
    cmd_vel_timeout: 0.5
```

### DOG용 수정
```yaml
# Wildcard key for multi-robot namespace support
/**:
  mlm_avoid_sync_node:
    ros__parameters:
      buffer_size: 30
      image_topic: "/robot1/image_raw"      # DOG 카메라 토픽
      scan_topic: "/robot1/scan_raw"
      odom_topic: "/robot1/odom"
      imu_topic: "/robot1/imu"
      synced_data_topic: "/mlm_avoid/synced_data"

  mlm_avoid_safety_node:
    ros__parameters:
      ttc_threshold_start: 2.0
      ttc_threshold_end: 3.0
      cmd_vel_timeout: 0.5
      synced_data_topic: "/mlm_avoid/synced_data"
      cmd_vel_nav_topic: "/robot1/cmd_vel_nav"
      cmd_vel_topic: "/robot1/cmd_vel"
```

---

## 7. avoid.launch.py (GT 원본에서 수정)

**파일:** `robot/ros2_ws/src/mlm_avoid_bringup/launch/avoid.launch.py`

### GT 원본
```python
def generate_launch_description():
    pkg_dir = get_package_share_directory('mlm_avoid_bringup')
    params_file = os.path.join(pkg_dir, 'config', 'avoid_params.yaml')

    return LaunchDescription([
        Node(
            package='mlm_avoid_sync',
            executable='sync_node',
            name='mlm_avoid_sync_node',
            parameters=[params_file],
        ),
        Node(
            package='mlm_avoid_safety',
            executable='safety_node',
            name='mlm_avoid_safety_node',
            parameters=[params_file],
        ),
    ])
```

### DOG용 수정
```python
def generate_launch_description():
    pkg_dir = get_package_share_directory('mlm_avoid_bringup')
    params_file = os.path.join(pkg_dir, 'config', 'avoid_params.yaml')

    # Namespace 지원을 위한 PushRosNamespace 추가
    return LaunchDescription([
        PushRosNamespace('robot1'),
        Node(
            package='mlm_avoid_sync',
            executable='sync_node',
            name='mlm_avoid_sync_node',
            parameters=[params_file],
        ),
        Node(
            package='mlm_avoid_safety',
            executable='safety_node',
            name='mlm_avoid_safety_node',
            parameters=[params_file],
        ),
    ])
```

---

## 8. usb_cam_param.yaml

**파일:** `robot/ros2_ws/src/peripherals/config/usb_cam_param.yaml`

### 변경 전
```yaml
/**:
  ros__parameters:
    video_device: "/dev/video0"
    framerate: 30.0
    io_method: "mmap"
    frame_id: "camera"
    pixel_format: "yuyv"
    av_device_format: "YUV422P"
    image_width: 640
    image_height: 480
    camera_name: "usb_cam"
```

### 변경 후
```yaml
/**:
  ros__parameters:
    video_device: "/dev/video0"
    framerate: 30.0
    io_method: "mmap"
    frame_id: "camera"
    pixel_format: "yuyv"
    # av_device_format 제거 - swscaler 크래시 방지
    image_width: 640
    image_height: 480
    camera_name: "usb_cam"
```

---

## 9. CycloneDDS 설정

**파일들:** `pc/cyclonedds/cyclonedds.xml`, `robot/cyclonedds/cyclonedds.xml`

### 변경 전
```xml
<Peers>
  <Peer address="172.16.11.203"/>  <!-- PC -->
  <Peer address="172.16.10.172"/>  <!-- Robot1 -->
  <Peer address="172.16.10.173"/>  <!-- Robot2 -->
</Peers>
```

### 변경 후
```xml
<Peers>
  <Peer address="172.16.11.222"/>  <!-- PC (IP 변경) -->
  <Peer address="172.16.10.172"/>  <!-- Robot1 -->
  <Peer address="172.16.10.173"/>  <!-- Robot2 -->
</Peers>
```

---

## 최종 cmd_vel 흐름

```
[변경 전]
Nav2 controller_server → /robot1/cmd_vel → odom_publisher → Motors

[변경 후]
Nav2 controller_server → /robot1/cmd_vel_nav → safety_node → /robot1/cmd_vel → odom_publisher → Motors
                                                    ↑
                              sync_node → synced_data (TTC 계산용)
```
