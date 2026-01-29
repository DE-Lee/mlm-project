# Perception Bridge 노드 구현 지침서

이 문서는 ROS2 Humble 환경에서 동작하는 Perception Bridge 노드 구현을 위한 지침서입니다.

## 프로젝트 목표

YOLO Detection 결과와 LiDAR 데이터를 융합하여, 로봇 주변의 객체(장애물 및 미션 객체) 정보를 가공하고 ROS2 토픽으로 발행하는 노드를 개발합니다. 이 노드의 출력은 학습 기반 로컬 플래너의 입력으로 사용됩니다.

## 시스템 아키텍처에서의 위치

```
[YOLO 추론 노드] → /detections (vision_msgs/Detection2DArray)
                          ↘
                           [Perception Bridge] → /perception/objects (ObjectArray)
                          ↗                            ↓
[LiDAR 드라이버] → /scan (sensor_msgs/LaserScan)      [data_logger.py] → CSV 파일
                                                       ↓
                                                [학습 파이프라인]
```

Perception Bridge는 센서 데이터를 가공하는 중간 계층 노드입니다. YOLO 추론은 별도 노드가 담당하며, 이 노드는 그 결과를 수신해서 LiDAR와 융합합니다.

## 구현 언어 및 환경

Python(rclpy)으로 구현합니다. 대상 환경은 ROS2 Humble입니다.

## 의존성 제약사항

ROS2는 colcon 기반의 전역 환경을 사용하므로 외부 패키지 설치에 매우 신중해야 합니다. 잘못된 의존성 하나가 전체 워크스페이스를 망가뜨릴 수 있습니다.

허용되는 의존성은 ROS2 Humble에 기본 포함된 것들로 한정합니다. rclpy, std_msgs, sensor_msgs, geometry_msgs, vision_msgs, visualization_msgs, tf2_ros, tf2_geometry_msgs, message_filters, 그리고 numpy가 포함됩니다. 추가로 이 프로젝트에서 정의한 perception_msgs 패키지를 사용합니다.

금지되는 의존성은 pip으로 별도 설치가 필요한 모든 패키지입니다. ultralytics, PyTorch, TensorFlow, Open3D, filterpy, scipy 등이 해당됩니다.

칼만 필터가 필요하다면 numpy만으로 직접 구현해야 합니다.

## 커스텀 메시지 정의 (perception_msgs 패키지)

Perception Bridge가 발행하는 메시지는 별도의 perception_msgs 패키지에 정의되어 있습니다. 이 패키지는 perception_bridge보다 먼저 빌드되어 있어야 합니다.

### Object.msg

단일 감지 객체를 나타내는 메시지입니다.

```
# 단일 감지 객체
# Perception Bridge가 YOLO + LiDAR 융합으로 추정한 객체 정보

# 로봇 기준 상대 좌표 (미터)
# rel_x > 0: 로봇 전방
# rel_y > 0: 로봇 좌측
float32 rel_x
float32 rel_y

# 객체 클래스
# 0 = 빈 슬롯 (고정 배열에서 사용되지 않는 원소)
# 1 = 장애물 (보행자, 휠체어, 침대 등 피해야 하는 객체)
# 2 = 미션 객체 (엘리베이터 버튼 등)
uint8 object_class

# 클래스 상수 정의
uint8 CLASS_EMPTY = 0
uint8 CLASS_OBSTACLE = 1
uint8 CLASS_MISSION = 2
```

### ObjectArray.msg

10개의 객체를 담는 고정 크기 배열 메시지입니다.

```
# 감지된 객체 배열 (고정 크기 10개)
# Perception Bridge가 발행하는 메인 출력 메시지

# 타임스탬프 및 좌표계 정보
std_msgs/Header header

# 감지된 객체 배열 (거리순 정렬, 가까운 것부터)
# 객체가 10개 미만이면 나머지는 빈 슬롯 (object_class=0)
Object[10] objects
```

## 입력 토픽

### YOLO Detection 결과

토픽 이름은 파라미터로 설정 가능해야 하며 기본값은 `/detections`입니다. 메시지 타입은 `vision_msgs/Detection2DArray`입니다.

각 Detection에서 사용할 정보는 BBox 좌표(center, size_x, size_y), class_id, confidence score입니다. class_id를 기반으로 장애물(class=1)인지 미션 객체(class=2)인지 분류합니다. 어떤 YOLO class_id가 어떤 object_class에 매핑되는지는 파라미터로 설정 가능해야 합니다.

### LiDAR 데이터

토픽 이름은 파라미터로 설정 가능해야 하며 기본값은 `/scan`입니다. 메시지 타입은 `sensor_msgs/LaserScan`입니다.

## 출력 토픽

토픽 이름 기본값은 `/perception/objects`입니다. 메시지 타입은 `perception_msgs/ObjectArray`입니다.

### 출력 규칙

배열 크기는 항상 10개로 고정입니다. 감지된 객체는 거리순으로 정렬하여 가까운 것부터 배치합니다. 객체가 10개 미만이면 나머지 슬롯은 빈 슬롯(object_class=0, rel_x=0.0, rel_y=0.0)으로 채웁니다. 객체가 10개를 초과하면 가장 가까운 10개만 포함합니다.

## 핵심 알고리즘

### LiDAR-BBox 융합

YOLO가 감지한 2D BBox를 LiDAR 데이터와 융합하여 객체의 3D 위치(로봇 기준 상대 좌표)를 추정합니다.

처리 과정은 다음과 같습니다. 카메라 BBox의 수평 범위를 LiDAR 각도 범위로 매핑합니다. 해당 각도 범위 내의 LiDAR 포인트들을 추출합니다. 추출된 포인트들 중 최근접점 또는 중심점을 객체 위치로 결정합니다.

필요한 calibration 정보는 카메라 intrinsic 파라미터(fx, fy, cx, cy)와 카메라-LiDAR 간 extrinsic 변환입니다. 이 정보는 파라미터 또는 TF를 통해 제공받습니다.

### 객체 분류

YOLO class_id를 object_class로 매핑합니다. 매핑 규칙은 파라미터로 설정합니다. 예를 들어 YOLO class 0(person), 1(wheelchair), 2(bed)는 object_class=1(장애물)로, YOLO class 5(elevator_button)는 object_class=2(미션)로 매핑할 수 있습니다.

### 거리 계산 및 정렬

각 객체의 거리는 sqrt(rel_x² + rel_y²)로 계산합니다. 모든 유효 객체를 거리순으로 정렬하여 가까운 것부터 배열에 배치합니다.

### 속도 추정 (선택)

객체의 속도(rel_vx, rel_vy)는 출력 메시지에 포함되지 않습니다. 필요한 경우 연속된 프레임 간의 위치 차이를 이용해 학습 단계 또는 추론 단계에서 계산합니다.

만약 노드 내부에서 속도 추정이 필요하다면 칼만 필터를 numpy로 직접 구현해야 합니다. filterpy 등 외부 라이브러리는 사용하지 않습니다.

## 파라미터 설계

params.yaml에서 설정 가능해야 하는 파라미터들입니다.

입력 관련으로는 detection_topic(기본값 "/detections")과 lidar_topic(기본값 "/scan")이 있습니다.

출력 관련으로는 output_topic(기본값 "/perception/objects")과 output_frame(기본값 "base_link")이 있습니다.

객체 분류 관련으로는 obstacle_class_ids(장애물로 분류할 YOLO class_id 리스트)와 mission_class_ids(미션 객체로 분류할 YOLO class_id 리스트)가 있습니다.

융합 관련으로는 max_detection_range_m(기본값 10.0)과 min_confidence(기본값 0.5)가 있습니다.

디버깅 관련으로는 enable_visualization(기본값 false)이 있습니다.

## 좌표계 규칙

모든 출력은 로봇 기준 좌표계(base_link)입니다. rel_x > 0이면 로봇 전방이고, rel_y > 0이면 로봇 좌측입니다. 필요시 TF2를 사용하여 센서 좌표계에서 base_link로 변환합니다.

## 동기화

Detection 메시지와 LiDAR 메시지는 서로 다른 주기로 발행될 수 있습니다. message_filters의 ApproximateTimeSynchronizer를 사용하여 두 메시지를 시간적으로 정렬한 뒤 처리하는 것을 권장합니다.

## 패키지 구조

```
perception_bridge/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── perception_bridge
├── launch/
│   └── perception_bridge.launch.py
├── config/
│   └── params.yaml
└── perception_bridge/
    ├── __init__.py
    ├── perception_bridge_node.py
    ├── lidar_bbox_fusion.py
    └── utils.py
```

## 디버깅 지원 (선택)

enable_visualization 파라미터가 true일 때, visualization_msgs/MarkerArray를 사용하여 RViz에서 객체 위치와 클래스를 시각화하는 기능이 있으면 유용합니다.

## 참고 파일

이 디렉터리에 포함된 참고 파일들입니다.

data_logger.py는 Perception Bridge가 발행하는 ObjectArray를 구독하여 CSV로 저장하는 노드입니다. 이 파일을 참고하여 Perception Bridge의 출력이 data_logger와 호환되는지 확인할 수 있습니다.

pedestrian_patrol.py와 smart_teleop_logger.py는 시뮬레이션 환경에서 사용되는 보조 노드들입니다. 직접적인 연관은 없지만 전체 시스템 맥락을 이해하는 데 도움이 됩니다.

## 명확한 제외 사항

다음은 이 노드의 구현 범위에 포함되지 않습니다.

YOLO 모델 학습 및 추론은 별도 노드/환경에서 처리합니다. 전역 경로 계획(Global Planning)과 모터 제어(PID/MPC)는 다른 모듈이 담당합니다. SLAM 지도 생성도 별도입니다. 학습 기반 로컬 플래너 모델 추론은 Nav2 플러그인에서 처리합니다. CSV 저장은 data_logger.py가 담당합니다.
