# mlm_avoid_safety

TTC(Time To Collision) 기반 Safety 노드 패키지.

## 개요

동기화된 센서 데이터를 받아 충돌까지 남은 시간(TTC)을 계산하고,
위험 상황에서 주행 명령 대신 긴급 정지를 적용한다.

## 노드

### safety_node

TTC 기반 개입 판단 및 cmd_vel mux 노드.

#### 구독 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/mlm_avoid/synced_data` | mlm_avoid_msgs/SyncedData | 동기화된 센서 데이터 |
| `/cmd_vel_nav` | geometry_msgs/Twist | 주행 명령 (path_player 등) |

#### 발행 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/cmd_vel` | geometry_msgs/Twist | 최종 로봇 명령 |

#### 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `ttc_threshold_start` | float | 2.0 | 개입 시작 TTC (초) |
| `ttc_threshold_end` | float | 3.0 | 개입 종료 TTC (초) |
| `cmd_vel_timeout` | float | 0.5 | 명령 타임아웃 (초) |

## 핵심 클래스

### TTCCalculator

TTC 계산 및 개입 판단 로직.

```python
from mlm_avoid_safety.ttc_calculator import TTCCalculator

calc = TTCCalculator(ttc_threshold_start=2.0, ttc_threshold_end=3.0)

# 최소 TTC 계산 (360도 전체 스캔)
min_ttc = calc.calculate_min_ttc(
    ranges=scan.ranges,
    angle_min=scan.angle_min,
    angle_increment=scan.angle_increment,
    vx=odom.twist.twist.linear.x,
    vy=odom.twist.twist.linear.y
)

# 개입 여부 결정 (히스테리시스 적용)
should_intervene = calc.should_intervene(
    ttc=min_ttc,
    currently_intervening=self.intervening
)
```

## TTC 계산 방식

1. 360도 LiDAR 스캔의 각 방향에 대해:
   - 해당 방향으로의 접근 속도 계산 (속도 벡터와 방향 벡터의 내적)
   - TTC = 거리 / 접근 속도
2. 모든 방향 중 최소 TTC 선택
3. 접근하지 않는 방향(멀어지는 방향)은 TTC = ∞

## 히스테리시스

경계값 근처에서 왔다갔다 하는 것을 방지:

```
TTC < ttc_threshold_start (2.0초) → 개입 시작 (긴급 정지)
TTC > ttc_threshold_end (3.0초)   → 개입 종료 (주행 재개)
그 사이                           → 현재 상태 유지
```

## cmd_vel Mux 로직

```
if 개입 중 (TTC < 2초):
    /cmd_vel = 정지 (0, 0)
else (TTC > 3초):
    /cmd_vel = /cmd_vel_nav (주행 명령 통과)
```

## 실행

```bash
ros2 run mlm_avoid_safety safety_node
```

## 테스트

```bash
cd /workspaces/ros2
python3 -m pytest src/mlm_avoid_safety/test/ -v
```
