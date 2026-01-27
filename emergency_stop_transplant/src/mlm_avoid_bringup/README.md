# mlm_avoid_bringup

긴급 정지 시스템 통합 launch 패키지.

## 개요

mlm_avoid 시스템과 path_player를 함께 실행하기 위한 launch 파일과
공통 파라미터 설정을 제공한다.

## 디렉토리 구조

```
mlm_avoid_bringup/
├── config/
│   └── avoid_params.yaml      # 파라미터 설정
└── launch/
    ├── full_system.launch.py  # path_player + safety 통합 실행
    ├── avoid.launch.py        # safety만 실행
    └── safety_only.launch.py  # safety만 실행 (avoid.launch.py와 동일)
```

## Launch 파일

### full_system.launch.py (권장)

path_player (주행) + mlm_avoid (긴급 정지) 통합 실행.

**실행되는 노드:**
- `mlm_avoid_sync_node` - 센서 동기화
- `mlm_avoid_safety_node` - TTC 기반 긴급 정지
- `path_player_server` - 경로 추종 서버
- `motion_controller` - 주행 제어기

```bash
ros2 launch mlm_avoid_bringup full_system.launch.py
```

### avoid.launch.py / safety_only.launch.py

Safety 노드만 실행 (다른 주행 시스템과 연동 시).

**실행되는 노드:**
- `mlm_avoid_sync_node` - 센서 동기화
- `mlm_avoid_safety_node` - TTC 기반 긴급 정지

```bash
ros2 launch mlm_avoid_bringup avoid.launch.py
```

### Launch Arguments

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `params_file` | config/avoid_params.yaml | 파라미터 파일 경로 |
| `ttc_start` | 2.0 | 개입 시작 TTC (초) |
| `ttc_end` | 3.0 | 개입 종료 TTC (초) |

## 시스템 구조

```
              ┌─────────────────────────────────────────────────────┐
              │                      Sensors                         │
              └─────────────────────────────────────────────────────┘
                 │         │         │         │
                 ▼         ▼         ▼         ▼
              /image    /scan     /odom     /imu
                 │         │         │         │
                 └────┬────┴────┬────┴────┬────┘
                      │         │         │
                      ▼         ▼         ▼
              ┌─────────────────────────────────────────────────────┐
              │              mlm_avoid_sync_node                     │
              │              (센서 데이터 동기화)                      │
              └─────────────────────────────────────────────────────┘
                                      │
                                      ▼
                          /mlm_avoid/synced_data
                                      │
                                      ▼
              ┌─────────────────────────────────────────────────────┐
              │             mlm_avoid_safety_node                    │
              │           (TTC 계산, cmd_vel mux)                    │
              └─────────────────────────────────────────────────────┘
                                      ▲
                                      │ /cmd_vel_nav
              ┌─────────────────────────────────────────────────────┐
              │               path_player_server                     │
              │                      │                               │
              │                      ▼                               │
              │               motion_controller                      │
              └─────────────────────────────────────────────────────┘
                                      │
                                      ▼
                                  /cmd_vel
                                      │
                                      ▼
                                   [로봇]
```

## 동작 흐름

1. **정상 주행**: TTC > 3초 → `/cmd_vel` = `/cmd_vel_nav` (주행 명령 통과)
2. **위험 감지**: TTC < 2초 → `/cmd_vel` = (0, 0) (긴급 정지)
3. **위험 해소**: TTC > 3초 → `/cmd_vel` = `/cmd_vel_nav` (주행 재개)

## 사용 예시

### 기본 실행

```bash
# 전체 시스템 (path_player + safety)
ros2 launch mlm_avoid_bringup full_system.launch.py

# Safety만 (다른 주행 시스템 사용 시)
ros2 launch mlm_avoid_bringup avoid.launch.py
```

### 파라미터 변경

```bash
# TTC 임계값 변경
ros2 launch mlm_avoid_bringup full_system.launch.py ttc_start:=1.5 ttc_end:=2.5
```

### 토픽 확인

```bash
# 동기화된 데이터 발행 확인
ros2 topic hz /mlm_avoid/synced_data

# 최종 cmd_vel 확인
ros2 topic echo /cmd_vel
```

## 빌드

```bash
colcon build --packages-select mlm_avoid_bringup
```

## 의존성

- mlm_avoid_msgs
- mlm_avoid_sync
- mlm_avoid_safety
- path_player_pkg
