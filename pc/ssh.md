Host dog
  HostName 172.16.10.172
  User pi
  pw raspberrypi

Host gt
  HostName 172.16.10.37
  User pi
  pw raspberrypi

sshpass 로 ssh host에 연결해서
내부 docker MentorPi container 환경에서 작업한다.

현재 구현중인 시스템인데
ssh.md로 접속한 ssh docker가 MentorPi #1의 컨테이너다.

로봇은 ros2 launch bringup bringup_ns.launch.py robot_name:=robot1 이 명령어로 처음 시작되고, 

로컬 pc에서 ros2 launch navigation navigation_pc.launch.py map:=mlm_slam.yaml robot_name:=robot1 이 명령어로 navigation을 실행시키며,  로컬 pc는 /home/lee/mlm_ws/pc 이 주소의 파일을 사용한다.

web-dashboard는
/home/lee/mlm_ws/web 이 폴더 내부에 있다.

npm run dev로 실행시킨 후 초기 위치 설정 후 목표점을 지정하면 실제 로봇이
목표점을 향해 주행해야 한다.

위의 시스템이 정상적으로 실행되는지 확인하고
현재 내부에 있는 모든 코드를 살펴서
멀티로봇 환경을 구현하는데에 문제가 있는지 확인해라

먼저 로봇의 namespace 설정과, frame_prefix를 
ros2  멀티로봇 환경 구축 전문가의 관점에서 작성이 잘되어있는지 확인하고,
되어있지 않다면, 직접 되게끔 구동해라.
로봇의 명령어로 시작되는 모든 과정을 하나도 빠짐없이 확인해라.
container는 zsh환경에서 진행되며,  로봇을 구동하면서 사용되는 런치파일들과  불러오는 패키지들, 환경 변수,
config파일들 등 로봇이 실제로 구동하면서 필요한 모든 정보들과 cycloneDDS를 통한 네트워크 통신을 위한 설정들을 확인하고,
흐름에 따라 전부 상세히 확인하고 점검한다.

점검이 완료되면 내게 찾은 모든 문제점을 보고하고 수정을 진행한다.

이후 로봇의 세팅이 완벽하면 로봇의 세팅에 맞춰 /home/lee/mlm_ws/pc에 있는 로컬pc 파일들을 점검한다. 마찬가지로 시스템의 흐름에 따라 점검하며, 로봇과 송수신하는 모든 일련의 과정들을 점검한다. 사용하는 파일들과 그 파일들이 사용하는 모든 파일들, 연결되어있는 모든 파일들과 사용하는 config, 환경변수, cycloneDDS, 네트워크 환경을 점검하고, 위에서 말한 전문가의 관점으로 내게 찾은 모든 문제점을 보고하고 수정을 진행한다.

pc와 로봇의 세팅이 완료되면 웹 대시보드를 점검한다.
프론트엔드 전문가의 관점을 추가하여, 점검후 완료되면 내게 찾은 모든 문제점을 보고하고 수정을 진행한다.

pc에서 robot2/odom
/tf에서 EKF TF만 누락

 문제 분석 및 해결 요약
원인
CycloneDDS 설정 불일치

로봇: AllowMulticast: spdp, FragmentSize: 1280B
PC (이전): AllowMulticast: true, FragmentSize: 4000B
WiFi 환경에서 멀티캐스트 제한으로 인해 일부 토픽 데이터 손실

해결
PC의 CycloneDDS 설정을 로봇과 일치시킴
ROS2 daemon 재시작
AMCL "Message Filter dropping" 오류 원인
이 오류는 TF 변환이 제때 도착하지 않아 scan 메시지가 큐에 쌓이는 문제였습니다. 이제 TF가 정상 수신되므로 Navigation을 다시 실행하면 해결될 것입니다.



방향 불일치 발견!

AMCL: yaw ≈ 97° (왼쪽 방향)
Odom: yaw ≈ 0° (전방 방향)
AMCL과 Odometry의 방향이 ~90도 차이납니다. 이것이 목표 방향과 다르게 주행하는 원인입니다.


IMU 캘리브레이션 파일을 확인했습니다. 
중요한 발견:

SM: 가속도계 스케일/교차축 보정 매트릭스 (3x3)
bias: 가속도계 바이어스 보정
문제점: -90° 회전 오프셋 보정이 없습니다!

현재 상태
AMCL pose: yaw ≈ 97° (맵 기준 로봇 방향)
Odom pose: yaw ≈ 0° (EKF 출력)
map→odom TF: 97.4° 회전 (AMCL의 보정 적용)
핵심 발견
TF 체인과 프레임 설정은 올바릅니다. ~97° 차이는 정상적인 AMCL 동작입니다.

그러나 IMU 캘리브레이션 파일에 회전 보정이 없습니다:


# /home/ubuntu/ros2_ws/src/calibration/config/imu_calib.yaml
SM: [가속도계 스케일 매트릭스]  # 회전 보정 없음!
bias: [가속도계 바이어스]
의심되는 문제점
IMU 방향 처리 문제:

IMU가 base_link 기준 -90° 회전되어 장착
TF에는 반영됨, 하지만 EKF가 IMU 데이터를 올바르게 변환하는지 확인 필요
로봇을 직접 돌려서 테스트 필요:

로봇을 90° 회전시키면 odom yaw가 정확히 90° 변하는지 확인
디버깅 방법
로봇에서 다음 테스트를 해보세요:


# 1. 로봇을 손으로 90° 회전시키면서 odom yaw 변화 관찰
ros2 topic echo /robot1/odom --field pose.pose.orientation

# 2. IMU yaw 변화 관찰 (동시에)
ros2 topic echo /robot1/imu --field orientation
만약 로봇을 90° 회전했는데:

odom yaw가 90° 변하면 → 정상
odom yaw가 다른 각도로 변하면 → IMU 융합 문제

중대한 문제 발견!
로봇을 물리적으로 회전시켰을 때:

센서	변화
IMU orientation	z: 0.508 → -0.018 (약 60° 회전 감지)
Odom orientation	z: 0.00023 → 0.00023 (변화 없음!)
문제: EKF가 IMU yaw를 odometry에 반영하지 않고 있습니다!

원인 분석
odom_publisher_node.py:274를 보면:


delta_yaw = self.angular_z * self.dt  # cmd_vel의 angular_z만 사용
self.pose_yaw += delta_yaw
odom_raw는 cmd_vel 명령 기반으로만 yaw를 계산합니다. 물리적 회전은 반영되지 않습니다.

EKF 설정에서:

imu0_relative: true - IMU yaw를 상대값으로 처리
odom0에서 yaw pose를 사용하지 않음
결과: 로봇이 물리적으로 회전해도 odometry yaw가 변하지 않아, Nav2가 잘못된 방향으로 명령을 내립니다.

해결 방법
EKF 설정에서 IMU yaw를 절대값으로 사용하도록 변경해야 합니다.

Read ekf.yaml (lines 161-190)
핵심 문제: imu0_relative: true 설정으로 인해 IMU yaw가 상대값으로 처리되어 실제 회전이 odometry에 반영되지 않습니다.

문제 발견!

odom_raw가 angular velocity z = 0을 매우 높은 신뢰도로 보고하고 있습니다.

odom_publisher_node.py:40-46에서:


ODOM_TWIST_COVARIANCE_STOP = [..., 1e-9]  # vyaw 신뢰도 매우 높음!
로봇이 정지 상태일 때 odom_raw는 "vyaw=0, 신뢰도 99.9999999%"라고 보고합니다. EKF가 이를 IMU보다 더 신뢰하고 있습니다.

해결: odom_raw에서 vyaw 비활성화

문제 발견!

imu_complementary_filter 문서에 따르면, fixed_frame은 TF 발행용이고, IMU 메시지의 frame_id는 입력 메시지의 header에서 가져옵니다.

즉, imu_corrected 토픽의 frame_id를 확인해야 합니다.

원인 발견!

IMU raw 데이터 소스인 ros_robot_controller가 frame_id: imu_link로 발행합니다.

ros_robot_controller를 확인합니다.

해결책 발견!

ros_robot_controller_node.py에서:

36행: imu_frame 파라미터 선언 (기본값: imu_link)
345행: IMU 메시지의 frame_id 설정

문제 발견!

controller.launch.py에서 imu_frame이 imu_link로 고정되어 있습니다. frame_prefix가 적용되지 않습니다!

로컬 파일을 수정합니다.



robot1 dog

 # ros2 topic echo /robot1/imu --once
header:
  stamp:
    sec: 1769565229
    nanosec: 830812403
  frame_id: robot1/imu_link
orientation:
  x: -0.02082683692743063
  y: 0.004976627399570379
  z: 0.0001898249194100702
  w: 0.9997706937142258
orientation_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
angular_velocity:
  x: -0.0040133199536340584
  y: -0.0035003269787333173
  z: 0.0006304620477081439
angular_velocity_covariance:
- 0.01
- 0.0
- 0.0
- 0.0
- 0.01
- 0.0
- 0.0
- 0.0
- 0.01
linear_acceleration:
  x: -0.09093027142065893
  y: -0.3971448635024256
  z: 9.794786507902934
linear_acceleration_covariance:
- 0.0004
- 0.0
- 0.0
- 0.0
- 0.0004
- 0.0
- 0.0
- 0.0
- 0.004
---

imu frame_prefix 완료


멀티로봇 분리 상태 확인 완료:

모든 시스템이 올바르게 분리되어 있습니다:

확인 항목	robot1	robot2	상태
AMCL 위치	(8.93, 5.53)	(2.02, 7.25)	✅ 분리됨
TF base_frame	robot1/base_footprint	robot2/base_footprint	✅
Nav2 bt_navigator	/robot1/goal_pose 구독	/robot2/goal_pose 구독	✅
Emergency Stop	mlm_avoid_safety_node → cmd_vel	동일 구조	✅
토픽 namespace	/robot1/*	/robot2/*	✅
