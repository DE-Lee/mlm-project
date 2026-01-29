# Synchronized sensor data for emergency stop system
# 긴급 정지 시스템을 위한 동기화된 센서 데이터
#
# All sensor data is approximately time-synchronized.
# 모든 센서 데이터는 대략적으로 시간 동기화되어 있음.

# Header with synchronized timestamp
# 동기화된 타임스탬프를 가진 헤더
std_msgs/Header header

# Camera image (RGB)
# 카메라 이미지 (RGB)
sensor_msgs/Image image

# LiDAR scan (360 degrees)
# LiDAR 스캔 (360도)
sensor_msgs/LaserScan scan

# Odometry (position and velocity)
# 오도메트리 (위치 및 속도)
nav_msgs/Odometry odom

# IMU (acceleration and angular velocity)
# IMU (가속도 및 각속도)
sensor_msgs/Imu imu
