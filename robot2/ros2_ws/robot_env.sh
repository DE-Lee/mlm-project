#!/bin/bash
# Robot1 환경변수 설정 파일
# 로봇 Docker 내부 ~/.zshrc에서 추출 (2026-01-26)

# ROS2 기본 설정
export ROS_DOMAIN_ID=3
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0

# 로봇 설정
export MACHINE_TYPE=MentorPi_Acker

# 센서 설정
export LIDAR_TYPE=MS200
export DEPTH_CAMERA_TYPE=usb_cam

# 빌드 설정
export need_compile=False
export working_space=/home/ubuntu

# CycloneDDS 설정 (로봇 내부 경로)
# export CYCLONEDDS_URI=file:///root/cyclonedds/cyclonedds.xml
