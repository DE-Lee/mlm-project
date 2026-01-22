#!/bin/bash
set -e

echo "=== MLM Navigation PC Setup ==="

# 1. ROS2 의존성 설치
echo "[1/4] Installing ROS2 dependencies..."
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rosbridge-suite \
  ros-humble-tf-transformations \
  ros-humble-laser-filters

# 2. 워크스페이스 설정
echo "[2/4] Setting up workspace..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_WS=~/ros2_ws

mkdir -p $TARGET_WS/src
cp -r $SCRIPT_DIR/ros2_ws/src/* $TARGET_WS/src/

# 3. 빌드
echo "[3/4] Building workspace..."
cd $TARGET_WS
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select navigation slam

# 4. CycloneDDS 설정
echo "[4/4] Configuring CycloneDDS..."
mkdir -p ~/cyclonedds

echo ""
echo "=== Network Configuration ==="
echo "Available interfaces:"
ip addr show | grep -E "^[0-9]+:" | awk -F: '{print "  " $2}'
echo ""

read -p "Enter PC network interface (e.g., wlp0s20f3): " NET_IF
read -p "Enter PC IP address: " PC_IP
read -p "Enter Robot IP address: " ROBOT_IP

cat > ~/cyclonedds/cyclonedds.xml << EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="$NET_IF"/>
      </Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
      <FragmentSize>1280 B</FragmentSize>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="$ROBOT_IP"/>
        <Peer address="$PC_IP"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

# 5. 환경 변수 추가
SHELL_RC=~/.bashrc
if [ -f ~/.zshrc ]; then
  read -p "Use zsh? (y/n): " USE_ZSH
  if [ "$USE_ZSH" = "y" ]; then
    SHELL_RC=~/.zshrc
  fi
fi

if ! grep -q "ROS_DOMAIN_ID=3" $SHELL_RC; then
  echo "" >> $SHELL_RC
  echo "# MLM Navigation Environment" >> $SHELL_RC
  echo "source /opt/ros/humble/setup.bash" >> $SHELL_RC
  echo "source ~/ros2_ws/install/setup.bash" >> $SHELL_RC
  echo "export ROS_DOMAIN_ID=3" >> $SHELL_RC
  echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> $SHELL_RC
  echo "export CYCLONEDDS_URI=file://\$HOME/cyclonedds/cyclonedds.xml" >> $SHELL_RC
  echo "Environment variables added to $SHELL_RC"
fi

echo ""
echo "=== Setup Complete ==="
echo "Run: source $SHELL_RC"
echo "Then: ros2 launch navigation navigation_pc.launch.py map:=mlm_map.yaml robot_name:=robot1"
