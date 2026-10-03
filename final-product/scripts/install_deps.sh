#!/usr/bin/env bash
# =====================================================================
#  One-time dependency install for a fresh Raspberry Pi
#  (Ubuntu 22.04 + ROS 2 Humble already installed in /opt/ros/humble).
#  Then build with:  scripts/build.sh
# =====================================================================
set -e

REPO_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"

echo ">>> apt packages"
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep python3-vcstool git build-essential \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-teleop-twist-keyboard

echo ">>> rosdep"
sudo rosdep init 2>/dev/null || true
rosdep update

source /opt/ros/humble/setup.bash

# Main workspace (-r: keep going if one key can't be resolved)
rosdep install --from-paths "$REPO_DIR/amr_ws/src" --ignore-src -y -r

# micro-ROS agent workspace (same command/skip-keys as micro_ros_setup's create_agent_ws.sh)
SKIP="rosidl_typesupport_opensplice_c rosidl_typesupport_opensplice_cpp rmw_opensplice_cpp rmw_connext_cpp rosidl_typesupport_connext_c rosidl_typesupport_connext_cpp microxrcedds_agent microxrcedds_client microcdr rmw_connextdds"
rosdep install --os=ubuntu:jammy --from-paths "$REPO_DIR/microros_ws/src" -i -y --skip-keys="$SKIP"

echo ">>> Dependencies installed. Next:  $REPO_DIR/scripts/build.sh"
