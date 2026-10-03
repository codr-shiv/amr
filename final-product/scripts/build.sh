#!/usr/bin/env bash
# =====================================================================
#  Build the AMR workspaces (run on the Raspberry Pi, ROS 2 Humble).
#
#    scripts/build.sh              build everything (micro-ROS agent + amr_ws)
#    scripts/build.sh agent        build only the micro-ROS agent workspace
#    scripts/build.sh amr [ARGS]   build only amr_ws; ARGS go to colcon,
#                                  e.g.  scripts/build.sh amr --packages-select amr_navigation
#
#  First time on a fresh Pi: run scripts/install_deps.sh before this.
# =====================================================================
set -e

REPO_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
ROS_SETUP=/opt/ros/humble/setup.bash
source "$ROS_SETUP"

TARGET="${1:-all}"
[ $# -gt 0 ] && shift

build_agent() {
  echo ">>> Building micro-ROS agent workspace: $REPO_DIR/microros_ws"
  cd "$REPO_DIR/microros_ws"
  # Same two steps as the official micro-ROS agent setup:
  # 1) the setup package itself, 2) its build_agent.sh (special CMake flags for the agent).
  colcon build --packages-select micro_ros_setup
  source install/local_setup.bash
  ros2 run micro_ros_setup build_agent.sh
}

build_amr() {
  echo ">>> Building main workspace: $REPO_DIR/amr_ws"
  cd "$REPO_DIR/amr_ws"
  colcon build --symlink-install "$@"
}

case "$TARGET" in
  all)   build_agent; build_amr "$@" ;;
  agent) build_agent ;;
  amr)   build_amr "$@" ;;
  *)     echo "Usage: $0 [all|agent|amr [colcon args]]"; exit 1 ;;
esac

echo
echo ">>> Build done. Start the robot with:  $REPO_DIR/amr_bringup.sh"
