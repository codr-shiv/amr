#!/usr/bin/env bash
# =====================================================================
#  AMR one-command startup
#
#    ./amr_bringup.sh            robot only (agent, TF, diff drive, LiDAR)
#    ./amr_bringup.sh slam       robot + SLAM Toolbox (mapping; run teleop in another terminal)
#    ./amr_bringup.sh nav        robot + Nav2 on the saved map (MAP below)
#
#  Ctrl-C stops EVERYTHING this script started.
#  Each program's output goes to its own log file in $LOG_DIR
#  (watch one live with:  tail -f ~/amr_logs/nav2.log).
# =====================================================================

# ------------------------- EDIT IF NEEDED ----------------------------
ROS_SETUP=/opt/ros/humble/setup.bash
WORKSPACES=(                          # sourced in this order if they exist
  "$HOME/microros_ws/install/setup.bash"
  "$HOME/amr-diff-drive/install/setup.bash"
  "$HOME/slam/install/setup.bash"
  "$HOME/nav_ws/install/setup.bash"
)
MAP="$HOME/maps/my_map.yaml"          # map used in "nav" mode
AGENT_PORT=8888
LASER_TF="--x 0.175 --y 0 --z 0.100 --roll 0 --pitch 0 --yaw 0"
LOG_DIR="$HOME/amr_logs"
# ---------------------------------------------------------------------

MODE="${1:-robot}"
case "$MODE" in robot|slam|nav) ;; *) echo "Usage: $0 [robot|slam|nav]"; exit 1 ;; esac

# ---- environment ----
source "$ROS_SETUP"
for ws in "${WORKSPACES[@]}"; do
  [ -f "$ws" ] && source "$ws" && echo "sourced $ws"
done
mkdir -p "$LOG_DIR"

PIDS=()
cleanup() {
  echo; echo ">>> Stopping everything..."
  for pid in "${PIDS[@]}"; do kill -INT -- "-$pid" 2>/dev/null; done   # polite Ctrl-C to each group
  sleep 5
  for pid in "${PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null; done  # force whatever is left
  echo ">>> All stopped."
  exit 0
}
trap cleanup INT TERM

# start NAME COMMAND...  -> runs in its own process group, logs to $LOG_DIR/NAME.log
start() {
  local name="$1"; shift
  setsid "$@" > "$LOG_DIR/$name.log" 2>&1 &
  PIDS+=($!)
  echo ">>> started $name (pid $!, log: $LOG_DIR/$name.log)"
}

# wait_topic TOPIC SECONDS -> waits until TOPIC delivers a message
wait_topic() {
  local topic="$1" limit="$2" t=0
  printf "    waiting for %s " "$topic"
  until timeout 5 ros2 topic echo --once "$topic" > /dev/null 2>&1; do
    t=$((t+5)); printf "."
    if [ "$t" -ge "$limit" ]; then echo " NOT READY after ${limit}s (continuing anyway)"; return 1; fi
  done
  echo " ok"
}

# ---- remove leftovers from a previous run (avoids duplicate nodes / port clash) ----
echo ">>> Cleaning up old processes"
pkill -f "micro_ros_agent udp4" ; pkill -f static_transform_publisher ; pkill -f diff_drive_controller
pkill -f rplidar ; pkill -f slam_toolbox ; pkill -f nav2 ; pkill -f lifecycle_manager
pkill -f amcl ; pkill -f map_server ; pkill -f teleop_twist_keyboard
sleep 2
ros2 daemon stop > /dev/null 2>&1; ros2 daemon start > /dev/null 2>&1

# ---- 1. micro-ROS agent (ESP32 must be powered) ----
start agent ros2 run micro_ros_agent micro_ros_agent udp4 --port "$AGENT_PORT"
wait_topic /encoder_telemetry 60

# ---- 2. static TF base_link -> laser ----
start laser_tf ros2 run tf2_ros static_transform_publisher $LASER_TF --frame-id base_link --child-frame-id laser

# ---- 3. diff drive controller ----
start diff_drive ros2 launch amr_diff_drive diff_drive.launch.py
wait_topic /odom 30

# ---- 4. RPLIDAR ----
start rplidar ros2 launch rplidar_ros rplidar_a1_launch.py
wait_topic /scan 30

# ---- 5. SLAM or Nav2 ----
if [ "$MODE" = "slam" ]; then
  start slam ros2 launch amr_navigation slam.launch.py
  echo ">>> SLAM running. In ANOTHER terminal:  ros2 run teleop_twist_keyboard teleop_twist_keyboard"
  echo ">>> Save map:  ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map"
elif [ "$MODE" = "nav" ]; then
  if [ ! -f "$MAP" ]; then echo "!!! Map not found: $MAP  (edit MAP at the top of this script)"; cleanup; fi
  start nav2 ros2 launch amr_navigation nav_map.launch.py map:="$MAP" use_composition:=True
  echo ">>> Nav2 starting. Waiting for localization..."
  until grep -q "lifecycle_manager_localization.*Managed nodes are active" "$LOG_DIR/nav2.log" 2>/dev/null; do
    if grep -q "Aborting bringup" "$LOG_DIR/nav2.log" 2>/dev/null; then
      echo "!!! Nav2 bringup FAILED. See: grep -v RTPS_READER_HISTORY $LOG_DIR/nav2.log"; break
    fi
    sleep 2
  done
  grep -q "lifecycle_manager_localization.*Managed nodes are active" "$LOG_DIR/nav2.log" && \
    echo ">>> Localization ACTIVE. Now in RViz: 2D Pose Estimate, then Nav2 Goal. (Do NOT run teleop.)"
fi

echo
echo "=== Running ($MODE mode). Press Ctrl-C here to stop everything. ==="
echo "    Live log example:  tail -f $LOG_DIR/nav2.log | grep -v RTPS_READER_HISTORY"
wait
