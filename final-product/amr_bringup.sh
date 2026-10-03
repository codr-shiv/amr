#!/usr/bin/env bash
# =====================================================================
#  AMR one-command startup
#
#    ./amr_bringup.sh            FULL STACK: robot + Nav2 on the saved map (= nav)
#    ./amr_bringup.sh nav        robot + Nav2 on the saved map (MAP below)
#    ./amr_bringup.sh slam       robot + SLAM Toolbox (mapping; run teleop in another terminal)
#    ./amr_bringup.sh robot      robot only (agent, TF, diff drive, LiDAR)
#
#  Use another map:   MAP=/abs/path/other_map.yaml ./amr_bringup.sh
#
#  Ctrl-C stops EVERYTHING this script started.
#  Each program's output goes to its own log file in $LOG_DIR
#  (watch one live with:  tail -f ~/amr_logs/diff_drive.log).
#  Nav2's output is ALSO shown in this terminal (RTPS_READER_HISTORY spam removed).
#
#  Build the workspaces once before the first run:  scripts/build.sh
# =====================================================================

# Repo root = the folder this script lives in (works from any cwd, and via a symlink)
REPO_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"

# ------------------------- EDIT IF NEEDED ----------------------------
ROS_SETUP=/opt/ros/humble/setup.bash
WORKSPACES=(                          # sourced in this order
  "$REPO_DIR/microros_ws/install/setup.bash"   # micro-ROS agent
  "$REPO_DIR/amr_ws/install/setup.bash"        # amr_diff_drive, rplidar_ros, amr_navigation, ...
)
MAP="${MAP:-$REPO_DIR/maps/my_map.yaml}"       # map used in "nav" mode
AGENT_PORT=8888
LOG_DIR="${LOG_DIR:-$HOME/amr_logs}"
# ---------------------------------------------------------------------

MODE="${1:-nav}"
case "$MODE" in robot|slam|nav) ;; *) echo "Usage: $0 [nav|slam|robot]   (default: nav)"; exit 1 ;; esac

# ---- environment ----
source "$ROS_SETUP"
for ws in "${WORKSPACES[@]}"; do
  if [ -f "$ws" ]; then
    source "$ws" && echo "sourced $ws"
  else
    echo "!!! Workspace not built: $ws"
    echo "    Build it first:  $REPO_DIR/scripts/build.sh"
    exit 1
  fi
done
if [ "$MODE" = "nav" ] && [ ! -f "$MAP" ]; then
  echo "!!! Map not found: $MAP  (edit MAP at the top of this script, or run: MAP=/abs/path.yaml $0)"
  exit 1
fi
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset = 0>}  ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<unset = 0>}"
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
pkill -f "ros2 launch amr_navigation"
sleep 2
ros2 daemon stop > /dev/null 2>&1; ros2 daemon start > /dev/null 2>&1

# ---- 1. micro-ROS agent (ESP32 must be powered) ----
start agent ros2 run micro_ros_agent micro_ros_agent udp4 --port "$AGENT_PORT"
wait_topic /encoder_telemetry 60

# ---- 2. static TF base_link -> laser (LiDAR mount offsets live in laser_tf.launch.py) ----
start laser_tf ros2 launch amr_navigation laser_tf.launch.py

# ---- 3. diff drive controller ----
start diff_drive ros2 launch amr_diff_drive diff_drive.launch.py
wait_topic /odom 30

# ---- 4. RPLIDAR ----
start rplidar ros2 launch rplidar_ros rplidar_a1_launch.py
wait_topic /scan 30

# ---- 5. SLAM or Nav2 ----
if [ "$MODE" = "slam" ]; then
  start slam ros2 launch amr_navigation slam.launch.py
  echo ">>> SLAM running. In ANOTHER terminal:"
  echo "      source $REPO_DIR/amr_ws/install/setup.bash"
  echo "      ros2 run teleop_twist_keyboard teleop_twist_keyboard     (or: ros2 run amr_teleop wasd_teleop)"
  echo ">>> Save map:  ros2 run nav2_map_server map_saver_cli -f ${MAP%.yaml}"
elif [ "$MODE" = "nav" ]; then
  # Same as running by hand:
  #   ros2 launch amr_navigation nav_map.launch.py map:=$MAP use_composition:=True 2>&1 | grep -v RTPS_READER_HISTORY
  # but in its own process group (so Ctrl-C here stops it), shown here AND saved to $LOG_DIR/nav2.log.
  setsid bash -c 'ros2 launch amr_navigation nav_map.launch.py map:="$1" use_composition:=True 2>&1 \
                    | grep --line-buffered -v RTPS_READER_HISTORY | tee "$2"' \
    nav2 "$MAP" "$LOG_DIR/nav2.log" &
  NAV2_PID=$!
  PIDS+=($NAV2_PID)
  echo ">>> started nav2 (pid $NAV2_PID, map: $MAP, log: $LOG_DIR/nav2.log)"
  echo ">>> Nav2 starting. Waiting for localization..."
  until grep -q "lifecycle_manager_localization.*Managed nodes are active" "$LOG_DIR/nav2.log" 2>/dev/null; do
    if grep -q "Aborting bringup" "$LOG_DIR/nav2.log" 2>/dev/null; then
      echo "!!! Nav2 bringup FAILED. See: $LOG_DIR/nav2.log"; break
    fi
    if ! kill -0 "$NAV2_PID" 2>/dev/null; then
      echo "!!! Nav2 exited. See: $LOG_DIR/nav2.log"; break
    fi
    sleep 2
  done
  grep -q "lifecycle_manager_localization.*Managed nodes are active" "$LOG_DIR/nav2.log" && \
    echo ">>> Localization ACTIVE. Now in RViz: 2D Pose Estimate, then Nav2 Goal. (Do NOT run teleop.)"
fi

echo
echo "=== Running ($MODE mode). Press Ctrl-C here to stop everything. ==="
echo "    Live log example:  tail -f $LOG_DIR/diff_drive.log"
wait
