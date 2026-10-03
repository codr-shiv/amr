# System bringup, TF and teleop

How the whole robot is started and stopped by `amr_bringup.sh`, how the workspaces are built and sourced, the TF tree,
logs, and keyboard teleop.

Code: [`amr_bringup.sh`](../../amr_bringup.sh), [`scripts/build.sh`](../../scripts/build.sh),
[`scripts/install_deps.sh`](../../scripts/install_deps.sh), [`amr_ws/src/amr_teleop/`](../../amr_ws/src/amr_teleop/).

---

## 1. Workspaces and build

| Workspace | Packages | Built with |
|---|---|---|
| `microros_ws/` | `micro_ros_setup`, `micro_ros_msgs`, `micro_ros_agent` | `scripts/build.sh agent`: `colcon build --packages-select micro_ros_setup`, source it, `ros2 run micro_ros_setup build_agent.sh` (`--packages-up-to micro_ros_agent -DUAGENT_BUILD_EXECUTABLE=OFF -DUAGENT_P2P_PROFILE=OFF`) |
| `amr_ws/` | `amr_diff_drive`, `amr_navigation`, `amr_teleop`, `rplidar_ros` | `scripts/build.sh amr [colcon args]`: `colcon build --symlink-install` |

`scripts/build.sh` (no argument) builds both. `--symlink-install` means edits to Python, launch and YAML files take
effect on the next launch without rebuilding (C++ `rplidar_ros` and new files still need a rebuild).

`scripts/install_deps.sh` (once per Pi): apt (`colcon`, `rosdep`, `vcstool`, Nav2, nav2-bringup, SLAM Toolbox,
teleop_twist_keyboard), `rosdep init/update`, `rosdep install` for both workspaces (micro-ROS with the skip-keys
`micro_ros_setup` itself uses).

Both scripts locate the repo from their own path (`readlink -f`), so they work from any directory and through the
`~/amr` symlink.

## 2. `amr_bringup.sh` step by step

```bash
./amr_bringup.sh [nav|slam|robot]      # default nav;  MAP=... and LOG_DIR=... can be set in the environment
```

1. **Arguments / config:** `REPO_DIR` from the script's real path; `MAP` default `$REPO_DIR/maps/my_map.yaml`;
   `AGENT_PORT=8888`; `LOG_DIR` default `~/amr_logs`. Unknown mode → usage, exit 1.
2. **Environment:** `source /opt/ros/humble/setup.bash`, then `microros_ws/install/setup.bash` and `amr_ws/install/setup.bash`.
   A missing workspace → "Workspace not built", exit 1. In nav mode a missing map → exit 1 (before anything starts).
   Prints `ROS_DOMAIN_ID` and `ROS_LOCALHOST_ONLY` (inherited from your shell).
3. **Cleanup of leftovers** from a previous run: `pkill -f` for `micro_ros_agent udp4`, `static_transform_publisher`,
   `diff_drive_controller`, `rplidar`, `slam_toolbox`, `nav2`, `lifecycle_manager`, `amcl`, `map_server`,
   `teleop_twist_keyboard`, `ros2 launch amr_navigation`; wait 2 s; restart the ROS 2 CLI daemon (fresh discovery cache).
4. **micro-ROS agent:** `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888` → wait ≤ 60 s for `/encoder_telemetry`.
5. **Laser TF:** `ros2 launch amr_navigation laser_tf.launch.py`.
6. **Diff drive:** `ros2 launch amr_diff_drive diff_drive.launch.py` → wait ≤ 30 s for `/odom`.
7. **LiDAR:** `ros2 launch rplidar_ros rplidar_a1_launch.py` → wait ≤ 30 s for `/scan`.
8. **Mode:**
   - `slam`: `ros2 launch amr_navigation slam.launch.py`; prints the teleop and map-save commands.
   - `nav`: `ros2 launch amr_navigation nav_map.launch.py map:=$MAP use_composition:=True`, piped through
     `grep --line-buffered -v RTPS_READER_HISTORY | tee ~/amr_logs/nav2.log` (shown in the terminal and logged); then polls the
     log every 2 s until `lifecycle_manager_localization.*Managed nodes are active` → "Localization ACTIVE",
     or stops waiting on `Aborting bringup` / the process exiting.
   - `robot`: nothing more.
9. `wait` until Ctrl-C.

**Helpers**
- `start NAME CMD...` runs `setsid CMD > $LOG_DIR/NAME.log 2>&1 &`: every program in **its own process group** (so its
  child processes can be signalled together) with its own log file; the group id is stored in `PIDS`.
- `wait_topic TOPIC SECONDS` repeats `timeout 5 ros2 topic echo --once TOPIC` until a message arrives; prints dots,
  and "NOT READY after Ns (continuing anyway)" on timeout. Continuing lets you see which part failed in its log.

**Stopping** (`trap cleanup INT TERM`): `kill -INT` to every process group (lets `ros2 launch` shut its nodes down
cleanly; `amr_diff_drive` sends a final zero to the wheels), wait 5 s, `kill -KILL` to whatever is left, exit 0.
If the script is killed with SIGKILL instead, the next start's cleanup step removes the leftovers.

## 3. Logs

| File (`~/amr_logs/`) | From |
|---|---|
| `agent.log` | micro-ROS agent (sessions, entity creation) |
| `laser_tf.log` | static transform publisher |
| `diff_drive.log` | `amr_diff_drive` (startup line, first encoder sample, latency stats every 10 s, warnings) |
| `rplidar.log` | LiDAR driver (model, health, scan mode) |
| `slam.log` / `nav2.log` | SLAM Toolbox / Nav2 (nav2.log already filtered) |

Each run overwrites them. Live: `tail -f ~/amr_logs/diff_drive.log`.

## 4. TF tree

```
map ──(AMCL in nav mode / SLAM Toolbox in slam mode)──► odom ──(amr_diff_drive, 50 Hz)──► base_link ──(static)──► laser
```

| Transform | Publisher | Rate |
|---|---|---|
| `map → odom` | AMCL (`tf_broadcast: true`) or SLAM Toolbox (`transform_publish_period 0.02`) | on filter update / 50 Hz |
| `odom → base_link` | `amr_diff_drive` | 50 Hz, predicted to "now" |
| `base_link → laser` | `laser_tf.launch.py` (x 0.175, z 0.100) | static (`/tf_static`) |

Conventions: REP-103 (x forward, y left, z up, counter-clockwise yaw) and REP-105 frame roles. Exactly one publisher
per transform; `ros2 run amr_navigation check_setup.sh` verifies this. Snapshot: `ros2 run tf2_tools view_frames`.
There's no URDF / `robot_state_publisher`, so RViz shows no robot model.

## 5. Keyboard teleop (`amr_teleop`)

```bash
source ~/amr/amr_ws/install/setup.bash
ros2 run amr_teleop wasd_teleop            # or: python3 amr_ws/src/amr_teleop/amr_teleop/wasd_teleop.py
```

| Key | `linear.x` | `angular.z` |
|---|---|---|
| `w` | +0.1 m/s | 0 |
| `s` | −0.1 m/s | 0 |
| `a` | 0 | +1.0 rad/s (left, counter-clockwise) |
| `d` | 0 | −1.0 rad/s |
| any other key | 0 | 0 (stop) |
| Ctrl-C | stop, restore terminal, exit | |

Implementation: node `wasd_teleop`, publisher `cmd_vel` (`geometry_msgs/Twist`, depth 10). The terminal is switched to raw
mode and polled every 0.1 s (`select`); a background thread publishes the *current* twist every 20 ms (50 Hz), so
`amr_diff_drive`'s 0.5 s `cmd_vel_timeout` never triggers while a key's command is active. On exit it publishes a zero
twist. `teleop_twist_keyboard` also works (installed by `install_deps.sh`) but only publishes on key presses.

Never run teleop while Nav2 is running: both publish `/cmd_vel`.

## 6. Network

- Pi and laptop: same `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY=0`, same subnet (DDS discovery uses multicast).
- ESP32: micro-ROS domain is set in the firmware; see [02-micro-ros-communication.md §8](02-micro-ros-communication.md).
- RViz and other heavy GUIs run on the laptop, not the Pi.
