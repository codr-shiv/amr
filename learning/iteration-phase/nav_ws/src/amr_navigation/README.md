# amr_navigation

SLAM Toolbox + Nav2 setup for the differential-drive AMR (ROS 2 **Humble**, Raspberry Pi).

This package contains **no robot drivers**. It sits on top of your existing robot bringup
(diff drive controller on `/cmd_vel` → `/odom` + `odom → base_link` TF, the LiDAR driver,
`robot_state_publisher` / static TF) and adds:

| What | File |
|---|---|
| Mapping (SLAM Toolbox) | `launch/slam.launch.py` |
| Navigation while mapping (SLAM + Nav2) | `launch/nav_slam.launch.py` |
| Navigation on a saved map (map_server + AMCL + Nav2) | `launch/nav_map.launch.py` |
| All Nav2 parameters (costmaps, controller, planner, AMCL, ...) | `config/nav2_params.yaml` |
| SLAM Toolbox parameters (frames fixed for this robot) | `config/slam_toolbox_params.yaml` |
| Pre-flight checker | `scripts/check_setup.sh` |
| Saved maps | `maps/` |

---

## 1. How it fits together

```
 RViz (laptop) --goal--> bt_navigator
                           ├─ planner_server     (global path on GLOBAL costmap, frame: map)
                           ├─ controller_server  (Regulated Pure Pursuit on LOCAL costmap, frame: odom)
                           │        │ cmd_vel_nav
                           │        ▼
                           │   velocity_smoother ──/cmd_vel──► your diff drive controller ──► ESP32 (micro-ROS)
                           └─ behavior_server    (spin / back up / wait recoveries)
  /scan (LiDAR) ──► SLAM Toolbox or AMCL ──► map → odom TF
  diff drive controller ──► odom → base_link TF + /odom
```

Required TF tree: `map → odom → base_link → <laser frame>`

| Transform | Published by |
|---|---|
| `map → odom` | SLAM Toolbox (mapping / nav_slam) **or** AMCL (nav_map) |
| `odom → base_link` | your diff drive controller node |
| `base_link → laser` | `robot_state_publisher` (URDF) or a static TF publisher |

---

## 2. Assumptions (defaults used in this package)

| Item | Default | Where to change |
|---|---|---|
| ROS distro | Humble, Ubuntu 22.04 | — |
| Robot base frame | `base_link` | `base_frame` / `robot_base_frame` / `base_frame_id` in both YAML files |
| Odom frame | `odom` | same |
| Odometry topic | `/odom` | `odom_topic` (3 places) in `nav2_params.yaml` |
| Velocity command topic | `/cmd_vel` (Nav2's velocity_smoother output, plain `Twist`) | — |
| LiDAR topic | `/scan` | `topic:` in both costmaps, `scan_topic` (AMCL, SLAM) |
| Wheel radius / separation | 0.056 m / 0.39 m (used only in your controller config) | your controllers YAML |
| Cruise speed | 0.25 m/s, 1.0 rad/s | controller + velocity_smoother |

---

## 3. Prerequisites

1. **Packages** (on the Pi):
   ```bash
   sudo apt update
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
   ```
2. **Robot bringup already works**: teleop drives the robot, `/scan` is published, and the TF tree
   `odom → base_link → laser` exists.
3. **Odometry is sane** (Nav2 amplifies odometry errors):
   - `angular.z = +0.5` turns the robot counter-clockwise (seen from above), and the yaw in odom increases.
   - With RViz Fixed Frame = `odom` and LaserScan *Decay Time* = 30 s, walls stay sharp while you drive straight and spin.
   - Odometry is computed from the encoders (not from the commanded velocity).
4. **Only one** `odom → base_link` publisher.

---

## 4. ⚠️ Edit these before the first run

Open `config/nav2_params.yaml` and search for `# EDIT`:

| # | What | Why it matters |
|---|---|---|
| 1 | **`footprint`** (local AND global costmap). Measure the robot's outer outline in metres, relative to `base_link` (usually the middle of the wheel axle): x forward, y left, corners counter-clockwise. The placeholder is 0.50 × 0.46 m. | Too small: the robot clips obstacles. Too big: it refuses doorways. |
| 2 | **`laser_max_range`** (AMCL) and **`max_laser_range`** (`slam_toolbox_params.yaml`) | Set to your LiDAR's real usable range (LD14 ≈ 8 m, RPLIDAR A1 ≈ 12 m). |
| 3 | **`odom_topic`** (3 places) | Only if your odometry topic isn't `/odom`. |
| 4 | **Frame names** | Only if your URDF doesn't use `base_link` / `odom`. |

If the robot is roughly round, you can replace `footprint:` with `robot_radius: 0.27` in both costmaps.

---

## 5. Install and build

```bash
# 1. put the package in your workspace
cd ~/your_ws/src
unzip ~/amr_navigation.zip          # creates ~/your_ws/src/amr_navigation

# 2. build
cd ~/your_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select amr_navigation --symlink-install

# 3. source (every new terminal, or add to ~/.bashrc)
source ~/your_ws/install/setup.bash
```

With `--symlink-install`, edits to YAML and launch files take effect on the next launch without
rebuilding. **Exception:** a map copied into `maps/` is only picked up after a rebuild. You can avoid
that by passing `map:=/full/path.yaml` instead.

---

## 6. Pre-flight check

Start your robot bringup, then run:
```bash
ros2 run amr_navigation check_setup.sh
# custom topics:  ros2 run amr_navigation check_setup.sh <odom> <scan> <cmd_vel>
```
It checks that `/scan` and `/odom` publish data, that something subscribes to `/cmd_vel`, that
`odom → base_link` and `base_link → laser` exist, which nodes publish `/tf`, and that exactly one
node publishes `odom → base_link`.
**Fix every FAIL before continuing.**

---

## 7. Topics Nav2 uses

Nav2 reads odometry from **`/odom`**, the LiDAR from **`/scan`**, and TF. Its final velocity
command (after the velocity smoother) is published as plain `Twist` on **`/cmd_vel`**, which your
diff drive controller already listens to. No bridge or remapping is needed.

The controller server's own output is internally named `cmd_vel_nav` and goes into the velocity
smoother; only the smoother publishes `/cmd_vel`. **Don't run teleop at the same time as Nav2**:
both would publish on `/cmd_vel` and fight.

---

## 8. Usage

Open terminals on the Pi (or SSH sessions). **Terminal 1 is always your robot bringup.**

### 8.1 Step A: build a map

```bash
# T2
ros2 launch amr_navigation slam.launch.py
# T3: drive around slowly (~0.15 m/s, turn slowly), revisit places to close loops
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Save the map** when it looks complete:
```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab
# -> ~/maps/lab.yaml + ~/maps/lab.pgm
```
Optional: also save SLAM Toolbox's pose graph (to continue mapping later) from the RViz
SlamToolboxPlugin panel → "Serialize Map".

### 8.2 Step B: first Nav2 test while mapping (recommended first)

```bash
# T2 (instead of slam.launch.py)
ros2 launch amr_navigation nav_slam.launch.py
```
SLAM starts immediately and Nav2 starts 5 s later. No initial pose is needed.

### 8.3 Step C: normal operation on the saved map (AMCL)

```bash
# T2
ros2 launch amr_navigation nav_map.launch.py map:=$HOME/maps/lab.yaml
```
Then in RViz: **2D Pose Estimate**. Click at the robot's real position and drag in the direction it
faces. Watch the green particle cloud shrink as the robot moves.

### 8.4 RViz on your laptop

The laptop and Pi must be on the same network with the same `ROS_DOMAIN_ID`:
```bash
export ROS_DOMAIN_ID=<same number as the Pi>
ros2 launch nav2_bringup rviz_launch.py
```
Useful displays: Map, `/local_costmap/costmap`, `/global_costmap/costmap`, `/plan`,
`/local_plan`, LaserScan `/scan`, TF, RobotModel.

### 8.5 Sending goals

- **RViz:** "Nav2 Goal" button, then click and drag.
- **Command line:**
  ```bash
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
  ```
- **Emergency stop:** Ctrl-C the navigation launch. The diff drive controller stops the wheels after
  its `cmd_vel_timeout` (0.5 s), and the ESP32 firmware stops them after 500 ms without commands.

---

## 9. First-run test sequence (do it in this order)

1. **Costmaps, robot standing still.** Launch `nav_slam`. In RViz, walk in front of the LiDAR:
   you appear in the local costmap with an inflation halo and **disappear** when you leave.
2. **1 m straight goal.** Keep a hand on the e-stop.
3. **Goal that needs a turn.** The robot should rotate in place first, then drive.
4. **Obstacle not in the map** (a box). The path should bend around it.
5. **Doorway.** If it refuses, check the footprint, then lower `inflation_radius` (0.40 → 0.30).
6. **Only then raise speed** in small steps: `desired_linear_vel`, `max_velocity`/`min_velocity`
   (velocity_smoother), 0.25 → 0.35 → 0.5 m/s.

---

## 10. Tuning guide

| Want | Change | File / section |
|---|---|---|
| Faster driving | `desired_linear_vel` **and** `velocity_smoother.max_velocity[0]` | nav2_params / controller_server, velocity_smoother |
| Faster turning | `rotate_to_heading_angular_vel`, `velocity_smoother.max_velocity[2]` | same |
| Smoother starts and stops | lower `max_accel` / `max_decel` | velocity_smoother |
| Keep further from walls | raise `inflation_radius`, lower `cost_scaling_factor` (keep RPP `inflation_cost_scaling_factor` equal) | both costmaps + controller |
| Pass narrower gaps | lower `inflation_radius`, check the footprint | both costmaps |
| Stops short / oscillates at goal | raise `xy_goal_tolerance` / `yaw_goal_tolerance` | controller_server |
| "Failed to make progress" too often | raise `movement_time_allowance` | controller_server |
| Cuts corners | lower `lookahead_dist` / `max_lookahead_dist` | FollowPath |
| Wobbles on straights | raise `lookahead_dist` | FollowPath |
| Pi CPU overloaded | lower costmap `update_frequency`, `controller_frequency`, AMCL `max_particles`; try `use_composition:=True` | several |
| AMCL loses position | raise `alpha1..4` (distrust odometry more), fix odometry calibration | amcl |

---

## 11. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Nav2 nodes stuck inactive / "lifecycle" errors | frame name or `use_sim_time` mismatch | check `base_link`/`odom` names; everything uses `use_sim_time: false` |
| `Timed out waiting for transform` / extrapolation errors | TF delay (Wi-Fi odometry) or a missing TF | run `check_setup.sh`; raise `transform_tolerance` to 0.5 |
| Nav2 says it's following the path but the robot doesn't move | nothing reaches `/cmd_vel`, or the velocity is below the motors' start threshold | `ros2 topic echo /cmd_vel` while navigating; `ros2 topic info /cmd_vel -v` (the smoother must publish, your controller must subscribe) |
| Robot jerks or fights | teleop (or another node) also publishing `/cmd_vel` | close teleop while Nav2 runs |
| Obstacles smear or trail in the local costmap | odometry lag or error | ESP32 timestamp fix + odometry calibration |
| Ghost obstacles never disappear | clearing not working | `clearing: true`, `raytrace_max_range` ≥ `obstacle_max_range` |
| "No valid path" to a reachable goal | inflation or footprint too large, or the goal is in inflated space | lower `inflation_radius`; pick a goal away from walls |
| Robot leaves the map (AMCL) | wrong initial pose or bad odometry | redo 2D Pose Estimate; calibrate odometry; raise `alpha1..4` |
| SLAM map smears when turning | LiDAR TF yaw wrong or mirrored scan; odometry lag | check `base_link → laser` (yaw, roll = π if upside down); timestamp fix |
| Map doesn't appear in nav_map | wrong map path | use the absolute path `map:=/home/<user>/maps/lab.yaml` |

Useful commands:
```bash
ros2 run tf2_tools view_frames                  # PDF of the TF tree
ros2 topic hz /scan
ros2 topic hz /odom
ros2 lifecycle get /controller_server           # should be "active [3]"
```

---

## 12. Package layout

```
amr_navigation/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── nav2_params.yaml            # all Nav2 parameters (costmaps, RPP, AMCL, ...)
│   └── slam_toolbox_params.yaml    # SLAM Toolbox (base_link, wall time)
├── launch/
│   ├── slam.launch.py              # mapping only
│   ├── nav_slam.launch.py          # SLAM + Nav2
│   └── nav_map.launch.py           # saved map + AMCL + Nav2
├── maps/                           # put map.yaml + map.pgm here (or pass map:=)
└── scripts/
    └── check_setup.sh              # pre-flight checks
```
