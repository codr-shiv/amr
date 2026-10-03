# Learning / Iteration phase

How the team got to the final robot: every attempt, including the ones that were dropped.
For the working robot itself, see [final-product](../final-product/README.md).

| Folder / file | What it is |
|---|---|
| [project-log.md](project-log.md) | The build as it happened: goals, component list, teams, daily updates, timeline, mentors' guidance |
| [iteration-phase/](iteration-phase/) | The team's working folders exactly as they were on the robot's Raspberry Pi (organized by work area, since the verticals worked separately). Nothing in it has been edited. |
| [approach-documentation/software.md](approach-documentation/software.md) | Every software approach tried, pros/cons, and why the final one was chosen |
| [approach-documentation/hardware.md](approach-documentation/hardware.md) | Every component considered or tried, the problems it solved or caused, and why the final part was chosen |
| [testing-videos/](testing-videos/README.md) | Test videos and photos, each with what was tested, the setup and the outcome |

## Timeline

| Date (2026) | Milestone |
|---|---|
| 22 Sep | Kickoff meeting, system architecture presented |
| 23 Sep | Teams formed (below), work starts |
| 24 Sep | Individual tasks: CAD printed, micro-ROS link up, SLAM Toolbox with fake odometry, Nav2 → ESP32 path with hard-coded values |
| 25 Sep | All individual tasks done; **bot drives** (20:57) |
| 26 Sep | **SLAM on the robot** (04:36) |
| 27 Sep | Parts reprinted, robot reassembled |
| 28 Sep | **Autonomous navigation with Nav2** (02:32): 5 days after work started |

Details: [project-log.md](project-log.md).

## Teams (verticals)

| # | Vertical | Task | Size | Members |
|---|---|---|---|---|
| R1 | SLAM | RPLIDAR driver and SLAM Toolbox | — | TODO |
| R2 | Controller & HW interface | differential drive controller and hardware interface | 3 | TODO |
| R3 | micro-ROS communication | micro-ROS link between ESP32 and Pi, topic formats | 5 | TODO |
| R4 | Nav2 | Nav2 configuration and navigation | 3 | TODO |
| E1 | Motor control | wheel velocity → PWM, PID loop on the ESP32 | 4 | TODO |
| E2 | Encoder / decoder | encoder decoding, calibration, odometry testing | 3 | TODO |
| HW | Hardware & CAD | chassis, motor and wheel mounts, LiDAR mount | — | TODO |

## Index of `iteration-phase/`

| Folder / file | Vertical | What it contains | Outcome | Where it ended up in `final-product/` |
|---|---|---|---|---|
| `amr-diff-drive/src/amr_diff_drive/` | R2 | Python differential-drive node: `/cmd_vel` → wheel speeds, encoders → `/odom` + TF, with ESP32-timestamp latency compensation and unit tests | **Adopted** | `amr_ws/src/amr_diff_drive` |
| `amr-diff-drive/*.zip` | R2 / R4 | Delivered snapshots: the diff drive package and two versions of the navigation package (the first one with a `cmd_vel_relay.py` for ros2_control) | Reference | — |
| `differential_drive/src/my_amr_control_pkg/` | R2 | ros2_control hardware interface (C++ `SystemInterface` talking to the ESP32 topics) + stock `diff_drive_controller`, URDF, controller YAML | **Dropped** (replaced by `amr_diff_drive`) | — |
| `nav_ws/src/amr_navigation/` | R4 | SLAM Toolbox + Nav2 launch files and parameters, pre-flight check script | **Adopted** | `amr_ws/src/amr_navigation` |
| `nav_ws/maps/`, `nav_ws/my_map.*` | R4 | Maps recorded with SLAM Toolbox (`humaramap`, two versions of `my_map`) | Latest `my_map` adopted | `maps/my_map.*` |
| `slam/src/rplidar_ros/` | R1 | Slamtec RPLIDAR driver (cloned from upstream) | **Adopted** | `amr_ws/src/rplidar_ros` |
| `slam/src/amr_slam/` | R1 | First SLAM Toolbox package (`real_slam.launch.py`, `mapper_params_real.yaml`) | Config kept as an alternative; launch file dropped | `amr_ws/src/amr_navigation/config/mapper_params_real.yaml` |
| `slam/src/launch/robot.launch.py` | R1 | Static TF `base_link → laser` | **Adopted** | `amr_ws/src/amr_navigation/launch/laser_tf.launch.py` |
| `slam/src/slam_toolbox/` | R1 | Empty folder (SLAM Toolbox was installed from apt instead) | Dropped | — |
| `microros_ws/`, `setup_script.sh` | R3 | micro-ROS agent workspace and the script that set it up | **Adopted** (setup replaced by `scripts/install_deps.sh` + `scripts/build.sh`) | `microros_ws/` |
| `odom_baselink/odom_baselink.py` | TODO | Prototype relaying an odometry topic (`/odom_raw`) to the `odom → base_link` TF, written while odometry was still being built | **Dropped** (the diff drive node publishes the TF itself) | — |
| `wasd_teleop.py` | TODO | Keyboard teleop publishing `/cmd_vel` at 50 Hz | **Adopted** | `amr_ws/src/amr_teleop` |
| `amr_bringup.sh` | TODO | First one-command startup script (robot / slam / nav modes) | **Adopted** and extended | `amr_bringup.sh` |
| `amr_logs/` | TODO | Logs of a full bringup run on 28 Sep 2026 (agent, diff drive latency statistics, LiDAR, laser TF) | Reference | — |
| `frames_2026-09-26_*.gv/.pdf` | TODO | `view_frames` snapshots of the TF tree during testing | Reference | — |

The ESP32 firmware (verticals E1 + E2) was developed outside this folder; its final version is in
`final-product/firmware/esp32/`. The micro-ROS communication tests also live in a separate repo:
[codr-shiv/amr-esp-comm](https://github.com/codr-shiv/amr-esp-comm).

## Links

- Architecture deck (Canva): https://canva.link/pxjnqahkfg13uoa
- Topics / message-format doc (inputs, outputs and message formats between Pi and ESP32): https://docs.google.com/document/d/1CKC5PaiLtuVq5qNwfhTToIpuMQxy-mmduqVFJ-J3ZQc/edit?usp=sharing
- Reference video shared at kickoff (23 Sep): https://youtu.be/HJAE5Pk8Nyw
