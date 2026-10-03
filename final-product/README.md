# AMR: Differential-Drive Autonomous Mobile Robot

ROS 2 **Humble** software for a differential-drive AMR:
**RPLIDAR A1 → Raspberry Pi (ROS 2, SLAM Toolbox, Nav2) ⇄ micro-ROS over Wi-Fi ⇄ ESP32 (PID + encoders) → Cytron motor driver → motors**.

![The AMR](docs/images/amr-robot.jpg)

One command starts the whole robot:

```bash
./amr_bringup.sh            # robot + Nav2 on the saved map (maps/my_map.yaml)
```

**Documentation**

| Topic | Where |
|---|---|
| Architecture diagrams (system level down to function and circuit level) | [docs/architecture.md](docs/architecture.md) |
| **Implementation guides** per subsystem (micro-ROS, ESP32 control, encoders, diff drive, RPLIDAR, SLAM, Nav2, bringup) | [docs/guides/](docs/guides/README.md) |
| Hardware: bill of materials, wiring, pinout | [docs/hardware.md](docs/hardware.md) |
| Calibration and tuning notes | [docs/calibration.md](docs/calibration.md) |
| Known issues log | [docs/known-issues.md](docs/known-issues.md) |
| Project presentation (slides: stack, diagrams, data) | [docs/presentation/AMR_Project_Presentation.pptx](docs/presentation/AMR_Project_Presentation.pptx) |
| Demo videos | [demo-videos/](demo-videos/README.md) |
| CAD models | [cad/](cad/README.md) |
| How we got here (iterations, approaches tried, test videos) | [../learning/](../learning/README.md) |
| Background to learn first (ROS 2, SLAM, Nav2, PID, ...) | [../resource-guide/](../resource-guide/README.md) |

---

## 1. Code structure

```
final-product/
├── amr_bringup.sh              # ONE script that starts everything (see section 4)
├── amr_ws/                     # main colcon workspace (runs on the Raspberry Pi)
│   └── src/
│       ├── amr_diff_drive/     # cmd_vel -> wheel speeds, encoders -> /odom + odom->base_link TF
│       ├── amr_navigation/     # SLAM Toolbox + Nav2 launch files, parameters, laser TF launch
│       ├── amr_teleop/         # WASD keyboard teleop (ros2 run amr_teleop wasd_teleop)
│       └── rplidar_ros/        # Slamtec RPLIDAR driver (vendored copy)
├── microros_ws/                # micro-ROS agent workspace (vendored copy)
├── firmware/esp32/             # ESP32 firmware: wheel PID + encoders + micro-ROS (Arduino sketch)
├── maps/                       # saved maps (map_server .yaml + .pgm)
├── scripts/
│   ├── install_deps.sh         # one-time apt + rosdep install on a fresh Pi
│   └── build.sh                # builds microros_ws and amr_ws
├── docs/                       # architecture, hardware, calibration, known issues, guides/, presentation/
├── demo-videos/                # links to demo videos, one per feature
└── cad/                        # mechanical CAD files
```

| Component | Language | What it does | Details |
|---|---|---|---|
| `amr_diff_drive` | Python | Converts `/cmd_vel` into wheel speed commands (`/left_vel`, `/right_vel`) and integrates encoder feedback (`/encoder_telemetry`) into `/odom` + the `odom → base_link` TF at 50 Hz, compensating for Wi-Fi latency with the ESP32's timestamps. Kinematics are in a ROS-free module with unit tests. | [README](amr_ws/src/amr_diff_drive/README.md) |
| `amr_navigation` | launch + YAML | SLAM Toolbox mapping, Nav2 navigation (AMCL on a saved map, Regulated Pure Pursuit controller), the static `base_link → laser` TF, and a pre-flight check script. | [README](amr_ws/src/amr_navigation/README.md) |
| `amr_teleop` | Python | Keyboard teleop that publishes `/cmd_vel` continuously at 50 Hz (`amr_diff_drive` stops the wheels 0.5 s after the last command). | `amr_teleop/wasd_teleop.py` |
| `rplidar_ros` | C++ | Slamtec's LiDAR driver (vendored copy); publishes `/scan`. | upstream README in the package |
| micro-ROS agent | C++ | Bridges the ESP32 (UDP port 8888) into the ROS 2 graph. | `microros_ws/` |
| ESP32 firmware | Arduino C++ | 50 Hz per-wheel feed-forward + PI speed control, quadrature encoder decoding, micro-ROS client with auto-reconnect and a serial tuning console. | [README](firmware/esp32/README.md) |

Two workspaces are used because the micro-ROS agent must be built with its own
`build_agent.sh` flags (`-DUAGENT_BUILD_EXECUTABLE=OFF -DUAGENT_P2P_PROFILE=OFF`);
`scripts/build.sh` does this for you.

---

## 2. System architecture

```
                         Raspberry Pi (ROS 2 Humble)                                         ESP32 (micro-ROS)
 ┌──────────────────────────────────────────────────────────────────────────┐        ┌──────────────────────────┐
 │  RViz / goal ─► Nav2 (bt_navigator, planner, RPP controller, smoother)   │        │  PID speed loop (L & R)  │
 │                    │ /cmd_vel (Twist)                                    │        │  quadrature decoding     │
 │                    ▼                                                     │ Wi-Fi  │                          │
 │  amr_diff_drive ── /left_vel, /right_vel (Float64, rad/s) ─► micro-ROS ──┼──UDP──►│ ─► Cytron ─► motors      │
 │        ▲  │                                                   agent      │  8888  │                          │
 │        │  └─► /odom + TF odom→base_link                        (udp4)  ◄──┼────────┤ ◄─ encoders             │
 │        └──── /encoder_telemetry (JointState: pos rad, vel rad/s) ◄───────┘        └──────────────────────────┘
 │                                                                          │
 │  rplidar_ros ─► /scan (frame "laser")     static TF base_link→laser      │
 │  map→odom:  AMCL (nav mode)  or  SLAM Toolbox (slam mode)                │
 └──────────────────────────────────────────────────────────────────────────┘
```

TF tree: `map → odom → base_link → laser`

| Transform / topic | Published by |
|---|---|
| `/encoder_telemetry` (`sensor_msgs/JointState`, names `left_wheel`, `right_wheel`) | ESP32 via micro-ROS agent |
| `/left_vel`, `/right_vel` (`std_msgs/Float64`, rad/s) | `amr_diff_drive` → ESP32 |
| `/odom` + `odom → base_link` | `amr_diff_drive` (50 Hz, latency-compensated) |
| `base_link → laser` (x 0.175, z 0.100) | `amr_navigation/launch/laser_tf.launch.py` (started by `amr_bringup.sh`) |
| `/scan` | `rplidar_ros` (A1, `/dev/ttyUSB0`) |
| `/map` + `map → odom` | AMCL + map_server (nav) **or** SLAM Toolbox (slam) |
| `/cmd_vel` | Nav2 velocity smoother (nav) **or** teleop (slam / robot) |

Robot geometry: wheel radius **0.056 m**, wheel separation **0.39 m**.

**Diff drive:** the early design used a ros2_control Hardware Interface plus the stock Differential Drive
Controller. On the final robot both roles are done by **`amr_diff_drive`**, a single Python node that talks to
the ESP32 topics directly. Why: see [learning/approach-documentation/software.md](../learning/approach-documentation/software.md).

Detailed diagrams: [docs/architecture.md](docs/architecture.md).

---

## 3. Setup and build (on the Raspberry Pi)

Requirements: Raspberry Pi with Ubuntu 22.04 and ROS 2 Humble (`/opt/ros/humble`), internet for the first build.

Clone the repository and link this folder to `~/amr`. All paths in this documentation use `~/amr`,
and the scripts work through the link.

```bash
git clone <repo-url> ~/amr-repo
ln -s ~/amr-repo/final-product ~/amr
cd ~/amr
scripts/install_deps.sh     # once: apt packages (Nav2, SLAM Toolbox, ...) + rosdep
scripts/build.sh            # builds microros_ws, then amr_ws (--symlink-install)
```

- `scripts/build.sh amr` rebuilds only `amr_ws`. Extra args go to colcon, e.g. `scripts/build.sh amr --packages-select amr_navigation`.
- `scripts/build.sh agent` rebuilds only the micro-ROS agent. It needs internet: the agent's CMake downloads Micro-XRCE-DDS-Agent.

**Network:** the Pi and the laptop (RViz) must use the same ROS domain. Put these two lines in `~/.bashrc`
on **both** machines (the bringup script inherits them from your shell and prints them at startup):

```bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
```

**LiDAR serial port:** the RPLIDAR is read from `/dev/ttyUSB0`, so your user needs serial access:
`sudo usermod -aG dialout $USER` (then log out and in), or install the driver's udev rule with
`amr_ws/src/rplidar_ros/scripts/create_udev_rules.sh`.

**ESP32:** set your Wi-Fi name/password and the Pi's current IP in the sketch, then flash it:
see the [firmware README](firmware/esp32/README.md).

---

## 4. Running

```bash
./amr_bringup.sh            # = nav: robot + Nav2 on maps/my_map.yaml   (normal operation)
./amr_bringup.sh slam       # robot + SLAM Toolbox (build a new map)
./amr_bringup.sh robot      # robot only: agent, laser TF, diff drive, LiDAR
MAP=/abs/path/other.yaml ./amr_bringup.sh     # navigate on a different map
```

What it does, in order (each step waits for its topic):

1. micro-ROS agent `udp4 --port 8888` → waits for `/encoder_telemetry` (ESP32 must be powered)
2. static TF `base_link → laser`
3. `ros2 launch amr_diff_drive diff_drive.launch.py` → waits for `/odom`
4. `ros2 launch rplidar_ros rplidar_a1_launch.py` → waits for `/scan`
5. **nav:** `ros2 launch amr_navigation nav_map.launch.py map:=<MAP> use_composition:=True 2>&1 | grep -v RTPS_READER_HISTORY`.
   Its output is shown in the terminal and saved to `~/amr_logs/nav2.log`.
   **slam:** `ros2 launch amr_navigation slam.launch.py`

Ctrl-C stops everything. Logs: `~/amr_logs/<name>.log` (`agent`, `laser_tf`, `diff_drive`, `rplidar`, `nav2`/`slam`).

**After startup (nav):** on the laptop, run `ros2 launch nav2_bringup rviz_launch.py`, click **2D Pose Estimate**, then **Nav2 Goal**.
Don't run teleop while Nav2 is running.

**Making a map (slam):** drive around with teleop in a second terminal, then save:

```bash
source ~/amr/amr_ws/install/setup.bash
ros2 run amr_teleop wasd_teleop            # or: ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run nav2_map_server map_saver_cli -f ~/amr/maps/my_map
```

Running pieces by hand (e.g. for debugging):

```bash
source /opt/ros/humble/setup.bash
source ~/amr/microros_ws/install/setup.bash
source ~/amr/amr_ws/install/setup.bash
ros2 run amr_navigation check_setup.sh     # pre-flight check of topics and TF
ros2 launch amr_navigation nav_map.launch.py map:=$HOME/amr/maps/my_map.yaml use_composition:=True 2>&1 | grep -v RTPS_READER_HISTORY
```

---

## 5. Maps

| File | Notes |
|---|---|
| `maps/my_map.yaml` | Default map for `./amr_bringup.sh` |

Map `.yaml` files reference their `.pgm` by a relative path, so keep each pair together.
Older maps from the iteration phase are in `learning/iteration-phase/nav_ws/`.

---

## 6. Known limitations and next steps

### 6.1 Known limitations
- **Not yet run on the real robot in this layout.** It was verified in a ROS 2 Humble container with simulated ESP32
  and LiDAR data (build, unit tests, all three bringup modes, a Nav2 goal). The same code ran on the robot before
  the repo was reorganized. Section 6.2 is the checklist for the first real run.
- **No robot model (URDF).** Only a static `base_link → laser` TF exists; RViz shows no robot body.
- **Nominal geometry.** The Nav2 footprint (0.50 × 0.46 m) is a placeholder, and wheel radius / separation are
  nominal values, not calibrated (section 6.3).
- **Wheel odometry only.** No IMU or sensor fusion (e.g. `robot_localization` EKF), so odometry drifts on slip.
- **Compile-time network settings.** The Wi-Fi name/password and the Pi's IP are constants in the ESP32 sketch;
  the Pi's IP isn't fixed, so the firmware has to be re-flashed when it changes.
- **No command arbitration.** Teleop and Nav2 both publish `/cmd_vel` with no mux, so they must never run together.
- `amr_navigation/config/mapper_params_real.yaml` (alternative SLAM config) uses two parameter names that
  SLAM Toolbox ignores (see the header of that file).

### 6.2 Deploy and first-run checklist
- [ ] Commit and push the repo.
- [ ] **Keep the old folders on the Pi** (`~/microros_ws`, `~/amr-diff-drive`, `~/slam`, `~/nav_ws`, old `~/amr_bringup.sh`)
      until everything below passes. They're your fallback.
- [ ] Clone and link to `~/amr` (section 3), then `scripts/install_deps.sh` and `scripts/build.sh` (first build on the
      Pi's ARM CPU, including the micro-ROS agent, which needs internet).
- [ ] In the Pi's `~/.bashrc`: remove `source ~/microros_ws/install/local_setup.bash` and `source install/setup.bash`
      (the old workspaces must not be sourced together with the new ones); keep the `ROS_DOMAIN_ID` / `ROS_LOCALHOST_ONLY` exports.
- [ ] LiDAR access: `sudo usermod -aG dialout $USER` (or the rplidar udev rule), see section 3.
- [ ] ESP32: set `SSID_NAME`, `SSID_PASSWORD` and the Pi's current IP (`AGENT_IP`) in the sketch, then compile and flash
      ([firmware README](firmware/esp32/README.md)). The sketch hasn't been compiled since it was added to the repo.

First run (wheels lifted first):
- [ ] `./amr_bringup.sh robot`: all three waits print `ok` (`/encoder_telemetry`, `/odom`, `/scan`).
      If `/encoder_telemetry` is NOT READY while the agent log shows the ESP32 connecting, it's most likely a **ROS domain
      mismatch**: the firmware doesn't set a domain (→ 0) while `~/.bashrc` sets `ROS_DOMAIN_ID=30`. Check with
      `ROS_DOMAIN_ID=0 ros2 topic list`; fix: [micro-ROS guide §8](docs/guides/02-micro-ros-communication.md).
- [ ] Rates: `ros2 topic hz /encoder_telemetry` ≈ 20 Hz, `/odom` ≈ 50 Hz, `/scan` ≈ 10 Hz.
- [ ] `ros2 run amr_navigation check_setup.sh`: no FAIL lines.
- [ ] `~/amr_logs/diff_drive.log` shows `encoder latency ... stamps rejected 0/N` (ESP32 time sync works).
- [ ] Teleop (`ros2 run amr_teleop wasd_teleop`): both wheels turn forward on `w`, `/odom` x increases,
      `a` turns the robot counter-clockwise. Details: [amr_diff_drive README §9](amr_ws/src/amr_diff_drive/README.md).
- [ ] Ctrl-C in the bringup terminal stops everything (`ros2 node list` is empty afterwards).
- [ ] `./amr_bringup.sh slam`: map builds while driving; save it under a **new** name
      (`ros2 run nav2_map_server map_saver_cli -f ~/amr/maps/test_map`) and check it loads with
      `MAP=$HOME/amr/maps/test_map.yaml ./amr_bringup.sh`.
- [ ] `./amr_bringup.sh`: "Localization ACTIVE", RViz on the laptop shows the map, scan and robot;
      2D Pose Estimate, then short goals following the [amr_navigation README §9](amr_ws/src/amr_navigation/README.md) test order.
- [ ] Drive through the narrowest passage the robot used to manage: the global costmap footprint was enlarged
      from 0.40 × 0.44 m to 0.50 × 0.46 m (same as the local costmap), so tight gaps may now be refused.

### 6.3 Measure (don't guess)
- [ ] **Footprint:** measure the real outline around `base_link` and set it in both costmaps
      in `amr_navigation/config/nav2_params.yaml`.
- [ ] **Odometry calibration:** follow [amr_diff_drive README §10](amr_ws/src/amr_diff_drive/README.md)
      (3 m straight, 5 spins) and update `diff_drive.yaml`.
- [ ] **LiDAR mount:** check x 0.175 m, z 0.100 m and zero yaw in `amr_navigation/launch/laser_tf.launch.py`
      against the robot (or the CAD). A wrong yaw smears the map when turning.

All procedures and current values: [docs/calibration.md](docs/calibration.md).

### 6.4 Tune (if needed)
Nav2 speed (0.25 m/s now), inflation radius, goal tolerances, AMCL noise, SLAM parameters: see the
[amr_navigation README §10](amr_ws/src/amr_navigation/README.md) tuning table. The ESP32 PID is already tuned per wheel.

### 6.5 Possible next steps
- Add a URDF (`robot_state_publisher`) generated from the CAD, replacing the static laser TF.
- Fuse an IMU with wheel odometry (`robot_localization` EKF).
- Add a `twist_mux` so teleop can safely override Nav2.
- Move the ESP32 network settings out of the source (e.g. set over serial and stored in flash).
