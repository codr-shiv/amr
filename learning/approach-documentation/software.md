# Software approaches

Every software approach that was tried for the AMR, what was good and bad about it, and why the final one was chosen.
Everything here is based on the code in [`../iteration-phase/`](../iteration-phase/) and the final code in
[`../../final-product/`](../../final-product/README.md). Anything the code can't tell (who decided, what was observed
on the robot but not written down) is marked **TODO (team)**.

---

## 1. Wheel control and odometry on the Pi

The Pi has to turn a body velocity (`/cmd_vel`) into two wheel speed setpoints for the ESP32, and turn the ESP32's
encoder readings back into odometry (`/odom` + the `odom → base_link` TF).

### Approach A: ros2_control hardware interface + stock `diff_drive_controller`
`iteration-phase/differential_drive/src/my_amr_control_pkg/`

A C++ `hardware_interface::SystemInterface` plugin creates its own ROS node that publishes `/left_vel`, `/right_vel`
and subscribes to `/encoder_telemetry`; the standard `diff_drive_controller` does the kinematics and odometry.
Needs a URDF with a `<ros2_control>` block, `controllers.yaml`, `controller_manager` and two spawners.

| Pros | Cons |
|---|---|
| Standard ROS 2 architecture (matches the original design slides) | Much more setup: URDF, controller YAML, plugin export, C++ build, ros2_control packages |
| Well-tested odometry and command handling from `ros2_controllers` | The encoder timestamps from the ESP32 are ignored, so Wi-Fi latency shows up directly as odometry lag |
| Easy to swap in other ros2_control controllers later | The plugin spins a separate ROS node inside `read()` to receive the micro-ROS topics, which works against ros2_control's design (it expects direct hardware access) |
| | The controller listens on its own namespaced command topic, so Nav2/teleop output had to be remapped or relayed (see §4) |

### Approach B: custom Python node `amr_diff_drive` (final)
`iteration-phase/amr-diff-drive/src/amr_diff_drive/` → `final-product/amr_ws/src/amr_diff_drive/`

One node subscribes to `/cmd_vel` and `/encoder_telemetry` and publishes `/left_vel`, `/right_vel`, `/odom` and the TF at 50 Hz.

| Pros | Cons |
|---|---|
| Uses the ESP32's `header.stamp` (synced to the Pi clock) to know when each encoder sample was measured, and predicts the pose forward to "now". The bringup logs show encoder latency of 10-s window means of 15–242 ms (median 81 ms), single samples up to 410 ms, over Wi-Fi (`iteration-phase/amr_logs/diff_drive.log`); in a simulated test this cut the odometry error at 60 ms latency from ~39 mm / 4.5° to ~0.1 mm / 0.01° | Custom code to maintain instead of a standard controller |
| No URDF / controller manager needed; listens on `/cmd_vel` directly | No ros2_control ecosystem features (e.g. switching controllers) |
| Kinematics in a ROS-free module with 20 unit tests | |
| Explicit handling of encoder resets, stale data, bad stamps, NaN commands | |

**Why B:** it solves the actual problem of this robot (odometry measured on a microcontroller and sent over Wi-Fi)
and removes a whole layer of configuration. Only one of the two may run, since both publish `odom → base_link`.
TODO (team): anything else observed on the robot with approach A.

### Side attempt: odometry → TF relay
`iteration-phase/odom_baselink/odom_baselink.py`

A template node that republished an odometry topic (`/odom_raw`) as the `odom → base_link` TF, written before the
odometry node existed. **Dropped:** `amr_diff_drive` publishes the TF itself, and running both would create two
publishers for the same transform.

---

## 2. SLAM (mapping)

Both configurations use SLAM Toolbox in online async mode (installed from apt; `slam/src/slam_toolbox/` stayed empty).

**Developed before odometry existed:** on 24 Sep the SLAM team ran the LiDAR into SLAM Toolbox with *fake* odometry and
hand-made TFs (`odom → base_link`, `base_link → laser`), so mapping could be set up in parallel with the drive train.
The `odom_baselink.py` relay (§1) and the static `robot.launch.py` come from that stage. Real wheel odometry replaced the
fake one at integration, and SLAM ran on the robot on 26 Sep ([project log](../project-log.md)).

| | `amr_slam` config (first) | `amr_navigation` config (final default) |
|---|---|---|
| Files | `slam/src/amr_slam/config/mapper_params_real.yaml` | `nav_ws/src/amr_navigation/config/slam_toolbox_params.yaml` |
| Based on | hand-written minimal file | SLAM Toolbox's `mapper_params_online_async.yaml`, frames fixed for this robot |
| New scan processed after | 0.05 m / 0.05 rad of motion | 0.3 m / 0.3 rad |
| TF timeout | 0.2 s (default) | 0.3 s (odometry arrives over Wi-Fi) |
| Laser range | intended 0.15–12 m, but written as `minimum_laser_range` / `maximum_laser_range`, which SLAM Toolbox doesn't recognize → default 20 m used | `max_laser_range: 12.0` (RPLIDAR A1) |

**Final:** the `amr_navigation` config is used by `./amr_bringup.sh slam`: fewer, better-spaced scans are lighter on the
Pi CPU and the parameter names are correct. The first config is kept as `mapper_params_real.yaml` and can still be run
through `slam.launch.py slam_params_file:=...`. TODO (team): map quality observed with each.

---

## 3. Localization and navigation

| Approach | What it is | Used for |
|---|---|---|
| SLAM + Nav2 (`nav_slam.launch.py`) | SLAM Toolbox provides the map and `map → odom` while Nav2 navigates | First Nav2 tests: no saved map or initial pose needed |
| Saved map + AMCL + Nav2 (`nav_map.launch.py`) | map_server loads a saved map, AMCL localizes with a particle filter | **Final** normal operation (default mode of `amr_bringup.sh`) |

Nav2 choices (`amr_navigation/config/nav2_params.yaml`):
- **Controller: Regulated Pure Pursuit** (not Nav2's default DWB): simple, smooth path tracking, light on the Pi, slows
  down on tight curves and near obstacles, rotates in place towards the path first.
- **Planner: NavFn**, global costmap in the `map` frame; local costmap in the `odom` frame (no jumps when AMCL corrects).
- **Composition** (`use_composition:=True`): all Nav2 servers in one process to save RAM on the Pi.
- Conservative start values: 0.25 m/s cruise speed, AMCL with 300–1500 particles to save CPU.

---

## 4. Getting velocity commands to the base

| Version | How `/cmd_vel` reached the wheels |
|---|---|
| First navigation package (`amr-diff-drive/amr_navigation.zip`) | Written for approach 1A: a `cmd_vel_relay.py` node copied Nav2's plain `Twist` on `/cmd_vel` to the ros2_control controller's topic (`/diff_drive_controller/cmd_vel`, `TwistStamped` with a fresh stamp, because the controller drops old stamps) |
| Second navigation package (`amr_navigation(1).zip`, final) | Relay removed: `amr_diff_drive` listens on `/cmd_vel` directly; odometry topic changed from `/diff_drive_controller/odom` to `/odom` |

**Why:** dropping ros2_control (§1) made the bridge unnecessary; one node less, one topic less.

---

## 5. Teleoperation

| Approach | Pros | Cons |
|---|---|---|
| `teleop_twist_keyboard` | Standard, many keys, adjustable speed | Publishes only on key presses, while `amr_diff_drive` zeroes the wheels 0.5 s after the last `/cmd_vel` (`cmd_vel_timeout`), so the robot only moves while keys keep arriving |
| `wasd_teleop.py` (final, now `amr_teleop`) | Publishes the current command continuously at 50 Hz from a background thread, so the 0.5 s timeout never triggers while driving; sends zero on exit | Fixed speeds (0.1 m/s, 1.0 rad/s) |

Both still work; `amr_teleop` is the one documented. TODO (team): what was observed with `teleop_twist_keyboard` on the robot.

---

## 6. Pi ⇄ ESP32 interface contract

Because the Pi and ESP32 sides were built by different teams in parallel, the interface was fixed early:
- **Units:** the mentors asked the motor-control and diff-drive teams to agree whether wheel velocities are sent in
  m/s or rad/s (24 Sep). The final contract is **rad/s** for commands (`/left_vel`, `/right_vel`) and rad / rad/s for
  encoder state; the motor team's first plan had been an m/s → PWM conversion.
- **Wheel state:** the mentors also asked whether wheel state should be linear or angular; it's **angular** (positions in
  rad, cumulative), which is what odometry integration needs.
- **Topic names and message formats** were defined by the micro-ROS team on 25 Sep and tested with dummy wheel data
  before the real firmware existed ([topics doc](https://docs.google.com/document/d/1CKC5PaiLtuVq5qNwfhTToIpuMQxy-mmduqVFJ-J3ZQc/edit?usp=sharing)).

Fixing the contract first let every vertical test against dummy data and integrate in one night (25 Sep).

---

## 7. Communication with the ESP32

micro-ROS over Wi-Fi (UDP to an agent on the Pi, port 8888) was used from the start (`setup_script.sh`).
Wired serial micro-ROS was the alternative. Wi-Fi keeps the robot untethered from a USB cable between boards
but adds latency and jitter, which is why the timestamping (§1B), the ESP32's time sync with the Pi
(`rmw_uros_sync_session`), disabling Wi-Fi power saving on the ESP32, and `chrony` on the Pi were added.
TODO (team): whether serial was tried, and why Wi-Fi was preferred.

---

## 8. Starting the system

| Approach | Problem |
|---|---|
| One terminal per component (agent, TF, diff drive, LiDAR, SLAM/Nav2) | Error-prone order, leftover processes after crashes, duplicate nodes |
| `amr_bringup.sh` (final) | Starts everything in order, waits for each topic, logs each program to its own file, Ctrl-C stops all of it, cleans up leftovers from previous runs |
