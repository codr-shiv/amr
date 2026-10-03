# RPLIDAR A1 (LiDAR driver and mounting)

How laser scans get into ROS 2: the driver, its launch file and parameters, what the published `/scan` looks like,
serial-port access, and the `base_link → laser` transform.

Code: [`amr_ws/src/rplidar_ros/`](../../amr_ws/src/rplidar_ros/) (Slamtec driver, package version 2.1.4, vendored copy),
[`amr_navigation/launch/laser_tf.launch.py`](../../amr_ws/src/amr_navigation/launch/laser_tf.launch.py) (mount transform).

---

## 1. The sensor (as detected on the robot)

From the driver log of 28 Sep 2026 (`learning/iteration-phase/amr_logs/rplidar.log`):

| Property | Value |
|---|---|
| Model / firmware / hardware rev | RPLIDAR A1, firmware 1.29, hardware rev 7, SDK 2.1.0 |
| Health | OK |
| Scan mode | **Sensitivity** |
| Sample rate | 8 kHz |
| Scan frequency | 10 Hz (≈ 800 points per revolution) |
| Max distance | 12.0 m (reported by the scan mode) |
| Min range | 0.15 m (set by the driver) |

## 2. How it's started

`amr_bringup.sh` step 4: `ros2 launch rplidar_ros rplidar_a1_launch.py` (log `~/amr_logs/rplidar.log`), then waits up to 30 s for `/scan`.

`launch/rplidar_a1_launch.py` starts `rplidar_node` (name `rplidar_node`) with these parameters:

| Parameter | Value in the A1 launch file | Node default | Meaning |
|---|---|---|---|
| `channel_type` | `serial` | `serial` | serial / tcp / udp |
| `serial_port` | `/dev/ttyUSB0` | `/dev/ttyUSB0` | USB-serial adapter of the A1 |
| `serial_baudrate` | `115200` | `1000000` | A1 uses 115200 |
| `frame_id` | `laser` | `laser_frame` | `header.frame_id` of `/scan`; must match the TF child frame |
| `inverted` | `false` | `false` | set true if mounted upside down |
| `angle_compensate` | `true` | `false` | resample to a fixed number of points per degree (below) |
| `scan_mode` | declared as a launch argument (default `Sensitivity`) but **not passed to the node** | empty | empty → the driver uses the sensor's *typical* mode, which is Sensitivity on the A1 |

Other node parameters, all at defaults: `topic_name: scan`, `flip_x_axis: false`, `auto_standby: false`,
`scan_frequency: 10.0`. Override on the command line, e.g. `ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/rplidar`.

## 3. What the driver does (`src/rplidar_node.cpp`)

1. Connects over serial, reads device info, checks health (aborts on error).
2. A-series: starts the motor by PWM (`setMotorSpeed(600)`), then starts scanning in the selected or typical mode.
   It computes `angle_compensate_multiple = points_per_revolution / 360 + 1` from the mode's sample rate and `scan_frequency`.
3. Loop: `grabScanDataHq()` blocks until one full revolution is collected (~100 ms), then `ascendScanData()` sorts the
   points by angle.
4. With `angle_compensate: true`, each point is written into a fixed array of `360 × multiple` slots by its angle, so
   every scan has the same number of evenly spaced beams (gaps stay at range 0 → `inf`).
5. `publish_scan()` builds the `sensor_msgs/LaserScan`:
   - `header.stamp` = time **before** `grabScanDataHq()` was called; `header.frame_id = laser`
   - angles converted from the sensor's clockwise convention to ROS counter-clockwise (`angle = π − sensor_angle`,
     data order reversed when needed), so the scan covers a full revolution in the `laser` frame
   - `range_min = 0.15`, `range_max = 12.0` (mode max distance); a measured distance of 0 → `+inf` (no return)
   - `ranges` in metres (`dist_mm_q2 / 4 / 1000`), `intensities` = quality / 4
   - `scan_time` = duration of the grab, `time_increment = scan_time / (n − 1)`
6. Publishes on `scan` with QoS reliable, keep last 10. (Nav2 costmaps and SLAM Toolbox subscribe with sensor-data QoS,
   which is compatible.)
7. Services `/stop_motor` and `/start_motor` (`std_srvs/Empty`) stop/start spinning.
8. On shutdown: motor speed 0, stop.

## 4. Serial port access

The A1's USB adapter is a Silicon Labs CP210x (`10c4:ea60`), which appears as `/dev/ttyUSB0`. The user running the
bringup needs permission to open it; either:
- `sudo usermod -aG dialout $USER` (log out and in once), or
- `amr_ws/src/rplidar_ros/scripts/create_udev_rules.sh`, which installs
  `KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"`
  → open permissions and a stable `/dev/rplidar` name (useful if another USB-serial device takes `ttyUSB0`;
  then pass `serial_port:=/dev/rplidar`).

## 5. Mounting transform `base_link → laser`

[`laser_tf.launch.py`](../../amr_ws/src/amr_navigation/launch/laser_tf.launch.py) (started by `amr_bringup.sh` step 2)
runs `tf2_ros static_transform_publisher` with `0.175 0 0.100 0 0 0 base_link laser`:

| x | y | z | roll | pitch | yaw |
|---|---|---|---|---|---|
| 0.175 m forward | 0 | 0.100 m up | 0 | 0 | 0 |

It's the only place these values are defined. Getting them right matters:
- a wrong **yaw** rotates every scan relative to the robot: walls smear or double when the robot turns, and SLAM/AMCL fail;
- a wrong **x** shifts obstacles in the costmap;
- if the LiDAR is mounted upside down, set roll = π (or the driver's `inverted: true`).

Check: RViz fixed frame `odom`, LaserScan decay time 30 s; drive straight and spin. Walls must stay sharp.

## 6. Who uses `/scan`

| Consumer | Uses |
|---|---|
| SLAM Toolbox | scan matching and mapping, `max_laser_range: 12.0` |
| AMCL | 60 beams per scan (`max_beams`), `laser_min_range 0.15`, `laser_max_range 12.0` |
| Nav2 local and global costmaps (obstacle layer) | marks obstacles up to 2.5 m, clears free space up to 3.0 m |

## 7. Checks and troubleshooting

```bash
ros2 topic hz /scan                              # ~10 Hz
ros2 topic echo /scan --once --field header      # frame_id: laser
ros2 run tf2_ros tf2_echo base_link laser        # Translation: [0.175, 0.000, 0.100]
ros2 service call /stop_motor std_srvs/srv/Empty # spin down (start_motor to restart)
```

| Symptom | Cause / fix |
|---|---|
| `Error, cannot bind to the specified serial port /dev/ttyUSB0` | no permission (§4), wrong port (`ls /dev/ttyUSB*`), or another process has it open |
| `health status: error` / motor not spinning | power (USB port current), cable; replug |
| Scan rotated / mirrored in RViz | laser TF yaw, or `inverted` |
| `/scan` in RViz but costmap empty | `frame_id` vs TF child frame mismatch (`laser`), missing `base_link → laser` |
| Robot body shows up as obstacles | parts of the chassis within the scan; raise `obstacle_min_range` / `raytrace_min_range` or mask them |
