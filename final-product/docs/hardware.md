# Hardware

What the final robot is built from and how it's wired. Values come from the code (firmware pin map,
`diff_drive.yaml`, `laser_tf.launch.py`) and the team's [project log](../../learning/project-log.md);
anything neither records is marked **TODO (team)**.

![The finished AMR: T-slot frame, RPLIDAR A1 on the orange 3D-printed mount at the front, Raspberry Pi, Cytron motor driver, ESP32 on a breadboard, blue 3D-printed wheel mounts](images/amr-robot.jpg)

## Bill of materials

| # | Component | Used for | Key specs used by the software | Model / qty / cost |
|---|---|---|---|---|
| 1 | Raspberry Pi | ROS 2 computer: navigation, SLAM, diff drive node, micro-ROS agent | Ubuntu 22.04, ROS 2 Humble | TODO (team): model, RAM |
| 2 | ESP32 dev board | Wheel speed control, encoder decoding, micro-ROS client over Wi-Fi | GPIO map below | TODO (team): board |
| 3 | Cytron dual-channel motor driver | Drives both motors from PWM + DIR signals | PWM 20 kHz, 10-bit | TODO (team): model (e.g. MDD10A / MDD20A) |
| 4 | Left planetary geared DC motor with quadrature encoder | Left wheel | 12 V; **26.9:1** gearbox (different from the right motor), 752.6 counts per wheel revolution (4× decoding) | TODO (team): exact model |
| 5 | Right planetary geared DC motor with quadrature encoder | Right wheel | **PG36M555-19.2K**: 12 V, 262 RPM, 45 N·cm, 19.2:1; encoder ME-37, 7 PPR → 28 counts per motor revolution (4×) → 536.1 counts per wheel revolution (calibrated) | from the project log's component list |
| 6 | Wheels (×2) | Drive | radius 0.056 m, separation 0.39 m (nominal) | TODO (team) |
| 7 | Slamtec RPLIDAR A1 | 2D laser scans for mapping, localization and obstacle avoidance | USB serial `/dev/ttyUSB0`, 115200 baud, ~10 Hz, 12 m range | 1 |
| 8 | Castor wheels | Support (levelled with the drive wheels) | — | TODO (team): qty |
| 9 | Onboard battery + power distribution | Power for motors, Pi, ESP32 while driving (a DC bench power supply was used for bench tests) | motors are 12 V | TODO (team): chemistry, voltage, capacity, regulators |
| 10 | Chassis | Structure: T-slot aluminium frame with a base board, 3D-printed wheel/motor mounts and LiDAR mount | outline ≈ 0.50 × 0.46 m assumed in Nav2 (placeholder) | see [CAD](../cad/README.md) |
| 11 | Wi-Fi router / access point | Network between Pi, ESP32 and laptop | 2.4 GHz (the ESP32 can't use 5 GHz) | — |

The two motors have different gear ratios; the firmware handles this with per-wheel CPR and PID gains, and caps both
wheels at 17 rad/s (the 19.2:1 motor's no-load speed is 262 RPM ≈ 27.4 rad/s). Encoder counts were calibrated per
motor, because the values differed between motors (project log, 24 Sep).

## ESP32 pinout

| Signal | Left wheel | Right wheel |
|---|---|---|
| Encoder A / B | GPIO 32 / 33 | GPIO 25 / 26 |
| PWM (speed) → Cytron | GPIO 18 (LEDC ch 0) | GPIO 23 (LEDC ch 1) |
| DIR (direction) → Cytron | GPIO 19 | GPIO 22 |

Encoder inputs use the ESP32's internal weak pull-ups and a hardware glitch filter.
Wiring diagram: [architecture.md §7](architecture.md#7-circuit-level).

## Sensor mounting

| Item | Value | Defined in |
|---|---|---|
| LiDAR position relative to `base_link` | x 0.175 m (forward), y 0, z 0.100 m, no rotation | `amr_ws/src/amr_navigation/launch/laser_tf.launch.py` |
| `base_link` | middle of the wheel axle (the point the robot rotates about) | convention used by all configs |

If the LiDAR is moved or rotated, update `laser_tf.launch.py` (and the CAD).

## Power wiring

**TODO (team):** battery → switch/fuse → motor driver supply; regulator → Raspberry Pi 5 V; ESP32 supply;
common ground between ESP32 and the motor driver. A photo or schematic of the wiring belongs here.

## Serial port access (LiDAR)

The Pi user must be allowed to open `/dev/ttyUSB0`: `sudo usermod -aG dialout $USER`, or install the driver's
udev rule with `amr_ws/src/rplidar_ros/scripts/create_udev_rules.sh`.
