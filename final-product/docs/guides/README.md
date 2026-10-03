# Implementation guides

In-depth guides to every part of the AMR, written from the actual code and configuration in this repo.
They explain *how each part is implemented and why*, down to parameters, formulas and message fields, so you can
change or debug it without the original team.

Read the [architecture overview](../architecture.md) first for the big picture.

| # | Guide | Covers |
|---|---|---|
| 1 | [System bringup, TF and teleop](01-system-bringup.md) | workspaces and build, `amr_bringup.sh` line by line, process groups and shutdown, logs, TF tree, keyboard teleop, network |
| 2 | [micro-ROS communication](02-micro-ros-communication.md) | agent and client, UDP transport, entities and QoS, static message memory, connection state machine, time sync, ROS domain ID, observed link stability |
| 3 | [ESP32 motor control](03-esp32-motor-control.md) | FreeRTOS task layout, shared state, control law (ramp, feed-forward + PI, anti-windup), Cytron PWM/DIR, modes, serial console, safety, tuning |
| 4 | [Encoder decoding](04-encoder-decoding.md) | quadrature 4× decoding in hardware, CPR, position and velocity estimation, filtering, telemetry message |
| 5 | [Differential drive controller and hardware interface](05-diff-drive-controller.md) | `amr_diff_drive`: kinematics, odometry integration, latency compensation, watchdogs, publishing; mapping to ros2_control and the ros2_control version tried first |
| 6 | [RPLIDAR A1](06-rplidar.md) | driver parameters and internals, `/scan` format, serial permissions, `base_link → laser` mount |
| 7 | [SLAM Toolbox](07-slam-toolbox.md) | how graph SLAM runs here, every parameter, the alternative config, making and saving maps |
| 8 | [Nav2](08-nav2.md) | what gets launched, lifecycle, goal → wheels chain, behavior tree, AMCL, costmaps, planner, Regulated Pure Pursuit, velocity smoother, operation, tuning |

Data flow through the guides, from sensors to motors:

```
RPLIDAR (6) ─► SLAM Toolbox (7) / AMCL + Nav2 (8) ─► /cmd_vel ─► amr_diff_drive (5) ─► micro-ROS (2) ─► ESP32 control (3) ─► motors
                                                                     ▲                                        │
                                                                     └──── /encoder_telemetry ◄── encoders (4)┘
```
