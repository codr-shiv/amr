# Known issues log

Open problems found in the code, configs and the 28 Sep 2026 run logs (`learning/iteration-phase/amr_logs/`).
Add a row when you find a new issue; move it to "Resolved" with the fix when it's done.

| # | Issue | Evidence | Impact | Suggested fix |
|---|---|---|---|---|
| 1 | **ROS domain mismatch risk.** The firmware doesn't set a ROS domain (→ domain 0); the docs recommend `ROS_DOMAIN_ID=30` on the Pi | `rclc_support_init(&support, 0, NULL, &allocator)` in `amr_esp32.ino` | Pi never sees `/encoder_telemetry`; bringup stalls at step 1 | Set the domain in the firmware, or leave `ROS_DOMAIN_ID` unset ([guide 2 §8](guides/02-micro-ros-communication.md)) |
| 2 | **Unstable micro-ROS link.** 203 agent sessions in 22 min (median 1.7 s apart); 387 `Encoder telemetry stale` warnings | `agent.log`, `diff_drive.log` | Motors stop on every disconnect | Check Wi-Fi signal, ping timeouts, command rate ([guide 2 §9](guides/02-micro-ros-communication.md)) |
| 3 | **High encoder latency.** 10-s window means 15–242 ms (median 81 ms), samples up to 410 ms | `diff_drive.log` | Latency above 0.2 s isn't compensated → odometry lag, smeared scans | Fix together with #2; consider raising `max_extrapolation_time` only after the link is stable |
| 4 | Nav2 footprint is a placeholder (0.50 × 0.46 m) | `nav2_params.yaml` | Clipping obstacles or refusing gaps | Measure the robot |
| 5 | Wheel radius / separation are nominal | `diff_drive.yaml` | Odometry drift, AMCL errors | Calibrate ([calibration.md](calibration.md)) |
| 6 | No URDF / robot model | only a static laser TF | No robot body in RViz | Add URDF + `robot_state_publisher` from the CAD |
| 7 | Teleop and Nav2 both publish `/cmd_vel` (no mux) | launch files | Commands fight if both run | Don't run together, or add `twist_mux` |
| 8 | Wi-Fi credentials and Pi IP are compile-time constants | `amr_esp32.ino` | Re-flash when the Pi's IP changes | Store settings in flash, set over serial |
| 9 | `mapper_params_real.yaml` uses two unknown parameter names | `minimum_laser_range`, `maximum_laser_range` | Ignored (default 20 m range used) | Rename to `max_laser_range` if that config is used |

## Resolved

| Date | Issue | Fix |
|---|---|---|
| 2026-09 | Speed limit mismatch (Pi 18 rad/s vs firmware 17) | Both 17 rad/s |
| 2026-09 | Global/local costmap footprints differed | Both 0.50 × 0.46 m |
| 2026-09 | `humaramap.yaml` used `~` in the image path (not expanded by map_server) | Relative path |
