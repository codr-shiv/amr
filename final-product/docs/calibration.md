# Calibration and tuning notes

Every value that depends on the physical robot, its current setting, and how to calibrate it.
**Status** says whether the value was measured/tuned or is still nominal.

| What | Current value | Status | Where it's set | How to calibrate |
|---|---|---|---|---|
| Wheel radius | 0.056 m | nominal | `amr_ws/src/amr_diff_drive/config/diff_drive.yaml` (`wheel_radius`) | [amr_diff_drive README §10](../amr_ws/src/amr_diff_drive/README.md): drive 3 m straight, compare with `/odom` |
| Wheel separation | 0.39 m | nominal | same file (`wheel_separation`) | same section: spin 5 full turns, compare yaw |
| Wheel speed limit | 17 rad/s | set | `diff_drive.yaml` (`max_wheel_speed`) **and** firmware `*_MAX_SPEED` | change both together |
| Wheel / encoder direction | R: motor and encoder inverted, L: not | set | firmware `*_MOTOR_INVERT`, `*_ENC_INVERT` | [firmware README, Debug and tuning commands](../firmware/esp32/README.md#debug-and-tuning-commands) (`pwm 300 300`, check `pos`) |
| Encoder counts per wheel rev | L 752.6, R 536.1 | set (gearbox-specific) | firmware `*_CPR` | turn a wheel exactly 10 revolutions by hand, read the count |
| Wheel PID + feed-forward | L: kff 49.72, min 11.8, kp 30, ki 12.5 · R: kff 43.54, min 12.0, kp 25, ki 10 | tuned in firmware | firmware `L_*` / `R_*` | [firmware README, Debug and tuning commands](../firmware/esp32/README.md#debug-and-tuning-commands): `plot 1`, `sq 10 2000`, adjust live, copy values into the sketch |
| LiDAR mount | x 0.175, y 0, z 0.100 m, yaw 0 | TODO: verify | `amr_ws/src/amr_navigation/launch/laser_tf.launch.py` | measure on the robot / from the CAD; with RViz fixed frame `odom` and scan decay 30 s, walls must stay sharp while spinning |
| Robot footprint (Nav2) | 0.50 × 0.46 m rectangle centred on `base_link` | placeholder | `amr_ws/src/amr_navigation/config/nav2_params.yaml` (local **and** global costmap) | measure the outer outline; [amr_navigation README §4](../amr_ws/src/amr_navigation/README.md) |
| LiDAR range | 12 m (AMCL, SLAM) | set for RPLIDAR A1 | `nav2_params.yaml` (`laser_max_range`), `slam_toolbox_params.yaml` (`max_laser_range`) | set to the sensor's usable range |
| Nav2 speed | 0.25 m/s, 1.0 rad/s | conservative start value | `nav2_params.yaml` (`desired_linear_vel`, `velocity_smoother`) | raise in small steps: [amr_navigation README §10](../amr_ws/src/amr_navigation/README.md) |
| Odometry covariance | pose/twist xy 0.001, yaw 0.01 | default | `diff_drive.yaml` | raise if AMCL trusts odometry too much |

Order that works: directions and CPR → wheel PID → wheel radius → wheel separation → LiDAR mount → footprint → Nav2 tuning.
Each step assumes the previous ones are right.

Record new values here when you change them (date, what, old → new, why).

| Date | What | Old → new | Why |
|---|---|---|---|
| 2026-09 | `max_wheel_speed` | 18 → 17 rad/s | match the firmware limit |
| 2026-09 | global costmap footprint | 0.40 × 0.44 → 0.50 × 0.46 m | make it match the local costmap (larger of the two) |
