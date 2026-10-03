# Demo videos

One video per capability of the final robot. Keep each video short and focused on one feature. Put the file in this
folder as `short-description.mp4` (H.264, compressed; see the note below), or link it if it's hosted elsewhere.

Where a recording from the build week already shows a capability, it's linked below (from
[learning/testing-videos](../../learning/testing-videos/README.md)); replace it with a dedicated demo when one is recorded.

| # | Feature | What the video should show | How to reproduce | Video |
|---|---|---|---|---|
| 1 | Teleoperation | Driving forward (`w`), backward (`s`) and turning (`a`/`d`); any other key stops the robot, and it also stops when teleop is closed | `./amr_bringup.sh robot` + `ros2 run amr_teleop wasd_teleop` | [build-week: driving with tuned PID, 28 Sep](../../learning/testing-videos/media/2026-09-28_driving-tuned-pid.mp4) |
| 2 | SLAM mapping | The map growing in RViz while the robot is driven around; a loop closure when it returns to the start | `./amr_bringup.sh slam` + teleop | [build-week: SLAM mapping in RViz, 26 Sep](../../learning/testing-videos/media/2026-09-26_slam-mapping-rviz.mp4) |
| 3 | Saving and loading a map | Saving with `map_saver_cli`, then starting nav mode on that map | `ros2 run nav2_map_server map_saver_cli -f ~/amr/maps/<name>`, `MAP=... ./amr_bringup.sh` | TODO |
| 4 | Localization (AMCL) | 2D Pose Estimate, then the particle cloud converging as the robot moves | `./amr_bringup.sh` | TODO |
| 5 | Point-to-point navigation | Nav2 Goal in RViz; planned path and the robot following it to the goal | `./amr_bringup.sh`, Nav2 Goal | [build-week: Nav2 autonomous navigation in RViz, 28 Sep (6 min)](../../learning/testing-videos/media/2026-09-28_nav2-autonomous-navigation.mp4) |
| 6 | Obstacle avoidance | An object not in the map placed on the path; local costmap marks it and the path bends around it | same as 5 | TODO |
| 7 | Recovery behaviour | Robot blocked; Nav2 spins / backs up and replans | same as 5 | TODO |
| 8 | Wheel speed control (ESP32) | Serial plotter: step response of both wheels (`plot 1`, `sq 10 2000`) | [firmware README §6](../firmware/esp32/README.md) | [build-week: PID velocity plots, 25 Sep](../../learning/testing-videos/media/2026-09-25_pid-velocity-plots.mp4) |

For each video, add a one-line note under the table if something noteworthy happens (e.g. a failure and why).

**Compressing a video before adding it** (keeps the repo small):
```bash
ffmpeg -i input.mp4 -c:v libx264 -crf 28 -preset slow -c:a aac -b:a 64k -movflags +faststart output.mp4
```
