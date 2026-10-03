# CAD models

Mechanical design files for the AMR.

**Expected contents**

| Item | Files |
|---|---|
| Full assembly | native source file(s) + a `STEP` export |
| Chassis / base plate | source + `STEP`, `DXF` if laser-cut |
| Motor mounts | source + `STL` (if 3D printed) |
| LiDAR mount | source + `STL` |
| Electronics mounts (Pi, ESP32, motor driver, battery) | source + `STL` |

**Conventions**
- Name files by part: `chassis_base.step`, `lidar_mount.stl`, ... and note the CAD tool + version below.
- Keep a `STEP` (and `STL` for printed parts) next to every native file so anyone can open them without the original tool.
- Units: millimetres.

**Keep the software in sync with the CAD**
- LiDAR position relative to `base_link` (middle of the wheel axle): `amr_ws/src/amr_navigation/launch/laser_tf.launch.py`
  (currently x 0.175 m, z 0.100 m).
- Robot outline for Nav2: `footprint` in `amr_ws/src/amr_navigation/config/nav2_params.yaml` (currently 0.50 × 0.46 m, placeholder).
- Wheel radius and separation: `amr_ws/src/amr_diff_drive/config/diff_drive.yaml`.

CAD tool / version: TODO (team)
