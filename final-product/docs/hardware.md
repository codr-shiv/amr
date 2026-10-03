# Hardware

What the final robot is built from and how it's wired. Values come from the code (firmware pin map,
`diff_drive.yaml`, `laser_tf.launch.py`) and the team's [project log](../../learning/project-log.md);
anything neither records is marked **TODO (team)**.

![The finished AMR: T-slot frame, RPLIDAR A1 on the orange 3D-printed mount at the front, Raspberry Pi, Cytron motor driver, ESP32 on a breadboard, blue 3D-printed wheel mounts](images/amr-robot.jpg)

## Bill of materials

| # | Component | Used for | Key specs used by the software | Model / qty / cost |
|---|---|---|---|---|
| 1 | Raspberry Pi | ROS 2 computer: navigation, SLAM, diff drive node, micro-ROS agent | Ubuntu 22.04, ROS 2 Humble | TODO (team): model, RAM |
| 2 | ESP32 DevKit (ESP32-WROOM, "ESP32 Dev Module" in Arduino IDE) | Wheel speed control, encoder decoding, micro-ROS client over Wi-Fi | GPIO map below | 1 |
| 3 | Cytron dual-channel motor driver | Drives both motors from PWM + DIR signals | PWM 20 kHz, 10-bit | TODO (team): model (e.g. MDD10A / MDD20A) |
| 4 | Left motor: Pro-Range 24 V planetary gear DC motor with Hall quadrature encoder | Left wheel | gear ratio ≈ **26.9 : 1**, **752.6 counts per wheel revolution** (4× decoding, measured) | 1 |
| 5 | Right motor: Pro-Range 24 V planetary gear DC motor with Hall quadrature encoder | Right wheel | gear ratio ≈ **19.1 : 1**, **536.1 counts per wheel revolution** (4× decoding, measured) | 1 |
| 6 | Wheels (×2) | Drive | radius 0.056 m, separation 0.39 m (nominal) | TODO (team) |
| 7 | Slamtec RPLIDAR A1 | 2D laser scans for mapping, localization and obstacle avoidance | USB serial `/dev/ttyUSB0`, 115200 baud, ~10 Hz, 12 m range | 1 |
| 8 | Castor wheels | Support (levelled with the drive wheels) | — | TODO (team): qty |
| 9 | Onboard battery + power distribution | Power for motors, Pi, ESP32 while driving (a DC bench power supply was used for bench tests) | 24 V motors (Cytron motor supply) | TODO (team): chemistry, voltage, capacity, regulators |
| 10 | Chassis | Structure: T-slot aluminium frame with a base board, 3D-printed wheel/motor mounts and LiDAR mount | outline ≈ 0.50 × 0.46 m assumed in Nav2 (placeholder) | see [CAD](../cad/README.md) |
| 11 | Wi-Fi router / access point | Network between Pi, ESP32 and laptop | 2.4 GHz (the ESP32 can't use 5 GHz) | — |

The two motors have different gearboxes; the firmware handles this with per-wheel CPR and controller parameters and caps
both wheels at 17 rad/s. `KFF` and `PWM_MIN` depend on the motor supply voltage (re-run `tools/03_motor_characterize`
after a battery change). Full hardware details from the hardware team: [firmware/esp32/README.md](../firmware/esp32/README.md).

## ESP32 pinout

| Signal | Left wheel | Right wheel |
|---|---|---|
| Encoder A / B | GPIO 32 / 33 | GPIO 25 / 26 |
| PWM (speed) → Cytron | GPIO 18 (LEDC ch 0) | GPIO 23 (LEDC ch 1) |
| DIR (direction) → Cytron | GPIO 19 | GPIO 22 |

Encoder inputs use the ESP32's internal weak pull-ups and a hardware glitch filter.

Wiring rules (hardware team): encoder VCC to the ESP32 **3V3** pin and encoder GND to ESP32 GND (ESP32 inputs aren't
5 V tolerant); ESP32 GND connected to the Cytron GND (common ground); motor supply only to the Cytron power terminals,
never to the ESP32. The serial firmware also uses GPIO 2 (status LED) and, with debug enabled, GPIO 16/17 (Serial2).
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
