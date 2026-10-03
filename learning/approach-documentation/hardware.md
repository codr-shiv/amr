# Hardware approaches

Every electrical/electronic component that was discussed or tried, what it was meant to solve, the issues found,
and why the final part was chosen. Sources: the code in [`../iteration-phase/`](../iteration-phase/), the final
firmware and configs, the logs, and the early architecture slides. Anything not recorded there is marked
**TODO (team)**; general knowledge about a part is labelled as such and isn't presented as something the team measured.

Final hardware list and wiring: [final-product/docs/hardware.md](../../final-product/docs/hardware.md).

---

## 1. Motor driver

| | L298N (early design) | Cytron dual-channel driver (final) |
|---|---|---|
| Where it appears | Early architecture slides ("Motor Drivers (e.g., L298N)") | Component list (3 × Cytron drivers, 23 Sep) and the final robot; firmware drives it in PWM + DIR mode |
| Problem it solves | Drive two DC motors from ESP32 logic signals | Same |
| Issues encountered | TODO (team) | TODO (team) |
| Pros | Cheap, widely available | One PWM + one DIR pin per motor, which is exactly what the firmware outputs; accepts 20 kHz PWM (inaudible) |
| General notes (not measured by the team) | Bipolar H-bridge: a few volts of drop and noticeable heat at higher currents | MOSFET H-bridge: much lower drop and heat |

**Why Cytron:** the L298N was only an example on the architecture slides ("e.g."); the component list issued on 23 Sep,
when work started, already had Cytron drivers, so no L298N was ever built into the robot. TODO (team): the reason for the
choice (e.g. current rating, heat, voltage drop, availability) and the model.

---

## 2. LiDAR

| | LDROBOT LD14 (considered) | Slamtec RPLIDAR A1 (final) |
|---|---|---|
| Where it appears | Architecture slides and the Nav2 docs list "RPLIDAR / LD14" as options | Driver cloned into `slam/src/rplidar_ros`; used in every map and test |
| Problem it solves | 360° 2D scans for SLAM, localization and obstacle avoidance | Same |
| Issues encountered | Not tried | Needs serial-port permission on the Pi (`dialout` group or udev rule); TODO (team): anything else |
| Pros | Compact | Mature ROS 2 driver from the manufacturer; ~12 m range in the Sensitivity mode used (8 kHz sample rate, 10 Hz scans, from `amr_logs/rplidar.log`) |

**Why RPLIDAR A1:** TODO (team) (e.g. already available, driver support).

---

## 3. Low-level controller: ESP32

| | |
|---|---|
| Problem it solves | Real-time wheel control (encoder counting, 50 Hz PID) that a Linux Pi can't do reliably, and a bridge to ROS 2 |
| Pros | Hardware quadrature counters (`ESP32Encoder`), two cores (control loop on its own core, never blocked by Wi-Fi), built-in Wi-Fi, supported by micro-ROS |
| Issues encountered | Wi-Fi latency and jitter on encoder data: 10-s window means of 15–242 ms (median 81 ms), single samples up to 410 ms (`amr_logs/diff_drive.log`), which led to timestamped telemetry and pose prediction on the Pi; the clock must be synced with the Pi for the stamps to be usable; the micro-ROS agent must be running or the ESP32 loses its link (the final firmware reconnects automatically); setup problems are listed in the [ESP32 troubleshooting](../../final-product/firmware/esp32/docs/05_troubleshooting.md) |
| Alternatives | TODO (team): was an Arduino / STM32 or a wired (serial) link considered? |

---

## 4. Main computer: Raspberry Pi

| | |
|---|---|
| Problem it solves | Runs ROS 2 Humble: LiDAR driver, SLAM Toolbox / AMCL, Nav2, odometry node, micro-ROS agent |
| Pros | Small, low power, runs Ubuntu 22.04 + ROS 2 natively |
| Issues encountered | Limited CPU/RAM for Nav2: handled with composition (all Nav2 servers in one process), fewer AMCL particles, lower costmap update rates, SLAM processing a scan only every 0.3 m / 0.3 rad; RViz runs on a laptop instead |
| Model | TODO (team) |

---

## 5. Motors and encoders

| | |
|---|---|
| Final | 2 × Pro-Range 24 V planetary gear DC motors with Hall quadrature encoders (hardware team): left ≈ 26.9 : 1 (752.6 counts per wheel revolution), right ≈ 19.1 : 1 (536.1 counts). 4× decoding |
| Issues encountered | Only one motor was tested on day 1 (24 Sep). Encoder calibration done on one motor gave values that didn't carry over to the others, so every motor was calibrated separately (24 Sep). After PID worked, the motor deadband under load still needed tuning (25 Sep). The two gear ratios differ, so each wheel has its own CPR, feed-forward and PI gains, and both are limited to 17 rad/s so they reach the same top speed |
| Pros | Planetary gearbox; built-in Hall encoder enables closed-loop speed control and wheel odometry |
| Open | TODO (team): why the two gearboxes differ |

---

## 6. Chassis and mechanical

| | |
|---|---|
| Requirements (23 Sep) | robust chassis, stable motor mounting, wheels on the motors without vibration, LiDAR at an appropriate height, enough room to work on the electronics |
| Final | T-slot aluminium frame with a base board; 3D-printed wheel/motor mounts and LiDAR mount; castor wheels |
| Issues encountered | Printed parts "fit almost properly" at first (24 Sep); castors had to be levelled with the drive wheels (25 Sep); some parts were reprinted on 27 Sep after an overnight print was cancelled halfway and the printer had problems, so the robot was reassembled that evening |
| Mentors' note | T-slots were optional: what matters is stability and easy access to the electronics |
| CAD | [final-product/cad/](../../final-product/cad/README.md) |

---

## 7. Power

| | |
|---|---|
| Bench testing | DC bench power supply (component list, 23 Sep) |
| On the robot | onboard battery; TODO (team): chemistry, voltage, capacity, how the 24 V motors, the Pi and the ESP32 are supplied |

---

## 8. Communication link Pi ↔ ESP32

| | micro-ROS over Wi-Fi / UDP (final) | micro-ROS over USB serial (alternative) |
|---|---|---|
| Pros | No cable between the boards; the ESP32 appears as normal ROS 2 topics | Lower, steadier latency; no network setup |
| Cons / issues | Latency and jitter (see §3); needs the Pi's IP compiled into the firmware, and the Pi's IP isn't fixed | Cable between boards; TODO (team) |
| Tried? | Yes (final) | TODO (team) |
