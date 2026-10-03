# ESP32 firmware: wheel PID + encoders + micro-ROS

`amr_esp32/amr_esp32.ino` is the Arduino sketch for the AMR's low-level controller:

- **Per-wheel velocity control:** feed-forward + PI, setpoint ramp, anti-windup, 50 Hz.
- **Quadrature encoder decoding:** 4× (full quad) via `ESP32Encoder`, giving position and filtered velocity.
- **micro-ROS over Wi-Fi (UDP):** receives wheel targets from the Raspberry Pi and sends encoder telemetry back.
- **Serial console:** bench tests and live PID tuning without ROS.

It is the "ESP32 (micro-ROS)" block of the system architecture. On the Pi side it talks to
[`amr_diff_drive`](../../amr_ws/src/amr_diff_drive/README.md) through the micro-ROS agent.

```
 amr_diff_drive (Pi) ──/left_vel, /right_vel──► micro-ROS agent ──UDP:8888──► ESP32 ──PWM+DIR──► Cytron motor driver ──► motors
        ▲                                        (udp4, Pi)                    │
        └──────────── /encoder_telemetry ◄─────────────────────────────────────┘◄── encoders
```

---

## 1. ROS 2 interface

| Direction | Topic | Type | QoS | Content |
|---|---|---|---|---|
| Pi → ESP32 | `/left_vel` | `std_msgs/msg/Float64` | reliable | left wheel target, **rad/s** |
| Pi → ESP32 | `/right_vel` | `std_msgs/msg/Float64` | reliable | right wheel target, **rad/s** |
| ESP32 → Pi | `/encoder_telemetry` | `sensor_msgs/msg/JointState` | best effort, **20 Hz** | `name: [left_wheel, right_wheel]`, `position` rad (cumulative since boot), `velocity` rad/s (low-pass filtered), `header.stamp` = Pi time |

Node name: `esp32_wifi_wheel_node`. Positive values mean the wheel drives the robot forward.

How this matches the Pi side (`amr_diff_drive/config/diff_drive.yaml`):

| Firmware | Pi (`amr_diff_drive`) |
|---|---|
| Stops the motors if `/left_vel` or `/right_vel` are older than **500 ms** (`CMD_TIMEOUT_MS`) | Publishes both at **50 Hz** continuously, including `0.0` when idle |
| Clamps targets to **17 rad/s**, scaling both wheels together (`*_MAX_SPEED`) | Clamps to the same `max_wheel_speed: 17.0` (change both together) |
| Joint names `left_wheel`, `right_wheel` | `left_joint_name`, `right_joint_name` |
| `header.stamp` from `rmw_uros_epoch_nanos()` after `rmw_uros_sync_session()` | `use_encoder_stamp: true` uses it for latency compensation |
| Falls back to `millis()` (time since boot) as the stamp if time sync failed | Rejects that stamp (offset > `max_stamp_offset`) and uses receive time instead, logging a warning |

---

## 2. Firmware architecture

| Part | Runs in | What it does |
|---|---|---|
| `controlTask` | FreeRTOS task, **core 1**, fixed **50 Hz** (`vTaskDelayUntil`) | read encoders → velocity filter → setpoint ramp → feed-forward + PI → PWM/DIR. Wi-Fi and micro-ROS never block it. |
| `loop()` | Arduino loop | micro-ROS state machine (wait → connect → spin/publish → reconnect), serial commands, serial status output |
| `sh` (struct `Shared`) | shared, protected by a spinlock | the only data exchanged between the two: targets and mode in, positions, velocities and PWM out |

Control law per wheel, in PWM counts (0..1023):

```
ref   = ramp(target, ACCEL_LIMIT)                          # rad/s, max 40 rad/s²
u     = kff·ref + sign(ref)·pwmMin + kp·(ref − meas) + I   # I clamped to ±I_LIMIT, frozen when saturated
meas  = VEL_ALPHA·raw + (1 − VEL_ALPHA)·meas               # raw = Δcounts · 2π / (CPR · dt)
```

With `target = 0` and the ramp at 0, the motor is switched off and the integrator reset (no creep at standstill).

micro-ROS connection states: `WAITING_AGENT` (ping every 500 ms) → `AGENT_AVAILABLE` (create node, subscriptions, publisher, time sync) → `AGENT_CONNECTED` (ping every 1 s, spin, publish at 20 Hz) → `AGENT_DISCONNECTED` (stop the motors, destroy entities, back to waiting). **No reset is needed** when the Pi or agent restarts.

---

## 3. Configuration (top of the sketch)

### Network

| Constant | Value | Notes |
|---|---|---|
| `SSID_NAME` / `SSID_PASSWORD` | placeholders (`YOUR_WIFI_SSID` / `YOUR_WIFI_PASSWORD`) | **Set before compiling.** The Wi-Fi network the Pi is on. |
| `AGENT_IP` | example `192.168.0.100` | **Set before compiling.** The Raspberry Pi's current IP; it isn't fixed, so check it on the Pi with `hostname -I` every time. |
| `AGENT_PORT` | `8888` | Must match `AGENT_PORT` in `amr_bringup.sh` |

### Wiring (Cytron motor driver, PWM + DIR mode)

| Signal | Left wheel (motor 1, 26.9:1) | Right wheel (motor 2, 19.1:1) |
|---|---|---|
| Encoder A / B | GPIO 32 / 33 | GPIO 25 / 26 |
| PWM (speed) | GPIO 18 (LEDC ch 0) | GPIO 23 (LEDC ch 1) |
| DIR (direction) | GPIO 19 | GPIO 22 |

PWM: 20 kHz, 10-bit (0..1023). Encoder inputs use the ESP32's internal weak pull-ups and a glitch filter.

### Per-wheel parameters

| Constant | Left | Right | Meaning |
|---|---|---|---|
| `*_CPR` | 752.6 | 536.1 | encoder counts per **wheel** revolution (4× decoding, after gearbox) |
| `*_ENC_INVERT` | `true` | `false` | flip so forward = positive position |
| `*_MOTOR_INVERT` | `true` | `false` | flip so positive PWM = forward |
| `*_KFF` | 49.72 | 43.54 | feed-forward, PWM per rad/s |
| `*_PWM_MIN` | 11.8 | 12.0 | PWM added to overcome static friction |
| `*_KP` | 30.0 | 25.0 | proportional gain, PWM per rad/s of error |
| `*_KI` | 12.5 | 10.0 | integral gain |
| `*_MAX_SPEED` | 17.0 | 17.0 | rad/s limit (both wheels scaled together to keep the curve) |

The two motors have different gearboxes, which is why CPR and gains differ per side.

### Control and timing

| Constant | Value | Meaning |
|---|---|---|
| `LOOP_HZ` | 50 | control rate |
| `VEL_ALPHA` | 0.3 | velocity low-pass filter (higher = less filtering) |
| `ACCEL_LIMIT` | 40.0 rad/s² | setpoint ramp |
| `I_LIMIT` | 400 | max PWM from the integral term |
| `PUB_INTERVAL_MS` | 50 | telemetry period (20 Hz) |
| `CMD_TIMEOUT_MS` | 500 | stop if no command for this long (`0` = never; don't use on the robot) |
| `DEBUG_ROS_RX` | `false` | print every received command (slows the loop) |

---

## 4. Build and flash (Arduino IDE)

> ⚠️ **Before every compile/upload**, edit these three constants at the top of `amr_esp32.ino`:
> - `SSID_NAME`, `SSID_PASSWORD`: your Wi-Fi network. The repo only has placeholders; don't commit your real password.
> - `AGENT_IP`: the Pi's **current** IP (`hostname -I` on the Pi). It can change between sessions.
>
> With the wrong values the ESP32 hangs at `Wi-Fi connecting` or `waiting for micro-ROS agent` (section 8).

1. **File > Preferences**: add the ESP32 board manager URL
   `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
2. **Tools > Board > Boards Manager**: install **esp32** by Espressif. The sketch supports both the 2.x and 3.x core LEDC APIs.
3. Install the libraries:
   - **micro_ros_arduino, Humble release** (`v2.0.7-humble.zip` from the micro_ros_arduino GitHub releases) via **Sketch > Include Library > Add .ZIP Library**. It must match the Pi's ROS 2 distro (Humble).
   - **ESP32Encoder** (by Kevin Harrington) via **Library Manager**.
4. Open `firmware/esp32/amr_esp32/amr_esp32.ino`, set `SSID_NAME`, `SSID_PASSWORD` and `AGENT_IP` (see the note above), select your ESP32 board and port, and **Upload**.

---

## 5. Running

The micro-ROS agent must run on the Pi. `amr_bringup.sh` starts it first and waits for `/encoder_telemetry`,
so normally you just power the ESP32 and run the bringup script (see the [main README](../../README.md)).

To run the agent by hand:
```bash
source /opt/ros/humble/setup.bash
source ~/amr/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Serial monitor (115200 baud) on boot:
```
# === AMR ESP32 firmware: PID + encoders + micro-ROS ===
# L: kff=49.72 min=11.8 kp=30.00 ki=12.50 max=17.0 rad/s  cpr=752.6
# R: kff=43.54 min=12.0 kp=25.00 ki=10.00 max=17.0 rad/s  cpr=536.1
# Wi-Fi connecting.....
# Wi-Fi connected, ESP32 IP = 192.168.0.xxx
# waiting for micro-ROS agent at 192.168.0.100:8888 ...
# micro-ROS: CONNECTED to agent
[ROS OK |ROS] tgt L=  0.00 R=  0.00 | meas L=  0.00 R=  0.00 rad/s | pos L=    0.00 R=    0.00 rad | pwm L=    0 R=    0
```

### Check from the Pi (wheels lifted off the ground)
```bash
ros2 topic hz /encoder_telemetry            # ~20 Hz
ros2 topic echo /encoder_telemetry --once
# Commands must be sent CONTINUOUSLY (>= 5 Hz) or the 500 ms timeout stops the motors:
ros2 topic pub -r 10 /left_vel  std_msgs/msg/Float64 "{data: 5.0}" &
ros2 topic pub -r 10 /right_vel std_msgs/msg/Float64 "{data: 5.0}"
```
Don't do this while `amr_diff_drive` is running: it publishes `/left_vel` and `/right_vel` too.

---

## 6. Serial commands (bench tests and tuning)

115200 baud, line ending **Newline**. These work without ROS.

| Command | Effect |
|---|---|
| `ros` | hand control back to ROS (default at boot) |
| `s` | STOP and hold; ROS commands are ignored until `ros` |
| `t 10` / `t 10 5` | closed-loop target in rad/s for both wheels / left and right (ignores ROS) |
| `sq 10 2000` | square wave 10 ↔ 0 rad/s, period 2000 ms (step-response tuning) |
| `pwm 300 300` | open-loop PWM (−1023..1023), PID bypassed |
| `kp L 12` | set a parameter for `L`, `R` or `B` (both): `kp`, `ki`, `kff`, `min`, `max` (resets the integrator) |
| `p` | print the parameters |
| `plot 0` / `plot 1` / `plot 2` | 1 Hz status text / speeds for the Arduino Serial Plotter / speeds + PWM % |

Tuning changes live only in RAM. Copy good values into the `L_*` / `R_*` constants and re-flash.

Typical tuning flow:
1. `pwm 300 300`: check that both wheels turn **forward**. If one doesn't, flip its `*_MOTOR_INVERT`.
2. Check that `pos` increases for both. If one decreases, flip its `*_ENC_INVERT`.
3. `plot 1`, then `sq 10 2000`: tune `kff` and `min` first (the steady state should be close without PI), then `kp` and `ki` for a fast step without overshoot.
4. `ros` to hand control back.

---

## 7. Safety behaviour

| Situation | Behaviour |
|---|---|
| Agent not connected / connection lost | motors stopped, stored ROS targets cleared, reconnects automatically |
| No `/left_vel` or `/right_vel` for 500 ms | motors stopped (both commands must be fresh) |
| NaN / inf / absurd (> 1e6) command | ignored |
| Target above `*_MAX_SPEED` | both wheels scaled by the same factor |
| Boot, before Wi-Fi | control task already running with targets = 0 |

---

## 8. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Stuck at `# Wi-Fi connecting.....` | wrong `SSID_NAME` / `SSID_PASSWORD`, or network out of range (the ESP32 needs 2.4 GHz) |
| Stuck at `waiting for micro-ROS agent` | agent not running on the Pi, or `AGENT_IP` isn't the Pi's current IP (`hostname -I`), or `AGENT_PORT` mismatch |
| `ros2 topic list` on the Pi doesn't show `/encoder_telemetry` | **ROS domain mismatch**: the sketch doesn't set a domain ID, so it uses domain 0; if the Pi uses `ROS_DOMAIN_ID=30`, set the domain in the firmware or leave it unset on the Pi ([micro-ROS guide §8](../../docs/guides/02-micro-ros-communication.md)). Also see the rows above |
| Wheels stop after ½ s | commands sent once (`--once`) instead of continuously; use `-r 10`, or run `amr_diff_drive` |
| `amr_diff_drive` warns `Encoder stamp not used (stamp offset ...)` | time sync failed, so the stamps are time-since-boot. Reset the ESP32 once the agent is up; check the Pi's clock (`chronyc tracking`). Odometry still works, just without latency compensation. |
| A wheel spins backwards / odometry turns the wrong way | fix `*_MOTOR_INVERT` / `*_ENC_INVERT` in the firmware (section 6), not on the Pi |
| Wheel oscillates or hums | lower `kp` / `ki` for that wheel, or raise the filtering (lower `VEL_ALPHA`) |

### Errors hit during the original setup

**APT GPG key conflict.** `sudo apt update` failed with
`E: Conflicting values set for option Signed-By regarding source http://packages.ros.org/ros2/ubuntu/ jammy...`.
An old ROS `.list` file with an inline key conflicted with the keyring method. Fix:
```bash
sudo rm -f /etc/apt/sources.list.d/ros*.list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

**"Dirty build" linker errors** (`undefined reference to dds::xrce::...`) when building the agent.
An earlier build ran without `source /opt/ros/humble/setup.bash` and left broken artifacts. Fix: wipe and rebuild
(`scripts/build.sh` sources ROS itself):
```bash
cd ~/amr/microros_ws && rm -rf build/ install/ log/
~/amr/scripts/build.sh agent
```

**`Waiting for at least 1 matching subscription(s)...`** when publishing to the ESP32.
The agent had been stopped, so the ESP32 had no bridge to ROS. Keep the agent running, since `amr_bringup.sh` does this.
With the old test sketch the ESP32 had to be reset (`RST`) after the agent restarted; the current firmware reconnects by itself.
