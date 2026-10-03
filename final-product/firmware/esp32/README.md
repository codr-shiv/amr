# ESP32 low-level controller

The ESP32 is the robot's real-time motor controller. It reads the two wheel encoders, runs a velocity control loop for each wheel, drives the motors through the Cytron driver, and exchanges data with the Raspberry Pi over micro-ROS.

```
                 left_vel, right_vel (rad/s)
 Raspberry Pi  ─────────────────────────────►  ESP32  ──PWM + DIR──►  Cytron driver ──► motors
 (micro-ROS    ◄─────────────────────────────         ◄──A/B pulses──  wheel encoders
  agent)         encoder_telemetry (rad, rad/s)
```

This folder covers only the ESP32 side. Everything the Pi needs to know about it is in [the ROS 2 interface](#ros-2-interface) below.

## Contents

| Path | What it is |
|---|---|
| [`firmware/amr_esp32_wifi/`](firmware/amr_esp32_wifi/) | **The firmware running on the robot.** micro-ROS over Wi-Fi (UDP). |
| [`firmware/amr_esp32_serial/`](firmware/amr_esp32_serial/) | The same firmware with micro-ROS over a USB cable (wired). See [status](#status). |
| [`tools/`](tools/) | Four stand-alone bench sketches for checking encoders, measuring the motors and tuning the PID. |
| [`docs/01_encoder_theory.md`](docs/01_encoder_theory.md) | How quadrature encoders work and how the ESP32 decodes them. |
| [`docs/02_pid_control.md`](docs/02_pid_control.md) | The wheel velocity controller and how it was tuned. |
| [`docs/03_microros_interface.md`](docs/03_microros_interface.md) | Topics, message format, Wi-Fi and serial transports, and how the control and micro-ROS code were combined. |
| [`docs/04_bringup_procedure.md`](docs/04_bringup_procedure.md) | Step-by-step procedure from a bare motor to a robot driven from ROS, with all commands. |
| [`docs/05_troubleshooting.md`](docs/05_troubleshooting.md) | Symptoms, causes and fixes. |

## Hardware

| Part | Details |
|---|---|
| Controller | ESP32 DevKit (ESP32-WROOM, "ESP32 Dev Module" in Arduino IDE) |
| Motors | 2 × Pro-Range 24 V planetary gear DC motor with Hall quadrature encoder |
| Motor driver | Cytron driver in PWM + DIR mode |
| Left motor | Gear ratio ≈ 26.9 : 1, **752.6 counts per wheel revolution** |
| Right motor | Gear ratio ≈ 19.1 : 1, **536.1 counts per wheel revolution** |

The two motors have different gearboxes. The firmware handles this by giving each wheel its own counts-per-revolution value and its own controller parameters. See [docs/02_pid_control.md](docs/02_pid_control.md#4-two-different-motors).

## Pin map

| Signal | Left wheel | Right wheel |
|---|---|---|
| Encoder A | GPIO 32 | GPIO 25 |
| Encoder B | GPIO 33 | GPIO 26 |
| PWM to Cytron | GPIO 18 | GPIO 23 |
| DIR to Cytron | GPIO 19 | GPIO 22 |

Also required:

- Encoder VCC to the ESP32 **3V3** pin and encoder GND to ESP32 GND. ESP32 inputs are not 5 V tolerant.
- ESP32 GND connected to the Cytron driver GND (common ground).
- Motor supply connected only to the Cytron power terminals, never to the ESP32.

The serial firmware additionally uses GPIO 2 (status LED) and, only when debug is enabled, GPIO 16 / 17 (Serial2).

## ROS 2 interface

| Topic | Direction | Type | Content |
|---|---|---|---|
| `left_vel` | Pi → ESP32 | `std_msgs/Float64` | Left wheel target speed, **rad/s** |
| `right_vel` | Pi → ESP32 | `std_msgs/Float64` | Right wheel target speed, **rad/s** |
| `encoder_telemetry` | ESP32 → Pi | `sensor_msgs/JointState`, best effort, 20 Hz | `name = ["left_wheel", "right_wheel"]`, `position` = cumulative wheel angle in **rad**, `velocity` = filtered wheel speed in **rad/s**, `header.stamp` = time the encoders were read |

Rules the Pi side must follow:

1. **Publish `left_vel` and `right_vel` continuously**, at 5 Hz or more (20 to 50 Hz is typical), even when the speed does not change. If either topic is silent for 500 ms, both wheels stop.
2. **Send wheel angular speed in rad/s.** To convert from a linear speed in m/s, divide by the wheel radius.
3. **Positive means robot forward for both wheels.**
4. Speeds above 17 rad/s are scaled down. Both wheels are scaled by the same factor, so the robot keeps its commanded curvature.

Full details are in [docs/03_microros_interface.md](docs/03_microros_interface.md).

## Quick start (Wi-Fi firmware)

1. Install the Arduino IDE, the **esp32** board package, and the libraries **ESP32Encoder** and **micro_ros_arduino** (the release that matches your ROS 2 distro). Details: [docs/04_bringup_procedure.md](docs/04_bringup_procedure.md#1-software-setup).
2. In `firmware/amr_esp32_wifi/`, copy `secrets.example.h` to `secrets.h` and fill in the Wi-Fi name, password and the Pi's IP address. `secrets.h` is ignored by git.
3. Open `amr_esp32_wifi.ino`, select board **ESP32 Dev Module**, and upload.
4. On the Pi, start the agent:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```
5. Check that data is flowing:
   ```bash
   ros2 topic hz /encoder_telemetry        # about 20 Hz
   ros2 topic echo /encoder_telemetry
   ```
6. With the wheels off the ground, send a test command:
   ```bash
   ros2 topic pub -r 20 /left_vel  std_msgs/msg/Float64 "{data: 5.0}" &
   ros2 topic pub -r 20 /right_vel std_msgs/msg/Float64 "{data: 5.0}"
   ```
   Both wheels should turn at 5 rad/s and stop within half a second of pressing Ctrl+C.

For the wired version, see [docs/03_microros_interface.md](docs/03_microros_interface.md#7-wired-usb-serial-transport).

## Safety behaviour

- Motors are held at zero from power-on until the micro-ROS agent is connected.
- Motors stop if the agent connection is lost. The ESP32 keeps retrying and reconnects on its own.
- Motors stop if velocity commands stop arriving for 500 ms.
- After a reconnect, the wheels wait for new commands. Old commands are never resumed.
- Invalid numbers from the network (NaN, infinity, absurd values) are ignored.

## Debug and tuning commands

With the Wi-Fi firmware, open the Arduino Serial Monitor at 115200 baud with line ending "Newline". A status line is printed every second.

| Command | Effect |
|---|---|
| `ros` | Give control back to ROS (the default at boot) |
| `s` | Stop and hold. ROS commands are ignored until `ros` |
| `t 10` or `t 10 5` | Manual target in rad/s for both wheels, or left and right |
| `sq 10 2000` | Square-wave test: 10 rad/s and 0, period 2000 ms |
| `pwm 300 300` | Open-loop PWM (−1023 to 1023), controller bypassed |
| `kp L 12` | Set a parameter for `L`, `R` or `B` (both). Parameters: `kp ki kff min max` |
| `p` | Print all parameters |
| `plot 0`, `plot 1`, `plot 2` | Status text, speeds for the Serial Plotter, speeds plus PWM % |

Values changed with these commands are lost on reset. To keep them, edit the constants at the top of the firmware.

## Current parameters

| Parameter | Left | Right | Meaning |
|---|---|---|---|
| `CPR` | 752.6 | 536.1 | Encoder counts per wheel revolution (measured) |
| `KFF` | 49.72 | 43.54 | Feedforward: PWM per rad/s (measured) |
| `PWM_MIN` | 11.8 | 12.0 | Deadband compensation (measured) |
| `KP` | 30.0 | 25.0 | Proportional gain (tuned) |
| `KI` | 12.5 | 10.0 | Integral gain (tuned) |
| `MAX_SPEED` | 17.0 | 17.0 | Speed limit in rad/s |
| `ENC_INVERT` / `MOTOR_INVERT` | false / false | true / true | Sign conventions so that positive = robot forward |

`KFF` and `PWM_MIN` depend on the motor supply voltage. Re-run the characterisation tool if the battery or supply changes.

## Status

- **Wi-Fi firmware:** this is the code in use on the robot.
- **Serial firmware:** derived from the Wi-Fi firmware. Its control code is identical, and it has been compile-checked and simulated against stand-in versions of the ESP32 and micro-ROS libraries. **It has not yet been run on the real hardware.** Test it with the wheels off the ground first.
