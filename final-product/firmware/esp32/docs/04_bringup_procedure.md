# Bring-up procedure

The full sequence from a bare motor to a robot driven from ROS. Each step checks one thing, so a fault shows up where it is easy to find. Do the steps in order when building a new robot, replacing a motor, or changing the wiring.

| Step | Tool or firmware | What it proves |
|---|---|---|
| 1 | Software setup | The toolchain and libraries work |
| 2 | Wiring | Power and signal connections are safe |
| 3 | `tools/01_encoder_raw_test` | The encoder gives valid quadrature signals |
| 4 | `tools/02_two_motor_encoder_test` | Counts per wheel revolution, and that no counts are lost at speed |
| 5 | `tools/03_motor_characterize` | Feedforward, deadband, top speed and signs for each motor |
| 6 | `tools/04_wheel_pid_tuning` | Controller gains |
| 7 | `firmware/amr_esp32_wifi` or `amr_esp32_serial` | The complete system with ROS |

## 1. Software setup

1. Install the **Arduino IDE** (2.x).
2. Install the ESP32 board package: Boards Manager → search "esp32" → install **esp32 by Espressif Systems**.
3. Install the encoder library: Library Manager → search "ESP32Encoder" → install **ESP32Encoder** by Kevin Harrington (madhephaestus).
4. Install micro-ROS (needed only for step 7):
   - Download the release of **micro_ros_arduino** that matches the Pi's ROS 2 distro from <https://github.com/micro-ROS/micro_ros_arduino/releases>.
   - Arduino IDE: Sketch → Include Library → Add .ZIP Library.
   - This library is precompiled. If the build fails at the linking stage, install the ESP32 board package version named in that release's README.
5. Select the board: Tools → Board → **ESP32 Dev Module**, and the correct port.

On Linux, give your user access to serial ports once:

```bash
sudo usermod -aG dialout $USER     # then log out and back in
```

## 2. Wiring

Safety rules for the whole procedure:

1. The motor supply must never touch any ESP32 pin.
2. The encoder is powered from 3.3 V, never from the motor supply.
3. The ESP32 and the motor driver share only ground and the four signal wires.

Identify the motor's six wires from the label on the encoder board: two thick wires for the motor, and four thin ones for encoder VCC, GND, A and B. Do not rely on colours.

| From | To |
|---|---|
| Left encoder A, B | GPIO 32, GPIO 33 |
| Right encoder A, B | GPIO 25, GPIO 26 |
| Both encoders VCC, GND | ESP32 3V3, GND |
| GPIO 18, GPIO 19 | Cytron PWM and DIR, left channel |
| GPIO 23, GPIO 22 | Cytron PWM and DIR, right channel |
| ESP32 GND | Cytron GND |
| Motor supply | Cytron power terminals |
| Motor wires | Cytron motor outputs |

Before connecting A and B to the ESP32, power the encoder from 3.3 V and check with a multimeter that A and B switch between about 0 V and 3.3 V as the shaft is turned slowly.

## 3. Raw encoder check

Sketch: `tools/01_encoder_raw_test`. Motor power off. Serial Monitor at 115200.

Turn the output shaft **very slowly** by hand (one turn in 5 to 10 seconds).

Expected:

```
=== Encoder raw test started ===
AB=10  count=1  invalid=0
AB=11  count=2  invalid=0
AB=01  count=3  invalid=0
AB=00  count=4  invalid=0
```

- AB changes one bit at a time, in the order 00 → 10 → 11 → 01 or the reverse.
- `count` rises in one direction and falls in the other.
- `invalid` stays at 0. It rises if you turn too fast, which is normal for this sketch.

Repeat for the other encoder by changing `PIN_A` and `PIN_B` to 25 and 26.

## 4. Counts per revolution and speed

Sketch: `tools/02_two_motor_encoder_test`. It uses the hardware counter, so it works at any speed.

### 4.1 Measure counts per wheel revolution

1. Mark the output shaft and a reference point on the motor body.
2. Press EN on the ESP32 to reset the count.
3. Turn the shaft exactly 10 turns by hand and stop on the mark.
4. `count ÷ 10` is the counts per wheel revolution for that motor.
5. Turn back 10 turns. The count must return to about 0.

Put the results into `CPR1` and `CPR2` in the sketch, and later into `L_CPR` and `R_CPR` in the other sketches. Current values: 752.6 (left) and 536.1 (right).

### 4.2 Check under power

Run each motor from its supply and watch `rpm`, `avg` and `revs`.

- The speed reading should be steady.
- Reversing the motor should give the same speed with a negative sign.
- When the motor stops, the count must stop changing.

**Return-to-mark test** (checks for lost counts at speed): line up the mark, reset, run the motor for 30 to 60 seconds, stop, then turn the shaft back by hand until the count reads 0. The mark should be back at the reference. If it is not, counts are being lost: see [troubleshooting](05_troubleshooting.md#encoders).

Output columns:

| Column | Meaning |
|---|---|
| `count` | Encoder steps since reset |
| `revs` | Wheel turns since reset (`count ÷ CPR`) |
| `rpm` | Speed over the last 200 ms |
| `avg` | Average speed since reset |

## 5. Motor characterisation

Sketch: `tools/03_motor_characterize`. **Lift the robot so both wheels turn freely.**

1. Upload, open the Serial Monitor, switch on motor power, and press Enter.
2. The sketch sweeps the PWM from 0 to 1023 forward, then in reverse. It takes about three minutes.
3. Read the results:

```
LEFT  forward: deadband~ ..  kff= ..  pwmMin= ..  maxSpeed= .. rad/s (.. rpm)
LEFT  reverse: ...
RIGHT forward: ...
RIGHT reverse: ...
```

4. If a line ends with `SIGN MISMATCH`, set `ENC_INVERT = true` for that motor in the later sketches.
5. For each motor, average the forward and reverse `kff` and `pwmMin`.
6. Choose `MAX_SPEED` as about 85 % of the **smaller** of the two maximum speeds. Use the same value for both wheels.

Repeat this step if the battery or supply voltage changes, because `kff` depends on it.

## 6. Controller tuning

Sketch: `tools/04_wheel_pid_tuning`. Wheels off the ground. Serial Monitor line ending: **Newline**.

Enter the values from step 5 at the top of the sketch.

### 6.1 Direction check (open loop)

```
pwm 300 300
```

Both wheels turn and both measured speeds are positive. Then `s` to stop. If a measured speed is negative, fix `ENC_INVERT` for that wheel before using any feedback gain.

To make "positive" mean "robot forward" on a wheel, set both `MOTOR_INVERT` and `ENC_INVERT` for that wheel to the opposite of their current values.

### 6.2 Feedforward only

Set `kp B 0` and `ki B 0`, then try `t 5`, `t 10`, `t 15`. Each wheel should be within about 10 to 20 % of the target. Correct with `KFF_new = KFF_old × target ÷ measured`.

### 6.3 Gains

Open Tools → Serial Plotter and type commands in its input box.

```
sq 10 2000        square wave: 10 rad/s and 0, one second each
kp L 20           raise until fast, back off if it oscillates
ki L 5            raise until the steady error disappears
plot 2            also show PWM %
```

Repeat for `R`. The full method and a table for reading the plot are in [02_pid_control.md](02_pid_control.md#7-how-the-parameters-were-obtained).

### 6.4 Both wheels

```
t 10            both at 10 rad/s
t 8 -8          spin in place
t 40 20         above the limit: scaled together, ratio kept
s               stop
```

Finish with a short drive on the floor at `t 8`. The robot should travel close to straight.

Type `p` and copy the final values into the constants in the firmware.

## 7. Firmware with micro-ROS

Copy the tuned constants (`CPR`, `KFF`, `PWM_MIN`, `KP`, `KI`, `MAX_SPEED` and the invert flags) into the firmware. The two firmware variants must contain the same values.

### 7.1 Wi-Fi

1. `cd esp32/firmware/amr_esp32_wifi && cp secrets.example.h secrets.h`, then edit `secrets.h`.
2. Upload `amr_esp32_wifi.ino`. The Serial Monitor shows:
   ```
   # Wi-Fi connected, ESP32 IP = ...
   # waiting for micro-ROS agent at <ip>:8888 ...
   [NO AGENT|ROS] tgt L=  0.00 R=  0.00 | meas ...
   ```
3. On the Pi:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```
4. The Serial Monitor prints `# micro-ROS: CONNECTED to agent` and the status line changes to `[ROS OK |ROS]`.

### 7.2 Serial (wired)

1. Upload `amr_esp32_serial.ino` from your computer. The LED on the board blinks.
2. Plug the ESP32 into the Pi and start the agent:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
   ```
3. The LED becomes solid when connected.

Do not open a serial monitor on that port while the agent runs.

### 7.3 Verify from the Pi

```bash
ros2 topic hz /encoder_telemetry
ros2 topic echo /encoder_telemetry
```

Turn a wheel by hand: its `position` changes, by 6.283 for one full turn, positive in the forward direction.

Then, with the wheels off the ground:

```bash
ros2 topic pub -r 20 /left_vel  std_msgs/msg/Float64 "{data: 5.0}" &
ros2 topic pub -r 20 /right_vel std_msgs/msg/Float64 "{data: 5.0}"
```

### 7.4 Acceptance checklist

- [ ] Both wheels reach the commanded speed, and telemetry `velocity` agrees.
- [ ] Positive commands drive both wheels in the robot's forward direction.
- [ ] Stopping the publishers stops the wheels within about 0.5 s.
- [ ] Stopping the agent stops the wheels. Restarting it reconnects without touching the ESP32.
- [ ] Powering the ESP32 before the agent is running works: it waits, then connects.
- [ ] A command above 17 rad/s is limited, and both wheels are scaled together.
- [ ] On the floor, equal commands drive the robot close to straight.

## Unit conversions

| From | To | Multiply by |
|---|---|---|
| rad/s | rpm | 9.549 |
| rpm | rad/s | 0.10472 |
| rad | degrees | 57.296 |
| rad | wheel turns | 0.15915 |
| rad | metres rolled | wheel radius in metres |
| m/s at the wheel | rad/s | 1 ÷ wheel radius in metres |
