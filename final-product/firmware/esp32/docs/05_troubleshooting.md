# Troubleshooting

Find the symptom, check the likely causes in order, and apply the fix. Work from the bottom of the system upwards: serial link, then encoders, then motors, then the controller, then micro-ROS.

## Upload and Serial Monitor

| Symptom | Likely cause | Fix |
|---|---|---|
| Garbage characters (`����`) in the Serial Monitor | Monitor lost sync after an upload | Keep it at 115200 and press EN on the ESP32 |
| | Another program is using the port | Close other serial monitors and stop any micro-ROS **serial** agent |
| | Ubuntu's ModemManager is probing the port | `sudo systemctl stop ModemManager` (and `disable` to make it permanent) |
| | Wrong board or CPU frequency | Tools → Board: ESP32 Dev Module, CPU Frequency: 240 MHz |
| | The serial firmware is running | Expected. That port carries micro-ROS binary data, not text |
| Old output mixed with new | Monitor was not cleared | Clear the output before a test |
| Upload fails: port busy | The agent or a monitor holds the port | Stop it, then upload |
| Upload fails: permission denied | User is not in the `dialout` group | `sudo usermod -aG dialout $USER`, then log out and in |
| The startup banner repeats without pressing reset | The ESP32 is rebooting in a loop | Power or wiring fault. Unplug the encoder and driver wires, then reconnect one at a time |
| Commands typed in the monitor do nothing | Line ending is not "Newline" | Set the line ending dropdown to Newline |

## Encoders

| Symptom | Likely cause | Fix |
|---|---|---|
| Count never changes | Encoder not powered, or wrong wires | Measure 3.3 V between encoder VCC and GND. Recheck A and B |
| | Open-collector outputs without pull-ups | Enable internal pull-ups or add 4.7 kΩ to 10 kΩ to 3.3 V |
| Count bounces between two values whichever way the shaft turns | One channel is dead or disconnected | Measure each channel with a multimeter while turning slowly |
| About half the expected counts per turn | One channel missing, or not using 4× decoding | Check both channels. Use `attachFullQuad` |
| Count goes the wrong way | Sign convention only | Swap A and B, or set `ENC_INVERT` |
| Count changes with the shaft still | Floating input or noise | Pull-ups, shorter wires, check ground |
| Correct by hand, wrong when the motor runs | Motor electrical noise | Separate encoder and motor wiring, twist A and B with ground, 100 nF across the motor terminals, external pull-ups |
| | Loose jumper wire under vibration | Reseat or solder the connections |
| | Signal does not reach a clean 3.3 V | Measure the high level at the ESP32 pin |
| `invalid` rises in `tools/01_encoder_raw_test` | Turning too fast for a polling sketch | Normal. Turn slower, or use tool 02 |
| | Rises even when turning very slowly | Noise or a bad connection |
| Speed reading differs from a hand count | Hand counting is imprecise at speed | Compare `revs` over the same 60 s, or use the return-to-mark test in the bring-up procedure |
| Counts per turn is not a whole number | Normal | Planetary gear ratios are not whole numbers. Use the measured value |

## Motors and driver

| Symptom | Likely cause | Fix |
|---|---|---|
| A wheel does not turn | Motor supply off, or no common ground | Check the supply and the ESP32-to-driver ground wire |
| | PWM or DIR wire on the wrong driver input | Check against the pin map |
| | `KFF` or `MAX_SPEED` is 0 | Type `p` and check the parameters |
| A wheel turns the wrong way | Sign convention | Flip both `MOTOR_INVERT` and `ENC_INVERT` for that wheel |
| Both wheels always run at the same PWM | Both pins on one LEDC channel | `L_CH` and `R_CH` must be different (core 2.x) |
| Does not start at low speed, then jumps | Deadband value wrong | Adjust `PWM_MIN` for that wheel |
| ESP32 resets when the motors start | Supply noise or ground problem | Power the ESP32 separately, use one short ground connection, add decoupling capacitors |
| Compile error on `ledcSetup` or `ledcAttach` | Different Arduino-ESP32 core version | The sketches here handle core 2.x and 3.x. Check that the code was not modified |

## Controller

| Symptom | Likely cause | Fix |
|---|---|---|
| Wheel runs to full speed as soon as `KP` > 0 | Encoder sign is opposite to the motor sign | Run `pwm 300 300`. The measured speed must be positive. Fix `ENC_INVERT` |
| Speed stays below the target | `KI` too low or zero | Raise `KI` |
| | PWM is at 100 % | The target is beyond the motor. Lower `MAX_SPEED` |
| Oscillation or buzzing | `KP` too high | Lower `KP` |
| Slow repeated swings around the target | `KI` too high | Lower `KI` |
| Noisy speed at constant target | Encoder quantization | Lower `VEL_ALPHA`. Keep the loop at 50 Hz |
| Robot curves at high speed only | One wheel is saturating | Lower `MAX_SPEED` so the slower motor has margin |
| Robot curves at all speeds | Wrong CPR for one wheel, or different wheel diameters | Re-measure counts per revolution |
| Behaviour changed after a battery change | `KFF` depends on supply voltage | Re-run `tools/03_motor_characterize` |
| Tuned values lost after reset | Serial changes are not saved | Type `p` and copy the values into the constants |

## micro-ROS

| Symptom | Likely cause | Fix |
|---|---|---|
| Stuck at "waiting for micro-ROS agent" | Agent not running | Start it on the Pi |
| | Wrong `AGENT_IP` or port in `secrets.h` | Check with `hostname -I` on the Pi. The Pi needs a static IP |
| | ESP32 and Pi on different networks | Compare the ESP32 IP printed at boot with the Pi's |
| | Firewall blocks UDP 8888 | Allow the port, for example `sudo ufw allow 8888/udp` |
| | Agent started in the wrong mode | `udp4` for the Wi-Fi firmware, `serial` for the serial firmware |
| Stuck at "Wi-Fi connecting...." | Wrong name or password, or a 5 GHz-only network | The ESP32 supports 2.4 GHz only. Check `secrets.h` |
| Agent sees the client but no topics appear | Library and agent built for different ROS 2 distros | Install the micro_ros_arduino release for the Pi's distro |
| Connects, then drops repeatedly | Weak Wi-Fi signal | Move closer, reduce interference, or use the serial firmware |
| `ros2 topic echo` works but my node receives nothing | QoS mismatch | Subscribe with best-effort QoS |
| Wheels do not move although commands are published | Published once, not continuously | Publish at 5 Hz or more. The 500 ms timeout stops the wheels |
| | Only one of the two topics is published | Both `left_vel` and `right_vel` are required |
| | Firmware is in manual mode after a serial command | Type `ros` |
| | Agent not connected | Status line shows `NO AGENT` |
| Wheels stop every half second | Command rate too low or irregular | Publish faster, or raise `CMD_TIMEOUT_MS` |
| Speed lower than commanded | Command above `MAX_SPEED` | Expected: both wheels are scaled together |
| `position` jumped to 0 | The ESP32 was reset | Expected. Handle it in the odometry code |
| Timestamps offset from the Pi clock | Coarse library clock | See [timestamps](03_microros_interface.md#5-timestamps) |
| Serial firmware: agent reports errors or disconnects | Something else writes to USB serial | Remove any `Serial.print`. Close serial monitors |
| Serial firmware: `/dev/ttyUSB0` not found | Different device name | `ls /dev/ttyUSB* /dev/ttyACM*`, or use `/dev/serial/by-id/` |
| Serial firmware: no debug output | Debug is off by default | Set `DEBUG_ENABLED 1` and use a USB-TTL adapter on GPIO 16 / 17 |
| Compile error: `secrets.h: No such file` | The file has not been created | Copy `secrets.example.h` to `secrets.h` and fill it in |

## Reading the status line (Wi-Fi firmware)

```
[ROS OK |ROS] tgt L=  5.00 R=  5.00 | meas L=  5.02 R=  4.98 rad/s | pos L=  123.45 R=  122.90 rad | pwm L=  262 R=  230
```

| Field | Meaning |
|---|---|
| `ROS OK` / `NO AGENT` | Whether the micro-ROS agent is connected |
| `ROS` / `SERIAL` / `PWM` / `SQUARE` | Who controls the wheels: ROS, a serial target, open-loop PWM, or the square-wave test |
| `tgt` | Target after the speed limit, in rad/s |
| `meas` | Measured wheel speed, in rad/s |
| `pos` | Cumulative wheel angle, in rad |
| `pwm` | Signed output, −1023 to 1023 |

Quick reading:

- `NO AGENT`: a network or agent problem. The motors are held at zero.
- `ROS OK` with `tgt` at 0 while the Pi is publishing: the commands are not arriving fast enough, or only one topic is published.
- `tgt` correct but `meas` far off with `pwm` at ±1023: the motor cannot reach that speed.
- `tgt` correct, `meas` at 0 and `pwm` not 0: a motor power, driver or encoder problem.
