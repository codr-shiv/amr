# Bench tools

Stand-alone Arduino sketches for checking the hardware and finding the controller parameters. None of them uses Wi-Fi or micro-ROS. Run them in order. The full procedure is in [docs/04_bringup_procedure.md](../docs/04_bringup_procedure.md).

| Sketch | Purpose | Motor power | Output |
|---|---|---|---|
| [`01_encoder_raw_test`](01_encoder_raw_test/) | Watch the A/B states and the decoding logic while turning one shaft slowly by hand | Off | State, count and invalid transitions |
| [`02_two_motor_encoder_test`](02_two_motor_encoder_test/) | Measure counts per wheel revolution and check both encoders at speed, using the hardware counter | Either | Count, revolutions, rpm, average rpm |
| [`03_motor_characterize`](03_motor_characterize/) | Open-loop PWM sweep of both motors. Gives feedforward, deadband, top speed and a sign check | On, wheels off the ground | Table plus a fitted result per motor |
| [`04_wheel_pid_tuning`](04_wheel_pid_tuning/) | The per-wheel controller with live tuning commands, without ROS | On, wheels off the ground first | Plot data for the Serial Plotter |

All sketches use the same pin map as the firmware and need the **ESP32Encoder** library, except tool 01, which needs no library.

Serial Monitor: 115200 baud. For tool 04, set the line ending to "Newline".
