# micro-ROS interface and firmware architecture

This document covers how the ESP32 talks to the Raspberry Pi, the exact message formats, the Wi-Fi and wired transports, and how the motor-control code and the micro-ROS code were combined into one firmware.

## 1. What micro-ROS is

ROS 2 nodes normally communicate through DDS, which needs far more memory than a microcontroller has. micro-ROS solves this with a client and an agent:

```
ESP32                                        Raspberry Pi
┌───────────────────────┐                   ┌───────────────────────┐
│ firmware              │   Wi-Fi (UDP)     │ micro-ROS agent       │
│  rclc (ROS 2 C API)   │ ◄───────────────► │  acts as the ESP32's  │ ◄── DDS ──► other ROS 2 nodes
│  small XRCE-DDS client│   or USB serial   │  proxy in the ROS graph│
└───────────────────────┘                   └───────────────────────┘
```

- The ESP32 runs a small client.
- The **agent** on the Pi does the DDS work on the ESP32's behalf. To every other ROS 2 node, the ESP32's topics look like ordinary topics.
- The micro-ROS library on the ESP32 must be built for the **same ROS 2 distro** as the agent.

## 2. Topics

| Topic | Direction | Type | QoS | Rate |
|---|---|---|---|---|
| `left_vel` | Pi → ESP32 | `std_msgs/msg/Float64` | Reliable (default) | Set by the Pi |
| `right_vel` | Pi → ESP32 | `std_msgs/msg/Float64` | Reliable (default) | Set by the Pi |
| `encoder_telemetry` | ESP32 → Pi | `sensor_msgs/msg/JointState` | Best effort | 20 Hz |

Node name: `esp32_wifi_wheel_node` (Wi-Fi firmware) or `esp32_serial_wheel_node` (serial firmware).

A subscriber to `encoder_telemetry` must also use **best-effort** QoS. A reliable subscriber will not receive data from a best-effort publisher. `ros2 topic echo` adapts automatically.

### Commands: `left_vel`, `right_vel`

| Field | Meaning | Unit |
|---|---|---|
| `data` | Target angular speed of that wheel | rad/s |

- Positive means robot forward for both wheels.
- To convert a linear wheel speed in m/s to rad/s, divide by the wheel radius in metres.
- 1 rad/s = 9.549 rpm.
- Values above 17 rad/s are scaled down, both wheels by the same factor.
- NaN, infinity and values beyond ±10⁶ are ignored.

### Telemetry: `encoder_telemetry`

| Field | Content |
|---|---|
| `header.stamp` | Time at which the encoders were read (see [timestamps](#5-timestamps)) |
| `header.frame_id` | Empty |
| `name` | `["left_wheel", "right_wheel"]` |
| `position` | `[left, right]` cumulative wheel angle in **rad** |
| `velocity` | `[left, right]` filtered wheel speed in **rad/s** |
| `effort` | Empty |

About `position`:

- It is the angle of the wheel (the gearbox output), not of the motor shaft.
- It is cumulative and signed. It is not wrapped to 0–2π. One wheel turn adds 6.283 rad.
- It starts at 0 when the ESP32 boots. If the ESP32 resets, it jumps back to 0. Odometry code on the Pi should use the change between consecutive messages and handle that jump.
- Distance rolled in metres = angle in rad × wheel radius in metres.

Both wheels are already converted with their own counts-per-revolution on the ESP32. The Pi never needs to know that the two motors differ.

## 3. What the Pi side must do

1. **Publish both velocity topics continuously**, at 5 Hz or more, even when the values do not change. If either topic is silent for `CMD_TIMEOUT_MS` (500 ms), both wheels stop.
2. Send speeds in **rad/s**, positive for forward.
3. Subscribe to `encoder_telemetry` with best-effort QoS.
4. Run the agent for the matching transport (sections 6 and 7).

## 4. Safety and connection handling

The firmware uses a four-state connection machine in `microRosStep()`:

```
WAITING_AGENT ──ping answered──► AGENT_AVAILABLE ──entities created──► AGENT_CONNECTED
      ▲                                │ creation failed                     │ ping fails
      └────────────────────────────────┴──────────── AGENT_DISCONNECTED ◄────┘
```

| State | What the ESP32 does | Motors |
|---|---|---|
| `WAITING_AGENT` | Pings the agent every 500 ms | Stopped |
| `AGENT_AVAILABLE` | Creates the node, subscribers, publisher and executor, then syncs the clock | Stopped |
| `AGENT_CONNECTED` | Processes commands, publishes telemetry, pings the agent every second | Follow ROS commands |
| `AGENT_DISCONNECTED` | Destroys all entities and goes back to waiting | Stopped |

Consequences:

- The ESP32 never hangs if the Pi or agent is off at boot. It waits and connects when the agent appears.
- If the agent or the link is lost, the motors stop and the ESP32 reconnects on its own.
- On disconnect the stored ROS targets are cleared, so old commands are never resumed.
- Independently of the connection state, a missing command for 500 ms stops the wheels.

## 5. Timestamps

`header.stamp` is the time at which the encoder counts were read in the control task, not the time the message was sent.

How it is produced:

1. The control task records the ESP32's own 64-bit microsecond timer (`esp_timer_get_time()`) at each encoder reading.
2. After connecting, and then every 60 s, the firmware synchronises with the agent (`rmw_uros_sync_session`) and computes the offset between the agent's clock and the ESP32 timer. Two consecutive measurements must agree within 20 ms before the offset is accepted.
3. Each message is stamped with `sample time + offset`. Stamps are kept strictly increasing across small re-sync corrections.
4. Before the first successful sync, the stamp is the time since the ESP32 booted.

**Known limitation.** On this board the library's synchronised clock (`rmw_uros_epoch_nanos()`) was observed to advance only in whole seconds. The intervals between stamps are exact, because they come from the microsecond timer. The **absolute** offset to the Pi's clock, however, can be wrong by up to about one second. That is fine for odometry, which uses the difference between consecutive stamps. If another component needs these stamps aligned with other sensors (for example LiDAR scans in SLAM), check the offset on the Pi first. One alternative is to stamp the data on the Pi when it is received.

## 6. Wi-Fi (UDP) transport

Firmware: [`firmware/amr_esp32_wifi`](../firmware/amr_esp32_wifi/). This is the configuration in use on the robot. The ESP32 and the Pi are on the same Wi-Fi network.

### Configuration

Wi-Fi name, password, the Pi's IP address and the agent port are in `secrets.h`, next to the sketch. This file is not committed to git.

```bash
cd esp32/firmware/amr_esp32_wifi
cp secrets.example.h secrets.h      # then edit secrets.h
```

The Pi needs a **static IP address**, because the ESP32 connects to a fixed address.

### Running the agent on the Pi

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

or with Docker (replace `<distro>` with your ROS 2 distro, for example `humble` or `jazzy`):

```bash
docker run -it --rm --net=host microros/micro-ros-agent:<distro> udp4 --port 8888
```

### Notes

- `WiFi.setSleep(false)` disables Wi-Fi modem sleep. With it enabled, UDP packets can be delayed by up to about 100 ms.
- The USB serial port is free, so the Serial Monitor shows status text and accepts the tuning commands while micro-ROS runs.

## 7. Wired (USB serial) transport

Firmware: [`firmware/amr_esp32_serial`](../firmware/amr_esp32_serial/). The ESP32 is plugged into the Pi with a USB cable, which also powers it. Topics, message formats, control code and safety behaviour are identical to the Wi-Fi firmware.

> This variant has been compile-checked and simulated, but **not yet run on the real hardware**. Test it with the wheels off the ground first.

### Differences from the Wi-Fi firmware

| | Wi-Fi | Serial |
|---|---|---|
| Transport setup | `set_microros_wifi_transports(...)` | `set_microros_transports()` (USB `Serial`, 115200 baud) |
| Configuration | `secrets.h` | None |
| Debug text and tuning commands | On USB `Serial` | Off by default. Optional on `Serial2` |
| Status indication | Serial Monitor | LED on GPIO 2: blinking = waiting, solid = connected |
| Node name | `esp32_wifi_wheel_node` | `esp32_serial_wheel_node` |

### The most important rule

With this firmware, the USB serial port carries the micro-ROS binary protocol. **Nothing else may use it.**

- Do not open the Arduino Serial Monitor while the agent is running. Only one program can use the port.
- Do not add `Serial.print()` to this firmware. Any extra byte corrupts the stream.

For this reason all debug output goes through `DBG_PRINTF` / `DBG_PRINTLN`, which do nothing unless `DEBUG_ENABLED` is 1.

### Optional debug port

To get the status text and tuning commands back:

1. Set `#define DEBUG_ENABLED 1`.
2. Connect a **3.3 V** USB-TTL adapter: adapter TX to GPIO 16 (RX2), adapter RX to GPIO 17 (TX2), GND to GND.
3. Open that adapter's port at 115200 baud.

GPIO 16 and 17 are not available on ESP32-WROVER modules. Change `DBG_RX_PIN` and `DBG_TX_PIN` if needed.

Without an adapter, use the status LED and inspect the topics on the Pi.

### Running the agent on the Pi

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

or with Docker:

```bash
docker run -it --rm -v /dev:/dev --privileged --net=host \
  microros/micro-ros-agent:<distro> serial --dev /dev/ttyUSB0 -b 115200
```

Notes:

- Find the port with `ls /dev/ttyUSB* /dev/ttyACM*` after plugging in the ESP32.
- The user needs permission for the port: `sudo usermod -aG dialout $USER`, then log out and back in.
- Opening the port usually resets the ESP32. It then reconnects by itself within a second or two.
- To upload new firmware, stop the agent first, because it holds the port.
- For a port name that does not change between boots, use the path under `/dev/serial/by-id/`.

### Choosing between the two

| | Wi-Fi | Serial |
|---|---|---|
| Wiring | None | One USB cable |
| Latency and jitter | Variable, depends on the network | Low and consistent |
| Reliability | Depends on signal and router | Very high |
| Debug over USB at the same time | Yes | No (needs a second adapter) |
| Best when | The ESP32 and Pi cannot be cabled together, or during development | The Pi is mounted on the robot next to the ESP32 |

## 8. How the control code and the micro-ROS code were combined

The firmware started as two separate working programs:

- a **control program**: encoders, per-wheel feedforward + PI, and serial tuning commands, all in `loop()`
- a **micro-ROS program**: Wi-Fi, two subscribers and one publisher, with placeholder variables for the encoder data

Putting both into one `loop()` would not have been enough. These were the problems and how each was solved.

| Problem | Why it matters | Solution in the combined firmware |
|---|---|---|
| The control loop needs an exact 20 ms period, but Wi-Fi and micro-ROS calls can block for milliseconds or longer | Irregular timing makes the speed measurement and the integral wrong | The control code runs in its own FreeRTOS task, `controlTask`, pinned to core 1 at priority 3 with `vTaskDelayUntil`. `loop()` only does communication |
| Two tasks read and write the same variables | A task could read half-updated values | All shared data is in one struct, `sh`, accessed only inside a short critical section (`LOCK()` / `UNLOCK()`) |
| The original micro-ROS program stopped in an endless loop on any error (`RCCHECK`) | If the agent was not running at boot, the ESP32 hung | Replaced by the reconnecting state machine in section 4 |
| No command timeout | If the Pi stopped sending, the wheels kept their last speed | `CMD_TIMEOUT_MS`: wheels stop when either topic is silent for 500 ms |
| Old commands after a reconnect | The robot could move unexpectedly | ROS targets are cleared on disconnect, and stale timestamps fail the timeout check |
| Serial commands and ROS both set the wheel targets | The two sources would fight | Explicit modes: `MODE_ROS` (default), `MODE_SERIAL_VEL`, `MODE_SERIAL_PWM`, `MODE_SQUARE`. A serial motion command takes over. `ros` gives control back |
| `left_vel` and `right_vel` arrive in separate callbacks | The speed limit must look at both wheels together | Callbacks only store the value and arrival time. The control task reads both and applies `setTargets()` |
| Telemetry memory allocated with `malloc` | Unnecessary on a microcontroller | Static arrays for the names, positions and velocities |
| Telemetry had no timestamp | Odometry needs the time of each sample | Stamp taken at the encoder reading (section 5) |
| Wi-Fi power saving | Adds delay to UDP packets | `WiFi.setSleep(false)` |

### Structure

```
                    ┌──────────────── loop()  (communication) ────────────────┐
 left_vel  ───────► │ micro-ROS callbacks ──┐                                  │
 right_vel ───────► │                       ▼                                  │
                    │                 ┌──────────┐     publishTelemetry() ───► │ ──► encoder_telemetry
 Serial commands ─► │ runCommand() ─► │  sh      │ ◄── reads pos / vel         │
                    └─────────────────│ (locked) │─────────────────────────────┘
                    ┌─────────────────│          │──── controlTask (50 Hz) ────┐
                    │ reads mode,     └──────────┘ writes pos, vel, pwm        │
                    │ targets, params      ▲                                   │
                    │      │               │                                   │
                    │      ▼               │                                   │
                    │ setTargets() ─► updateWheel(L), updateWheel(R) ─► PWM + DIR
                    │                      ▲                                   │
                    │                      └── encoder counts (PCNT hardware)  │
                    └───────────────────────────────────────────────────────────┘
```

One control cycle (`controlStep()`):

1. Copy the commands and parameters out of `sh` under the lock.
2. Decide the targets from the current mode. In `MODE_ROS`, use the ROS targets only if the agent is connected and both commands are fresh. Otherwise use zero.
3. Run `updateWheel()` for each wheel: measure, ramp, feedforward + PI, output.
4. Write the new angle, speed, PWM and the sample time back into `sh` under the lock.

`loop()` then, about every millisecond:

1. Runs the micro-ROS state machine: receives commands, publishes telemetry every 50 ms, checks the connection, re-syncs the clock.
2. Handles serial commands.
3. Prints the status line or plot data.

Motor outputs and encoder reads happen only in `controlTask`. `loop()` never touches the motors directly.

## 9. Testing the interface from the Pi

```bash
# is the ESP32 visible?
ros2 node list
ros2 topic list

# telemetry
ros2 topic hz /encoder_telemetry             # about 20 Hz
ros2 topic echo /encoder_telemetry

# drive both wheels at 5 rad/s (wheels off the ground)
ros2 topic pub -r 20 /left_vel  std_msgs/msg/Float64 "{data: 5.0}" &
ros2 topic pub -r 20 /right_vel std_msgs/msg/Float64 "{data: 5.0}"
```

Checks:

- `velocity` in the telemetry settles at about 5.0 for both wheels.
- `position` increases steadily for both wheels.
- Pressing Ctrl+C stops the wheels within about half a second (the command timeout).
- Stopping the agent stops the wheels. Restarting it reconnects without resetting the ESP32.
- Publishing only once (without `-r`) makes the wheels move briefly and then stop. That is the timeout working, not a fault.
