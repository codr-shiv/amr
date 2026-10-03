# Encoder decoding (wheel position and velocity)

How the quadrature encoder signals become wheel angle (rad) and speed (rad/s) on the ESP32, and how they reach
the Pi as `/encoder_telemetry`.

Code: `initWheel()`, `updateWheel()` and `publishTelemetry()` in
[`firmware/esp32/amr_esp32/amr_esp32.ino`](../../firmware/esp32/amr_esp32/amr_esp32.ino).

---

## 1. Quadrature signals

Each motor has an encoder with two square-wave outputs, A and B, 90° out of phase. Every A or B edge is one "count";
which signal leads gives the direction.

| Decoding | Edges counted | Counts per encoder cycle |
|---|---|---|
| 1× | rising edges of A | 1 |
| 2× | both edges of A | 2 |
| **4× (used)** | both edges of A and B | 4 |

## 2. Hardware counting: `ESP32Encoder`

```cpp
ESP32Encoder::useInternalWeakPullResistors = puType::up;   // encoder outputs are pulled up internally
enc->attachFullQuad(encA, encB);                            // 4× decoding in the PCNT hardware
enc->setFilter(1023);                                       // glitch filter
enc->clearCount();                                          // start at 0
```

- The ESP32's **PCNT** (pulse counter) peripheral counts every edge in hardware, so no count is lost even if software is
  busy. The library extends the 16-bit hardware counter to a 64-bit count (`getCount()` returns `int64_t`) using overflow
  interrupts, so the position never wraps in practice.
- `setFilter(1023)`: pulses shorter than 1023 APB clock cycles (80 MHz → ~12.8 µs) are ignored as noise.
  At the 17 rad/s limit the left wheel produces ~2040 counts/s (~490 µs between counts), so real edges are far longer
  than the filter.
- Pins: left A/B = GPIO 32/33, right A/B = GPIO 25/26.

## 3. Counts per revolution (CPR)

| Wheel | Motor / gearbox | `CPR` (counts per **wheel** revolution, 4×) | Implied counts per motor-shaft revolution |
|---|---|---|---|
| Left | 26.9:1 (a different motor from the right one) | 752.6 | 752.6 / 26.9 ≈ 28 |
| Right | PG36M555-19.2K, 19.2:1 (firmware comment: 19.1:1) | 536.1 | 536.1 / 19.1 ≈ 28 |

The encoder is the **ME-37, 7 PPR** (7 pulses per channel per motor-shaft revolution, from the motor's product page),
so 4× decoding gives 7 × 4 = **28 counts per motor revolution**, multiplied by the gear ratio for a wheel revolution:
28 × 19.2 = 537.6 (nominal) vs 536.1 used; 28 × 26.9 = 753.2 vs 752.6 used. The used values were calibrated per motor
(the project log notes that the counts differed between motors). To re-measure, turn the wheel exactly 10 revolutions by
hand and divide the count change by 10.

## 4. From counts to position and velocity (every 20 ms)

```cpp
int64_t c = w.enc->getCount();
if (w.encInvert) c = -c;                                       // forward must be positive
float raw = TWO_PI * (float)(c - w.lastCount) / (w.cpr * dt);  // rad/s over this 20 ms window
w.lastCount = c;
w.meas = VEL_ALPHA * raw + (1.0f - VEL_ALPHA) * w.meas;        // low-pass, VEL_ALPHA = 0.3
w.pos  = TWO_PI * (double)c / (double)w.cpr;                   // cumulative wheel angle, rad (double)
```

- **Position** is the absolute count since boot converted to radians, kept in `double` (a `float` would lose precision
  after many revolutions). It is never reset while running; the Pi only uses differences between samples.
- **Velocity** is a finite difference over one control period. Its resolution is one count per 20 ms:
  left `2π/(752.6·0.02)` ≈ **0.42 rad/s**, right `2π/(536.1·0.02)` ≈ **0.59 rad/s**. At low speed the raw value jumps
  between multiples of this step, which is why it's filtered.
- **Filter:** first-order IIR (exponential moving average) with α = 0.3 at 50 Hz. Time constant
  τ = −dt / ln(1−α) ≈ 0.020 / 0.357 ≈ **56 ms**. Smooth enough for the PI loop, fast enough to follow the 40 rad/s² ramp.
- **Direction flags:** on the left wheel both the encoder and the motor are inverted (`L_ENC_INVERT = L_MOTOR_INVERT = true`), as is typical when the two motors are mounted facing opposite directions.

`meas` feeds the PI controller; `pos` and `meas` are copied to the shared struct for telemetry.

## 5. Telemetry to the Pi

`publishTelemetry()` runs every 50 ms (20 Hz) in `loop()` while connected:

| Field | Value |
|---|---|
| `header.stamp` | Pi-synchronized epoch time (`rmw_uros_epoch_nanos()`), or `millis()` if sync failed |
| `header.frame_id` | empty |
| `name` | `["left_wheel", "right_wheel"]` |
| `position` | `[posL, posR]` rad, cumulative since boot |
| `velocity` | `[velL, velR]` rad/s, filtered |
| `effort` | empty |

Published best effort on `/encoder_telemetry`. The stamp is taken when publishing, up to 20 ms after the control task
measured the values.

## 6. How the Pi uses it

`amr_diff_drive` (see [05-diff-drive-controller.md](05-diff-drive-controller.md)):
- looks up the wheels **by name**, not array index;
- integrates **position differences** into the pose (robust to lost or late packets, because the next difference contains
  the missing motion);
- uses the **velocities** only for the odometry twist and for predicting the pose forward to "now";
- treats a position jump > 10 rad between messages as an encoder reset (e.g. ESP32 reboot → counts back to 0): the pose is
  not moved and the reference is re-latched.

## 7. Checks

```bash
ros2 topic hz /encoder_telemetry                  # ~20 Hz
ros2 topic echo /encoder_telemetry --once         # names, positions, velocities, stamp
ros2 topic echo /encoder_telemetry --field header.stamp   # nanosec must change every message
```
On the ESP32 serial monitor, turn a wheel by hand: `pos` changes by 2π (6.28) per revolution and is positive when the
wheel turns forward.
