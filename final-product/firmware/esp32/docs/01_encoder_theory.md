# Encoder theory and decoding

This document explains how the wheel encoders work, how the ESP32 turns their pulses into wheel angle and speed, and where the numbers used in the firmware come from.

A note on terms: the **encoder** is the sensor on the motor. The **decoder** is the logic that interprets its two signals. Here the decoder is the ESP32's pulse-counter hardware plus a few lines of firmware. It is not the "n-to-2ⁿ decoder" from digital electronics.

## 1. What an encoder is

An encoder turns rotation into electrical pulses. Counting the pulses gives how far the shaft has turned. Counting pulses per unit of time gives how fast it is turning.

| Type | What it reports | Used here |
|---|---|---|
| Incremental | Pulses only. It knows movement since power-on, not the absolute angle | Yes |
| Absolute | The actual angle, even right after power-on | No |
| Optical | A slotted disc between an LED and a photodetector | No |
| Magnetic (Hall effect) | A multi-pole ring magnet on the motor shaft with two Hall sensors beside it | Yes |

The motors on this robot have an incremental Hall-effect encoder mounted on the **motor shaft**, before the gearbox.

## 2. Quadrature: two signals, 90° apart

A single signal (channel A) would show that the shaft is moving but not which way. A second sensor (channel B) is placed so that its signal is shifted by a quarter of a pulse period, which is 90° electrical. "Quadrature" means a quarter out of phase.

```
One pulse period = 360° electrical

A:  ___|‾‾‾‾‾‾‾‾|________|‾‾‾‾‾‾‾‾|________
B:  _______|‾‾‾‾‾‾‾‾|________|‾‾‾‾‾‾‾‾|____
       ①   ②    ③   ④
       A↑  B↑   A↓  B↓        four edges per period
```

One pulse period is 360° **electrical**. It is not a full mechanical turn. One mechanical turn of the motor shaft contains as many periods as the encoder's PPR (section 5).

### Why the sensors are a quarter period apart

- At the same position, both would switch at the same instant and the direction could not be found.
- Exactly one pole apart, B would always be the opposite of A, which again carries no direction information.
- At a quarter period, whenever one sensor is switching, the other is in the middle of a pole and gives a clean, stable level.

## 3. How direction is detected

### The physical idea

The magnet ring has alternating N and S poles. A Hall sensor outputs 1 in front of one pole type and 0 in front of the other, so its signal changes only when a **boundary between two poles** passes it.

Direction is found by asking which sensor a given boundary reaches first. It is not about which pole is first.

```
        rotation forward →
   ... N  S  N  S  N  S ...     magnet ring moving past
            [A] [B]              two fixed sensors
```

- Forward: every boundary reaches A first and B a quarter period later. **A leads B.**
- Backward: every boundary reaches B first. **B leads A.**

### The simplest rule

At the moment A rises, read B:

- B = 0: the boundary has not reached B yet, so the shaft is moving from A towards B. Forward.
- B = 1: B has already switched. Backward.

### The state sequence

Writing both signals as a 2-bit number AB, the shaft moves through four states in a fixed circular order:

```
          00
        ↗    ↘          forward :  00 → 10 → 11 → 01 → 00 …
      01      10         backward:  00 → 01 → 11 → 10 → 00 …
        ↖    ↙
          11
```

Only one bit changes at each step. This is a Gray code. From any state there are exactly two legal next states, one for each direction. A jump where both bits change (00 ↔ 11 or 01 ↔ 10) is physically impossible in one step, so it means a state was missed or there was electrical noise.

### The lookup table

A decoder remembers the previous state and reads the current one. The pair gives the step:

| Previous → current | Step |
|---|---|
| 00→10, 10→11, 11→01, 01→00 | +1 (forward) |
| 00→01, 01→11, 11→10, 10→00 | −1 (backward) |
| No change | 0 |
| 00↔11, 01↔10 | 0, counted as invalid |

In code this is a 16-entry table indexed by `(previous << 2) | current`. [`tools/01_encoder_raw_test`](../tools/01_encoder_raw_test/) implements exactly this so the logic can be watched on the Serial Monitor.

### The compact rule

The same table can be written as two rules:

| Which pin changed | State after the change | Direction |
|---|---|---|
| A | A ≠ B | Forward |
| A | A = B | Backward |
| B | A = B | Forward |
| B | A ≠ B | Backward |

This is what the ESP32's pulse-counter hardware implements.

### "Forward" is a convention

The encoder does not know which way the robot faces. Swapping the A and B wires reverses the sign. The left and right motors are mounted as mirror images, so one of them counts backwards when the robot drives forward. The firmware corrects this with `ENC_INVERT` and `MOTOR_INVERT` for each wheel, so that **positive always means robot forward**.

## 4. A "step" and the count

A step is one edge on A or B. It is the smallest movement the encoder can detect. The decoder keeps a running total:

- a forward step adds 1
- a backward step subtracts 1

This total is the **count**. It is the shaft position since power-on, measured in encoder steps. Turning ten turns forward and ten turns back returns it to zero.

## 5. PPR, CPR and decoding modes

**PPR (pulses per revolution)** is the number of full pulse periods on one channel during one turn of the shaft the encoder is mounted on. For a Hall encoder it equals the number of pole pairs on the magnet. It is a property of the hardware.

**CPR (counts per revolution)** is the number of steps the decoder counts per turn. It depends on how many of the four edges are counted:

| Mode | Edges counted | Counts per pulse | CPR |
|---|---|---|---|
| 1× | Rising edges of A | 1 | PPR |
| 2× | Rising and falling edges of A | 2 | 2 × PPR |
| 4× | Every edge of A and B | 4 | 4 × PPR |

The firmware uses **4×** decoding. It gives four times the resolution of 1× from the same hardware.

### The gearbox

The encoder is on the motor shaft, and the wheel is on the gearbox output. The wheel turns slower than the motor by the gear ratio, so:

```
CPR at the wheel = PPR × 4 × gear ratio
```

## 6. The numbers for this robot

The encoder is listed by the seller as 7 PPR, which gives 7 × 4 = **28 counts per motor-shaft turn** with 4× decoding.

The counts per **wheel** revolution were measured directly, because seller data for gear ratios is often inexact:

1. Mark the output shaft and a reference point on the motor body.
2. Reset the count.
3. Turn the output shaft exactly 10 full turns by hand and read the count.
4. Divide by 10.

| | Left motor | Right motor |
|---|---|---|
| Measured counts per wheel revolution | **752.6** (7526 counts in 10 turns) | **536.1** |
| Implied gear ratio (÷ 28) | 26.88 ≈ 26.9 : 1 | 19.15 ≈ 19.1 : 1 |
| One count equals | 0.478° of wheel rotation | 0.672° of wheel rotation |

Turning the left shaft back 10 turns returned the count to exactly 0. That confirms both channels work, direction detection is correct, and no counts are lost or invented.

Notes on these values:

- Both gear ratios match standard two-stage planetary ratios (about 5.18 × 5.18 and 3.71 × 5.18). That supports the 7 PPR figure, but 7 PPR was not verified directly. The test measures only the product PPR × 4 × gear ratio.
- **The firmware uses only the measured counts per wheel revolution.** The split between PPR and gear ratio does not affect any calculation.
- For more precision, repeat the measurement over 30 to 50 turns. The stopping error stays the same, so it is divided by a larger number.

## 7. From counts to angle and speed

For each wheel, using that wheel's own CPR:

```
angle (rad)   = 2π × count / CPR
speed (rad/s) = 2π × (count_now − count_previous) / (CPR × Δt)
```

The angle is **cumulative and signed**. It keeps growing as the wheel turns (one turn is 6.283 rad, ten turns 62.83 rad) and becomes negative in reverse. It is not wrapped to 0–2π. It starts at 0 when the ESP32 boots.

Because each wheel converts with its own CPR on the ESP32, everything downstream (the controller, the telemetry and the Raspberry Pi) works in the same physical units and never sees counts.

### Ways to measure speed

| Method | How | Good at | Weak at |
|---|---|---|---|
| Fixed time ("M method") | Count the steps in a fixed window Δt | Medium and high speed | Low speed (few steps per window) |
| Period ("T method") | Measure the time between two steps | Low speed | High speed, and it never reads zero |
| Combined ("M/T") | Count steps and measure the exact time they took | Both | More complex |

The firmware uses the fixed-time method with a 20 ms window (50 Hz).

### Quantization

With a fixed window, the smallest speed change that can be seen is one count per window:

```
resolution = 2π / (CPR × Δt)
```

| | Left (752.6) | Right (536.1) |
|---|---|---|
| Resolution at 50 Hz (Δt = 20 ms) | 0.42 rad/s | 0.59 rad/s |
| Resolution at 100 Hz (Δt = 10 ms) | 0.83 rad/s | 1.17 rad/s |

This is why the loop runs at 50 Hz and not faster: a shorter window contains fewer counts and gives a noisier speed. The firmware also smooths the reading with a low-pass filter:

```
speed_filtered = α × speed_raw + (1 − α) × speed_filtered        (α = VEL_ALPHA = 0.3)
```

A smaller α gives a smoother but more delayed reading.

## 8. How the ESP32 reads the encoders

| Method | Who notices each edge | Used in |
|---|---|---|
| Polling | The main loop keeps reading the pins | `tools/01_encoder_raw_test` only |
| Interrupts | Each edge interrupts the CPU and runs a small function | Not used |
| Hardware counter (PCNT) | A dedicated counter circuit in the ESP32 | All other tools and the firmware |

The **PCNT (pulse counter)** peripheral decodes quadrature in hardware:

- Edges on A count up or down depending on the level of B, and edges on B depending on the level of A. This is the compact rule from section 3.
- It uses no CPU time per edge, so it does not miss counts while the CPU is busy with Wi-Fi, micro-ROS or printing.
- It has a glitch filter. `setFilter(1023)` ignores pulses shorter than about 12.8 µs.
- The hardware counter is 16-bit. The **ESP32Encoder** library extends it to 64 bits with an interrupt that fires only when the counter reaches its limit, which is once every several seconds at most.
- The ESP32 has 8 PCNT units. Each encoder uses one.

In code:

```cpp
ESP32Encoder enc;
enc.attachFullQuad(pinA, pinB);   // 4× decoding in hardware
enc.setFilter(1023);              // glitch filter
int64_t count = enc.getCount();   // reads the value the hardware has been keeping
```

At full speed the motors produce a few thousand counts per second, far below what the hardware can handle.

## 9. Hardware rules

- **Voltage:** ESP32 pins are 3.3 V only. Power the encoder from 3.3 V so its outputs are 3.3 V. If an encoder needs 5 V, use a level shifter or resistor divider on A and B.
- **Never connect the motor supply to the encoder or the ESP32.**
- **Pull-ups:** many Hall encoders have open-collector outputs that can only pull the line low. The firmware enables the ESP32's internal pull-ups. External 4.7 kΩ to 10 kΩ pull-ups to 3.3 V are stronger and reduce noise pickup on the robot.
- **Noise:** keep encoder wires away from motor power wires, twist A and B with ground, and fit a 100 nF ceramic capacitor across each motor's terminals.
- **Pins:** avoid the strapping pins (GPIO 0, 2, 5, 12, 15) and the input-only pins 34 to 39, which have no internal pull-ups.
- **GPIO 25 and 26** (right encoder) are ADC2 pins. ADC2 cannot be used for analog readings while Wi-Fi is on, but digital input and the pulse counter are unaffected.

## 10. Quick reference

| Quantity | Formula | Left | Right |
|---|---|---|---|
| Counts per motor-shaft turn | PPR × 4 | 28 | 28 |
| Counts per wheel turn | measured | 752.6 | 536.1 |
| Gear ratio | CPR ÷ 28 | ≈ 26.9 | ≈ 19.1 |
| Wheel angle per count | 360° ÷ CPR | 0.478° | 0.672° |
| Wheel angle (rad) | 2π × count ÷ CPR | | |
| Wheel speed (rad/s) | 2π × Δcount ÷ (CPR × Δt) | | |
| rad/s to rpm | × 9.549 | | |
| Distance rolled (m) | angle (rad) × wheel radius (m) | | |
