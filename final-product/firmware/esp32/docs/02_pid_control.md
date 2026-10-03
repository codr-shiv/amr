# Wheel velocity control (PID)

This document explains the control loop that makes each wheel turn at the speed the Raspberry Pi asks for, the reasoning behind each part, and how its parameters were measured and tuned.

## 1. Why a control loop is needed

Sending a fixed PWM to a motor does not give a fixed speed. The speed changes with battery voltage, load, floor surface and friction, and two motors never behave identically. On this robot the two motors even have different gearboxes.

A closed loop measures the real speed, compares it with the target and corrects continuously:

```
target ──►(+)── error ──► controller ── PWM ──► driver + motor ──┬──► wheel speed
           ▲ −                                                    │
           └────────── measured speed (from the encoder) ◄────────┘
```

### Inner and outer loops

The robot uses cascade control, with two nested loops:

| Loop | Runs on | Rate | Job |
|---|---|---|---|
| Outer | Raspberry Pi | 20 to 50 Hz | Decide what speed each wheel should have |
| Inner | ESP32 | 50 Hz, fixed period | Make each wheel actually reach that speed |

The inner loop runs on the ESP32 because it needs exact timing, which Linux on the Pi cannot guarantee.

## 2. The three PID terms

With error `e = target − measured`:

| Term | Formula | Reacts to | Effect | Drawback |
|---|---|---|---|---|
| P (proportional) | `Kp × e` | The present error | Fast response | Alone, it leaves a steady error. Too high causes oscillation |
| I (integral) | `Ki × ∫e dt` | Accumulated past error | Removes the steady error | Overshoot, and windup when the output saturates |
| D (derivative) | `Kd × de/dt` | The trend of the error | Damping, less overshoot | Amplifies measurement noise |

**Why P alone leaves an error:** to keep a motor turning, some PWM is always needed. With P alone, PWM = Kp × error, so a non-zero PWM requires a non-zero error. The wheel settles slightly below the target.

**Why there is no D term here:** the speed comes from counting encoder steps in a 20 ms window, which is a coarse, noisy signal (see [encoder theory, quantization](01_encoder_theory.md#quantization)). A derivative would amplify that noise. Feedforward plus PI is the standard choice for wheel speed control.

### Effect of raising each gain

| Gain | Rise time | Overshoot | Steady error | Stability |
|---|---|---|---|---|
| Kp ↑ | Faster | More | Smaller, never zero | Worse |
| Ki ↑ | Faster | More | Removed | Worse |
| Kd ↑ | Little change | Less | No effect | Better |

## 3. The controller in the firmware

For each wheel, every 20 ms:

```
PWM = KFF × ref + PWM_MIN × sign(ref)      feedforward   (does most of the work)
    + KP × error                           proportional  (fast correction)
    + integral of (KI × error)             integral      (removes the remaining error)
```

where `ref` is the ramped target and `error = ref − measured speed`.

The steps, in the order they run in `updateWheel()`:

### 3.1 Measure

Read the encoder count, convert the change since the last cycle to rad/s using that wheel's CPR, and low-pass filter it (`VEL_ALPHA = 0.3`).

### 3.2 Ramp the target

The target is not applied as a jump. `ref` moves towards it by at most `ACCEL_LIMIT × Δt` each cycle (`ACCEL_LIMIT = 40 rad/s²`, so 0 to 10 rad/s takes 0.25 s). This avoids current spikes and wheel slip, and makes both wheels change speed together even though their motors differ.

### 3.3 Feedforward

For a DC motor, steady speed is close to proportional to PWM. So the PWM a given speed needs can be predicted directly:

```
feedforward = KFF × ref + PWM_MIN (with the sign of ref)
```

- `KFF` is the PWM needed per rad/s.
- `PWM_MIN` is the offset needed to overcome static friction (the deadband).

Feedforward acts immediately, without waiting for an error to build up. The P and I terms then only correct what is left, so they can be smaller and the response has less overshoot.

### 3.4 Proportional and integral

The integral is stored directly in PWM units (`iTerm += KI × error × Δt`), so changing `KI` while running does not cause a jump.

### 3.5 Anti-windup

If the wheel cannot reach the target (blocked, or the PWM is already at maximum), the error stays large and a plain integral would grow without limit. When the wheel is freed it would then race until the integral unwinds. Two protections prevent this:

- The integral contribution is clamped to ±`I_LIMIT` (400 PWM units).
- The integral is frozen while the output is saturated and the error would push it further into saturation.

The integral is also cleared whenever the wheel is commanded to stop and has ramped to zero.

### 3.6 Output

The result is limited to ±1023. The sign sets the DIR pin and the magnitude sets the PWM duty.

### 3.7 Speed limit for both wheels together

`setTargets()` limits each target to that wheel's `MAX_SPEED`. If either target is too high, **both** are multiplied by the same factor. The ratio between the wheels, and so the curvature of the robot's path, is preserved. A request for 30 and 15 rad/s becomes 17 and 8.5.

## 4. Two different motors

The left motor (≈ 26.9 : 1) and the right motor (≈ 19.1 : 1) give different speeds for the same PWM. Three things in the firmware deal with this:

1. **Separate CPR.** Each wheel converts counts to rad/s with its own value, so both speeds are in the same unit before they are compared with a target.
2. **Separate controller parameters.** Each wheel has its own `KFF`, `PWM_MIN`, `KP` and `KI`. The feedforward gives each motor the PWM that motor needs for the requested speed, and PI removes the rest. Commanding the same rad/s to both wheels therefore produces different PWM values and the same wheel speed.
3. **A common speed limit.** `MAX_SPEED` is the same for both and is set below what the slower motor can reach, so the faster motor is never asked for a speed the slower one cannot match.

Remaining differences are physical: the two wheels have different torque and slightly different speed resolution. The ramp and the PI terms handle these in normal driving. Two identical motors would still be the better hardware choice.

## 5. Current values

| Parameter | Left | Right | Source |
|---|---|---|---|
| `CPR` | 752.6 | 536.1 | Measured (10-turn test) |
| `KFF` (PWM per rad/s) | 49.72 | 43.54 | Measured with `tools/03_motor_characterize` |
| `PWM_MIN` | 11.8 | 12.0 | Measured with `tools/03_motor_characterize` |
| `KP` (PWM per rad/s of error) | 30.0 | 25.0 | Tuned with `tools/04_wheel_pid_tuning` |
| `KI` (PWM per rad of accumulated error) | 12.5 | 10.0 | Tuned with `tools/04_wheel_pid_tuning` |
| `MAX_SPEED` (rad/s) | 17.0 | 17.0 | Chosen below the slower wheel's maximum |

Shared settings:

| Constant | Value | Meaning |
|---|---|---|
| `LOOP_HZ` | 50 | Control rate (20 ms period) |
| `VEL_ALPHA` | 0.3 | Speed filter. 1 = no filtering, smaller = smoother and slower |
| `ACCEL_LIMIT` | 40 rad/s² | Maximum rate of change of the target |
| `I_LIMIT` | 400 | Maximum PWM from the integral term |
| `PWM_FREQ` | 20 kHz | Inaudible and within the Cytron driver's limit |
| `PWM_BITS` | 10 | Duty range 0 to 1023 |

`KFF` and `PWM_MIN` depend on the motor supply voltage. If the battery or supply changes, re-run the characterisation.

## 6. PWM and the motor driver

The ESP32 cannot output an analog voltage. It switches a pin on and off 20,000 times per second and varies the fraction of time it is on (the duty cycle). The motor responds to the average.

The Cytron driver is used in **PWM + DIR** (sign-magnitude) mode: one pin sets the direction, the other carries the PWM.

The ESP32 generates PWM with its **LEDC** peripheral, which has 16 channels. Each motor must use its own channel (`L_CH = 0`, `R_CH = 1`) so the two duty cycles are independent. On Arduino-ESP32 core 3.x the channel is assigned automatically and these constants are ignored. The firmware supports both core 2.x and 3.x.

## 7. How the parameters were obtained

### Step 1: characterise each motor (open loop)

Tool: [`tools/03_motor_characterize`](../tools/03_motor_characterize/). **Wheels off the ground.**

It raises the PWM from 0 to 1023 in steps, forward and then reverse, waits at each step, and measures the steady speed of both wheels. At the end it fits a straight line `PWM = KFF × speed + PWM_MIN` for each motor and direction, and prints:

- the deadband (smallest PWM that makes the wheel turn)
- `KFF` and `PWM_MIN`
- the maximum speed at 100 % PWM
- a sign check: if positive PWM gives negative speed, that wheel needs `ENC_INVERT = true`

Use the average of the forward and reverse values. Set `MAX_SPEED` to about 85 % of the smaller maximum.

### Step 2: check feedforward alone

Tool: [`tools/04_wheel_pid_tuning`](../tools/04_wheel_pid_tuning/) with `kp` and `ki` set to 0.

Send `t 5`, `t 10`, `t 15`. Each wheel should already run within roughly 10 to 20 % of the target. If one wheel is consistently off, correct its feedforward:

```
KFF_new = KFF_old × target / measured
```

With feedback off, a wrong encoder sign cannot cause a runaway, so this is also the safe moment to check directions.

### Step 3: tune KP, then KI

Open the Arduino Serial Plotter and start a square-wave target with `sq 10 2000`.

1. With `ki` at 0, raise `kp` until the measured speed follows the target quickly. If it wobbles, looks jagged or the motor buzzes, halve it.
2. Raise `ki` until any steady gap closes within about 0.3 to 0.5 s. If it overshoots and swings slowly, lower it.
3. Use `plot 2` to check the PWM. It should stay well below 100 % in normal running.
4. Press lightly on the tyre. The speed should dip and recover.
5. Repeat for the other wheel, then test both together and on the floor.

A reasonable starting value is `KP ≈ 0.4 × KFF`. Start `KI` small and raise it in steps.

### Reading the plot

| What you see | Cause | Change |
|---|---|---|
| Rises slowly, never quite reaches the target | Gains or feedforward too low | Raise `KP`, then `KI`; check `KFF` |
| Overshoots once, then settles | Slightly aggressive, usually acceptable | Lower `KI` a little |
| Keeps oscillating or buzzing | `KP` too high | Lower `KP` |
| Slow repeated swings | `KI` too high | Lower `KI` |
| Jerky at very low speed | Deadband compensation off | Adjust `PWM_MIN` |
| Noisy reading at constant speed | Encoder quantization | Lower `VEL_ALPHA` |
| PWM stuck at 100 % | Target above what the motor can do | Lower `MAX_SPEED` |
| Wheel runs away at full speed as soon as `KP` > 0 | Encoder sign opposite to motor sign | Fix `ENC_INVERT` for that wheel |

Values set with serial commands are lost on reset. Use `p` to print them and copy them into the constants at the top of the firmware.
