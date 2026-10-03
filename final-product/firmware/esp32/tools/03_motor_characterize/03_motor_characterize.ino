// =====================================================================
// TOOL 03: OPEN-LOOP CHARACTERISATION OF BOTH MOTORS
// ---------------------------------------------------------------------
// Sweeps PWM from 0 to max (forward, then reverse) on BOTH motors at the
// same time, measures each wheel's steady speed, and at the end prints,
// for EACH motor separately:
//   - deadband  : smallest PWM that makes the wheel turn
//   - kff       : PWM needed per 1 rad/s   (feedforward gain)
//   - pwmMin    : PWM offset from the straight-line fit
//   - maxSpeed  : speed at 100% PWM (rad/s and rpm)
//   - a sign check (does positive PWM give positive encoder speed?)
//
// !!! LIFT THE WHEELS OFF THE GROUND BEFORE RUNNING !!!
// Driver: Cytron in PWM + DIR mode (MDD10A / MD10C / MD13S / MD20A ...)
// Library: ESP32Encoder (Kevin Harrington / madhephaestus)
// =====================================================================

#include <ESP32Encoder.h>
#include "esp_arduino_version.h"

// ---------------- PINS (keep identical in the PID sketch) ----------------
// LEFT  = Motor 1 (26.9:1, CPR 752.6)
const int L_ENC_A = 32, L_ENC_B = 33;
const int L_PWM   = 18, L_DIR   = 19;
// RIGHT = Motor 2 (19.1:1, CPR 536.1)
const int R_ENC_A = 25, R_ENC_B = 26;
const int R_PWM   = 23, R_DIR   = 22;

const float L_CPR = 752.6;   // counts per OUTPUT-shaft revolution (measured)
const float R_CPR = 536.1;

// ---------------- PWM ----------------
const int PWM_FREQ = 20000;  // 20 kHz: inaudible, within Cytron's limit
const int PWM_BITS = 10;     // duty 0..1023
const int PWM_MAX  = 1023;
const int L_CH = 0, R_CH = 1;   // LEDC channels (only used on Arduino core 2.x)

// ---------------- SWEEP SETTINGS ----------------
const int      PWM_STEP   = 32;     // PWM increment per step
const uint32_t SETTLE_MS  = 1500;   // wait for speed to settle after each change
const uint32_t MEASURE_MS = 1000;   // averaging window for speed
const float    MOVING_THR = 0.5;    // rad/s: above this the wheel counts as "moving"
const float    FIT_THR    = 1.0;    // rad/s: only points above this are used for the fit

ESP32Encoder encL, encR;

struct Pt { int pwm; float wL, wR; };
Pt fwd[40], rev[40];
int nFwd = 0, nRev = 0;

// ---------- PWM helpers (work on Arduino-ESP32 core 2.x and 3.x) ----------
void pwmSetup(int pin, int ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(ch, PWM_FREQ, PWM_BITS);
  ledcAttachPin(pin, ch);
#endif
}

void pwmWrite(int pin, int ch, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}

// signed command: sign -> DIR pin, magnitude -> PWM duty
void drive(int pwmPin, int ch, int dirPin, int u) {
  u = constrain(u, -PWM_MAX, PWM_MAX);
  digitalWrite(dirPin, u >= 0 ? HIGH : LOW);
  pwmWrite(pwmPin, ch, abs(u));
}

void driveBoth(int u) {
  drive(L_PWM, L_CH, L_DIR, u);
  drive(R_PWM, R_CH, R_DIR, u);
}

// average speed (rad/s) of both wheels over MEASURE_MS
void measureBoth(float &wL, float &wR) {
  int64_t l0 = encL.getCount(), r0 = encR.getCount();
  uint32_t t0 = micros();
  delay(MEASURE_MS);
  int64_t l1 = encL.getCount(), r1 = encR.getCount();
  float dt = (micros() - t0) / 1e6f;
  wL = TWO_PI * (l1 - l0) / (L_CPR * dt);
  wR = TWO_PI * (r1 - r0) / (R_CPR * dt);
}

// slowly bring the motors back to zero (never slam a reversal)
void rampToZero(int from) {
  for (int u = from; abs(u) > 0; u -= (u > 0 ? PWM_STEP : -PWM_STEP)) {
    if ((from > 0 && u < 0) || (from < 0 && u > 0)) break;
    driveBoth(u);
    delay(60);
  }
  driveBoth(0);
  delay(2000);
}

void sweep(int sign, Pt *pts, int &n) {
  Serial.println(sign > 0 ? "\n--- FORWARD sweep ---" : "\n--- REVERSE sweep ---");
  Serial.println("dir,pwm,L_rad_s,L_rpm,R_rad_s,R_rpm");
  n = 0;
  for (int pwm = 0; ; pwm += PWM_STEP) {
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    driveBoth(sign * pwm);
    delay(SETTLE_MS);
    float wL, wR;
    measureBoth(wL, wR);
    pts[n++] = { pwm, wL, wR };
    Serial.printf("%s,%d,%.2f,%.1f,%.2f,%.1f\n", sign > 0 ? "F" : "R", pwm,
                  wL, wL * 60 / TWO_PI, wR, wR * 60 / TWO_PI);
    if (pwm == PWM_MAX || n >= 40) break;
  }
  rampToZero(sign * PWM_MAX);
}

// straight-line fit  pwm = kff * |w| + pwmMin  over points that are clearly moving
void analyse(const char *label, Pt *pts, int n, bool left, int sign) {
  int deadband = -1;
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  int m = 0;
  float wMaxSigned = left ? pts[n - 1].wL : pts[n - 1].wR;

  for (int i = 0; i < n; i++) {
    float w = fabs(left ? pts[i].wL : pts[i].wR);
    if (deadband < 0 && w > MOVING_THR) deadband = pts[i].pwm;
    if (w > FIT_THR) {
      sx += w; sy += pts[i].pwm; sxx += w * w; sxy += w * pts[i].pwm; m++;
    }
  }

  Serial.printf("%-5s %-7s: ", left ? "LEFT" : "RIGHT", label);
  if (m < 2) { Serial.println("not enough moving points - check wiring/power"); return; }
  double kff  = (m * sxy - sx * sy) / (m * sxx - sx * sx);
  double pmin = (sy - kff * sx) / m;
  Serial.printf("deadband~%4d  kff=%6.2f PWM/(rad/s)  pwmMin=%6.1f  maxSpeed=%6.2f rad/s (%5.1f rpm)",
                deadband, kff, pmin, fabs(wMaxSigned), fabs(wMaxSigned) * 60 / TWO_PI);
  // sign check: forward sweep should give positive speed, reverse negative
  if ((sign > 0 && wMaxSigned < 0) || (sign < 0 && wMaxSigned > 0))
    Serial.print("   <-- SIGN MISMATCH: set ENC_INVERT=true for this motor");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(L_DIR, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pwmSetup(L_PWM, L_CH);
  pwmSetup(R_PWM, R_CH);
  driveBoth(0);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;  // delete this line if it doesn't compile (older library)
  encL.attachFullQuad(L_ENC_A, L_ENC_B);
  encR.attachFullQuad(R_ENC_A, R_ENC_B);
  encL.setFilter(1023);
  encR.setFilter(1023);

  Serial.println("\n=== OPEN-LOOP MOTOR CHARACTERISATION ===");
  Serial.println("LIFT THE WHEELS OFF THE GROUND, switch on motor power,");
  Serial.println("then send any character (press Enter) to start...");
  while (!Serial.available()) delay(10);
  while (Serial.available()) Serial.read();

  sweep(+1, fwd, nFwd);
  sweep(-1, rev, nRev);
  driveBoth(0);

  Serial.println("\n================ RESULTS ================");
  analyse("forward", fwd, nFwd, true, +1);
  analyse("reverse", rev, nRev, true, -1);
  analyse("forward", fwd, nFwd, false, +1);
  analyse("reverse", rev, nRev, false, -1);
  Serial.println("\nUse the AVERAGE of forward/reverse kff and pwmMin for each motor");
  Serial.println("in the PID sketch. Use ~85% of the SMALLER maxSpeed as the robot's");
  Serial.println("wheel speed limit (it will be lower again under load).");
  Serial.println("Done. Motors stopped.");
}

void loop() {}
