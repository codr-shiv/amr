// =====================================================================
// TOOL 04: PER-WHEEL VELOCITY CONTROL  (feedforward + PI, one set of
//          parameters for EACH motor) with live tuning over Serial
//          Stand-alone bench sketch: no Wi-Fi, no micro-ROS.
// ---------------------------------------------------------------------
// For each wheel, every control cycle:
//   1. read encoder  -> filtered speed (rad/s) using THAT wheel's CPR
//   2. ramp the setpoint (acceleration limit)
//   3. PWM = kff*ref + pwmMin (feedforward)  +  kp*error  +  integral
//   4. anti-windup, saturation, write PWM + DIR to the Cytron driver
//
// Units: speeds in rad/s at the OUTPUT shaft (1 rad/s = 9.549 rpm)
//
// ---------------- SERIAL COMMANDS (set line ending to "Newline") -------
//   t 10          both wheels target 10 rad/s
//   t 10 5        left 10, right 5 rad/s
//   s             stop (targets 0, square wave off, closed loop)
//   sq 15 2000    square-wave test: 15 rad/s <-> 0 every 1000 ms (period 2000)
//   sq 0          square wave off
//   pwm 300 300   OPEN-LOOP: raw PWM left/right (-1023..1023), PID bypassed
//   kp L 12       set a gain for L, R or B (both):  kp ki kff min max
//   p             print all parameters
//   plot 0|1|2    0 = off, 1 = speeds, 2 = speeds + PWM %
//
// Library: ESP32Encoder (Kevin Harrington / madhephaestus)
// =====================================================================

#include <ESP32Encoder.h>
#include "esp_arduino_version.h"

// ======================= LEFT = MOTOR 1 (26.9:1) =======================
const int   L_ENC_A = 32, L_ENC_B = 33, L_PWM = 18, L_DIR = 19, L_CH = 0;
const float L_CPR          = 752.6;
const bool  L_ENC_INVERT   = false;  // true if +PWM gives -speed (see characterisation sign check)
const bool  L_MOTOR_INVERT = false;  // flips motor direction (then flip ENC_INVERT too)
// ---- tuning for LEFT (kff/min from tool 03, then tune kp/ki) ----
float L_KFF       = 49.72;    // PWM per rad/s          (from characterisation)
float L_PWM_MIN   = 11.8;    // deadband compensation  (from characterisation)
float L_KP        = 30.0;    // PWM per rad/s of error
float L_KI        = 12.50;    // PWM per rad (integral of error)
float L_MAX_SPEED = 17.00;   // rad/s this wheel may be commanded

// ======================= RIGHT = MOTOR 2 (19.1:1) ======================
const int   R_ENC_A = 25, R_ENC_B = 26, R_PWM = 23, R_DIR = 22, R_CH = 1;
const float R_CPR          = 536.1;
const bool  R_ENC_INVERT   = true;
const bool  R_MOTOR_INVERT = true;
// ---- tuning for RIGHT ----
float R_KFF       = 43.54;
float R_PWM_MIN   = 12.0;
float R_KP        = 25.0;
float R_KI        = 10.0;
float R_MAX_SPEED = 17.00;

// ============================ GENERAL =================================
const int      PWM_FREQ       = 20000;
const int      PWM_BITS       = 10;
const int      PWM_MAX        = 1023;
const float    LOOP_HZ        = 50.0;   // control rate (20 ms)
const float    VEL_ALPHA      = 0.3;    // speed filter: 1 = no filtering, smaller = smoother but laggier
const float    ACCEL_LIMIT    = 40.0;   // rad/s per second: max setpoint change rate
const float    I_LIMIT        = 400.0;  // max PWM contributed by the integral term
const uint32_t CMD_TIMEOUT_MS = 0;      // 0 = off (bench). Set ~500 once commands come from the Pi!
const int      PRINT_EVERY    = 2;      // print every 2nd cycle -> 25 Hz

// ============================ WHEEL STATE ==============================
struct Wheel {
  const char   *name;
  int           pwmPin, dirPin, ch;
  float         cpr;
  bool          encInvert, motorInvert;
  float         kff, pwmMin, kp, ki, maxSpeed;   // per-wheel parameters
  ESP32Encoder *enc;
  int64_t       lastCount;
  float         target;   // requested speed (rad/s)
  float         ref;      // ramped setpoint actually tracked
  float         meas;     // filtered measured speed (rad/s)
  float         pos;      // wheel angle (rad)
  float         iTerm;    // integral contribution (PWM units)
  int           pwmOut;   // last command sent (signed)
};

ESP32Encoder encL, encR;
Wheel L, R;   // globals start zero-initialised

bool     openLoop   = false;
int      plotMode   = 1;
float    sqAmp      = 0;
uint32_t sqPeriodMs = 2000, sqStartMs = 0;
uint32_t lastCmdMs  = 0;
uint32_t lastUs     = 0;
uint32_t cycle      = 0;

// ---------------- PWM helpers (Arduino-ESP32 core 2.x and 3.x) ----------------
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

void initWheel(Wheel &w, const char *name, int encA, int encB, int pwmPin, int dirPin, int ch,
               float cpr, bool encInv, bool motInv, ESP32Encoder *enc) {
  w.name = name; w.pwmPin = pwmPin; w.dirPin = dirPin; w.ch = ch;
  w.cpr = cpr; w.encInvert = encInv; w.motorInvert = motInv; w.enc = enc;
  pinMode(dirPin, OUTPUT);
  pwmSetup(pwmPin, ch);
  enc->attachFullQuad(encA, encB);
  enc->setFilter(1023);
  enc->clearCount();
}

// signed PWM -> DIR pin + duty
void setMotor(Wheel &w, int u) {
  u = constrain(u, -PWM_MAX, PWM_MAX);
  w.pwmOut = u;
  if (w.motorInvert) u = -u;
  digitalWrite(w.dirPin, u >= 0 ? HIGH : LOW);
  pwmWrite(w.pwmPin, w.ch, abs(u));
}

// scale BOTH targets together if either exceeds its wheel's limit,
// so the ratio between the wheels (= the robot's curvature) is preserved
void setTargets(float l, float r) {
  float s = 1.0f;
  if (fabsf(l) > L.maxSpeed) s = fminf(s, L.maxSpeed / fabsf(l));
  if (fabsf(r) > R.maxSpeed) s = fminf(s, R.maxSpeed / fabsf(r));
  L.target = l * s;
  R.target = r * s;
}

// ---------------- the per-wheel controller ----------------
void updateWheel(Wheel &w, float dt) {
  // 1. measure
  int64_t c = w.enc->getCount();
  if (w.encInvert) c = -c;
  float raw = TWO_PI * (float)(c - w.lastCount) / (w.cpr * dt);
  w.lastCount = c;
  w.meas = VEL_ALPHA * raw + (1.0f - VEL_ALPHA) * w.meas;
  w.pos  = TWO_PI * (float)c / w.cpr;

  if (openLoop) return;   // PWM set directly by the "pwm" command

  // 2. ramp the setpoint
  float maxStep = ACCEL_LIMIT * dt;
  w.ref += constrain(w.target - w.ref, -maxStep, maxStep);

  // stopped: zero output, clear integral
  if (w.target == 0.0f && fabsf(w.ref) < 1e-3f) {
    w.ref = 0; w.iTerm = 0;
    setMotor(w, 0);
    return;
  }

  // 3. feedforward + PI
  float e  = w.ref - w.meas;
  float ff = w.kff * w.ref + (w.ref > 0 ? w.pwmMin : (w.ref < 0 ? -w.pwmMin : 0));
  float p  = w.kp * e;
  float iNew = constrain(w.iTerm + w.ki * e * dt, -I_LIMIT, I_LIMIT);
  float u = ff + p + iNew;

  // 4. anti-windup: don't let the integral grow while the output is saturated
  bool pushingHigher = (u >  PWM_MAX && e > 0);
  bool pushingLower  = (u < -PWM_MAX && e < 0);
  if (!pushingHigher && !pushingLower) w.iTerm = iNew;

  setMotor(w, (int)lroundf(ff + p + w.iTerm));
}

// ---------------- serial command interface ----------------
void printParams() {
  Wheel *ws[2] = { &L, &R };
  for (Wheel *w : ws)
    Serial.printf("# %s: kff=%.2f min=%.1f kp=%.2f ki=%.2f max=%.1f rad/s  cpr=%.1f\n",
                  w->name, w->kff, w->pwmMin, w->kp, w->ki, w->maxSpeed, w->cpr);
}

void setParam(Wheel &w, const char *key, float v) {
  if      (!strcmp(key, "kp"))  w.kp = v;
  else if (!strcmp(key, "ki"))  w.ki = v;
  else if (!strcmp(key, "kff")) w.kff = v;
  else if (!strcmp(key, "min")) w.pwmMin = v;
  else if (!strcmp(key, "max")) w.maxSpeed = v;
  else { Serial.printf("# unknown parameter '%s'\n", key); return; }
  w.iTerm = 0;
}

void runCommand(char *s) {
  float a, b; char key[8], who;
  lastCmdMs = millis();

  if (sscanf(s, "t %f %f", &a, &b) == 2)      { openLoop = false; sqAmp = 0; setTargets(a, b); }
  else if (sscanf(s, "t %f", &a) == 1)         { openLoop = false; sqAmp = 0; setTargets(a, a); }
  else if (sscanf(s, "sq %f %f", &a, &b) == 2) { openLoop = false; sqAmp = a; sqPeriodMs = (b >= 200) ? (uint32_t)b : 2000; sqStartMs = millis(); if (a == 0) setTargets(0, 0); }
  else if (sscanf(s, "sq %f", &a) == 1)        { openLoop = false; sqAmp = a; sqStartMs = millis(); if (a == 0) setTargets(0, 0); }
  else if (sscanf(s, "pwm %f %f", &a, &b) == 2) {
    openLoop = true; sqAmp = 0;
    L.target = R.target = L.ref = R.ref = 0; L.iTerm = R.iTerm = 0;
    setMotor(L, (int)a); setMotor(R, (int)b);
  }
  else if (sscanf(s, "plot %f", &a) == 1)      { plotMode = (int)a; }
  else if (!strcmp(s, "s"))                    { openLoop = false; sqAmp = 0; setTargets(0, 0); setMotor(L, 0); setMotor(R, 0); }
  else if (!strcmp(s, "p"))                    { printParams(); return; }
  else if (sscanf(s, "%7s %c %f", key, &who, &a) == 3) {
    who = toupper(who);
    if (who == 'L' || who == 'B') setParam(L, key, a);
    if (who == 'R' || who == 'B') setParam(R, key, a);
    printParams();
    return;
  }
  else { Serial.printf("# ? '%s'\n", s); return; }
}

void handleSerial() {
  static char buf[48];
  static int n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n) { buf[n] = 0; runCommand(buf); n = 0; }
    } else if (n < (int)sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  delay(500);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;  // delete if it doesn't compile (older library)
  initWheel(L, "L", L_ENC_A, L_ENC_B, L_PWM, L_DIR, L_CH, L_CPR, L_ENC_INVERT, L_MOTOR_INVERT, &encL);
  initWheel(R, "R", R_ENC_A, R_ENC_B, R_PWM, R_DIR, R_CH, R_CPR, R_ENC_INVERT, R_MOTOR_INVERT, &encR);

  L.kff = L_KFF; L.pwmMin = L_PWM_MIN; L.kp = L_KP; L.ki = L_KI; L.maxSpeed = L_MAX_SPEED;
  R.kff = R_KFF; R.pwmMin = R_PWM_MIN; R.kp = R_KP; R.ki = R_KI; R.maxSpeed = R_MAX_SPEED;

  setMotor(L, 0);
  setMotor(R, 0);

  Serial.println("# === per-wheel velocity PID ready ===");
  Serial.println("# commands: t <l> [r] | s | sq <amp> [period_ms] | pwm <l> <r> | kp|ki|kff|min|max L|R|B <v> | p | plot 0|1|2");
  printParams();
  lastUs = micros();
  lastCmdMs = millis();
}

void loop() {
  handleSerial();

  uint32_t nowUs = micros();
  if (nowUs - lastUs < (uint32_t)(1e6f / LOOP_HZ)) return;
  float dt = (nowUs - lastUs) / 1e6f;
  lastUs = nowUs;

  // square-wave test signal
  if (sqAmp != 0) {
    bool high = ((millis() - sqStartMs) % sqPeriodMs) < sqPeriodMs / 2;
    setTargets(high ? sqAmp : 0, high ? sqAmp : 0);
  }

  // safety: stop if commands stop arriving (enable when driven from the Pi)
  if (CMD_TIMEOUT_MS > 0 && millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    setTargets(0, 0);
    if (openLoop) { openLoop = false; setMotor(L, 0); setMotor(R, 0); }
  }

  updateWheel(L, dt);
  updateWheel(R, dt);

  if (plotMode && (++cycle % PRINT_EVERY == 0)) {
    if (plotMode == 1)
      Serial.printf("L_ref:%.2f,L_meas:%.2f,R_ref:%.2f,R_meas:%.2f\n", L.ref, L.meas, R.ref, R.meas);
    else
      Serial.printf("L_ref:%.2f,L_meas:%.2f,R_ref:%.2f,R_meas:%.2f,L_pwm%%:%.1f,R_pwm%%:%.1f\n",
                    L.ref, L.meas, R.ref, R.meas,
                    100.0f * L.pwmOut / PWM_MAX, 100.0f * R.pwmOut / PWM_MAX);
  }
}