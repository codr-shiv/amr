// TOOL 02: TWO-MOTOR ENCODER TEST - hardware quadrature decoding (PCNT) for both motors
// Prints count, revolutions, instantaneous rpm and average rpm since start for each motor.
// Library: ESP32Encoder (by Kevin Harrington / madhephaestus) - install from Library Manager.

#include <ESP32Encoder.h>

// ---------------- Motor 1 ----------------
const int   M1_A = 32;          // encoder channel A
const int   M1_B = 33;          // encoder channel B
const float CPR1 = 752.6;       // counts per OUTPUT-shaft revolution (measured)

// ---------------- Motor 2 ----------------
const int   M2_A = 25;          // encoder channel A
const int   M2_B = 26;          // encoder channel B
const float CPR2 = 536.1;       // counts per OUTPUT-shaft revolution (measured)

const uint32_t PRINT_MS = 200;  // print / speed-calculation interval (ms)

ESP32Encoder enc1, enc2;        // each object uses its own hardware PCNT unit
int64_t  last1 = 0, last2 = 0;  // counts at previous print (for instantaneous speed)
uint32_t lastMs = 0, startMs = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;  // remove this line if it doesn't compile (older library)

  enc1.attachFullQuad(M1_A, M1_B);   // 4x decoding in hardware
  enc2.attachFullQuad(M2_A, M2_B);
  enc1.setFilter(1023);              // ignore glitches shorter than ~12.8 us
  enc2.setFilter(1023);
  enc1.clearCount();
  enc2.clearCount();

  startMs = lastMs = millis();
  Serial.println("=== Two-motor encoder test started - start your stopwatch NOW ===");
}

void loop() {
  uint32_t now = millis();
  if (now - lastMs < PRINT_MS) return;

  int64_t c1 = enc1.getCount();
  int64_t c2 = enc2.getCount();

  float dt   = (now - lastMs)  / 1000.0f;   // seconds since last print
  float secs = (now - startMs) / 1000.0f;   // seconds since start

  // revolutions since start
  float rev1 = c1 / CPR1;
  float rev2 = c2 / CPR2;

  // instantaneous speed over the last PRINT_MS window
  float rpm1 = (c1 - last1) / dt / CPR1 * 60.0f;
  float rpm2 = (c2 - last2) / dt / CPR2 * 60.0f;

  // average speed since start (compare this with your stopwatch count)
  float avg1 = rev1 / secs * 60.0f;
  float avg2 = rev2 / secs * 60.0f;

  Serial.printf("t=%6.1fs | M1: count=%7lld revs=%8.2f rpm=%7.1f avg=%7.1f"
                " | M2: count=%7lld revs=%8.2f rpm=%7.1f avg=%7.1f\n",
                secs, c1, rev1, rpm1, avg1, c2, rev2, rpm2, avg2);

  last1 = c1;
  last2 = c2;
  lastMs = now;
}
