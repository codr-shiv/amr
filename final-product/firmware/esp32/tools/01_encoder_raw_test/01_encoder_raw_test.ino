// =====================================================================
// TOOL 01: RAW QUADRATURE CHECK (one encoder, turned SLOWLY by hand)
// ---------------------------------------------------------------------
// Purpose: see the A/B signals and the decoding logic with your own eyes.
// It POLLS the two pins and prints every state change, so it only keeps
// up with slow hand rotation. It is a learning/diagnostic tool, not the
// method used on the robot (the robot uses the PCNT hardware counter).
//
// What to expect while turning slowly one way:
//   AB cycles 00 -> 10 -> 11 -> 01 -> 00 (or the reverse), one bit at a time
//   count goes up steadily (down when turning the other way)
//   invalid stays at 0 (it rises if you turn too fast, or if there is noise)
//
// Wiring: encoder VCC -> 3V3, GND -> GND, A -> GPIO 32, B -> GPIO 33
//         (for the right motor's encoder use GPIO 25 / 26)
// Serial Monitor: 115200 baud
// =====================================================================

const int PIN_A = 32, PIN_B = 33;

// Lookup table. index = (previous AB state << 2) | current AB state
//   +1 = one step forward, -1 = one step backward,
//    0 = no change, or both bits changed at once (a state was skipped)
const int8_t QEM[16] = { 0, -1,  1,  0,
                         1,  0,  0, -1,
                        -1,  0,  0,  1,
                         0,  1, -1,  0 };

int  prev    = -1;
long count   = 0;   // running position in encoder steps
long invalid = 0;   // number of skipped states (each one costs 2 steps)

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== Encoder raw test started ===");
  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
}

void loop() {
  int a = digitalRead(PIN_A), b = digitalRead(PIN_B);
  int curr = (a << 1) | b;
  if (curr != prev) {
    if (prev >= 0) {
      int8_t step = QEM[(prev << 2) | curr];
      if (step == 0) invalid++;   // both bits changed = missed a state or noise
      count += step;
    }
    Serial.printf("AB=%d%d  count=%ld  invalid=%ld\n", a, b, count, invalid);
    prev = curr;
  }
}
