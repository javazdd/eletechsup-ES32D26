// Eletechsup ES32D26 relays via 74HC595 + ULN2803A
// Mapping (from user continuity):
// SER/DATA -> GPIO12
// SRCLK    -> GPIO22
// RCLK     -> GPIO23 (latch)
// OE       -> GPIO13 (active LOW)
// MR       -> tied HIGH (not controlled)

const int PIN_DATA  = 12;
const int PIN_CLK   = 22;
const int PIN_LATCH = 23;
const int PIN_OE    = 13;  // active LOW

inline void srLatch() {
  digitalWrite(PIN_LATCH, LOW);
  digitalWrite(PIN_LATCH, HIGH);
}

inline void srWrite(uint8_t value) {
  // Shift out LSB-first; channel order may vary but all 8 will toggle
  for (int i = 0; i < 8; ++i) {
    digitalWrite(PIN_CLK, LOW);
    digitalWrite(PIN_DATA, (value >> i) & 0x01);
    digitalWrite(PIN_CLK, HIGH);
  }
  srLatch();
}

void setup() {
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_LATCH, OUTPUT);
  pinMode(PIN_OE, OUTPUT);

  digitalWrite(PIN_DATA, LOW);
  digitalWrite(PIN_CLK, LOW);
  digitalWrite(PIN_LATCH, HIGH);

  // Enable outputs (active LOW)
  digitalWrite(PIN_OE, LOW);

  // All relays OFF initially (ULN2803 sinks when input is HIGH)
  srWrite(0x00);
}

void loop() {
  // Turn each relay ON for 1s, then OFF
  for (int r = 0; r < 8; ++r) {
    uint8_t onVal = (1 << r);  // active-HIGH into ULN2803 -> relay ON
    srWrite(onVal);
    delay(1000);
    srWrite(0x00);
    delay(400);
  }
}
