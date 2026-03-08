// ===== XIAO ESP32S3 Sense =====
// Bit-bang UART 8N1 on GPIO
// TX = D6, RX = D7
// wake input = D0 (created only, logic later)
// ===== Arduino Nano 33 BLE =====
// Bit-bang UART 8N1
// RX = D5, TX = D6
// wake output = D3 (created only, logic later)

#include <Arduino.h>

static const int PIN_TX = D6;
static const int PIN_RX = D7;
static const int PIN_WAKE_IN = D0;

// Safer baud for bit-bang
static const uint32_t BAUD = 4800;
static const uint32_t BIT_US = 1000000UL / BAUD;

static inline void bb_wait_bit() {
  delayMicroseconds(BIT_US);
}

void bb_tx_byte(uint8_t b) {
  // Disable interrupts to keep timing stable while sending
  noInterrupts();

  // Start bit (LOW)
  digitalWrite(PIN_TX, LOW);
  bb_wait_bit();

  // Data bits LSB first
  for (int i = 0; i < 8; i++) {
    digitalWrite(PIN_TX, (b & 0x01) ? HIGH : LOW);
    bb_wait_bit();
    b >>= 1;
  }

  // Stop bit (HIGH)
  digitalWrite(PIN_TX, HIGH);
  bb_wait_bit();

  interrupts();
}

// Blocking receive of 1 byte (returns true if received within timeout_us)
bool bb_rx_byte(uint8_t &out, uint32_t timeout_us = 200000) {
  uint32_t t0 = micros();

  // Wait for start bit (line goes LOW)
  //noInterrupts();
  delayMicroseconds(BIT_US + BIT_US / 2);
  while (digitalRead(PIN_RX) == HIGH) {
    if ((micros() - t0) > timeout_us) return false;
  }
  // interrupts();

  // Align to middle of first data bit: 1.5 bit times from start edge
  delayMicroseconds(BIT_US + BIT_US / 2);

  uint8_t b = 0;
  for (int i = 0; i < 8; i++) {
    uint8_t bit = digitalRead(PIN_RX) ? 1 : 0;
    b |= (bit << i);
    bb_wait_bit();
  }

  // Stop bit time (optional sample)
  bb_wait_bit();

  out = b;
  return true;
}

uint8_t checksum_xor(const uint8_t *buf, size_t n) {
  uint8_t x = 0;
  for (size_t i = 0; i < n; i++) x ^= buf[i];
  return x;
}

void send_frame(uint16_t seq, uint16_t cw, uint32_t ct) {
  uint8_t frame[11];
  frame[0] = 0xA5;
  frame[1] = 0x5A;
  frame[2] = (uint8_t)(seq & 0xFF);
  frame[3] = (uint8_t)(seq >> 8);
  frame[4] = (uint8_t)(cw & 0xFF);
  frame[5] = (uint8_t)(cw >> 8);
  frame[6] = (uint8_t)(ct & 0xFF);
  frame[7] = (uint8_t)((ct >> 8) & 0xFF);
  frame[8] = (uint8_t)((ct >> 16) & 0xFF);
  frame[9] = (uint8_t)((ct >> 24) & 0xFF);
  frame[10] = checksum_xor(frame, 10);

  for (size_t i = 0; i < sizeof(frame); i++) {
    bb_tx_byte(frame[i]);
    delay(2);   // << ajout pour debug
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_TX, OUTPUT);
  digitalWrite(PIN_TX, HIGH);  // idle HIGH

  pinMode(PIN_RX, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);  // IHM debub envoie trame

  pinMode(PIN_WAKE_IN, INPUT_PULLUP);  // created; logic later

  Serial.println("XIAO bit-bang link test ready");
}

void loop() {

  static uint16_t seq = 0;

  // Fake counters (replace later with your real aggregated counts)
  uint16_t count_window = seq % 7;
  uint32_t count_total = seq;

  send_frame(seq, count_window, count_total);
  // IHM debub envoie trame
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  // IHM debub envoie trame

  Serial.print("Sent frame seq=");
  Serial.println(seq);

  // Optional: read ACK (expect 'A' then seq LSB/MSB)
  uint8_t b;
  if (bb_rx_byte(b, 50000) && b == 'A') {
    uint8_t lo, hi;
    if (bb_rx_byte(lo, 20000) && bb_rx_byte(hi, 20000)) {
      uint16_t ackSeq = (uint16_t)lo | ((uint16_t)hi << 8);
      Serial.print("ACK seq=");
      Serial.println(ackSeq);
    }
  }

  seq++;
  delay(1000);
}