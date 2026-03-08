#include <Arduino.h>

static const int PIN_RX = D5;
static const uint32_t BAUD = 4800;
static const uint32_t BIT_US = 1000000UL / BAUD;

bool bb_rx_byte(uint8_t &out, uint32_t timeout_us = 500000) {
  uint32_t t0 = micros();

  while (digitalRead(PIN_RX) == HIGH) {
    if (micros() - t0 > timeout_us) return false;
  }

  noInterrupts();

  delayMicroseconds(BIT_US + BIT_US / 2);

  uint8_t b = 0;
  for (int i = 0; i < 8; i++) {
    b |= (digitalRead(PIN_RX) ? 1 : 0) << i;
    delayMicroseconds(BIT_US);
  }

  delayMicroseconds(BIT_US);

  interrupts();

  out = b;
  return true;
}

bool recv_frame(uint8_t *frame) {
  uint8_t b;

  do {
    if (!bb_rx_byte(b)) return false;
  } while (b != 0xA5);

  if (!bb_rx_byte(b)) return false;
  if (b != 0x5A) return false;

  frame[0] = 0xA5;
  frame[1] = 0x5A;

  for (int i = 2; i < 11; i++) {
    if (!bb_rx_byte(frame[i])) return false;
  }

  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  pinMode(PIN_RX, INPUT_PULLUP);
  Serial.println("RX debug ready");
}

void loop() {
  uint8_t frame[11];

  if (recv_frame(frame)) {
    Serial.println("\n====== NEW PACKET ======");

    for (int i = 0; i < 11; i++) {
      if (frame[i] < 16) Serial.print("0");
      Serial.print(frame[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}