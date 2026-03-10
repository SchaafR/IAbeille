#include <Arduino.h>

// =====================
// XIAO ESP32S3 Sense
// Bit-bang UART
// TX = D6
// RX = D7
// WAKE IN = D0
// =====================

static const int PIN_TX = D6;
static const int PIN_RX = D7;
static const int PIN_WAKE_IN = D0;

static const uint32_t BAUD = 4800;
static const uint32_t BIT_US = 1000000UL / BAUD;

enum MsgType : uint8_t {
  CMD_RUN   = 0x01,
  RESULT    = 0x02,
  ACK       = 0x03,
  CMD_SLEEP = 0x04,
  ERROR_MSG = 0x05
};

static inline void bb_wait_bit() {
  delayMicroseconds(BIT_US);
}

uint8_t checksum_xor(const uint8_t *buf, size_t n) {
  uint8_t x = 0;
  for (size_t i = 0; i < n; i++) x ^= buf[i];
  return x;
}

void bb_tx_byte(uint8_t b) {
  noInterrupts();

  digitalWrite(PIN_TX, LOW);   // start bit
  bb_wait_bit();

  for (int i = 0; i < 8; i++) {
    digitalWrite(PIN_TX, (b & 0x01) ? HIGH : LOW);
    bb_wait_bit();
    b >>= 1;
  }

  digitalWrite(PIN_TX, HIGH);  // stop bit
  bb_wait_bit();

  interrupts();
}

bool bb_rx_byte(uint8_t &out, uint32_t timeout_us = 500000) {
  uint32_t t0 = micros();

  while (digitalRead(PIN_RX) == HIGH) {
    if ((micros() - t0) > timeout_us) return false;
  }

  noInterrupts();

  delayMicroseconds(BIT_US + BIT_US / 2);

  uint8_t b = 0;
  for (int i = 0; i < 8; i++) {
    b |= (digitalRead(PIN_RX) ? 1 : 0) << i;
    delayMicroseconds(BIT_US);
  }

  delayMicroseconds(BIT_US); // stop bit

  interrupts();

  out = b;
  return true;
}

void send_packet(uint8_t type, const uint8_t *payload, uint8_t len) {
  uint8_t chk = 0;

  bb_tx_byte(0xA5); delay(2); chk ^= 0xA5;
  bb_tx_byte(0x5A); delay(2); chk ^= 0x5A;
  bb_tx_byte(type); delay(2); chk ^= type;
  bb_tx_byte(len);  delay(2); chk ^= len;

  for (uint8_t i = 0; i < len; i++) {
    bb_tx_byte(payload[i]);
    delay(2);
    chk ^= payload[i];
  }

  bb_tx_byte(chk);
  delay(2);
}

bool recv_packet(uint8_t &type, uint8_t *payload, uint8_t &len, uint32_t timeout_us = 500000) {
  uint8_t b;
  uint8_t chk = 0;

  do {
    if (!bb_rx_byte(b, timeout_us)) return false;
  } while (b != 0xA5);
  chk ^= b;

  if (!bb_rx_byte(b, timeout_us)) return false;
  if (b != 0x5A) return false;
  chk ^= b;

  if (!bb_rx_byte(type, timeout_us)) return false;
  chk ^= type;

  if (!bb_rx_byte(len, timeout_us)) return false;
  chk ^= len;

  if (len > 32) return false;

  for (uint8_t i = 0; i < len; i++) {
    if (!bb_rx_byte(payload[i], timeout_us)) return false;
    chk ^= payload[i];
  }

  uint8_t rx_chk;
  if (!bb_rx_byte(rx_chk, timeout_us)) return false;

  return (chk == rx_chk);
}

void send_result_packet(uint8_t request_id) {
  // Valeurs de test
  static uint16_t fake_count_total = 100;

  uint16_t count_window = fake_count_total % 7;
  uint32_t count_total = fake_count_total;
  uint8_t active_tracks = (fake_count_total % 3);
  uint8_t status = 0;

  uint8_t payload[9];
  payload[0] = request_id;
  payload[1] = (uint8_t)(count_window & 0xFF);
  payload[2] = (uint8_t)(count_window >> 8);
  payload[3] = (uint8_t)(count_total & 0xFF);
  payload[4] = (uint8_t)((count_total >> 8) & 0xFF);
  payload[5] = (uint8_t)((count_total >> 16) & 0xFF);
  payload[6] = (uint8_t)((count_total >> 24) & 0xFF);
  payload[7] = active_tracks;
  payload[8] = status;

  send_packet(RESULT, payload, sizeof(payload));

  Serial.print("RESULT sent: req=");
  Serial.print(request_id);
  Serial.print(" cw=");
  Serial.print(count_window);
  Serial.print(" ct=");
  Serial.print(count_total);
  Serial.print(" active=");
  Serial.print(active_tracks);
  Serial.print(" status=");
  Serial.println(status);

  fake_count_total++;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_TX, OUTPUT);
  digitalWrite(PIN_TX, HIGH);

  pinMode(PIN_RX, INPUT_PULLUP);
  pinMode(PIN_WAKE_IN, INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("XIAO ready");
}

void loop() {
  uint8_t type, len;
  uint8_t payload[32];

  if (recv_packet(type, payload, len, 800000)) {
    if (type == CMD_RUN && len >= 1) {
      uint8_t request_id = payload[0];

      Serial.print("CMD_RUN received, req=");
      Serial.println(request_id);

      digitalWrite(LED_BUILTIN, HIGH);

      // Ici plus tard:
      // 1) sortie deep sleep
      // 2) capture camera
      // 3) inference
      // 4) comptage

      delay(300); // simulation travail

      send_result_packet(request_id);

      digitalWrite(LED_BUILTIN, LOW);
    }
    else if (type == ACK && len >= 2) {
      Serial.print("ACK received, req=");
      Serial.print(payload[0]);
      Serial.print(" code=");
      Serial.println(payload[1]);
    }
    else if (type == CMD_SLEEP && len >= 1) {
      Serial.print("CMD_SLEEP received, req=");
      Serial.println(payload[0]);

      // Ici plus tard:
      // entrer en deep sleep
    }
    else {
      Serial.print("Unknown packet type=");
      Serial.println(type);
    }
  }
}