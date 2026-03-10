#include <Arduino.h>

// =====================
// Arduino Nano 33 BLE
// Bit-bang UART
// RX = D5
// TX = D6
// WAKE OUT = D3
// =====================

static const int PIN_RX = D5;
static const int PIN_TX = D6;
static const int PIN_WAKE_OUT = D3;

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

bool recv_packet(uint8_t &type, uint8_t *payload, uint8_t &len, uint32_t timeout_us = 1000000) {
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

void send_cmd_run(uint8_t request_id) {
  uint8_t payload[1] = { request_id };
  send_packet(CMD_RUN, payload, 1);

  Serial.print("CMD_RUN sent, req=");
  Serial.println(request_id);
}

void send_ack(uint8_t request_id, uint8_t code) {
  uint8_t payload[2] = { request_id, code };
  send_packet(ACK, payload, 2);

  Serial.print("ACK sent, req=");
  Serial.print(request_id);
  Serial.print(" code=");
  Serial.println(code);
}

void send_cmd_sleep(uint8_t request_id) {
  uint8_t payload[1] = { request_id };
  send_packet(CMD_SLEEP, payload, 1);

  Serial.print("CMD_SLEEP sent, req=");
  Serial.println(request_id);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(PIN_RX, INPUT_PULLUP);

  pinMode(PIN_TX, OUTPUT);
  digitalWrite(PIN_TX, HIGH);

  pinMode(PIN_WAKE_OUT, OUTPUT);
  digitalWrite(PIN_WAKE_OUT, LOW);

  Serial.println("Nano BLE ready");
}

void loop() {
  static uint8_t request_id = 1;

  // Signal de réveil pour plus tard (placeholder)
  digitalWrite(PIN_WAKE_OUT, HIGH);
  delay(10);

  // 1) Demander à la XIAO d'exécuter une tâche
  send_cmd_run(request_id);

  // 2) Attendre le résultat
  uint8_t type, len;
  uint8_t payload[32];

  bool got_result = false;
  uint32_t t0 = millis();

  while ((millis() - t0) < 3000) {
    if (recv_packet(type, payload, len, 800000)) {
      if (type == RESULT && len >= 9) {
        uint8_t req = payload[0];
        uint16_t count_window = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
        uint32_t count_total =
            (uint32_t)payload[3] |
            ((uint32_t)payload[4] << 8) |
            ((uint32_t)payload[5] << 16) |
            ((uint32_t)payload[6] << 24);
        uint8_t active_tracks = payload[7];
        uint8_t status = payload[8];

        Serial.print("RESULT received, req=");
        Serial.print(req);
        Serial.print(" cw=");
        Serial.print(count_window);
        Serial.print(" ct=");
        Serial.print(count_total);
        Serial.print(" active=");
        Serial.print(active_tracks);
        Serial.print(" status=");
        Serial.println(status);

        // 3) ACK
        send_ack(req, 0);

        delay(50);

        // 4) Commande sleep
        send_cmd_sleep(req);

        got_result = true;
        break;
      }
    }
  }

  // Fin placeholder réveil
  digitalWrite(PIN_WAKE_OUT, LOW);

  if (!got_result) {
    Serial.print("Timeout waiting RESULT for req=");
    Serial.println(request_id);
  }

  request_id++;
  delay(3000);
}