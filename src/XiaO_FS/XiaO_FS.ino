#include <Arduino.h>
#include <BeeGuardAI_Hornet_Bee_Bees_BRAML_inferencing.h>
#include "esp_camera.h"
#include <ctype.h>

// =====================================================
// XIAO ESP32S3 Sense - Configuration Pins
// =====================================================
static const int PIN_TX = D6;
static const int PIN_RX = D7;
static const int PIN_WAKE_IN = D0; // Reçoit SIGINT de la Nano (D3)

static const uint32_t BAUD = 4800;
static const uint32_t BIT_US = 1000000UL / BAUD;

// Protocole
enum MsgType : uint8_t {
  CMD_RUN   = 0x01,
  RESULT    = 0x02,
  ACK       = 0x03,
  CMD_SLEEP = 0x04,
  ERROR_MSG = 0x05
};

enum ErrorCode : uint8_t {
  ERR_NONE            = 0x00,
  ERR_CAMERA_CAPTURE  = 0x01,
  ERR_CLASSIFIER      = 0x02,
  ERR_BAD_PACKET      = 0x03
};

// =====================================================
// Configuration Caméra OV2640
// =====================================================
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    10
#define SIOD_GPIO_NUM    40
#define SIOC_GPIO_NUM    39
#define Y9_GPIO_NUM      48
#define Y8_GPIO_NUM      11
#define Y7_GPIO_NUM      12
#define Y6_GPIO_NUM      14
#define Y5_GPIO_NUM      16
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM      17
#define Y2_GPIO_NUM      15
#define VSYNC_GPIO_NUM   38
#define HREF_GPIO_NUM    47
#define PCLK_GPIO_NUM    13

static constexpr int IN_W = EI_CLASSIFIER_INPUT_WIDTH;
static constexpr int IN_H = EI_CLASSIFIER_INPUT_HEIGHT;
static constexpr size_t IN_CH = 3;
static constexpr size_t SNAPSHOT_BYTES = (size_t)IN_W * (size_t)IN_H * IN_CH;

static camera_config_t camera_config = {
  .pin_pwdn       = PWDN_GPIO_NUM,
  .pin_reset      = RESET_GPIO_NUM,
  .pin_xclk       = XCLK_GPIO_NUM,
  .pin_sscb_sda   = SIOD_GPIO_NUM,
  .pin_sscb_scl   = SIOC_GPIO_NUM,
  .pin_d7          = Y9_GPIO_NUM,
  .pin_d6          = Y8_GPIO_NUM,
  .pin_d5          = Y7_GPIO_NUM,
  .pin_d4          = Y6_GPIO_NUM,
  .pin_d3          = Y5_GPIO_NUM,
  .pin_d2          = Y4_GPIO_NUM,
  .pin_d1          = Y3_GPIO_NUM,
  .pin_d0          = Y2_GPIO_NUM,
  .pin_vsync       = VSYNC_GPIO_NUM,
  .pin_href        = HREF_GPIO_NUM,
  .pin_pclk        = PCLK_GPIO_NUM,
  .xclk_freq_hz    = 20000000,
  .ledc_timer      = LEDC_TIMER_0,
  .ledc_channel    = LEDC_CHANNEL_0,
  .pixel_format    = PIXFORMAT_RGB565,
  .frame_size      = FRAMESIZE_QVGA,
  .jpeg_quality    = 12,
  .fb_count        = 1,
  .fb_location     = CAMERA_FB_IN_PSRAM,
  .grab_mode       = CAMERA_GRAB_WHEN_EMPTY,
};

static uint8_t *snapshot = nullptr;

// =====================================================
// Bit-bang UART (Fonctions de base)
// =====================================================
static inline void bb_wait_bit() { delayMicroseconds(BIT_US); }

void bb_tx_byte(uint8_t b) {
  noInterrupts();
  digitalWrite(PIN_TX, LOW); bb_wait_bit();
  for (int i = 0; i < 8; i++) {
    digitalWrite(PIN_TX, (b & 0x01) ? HIGH : LOW);
    bb_wait_bit();
    b >>= 1;
  }
  digitalWrite(PIN_TX, HIGH); bb_wait_bit();
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
  delayMicroseconds(BIT_US);
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
    bb_tx_byte(payload[i]); delay(2); chk ^= payload[i];
  }
  bb_tx_byte(chk); delay(2);
}

bool recv_packet(uint8_t &type, uint8_t *payload, uint8_t &len, uint32_t timeout_us = 800000) {
  uint8_t b, chk = 0;
  uint32_t tStart = micros();
  do { if (!bb_rx_byte(b, timeout_us)) return false; } while (b != 0xA5);
  chk ^= b;
  if (!bb_rx_byte(b, timeout_us) || b != 0x5A) return false; chk ^= b;
  if (!bb_rx_byte(type, timeout_us)) return false; chk ^= type;
  if (!bb_rx_byte(len, timeout_us)) return false; chk ^= len;
  if (len > 32) return false;
  for (uint8_t i = 0; i < len; i++) {
    if (!bb_rx_byte(payload[i], timeout_us)) return false;
    chk ^= payload[i];
  }
  uint8_t rx_chk;
  if (!bb_rx_byte(rx_chk, timeout_us)) return false;
  return (chk == rx_chk);
}

// =====================================================
// Gestion Energie & Sommeil
// =====================================================
void allerAuSommeil() {
  //Serial.println(">>> Entrée en Deep Sleep (Réveil via D0)");
  esp_camera_deinit(); // Libère les ressources caméra
  delay(100);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_WAKE_IN, 1); // Réveil sur signal HIGH
  Serial.flush();
  esp_deep_sleep_start();
}

// =====================================================
// Logique IA & Caméra
// =====================================================
bool capture_to_snapshot_rgb() {
  // On vide les buffers pour avoir une image fraîche (AEC/AGC)
  for(int i=0; i<3; i++) {
    camera_fb_t *tmp = esp_camera_fb_get();
    if(tmp) esp_camera_fb_return(tmp);
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return false;

  if (!snapshot) snapshot = (uint8_t *)malloc(SNAPSHOT_BYTES);
  if (!snapshot) { esp_camera_fb_return(fb); return false; }

  // Redimensionnement simple (Nearest Neighbor) vers format Edge Impulse
  float scale_x = (float)fb->width / (float)IN_W;
  float scale_y = (float)fb->height / (float)IN_H;
  uint8_t *dst = snapshot;

  for (int y = 0; y < IN_H; y++) {
    int src_y = (int)(y * scale_y);
    for (int x = 0; x < IN_W; x++) {
      int src_x = (int)(x * scale_x);
      size_t px_off = (src_y * fb->width + src_x) * 2;
      uint16_t p = (uint16_t)fb->buf[px_off] | ((uint16_t)fb->buf[px_off + 1] << 8);
      *dst++ = (uint8_t)(((p >> 11) & 0x1F) * 255 / 31); // R
      *dst++ = (uint8_t)(((p >> 5)  & 0x3F) * 255 / 63); // G
      *dst++ = (uint8_t)(((p >> 0)  & 0x1F) * 255 / 31); // B
    }
  }
  esp_camera_fb_return(fb);
  return true;
}

int ei_get_data(size_t offset, size_t length, float *out_ptr) {
  for (size_t i = 0; i < length; i++) out_ptr[i] = (float)snapshot[offset + i] - 128.0f;
  return 0;
}

void process_cmd_run(uint8_t req_id) {
  digitalWrite(LED_BUILTIN, HIGH);
  if (!capture_to_snapshot_rgb()) {
    send_packet(ERROR_MSG, (uint8_t[]){req_id, ERR_CAMERA_CAPTURE}, 2);
    return;
  }

  signal_t signal = { .get_data = &ei_get_data, .total_length = SNAPSHOT_BYTES };
  ei_impulse_result_t result = {0};
  if (run_classifier(&signal, &result, false) != EI_IMPULSE_OK) {
    send_packet(ERROR_MSG, (uint8_t[]){req_id, ERR_CLASSIFIER}, 2);
    return;
  }

  uint16_t h=0, b=0, bs=0;
  for (size_t i = 0; i < result.bounding_boxes_count; i++) {
    auto bb = result.bounding_boxes[i];
    if (bb.value < 0.5) continue;
    if (strstr(bb.label, "Hornet")) h++;
    else if (strstr(bb.label, "Bees")) bs++;
    else if (strstr(bb.label, "Bee")) b++;
  }

  // Envoi résultat (Little Endian)
  uint8_t res[8] = { req_id, (uint8_t)(h&0xFF), (uint8_t)(h>>8), (uint8_t)(b&0xFF), (uint8_t)(b>>8), (uint8_t)(bs&0xFF), (uint8_t)(bs>>8), 0 };
  send_packet(RESULT, res, 8);
  digitalWrite(LED_BUILTIN, LOW);
}

// =====================================================
// Setup & Loop
// =====================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_TX, OUTPUT); digitalWrite(PIN_TX, HIGH);
  pinMode(PIN_RX, INPUT_PULLUP);
  pinMode(PIN_WAKE_IN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  if (esp_camera_init(&camera_config) != ESP_OK) allerAuSommeil();
  //Serial.println("XIAO Awake & Ready");
}

void loop() {
  uint8_t type, len, payload[32];
  if (recv_packet(type, payload, len, 5000000)) { // Timeout 5s
    if (type == CMD_RUN) process_cmd_run(payload[0]);
    else if (type == CMD_SLEEP) allerAuSommeil();
  }

  // Sécurité : si la Nano ne donne pas d'ordre pendant 15s, dodo.
  if (millis() > 15000) allerAuSommeil();
}