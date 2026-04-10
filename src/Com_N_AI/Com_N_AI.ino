#include <Arduino.h>
#include <BeeGuardAI_Hornet_Bee_Bees_G1V2_inferencing.h>
#include "esp_camera.h"
#include <ctype.h>
#include <string.h>

// Décommente cette ligne pour activer le debug série
//#define DEBUG_XIAO

// =====================================================
// XIAO ESP32S3 Sense - émission continue tant que l'alim existe
// - auto démarrage au boot
// - capture caméra + inference Edge Impulse
// - envoi continu des scores de confiance (%)
// - aucune réception UART
// - aucune commande RUN/SLEEP
// - checksum XOR à la fin de chaque trame
// - arrêt uniquement par coupure d'alimentation externe
// =====================================================

// =====================================================
// Configuration Pins
// =====================================================
static const int PIN_TX = D6;
static const uint32_t BAUD = 4800;
static const uint32_t BIT_US = 1000000UL / BAUD;

// Intervalle entre 2 trames
static const uint32_t LOOP_DELAY_MS = 200;

// =====================================================
// Protocole
// Trame :
// [0]  0xA5
// [1]  0x5A
// [2]  type
// [3]  len
// [4..] payload
// [fin] checksum XOR de tous les octets précédents
// =====================================================
enum MsgType : uint8_t {
  RESULT    = 0x02,
  ERROR_MSG = 0x05
};

enum ErrorCode : uint8_t {
  ERR_NONE            = 0x00,
  ERR_CAMERA_CAPTURE  = 0x01,
  ERR_CLASSIFIER      = 0x02
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
  .pin_d7         = Y9_GPIO_NUM,
  .pin_d6         = Y8_GPIO_NUM,
  .pin_d5         = Y7_GPIO_NUM,
  .pin_d4         = Y6_GPIO_NUM,
  .pin_d3         = Y5_GPIO_NUM,
  .pin_d2         = Y4_GPIO_NUM,
  .pin_d1         = Y3_GPIO_NUM,
  .pin_d0         = Y2_GPIO_NUM,
  .pin_vsync      = VSYNC_GPIO_NUM,
  .pin_href       = HREF_GPIO_NUM,
  .pin_pclk       = PCLK_GPIO_NUM,
  .xclk_freq_hz   = 20000000,
  .ledc_timer     = LEDC_TIMER_0,
  .ledc_channel   = LEDC_CHANNEL_0,
  .pixel_format   = PIXFORMAT_RGB565,
  .frame_size     = FRAMESIZE_QVGA,
  .jpeg_quality   = 12,
  .fb_count       = 1,
  .fb_location    = CAMERA_FB_IN_PSRAM,
  .grab_mode      = CAMERA_GRAB_WHEN_EMPTY,
};

static uint8_t *snapshot = nullptr;
static uint8_t g_frame_id = 0;

// =====================================================
// Structures
// =====================================================
struct Scores {
  uint8_t hornet; // 0..100
  uint8_t bee;    // 0..100
  uint8_t bees;   // 0..100
};

// =====================================================
// Prototypes
// =====================================================
static inline void bb_wait_bit();
void bb_tx_byte(uint8_t b);
void send_packet(uint8_t type, const uint8_t *payload, uint8_t len);

bool capture_to_snapshot_rgb();
int ei_get_data(size_t offset, size_t length, float *out_ptr);

static bool str_ieq(const char *a, const char *b);
static uint8_t score_to_percent(float v);

bool run_inference_and_score(Scores &scores, ei_impulse_result_t &result);
void send_result_frame(uint8_t frame_id);
void send_error_frame(uint8_t frame_id, uint8_t err);
void debugInferenceAndPayload(const Scores &scores, const ei_impulse_result_t &result, const uint8_t *payload, uint8_t len);

// =====================================================
// Bit-bang UART TX only
// =====================================================
static inline void bb_wait_bit() {
  delayMicroseconds(BIT_US);
}

void bb_tx_byte(uint8_t b) {
  noInterrupts();

  digitalWrite(PIN_TX, LOW);
  bb_wait_bit();

  for (int i = 0; i < 8; i++) {
    digitalWrite(PIN_TX, (b & 0x01) ? HIGH : LOW);
    bb_wait_bit();
    b >>= 1;
  }

  digitalWrite(PIN_TX, HIGH);
  bb_wait_bit();

  interrupts();
}

void send_packet(uint8_t type, const uint8_t *payload, uint8_t len) {
  uint8_t chk = 0;

  bb_tx_byte(0xA5); chk ^= 0xA5; delay(2);
  bb_tx_byte(0x5A); chk ^= 0x5A; delay(2);
  bb_tx_byte(type); chk ^= type; delay(2);
  bb_tx_byte(len);  chk ^= len;  delay(2);

  for (uint8_t i = 0; i < len; i++) {
    bb_tx_byte(payload[i]);
    chk ^= payload[i];
    delay(2);
  }

  // checksum XOR final
  bb_tx_byte(chk);
  delay(2);
}

// =====================================================
// Capture image
// =====================================================
bool capture_to_snapshot_rgb() {
  for (int i = 0; i < 3; i++) {
    camera_fb_t *tmp = esp_camera_fb_get();
    if (tmp) esp_camera_fb_return(tmp);
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return false;

  if (fb->format != PIXFORMAT_RGB565) {
    esp_camera_fb_return(fb);
    return false;
  }

  if (!snapshot) {
    snapshot = (uint8_t *)malloc(SNAPSHOT_BYTES);
    if (!snapshot) {
      esp_camera_fb_return(fb);
      return false;
    }
  }

  const float scale_x = (float)fb->width / (float)IN_W;
  const float scale_y = (float)fb->height / (float)IN_H;

  uint8_t *dst = snapshot;

  for (int y = 0; y < IN_H; y++) {
    int src_y = (int)(y * scale_y);
    if (src_y >= (int)fb->height) src_y = fb->height - 1;

    for (int x = 0; x < IN_W; x++) {
      int src_x = (int)(x * scale_x);
      if (src_x >= (int)fb->width) src_x = fb->width - 1;

      size_t px_off = ((size_t)src_y * (size_t)fb->width + (size_t)src_x) * 2;
      if (px_off + 1 >= fb->len) {
        esp_camera_fb_return(fb);
        return false;
      }

      uint16_t p = (uint16_t)fb->buf[px_off] | ((uint16_t)fb->buf[px_off + 1] << 8);

      *dst++ = (uint8_t)(((p >> 11) & 0x1F) * 255 / 31);
      *dst++ = (uint8_t)(((p >> 5)  & 0x3F) * 255 / 63);
      *dst++ = (uint8_t)(((p >> 0)  & 0x1F) * 255 / 31);
    }
  }

  esp_camera_fb_return(fb);
  return true;
}

int ei_get_data(size_t offset, size_t length, float *out_ptr) {
  for (size_t i = 0; i < length; i++) {
    out_ptr[i] = (float)snapshot[offset + i] - 128.0f;
  }
  return 0;
}

// =====================================================
// Helpers
// =====================================================
static bool str_ieq(const char *a, const char *b) {
  while (*a && *b) {
    if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return false;
    a++;
    b++;
  }
  return (*a == '\0' && *b == '\0');
}

static uint8_t score_to_percent(float v) {
  if (v < 0.0f) v = 0.0f;
  if (v > 1.0f) v = 1.0f;
  return (uint8_t)(v * 100.0f + 0.5f);
}

// =====================================================
// Debug
// =====================================================
void debugInferenceAndPayload(const Scores &scores, const ei_impulse_result_t &result, const uint8_t *payload, uint8_t len) {
#ifndef DEBUG_XIAO
  return;
#endif

  Serial.println("========== DEBUG XIAO ==========");
  Serial.println("[INFERENCE]");
  Serial.print("Bounding boxes count = ");
  Serial.println(result.bounding_boxes_count);

  for (size_t i = 0; i < result.bounding_boxes_count; i++) {
    const auto &bb = result.bounding_boxes[i];
    if (bb.value == 0) continue;

    Serial.print(" - ");
    Serial.print(bb.label);
    Serial.print(" | score=");
    Serial.print(bb.value, 3);
    Serial.print(" | percent=");
    Serial.print(score_to_percent(bb.value));
    Serial.print("%");
    Serial.print(" | x=");
    Serial.print(bb.x);
    Serial.print(" y=");
    Serial.print(bb.y);
    Serial.print(" w=");
    Serial.print(bb.width);
    Serial.print(" h=");
    Serial.println(bb.height);
  }

  Serial.println("[SCORES]");
  Serial.print("Hornet = ");
  Serial.print(scores.hornet);
  Serial.println("%");
  Serial.print("Bee    = ");
  Serial.print(scores.bee);
  Serial.println("%");
  Serial.print("Bees   = ");
  Serial.print(scores.bees);
  Serial.println("%");

  Serial.println("[PAYLOAD]");
  for (uint8_t i = 0; i < len; i++) {
    Serial.print(payload[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println("================================");
}

// =====================================================
// Inference
// =====================================================
bool run_inference_and_score(Scores &scores, ei_impulse_result_t &result) {
  if (!capture_to_snapshot_rgb()) {
    return false;
  }

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_NN_INPUT_FRAME_SIZE;
  signal.get_data = &ei_get_data;

  result = {0};

  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
  if (err != EI_IMPULSE_OK) {
    return false;
  }

  scores.hornet = 0;
  scores.bee = 0;
  scores.bees = 0;

  for (size_t i = 0; i < result.bounding_boxes_count; i++) {
    const auto &bb = result.bounding_boxes[i];
    if (bb.value == 0) continue;

    uint8_t percent = score_to_percent(bb.value);

    if (str_ieq(bb.label, "Hornet")) {
      if (percent > scores.hornet) scores.hornet = percent;
    }
    else if (str_ieq(bb.label, "Bee")) {
      if (percent > scores.bee) scores.bee = percent;
    }
    else if (str_ieq(bb.label, "Bees")) {
      if (percent > scores.bees) scores.bees = percent;
    }
  }

  return true;
}

// =====================================================
// Emission trames
// =====================================================
void send_result_frame(uint8_t frame_id) {
  digitalWrite(LED_BUILTIN, HIGH);

  Scores scores;
  ei_impulse_result_t result = {0};

  bool ok = run_inference_and_score(scores, result);
  if (!ok) {
    send_error_frame(frame_id, ERR_CLASSIFIER);
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }

  uint8_t payload[4] = {
    frame_id,
    scores.hornet,
    scores.bee,
    scores.bees
  };

  debugInferenceAndPayload(scores, result, payload, 4);
  send_packet(RESULT, payload, 4);

  (LED_BUILTIN, LOW);
}

void send_error_frame(uint8_t frame_id, uint8_t err) {
  uint8_t payload[2] = {
    frame_id,
    err
  };
  send_packet(ERROR_MSG, payload, 2);
}

// =====================================================
// Setup & Loop
// =====================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_TX, OUTPUT);
  digitalWrite(PIN_TX, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  if (esp_camera_init(&camera_config) != ESP_OK) {
    send_error_frame(g_frame_id++, ERR_CAMERA_CAPTURE);
    while (true) {
      delay(1000);
    }
  }

  delay(300);
}

void loop() {
  send_result_frame(g_frame_id++);
  delay(LOOP_DELAY_MS);
}