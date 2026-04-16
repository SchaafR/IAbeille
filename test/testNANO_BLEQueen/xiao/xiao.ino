#include <Arduino.h>
#include <BeeGuardAI_Hornet_Bee_Bees_BRAML_inferencing.h>
#include "esp_camera.h"
#include <ctype.h>
#include <string.h>

// Décommente cette ligne pour activer le debug
//#define DEBUG_XIAO

// =====================================================
// XIAO ESP32S3 Sense - Mode piloté par Nano 33 BLE
// Fusion:
// - protocole UART bit-bang + deep sleep
// - capture caméra + inference Edge Impulse
// - tracker anti-doublon léger pour Hornet
// - fonction unique de debug pour afficher
//   les inférences et la payload envoyée
// =====================================================

// =====================================================
// Configuration Pins
// =====================================================
static const int PIN_TX = D6;
static const int PIN_RX = D7;
static const int PIN_WAKE_IN = D0; // Reçoit SIGINT de la Nano (D3)

static const uint32_t BAUD = 4800;
static const uint32_t BIT_US = 1000000UL / BAUD;

volatile bool block = true;

// =====================================================
// Protocole
// =====================================================
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

// =====================================================
// Paramètres anti-doublon Hornet
// =====================================================
static constexpr float HORNET_MIN_SCORE = 0.60f;
static constexpr uint32_t TRACK_TTL_MS = 1000;
static constexpr uint8_t CONFIRM_FRAMES = 2;
static constexpr uint8_t MAX_TRACKS = 8;
static constexpr float MATCH_DIST_RATIO = 0.10f;

// Mettre à true si tu veux aussi les logs détaillés du tracker
static constexpr bool ENABLE_DEBUG_TRACK = false;

// =====================================================
// Structures
// =====================================================
struct Counts {
  uint16_t hornet;
  uint16_t bee;
  uint16_t bees;
};

struct HornetDetection {
  int x;
  int y;
  int w;
  int h;
  float score;
};

struct HornetTrack {
  bool active;
  bool counted;
  int cx;
  int cy;
  int w;
  int h;
  uint8_t seen_frames;
  uint32_t first_seen_ms;
  uint32_t last_seen_ms;
};

static HornetTrack g_tracks[MAX_TRACKS];
static uint32_t g_hornet_event_count = 0;

// =====================================================
// Prototypes
// =====================================================
static inline void bb_wait_bit();
void bb_tx_byte(uint8_t b);
bool bb_rx_byte(uint8_t &out, uint32_t timeout_us = 500000);
void send_packet(uint8_t type, const uint8_t *payload, uint8_t len);
bool recv_packet(uint8_t &type, uint8_t *payload, uint8_t &len, uint32_t timeout_us = 800000);

void allerAuSommeil();

bool capture_to_snapshot_rgb();
int ei_get_data(size_t offset, size_t length, float *out_ptr);

static bool str_ieq(const char *a, const char *b);

static inline int center_x(int x, int w);
static inline int center_y(int y, int h);
static inline int match_dist_px();

void reset_all_tracks();
void expire_old_tracks(uint32_t now_ms);
int find_best_track_for_detection(const HornetDetection &det, const bool used_tracks[]);
int allocate_track(const HornetDetection &det, uint32_t now_ms);
void update_track(int idx, const HornetDetection &det, uint32_t now_ms);

bool run_inference_and_count(Counts &counts, ei_impulse_result_t &result);
void process_cmd_run(uint8_t req_id);
void debugInferenceAndPayload(const Counts &counts, const ei_impulse_result_t &result, const uint8_t *payload, uint8_t len);

// =====================================================
// Bit-bang UART
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

bool bb_rx_byte(uint8_t &out, uint32_t timeout_us) {
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
    bb_tx_byte(payload[i]);
    delay(2);
    chk ^= payload[i];
  }

  bb_tx_byte(chk);
  delay(2);
}

bool recv_packet(uint8_t &type, uint8_t *payload, uint8_t &len, uint32_t timeout_us) {
  uint8_t b, chk = 0;

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

// =====================================================
// Sommeil
// =====================================================
void allerAuSommeil() {
  esp_camera_deinit();
  delay(100);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_WAKE_IN, 1);
  Serial.flush();
  esp_deep_sleep_start();
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

static inline int center_x(int x, int w) {
  return x + (w / 2);
}

static inline int center_y(int y, int h) {
  return y + (h / 2);
}

static inline int match_dist_px() {
  int m = (IN_W < IN_H) ? IN_W : IN_H;
  int d = (int)(m * MATCH_DIST_RATIO);
  if (d < 6) d = 6;
  return d;
}

// =====================================================
// Tracker anti-doublon
// =====================================================
void reset_all_tracks() {
  for (int i = 0; i < MAX_TRACKS; i++) {
    g_tracks[i].active = false;
    g_tracks[i].counted = false;
    g_tracks[i].cx = 0;
    g_tracks[i].cy = 0;
    g_tracks[i].w = 0;
    g_tracks[i].h = 0;
    g_tracks[i].seen_frames = 0;
    g_tracks[i].first_seen_ms = 0;
    g_tracks[i].last_seen_ms = 0;
  }
}

void expire_old_tracks(uint32_t now_ms) {
  for (int i = 0; i < MAX_TRACKS; i++) {
    if (!g_tracks[i].active) continue;

    if ((now_ms - g_tracks[i].last_seen_ms) > TRACK_TTL_MS) {
      if (ENABLE_DEBUG_TRACK) {
        Serial.print("[TRACK] expired idx=");
        Serial.println(i);
      }
      g_tracks[i].active = false;
    }
  }
}

int find_best_track_for_detection(const HornetDetection &det, const bool used_tracks[]) {
  const int cx = center_x(det.x, det.w);
  const int cy = center_y(det.y, det.h);

  const int max_d = match_dist_px();
  const int max_d2 = max_d * max_d;

  int best_idx = -1;
  int best_d2 = 0x7FFFFFFF;

  for (int i = 0; i < MAX_TRACKS; i++) {
    if (!g_tracks[i].active) continue;
    if (used_tracks[i]) continue;

    int dx = cx - g_tracks[i].cx;
    int dy = cy - g_tracks[i].cy;
    int d2 = dx * dx + dy * dy;

    if (d2 <= max_d2 && d2 < best_d2) {
      best_d2 = d2;
      best_idx = i;
    }
  }

  return best_idx;
}

int allocate_track(const HornetDetection &det, uint32_t now_ms) {
  for (int i = 0; i < MAX_TRACKS; i++) {
    if (!g_tracks[i].active) {
      g_tracks[i].active = true;
      g_tracks[i].counted = false;
      g_tracks[i].cx = center_x(det.x, det.w);
      g_tracks[i].cy = center_y(det.y, det.h);
      g_tracks[i].w = det.w;
      g_tracks[i].h = det.h;
      g_tracks[i].seen_frames = 1;
      g_tracks[i].first_seen_ms = now_ms;
      g_tracks[i].last_seen_ms = now_ms;

      if (ENABLE_DEBUG_TRACK) {
        Serial.print("[TRACK] create idx=");
        Serial.println(i);
      }

      if (!g_tracks[i].counted && CONFIRM_FRAMES <= 1) {
        g_tracks[i].counted = true;
        g_hornet_event_count++;
      }

      return i;
    }
  }

  if (ENABLE_DEBUG_TRACK) {
    Serial.println("[TRACK] no free slot");
  }

  return -1;
}

void update_track(int idx, const HornetDetection &det, uint32_t now_ms) {
  if (idx < 0 || idx >= MAX_TRACKS) return;
  if (!g_tracks[idx].active) return;

  g_tracks[idx].cx = center_x(det.x, det.w);
  g_tracks[idx].cy = center_y(det.y, det.h);
  g_tracks[idx].w = det.w;
  g_tracks[idx].h = det.h;
  g_tracks[idx].last_seen_ms = now_ms;

  if (g_tracks[idx].seen_frames < 255) {
    g_tracks[idx].seen_frames++;
  }

  if (!g_tracks[idx].counted && g_tracks[idx].seen_frames >= CONFIRM_FRAMES) {
    g_tracks[idx].counted = true;
    g_hornet_event_count++;
  }
}

// =====================================================
// Debug unique
// =====================================================
void debugInferenceAndPayload(const Counts &counts, const ei_impulse_result_t &result, const uint8_t *payload, uint8_t len) {
#ifndef DEBUG_XIAO
  return;
#endif

  Serial.println("========== DEBUG XIAO ==========");

  Serial.println("[INFERENCE]");
  Serial.print("Bounding boxes count = ");
  Serial.println(result.bounding_boxes_count);

  if (result.bounding_boxes_count == 0) {
    Serial.println("No objects found");
  } else {
    for (size_t i = 0; i < result.bounding_boxes_count; i++) {
      const auto &bb = result.bounding_boxes[i];
      if (bb.value == 0) continue;

      Serial.print(" - ");
      Serial.print(bb.label);
      Serial.print(" | score=");
      Serial.print(bb.value, 3);
      Serial.print(" | x=");
      Serial.print(bb.x);
      Serial.print(" y=");
      Serial.print(bb.y);
      Serial.print(" w=");
      Serial.print(bb.width);
      Serial.print(" h=");
      Serial.println(bb.height);
    }
  }

  Serial.println("[COUNTS]");
  Serial.print("Hornet frame count = ");
  Serial.println(counts.hornet);
  Serial.print("Bee frame count    = ");
  Serial.println(counts.bee);
  Serial.print("Bees frame count   = ");
  Serial.println(counts.bees);
  Serial.print("Hornet event total = ");
  Serial.println(g_hornet_event_count);

  Serial.println("[PAYLOAD SENT]");
  Serial.print("Length = ");
  Serial.println(len);

  Serial.print("HEX : ");
  for (uint8_t i = 0; i < len; i++) {
    if (payload[i] < 0x10) Serial.print("0");
    Serial.print(payload[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("DEC : ");
  for (uint8_t i = 0; i < len; i++) {
    Serial.print(payload[i]);
    Serial.print(" ");
  }
  Serial.println();

  Serial.println("================================");
}

// =====================================================
// Inference + comptage
// =====================================================
bool run_inference_and_count(Counts &counts, ei_impulse_result_t &result) {
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

  counts.hornet = 0;
  counts.bee = 0;
  counts.bees = 0;

  const uint32_t now_ms = millis();
  expire_old_tracks(now_ms);

  bool used_tracks[MAX_TRACKS];
  for (int i = 0; i < MAX_TRACKS; i++) {
    used_tracks[i] = false;
  }

  for (size_t i = 0; i < result.bounding_boxes_count; i++) {
    auto bb = result.bounding_boxes[i];

    if (bb.value == 0) continue;

    if (str_ieq(bb.label, "Hornet")) {
      counts.hornet++;

      if (bb.value < HORNET_MIN_SCORE) {
        continue;
      }

      HornetDetection det;
      det.x = bb.x;
      det.y = bb.y;
      det.w = bb.width;
      det.h = bb.height;
      det.score = bb.value;

      int track_idx = find_best_track_for_detection(det, used_tracks);

      if (track_idx >= 0) {
        update_track(track_idx, det, now_ms);
        used_tracks[track_idx] = true;
      } else {
        int new_idx = allocate_track(det, now_ms);
        if (new_idx >= 0) {
          used_tracks[new_idx] = true;
        }
      }
    }
    else if (str_ieq(bb.label, "Bee")) {
      counts.bee++;
    }
    else if (str_ieq(bb.label, "Bees")) {
      counts.bees++;
    }
  }

  return true;
}

// =====================================================
// Traitement commande RUN
// =====================================================
void process_cmd_run(uint8_t req_id) {
  digitalWrite(LED_BUILTIN, HIGH);

  Counts counts;
  ei_impulse_result_t result = {0};

  bool ok = run_inference_and_count(counts, result);

  if (!ok) {
    uint8_t err_payload[2] = { req_id, ERR_CLASSIFIER };
    send_packet(ERROR_MSG, err_payload, 2);
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }

  // [0] req_id
  // [1..2] hornet frame count (LE)
  // [3..4] bee frame count (LE)
  // [5..6] bees frame count (LE)
  // [7] hornet event count anti-doublon (LSB only)
  uint8_t res[8] = {
    req_id,
    (uint8_t)(counts.hornet & 0xFF),
    (uint8_t)((counts.hornet >> 8) & 0xFF),
    (uint8_t)(counts.bee & 0xFF),
    (uint8_t)((counts.bee >> 8) & 0xFF),
    (uint8_t)(counts.bees & 0xFF),
    (uint8_t)((counts.bees >> 8) & 0xFF),
    (uint8_t)(g_hornet_event_count & 0xFF)
  };

  debugInferenceAndPayload(counts, result, res, 8);
  send_packet(RESULT, res, 8);

  digitalWrite(LED_BUILTIN, LOW);
}

// =====================================================
// Setup & Loop
// =====================================================

void IRAM_ATTR monInterruption() {
  block = false;
}

void setup() {
  Serial.begin(115200);

  // VERSION 2 : Deep sleep + ext0 wakeup
  //esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_WAKE_IN, 1);
  // 2. Au lieu du while(block), on dort
  // La consommation tombe à ~2 mA
  //esp_light_sleep_start();

  // VERSION 1 : Busy wait sur interruption
  //attachInterrupt(digitalPinToInterrupt(PIN_WAKE_IN), monInterruption, RISING);
  //while(block);

  pinMode(PIN_TX, OUTPUT);
  digitalWrite(PIN_TX, HIGH);

  pinMode(PIN_RX, INPUT_PULLUP);
  pinMode(PIN_WAKE_IN, INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  reset_all_tracks();

  if (esp_camera_init(&camera_config) != ESP_OK) {
    while(1){
      delay(2000);
    }
  }
}

void loop() {
  uint8_t type, len, payload[32];

  if (recv_packet(type, payload, len, 5000000)) {
    if (type == CMD_RUN && len >= 1) {
      process_cmd_run(payload[0]);
    }
    else if (type == CMD_SLEEP) {
      esp_reset();
    }
    else {
      uint8_t err_payload[2] = { 0, ERR_BAD_PACKET };
      send_packet(ERROR_MSG, err_payload, 2);
    }
  }

  if (millis() > 15000) {
    esp_reset();
  }
}