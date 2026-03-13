#include <Arduino.h>
#include <BeeGuardAI_Hornet_Bee_Bees_BRAML_inferencing.h>
#include "esp_camera.h"
#include <ctype.h>

// =====================================================
// XIAO ESP32S3 Sense
// Half-duplex bit-bang UART
// TX = D6
// RX = D7
// WAKE IN = D0
// =====================================================

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

enum ErrorCode : uint8_t {
  ERR_NONE            = 0x00,
  ERR_CAMERA_CAPTURE  = 0x01,
  ERR_CLASSIFIER      = 0x02,
  ERR_BAD_PACKET      = 0x03
};

// =====================================================
// Camera config - XIAO ESP32S3 Sense OV2640
// =====================================================
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1

#define XCLK_GPIO_NUM   10
#define SIOD_GPIO_NUM   40
#define SIOC_GPIO_NUM   39

#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11
#define Y7_GPIO_NUM     12
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     16
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     15

#define VSYNC_GPIO_NUM  38
#define HREF_GPIO_NUM   47
#define PCLK_GPIO_NUM   13

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
// Bit-bang UART
// =====================================================
static inline void bb_wait_bit() {
  delayMicroseconds(BIT_US);
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

  // milieu du premier bit de données
  delayMicroseconds(BIT_US + BIT_US / 2);

  uint8_t b = 0;
  for (int i = 0; i < 8; i++) {
    b |= (digitalRead(PIN_RX) ? 1 : 0) << i;
    delayMicroseconds(BIT_US);
  }

  // stop bit
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

bool recv_packet(uint8_t &type, uint8_t *payload, uint8_t &len, uint32_t timeout_us = 800000) {
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

// =====================================================
// Camera / EI helpers
// =====================================================
bool camera_init() {
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed 0x%x\n", err);
    return false;
  }
  return true;
}

bool capture_to_snapshot_rgb() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }

  if (fb->format != PIXFORMAT_RGB565) {
    Serial.printf("Unexpected pixel format: %d\n", fb->format);
    esp_camera_fb_return(fb);
    return false;
  }

  const int cap_w = (int)fb->width;
  const int cap_h = (int)fb->height;

  const size_t expected = (size_t)cap_w * (size_t)cap_h * 2;
  if (fb->len < expected) {
    Serial.printf("Frame len too small: %u (expected >= %u)\n",
                  (unsigned)fb->len, (unsigned)expected);
    esp_camera_fb_return(fb);
    return false;
  }

  if (!snapshot) {
    snapshot = (uint8_t *)malloc(SNAPSHOT_BYTES);
    if (!snapshot) {
      Serial.println("Out of memory allocating snapshot");
      esp_camera_fb_return(fb);
      return false;
    }
  }

  const float scale_x = (float)cap_w / (float)IN_W;
  const float scale_y = (float)cap_h / (float)IN_H;

  const uint8_t *src = fb->buf;
  uint8_t *dst = snapshot;

  for (int y = 0; y < IN_H; y++) {
    int src_y = (int)(y * scale_y);
    if (src_y >= cap_h) src_y = cap_h - 1;

    for (int x = 0; x < IN_W; x++) {
      int src_x = (int)(x * scale_x);
      if (src_x >= cap_w) src_x = cap_w - 1;

      const size_t px_off = ((size_t)src_y * (size_t)cap_w + (size_t)src_x) * 2;

      if (px_off + 1 >= fb->len) {
        Serial.println("OOB read prevented");
        esp_camera_fb_return(fb);
        return false;
      }

      const uint16_t p = (uint16_t)src[px_off] | ((uint16_t)src[px_off + 1] << 8);

      const uint8_t r = (uint8_t)(((p >> 11) & 0x1F) * 255 / 31);
      const uint8_t g = (uint8_t)(((p >> 5)  & 0x3F) * 255 / 63);
      const uint8_t b = (uint8_t)(((p >> 0)  & 0x1F) * 255 / 31);

      *dst++ = r;
      *dst++ = g;
      *dst++ = b;
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
// Label helpers
// =====================================================
static bool str_ieq(const char *a, const char *b) {
  while (*a && *b) {
    if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return false;
    a++;
    b++;
  }
  return (*a == '\0' && *b == '\0');
}

struct Counts {
  uint16_t hornet;
  uint16_t bee;
  uint16_t bees;
};

bool run_inference_and_count(Counts &counts);
void process_cmd_run(uint8_t request_id);
void send_result_packet(uint8_t request_id, uint16_t hornet, uint16_t bee, uint16_t bees, uint8_t status);
void send_error_packet(uint8_t request_id, uint8_t error_code);

bool run_inference_and_count(Counts &counts) {
  if (!capture_to_snapshot_rgb()) {
    return false;
  }

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_NN_INPUT_FRAME_SIZE;
  signal.get_data = &ei_get_data;

  ei_impulse_result_t result = { 0 };

  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
  if (err != EI_IMPULSE_OK) {
    Serial.print("run_classifier error: ");
    Serial.println((int)err);
    return false;
  }

  counts.hornet = 0;
  counts.bee = 0;
  counts.bees = 0;

  Serial.println("Inference result:");

  if (result.bounding_boxes_count == 0) {
    Serial.println("No objects found");
    return true;
  }

  for (size_t i = 0; i < result.bounding_boxes_count; i++) {
    auto bb = result.bounding_boxes[i];
    if (bb.value == 0) continue;

    Serial.print(" - ");
    Serial.print(bb.label);
    Serial.print(" (");
    Serial.print(bb.value, 3);
    Serial.print(") x=");
    Serial.print(bb.x);
    Serial.print(" y=");
    Serial.print(bb.y);
    Serial.print(" w=");
    Serial.print(bb.width);
    Serial.print(" h=");
    Serial.println(bb.height);

    if (str_ieq(bb.label, "Hornet")) {
      counts.hornet++;
    }
    else if (str_ieq(bb.label, "Bee")) {
      counts.bee++;
    }
    else if (str_ieq(bb.label, "Bees")) {
      counts.bees++;
    }
  }

  Serial.print("Counts => Hornet=");
  Serial.print(counts.hornet);
  Serial.print(" Bee=");
  Serial.print(counts.bee);
  Serial.print(" Bees=");
  Serial.println(counts.bees);

  return true;
}

// =====================================================
// Protocol helpers
// =====================================================
void send_result_packet(uint8_t request_id, uint16_t hornet, uint16_t bee, uint16_t bees, uint8_t status) {
  uint8_t payload[8];

  payload[0] = request_id;

  payload[1] = (uint8_t)(hornet & 0xFF);
  payload[2] = (uint8_t)((hornet >> 8) & 0xFF);

  payload[3] = (uint8_t)(bee & 0xFF);
  payload[4] = (uint8_t)((bee >> 8) & 0xFF);

  payload[5] = (uint8_t)(bees & 0xFF);
  payload[6] = (uint8_t)((bees >> 8) & 0xFF);

  payload[7] = status;

  send_packet(RESULT, payload, sizeof(payload));

  Serial.print("RESULT sent, req=");
  Serial.print(request_id);
  Serial.print(" hornet=");
  Serial.print(hornet);
  Serial.print(" bee=");
  Serial.print(bee);
  Serial.print(" bees=");
  Serial.print(bees);
  Serial.print(" status=");
  Serial.println(status);
}

void send_error_packet(uint8_t request_id, uint8_t error_code) {
  uint8_t payload[2] = { request_id, error_code };
  send_packet(ERROR_MSG, payload, sizeof(payload));

  Serial.print("ERROR_MSG sent, req=");
  Serial.print(request_id);
  Serial.print(" err=");
  Serial.println(error_code);
}

void process_cmd_run(uint8_t request_id) {
  Serial.print("CMD_RUN received, req=");
  Serial.println(request_id);

  digitalWrite(LED_BUILTIN, HIGH);

  Counts counts;
  bool ok = run_inference_and_count(counts);

  if (ok) {
    send_result_packet(request_id, counts.hornet, counts.bee, counts.bees, 0);
  } else {
    send_error_packet(request_id, ERR_CLASSIFIER);
  }

  digitalWrite(LED_BUILTIN, LOW);
}

// =====================================================
// Setup / Loop
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_TX, OUTPUT);
  digitalWrite(PIN_TX, HIGH);

  pinMode(PIN_RX, INPUT_PULLUP);
  pinMode(PIN_WAKE_IN, INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("XIAO ready");

  if (!camera_init()) {
    Serial.println("Camera init failed");
    while (1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
    }
  }

  Serial.print("Model input: ");
  Serial.print(IN_W);
  Serial.print("x");
  Serial.print(IN_H);
  Serial.print("x");
  Serial.println(IN_CH);
}

void loop() {
  uint8_t type, len;
  uint8_t payload[32];

  if (recv_packet(type, payload, len, 800000)) {
    if (type == CMD_RUN && len >= 1) {
      uint8_t request_id = payload[0];
      process_cmd_run(request_id);
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

      // Placeholder pour plus tard :
      // mettre la caméra en veille / deep sleep ESP32S3
    }
    else {
      Serial.print("Unknown packet type=");
      Serial.print(type);
      Serial.print(" len=");
      Serial.println(len);
    }
  }
}

