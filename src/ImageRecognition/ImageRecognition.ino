#include <BeeGuardAI_Hornet_Bee_Bees_BRAML_inferencing.h>
#include "esp_camera.h"

//======================
//FOR DEBUG
//======================
// #include "esp_heap_caps.h"

// static void dump_heap(const char *tag) {
//   Serial.printf("[%s] heap free=%u, min=%u, psram free=%u\n",
//     tag,
//     (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
//     (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT),
//     (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM)
//   );
// }

// =====================
// XIAO ESP32S3 Sense (OV2640) camera pins
// =====================
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1

#define XCLK_GPIO_NUM 10
#define SIOD_GPIO_NUM 40
#define SIOC_GPIO_NUM 39

#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 11
#define Y7_GPIO_NUM 12
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 16
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 17
#define Y2_GPIO_NUM 15

#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM 47
#define PCLK_GPIO_NUM 13

// Capture size (QVGA) then crop to model size
static constexpr int CAP_W = 320;
static constexpr int CAP_H = 240;

static constexpr int IN_W = EI_CLASSIFIER_INPUT_WIDTH;   // 96
static constexpr int IN_H = EI_CLASSIFIER_INPUT_HEIGHT;  // 96
static constexpr size_t IN_CH = 3;
static constexpr size_t SNAPSHOT_BYTES = (size_t)IN_W * (size_t)IN_H * IN_CH;

static camera_config_t camera_config = {
  .pin_pwdn = PWDN_GPIO_NUM,
  .pin_reset = RESET_GPIO_NUM,
  .pin_xclk = XCLK_GPIO_NUM,
  .pin_sscb_sda = SIOD_GPIO_NUM,
  .pin_sscb_scl = SIOC_GPIO_NUM,

  .pin_d7 = Y9_GPIO_NUM,
  .pin_d6 = Y8_GPIO_NUM,
  .pin_d5 = Y7_GPIO_NUM,
  .pin_d4 = Y6_GPIO_NUM,
  .pin_d3 = Y5_GPIO_NUM,
  .pin_d2 = Y4_GPIO_NUM,
  .pin_d1 = Y3_GPIO_NUM,
  .pin_d0 = Y2_GPIO_NUM,
  .pin_vsync = VSYNC_GPIO_NUM,
  .pin_href = HREF_GPIO_NUM,
  .pin_pclk = PCLK_GPIO_NUM,

  .xclk_freq_hz = 20000000,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,

  // IMPORTANT: RGB
  .pixel_format = PIXFORMAT_RGB565,

  // Capture QVGA 320x240
  .frame_size = FRAMESIZE_QVGA,

  .jpeg_quality = 12,
  .fb_count = 1,
  .fb_location = CAMERA_FB_IN_PSRAM,
  .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static uint8_t *snapshot = nullptr;  // RGB888 240x240x3

bool camera_init() {
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed 0x%x\n", err);
    return false;
  }
  return true;
}

// Convert one RGB565 pixel to RGB888
static inline void rgb565_to_rgb888(uint16_t p, uint8_t &r, uint8_t &g, uint8_t &b) {
  r = (uint8_t)(((p >> 11) & 0x1F) * 255 / 31);
  g = (uint8_t)(((p >> 5) & 0x3F) * 255 / 63);
  b = (uint8_t)(((p >> 0) & 0x1F) * 255 / 31);
}

bool capture_to_snapshot_rgb() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }

  // On attend du RGB565
  if (fb->format != PIXFORMAT_RGB565) {
    Serial.printf("Unexpected pixel format: %d (expected RGB565)\n", fb->format);
    esp_camera_fb_return(fb);
    return false;
  }

  const int cap_w = (int)fb->width;
  const int cap_h = (int)fb->height;

  // RGB565 => 2 bytes/pixel
  const size_t expected = (size_t)cap_w * (size_t)cap_h * 2;
  if (fb->len < expected) {
    Serial.printf("Frame len too small: %u (expected >= %u)\n",
                  (unsigned)fb->len, (unsigned)expected);
    esp_camera_fb_return(fb);
    return false;
  }

  if (!snapshot) {
    snapshot = (uint8_t*)malloc(SNAPSHOT_BYTES); // IN_W*IN_H*3
    if (!snapshot) {
      Serial.println("Out of memory allocating snapshot");
      esp_camera_fb_return(fb);
      return false;
    }
  }

  // Resize 320x240 → 96x96 (nearest neighbor)
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

      // sécurité anti-OOB (devrait jamais arriver si expected OK)
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

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("Starting XIAO ESP32S3 Sense + FOMO 96x96");

  if (!camera_init()) {
    Serial.println("Camera init failed");
    while (1) delay(100);
  }

  Serial.print("Model input: ");
  Serial.print(IN_W);
  Serial.print("x");
  Serial.print(IN_H);
  Serial.print("x");
  Serial.println(IN_CH);

  Serial.print("DSP input frame size: ");
  Serial.println(EI_CLASSIFIER_NN_INPUT_FRAME_SIZE);
}

  void loop() {
    if (!capture_to_snapshot_rgb()) {
      delay(200);
      return;
    }


    signal_t signal;
    signal.total_length = EI_CLASSIFIER_NN_INPUT_FRAME_SIZE;
    signal.get_data = &ei_get_data;

    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
    if (err != EI_IMPULSE_OK) {
      Serial.print("run_classifier error: ");
      Serial.println(err);
      delay(200);
      return;
    }

    // FOMO: bounding boxes
    if (result.bounding_boxes_count == 0) {
      Serial.println("No objects found");
    } else {
      for (size_t i = 0; i < result.bounding_boxes_count; i++) {
        auto bb = result.bounding_boxes[i];
        if (bb.value == 0) continue;

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
      }
    }

    delay(200);
  }