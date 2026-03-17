#include <BeeGuardAI_Hornet_Bee_Bees_G1_V3_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#include "esp_camera.h"

// =====================
// XIAO ESP32S3 Sense Pins
// =====================
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39

#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15

#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

// Paramètres Edge Impulse
static constexpr int IN_W = EI_CLASSIFIER_INPUT_WIDTH;   // 240
static constexpr int IN_H = EI_CLASSIFIER_INPUT_HEIGHT;  // 240
static constexpr size_t IN_CH = 3;                       // RGB
static uint8_t *snapshot = nullptr;                      // Buffer final RGB888

// =====================
// Initialisation Caméra
// =====================
bool camera_init() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  
  config.frame_size = FRAMESIZE_240X240; // Taille native pour le modèle
  config.pixel_format = PIXFORMAT_RGB565; 
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.fb_count = 2; // Plus stable
  config.jpeg_quality = 12;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  // Correction orientation pour le module Sense
  if (s->id.PID == OV2640_PID) {
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
  }
  return true;
}

// =====================
// Capture et Conversion
// =====================
bool capture_to_snapshot_rgb() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Capture failed");
    return false;
  }

  // Allocation du buffer snapshot si nécessaire
  if (!snapshot) {
    snapshot = (uint8_t*)malloc(IN_W * IN_H * IN_CH);
    if (!snapshot) {
      Serial.println("Memory error");
      esp_camera_fb_return(fb);
      return false;
    }
  }

  // Conversion simplifiée RGB565 vers RGB888
  uint16_t *src = (uint16_t *)fb->buf;
  uint8_t *dst = snapshot;

  for (int i = 0; i < (IN_W * IN_H); i++) {
    uint16_t p = src[i];
    // Formule rapide : extraction et mise à l'échelle 8-bit
    *dst++ = (uint8_t)((p & 0xF800) >> 8); // R
    *dst++ = (uint8_t)((p & 0x07E0) >> 3); // G
    *dst++ = (uint8_t)((p & 0x001F) << 3); // B
  }

  esp_camera_fb_return(fb);
  return true;
}

// Callback Edge Impulse
int ei_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_offset = offset / 3;
  size_t channel_offset = offset % 3;

  for (size_t i = 0; i < length; i++) {
    out_ptr[i] = (float)snapshot[offset + i];
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  while(!Serial); // Attendre l'ouverture du moniteur série
  
  Serial.println("Initialisation BeeGuard AI...");

  if (!camera_init()) {
    Serial.println("ERREUR: Camera Init");
    while (1) delay(100);
  }

  Serial.println("Caméra OK. Prêt pour l'inférence.");
}

void loop() {
  if (!capture_to_snapshot_rgb()) {
    return;
  }

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_NN_INPUT_FRAME_SIZE;
  signal.get_data = &ei_get_data;

  ei_impulse_result_t result = { 0 };
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);

  if (err != EI_IMPULSE_OK) {
    Serial.printf("Erreur classifier: %d\n", err);
    return;
  }

  // Affichage FOMO
  bool found = false;
  for (size_t i = 0; i < result.bounding_boxes_count; i++) {
    auto bb = result.bounding_boxes[i];
    if (bb.value >= 0.5) { // Seuil de confiance 50%
      found = true;
      Serial.printf("Objet: %s (%.2f) [x:%u, y:%u]\n", bb.label, bb.value, bb.x, bb.y);
    }
  }

  if (!found) Serial.println("Rien détecté.");

  delay(100); 
}