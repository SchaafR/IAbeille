#include <Arduino.h>
#include <mbed.h>
#include <chrono>
#include <DHT.h>
#include <BH1750.h>
#include <Wire.h>
#include "Sound.h"
#include <PDM.h>

#define DEBUG 0

#if DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_BEGIN(x) \
  Serial.begin(x); \
  while (!Serial) \
    ;
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_BEGIN(x)
#endif

// =====================
// Arduino Nano 33 BLE
// Bit-bang UART
// RX = D5
// TX = D6
// WAKE OUT = D3
// Buzzer = D8
// DHT22 DATA = D10
// LoRaWAN module = Serial1
// =====================

// ------- AI Output --------
float lastTemp = NAN;
float lastHum = NAN;

uint8_t lastBatteryStatus = 0;
uint16_t lastLux = 0;

uint8_t lastHornet_sound = 0;  // Sera utilisé pour t_5
uint8_t lastQueen_sound = 0;   // Sera utilisé pour t_4
uint8_t lastHornet = 0;        // Sera utilisé pour t_1
uint8_t lastBee = 0;           // Sera utilisé pour t_2
uint8_t lastBees = 0;          // Sera utilisé pour t_3
uint8_t lastStatus = 0;        // Sera utilisé pour t_0 (0-3)

// ---------- Downlink ----------
// 0: Mode IA (Détection frelon), 1: 30 mins, 2: 2 mins (Test)
uint8_t modeFonctionnement = 0;
unsigned long intervalleEnvoi = 0;  // ms
unsigned long dernierEnvoiLoRa = 0;
int rssiJoin = 0;

// ---------- Bit-bang UART ----------
static const int PIN_RX = D5;
static const int PIN_TX = D6;
static const int PIN_WAKE_OUT = D3;
static const int PIN_SHDN = D2;

// ---------- Buzzer ----------
static const int PIN_BUZZER = D8;
static const int DELAY_BUZZER = 1000;

// ---------- DHT22 ----------
static const int PIN_DHT = D10;
#define DHTTYPE DHT22
DHT dht(PIN_DHT, DHTTYPE);

// ---------- BH1750 ----------
BH1750 lightMeter;

// ---------- Configuration Batterie ----------
const int PIN_BATTERY = A0;
const float R1 = 6800.0;
const float R2 = 22000.0;
const float BAT_RATIO = (R1 + R2) / R2;

const float VOLT_MAX = 4.20;  // 100%
const float VOLT_MIN = 3.30;  // 0%

int cyclesSansEnvoi = 0;
int MAX_CYCLES_SILENCE = 60;

// ---------- LoRaWAN ----------
const char *DevEUI = "70B3D57ED00761C6";
const char *appEui = "0000000000000000";
const char *appKey = "73D3232B6CA32A68C5695B8DCC3E4672";

// ---------- Machine à États ----------
enum EtatSysteme {
  ETAT_SOMMEIL,
  ETAT_VERIFICATION,
  ETAT_ANALYSE_AUDIO,
  ETAT_ANALYSE_CAMERA,
  ETAT_ENVOI_LORAWAN
};

EtatSysteme etatActuel = ETAT_SOMMEIL;
int compteurSommeil = 0;
uint8_t request_id = 1;

// --------------- Audio -----------------------

int16_t sharedBuffer[MAX_SAMPLES];
volatile int samplesRead = 0;
volatile bool recordComplete = false;

// Callback d'interruption du micro
void onPDMdata() {
    int bytesAvailable = PDM.available();
    int samplesToRead = bytesAvailable / 2;
    if (samplesRead + samplesToRead <= MAX_SAMPLES) {
        PDM.read(sharedBuffer + samplesRead, bytesAvailable);
        samplesRead += samplesToRead;
    } else {
        recordComplete = true;
    }
}

bool capture_audio(int duration_ms) {
    int samplesNeeded = (duration_ms == 2000) ? 32000 : 16000;
    samplesRead = 0;
    recordComplete = false;

    if (!PDM.begin(1, 16000)) {
        Serial.println("Erreur PDM !");
        return false;
    }
    PDM.setGain(25);
    PDM.onReceive(onPDMdata);

    unsigned long start = millis();
    while (!recordComplete && (millis() - start < duration_ms + 500)) {
        yield();
    }

    PDM.end(); // Libère les ressources pour l'inférence
    return recordComplete;
}

// ---------- Fonctions d'énergie ----------

void entrerModeSommeil(uint32_t dureeMs) {
  Wire.begin();  // S'assurer que le bus est prêt pour envoyer les ordres d'extinction
  // 1. Éteindre le BH1750 (Lumière) manuellement via I2C
  // L'adresse par défaut est souvent 0x23. La commande Power Down est 0x00.
  Wire.beginTransmission(0x23);
  Wire.write(0x00);
  Wire.endTransmission();

  // 2. Éteindre les autres capteurs internes (ton code existant)
  writeI2C(0x6B, 0x10, 0x00);  // IMU
  writeI2C(0x6B, 0x20, 0x00);
  writeI2C(0x1E, 0x22, 0x03);  // Magneto
  writeI2C(0x39, 0x80, 0x00);  // Proximité
  writeI2C(0x5F, 0x20, 0x00);  // Humidité/Temp interne
  writeI2C(0x5C, 0x10, 0x00);  // Baromètre

  // 3. Arrêter le Micro PDM et le bit-bang DHT
  PDM.end();
  pinMode(PIN_DHT, INPUT);  // On "libère" la pin du DHT22 pour éviter les fuites

  // 4. Couper le régulateur Pololu (LoRa/Caméra)
  digitalWrite(PIN_SHDN, LOW);

  // 5. Désactiver la LED (si applicable)
  digitalWrite(LED_PWR, LOW);

  // 6. Désactiver le bus I2C (Haute impédance)
  pinMode(PIN_WIRE_SDA, INPUT);
  pinMode(PIN_WIRE_SCL, INPUT);
  // 7. Sommeil
  rtos::ThisThread::sleep_for(std::chrono::milliseconds(dureeMs));
}

// Fonction utilitaire légère pour écrire dans les registres
void writeI2C(uint8_t address, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void reveillerCapteurs() {

  Wire.begin();
  dht.begin();
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  digitalWrite(PIN_SHDN, HIGH);
  delay(10);
  if (!PDM.begin(1, 16000)) {
    DEBUG_PRINTLN("Échec démarrage PDM");
  }

  //digitalWrite(LED_PWR,LOW);

#if DEBUG
  Serial.begin(9600);
#endif
}

void checkDownlink() {
  if (Serial1.available()) {
    String line = Serial1.readStringUntil('\n');
    DEBUG_PRINT("[LoRa RX] : ");
    DEBUG_PRINTLN(line);

    // --- 1. Extraction du RSSI ---
    if (line.indexOf("RSSI") != -1) {
      int posRSSI = line.indexOf("RSSI");
      int posComma = line.indexOf(",", posRSSI);
      if (posComma != -1) {
        String val = line.substring(posRSSI + 5, posComma);
        val.trim();
        rssiJoin = val.toInt();
        DEBUG_PRINT("=> RSSI capturé via Downlink : ");
        DEBUG_PRINTLN(rssiJoin);
      }
    }

    // --- 2. Gestion des commandes (ton code actuel) ---
    int posRX = line.indexOf("RX:");
    if (posRX >= 0) {
      String hexVal = line.substring(posRX + 3);
      hexVal.trim();
      hexVal.replace("\"", "");
      int commande = (int)strtol(hexVal.c_str(), NULL, 16);

      switch (commande) {
        case 0: modeFonctionnement = 0; break;
        case 1:
          modeFonctionnement = 1;
          intervalleEnvoi = 30 * 60 * 1000;
          break;
        case 2:
          modeFonctionnement = 2;
          intervalleEnvoi = 2 * 60 * 1000;
          break;
      }
    }
  }
}

void updateBatteryStatus() {
  long sumRaw = 0;
  for (int i = 0; i < 20; i++) {
    sumRaw += analogRead(PIN_BATTERY);
    delayMicroseconds(500);
  }

  float averageRaw = sumRaw / 20.0;
  float voltageA0 = (averageRaw * 3.3) / 4095.0;
  float batteryVoltage = voltageA0 * BAT_RATIO;

  float percentage = (batteryVoltage - VOLT_MIN) * 100.0 / (VOLT_MAX - VOLT_MIN);
  lastBatteryStatus = (uint8_t)constrain((int)round(percentage), 0, 100);
}

// ---------- UART bit-bang ----------
static const uint32_t BAUD = 4800;
static const uint32_t BIT_US = 1000000UL / BAUD;

enum MsgType : uint8_t {
  CMD_RUN = 0x01,
  RESULT = 0x02,
  ACK = 0x03,
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

  bb_tx_byte(0xA5);
  delay(2);
  chk ^= 0xA5;

  bb_tx_byte(0x5A);
  delay(2);
  chk ^= 0x5A;

  bb_tx_byte(type);
  delay(2);
  chk ^= type;

  bb_tx_byte(len);
  delay(2);
  chk ^= len;

  for (uint8_t i = 0; i < len; i++) {
    bb_tx_byte(payload[i]);
    delay(2);
    chk ^= payload[i];
  }

  bb_tx_byte(chk);
  delay(2);
}

bool recv_packet(uint8_t &type, uint8_t *payload, uint8_t &len,
                 uint32_t timeout_us = 1000000) {
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

// ---------- Capteurs ----------
void printDHT22() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    lastTemp = NAN;
    lastHum = NAN;
    return;
  }

  lastTemp = t;
  lastHum = h;
}

void printLux() {
  float lux = lightMeter.readLightLevel();

  if (lux < 0 || isnan(lux)) {
    lastLux = 0;
    return;
  }

  lastLux = (uint16_t)constrain((int)round(lux), 0, 65535);
}

// // ---------- Commandes XIAO ----------
// void send_cmd_run(uint8_t request_id) {
//   uint8_t payload[1] = { request_id };
//   send_packet(CMD_RUN, payload, 1);
// }

// void send_ack(uint8_t request_id, uint8_t code) {
//   uint8_t payload[2] = { request_id, code };
//   send_packet(ACK, payload, 2);
// }

// void send_cmd_sleep(uint8_t request_id) {
//   uint8_t payload[1] = { request_id };
//   send_packet(CMD_SLEEP, payload, 1);
// }

// ---------- LoRaWAN ----------

bool connectToTTN() {
  while (Serial1.available()) Serial1.read();

  Serial1.print("AT+ID=DevEUI,\"");
  Serial1.print(DevEUI);
  Serial1.println("\"");
  delay(500);

  Serial1.print("AT+ID=AppEUI,\"");
  Serial1.print(appEui);
  Serial1.println("\"");
  delay(500);

  Serial1.print("AT+KEY=APPKEY,\"");
  Serial1.print(appKey);
  Serial1.println("\"");
  delay(500);

  Serial1.println("AT+MODE=LWOTAA");
  delay(500);

  Serial1.println("AT+DR=EU868");
  delay(500);
  /* 0 pour SF12, 5 pour SF7 */
  Serial1.println("AT+DR=3");
  delay(500);

  DEBUG_PRINTLN("Tentative JOIN...");

  unsigned long timeout = millis() + 30000;
  while (millis() < timeout) {
    Serial1.println("AT+JOIN");
    if (Serial1.available()) {
      String line = Serial1.readString();
      DEBUG_PRINT(line);

      if (line.indexOf("joined") != -1) {
        DEBUG_PRINTLN("JOIN réussi");
        return true;
      }
      delay(1000);
    }
  }

  DEBUG_PRINTLN("Timeout JOIN");
  return false;
}

void sendLoRaPayload(const String &payload) {
  //delay(10000);

  DEBUG_PRINT("Envoi payload LoRa : ");
  DEBUG_PRINTLN(payload);

  Serial1.print("AT+MSGHEX=");
  Serial1.println(payload);

  unsigned long start = millis();
  while (millis() - start < 5000) {
    if (Serial1.available()) {
      String resp = Serial1.readString();
      DEBUG_PRINT(resp);
    }
  }
}

String buildLoRaPayload() {
  int16_t temp10 = isnan(lastTemp) ? 0 : (int16_t)round(lastTemp * 10.0);
  uint8_t t0_state = lastStatus;
  uint16_t lux = lastLux;
  uint8_t hum = isnan(lastHum) ? 0 : (uint8_t)constrain((int)round(lastHum), 0, 100);
  uint8_t bv = lastBatteryStatus;
  uint16_t rssi = (uint16_t)(-rssiJoin);
  uint8_t rssi_1 = (uint8_t)((rssi >> 8) & 0xFF);
  uint8_t rssi_2 = (uint8_t)(rssi & 0xFF);

  uint8_t b_t_msb = (uint8_t)((temp10 >> 8) & 0xFF);
  uint8_t b_t_lsb = (uint8_t)(temp10 & 0xFF);

  uint8_t b_l_msb = (uint8_t)((lux >> 8) & 0xFF);
  uint8_t b_l_lsb = (uint8_t)(lux & 0xFF);

  char hexPayload[29];  // 14 octets * 2 chars + null terminator
  sprintf(hexPayload, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
          b_t_msb, /* t_raw */
          b_t_lsb,
          lastHornet, /*t_1*/
          t0_state,   /*t_0*/
          b_l_msb,    /*l*/
          b_l_lsb,
          hum,              /*h*/
          bv,               /*bv*/
          lastHornet_sound, /*t_2*/
          rssi_1,           /*rssi*/
          rssi_2,
          lastBee,        /*t_3*/
          lastBees,       /*t_4*/
          lastQueen_sound /*t_5*/
  );

  DEBUG_PRINT("Payload construit : ");
  DEBUG_PRINTLN(hexPayload);

  return String(hexPayload);
}

void setup() {

  // usb shut up ble shut up
  DEBUG_BEGIN(115200);
  delay(5000);

  pinMode(LED_PWR, OUTPUT);
  digitalWrite(LED_PWR, LOW);
  //pinMode(PIN_ENABLE_SENSORS_3V3, OUTPUT);
  //pinMode(PIN_ENABLE_I2C_PULLUP, OUTPUT);

  pinMode(PIN_SHDN, OUTPUT);
  digitalWrite(PIN_SHDN, HIGH);

  analogReadResolution(12);

  pinMode(PIN_RX, INPUT_PULLUP);
  pinMode(PIN_TX, OUTPUT);
  digitalWrite(PIN_TX, HIGH);

  pinMode(PIN_WAKE_OUT, OUTPUT);
  digitalWrite(PIN_WAKE_OUT, LOW);

  Wire.begin();
  dht.begin();
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);

  Serial1.begin(9600);

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, HIGH);
  delay(DELAY_BUZZER);
  digitalWrite(PIN_BUZZER, LOW);

  DEBUG_PRINTLN("Initialisation terminée");
}

// ---------- Loop ----------
void loop() {
  switch (etatActuel) {

    case ETAT_SOMMEIL:

      if (modeFonctionnement == 2) {
        // mode test : 30 secondes
        entrerModeSommeil(10000);
      } else {
        // modes normaux : 30 mins
        entrerModeSommeil(1800000);
      }

      compteurSommeil++;
      if (compteurSommeil >= 1) {
        compteurSommeil = 0;
        reveillerCapteurs();
        etatActuel = ETAT_VERIFICATION;
      }
      break;

    case ETAT_VERIFICATION:
      updateBatteryStatus();
      printDHT22();
      printLux();
      /* Sécurité */
      /*if(lastBatteryStatus<25){
etatActuel = ETAT_SOMMEIL;
break;
}*/

      if (lastTemp > 12.0 && lastLux > 100.0) {

        DEBUG_PRINTLN("-> Mode jour");

        // dans tous les cas remis à zéro dans envoi Lorawan
        cyclesSansEnvoi++;

        // test
        if (modeFonctionnement == 2) {
          DEBUG_PRINTLN("-> Mode test");
          if (cyclesSansEnvoi >= 4) {
            lastStatus = 0;
            lastHornet = 0;
            etatActuel = ETAT_ENVOI_LORAWAN;
          } else {
            etatActuel = ETAT_ANALYSE_AUDIO;
          }
        } else if (modeFonctionnement == 1) {
          DEBUG_PRINTLN("-> Mode camera");
          if (cyclesSansEnvoi >= 4) {
            lastStatus = 0;
            lastHornet = 0;
            etatActuel = ETAT_ENVOI_LORAWAN;
          } else {
            etatActuel = ETAT_ANALYSE_CAMERA;
          }
        } else {
          DEBUG_PRINTLN("-> Mode son");
          if (cyclesSansEnvoi >= 4) {
            lastStatus = 0;
            lastHornet = 0;
            etatActuel = ETAT_ENVOI_LORAWAN;
          } else {
            etatActuel = ETAT_ANALYSE_AUDIO;
          }
        }

      } else {
        DEBUG_PRINTLN("-> Mode nuit");
        cyclesSansEnvoi++;
        if (cyclesSansEnvoi >= 4) {
          lastStatus = 0;
          lastHornet = 0;
          etatActuel = ETAT_ENVOI_LORAWAN;
        }
      }

      break;

    // Hornet + Queen
    case ETAT_ANALYSE_AUDIO:
      {
        float sumHornet = 0;
        float sumQueen = 0;
        int successfulCaptures = 0;

        DEBUG_PRINTLN("Début Analyse Audio (5 cycles)...");

        for (int i = 0; i < 3; i++) {
          if (capture_audio(2000)) { // Capture 2s
            AudioResults res = run_all_inferences();
            sumHornet += res.hornet;
            sumQueen += res.queen;
            successfulCaptures++;
            
            DEBUG_PRINT("Essai "); DEBUG_PRINT(i+1);
            DEBUG_PRINT(" - Hornet: "); DEBUG_PRINTLN(res.hornet);
            DEBUG_PRINT(" - Queen: "); DEBUG_PRINTLN(res.queen);
          } else {
            DEBUG_PRINTLN("Échec capture son");
          }
        }

        if (successfulCaptures > 0) {
          // Calcul des moyennes
          float avgHornet = sumHornet / successfulCaptures;
          float avgQueen = sumQueen / successfulCaptures;

          // Mise à jour des variables globales pour l'envoi LoRa/UART
          lastHornet_sound = (uint8_t)(avgHornet * 100);
          lastQueen_sound = (uint8_t)(avgQueen * 100);

          // Logique de décision (Seuil à 70% comme demandé)
          if (avgHornet > 0.70) {
            DEBUG_PRINTLN("=> Frelon détecté ! Activation CAMERA.");
            etatActuel = ETAT_ANALYSE_CAMERA;
          } else {
            DEBUG_PRINTLN("=> RAS. Retour sommeil.");
            etatActuel = ETAT_SOMMEIL;
          }
        } else {
          DEBUG_PRINTLN("Erreur critique : Aucune capture réussie.");
          etatActuel = ETAT_SOMMEIL;
        }
      }
      break;

    /* Payload 4 octets : req_id score_1 score_1 score_1 */
    case ETAT_ANALYSE_CAMERA:
      {
        digitalWrite(PIN_WAKE_OUT, HIGH);
        delay(500);

        uint8_t type, len;
        uint8_t payload[32];
        bool got_result = false;
        uint32_t t0 = millis();

        uint8_t frame_id = 0;

        // On attend une trame valide envoyée automatiquement par la XIAO
        while ((millis() - t0) < 5000) {
          if (recv_packet(type, payload, len, 800000)) {

            if (type == RESULT && len == 4) {
              frame_id = payload[0];
              lastHornet = payload[1];
              lastBee = payload[2];
              lastBees = payload[3];

              // lastStatus n'est plus transmis par la XIAO
              // On le reconstruit localement si besoin
              lastStatus = (lastHornet > 0) ? 1 : 0;

              DEBUG_PRINT("Frame ID : ");
              DEBUG_PRINTLN(frame_id);

              DEBUG_PRINT("Hornet % = ");
              DEBUG_PRINTLN(lastHornet);

              DEBUG_PRINT("Bee % = ");
              DEBUG_PRINTLN(lastBee);

              DEBUG_PRINT("Bees % = ");
              DEBUG_PRINTLN(lastBees);

              got_result = true;
              break;
            } else if (type == ERROR_MSG && len >= 2) {
              DEBUG_PRINT("Camera error, frame=");
              DEBUG_PRINT(payload[0]);
              DEBUG_PRINT(" code=");
              DEBUG_PRINTLN(payload[1]);
              got_result = false;
              break;
            }
          }
        }

        digitalWrite(PIN_WAKE_OUT, LOW);

        if (!got_result) {
          etatActuel = ETAT_SOMMEIL;
        } else {
          request_id++;

          if (lastHornet > 0) {
            etatActuel = ETAT_ENVOI_LORAWAN;
          } else {
            etatActuel = ETAT_SOMMEIL;
          }
        }
      }
      break;

    case ETAT_ENVOI_LORAWAN:
      digitalWrite(PIN_SHDN, HIGH);
      delay(100);

      if (connectToTTN()) {
        String loraPayload = buildLoRaPayload();
        sendLoRaPayload(loraPayload);
        cyclesSansEnvoi = 0;

        DEBUG_PRINTLN("Attente Downlink (RX1/RX2)...");
        unsigned long listenStart = millis();
        // On écoute pendant 5 à 10 secondes pour être sûr de capter le message RX
        while (millis() - listenStart < 8000) {
          checkDownlink();
          delay(10);  // Petit délai pour laisser le buffer série se remplir
        }

      } else {
        DEBUG_PRINTLN("Échec du JOIN avant envoi");
      }

      digitalWrite(PIN_SHDN, LOW);
      etatActuel = ETAT_SOMMEIL;
      break;
  }
}