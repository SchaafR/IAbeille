#include <Arduino.h>
#include <mbed.h>
#include <chrono>
#include <DHT.h>
#include <BH1750.h>
#include <Wire.h>
#include "HornetAudio.h"

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

uint16_t lastHornet = 0; // Sera utilisé pour t_1
uint8_t lastStatus = 0;  // Sera utilisé pour t_0 (0-3)

// ---------- Downlink ----------
// 0: Mode IA (Détection frelon), 1: 30 mins, 2: 2 mins (Test)
uint8_t modeFonctionnement = 2; 
unsigned long intervalleEnvoi = 0; // ms
unsigned long dernierEnvoiLoRa = 0;

// ---------- Bit-bang UART ----------
static const int PIN_RX = D5;
static const int PIN_TX = D6;
static const int PIN_WAKE_OUT = D3;
static const int PIN_SHDN = D2;

// ---------- Buzzer ----------
static const int PIN_BUZZER = D8;

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

const float VOLT_MAX = 4.20; // 100%
const float VOLT_MIN = 3.30; // 0%

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

// ---------- Fonctions d'énergie ----------
void entrerModeSommeil(unsigned long dureeMs) {
  Serial.flush();
  digitalWrite(PIN_WAKE_OUT, LOW);
  digitalWrite(PIN_SHDN, LOW);
  digitalWrite(LED_PWR, LOW);
  digitalWrite(PIN_ENABLE_SENSORS_3V3, LOW);
  digitalWrite(PIN_ENABLE_I2C_PULLUP, LOW);

  rtos::ThisThread::sleep_for(std::chrono::milliseconds(dureeMs));
}

void reveillerCapteurs() {
  digitalWrite(PIN_SHDN, HIGH);
  digitalWrite(LED_PWR, HIGH);
  digitalWrite(PIN_ENABLE_SENSORS_3V3, HIGH);
  digitalWrite(PIN_ENABLE_I2C_PULLUP, HIGH);

  delay(500);

  Wire.begin();
  dht.begin();
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
}

void checkDownlink() {
  if (Serial1.available()) {
    String line = Serial1.readStringUntil('\n');
    Serial.print("[LoRa RX] : "); Serial.println(line);

    int pos = line.indexOf("RX:");
    if (pos >= 0) {
      String hexVal = line.substring(pos + 3);
      hexVal.trim();
      hexVal.replace("\"", "");

      // Conversion de la valeur Hexa reçue en entier
      int commande = (int)strtol(hexVal.c_str(), NULL, 16);

      switch (commande) {
        case 0:
          modeFonctionnement = 0;
          Serial.println("=> Mode activé : Détection IA Frelon");
          break;
        case 1:
          modeFonctionnement = 1;
          intervalleEnvoi = 30 * 60 * 1000; // 30 minutes
          Serial.println("=> Mode activé : Envoi périodique 30 min");
          break;
        case 2:
          modeFonctionnement = 2;
          intervalleEnvoi = 2 * 60 * 1000; // 2 minutes (Test)
          Serial.println("=> Mode activé : Envoi périodique 2 min");
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

// ---------- Commandes XIAO ----------
void send_cmd_run(uint8_t request_id) {
  uint8_t payload[1] = { request_id };
  send_packet(CMD_RUN, payload, 1);
}

void send_ack(uint8_t request_id, uint8_t code) {
  uint8_t payload[2] = { request_id, code };
  send_packet(ACK, payload, 2);
}

void send_cmd_sleep(uint8_t request_id) {
  uint8_t payload[1] = { request_id };
  send_packet(CMD_SLEEP, payload, 1);
}

// ---------- LoRaWAN ----------
bool connectToTTN() {
  while (Serial1.available()) Serial1.read();

  Serial.println("Config DevEUI...");
  Serial1.print("AT+ID=DevEUI,\"");
  Serial1.print(DevEUI);
  Serial1.println("\"");
  delay(500);

  Serial.println("Config AppEUI...");
  Serial1.print("AT+ID=AppEUI,\"");
  Serial1.print(appEui);
  Serial1.println("\"");
  delay(500);

  Serial.println("Config APPKEY...");
  Serial1.print("AT+KEY=APPKEY,\"");
  Serial1.print(appKey);
  Serial1.println("\"");
  delay(500);

  Serial.println("Mode OTAA...");
  Serial1.println("AT+MODE=LWOTAA");
  delay(500);

  Serial.println("Région EU868...");
  Serial1.println("AT+DR=EU868");
  delay(500);

  Serial.println("Tentative JOIN...");
  Serial1.println("AT+JOIN");

  unsigned long timeout = millis() + 30000;
  while (millis() < timeout) {
    if (Serial1.available()) {
      String line = Serial1.readString();
      Serial.print(line);

      if (line.indexOf("joined") != -1) {
        Serial.println("JOIN réussi");
        return true;
      }
      if (line.indexOf("Join failed") != -1) {
        Serial.println("JOIN échoué");
        return false;
      }
    }
  }

  Serial.println("Timeout JOIN");
  return false;
}

void sendLoRaPayload(const String &payload) {
  delay(10000);

  Serial.print("Envoi payload LoRa : ");
  Serial.println(payload);

  Serial1.print("AT+MSGHEX=");
  Serial1.println(payload);

  unsigned long start = millis();
  while (millis() - start < 5000) {
    if (Serial1.available()) {
      String resp = Serial1.readString();
      Serial.print(resp);
    }
  }
}

String buildLoRaPayload() {
  int16_t temp10 = isnan(lastTemp) ? 0 : (int16_t)round(lastTemp * 10.0);
  uint16_t hornet = lastHornet;
  uint8_t t0_state = lastStatus;
  uint16_t lux = lastLux;
  uint8_t hum = isnan(lastHum) ? 0 : (uint8_t)constrain((int)round(lastHum), 0, 100);
  uint8_t bv = lastBatteryStatus;

  uint8_t b_t_msb  = (uint8_t)((temp10 >> 8) & 0xFF);
  uint8_t b_t_lsb  = (uint8_t)(temp10 & 0xFF);

  uint8_t b_t1_msb = (uint8_t)((hornet >> 8) & 0xFF);
  uint8_t b_t1_lsb = (uint8_t)(hornet & 0xFF);

  uint8_t b_l_msb  = (uint8_t)((lux >> 8) & 0xFF);
  uint8_t b_l_lsb  = (uint8_t)(lux & 0xFF);

  char hexPayload[19];
  sprintf(hexPayload, "%02X%02X%02X%02X%02X%02X%02X%02X%02X",
          b_t_msb, b_t_lsb,
          b_t1_msb, b_t1_lsb,
          t0_state,
          b_l_msb, b_l_lsb,
          hum,
          bv
  );

  Serial.print("Payload construit : ");
  Serial.println(hexPayload);

  return String(hexPayload);
}

void setup() {

  Serial.begin(115200);
  delay(5000);

  pinMode(LED_PWR, OUTPUT);
  pinMode(PIN_ENABLE_SENSORS_3V3, OUTPUT);
  pinMode(PIN_ENABLE_I2C_PULLUP, OUTPUT);

  pinMode(PIN_SHDN, OUTPUT);
  digitalWrite(PIN_SHDN, HIGH);

  analogReadResolution(12);

  pinMode(PIN_RX, INPUT_PULLUP);
  pinMode(PIN_TX, OUTPUT);
  digitalWrite(PIN_TX, HIGH);

  pinMode(PIN_WAKE_OUT, OUTPUT);
  digitalWrite(PIN_WAKE_OUT, LOW);

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, HIGH);
  delay(50);
  digitalWrite(PIN_BUZZER, LOW);

  Wire.begin();
  dht.begin();
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);

  Serial1.begin(9600);

  Serial.println("Initialisation terminée");
  
}

// ---------- Loop ----------
void loop() {
  switch (etatActuel) {

    case ETAT_SOMMEIL:

      if(modeFonctionnement==2){
        // mode test
        entrerModeSommeil(10000);
      }
      else{
        // modes normaux
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

      if (lastTemp > 12.0 && lastLux > 100.0) {

        // dans tous les cas remis à zéro dans envoi Lorawan
        cyclesSansEnvoi++;

        if(modeFonctionnement==2){
          Serial.println("Here");
          if (cyclesSansEnvoi >= 2) {
            lastStatus = 0;
            lastHornet = 0;
            etatActuel = ETAT_ENVOI_LORAWAN;
          } 
          else {
            etatActuel = ETAT_ANALYSE_AUDIO;
          }
        }
        else if(modeFonctionnement==1){
          if (cyclesSansEnvoi >= 2) {
            lastStatus = 0;
            lastHornet = 0;
            etatActuel = ETAT_ENVOI_LORAWAN;
          } 
          else {
            etatActuel = ETAT_ANALYSE_CAMERA;
          }
        }
        else{
          if (cyclesSansEnvoi >= MAX_CYCLES_SILENCE && lastLux > 100.0) {
            lastStatus = 0;
            lastHornet = 0;
            etatActuel = ETAT_ENVOI_LORAWAN;
          } else {
              etatActuel = ETAT_ANALYSE_AUDIO;
          }
        }

      } else {
        cyclesSansEnvoi++;

        if (cyclesSansEnvoi >= MAX_CYCLES_SILENCE*2) {
            lastStatus = 0;
            lastHornet = 0;
            etatActuel = ETAT_ENVOI_LORAWAN;
        } 

        etatActuel = ETAT_SOMMEIL;
      }

      break;

    case ETAT_ANALYSE_AUDIO:
    {  

      float probaHornet = -1;
      for(int i = 0; i < 3; i++){
        if (HornetDetector::capture(20)) {
            probaHornet = HornetDetector::getHornetScore();
        } 
      }

      if (probaHornet > 0.70) { 
          Serial.println("Camera");
          etatActuel = ETAT_ANALYSE_CAMERA;
      } else {
          //Serial.println("=> Rien de suspect. Retour vérification.");
          etatActuel = ETAT_SOMMEIL; 
      }
    } 
    break;

    case ETAT_ANALYSE_CAMERA:
    {
      digitalWrite(PIN_WAKE_OUT, HIGH);
      delay(100);

      send_cmd_run(request_id);

      uint8_t type, len;
      uint8_t payload[32];
      bool got_result = false;
      uint32_t t0 = millis();

      while ((millis() - t0) < 3000) {
        if (recv_packet(type, payload, len, 800000)) {
          if (type == RESULT && len >= 8) {
            uint8_t req = payload[0];
            lastHornet = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
            lastStatus = payload[7];

            send_ack(req, 0);
            delay(50);
            send_cmd_sleep(req);

            got_result = true;
            break;
          } else if (type == ERROR_MSG && len >= 2) {
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
      delay(1000);

      if (connectToTTN()) {
        String loraPayload = buildLoRaPayload();
        sendLoRaPayload(loraPayload);
        cyclesSansEnvoi = 0;

        Serial.println("Attente Downlink (RX1/RX2)...");
        unsigned long listenStart = millis();
        // On écoute pendant 5 à 10 secondes pour être sûr de capter le message RX
        while (millis() - listenStart < 10000) { 
          checkDownlink(); 
          delay(10); // Petit délai pour laisser le buffer série se remplir
        }

      } else {
        Serial.println("Échec du JOIN avant envoi");
      }

      digitalWrite(PIN_SHDN, LOW);
      etatActuel = ETAT_SOMMEIL;
      break;

  }
}
