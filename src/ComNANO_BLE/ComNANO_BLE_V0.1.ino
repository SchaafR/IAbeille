#include <Arduino.h>
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
// Buzzer = D12
// DHT22 DATA = D7
// LoRaWAN module = Serial1
// =====================

// ---------- Modes de fonctionnement (Downlink) -----------
// 0: Mode IA (Détection frelon), 1: 30 mins, 2: 2 mins (Test)
uint8_t modeFonctionnement = 0; 
unsigned long intervalleEnvoi = 0; // En millisecondes
unsigned long dernierEnvoiLoRa = 0;

// ------- AI Output --------
float lastTemp = NAN;
float lastHum = NAN;

uint8_t lastBatteryStatus = 0;
uint16_t lastLux = 0;

uint16_t lastHornet = 0;
uint16_t lastBee = 0;
uint16_t lastBees = 0;
uint8_t lastStatus = 0;

// ---------- Bit-bang UART ----------
static const int PIN_RX = D5;
static const int PIN_TX = D6;
static const int PIN_WAKE_OUT = D3;

// ---------- Buzzer ----------
static const int PIN_BUZZER = D12;

// ---------- DHT22 ----------
static const int PIN_DHT = D7;
#define DHTTYPE DHT22
DHT dht(PIN_DHT, DHTTYPE);

// BH1750
BH1750 lightMeter;

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

// Fonctions d'énergie
void entrerModeSommeil(unsigned long dureeMs) {
  digitalWrite(LED_PWR, LOW);
  digitalWrite(PIN_ENABLE_SENSORS_3V3, LOW);
  digitalWrite(PIN_ENABLE_I2C_PULLUP, LOW);
  delay(dureeMs);
}

void reveillerCapteurs() {
  digitalWrite(LED_PWR, HIGH);
  digitalWrite(PIN_ENABLE_SENSORS_3V3, HIGH);
  digitalWrite(PIN_ENABLE_I2C_PULLUP, HIGH);
  delay(100);
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

  digitalWrite(PIN_TX, LOW);  // start bit
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

  delayMicroseconds(BIT_US);  // stop bit

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


void printDHT22() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("DHT22 : erreur de lecture");
    lastTemp = NAN;
    lastHum = NAN;
    return;
  }

  lastTemp = t;
  lastHum = h;

  Serial.print("DHT22 -> Temperature: ");
  Serial.print(t);
  Serial.print(" °C | Humidite: ");
  Serial.print(h);
  Serial.println(" %");
}

void printLux() {
  float lux = lightMeter.readLightLevel();

  if (lux < 0 || isnan(lux)) {
    Serial.println("BH1750 : erreur de lecture");
    lastLux = 0;
    return;
  }

  lastLux = (uint16_t)constrain((int)round(lux), 0, 65535);

  Serial.print("BH1750 -> Lux: ");
  Serial.println(lastLux);
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

  Serial1.println("AT+JOIN");

  unsigned long timeout = millis() + 30000;
  while (millis() < timeout) {
    if (Serial1.available()) {
      String line = Serial1.readString();
      Serial.print(line);
      if (line.indexOf("joined") != -1) return true;
      if (line.indexOf("Join failed") != -1) return false;
    }
  }
  return false;
}

void sendLoRaPayload(const String &payload) {
  Serial.println("Attente");
  delay(10000);

  Serial.print("Tentative d'envoi des données : ");
  Serial.println(payload);

  Serial1.print("AT+MSGHEX=");
  Serial1.println(payload);

  unsigned long start = millis();
  while (millis() - start < 5000) {
    if (Serial1.available()) {
      String resp = Serial1.readString();
      Serial.print("Réponse module : ");
      Serial.print(resp);

      if (resp.indexOf("Done") != -1) {
        Serial.println("Envoie effectué");
      }
    }
  }

  Serial.println("Repos");
}

String buildLoRaPayload() {
  int16_t temp10 = isnan(lastTemp) ? 0 : (int16_t)round(lastTemp * 10.0);
  uint8_t hum = isnan(lastHum) ? 0 : (uint8_t)constrain((int)round(lastHum), 0, 100);
  uint8_t bv = lastBatteryStatus;
  uint16_t hornet = lastHornet;
  uint16_t bee = lastBee;
  uint16_t bees = lastBees;
  uint16_t lux = lastLux;

  uint8_t b0 = (uint8_t)((temp10 >> 8) & 0xFF);
  uint8_t b1 = (uint8_t)(temp10 & 0xFF);
  uint8_t b4 = (uint8_t)((hornet >> 8) & 0xFF);
  uint8_t b5 = (uint8_t)(hornet & 0xFF);
  uint8_t b6 = (uint8_t)((bee >> 8) & 0xFF);
  uint8_t b7 = (uint8_t)(bee & 0xFF);
  uint8_t b8 = (uint8_t)((bees >> 8) & 0xFF);
  uint8_t b9 = (uint8_t)(bees & 0xFF);
  uint8_t b10 = (uint8_t)((lux >> 8) & 0xFF);
  uint8_t b11 = (uint8_t)(lux & 0xFF);

  char hexPayload[25];
  sprintf(hexPayload, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
          b0, b1, hum, bv, b4, b5, b6, b7, b8, b9, b10, b11);

  Serial.print("Payload LoRa construite : ");
  Serial.println(hexPayload);

  Serial.print("  temp10 = ");
  Serial.println(temp10);
  Serial.print("  h      = ");
  Serial.println(hum);
  Serial.print("  bv     = ");
  Serial.println(bv);
  Serial.print("  hornet = ");
  Serial.println(hornet);
  Serial.print("  bee    = ");
  Serial.println(bee);
  Serial.print("  bees   = ");
  Serial.println(bees);
  Serial.print("  lux    = ");
  Serial.println(lux);

  return String(hexPayload);
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

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Configuration énergie
  pinMode(LED_PWR, OUTPUT);
  pinMode(PIN_ENABLE_SENSORS_3V3, OUTPUT);
  pinMode(PIN_ENABLE_I2C_PULLUP, OUTPUT);

  // Bit-bang UART
  pinMode(PIN_RX, INPUT_PULLUP);

  pinMode(PIN_TX, OUTPUT);
  digitalWrite(PIN_TX, HIGH);

  pinMode(PIN_WAKE_OUT, OUTPUT);
  digitalWrite(PIN_WAKE_OUT, LOW);

  // Buzzer
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_BUZZER, HIGH);
  delay(50);
  digitalWrite(PIN_BUZZER, LOW);

  // DHT22
  dht.begin();

  // BH1750
  Wire.begin();
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("BH1750 initialise");
  } else {
    Serial.println("Erreur de connexion BH1750");
  }

  Serial.println("Nano BLE ready");
  Serial.println("DHT22 + buzzer initialized");

  // LoRaWAN
  Serial1.begin(9600);
  Serial.println("Connexion");

  delay(2000);

  if (connectToTTN()) {
    Serial.println("CONNEXION REUSSIE");
  } else {
    Serial.println("ATTENTE OU ECHEC");
  }
  
  // Hornet Audio detection
  if (!HornetAudio::begin()) {
    Serial.println("Erreur fatale : Microphone non détecté");
  }
}

// ---------- Loop ----------
void loop() {
  switch (etatActuel) {

    case ETAT_SOMMEIL:
      Serial.println("[ÉTAT] Sommeil (1 min)...");
      entrerModeSommeil(60000); // 1 minute
      compteurSommeil++;
      
      if (compteurSommeil >= 1) {
        compteurSommeil = 0;
        reveillerCapteurs();
        
        // Initialisation de nos capteurs s'ils ne l'ont pas refait tous seuls
        dht.begin(); 
        lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
        
        etatActuel = ETAT_VERIFICATION;
      }
      break;

    case ETAT_VERIFICATION:
      // On vérifie toujours si un Downlink est arrivé
      checkDownlink();

      if (modeFonctionnement == 0) {
        // --- MODE 0 : IA 
        if (checkConditionsSeuils()) { 
          etatActuel = ETAT_ANALYSE_AUDIO;
        }
      } 
      else {
        // --- MODES 1 & 2 : Périodique ---
        if (millis() - dernierEnvoiLoRa >= intervalleEnvoi) {
          Serial.println("[TIMER] Intervalle atteint, passage à l'envoi.");
          etatActuel = ETAT_ENVOI_LORAWAN;
        }
      }
      break;

    case ETAT_ANALYSE_AUDIO:
      Serial.println("[ÉTAT] Analyse Audio Réelle...");
      {
        float probaHornet = HornetAudio::getHornetProbability();
        Serial.print("Probabilité frelon : ");
        Serial.println(probaHornet, 3);

        if (probaHornet > 0.85) { // Seuil de certitude à 85% 
          Serial.println("=> Buzz frelon détecté. Activation Caméra.");
          etatActuel = ETAT_ANALYSE_CAMERA;
        } else if (probaHornet < 0) {
          Serial.println("=> Erreur capture audio. Retour vérification.");
          etatActuel = ETAT_VERIFICATION; 
        } else {
          Serial.println("=> Rien de suspect. Retour vérification.");
          etatActuel = ETAT_VERIFICATION; 
        }
      }
      break;

    case ETAT_ANALYSE_CAMERA:
      Serial.println("[ÉTAT] Analyse Caméra via XIAO...");
      
      digitalWrite(PIN_WAKE_OUT, HIGH);
      delay(10);
      
      send_cmd_run(request_id);
      
      {
        uint8_t type, len;
        uint8_t payload[32];
        bool got_result = false;
        uint32_t t0 = millis();

        while ((millis() - t0) < 3000) {
          if (recv_packet(type, payload, len, 800000)) {
            if (type == RESULT && len >= 8) {
              uint8_t req = payload[0];
              lastHornet = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
              lastBee = (uint16_t)payload[3] | ((uint16_t)payload[4] << 8);
              lastBees = (uint16_t)payload[5] | ((uint16_t)payload[6] << 8);
              lastStatus = payload[7];

              Serial.print("RESULT received, hornet=");
              Serial.println(lastHornet);

              send_ack(req, 0);
              delay(50);
              send_cmd_sleep(req);

              got_result = true;
              break;
            } else if (type == ERROR_MSG && len >= 2) {
              got_result = true; // On sort pour pas bloquer
              break;
            }
          }
        }

        digitalWrite(PIN_WAKE_OUT, LOW);

        if (!got_result) {
          Serial.println("Timeout waiting RESULT");
          etatActuel = ETAT_VERIFICATION; // On annule et on recommence la vérification
        } else {
          request_id++;
          
          if (lastHornet > 0) { // Frelon détecté (selon vos données > 0)
            Serial.println("=> Frelon DÉTECTÉ ! Passage LoRaWAN.");
            etatActuel = ETAT_ENVOI_LORAWAN;
          } else {
            Serial.println("=> Pas de frelon. Retour à l'état vérification.");
            etatActuel = ETAT_VERIFICATION;
          }
        }
      }
      break;

    case ETAT_ENVOI_LORAWAN:
      Serial.println("[ÉTAT] Envoi LoRaWAN...");
      {
        String loraPayload = buildLoRaPayload();
        sendLoRaPayload(loraPayload);
        
        dernierEnvoiLoRa = millis(); // On reset le timer ici
        
        // Après l'envoi, on écoute pendant quelques secondes pour le Downlink
        unsigned long listenStart = millis();
        while (millis() - listenStart < 2000) { 
          checkDownlink(); 
        }

        etatActuel = ETAT_VERIFICATION;
      }
      break;
  }
}
