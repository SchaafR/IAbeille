#include <Arduino.h>
#include "ArduinoLowPower.h"
#include <DHT.h>
#include <BH1750.h>
#include <Wire.h>
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

// BH1750
BH1750 lightMeter;

// ---------- Configuration Batterie ----------
const int PIN_BATTERY = A0;
const float R1 = 6800.0; 
const float R2 = 22000.0; 
const float BAT_RATIO = (R1 + R2) / R2; 

const float VOLT_MAX = 4.20; // 100%
const float VOLT_MIN = 3.30; // 0%

int cyclesSansEnvoi = 0; 
int MAX_CYCLES_SILENCE = 1;

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
  Serial.flush(); // Attend la fin des transmissions série
  digitalWrite(PIN_WAKE_OUT, LOW); 
  // 2. Coupure de l'alimentation générale des capteurs (Pin D2 du planning)
  digitalWrite(PIN_SHDN, LOW);
  digitalWrite(LED_PWR, LOW);
  digitalWrite(PIN_ENABLE_SENSORS_3V3, LOW);
  digitalWrite(PIN_ENABLE_I2C_PULLUP, LOW);
  LowPower.sleep(dureeMs);
}

void reveillerCapteurs() {
  // 1. Rallumage physique des alimentations (Pin D2 Pololu + Pins internes)
  digitalWrite(PIN_SHDN, HIGH); 
  digitalWrite(LED_PWR, HIGH);
  digitalWrite(PIN_ENABLE_SENSORS_3V3, HIGH);
  digitalWrite(PIN_ENABLE_I2C_PULLUP, HIGH);
  delay(500); 
  Wire.begin(); 
  dht.begin(); 
  
  // if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
  //  // Serial.println("Erreur : BH1750 ne répond pas après réveil");
  // } else {
  //   Serial.println("Capteurs réinitialisés avec succès");
  // }
}

void updateBatteryStatus() {
  long sumRaw = 0;
  for(int i = 0; i < 20; i++) {
    sumRaw += analogRead(PIN_BATTERY);
    delayMicroseconds(500);
  }
  float averageRaw = sumRaw / 20.0;
  
  // 3.3V est la référence de tension interne du Nano 33 BLE
  float voltageA0 = (averageRaw * 3.3) / 4095.0;
  float batteryVoltage = voltageA0 * BAT_RATIO;

  float percentage = (batteryVoltage - VOLT_MIN) * 100.0 / (VOLT_MAX - VOLT_MIN);
  
  // On contraint entre 0 et 100
  lastBatteryStatus = (uint8_t)constrain((int)round(percentage), 0, 100);

  // Serial.print("Batterie : ");
  // Serial.print(batteryVoltage, 2);
  // Serial.print("V (");
  // Serial.print(lastBatteryStatus);
  // Serial.println("%)");
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
   // Serial.println("DHT22 : erreur de lecture");
    lastTemp = NAN;
    lastHum = NAN;
    return;
  }

  lastTemp = t;
  lastHum = h;

  // Serial.print("DHT22 -> Temperature: ");
  // Serial.print(t);
  // Serial.print(" °C | Humidite: ");
  // Serial.print(h);
  // Serial.println(" %");
}

void printLux() {
  float lux = lightMeter.readLightLevel();

  if (lux < 0 || isnan(lux)) {
    Serial.println("BH1750 : erreur de lecture");
    lastLux = 0;
    return;
  }

  lastLux = (uint16_t)constrain((int)round(lux), 0, 65535);

  // Serial.print("BH1750 -> Lux: ");
  // Serial.println(lastLux);
}

void send_cmd_run(uint8_t request_id) {
  uint8_t payload[1] = { request_id };
  send_packet(CMD_RUN, payload, 1);

  // Serial.print("CMD_RUN sent, req=");
  // Serial.println(request_id);
}

void send_ack(uint8_t request_id, uint8_t code) {
  uint8_t payload[2] = { request_id, code };
  send_packet(ACK, payload, 2);

  // Serial.print("ACK sent, req=");
  // Serial.print(request_id);
  // Serial.print(" code=");
  // Serial.println(code);
}

void send_cmd_sleep(uint8_t request_id) {
  uint8_t payload[1] = { request_id };
  send_packet(CMD_SLEEP, payload, 1);

  // Serial.print("CMD_SLEEP sent, req=");
  // Serial.println(request_id);
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
      // Serial.print(line);
      if (line.indexOf("joined") != -1) return true;
      if (line.indexOf("Join failed") != -1) return false;
    }
  }
  return false;
}

void sendLoRaPayload(const String &payload) {
  // Serial.println("Attente");
  delay(10000);

  // Serial.print("Tentative d'envoi des données : ");
  // Serial.println(payload);

  Serial1.print("AT+MSGHEX=");
  Serial1.println(payload);

  unsigned long start = millis();
  while (millis() - start < 5000) {
    if (Serial1.available()) {
      String resp = Serial1.readString();
      // Serial.print("Réponse module : ");
      // Serial.print(resp);

      // if (resp.indexOf("Done") != -1) {
      //   Serial.println("Envoie effectué");
      // }
    }
  }

  // Serial.println("Repos");
}

String buildLoRaPayload() {
  // 1. Préparation des données selon les types demandés
  int16_t temp10 = isnan(lastTemp) ? 0 : (int16_t)round(lastTemp * 10.0); // t (2 octets)
  uint16_t hornet = lastHornet;                                         // t_1 (2 octets)
  uint8_t t0_state = lastStatus;                                        // t_0 (1 octet, valeur 0-3)
  uint16_t lux = lastLux;                                               // l (2 octets)
  uint8_t hum = isnan(lastHum) ? 0 : (uint8_t)constrain((int)round(lastHum), 0, 100); // h (1 octet)
  uint8_t bv = lastBatteryStatus;                                       // bv (1 octet)

  // 2. Décomposition en octets (MSB first / Big Endian)
  uint8_t b_t_msb  = (uint8_t)((temp10 >> 8) & 0xFF);
  uint8_t b_t_lsb  = (uint8_t)(temp10 & 0xFF);
  
  uint8_t b_t1_msb = (uint8_t)((hornet >> 8) & 0xFF);
  uint8_t b_t1_lsb = (uint8_t)(hornet & 0xFF);

  uint8_t b_l_msb  = (uint8_t)((lux >> 8) & 0xFF);
  uint8_t b_l_lsb  = (uint8_t)(lux & 0xFF);

  // 3. Construction de la chaîne hexadécimale : tt_1t_0lhbv
  // Longueur : 9 octets = 18 caractères hex + 1 pour le caractère nul \0
  char hexPayload[19]; 
  sprintf(hexPayload, "%02X%02X%02X%02X%02X%02X%02X%02X%02X",
          b_t_msb, b_t_lsb,   // t
          b_t1_msb, b_t1_lsb, // t_1
          t0_state,           // t_0 (un seul octet %02X)
          b_l_msb, b_l_lsb,   // l
          hum,                // h
          bv                  // bv
  );

  // Debug Serial
  // Serial.print("Payload LoRa (tt_1t_0lhbv) : ");
  // Serial.println(hexPayload);
  // Serial.print("  t (tempx10)="); Serial.println(temp10);
  // Serial.print("  t_1 (hornet)="); Serial.println(hornet);
  // Serial.print("  t_0 (etat)=");   Serial.println(t0_state);
  // Serial.print("  l (lux)=");      Serial.println(lux);
  // Serial.print("  h (hum)=");      Serial.println(hum);
  // Serial.print("  bv (bat)=");     Serial.println(bv);

  return String(hexPayload);
}
void setup() {
  Serial.begin(115200);
  // SÉCURITÉ FLASH : Donne 5 secondes pour téléverser un nouveau code 
  // avant que la machine à états ne risque de passer en sommeil.
  delay(5000); 

  // Configuration énergie et pins de contrôle
  pinMode(LED_PWR, OUTPUT);
  pinMode(PIN_ENABLE_SENSORS_3V3, OUTPUT);
  pinMode(PIN_ENABLE_I2C_PULLUP, OUTPUT);
  
  // INITIALISATION DU POLOLU (D2)
  pinMode(PIN_SHDN, OUTPUT);
  digitalWrite(PIN_SHDN, HIGH); // Allume l'alimentation des capteurs au démarrage

  analogReadResolution(12);

  // Bit-bang UART vers ESP32/XIAO
  pinMode(PIN_RX, INPUT_PULLUP);
  pinMode(PIN_TX, OUTPUT);
  digitalWrite(PIN_TX, HIGH); // État IDLE du UART est HIGH

  // Signal de réveil physique vers ESP32
  pinMode(PIN_WAKE_OUT, OUTPUT);
  digitalWrite(PIN_WAKE_OUT, LOW); // On ne réveille pas l'IA tout de suite

  // Buzzer (Bip de démarrage)
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, HIGH);
  delay(50);
  digitalWrite(PIN_BUZZER, LOW);

  // Initialisation des capteurs (Première lecture)
  Wire.begin();
  dht.begin();
  // if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
  //   Serial.println("BH1750 OK");
  // } else {
  //   Serial.println("Erreur BH1750");
  // }

  // Configuration LoRaWAN (Module sur Serial1)
  Serial1.begin(9600);
  // Serial.println("Tentative de JOIN TTN...");

  // if (connectToTTN()) {
  //   Serial.println("CONNEXION REUSSIE");
  // } else {
  //   Serial.println("ECHEC JOIN (Vérifiez couverture ou clés)");
  // }
  
  // Serial.println("Initialisation terminée.");
}

// ---------- Loop ----------
void loop() {
  switch (etatActuel) {

    case ETAT_SOMMEIL:
      // Serial.println("[ÉTAT] Sommeil (1 min)...");
      entrerModeSommeil(60000); // 1 minute
      compteurSommeil++;
      
      if (compteurSommeil >= 1) {
        compteurSommeil = 0;
        reveillerCapteurs(); 
        etatActuel = ETAT_VERIFICATION;
      }
      break;

    case ETAT_VERIFICATION:
      // Serial.println("[ÉTAT] Vérification T° et Lux...");

      updateBatteryStatus();
      printDHT22();
      printLux();

      if (lastTemp > 12.0 && lastLux > 100.0) {
        if (cyclesSansEnvoi >= MAX_CYCLES_SILENCE && lastLux > 100.0) {
          // Serial.println("=> Rapport périodique forcé (Heartbeat).");
          lastStatus = 0; // On définit l'état à 0 (RAS)
          lastHornet = 0; // Pas de frelon
          etatActuel = ETAT_ENVOI_LORAWAN;
        }
        else {
          // Serial.println("=> Conditions favorables détectées.");
          etatActuel = ETAT_ANALYSE_AUDIO;
        }
      } 
      else {
        // Serial.println("=> Défavorable: Retour au sommeil.");
        if (lastLux > 100.0) cyclesSansEnvoi++; 
        etatActuel = ETAT_SOMMEIL;
      }
      break;

    case ETAT_ANALYSE_AUDIO:
      // Serial.println("[ÉTAT] Analyse Audio...");
      
      {
        // Simulation IA Audio (0 ou 1)
        int resultatAudio = random(0, 2); 
        
        if (resultatAudio == 1) {
          // Serial.println("=> Buzz détecté (1). Activation Caméra.");
          etatActuel = ETAT_ANALYSE_CAMERA;
        } else {
          // Serial.println("=> Rien (0). Retour vérification.");
          cyclesSansEnvoi++;
          etatActuel = ETAT_SOMMEIL;
        }
      }
      break;

    case ETAT_ANALYSE_CAMERA:
      // Serial.println("[ÉTAT] Analyse Caméra via XIAO...");
      
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
              lastStatus = payload[7];

              // Serial.print("RESULT received, hornet=");
              // Serial.println(lastHornet);

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
          // Serial.println("Timeout waiting RESULT");
          etatActuel = ETAT_VERIFICATION; // On annule et on recommence la vérification
        } else {
          request_id++;
          
          if (lastHornet > 0) { // Frelon détecté (selon vos données > 0)
            // Serial.println("=> Frelon DÉTECTÉ ! Passage LoRaWAN.");
            etatActuel = ETAT_ENVOI_LORAWAN;
          } else {
            // Serial.println("=> Pas de frelon. Retour à l'état vérification.");
            etatActuel = ETAT_VERIFICATION;
          }
        }
      }
      break;

    case ETAT_ENVOI_LORAWAN:
      // Serial.println("[ÉTAT] Réveil LoRaWAN...");
      digitalWrite(PIN_SHDN, HIGH); // On donne du jus au module
      delay(1000); // On attend que le module démarre

      // Obligatoire : On se reconnecte au réseau
      if (connectToTTN()) { 
        String loraPayload = buildLoRaPayload();
        sendLoRaPayload(loraPayload);
        cyclesSansEnvoi = 0;
      } else {
        // Serial.println("Échec du Join après réveil.");
      }

      // On recoupe tout pour économiser
      digitalWrite(PIN_SHDN, LOW); 
      etatActuel = ETAT_SOMMEIL;
      break;
  }
}