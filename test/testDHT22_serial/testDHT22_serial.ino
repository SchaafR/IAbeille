#include <Wire.h>
#include <DHT.h>

#define LUX_ADDR 0x23

// DHT11
#define DHTPIN 4
#define DHTTYPE DHT11
#define DHT_VCC_PIN 6

DHT dht(DHTPIN, DHTTYPE);

unsigned long lastTime = 0;
const unsigned long timerDelay = 2000;

// ====== THRESHOLD / WAKE CONFIG ======
const int LED_PIN  = 2;     // <-- choisis une pin libre (ex: D2)
const int WAKE_PIN = 3;     // <-- pin qui ira vers l'autre MCU (ex: D3). Mets -1 si tu ne l'utilises pas.

const float LUX_THRESHOLD_ON  = 30.0;  // lux en dessous => on commence à considérer "nuit"
const float LUX_THRESHOLD_OFF = 50.0;  // lux au-dessus => on considère "jour" (hystérésis)

const unsigned long TRIGGER_HOLD_MS = 5000; // condition vraie pendant 5s avant déclenchement
const unsigned long WAKE_PULSE_MS   = 100;  // impulsion de réveil 100ms

bool alarmActive = false;         // état "on est en alerte"
bool wakeAlreadySent = false;     // pour ne pulser qu'une fois
unsigned long condStartMs = 0;    // quand la condition est devenue vraie
unsigned long wakePulseStartMs = 0;
bool wakePulsing = false;

// ====================================

float readLux() {
  Wire.beginTransmission(LUX_ADDR);
  Wire.write(0x01);
  if (Wire.endTransmission() != 0) return -1;
  delay(10);

  Wire.beginTransmission(LUX_ADDR);
  Wire.write(0x20);
  if (Wire.endTransmission() != 0) return -1;

  delay(250);

  uint8_t n = Wire.requestFrom(LUX_ADDR, (uint8_t)2);
  if (n != 2) return -1;

  uint16_t raw = ((uint16_t)Wire.read() << 8) | Wire.read();
  return raw / 1.2f;
}

void startWakePulse() {
  if (WAKE_PIN < 0) return;
  digitalWrite(WAKE_PIN, HIGH);
  wakePulsing = true;
  wakePulseStartMs = millis();
}

void updateWakePulse() {
  if (!wakePulsing) return;
  if (millis() - wakePulseStartMs >= WAKE_PULSE_MS) {
    digitalWrite(WAKE_PIN, LOW);
    wakePulsing = false;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Alimentation DHT11 via GPIO
  pinMode(DHT_VCC_PIN, OUTPUT);
  digitalWrite(DHT_VCC_PIN, HIGH);
  delay(1000);
  dht.begin();

  Wire.begin();
  Wire.setClock(100000);

  // LED + WAKE pins
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  if (WAKE_PIN >= 0) {
    pinMode(WAKE_PIN, OUTPUT);
    digitalWrite(WAKE_PIN, LOW); // important: niveau stable au repos
  }

  Serial.println("MKR WAN 1310 - DHT11 + Lux + Threshold/Wake");
}

void loop() {
  // gère la fin d'impulsion WAKE si en cours
  updateWakePulse();

  if (millis() - lastTime >= timerDelay) {
    float t = dht.readTemperature();
    float h = dht.readHumidity();

    if (isnan(t) || isnan(h)) {
      Serial.println("Erreur lecture DHT11");
    } else {
      Serial.print("Temperature: ");
      Serial.print(t, 1);
      Serial.print(" °C | Humidity: ");
      Serial.print(h, 0);
      Serial.println(" %");
    }

    float lux = readLux();
    if (lux < 0) {
      Serial.println("Erreur lecture LUX");
    } else {
      Serial.print("LUX: ");
      Serial.print(lux, 1);
      Serial.println(" lx");

      // ====== THRESHOLD LOGIC (avec hystérésis + hold time) ======
      bool conditionNow = (lux <= LUX_THRESHOLD_ON);

      if (!alarmActive) {
        // on n'est pas en alerte : on attend que la condition soit vraie assez longtemps
        if (conditionNow) {
          if (condStartMs == 0) condStartMs = millis();
          if (millis() - condStartMs >= TRIGGER_HOLD_MS) {
            alarmActive = true;
            digitalWrite(LED_PIN, HIGH);

            if (!wakeAlreadySent) {
              startWakePulse();       // futur: réveil autre MCU
              wakeAlreadySent = true;
            }

            Serial.println(">>> ALERTE: seuil atteint (LED ON, WAKE pulse)");
          }
        } else {
          condStartMs = 0; // reset si la condition n'est plus vraie
        }
      } else {
        // on est en alerte : on ne coupe que si on dépasse le seuil OFF (hystérésis)
        if (lux >= LUX_THRESHOLD_OFF) {
          alarmActive = false;
          digitalWrite(LED_PIN, LOW);

          // prêt pour le prochain événement
          wakeAlreadySent = false;
          condStartMs = 0;

          Serial.println("<<< FIN ALERTE: retour au-dessus seuil OFF (LED OFF)");
        }
      }
      // ==========================================================
    }

    Serial.println("---");
    lastTime = millis();
  }
}