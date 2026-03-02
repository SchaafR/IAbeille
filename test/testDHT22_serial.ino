#include <Wire.h>
#include <DHT.h>

#define LUX_ADDR 0x23

// DHT11
#define DHTPIN 4          // D4
#define DHTTYPE DHT11
#define DHT_VCC_PIN 6     // D6 fournit le 3.3V via GPIO

DHT dht(DHTPIN, DHTTYPE);

unsigned long lastTime = 0;
const unsigned long timerDelay = 2000;

float readLux() {
  // Power on
  Wire.beginTransmission(LUX_ADDR);
  Wire.write(0x01);
  if (Wire.endTransmission() != 0) return -1;
  delay(10);

  // One-Time High Resolution Mode
  Wire.beginTransmission(LUX_ADDR);
  Wire.write(0x20);
  if (Wire.endTransmission() != 0) return -1;

  delay(250);

  uint8_t n = Wire.requestFrom(LUX_ADDR, (uint8_t)2);
  if (n != 2) return -1;

  uint16_t raw = ((uint16_t)Wire.read() << 8) | Wire.read();
  return raw / 1.2f;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Alimentation DHT11 via GPIO
  pinMode(DHT_VCC_PIN, OUTPUT);
  pinMode(1, OUTPUT);
  digitalWrite(DHT_VCC_PIN, HIGH);   // ~3.3V
  //BUZZER 
  digitalWrite(1, HIGH);   // ~3.3V
  delay(500);
  digitalWrite(1, LOW);   // ~3.3V

  delay(1000);                       // temps de démarrage capteur
  
  dht.begin();

  Wire.begin();                      // MKR: SDA/SCL automatiques
  Wire.setClock(100000);             // conseillé avec câble long

  Serial.println("MKR WAN 1310 - DHT11 + Lux");
}

void loop() {
  if (millis() - lastTime >= timerDelay) {
    // --- DHT11 ---
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

    // --- Lux ---
    float lux = readLux();
    if (lux < 0) {
      Serial.println("Erreur lecture LUX");
    } else {
      Serial.print("LUX: ");
      Serial.print(lux, 1);
      Serial.println(" lx");
    }

    Serial.println("---");
    lastTime = millis();
  }
}