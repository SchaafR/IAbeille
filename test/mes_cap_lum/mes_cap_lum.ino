#include <Wire.h>
#include <BH1750.h>
#include "DHT.h"

# define DHTPIN 2
# define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

BH1750 lightMeter;

void setup() {
  Serial.begin(9600);
  Wire.begin(); 
  dht.begin();
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    //Serial.println(F("BH1750 Initialisé"));
  } else {
    //Serial.println(F("Erreur de connexion au capteur"));
  }
}

void loop() {
  // --- Phase 1 : Mesure active ---
  //Serial.println("OTII_REPOS");
  delay(5000); // Observe la conso stable pendant la mesure continue
  //Serial.println("Mesure en cours...");
  float lux = lightMeter.readLightLevel();
  // Serial.print("Lumiere: ");
  // Serial.println(lux);

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  
  if (isnan(h) || isnan(t)) {
    Serial.println("Erreur de lecture !");

  } else {
    // Serial.print("Humidité: "); Serial.print(h); Serial.println("%");
    // Serial.print("Température: "); Serial.print(t); Serial.println("%");
  }
  //Serial.println("OTII_FIN_MESURE");


  // --- Phase 2 : Optionnel (si supporté par le capteur) ---
  //Certains modèles permettent de mettre le capteur en veille
  //Serial.println("En veille");
  //LightMeter.configure(BH1750::ONE_TIME_HIGH_RES_MODE);
}