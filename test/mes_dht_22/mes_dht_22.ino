# include "DHT.h"

# define DHTPIN 2
# define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  dht.begin();
}

void loop() {
  // Attendre 5 secondes pour bien voir le courant de repos sur Otii
  Serial.println("OTII_REPOS");
  delay(5000);

  // Phase de lecture (pic de consommation)
  Serial.println("OTII_LECTURE_ACTIVE");
  float h = dht.readHumidity();
  float t = dht.readTemperature();


  if (isnan(h) || isnan(t)) {
    Serial.println("Erreur de lecture !");
  } else {
    Serial.print("Humidité: "); Serial.print(h); Serial.println("%");
    Serial.print("Température: "); Serial.print(t); Serial.println("%");
  }
  Serial.println("OTII_FIN_MESURE");
}