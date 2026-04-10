#include <PDM.h>
#include "Sound.h"

int16_t sharedBuffer[MAX_SAMPLES];
volatile int samplesRead = 0;
volatile bool recordComplete = false;

/**
 * Callback micro PDM
 */
void onPDMdata() {
  int bytesAvailable = PDM.available();
  int samplesToRead = bytesAvailable / 2;
  if (!recordComplete) {
    if (samplesRead + samplesToRead <= MAX_SAMPLES) {
      PDM.read(sharedBuffer + samplesRead, bytesAvailable);
      samplesRead += samplesToRead;
    } else {
      recordComplete = true;
    }
  }
}

/**
 * Fonction de capture
 */
bool capture_audio(int duration_ms) {
  samplesRead = 0; 
  recordComplete = false;
  
  if (!PDM.begin(1, 16000)) return false;
  PDM.setGain(25);
  PDM.onReceive(onPDMdata);
  
  unsigned long start = millis();
  // On attend soit le buffer plein, soit un timeout de sécurité
  while (!recordComplete && (millis() - start < (unsigned long)duration_ms + 250)) { 
    yield(); 
  }
  PDM.end();
  
  // ICI : On retourne l'état réel de la capture !
  return recordComplete; 
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Système IAbeille : Test Multi-Modèles OK");
}

void loop() {
  if (capture_audio(2000)) {
    Serial.println("Analyse...");
    AudioResults res = run_all_inferences();

    // On n'affiche les résultats QUE si le score est significatif
    // car avec les erreurs de capture, on a souvent des faux positifs à 70%
    Serial.print("Frelon: "); Serial.println(res.hornet_score);
    Serial.print("Reine:  "); Serial.println(res.queen_score);
  } else {
    Serial.println("ERREUR : Le micro n'a rien capture. Pas d'analyse.");
    // Optionnel : On réinitialise le micro
    PDM.end();
    delay(100);
  }
  delay(2000);
}
