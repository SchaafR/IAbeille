#ifndef HORNET_H
#define HORNET_H

#include <Arduino.h>
#include <PDM.h>
#include <hornets_lib_3.h>

static int16_t sampleBuffer[16000];
static volatile int samplesRead;
static volatile bool recordComplete = false;

class HornetDetector {
private:
    static void onPDMdata() {
        int bytesAvailable = PDM.available();
        int samplesToRead = bytesAvailable / 2;
        
        if (samplesRead + samplesToRead <= 16000) {
            PDM.read(sampleBuffer + samplesRead, bytesAvailable);
            samplesRead += samplesToRead;
        } else {
            recordComplete = true;
        }
    }

public:
    /**
     * Initialisation du micro (utilisé en interne par capture)
     */
    static bool begin(int gain = 20) {
        // On s'assure qu'il est bien arrêté avant de démarrer
        PDM.end();
        delay(10); 
        
        if (!PDM.begin(1, 16000)) {
            return false;
        }
        PDM.setGain(gain);
        PDM.onReceive(onPDMdata);
        return true;
    }

    /**
     * Gère le cycle complet d'activation et de capture
     */
    static bool capture(int gain = 20) {
        samplesRead = 0;
        recordComplete = false;

        // 1. Démarrage du micro juste avant l'écoute
        if (!begin(gain)) {
            return false;
        }

        unsigned long startTimeout = millis();

        // 2. Attente de la capture
        while (!recordComplete) {
            if (millis() - startTimeout > 2500) {
                PDM.end();
                return false;
            }
            yield(); // Laisse le processeur gérer les interruptions
        }

        // 3. ARRÊT IMMÉDIAT du micro après capture
        // Libère les ressources pour l'inférence et évite le blocage
        PDM.end(); 
        return true;
    }

    /**
     * Exécute l'inférence sur les données capturées
     */
    static float getHornetScore() {
        ei::signal_t signal;
        signal.total_length = 16000;
        signal.get_data = [](size_t offset, size_t length, float *out_ptr) -> int {
            for (size_t i = 0; i < length; i++) {
                out_ptr[i] = (float)sampleBuffer[offset + i];
            }
            return 0;
        };

        ei_impulse_result_t result = { 0 };
        // Le 3ème paramètre 'false' désactive le debug pour gagner de la RAM
        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

        if (res != EI_IMPULSE_OK) {
            return -1.0;
        }

        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            if (strcmp(result.classification[ix].label, "hornet") == 0) {
                return result.classification[ix].value;
            }
        }
        return 0.0;
    }
};

#endif