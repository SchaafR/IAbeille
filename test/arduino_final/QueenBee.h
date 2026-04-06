#ifndef HORNET_H
#define HORNET_H

#include <Arduino.h>
#include <PDM.h>
#include <queennoqueen.h> 

// 16000 Hz * 2 secondes = 32000 échantillons
#define AUDIO_BUFFER_SIZE 32000
static int16_t sampleBuffer[AUDIO_BUFFER_SIZE];
static volatile int samplesRead;
static volatile bool recordComplete = false;

class HornetDetector {
private:
    static void onPDMdata() {
        int bytesAvailable = PDM.available();
        int samplesToRead = bytesAvailable / 2;
        
        // On vérifie par rapport à la nouvelle taille de 32000
        if (samplesRead + samplesToRead <= AUDIO_BUFFER_SIZE) {
            PDM.read(sampleBuffer + samplesRead, bytesAvailable);
            samplesRead += samplesToRead;
        } else {
            recordComplete = true;
        }
    }

public:
    /**
     * Initialisation du micro
     */
    static bool begin(int gain = 20) {
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
     * Gère le cycle complet d'activation et de capture (2 secondes)
     */
    static bool capture(int gain = 20) {
        samplesRead = 0;
        recordComplete = false;

        if (!begin(gain)) {
            return false;
        }

        unsigned long startTimeout = millis();

        // Timeout augmenté à 3500ms pour laisser le temps aux 2s de se remplir
        while (!recordComplete) {
            if (millis() - startTimeout > 3500) {
                PDM.end();
                return false;
            }
            yield(); 
        }

        // Arrêt immédiat pour libérer les ressources
        PDM.end(); 
        return true;
    }

    /**
     * Exécute l'inférence sur les 32000 échantillons
     */
    static float getHornetScore() {
        ei::signal_t signal;
        // La longueur totale doit correspondre à 2 secondes (32000)
        signal.total_length = AUDIO_BUFFER_SIZE;
        signal.get_data = [](size_t offset, size_t length, float *out_ptr) -> int {
            for (size_t i = 0; i < length; i++) {
                out_ptr[i] = (float)sampleBuffer[offset + i];
            }
            return 0;
        };

        ei_impulse_result_t result = { 0 };
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
