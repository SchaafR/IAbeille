#ifndef HORNET_AUDIO_H
#define HORNET_AUDIO_H

#include <Arduino.h>
#include <PDM.h>
#include <hornets_lib_3.h> // Votre librairie Edge Impulse

class HornetAudio {
private:
    static inline int16_t sampleBuffer[16000]; 
    static inline volatile int samplesRead = 0;
    static inline volatile bool recordComplete = false; 

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
    static bool begin() {
        if (!PDM.begin(1, 16000)) {
            return false;
        }
        PDM.setGain(20); 
        PDM.onReceive(onPDMdata); 
        return true;
    }

    static float getHornetProbability() {
        samplesRead = 0; 
        recordComplete = false;
        unsigned long startTimeout = millis();

        // Capture d'une seconde d'audio 
        while (!recordComplete) {
            if (millis() - startTimeout > 2000) {
                PDM.end(); 
                begin();
                return -1.0; // Erreur micro
            }
            delay(1); 
        }

        // Préparation du signal pour l'inférence 
        ei::signal_t signal;
        signal.total_length = 16000;
        signal.get_data = [](size_t offset, size_t length, float *out_ptr) -> int {
            for (size_t i = 0; i < length; i++) {
                out_ptr[i] = (float)sampleBuffer[offset + i];
            }
            return 0;
        };

        // Lancement de l'inférence 
        ei_impulse_result_t result = { 0 };
        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

        if (res != EI_IMPULSE_OK) return -2.0; // Erreur inférence 

        // Extraction de la probabilité pour le label "hornet" 
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            if (strcmp(result.classification[ix].label, "hornet") == 0) { 
                return result.classification[ix].value; 
            }
        }
        return 0.0;
    }
};

#endif
