#ifndef HORNET_AUDIO_H
#define HORNET_AUDIO_H

#include <Arduino.h>
#include <PDM.h>
#include <hornets_lib_3.h> // Votre librairie Edge Impulse [cite: 627]

class HornetAudio {
private:
    static inline int16_t sampleBuffer[16000]; [cite: 627]
    static inline volatile int samplesRead = 0; [cite: 627]
    static inline volatile bool recordComplete = false; [cite: 627]

    static void onPDMdata() { [cite: 632]
        int bytesAvailable = PDM.available(); [cite: 632]
        int samplesToRead = bytesAvailable / 2; [cite: 633]
        if (samplesRead + samplesToRead <= 16000) { [cite: 633]
            PDM.read(sampleBuffer + samplesRead, bytesAvailable); [cite: 633]
            samplesRead += samplesToRead; [cite: 634]
        } else {
            recordComplete = true; [cite: 634]
        }
    }

public:
    static bool begin() {
        if (!PDM.begin(1, 16000)) { [cite: 629]
            return false;
        }
        PDM.setGain(20); [cite: 630]
        PDM.onReceive(onPDMdata); [cite: 630]
        return true;
    }

    static float getHornetProbability() {
        samplesRead = 0; [cite: 635]
        recordComplete = false; [cite: 636]
        unsigned long startTimeout = millis();

        // Capture d'une seconde d'audio [cite: 635, 640]
        while (!recordComplete) {
            if (millis() - startTimeout > 2000) { [cite: 637]
                PDM.end(); [cite: 638]
                begin();
                return -1.0; // Erreur micro
            }
            delay(1); [cite: 639]
        }

        // Préparation du signal pour l'inférence [cite: 641, 644]
        ei::signal_t signal;
        signal.total_length = 16000; [cite: 641]
        signal.get_data = [](size_t offset, size_t length, float *out_ptr) -> int {
            for (size_t i = 0; i < length; i++) {
                out_ptr[i] = (float)sampleBuffer[offset + i]; [cite: 643]
            }
            return 0;
        };

        // Lancement de l'inférence [cite: 647]
        ei_impulse_result_t result = { 0 }; [cite: 646]
        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false); [cite: 647]

        if (res != EI_IMPULSE_OK) return -2.0; // Erreur inférence [cite: 648]

        // Extraction de la probabilité pour le label "hornet" [cite: 650]
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            if (strcmp(result.classification[ix].label, "hornet") == 0) { [cite: 650]
                return result.classification[ix].value; [cite: 650]
            }
        }
        return 0.0;
    }
};

#endif
