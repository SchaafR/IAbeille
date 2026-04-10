#define EI_CLASSIFIER_SOFTWARE_FFT_ONLY 1

#include "Sound.h"
#include "sound_lib.h"

/**
 * RÉFÉRENCES EXTERNES
 * Correspondant aux IDs : 938676_1 et 943803_2
 */
extern ei_impulse_handle_t impulse_handle_938676_1; 
extern const ei_impulse_t impulse_938676_1;

extern ei_impulse_handle_t impulse_handle_943803_2; 
extern const ei_impulse_t impulse_943803_2;

/**
 * Callback pour lire les données du sharedBuffer
 */
static int get_audio_signal_data(size_t offset, size_t length, float *out_ptr) {
    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = (float)sharedBuffer[offset + i];
    }
    return 0;
}

AudioResults run_all_inferences() {
    signal_t signal;
    ei_impulse_result_t result;
    AudioResults final_scores = {0.0f, 0.0f};
    
    signal.get_data = &get_audio_signal_data;

    // --- 1. INFÉRENCE FRELON (938676_1) ---
    signal.total_length = impulse_938676_1.dsp_input_frame_size;
    if (process_impulse(&impulse_handle_938676_1, &signal, &result, false) == EI_IMPULSE_OK) {
        for (uint16_t i = 0; i < impulse_938676_1.label_count; i++) {
            if (strcmp(result.classification[i].label, "hornet") == 0) {
                final_scores.hornet_score = result.classification[i].value;
            }
        }
    }

    // --- 2. INFÉRENCE REINE (943803_2) ---
    signal.total_length = impulse_943803_2.dsp_input_frame_size;
    if (process_impulse(&impulse_handle_943803_2, &signal, &result, false) == EI_IMPULSE_OK) {
        for (uint16_t i = 0; i < impulse_943803_2.label_count; i++) {
            if (strcmp(result.classification[i].label, "queen") == 0) {
                final_scores.queen_score = result.classification[i].value;
            }
        }
    }

    return final_scores;
}