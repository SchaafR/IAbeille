/*
 * Copyright (c) 2026 EdgeImpulse Inc.
 * Version fusionnée manuellement pour supporter les projets 938676 et 943803
 */

#ifndef _EI_CLASSIFIER_MODEL_METADATA_H_
#define _EI_CLASSIFIER_MODEL_METADATA_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "edge-impulse-sdk/classifier/ei_constants.h"

// Définitions standards du SDK
#define EI_CLASSIFIER_NONE                       255
#define EI_CLASSIFIER_UTENSOR                    1
#define EI_CLASSIFIER_TFLITE                     2
#define EI_CLASSIFIER_CUBEAI                     3
#define EI_CLASSIFIER_TFLITE_FULL                4
#define EI_CLASSIFIER_TENSAIFLOW                 5
#define EI_CLASSIFIER_TENSORRT                   6
#define EI_CLASSIFIER_DRPAI                      7
#define EI_CLASSIFIER_TFLITE_TIDL                8
#define EI_ANOMALY_TYPE_UNKNOWN                  0

#define EI_CLASSIFIER_SENSOR_MICROPHONE          1
#define EI_CLASSIFIER_DATATYPE_INT8              9

// --- CONFIGURATION DE FUSION DES DEUX MODÈLES ---

#define EI_CLASSIFIER_PROJECT_ID                 999999
#define EI_CLASSIFIER_PROJECT_NAME               "Fusion_IAbeilles_Final"

// 1. TAILLE D'ENTRÉE : On prend le max (3250 pour le frelon)
// Indispensable pour éviter le Buffer Overflow
#define EI_CLASSIFIER_NN_INPUT_FRAME_SIZE        3250 

// 2. ÉCHANTILLONS : On prend le max (2 secondes = 32000 samples)
// Pour que le modèle Reine (2s) puisse lire ses données complètes
#define EI_CLASSIFIER_RAW_SAMPLE_COUNT           32000
#define EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME      1
#define EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE       32000

// 3. FIX ERREUR "SIZE 26" : On force le calcul logiciel des FFT
// Supprime l'erreur "HW RFFT failed" sur la Nano 33 BLE
#define EI_CLASSIFIER_SOFTWARE_FFT_ONLY          1 
#define EI_CLASSIFIER_NON_STANDARD_FFT_SIZES     1

// 4. MÉMOIRE RAM (ARENA) : On prend le max des deux + marge
#define EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE  50000

// 5. PARAMÈTRES DE CALCUL
#define EI_CLASSIFIER_INTERVAL_MS                0.0625
#define EI_CLASSIFIER_FREQUENCY                  16000
#define EI_CLASSIFIER_NN_OUTPUT_COUNT            2
#define EI_CLASSIFIER_LABEL_COUNT                2
#define EI_CLASSIFIER_SENSOR                     EI_CLASSIFIER_SENSOR_MICROPHONE
#define EI_CLASSIFIER_QUANTIZATION_ENABLED       1

// 6. ACTIVATION DES MODULES FFT (Union des besoins)
#define EI_CLASSIFIER_HAS_FFT_INFO               1
#define EI_CLASSIFIER_LOAD_FFT_512               1
#define EI_CLASSIFIER_LOAD_FFT_1024              1

// --- FIN DE LA ZONE DE FUSION ---

#define EI_CLASSIFIER_INFERENCING_ENGINE         EI_CLASSIFIER_TFLITE
#define EI_CLASSIFIER_COMPILED                   1
#define EI_CLASSIFIER_TFLITE_INPUT_DATATYPE      EI_CLASSIFIER_DATATYPE_INT8
#define EI_CLASSIFIER_TFLITE_OUTPUT_DATATYPE     EI_CLASSIFIER_DATATYPE_INT8

// Typedefs indispensables pour le SDK Edge Impulse
typedef struct { const char *name; int axis; } ei_dsp_named_axis_t;
typedef struct { uint32_t block_id; uint16_t implementation_version; int axes; float scale_axes; bool average; bool minimum; bool maximum; bool rms; bool stdev; bool skewness; bool kurtosis; int moving_avg_num_windows; } ei_dsp_config_flatten_t;
typedef struct { uint32_t block_id; uint16_t implementation_version; int axes; ei_dsp_named_axis_t * named_axes; size_t named_axes_size; const char * channels; } ei_dsp_config_image_t;
typedef struct { uint32_t block_id; uint16_t implementation_version; int axes; ei_dsp_named_axis_t * named_axes; size_t named_axes_size; int num_cepstral; float frame_length; float frame_stride; int num_filters; int fft_length; int win_size; int low_frequency; int high_frequency; float pre_cof; int pre_shift; } ei_dsp_config_mfcc_t;
typedef struct { uint32_t block_id; uint16_t implementation_version; int axes; ei_dsp_named_axis_t * named_axes; size_t named_axes_size; float frame_length; float frame_stride; int num_filters; int fft_length; int low_frequency; int high_frequency; int win_size; int noise_floor_db; } ei_dsp_config_mfe_t;
typedef struct { uint32_t block_id; uint16_t implementation_version; int axes; float scale_axes; } ei_dsp_config_raw_t;
typedef struct { uint32_t block_id; uint16_t implementation_version; int axes; float scale_axes; int input_decimation_ratio; const char * filter_type; float filter_cutoff; int filter_order; const char * analysis_type; int fft_length; int spectral_peaks_count; float spectral_peaks_threshold; const char * spectral_power_edges; bool do_log; bool do_fft_overlap; int wavelet_level; const char * wavelet; bool extra_low_freq; } ei_dsp_config_spectral_analysis_t;
typedef struct { uint32_t block_id; uint16_t implementation_version; int axes; ei_dsp_named_axis_t * named_axes; size_t named_axes_size; float frame_length; float frame_stride; int fft_length; int noise_floor_db; bool show_axes; } ei_dsp_config_spectrogram_t;
typedef struct { int:0; } ei_post_processing_output_t;

#endif // _EI_CLASSIFIER_MODEL_METADATA_H_
