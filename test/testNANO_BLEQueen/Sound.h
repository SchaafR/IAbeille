#ifndef SOUND_H
#define SOUND_H

#include <Arduino.h>

// 2 secondes à 16kHz = 32000 samples
#define MAX_SAMPLES 32000

// Variables globales partagées entre le micro et l'IA
extern int16_t sharedBuffer[MAX_SAMPLES];
extern volatile int samplesRead;
extern volatile bool recordComplete;

// Structure pour transporter les scores vers la machine d'état
struct AudioResults {
    float hornet;
    float queen;
};

// Prototypes des fonctions
bool capture_audio(int duration_ms);
AudioResults run_all_inferences(); // Retourne bien un AudioResults

#endif