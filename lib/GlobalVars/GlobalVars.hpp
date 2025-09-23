#ifndef GLOBALDEFINITIONS_HPP
#define GLOBALDEFINITIONS_HPP

#include <Arduino.h>

// ✅ Cambiar constantes a #define para tiempo de compilación
constexpr int NUM_DEVICES = 3;
constexpr int SEMI_PERIOD_US = 8250;
constexpr int DEBOUNCE_TIME_US = 1000;

// Declaraciones extern de todas las variables globales
extern volatile bool potManualControl;
extern volatile int manualPotPercentage;
extern volatile bool startRequested;
extern volatile uint32_t startRequestTime;
extern esp_timer_handle_t startDelayTimer;
extern volatile uint32_t START_DELAY_MS;
extern volatile bool testMode;
extern volatile bool systemStarted;
extern volatile bool direction;
extern bool ioControlEnabled;  
extern volatile bool interruptsEnabled;
extern volatile bool pwmGenerationEnabled;


#endif