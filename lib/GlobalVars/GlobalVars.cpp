// GlobalVars.cpp
#include "GlobalVars.hpp"

// Definiciones ÚNICAS de todas las variables globales
volatile bool potManualControl = false;
volatile int manualPotPercentage = 50;
volatile bool startRequested = false;
volatile uint32_t startRequestTime = 0;
esp_timer_handle_t startDelayTimer;
volatile uint32_t START_DELAY_MS = 3000;
volatile bool testMode = false;
volatile bool systemStarted = false;
volatile bool direction = false;
bool ioControlEnabled = false; 


// ✅ ELIMINAR estas líneas porque ahora son #define
// const int NUM_DEVICES = 3;
// const int SEMI_PERIOD_US = 8333;