// GlobalVars.cpp
#include "GlobalVars.hpp"

esp_timer_handle_t startDelayTimer = nullptr;
QueueHandle_t i2cQueue = nullptr;
SemaphoreHandle_t i2cMutex = NULL;

// Definiciones ÚNICAS de todas las variables globales
volatile bool potManualControl = false;
volatile int manualPotPercentage = 50;
volatile bool startRequested = false;
volatile uint32_t startRequestTime = 0;
volatile uint32_t START_DELAY_MS = 3000;
volatile bool testMode = false;
volatile bool systemStarted = false;
volatile bool direction = false;
bool ioControlEnabled = false; 

// GlobalVars.cpp - AGREGAR estas variables
volatile bool scrEnabled[NUM_DEVICES] = {true, true, true}; // Todos activos por defecto
volatile int activeSCRsCount = 3; // Contador de SCRs activos

// ✅ ELIMINAR estas líneas porque ahora son #define
// const int NUM_DEVICES = 3;
// const int SEMI_PERIOD_US = 8333;