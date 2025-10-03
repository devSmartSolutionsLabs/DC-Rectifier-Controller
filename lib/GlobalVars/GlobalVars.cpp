#include "GlobalVars.hpp"

// ================== DEFINICIÓN DE MUTEX Y COLAS ==================
QueueHandle_t i2cQueue = nullptr;
SemaphoreHandle_t i2cMutex = nullptr;
esp_timer_handle_t startDelayTimer = nullptr;

// ================== DEFINICIÓN DE VARIABLES DE CONTROL ==================
volatile bool systemStarted = false;
volatile bool startRequested = false;
volatile uint32_t startRequestTime = 0;
volatile bool testMode = false;
volatile bool direction = false;

// ================== DEFINICIÓN DE VARIABLES DE POTENCIA ==================
volatile uint32_t potPercentage = 0;  // ✅ AGREGADA - inicializada en 0
volatile bool potManualControl = false;
volatile int manualPotPercentage = 50;

// ================== DEFINICIÓN DE VARIABLES DE SCRs ==================
volatile bool scrEnabled[NUM_DEVICES] = {true, true, true}; // Todos activos por defecto
volatile int activeSCRsCount = 3; // Contador de SCRs activos

// ================== DEFINICIÓN DE VARIABLES DE ESTADO ==================
volatile bool interruptsEnabled = true;
volatile bool pwmGenerationEnabled = true;
bool ioControlEnabled = false;
