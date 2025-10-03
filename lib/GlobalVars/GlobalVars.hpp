#ifndef GLOBALDEFINITIONS_HPP
#define GLOBALDEFINITIONS_HPP

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "esp_timer.h"

enum I2CDeviceType { DEV_ADS1115, DEV_MCP23017 };

struct I2CRequest {
    I2CDeviceType device;
    uint8_t address;
    uint8_t reg;
    uint8_t value;
    float* resultF;     // para lecturas float (ADS)
    uint8_t* resultB;   // para lecturas byte (MCP)
    bool isWrite;
};

extern QueueHandle_t i2cQueue;
extern SemaphoreHandle_t i2cMutex;  // declaración para otros archivos

// Función segura para tomar mutex
bool takeI2CMutex(TickType_t timeout = pdMS_TO_TICKS(100));
void giveI2CMutex();

// ✅ Cambiar constantes a #define para tiempo de compilación
constexpr int NUM_DEVICES = 3;
constexpr int SEMI_PERIOD_US = 8333;
constexpr int DEBOUNCE_TIME_US = 200;

// Nuevas constantes para activación progresiva de SCRs
constexpr int SCR_1_PHASE_THRESHOLD = 0;    // 0-5%: 1 SCR activo
constexpr int SCR_2_PHASE_THRESHOLD = 1;   // 5-10%: 2 SCRs activos
// Más de 10%: 3 SCRs activos

// Variables para control de SCRs activos
extern volatile bool scrEnabled[NUM_DEVICES];
extern volatile int activeSCRsCount;

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