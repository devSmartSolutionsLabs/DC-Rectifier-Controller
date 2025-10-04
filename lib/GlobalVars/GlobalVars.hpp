#ifndef GLOBALDEFINITIONS_HPP
#define GLOBALDEFINITIONS_HPP

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "esp_timer.h"

// ================== CONSTANTES DE COMPILACIÓN ==================
#define NUM_DEVICES 3
#define SEMI_PERIOD_US 8333
#define DEBOUNCE_TIME_US 200
#define START_DELAY_MS 3000

// Umbrales para activación progresiva de SCRs
#define SCR_1_PHASE_THRESHOLD 33    // 0-33%: 1 SCR activo
#define SCR_2_PHASE_THRESHOLD 66    // 34-66%: 2 SCRs activos
// Más de 66%: 3 SCRs activos

// Direcciones I2C
#define ADS1115_ADDRESS_LOW 0x48
#define ADS1115_ADDRESS_HIGH 0x49

// Canales ADS1115
#define POT_CHANNEL 0
constexpr uint8_t CURRENT_CHANNELS[NUM_DEVICES] = {1, 2, 0}; // Ajusta según tu configuración

// ================== ESTRUCTURAS ==================
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

// ================== DECLARACIONES EXTERNAS ==================
// Mutex y Colas
extern QueueHandle_t i2cQueue;
extern SemaphoreHandle_t i2cMutex;

// Timers
extern esp_timer_handle_t startDelayTimer;

// Variables de control del sistema
extern volatile bool systemStarted;
extern volatile bool startRequested;
extern volatile uint32_t startRequestTime;
extern volatile bool testMode;
extern volatile bool direction;

// Variables de control de potencia
extern volatile uint32_t potPercentage;  // ✅ AGREGADA - falta en tu código
extern volatile bool potManualControl;
extern volatile int manualPotPercentage;

// Variables de control de SCRs
extern volatile bool scrEnabled[NUM_DEVICES];
extern volatile int activeSCRsCount;

// Variables de estado
extern volatile bool interruptsEnabled;
extern volatile bool pwmGenerationEnabled;
extern bool ioControlEnabled;

extern bool verboseLog;  // false = solo mensajes importantes
extern float filteredPotVoltage;
extern uint32_t scrDelayUs;


#endif