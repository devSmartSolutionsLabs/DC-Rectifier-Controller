#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// 🔧 CONFIGURACIONES GENERALES
// ============================================================
#define NUM_DEVICES       3
#define SEMI_PERIOD_US    8333  // Semiperiodo 60 Hz

// ============================================================
// 🔄 FLAGS GLOBALES Y VARIABLES DE ESTADO
// ============================================================
extern bool verboseLog;

// === Sincronización ===
extern QueueHandle_t i2cQueue;
extern SemaphoreHandle_t i2cMutex;
extern esp_timer_handle_t startDelayTimer;

// === Control del sistema ===
extern volatile bool systemStarted;
extern volatile bool startRequested;
extern volatile uint32_t startRequestTime;
extern volatile bool testMode;
extern volatile bool direction;

// === Potenciómetro ===
extern volatile uint32_t potPercentage;
extern volatile bool potManualControl;
extern volatile int manualPotPercentage;

// === SCRs ===
extern volatile bool scrEnabled[NUM_DEVICES];
extern volatile int activeSCRsCount;

// === Estado general ===
extern volatile bool interruptsEnabled;
extern volatile bool pwmGenerationEnabled;
extern bool ioControlEnabled;

// === Señales analógicas y tiempos ===
extern float filteredPotVoltage;
extern uint32_t scrDelayUs;

#ifdef __cplusplus
}
#endif
