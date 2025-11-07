#pragma once
#include <stdint.h>

// ================== Constantes de tiempo (compartidas) ==================
constexpr uint32_t HALF_PERIOD_US      = 8333;   // 60 Hz
constexpr uint32_t ZC_DETECT_OFFSET    = 50;     // tu detección llega ~50 us tarde
constexpr uint32_t MAX_DELAY_FROM_ZC_US= HALF_PERIOD_US - ZC_DETECT_OFFSET;

constexpr uint32_t WORKING_TIME        = 6000;   // ventana útil después del ZC
constexpr uint32_t MIN_DELAY_US        = MAX_DELAY_FROM_ZC_US - WORKING_TIME;

static_assert(MIN_DELAY_US < MAX_DELAY_FROM_ZC_US, "MIN must be < MAX");

// Debounce por flanco del pulso de ZC
constexpr uint32_t ZC_RISE_DEBOUNCE_US = 200;
constexpr uint32_t ZC_FALL_DEBOUNCE_US = 200;

// ================== Parámetros ajustables (definidos en .cpp) ==================
extern int32_t  POT_MIN_COUNTS;          // p.ej. 200
extern int32_t  POT_MAX_COUNTS;          // p.ej. 20000
extern uint32_t DYNAMIC_CTRL_PERIOD_MS;  // p.ej. 50 ms

// Ventana del EMA para INA226 (ms)
extern uint32_t INA_EMA_WINDOW_MS;       // p.ej. 120

// ================== Telemetría compartida (definida en .cpp) ==================
extern volatile int32_t g_pot_raw;
extern volatile int32_t g_ina_bus_mV_avg;
extern volatile int32_t g_ina_shunt_uV_avg;
extern volatile bool    g_ina_ok;

// Timestamps de ZC (us) por fase, actualizados desde las ISRs
extern volatile uint64_t last_zc_rise_tick[3];
extern volatile uint64_t last_zc_fall_tick[3];


// ===================== Control de Corriente =====================
extern volatile float corriente_objetivo;        // Corriente objetivo en A
extern volatile float corriente_actual;          // Corriente medida en A
extern volatile uint32_t delay_actual;   // Delay actual en us
extern volatile bool control_corriente_activo;  // Modo de control
extern volatile const float CORRIENTE_MAXIMA; // 5000A máximo
extern volatile const float CORRIENTE_MINIMA;    // 0A mínimo

// Parámetros del controlador
extern volatile const uint32_t DELAY_MINIMO;      // us - mínimo delay seguro
extern volatile const uint32_t DELAY_MAXIMO; // us - máximo delay
extern volatile const uint32_t PASO_DELAY;          // us - paso de ajuste
extern volatile const float UMBRAL_CORRIENTE;    // A - tolerancia

extern volatile uint32_t MAX_DELAY_US;