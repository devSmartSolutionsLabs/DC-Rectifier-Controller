#include "variables.hpp"
#include "esp_attr.h"   // para DRAM_ATTR

// ================== Parámetros ajustables (valores por defecto) ==================
int32_t  POT_MIN_COUNTS         = 3500;
int32_t  POT_MAX_COUNTS         = 20000;
uint32_t DYNAMIC_CTRL_PERIOD_MS = 50;
volatile uint32_t MAX_DELAY_US  = 8315;


uint32_t INA_EMA_WINDOW_MS      = 200;   // ventana EMA para promediar INA226

// ================== Telemetría compartida ==================
volatile int32_t g_pot_raw          = 0;
volatile int32_t g_ina_bus_mV_avg   = 0;
volatile int32_t g_ina_shunt_uV_avg = 0;
volatile bool    g_ina_ok           = false;

// ================== Timestamps de ZC (DRAM) ==================
volatile uint64_t last_zc_rise_tick[3] = {0, 0, 0};
volatile uint64_t last_zc_fall_tick[3] = {0, 0, 0};

// ===================== Control de Corriente =====================
volatile float corriente_objetivo = 0.0f;        // Corriente objetivo en A
volatile float corriente_actual = 0.0f;          // Corriente medida en A
volatile uint32_t delay_actual = MAX_DELAY_US;   // Delay actual en us
volatile bool control_corriente_activo = false;  // Modo de control
volatile const float CORRIENTE_MAXIMA = 5000.0f; // 5000A máximo
volatile const float CORRIENTE_MINIMA = 0.0f;    // 0A mínimo

// Parámetros del controlador
volatile const uint32_t DELAY_MINIMO = 7800;      // us - mínimo delay seguro
volatile const uint32_t DELAY_MAXIMO = MAX_DELAY_US; // us - máximo delay
volatile const uint32_t PASO_DELAY = 1;          // us - paso de ajuste
volatile const float UMBRAL_CORRIENTE = 0.1f;    // A - tolerancia

