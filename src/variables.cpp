#include "variables.hpp"
#include "esp_attr.h"   // para DRAM_ATTR

// ================== Parámetros ajustables (valores por defecto) ==================
int32_t  POT_MIN_COUNTS         = 200;
int32_t  POT_MAX_COUNTS         = 20000;
uint32_t DYNAMIC_CTRL_PERIOD_MS = 50;


uint32_t INA_EMA_WINDOW_MS      = 200;   // ventana EMA para promediar INA226

// ================== Telemetría compartida ==================
volatile int32_t g_pot_raw          = 0;
volatile int32_t g_ina_bus_mV_avg   = 0;
volatile int32_t g_ina_shunt_uV_avg = 0;
volatile bool    g_ina_ok           = false;

// ================== Timestamps de ZC (DRAM) ==================
DRAM_ATTR volatile uint64_t last_zc_rise_tick[3] = {0,0,0};
DRAM_ATTR volatile uint64_t last_zc_fall_tick[3] = {0,0,0};
