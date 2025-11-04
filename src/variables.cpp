#include <cstdio>
#include "esp_attr.h"    // para DRAM_ATTR



constexpr uint32_t HALF_PERIOD_US   = 8333;  // 60 Hz
constexpr uint32_t WORKING_TIME   = 6000;  // 60 Hz
constexpr uint32_t ZC_DETECT_OFFSET = 50;   // tu ZC llega ~500us tarde vs ZC real (medido)
constexpr uint32_t MAX_DELAY_FROM_ZC_US = HALF_PERIOD_US - ZC_DETECT_OFFSET; // según tu osciloscopio

// Mantén tu mínimo (tu hardware lo usa)
constexpr uint32_t MIN_DELAY_US = MAX_DELAY_FROM_ZC_US - WORKING_TIME;  // ya lo tienes
static_assert(MIN_DELAY_US < MAX_DELAY_FROM_ZC_US, "MIN must be < MAX");

// (Opcional) debounce un poco más ajustado si tu ZC es limpio
// constexpr uint32_t DEBOUNCE_TIME_US  = 400;
// Rango de cuentas del potenciómetro (ADS1115 @ 0x48)
extern int32_t POT_MIN_COUNTS;          // ej. 200
extern int32_t POT_MAX_COUNTS;          // ej. 20000

// Periodo de iteración de la tarea
extern uint32_t DYNAMIC_CTRL_PERIOD_MS; // ej. 50

// Debounce específicos por flanco (menores a ~500us del pulso ZC)
constexpr uint32_t ZC_RISE_DEBOUNCE_US = 200;
constexpr uint32_t ZC_FALL_DEBOUNCE_US = 200;

static DRAM_ATTR volatile uint64_t last_zc_rise_tick[3] = {0,0,0};
static DRAM_ATTR volatile uint64_t last_zc_fall_tick[3] = {0,0,0};

static int32_t g_pot_raw = 0;        // ADS1115 counts
static int32_t g_bus_mV = 0;         // INA226 bus mV
static int32_t g_shunt_uV = 0;       // INA226 shunt uV