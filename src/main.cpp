#include <cmath>
#include <cstdio>
#include <stdarg.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/i2c.h"

#include "esp_timer.h"
#include "esp_intr_alloc.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "esp_log.h"

// ===================== Configuración general =====================
constexpr uint32_t ZC_PULSE_WIDTH_US = 20;
constexpr uint32_t DEBOUNCE_TIME_US  = 700;
constexpr uint32_t MIN_DELAY_US      = 6800;
constexpr uint32_t MAX_DELAY_US      = 8320;

constexpr gpio_num_t I2C_SDA_PIN = GPIO_NUM_5;
constexpr gpio_num_t I2C_SCL_PIN = GPIO_NUM_4;
constexpr i2c_port_t I2C_PORT    = I2C_NUM_0;

constexpr uint8_t ADS1115_ADDR_1 = 0x48;
constexpr uint8_t ADS1115_ADDR_2 = 0x49;
constexpr uint8_t MCP23017_ADDR  = 0x27;

// MCP23017
constexpr uint8_t MCP23017_IODIRA = 0x00;
constexpr uint8_t MCP23017_IODIRB = 0x01;
constexpr uint8_t MCP23017_GPPUB  = 0x0D;
constexpr uint8_t MCP23017_GPIOA  = 0x12;
constexpr uint8_t MCP23017_GPIOB  = 0x13;

// ADS1115
constexpr uint8_t  ADS1115_REG_CONVERSION   = 0x00;
constexpr uint8_t  ADS1115_REG_CONFIG       = 0x01;
constexpr uint16_t ADS1115_CONFIG_START     = 0xC1C3;
constexpr uint16_t ADS1115_CONFIG_DIFF_0_1  = 0xC583;

constexpr int32_t I2C_MAX_VALUE = 32767;
#define FILTER_SIZE 16

// ===================== Parámetros de pulsación =====================
static const uint64_t LONG_PRESS_MS  = 3000;
static const uint64_t MAX_GAP_TOL_MS = 800;   // tolerancia a huecos de lectura
static const uint64_t FAIL_DT_CAP_MS = 50;    // cap al dt cuando falla I2C

// ===================== Logger RAM (única impresión en monitor) =====================
enum LogLevel : uint8_t { L_INFO=0, L_WARN=1, L_ERROR=2 };

struct LogMsg {
    LogLevel level;
    const char* tag;
    char text[160];
};

static QueueHandle_t log_q = nullptr;

#define LOGI(TAG, FMT, ...) do { \
    if (log_q) { LogMsg _m{L_INFO, TAG, {0}}; \
        snprintf(_m.text, sizeof(_m.text), FMT, ##__VA_ARGS__); \
        xQueueSend(log_q, &_m, 0); } \
} while(0)
#define LOGW(TAG, FMT, ...) do { \
    if (log_q) { LogMsg _m{L_WARN, TAG, {0}}; \
        snprintf(_m.text, sizeof(_m.text), FMT, ##__VA_ARGS__); \
        xQueueSend(log_q, &_m, 0); } \
} while(0)
#define LOGE(TAG, FMT, ...) do { \
    if (log_q) { LogMsg _m{L_ERROR, TAG, {0}}; \
        snprintf(_m.text, sizeof(_m.text), FMT, ##__VA_ARGS__); \
        xQueueSend(log_q, &_m, 0); } \
} while(0)

static void logger_task(void*) {
    LogMsg m;
    for(;;){
        xQueueReceive(log_q, &m, portMAX_DELAY);
        // no-op: podrías contar métricas aquí si quieres
    }
}

// ===================== Sincronización global =====================
static SemaphoreHandle_t i2c_mutex   = NULL;
static SemaphoreHandle_t system_mutex= NULL;

// ===================== Timebase 1 MHz para debounce ZC =====================
static gptimer_handle_t zc_timebase = NULL;
static DRAM_ATTR volatile uint64_t last_zc_tick[3] = {0,0,0};

// ===================== Fases y control =====================
typedef struct {
    gpio_num_t zc_pin;
    gpio_num_t output_pin;
    volatile uint32_t delay_us;
    volatile uint64_t last_zc_time;
    gptimer_handle_t timer;
    volatile bool enabled;
    int phase_index;
} phase_config_t;

#define NUM_PHASES 3
constexpr int32_t RAW_V_MIN            = 15799;
constexpr int32_t RAW_V_MAX            = 17799;
constexpr int32_t ONE_PHASE_THRESHOLD  = 16999;
constexpr int32_t TWO_PHASE_THRESHOLD  = 17499;

static DRAM_ATTR phase_config_t phases[NUM_PHASES] = {
    { GPIO_NUM_38, GPIO_NUM_48, MAX_DELAY_US, 0, NULL, false, 0 },
    { GPIO_NUM_21, GPIO_NUM_47, MAX_DELAY_US, 0, NULL, false, 1 },
    { GPIO_NUM_14, GPIO_NUM_13, MAX_DELAY_US, 0, NULL, false, 2 }
};

// Filtros
static int32_t reading_buffer[FILTER_SIZE] = {0};
static int     buffer_index  = 0;
static float   voltage_buffer[FILTER_SIZE] = {0};
static int     voltage_index = 0;

// Estado
static bool     system_enabled = false;
static bool     system_ready   = false;
static bool     relay_a1_state = false;
static uint64_t button_start_press_time = 0;
static bool     button_start_pressed    = false;
static uint64_t system_activation_time  = 0;

// RMS
static float    current_rms_voltage = 0.0f;
static uint64_t last_rms_measurement = 0;
static const uint64_t RMS_MEASUREMENT_INTERVAL = 500; // ms

// Helpers tiempo
static inline uint64_t now_us() { return (uint64_t)esp_timer_get_time(); }
static inline uint64_t now_ms() { return now_us() / 1000ULL; }

// ===================== Forward decl =====================
static void zero_crossing_isr_handler(void* arg);
static bool scr_fire_timer_isr(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*);

bool     mcp23017_read_register_protected(uint8_t reg, uint8_t *value);
bool     mcp23017_write_register_protected(uint8_t reg, uint8_t value);
int32_t  ads1115_read_raw_protected();
void     i2c_master_init();
void     init_mcp23017();
void     initialize_phase(phase_config_t *phase, int timer_idx);
static   void init_zc_timebase_1mhz();

// ===================== I2C base =====================
bool mcp23017_write_register(uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MCP23017_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) { LOGE("I2C", "MCP W reg=0x%02X err=%d", reg, ret); return false; }
    return true;
}

bool mcp23017_read_register(uint8_t reg, uint8_t *value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MCP23017_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MCP23017_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) { LOGE("I2C", "MCP R reg=0x%02X err=%d", reg, ret); return false; }
    return true;
}

int32_t ads1115_read_raw() {
    uint8_t data[2];
    uint8_t config_buf[3] = { ADS1115_REG_CONFIG,
                              (uint8_t)(ADS1115_CONFIG_START >> 8),
                              (uint8_t)(ADS1115_CONFIG_START & 0xFF) };

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR_1 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, config_buf, 3, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) { LOGE("I2C", "ADS(0x48) cfg err=%d", ret); return 0; }

    vTaskDelay(pdMS_TO_TICKS(10));

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR_1 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, ADS1115_REG_CONVERSION, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR_1 << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) { LOGE("I2C", "ADS(0x48) conv err=%d", ret); return 0; }

    int16_t raw_value = (data[0] << 8) | data[1];
    return raw_value;
}

// ===================== Protegidas con mutex =====================
bool mcp23017_write_register_protected(uint8_t reg, uint8_t value) {
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bool ok = mcp23017_write_register(reg, value);
        xSemaphoreGive(i2c_mutex);
        return ok;
    }
    LOGW("I2C", "Timeout MCP W reg=0x%02X", reg);
    return false;
}

bool mcp23017_read_register_protected(uint8_t reg, uint8_t *value) {
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int retry = 0; retry < 3; retry++) {
            if (mcp23017_read_register(reg, value)) { xSemaphoreGive(i2c_mutex); return true; }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        xSemaphoreGive(i2c_mutex);
        i2c_driver_delete(I2C_PORT);
        vTaskDelay(pdMS_TO_TICKS(100));
        i2c_master_init();
        init_mcp23017();
        LOGW("I2C", "Bus recovery ejecutado");
    }
    return false;
}

int32_t ads1115_read_raw_protected() {
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        int32_t r = ads1115_read_raw();
        xSemaphoreGive(i2c_mutex);
        return r;
    }
    LOGW("I2C", "Timeout ADS(0x48) read");
    return 0;
}

// ===================== MCP y utilitarios =====================
void init_mcp23017() {
    mcp23017_write_register_protected(MCP23017_IODIRA, 0x00);
    mcp23017_write_register_protected(MCP23017_IODIRB, 0x03);
    mcp23017_write_register_protected(MCP23017_GPPUB,  0x03);
    mcp23017_write_register_protected(MCP23017_GPIOA,  0x00);
    LOGI("BOOT", "MCP23017 init A0,A1 out; B0,B1 in+pullup");
}

void update_relays() {
    uint8_t relay_state = 0x00;
    if (system_enabled)  relay_state |= 0x01;
    if (relay_a1_state)  relay_state |= 0x02;
    if (system_ready)    relay_state |= 0x08;
    mcp23017_write_register_protected(MCP23017_GPIOA, relay_state);
    LOGI("RELAYS", "A0=%s A1=%s A3=%s",
        system_enabled?"ON":"OFF",
        relay_a1_state?"ON":"OFF",
        system_ready?"ON":"OFF");
}

void update_phases_based_on_potentiometer(int32_t filtered_value) {
    if (!system_ready) {
        for (int i=0;i<NUM_PHASES;i++) phases[i].enabled = false;
        return;
    }

    if (relay_a1_state) {
        if (filtered_value <= ONE_PHASE_THRESHOLD) {
            phases[0].enabled=false; phases[1].enabled=false; phases[2].enabled=true;
            LOGI("MODE","1 fase (B) v=%ld", filtered_value);
        } else if (filtered_value <= TWO_PHASE_THRESHOLD) {
            phases[0].enabled=false; phases[1].enabled=true; phases[2].enabled=true;
            LOGI("MODE","2 fases (B,C) v=%ld", filtered_value);
        } else {
            phases[0].enabled=true; phases[1].enabled=true; phases[2].enabled=true;
            LOGI("MODE","3 fases (A,B,C) v=%ld", filtered_value);
        }
    } 
    ////////////
    else {
        if (filtered_value <= ONE_PHASE_THRESHOLD) {
            phases[0].enabled=false; phases[1].enabled=false; phases[2].enabled=true;
            LOGI("MODE","1 fase (B) v=%ld", filtered_value);
        } else if (filtered_value <= TWO_PHASE_THRESHOLD) {
            phases[0].enabled=false; phases[1].enabled=true; phases[2].enabled=true;
            LOGI("MODE","2 fases (B,C) v=%ld", filtered_value);
        } else {
            phases[0].enabled=true; phases[1].enabled=true; phases[2].enabled=true;
            LOGI("MODE","3 fases (A,B,C) v=%ld", filtered_value);
        }
    }
    /*else {
        phases[0].enabled= true; phases[1].enabled=true; phases[2].enabled=true;
        LOGI("MODE","Reversa 2 fases (B,C) v=%ld", filtered_value);
    }*/
}

void read_buttons() {
    static uint64_t last_ms = 0, hold_ms = 0, gap_ms = 0;
    static bool last_b0 = false;
    static bool last_known_b0 = false;
    static bool last_known_b1 = false;

    const uint64_t now = now_ms();
    uint64_t dt = (last_ms==0)? 0 : (now - last_ms);
    last_ms = now;

    // Lee MCP; si falla, usa últimos estados válidos y limita dt
    uint8_t pb = 0;
    bool ok = mcp23017_read_register_protected(MCP23017_GPIOB, &pb);

    bool b0_active, b1_active;
    if (ok) {
        b0_active = !(pb & 0x01);  // START activo en 0
        b1_active = !(pb & 0x02);  // DIRECCIÓN activo en 0
        last_known_b0 = b0_active;
        last_known_b1 = b1_active;
    } else {
        b0_active = last_known_b0;
        b1_active = last_known_b1;
        if (dt > FAIL_DT_CAP_MS) dt = FAIL_DT_CAP_MS;
    }

    // Acumulador tolerante a huecos para long-press
    if (b0_active) {
        if (gap_ms <= MAX_GAP_TOL_MS) hold_ms += dt;
        else                          hold_ms  = 0;
        gap_ms = 0;
    } else {
        gap_ms += dt;
        if (gap_ms > MAX_GAP_TOL_MS) hold_ms = 0;
    }

    // Logs de flanco
    if (b0_active && !last_b0) LOGI("BTN","START down");
    if (!b0_active && last_b0) LOGI("BTN","START up");
    last_b0 = b0_active;

    // ACTIVAR: mantener START 3 s
    if (!system_enabled && hold_ms >= LONG_PRESS_MS) {
        system_enabled = true;     // A0 ON
        system_ready   = false;    // A3 OFF
        relay_a1_state = !b1_active;
        system_activation_time = now;
        update_relays();
        LOGI("STATE","ACTIVADO (%.1fs)", (double)hold_ms/1000.0);
        hold_ms = 0;
        gap_ms  = 0;
    }

    // Habilitar pulsos 2 s después de activar
    if (system_enabled && !system_ready) {
        if ((now - system_activation_time) >= 2000) {
            system_ready = true;   // A3 ON
            update_relays();
            LOGI("STATE","LISTO, pulsos habilitados");
        }
    }

    // Dirección: permite SOLO cuando listo (puedes cambiar a 'system_enabled' si quieres el comportamiento antiguo)
    if (system_enabled && system_ready) {
        bool new_a1 = !b1_active;
        if (new_a1 != relay_a1_state) { relay_a1_state = new_a1; update_relays(); }
    } else if (!system_enabled && relay_a1_state) {
        relay_a1_state = false; update_relays();
    }

    // APAGADO SEGURO (forma de tu código antiguo):
    // Al SOLTAR START de verdad: 1) Apaga SCRs y A3; 2) espera 2 s; 3) Apaga A0
    static bool shutting_down = false;
    static uint64_t off_deadline = 0;

    // Paso 2 y 3 en curso
    if (shutting_down) {
        if (now >= off_deadline) {
            system_enabled = false;   // A0 OFF tras 2 s
            update_relays();
            LOGI("STATE","Apagado completo: A0 OFF");
            shutting_down = false;
        }
        return; // mientras apagas, no proceses más lógica
    }

    // Detecta SOLTAR START real con sistema activo
    if (system_enabled && !b0_active && gap_ms > MAX_GAP_TOL_MS) {
        LOGI("STATE","Inicio apagado seguro: A3 OFF y SCRs OFF");
        system_ready = false; // A3 OFF

        // Apaga pulsos inmediatamente
        for (int i=0;i<NUM_PHASES;i++){
            phases[i].enabled = false;
            gpio_set_level(phases[i].output_pin, 0);
            gptimer_stop(phases[i].timer);
        }
        update_relays();

        // Programa A0 OFF en 2 s
        shutting_down = true;
        off_deadline  = now + 2000;

        // limpia acumuladores de botón
        hold_ms = 0;
        return;
    }
}



// ===================== Filtro =====================
int32_t get_i2c_filtered_value(int32_t raw_value) {
    int32_t current_reading = raw_value;
    if (current_reading == 0 && buffer_index > 0)
        return reading_buffer[(buffer_index - 1 + FILTER_SIZE) % FILTER_SIZE];

    reading_buffer[buffer_index] = current_reading;
    buffer_index = (buffer_index + 1) % FILTER_SIZE;

    int64_t sum = 0;
    for (int i=0;i<FILTER_SIZE;i++) sum += reading_buffer[i];
    int32_t filtered_value = (int32_t)(sum / FILTER_SIZE);

    if (filtered_value < RAW_V_MIN) return RAW_V_MIN;
    if (filtered_value > RAW_V_MAX) return RAW_V_MAX;
    return filtered_value;
}

// ===================== ISR SCR timer =====================
static bool IRAM_ATTR scr_fire_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t*, void *user_ctx) {
    phase_config_t *phase = (phase_config_t *)user_ctx;
    if (!phase->enabled) { gptimer_stop(timer); return false; }

    int zc_state = gpio_get_level(phase->zc_pin);
    if (zc_state == 0) {
        gpio_set_level(phase->output_pin, 1);
    } else {
        gptimer_alarm_config_t alarm_config = { .alarm_count = 2, .reload_count = 0, .flags = {0} };
        gptimer_set_alarm_action(timer, &alarm_config);
        gptimer_set_raw_count(timer, 0);
        gptimer_start(timer);
        gpio_set_level(phase->output_pin, 0);
    }
    return false;
}

// ===================== ISR ZC sin APIs no-IRAM =====================
static void IRAM_ATTR zero_crossing_isr_handler(void* arg) {
    phase_config_t *phase = (phase_config_t *)arg;
    uint64_t now_tick = 0;
    gptimer_get_raw_count(zc_timebase, &now_tick); // 1 tick = 1 us

    uint64_t last = last_zc_tick[phase->phase_index];
    if ((uint64_t)(now_tick - last) < (uint64_t)DEBOUNCE_TIME_US) return;
    last_zc_tick[phase->phase_index] = now_tick;

    gpio_set_level(phase->output_pin, 0);
    gptimer_stop(phase->timer);

    if (phase->enabled) {
        uint32_t current_delay = phase->delay_us;
        if (current_delay < MIN_DELAY_US) current_delay = MIN_DELAY_US;
        if (current_delay > MAX_DELAY_US) current_delay = MAX_DELAY_US;

        gptimer_alarm_config_t alarm_config = { .alarm_count = (uint64_t)current_delay,
                                                .reload_count = 0, .flags = {0} };
        gptimer_set_alarm_action(phase->timer, &alarm_config);
        gptimer_set_raw_count(phase->timer, 0);
        gptimer_start(phase->timer);
    }
}

// ===================== Tareas =====================
void button_control_task(void*) {
    esp_task_wdt_add(NULL);
    const TickType_t period = pdMS_TO_TICKS(20); // más rápido para captar rebotes
    for(;;){
        esp_task_wdt_reset();
        read_buttons();
        vTaskDelay(period);
    }
}

void dynamic_control_task(void*) {
    esp_task_wdt_add(NULL);
    uint32_t delay_range = MAX_DELAY_US - MIN_DELAY_US;

    for (int i=0;i<FILTER_SIZE;i++){
        reading_buffer[i] = ads1115_read_raw_protected();
        voltage_buffer[i] = 0.0f;
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    for(;;){
        esp_task_wdt_reset();
        uint64_t t = now_ms();

        int32_t raw_value = ads1115_read_raw_protected();
        int32_t i2c_value_filtered = get_i2c_filtered_value(raw_value);
        if (i2c_value_filtered < RAW_V_MIN) i2c_value_filtered = RAW_V_MIN;
        if (i2c_value_filtered > RAW_V_MAX) i2c_value_filtered = RAW_V_MAX;

        if ((int64_t)(t - last_rms_measurement) >= (int64_t)RMS_MEASUREMENT_INTERVAL) {
            float new_rms = 0.0f; // desactivada
            if (new_rms >= 0) current_rms_voltage = new_rms;
            last_rms_measurement = t;
        }

        voltage_buffer[voltage_index] = current_rms_voltage;
        voltage_index = (voltage_index + 1) % FILTER_SIZE;

        float voltage_sum = 0; int valid_voltage_samples=0;
        for (int i=0;i<FILTER_SIZE;i++){ if (voltage_buffer[i]>=0){ voltage_sum+=voltage_buffer[i]; valid_voltage_samples++; } }
        float diff_voltage_filtered = (valid_voltage_samples>0)? voltage_sum/valid_voltage_samples : 0.0f;
        (void)diff_voltage_filtered;

        float ratio_saturated = 0.0f;
        if (relay_a1_state) {
            if (i2c_value_filtered <= ONE_PHASE_THRESHOLD) {
                ratio_saturated = (float)(i2c_value_filtered-RAW_V_MIN)/(ONE_PHASE_THRESHOLD-RAW_V_MIN);
            } else if (i2c_value_filtered <= TWO_PHASE_THRESHOLD) {
                ratio_saturated = (float)(35 + i2c_value_filtered-ONE_PHASE_THRESHOLD)/(TWO_PHASE_THRESHOLD-ONE_PHASE_THRESHOLD);
            } else {
                ratio_saturated = (float)(200 + i2c_value_filtered-TWO_PHASE_THRESHOLD)/(RAW_V_MAX-TWO_PHASE_THRESHOLD);
            }
        } else {
            ratio_saturated = (float)(i2c_value_filtered-RAW_V_MIN)/(RAW_V_MAX-RAW_V_MIN);
        }
        if (ratio_saturated < 0.0f) ratio_saturated = 0.0f;
        if (ratio_saturated > 1.0f) ratio_saturated = 1.0f;

        uint32_t new_delay_base = (uint32_t)((1.0f - ratio_saturated) * delay_range) + MIN_DELAY_US;

        if (system_enabled) {
            phases[0].delay_us = new_delay_base;
            phases[1].delay_us = new_delay_base;
            phases[2].delay_us = new_delay_base;
            update_phases_based_on_potentiometer(i2c_value_filtered);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Monitor: ÚNICA tarea que imprime
void system_health_monitor(void*){
    printf("[HEALTH] Logger RAM activo. Sin persistencia.\n");
    static uint32_t sample = 0;
    for(;;){
        uint8_t pb=0; bool ok_btn = mcp23017_read_register_protected(MCP23017_GPIOB, &pb);
        int b0 = ok_btn ? !(pb&0x01) : -1;
        int b1 = ok_btn ? !(pb&0x02) : -1;

        printf(
            "[HEALTH %lu] ok=%d rawB=0x%02X | B0=%d B1=%d | en=%d ready=%d dir=%d | "
            "delayA=%lu delayB=%lu delayC=%lu\n",
            (unsigned long)sample++,
            (int)ok_btn,
            (unsigned)pb,
            b0, b1,
            (int)system_enabled,
            (int)system_ready,
            (int)relay_a1_state,
            (unsigned long)phases[0].delay_us,
            (unsigned long)phases[1].delay_us,
            (unsigned long)phases[2].delay_us
        );

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ===================== Timebase 1 MHz =====================
static void init_zc_timebase_1mhz() {
    gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
        .intr_priority = 3,
        .flags = { .intr_shared=false, .allow_pd=false, .backup_before_sleep=false }
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &zc_timebase));
    ESP_ERROR_CHECK(gptimer_enable(zc_timebase));
    ESP_ERROR_CHECK(gptimer_start(zc_timebase));
}

// ===================== Inicialización de fase =====================
void initialize_phase(phase_config_t *phase, int timer_idx) {
    gpio_config_t zc_config = {
        .pin_bit_mask = (1ULL << phase->zc_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&zc_config);

    gpio_config_t output_config = {
        .pin_bit_mask = (1ULL << phase->output_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&output_config);
    gpio_set_level(phase->output_pin, 0);

    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
        .intr_priority = 2,
        .flags = { .intr_shared=false, .allow_pd=false, .backup_before_sleep=false }
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &phase->timer));
    gptimer_event_callbacks_t cbs = { .on_alarm = scr_fire_timer_isr };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(phase->timer, &cbs, phase));
    ESP_ERROR_CHECK(gptimer_enable(phase->timer));

    phase->phase_index = timer_idx;
    gpio_isr_handler_add(phase->zc_pin, zero_crossing_isr_handler, (void*)phase);

    LOGI("BOOT","Fase %c ZC=%d SCR=%d timer=%d",
         (phase==&phases[0]?'A':(phase==&phases[1]?'B':'C')),
         (int)phase->zc_pin, (int)phase->output_pin, timer_idx);
}

// ===================== I2C init / Enables =====================
void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = { .clk_speed = 100000 },
        .clk_flags = 0
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
    LOGI("BOOT","I2C master SDA=%d SCL=%d", I2C_SDA_PIN, I2C_SCL_PIN);
}

void initialize_mcp_enables() {
    const gpio_num_t pin_15 = GPIO_NUM_15;
    const gpio_num_t pin_41 = GPIO_NUM_41;

    gpio_config_t io_config = {
        .pin_bit_mask = (1ULL << pin_15) | (1ULL << pin_41),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_config);

    gpio_set_level(pin_15, 1);
    gpio_set_level(pin_41, 1);
    LOGI("BOOT","Enables MCP: GPIO15=1 GPIO41=1");
}

// ===================== app_main =====================
extern "C" void app_main(void) {
    esp_log_level_set("*", ESP_LOG_WARN); // silencia IDF por consola

    // WDT
    esp_task_wdt_config_t twdt_config = { .timeout_ms = 60000, .idle_core_mask = 0, .trigger_panic = true };
    esp_task_wdt_init(&twdt_config);

    // Logger RAM
    log_q = xQueueCreate(64, sizeof(LogMsg));
    xTaskCreate(logger_task, "Logger", 4096, NULL, 2, NULL);

    // Mutex
    i2c_mutex    = xSemaphoreCreateMutex();
    system_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL || system_mutex == NULL) {
        printf("[HEALTH] ERROR creando mutex\n");
        return;
    }

    initialize_mcp_enables();
    i2c_master_init();
    init_mcp23017();

    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    init_zc_timebase_1mhz();

    initialize_phase(&phases[0], 0);
    initialize_phase(&phases[1], 1);
    initialize_phase(&phases[2], 2);

    xTaskCreate(dynamic_control_task, "DynamicControl", 8192, NULL, 5, NULL);
    xTaskCreate(button_control_task,  "ButtonControl",  6144, NULL, 6, NULL);
    xTaskCreate(system_health_monitor,"HealthMonitor",  4096, NULL, 1, NULL);

    esp_task_wdt_add(NULL);

    for(;;){
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
