#include <cmath>
#include <cstdio>
#include <stdarg.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/i2c.h"

#include "esp_timer.h"
#include "esp_intr_alloc.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "esp_log.h"

#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "http_ota.hpp"
#include "ads1115.hpp"
#include "variables.cpp"
#include "INA226.hpp"

static ADS1115* adc0 = nullptr; // 0x48
static ADS1115* adc1 = nullptr; // 0x49

static INA226* ina = nullptr;
constexpr uint8_t INA226_ADDR = 0x40; // A0=A1=GND por defecto

// ===================== Wi-Fi util =====================
static EventGroupHandle_t s_wifi_eg = nullptr;
#define WIFI_GOT_IP BIT0

static void ip_event_handler(void*, esp_event_base_t base, int32_t id, void*){
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) xEventGroupSetBits(s_wifi_eg, WIFI_GOT_IP);
}

static inline bool ok_or_known(esp_err_t e){ return e == ESP_OK || e == ESP_ERR_INVALID_STATE; }
#define TRY_SKIP_INVALID(x) do { esp_err_t __e = (x); if (!ok_or_known(__e)) { printf(#x " -> err=%d\n", __e); return __e; } } while(0)

static esp_err_t init_nvs(){
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

static esp_err_t wifi_init_sta_safe(const char* ssid, const char* pass){
    TRY_SKIP_INVALID(init_nvs());
    TRY_SKIP_INVALID(esp_netif_init());
    esp_err_t err = esp_event_loop_create_default();
    if (!ok_or_known(err)) return err;

    esp_netif_t* netif = esp_netif_create_default_wifi_sta();
    if (!netif) return ESP_FAIL;

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    TRY_SKIP_INVALID(esp_wifi_init(&cfg));

    wifi_config_t wc = {};
    strncpy((char*)wc.sta.ssid, ssid, sizeof(wc.sta.ssid));
    strncpy((char*)wc.sta.password, pass, sizeof(wc.sta.password));
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wc.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

    TRY_SKIP_INVALID(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, nullptr));
    TRY_SKIP_INVALID(esp_wifi_set_mode(WIFI_MODE_STA));
    TRY_SKIP_INVALID(esp_wifi_set_config(WIFI_IF_STA, &wc));
    TRY_SKIP_INVALID(esp_wifi_start());
    TRY_SKIP_INVALID(esp_wifi_connect());

    if (!s_wifi_eg) s_wifi_eg = xEventGroupCreate();
    EventBits_t bits = xEventGroupWaitBits(s_wifi_eg, WIFI_GOT_IP, pdFALSE, pdFALSE, pdMS_TO_TICKS(8000));
    return (bits & WIFI_GOT_IP) ? ESP_OK : ESP_ERR_TIMEOUT;
}

// Variables para medir ancho de pulso ZC
static volatile uint64_t zc_rise_time[3] = {0,0,0};
static volatile uint32_t zc_pulse_width[3] = {0,0,0};

// ===================== Configuración general =====================
constexpr uint32_t ZC_PULSE_WIDTH = 500;
constexpr uint32_t ZC_PULSE_WIDTH_US = 500;
constexpr uint32_t DEBOUNCE_TIME_US  = 1000;

constexpr int32_t  WORK_MIN      = 900;
int32_t  POT_MIN_COUNTS        = 200;
int32_t  POT_MAX_COUNTS        = 20000;
uint32_t DYNAMIC_CTRL_PERIOD_MS = 50;
// (Opcional) debounce un poco más ajustado si tu ZC es limpio
// constexpr uint32_t DEBOUNCE_TIME_US  = 400;

constexpr uint32_t MAX_DELAY_US  = 8325;
constexpr int32_t  STEP_COUNTS   = 70;
constexpr uint32_t US_PER_STEP   = 1;

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
constexpr uint16_t ADS1115_CONFIG_DIFF_0_1  = 0xC283;

#define FILTER_SIZE 8

// ===================== Parámetros de pulsación =====================
static const uint64_t STEP1_MS       = 2000;
static const uint64_t STEP2_MS       = 3000;
static const uint64_t MAX_GAP_TOL_MS = 800;
static const uint64_t FAIL_DT_CAP_MS = 50;

// ===================== Logger RAM =====================
enum LogLevel : uint8_t { L_INFO=0, L_WARN=1, L_ERROR=2 };
struct LogMsg { LogLevel level; const char* tag; char text[160]; };
static QueueHandle_t log_q = nullptr;

#define LOGI(TAG, FMT, ...) do { if (log_q){ LogMsg _m{L_INFO, TAG, {0}}; snprintf(_m.text, sizeof(_m.text), FMT, ##__VA_ARGS__); xQueueSend(log_q, &_m, 0);} } while(0)
#define LOGW(TAG, FMT, ...) do { if (log_q){ LogMsg _m{L_WARN, TAG, {0}}; snprintf(_m.text, sizeof(_m.text), FMT, ##__VA_ARGS__); xQueueSend(log_q, &_m, 0);} } while(0)
#define LOGE(TAG, FMT, ...) do { if (log_q){ LogMsg _m{L_ERROR, TAG, {0}}; snprintf(_m.text, sizeof(_m.text), FMT, ##__VA_ARGS__); xQueueSend(log_q, &_m, 0);} } while(0)

static void logger_task(void*){
    LogMsg m;
    for(;;){ xQueueReceive(log_q, &m, portMAX_DELAY); /* hook opcional */ }
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
    volatile bool pulse_high;
} phase_config_t;

#define NUM_PHASES 3
constexpr int32_t RAW_V_MIN            = 700;
constexpr int32_t RAW_V_MAX            = 20000;
constexpr int32_t ONE_PHASE_THRESHOLD  = 12000;
constexpr int32_t TWO_PHASE_THRESHOLD  = 18000;

constexpr uint32_t ZC_MARGIN_US       = 150;
constexpr uint32_t PULSE_WIDTH_US     = ZC_PULSE_WIDTH_US;

static DRAM_ATTR phase_config_t phases[NUM_PHASES] = {
    { GPIO_NUM_38, GPIO_NUM_48, MAX_DELAY_US, 0, NULL, false, 0, false },
    { GPIO_NUM_21, GPIO_NUM_47, MAX_DELAY_US, 0, NULL, false, 1, false },
    { GPIO_NUM_14, GPIO_NUM_13, MAX_DELAY_US, 0, NULL, false, 2, false }
};

// Filtros
static int32_t reading_buffer[FILTER_SIZE] = {0};
static int     buffer_index  = 0;
static float   voltage_buffer[FILTER_SIZE] = {0};
static int     voltage_index = 0;

// ======== Estado de relés y sistema (A0/A1/A2/A3) ========
static bool a0_on = false;
static bool a1_on = false;
static bool a2_on = false;
static bool a3_on = false;

static bool scr_enabled = false;

// Botones y tiempos
static uint64_t start_hold_ms = 0;
static uint64_t gap_ms = 0;
static bool last_b0 = false;
static bool last_known_b0 = false;
static bool last_known_b1 = false;
static bool step1_done = false;
static bool step2_done = false;

// Apagado escalonado
static bool shutting_down = false;
static uint64_t shutdown_deadline_ms = 0;

// RMS (placeholder para futuro uso)
static float    current_rms_voltage = 0.0f;
static uint64_t last_rms_measurement = 0;
static const uint64_t RMS_MEASUREMENT_INTERVAL = 500; // ms

// Helpers tiempo
static inline uint64_t now_us(){ return (uint64_t)esp_timer_get_time(); }
static inline uint64_t now_ms(){ return now_us() / 1000ULL; }

// ===================== Forward decl =====================
static void IRAM_ATTR zero_crossing_isr_handler(void* arg);
static bool  IRAM_ATTR scr_fire_timer_isr(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*);

bool     mcp23017_read_register_protected(uint8_t reg, uint8_t *value);
bool     mcp23017_write_register_protected(uint8_t reg, uint8_t value);
int32_t  ads1115_read_raw_protected();
void     i2c_master_init();
void     init_mcp23017();
void     initialize_phase(phase_config_t *phase, int timer_idx);
static   void init_zc_timebase_1mhz();

// ===================== I2C base =====================
bool mcp23017_write_register(uint8_t reg, uint8_t value){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MCP23017_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK){ LOGE("I2C", "MCP W reg=0x%02X err=%d", reg, ret); return false; }
    return true;
}

bool mcp23017_read_register(uint8_t reg, uint8_t *value){
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
    if (ret != ESP_OK){ LOGE("I2C", "MCP R reg=0x%02X err=%d", reg, ret); return false; }
    return true;
}

int32_t ads1115_read_raw(){
    if (!adc0) return 0;
    int16_t raw = 0;
    bool ok = adc0->singleShot(
        ADS1115::Mux::AIN0_GND,
        ADS1115::PGA::FS_6V144,
        ADS1115::DataRate::SPS_128,
        raw
    );
    if (!ok) { LOGE("I2C","ADS(0x48) read fail"); return 0; }
    return (int32_t)raw;
}


// ===================== Protegidas con mutex =====================
bool mcp23017_write_register_protected(uint8_t reg, uint8_t value){
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bool ok = mcp23017_write_register(reg, value);
        xSemaphoreGive(i2c_mutex);
        return ok;
    }
    LOGW("I2C", "Timeout MCP W reg=0x%02X", reg);
    return false;
}

bool mcp23017_read_register_protected(uint8_t reg, uint8_t *value){
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

int32_t ads1115_read_raw_protected(){
    return ads1115_read_raw();
}

int32_t ads1115_read_small_signal_49() {
    if (!adc1) return 0;
    int16_t raw = 0;
    bool ok = adc1->singleShot(
        ADS1115::Mux::DIFF_0_1,
        ADS1115::PGA::FS_0V256,
        ADS1115::DataRate::SPS_16,
        raw
    );
    if (!ok) return 0;

    // Conversión a mV consistente con ±0.256 V
    float voltage_mv = raw * (ADS1115::fsr_mV(ADS1115::PGA::FS_0V256) / 32768.0f);
    printf("[ADS1115 0x49] Raw: %d, Voltage: %.3f mV\n", (int)raw, voltage_mv);
    return (int32_t)raw;
}

// Función para convertir a milivoltios
float ads1115_raw_to_mv(int32_t raw_value) {
    // ±256mV FSR, 16 bits
    // raw_value: -32768 to +32767
    return (raw_value * 256.0f) / 32768.0f;
}

int32_t ads1115_read_small_signal_49_protected(){
    return ads1115_read_small_signal_49();
}


// ===================== MCP y utilitarios =====================
void init_mcp23017(){
    mcp23017_write_register_protected(MCP23017_IODIRA, 0x00);
    mcp23017_write_register_protected(MCP23017_IODIRB, 0x03);
    mcp23017_write_register_protected(MCP23017_GPPUB,  0x03);
    mcp23017_write_register_protected(MCP23017_GPIOA,  0x00);
    LOGI("BOOT", "MCP23017 init A0..A3 out; B0,B1 in+pullup");
}

static inline void apply_relays(){
    uint8_t relay_state = 0x00;
    if (a0_on) relay_state |= 0x01;
    if (a1_on) relay_state |= 0x02;
    if (a2_on) relay_state |= 0x04;
    if (a3_on) relay_state |= 0x08;
    mcp23017_write_register_protected(MCP23017_GPIOA, relay_state);
    LOGI("RELAYS","A0=%s A1=%s A2=%s A3=%s", a0_on?"ON":"OFF", a1_on?"ON":"OFF", a2_on?"ON":"OFF", a3_on?"ON":"OFF");
}

static inline void set_direction_from_b1_raw(bool b1_active_low){
    a2_on = !b1_active_low;     // B1=0 -> A2 ON
    a3_on = b1_active_low;    // B1=1 -> A3 ON
}

void update_phases_based_on_potentiometer(int32_t filtered_value){
    if (!scr_enabled) { for (int i=0;i<NUM_PHASES;i++) phases[i].enabled = false; return; }

    if (filtered_value <= RAW_V_MIN){
        phases[0].enabled=false; phases[1].enabled=false; phases[2].enabled=false;
        LOGI("MODE","0 ALL PHASES DISABLED");
    } else if (filtered_value > RAW_V_MIN && filtered_value <= ONE_PHASE_THRESHOLD){
        phases[0].enabled=true; phases[1].enabled=true; phases[2].enabled=true;
        LOGI("MODE","1 fase (B) v=%ld", filtered_value);
    } else if (filtered_value <= TWO_PHASE_THRESHOLD){
        phases[0].enabled=true; phases[1].enabled=true; phases[2].enabled=true;
        LOGI("MODE","2 fases (B,C) v=%ld", filtered_value);
    } else {
        phases[0].enabled=true; phases[1].enabled=true; phases[2].enabled=true;
        LOGI("MODE","3 fases (A,B,C) v=%ld", filtered_value);
    }
}

void read_buttons(){
    static uint64_t last_ms = 0;
    const uint64_t now = now_ms();
    uint64_t dt = (last_ms==0)? 0 : (now - last_ms);
    last_ms = now;

    uint8_t pb = 0; bool ok = mcp23017_read_register_protected(MCP23017_GPIOB, &pb);

    bool b0_active, b1_active_low;
    if (ok){
        b0_active     = !(pb & 0x01);
        b1_active_low = !(pb & 0x02);
        last_known_b0 = b0_active;
        last_known_b1 = b1_active_low;
    } else {
        b0_active     = last_known_b0;
        b1_active_low = last_known_b1;
        if (dt > FAIL_DT_CAP_MS) dt = FAIL_DT_CAP_MS;
    }

    if (b0_active){
        if (gap_ms <= MAX_GAP_TOL_MS) start_hold_ms += dt; else start_hold_ms = 0;
        gap_ms = 0;
    } else {
        gap_ms += dt;
        if (gap_ms > MAX_GAP_TOL_MS) start_hold_ms = 0;
    }

    if (b0_active && !last_b0) LOGI("BTN","START down");
    if (!b0_active && last_b0) LOGI("BTN","START up");
    last_b0 = b0_active;

    if (!shutting_down){
        if (!step1_done && start_hold_ms >= STEP1_MS){
            a0_on = true; scr_enabled = true; a1_on = false; a2_on = false; a3_on = false;
            step1_done = true;
            LOGI("STATE","STEP1: A0 ON + SCR habilitados"); apply_relays();
        }
        if (!step2_done && start_hold_ms >= STEP2_MS){
            a1_on = true; set_direction_from_b1_raw(b1_active_low); step2_done = true;
            LOGI("STATE","STEP2: A1 ON + Dirección fijada"); apply_relays();
        }
    }

    const bool any_on = (a0_on || a1_on || a2_on || a3_on || scr_enabled);
    if (!b0_active && any_on && !shutting_down && gap_ms > MAX_GAP_TOL_MS){
        a1_on = false; scr_enabled = false; a0_on = false;
        for (int i=0;i<NUM_PHASES;i++){ phases[i].enabled=false; gpio_set_level(phases[i].output_pin,0); gptimer_stop(phases[i].timer); }
        apply_relays();
        LOGI("STATE","Apagado inmediato: A1 OFF + SCR OFF + A0 OFF");
        shutting_down = true; shutdown_deadline_ms = now + 1000; start_hold_ms = 0;
    }

    if (shutting_down && now >= shutdown_deadline_ms){
        a2_on = false; a3_on = false; apply_relays();
        LOGI("STATE","Apagado final: A1 OFF y A2/A3 OFF");
        shutting_down = false; step1_done = false; step2_done = false;
    }
}

// ===================== Filtro =====================
int32_t get_i2c_filtered_value(int32_t raw_value){
    static bool init = false;
    if (!init){ for (int i=0;i<FILTER_SIZE;i++) reading_buffer[i]=raw_value; buffer_index=0; init=true; }
    reading_buffer[buffer_index] = raw_value;
    buffer_index = (buffer_index + 1) % FILTER_SIZE;
    int64_t sum = 0; for (int i=0;i<FILTER_SIZE;i++) sum += reading_buffer[i];
    return (int32_t)(sum / FILTER_SIZE);
}

static inline int32_t clamp_ads(int32_t v){
    if (v < RAW_V_MIN) return RAW_V_MIN;
    if (v > RAW_V_MAX) return RAW_V_MAX;
    return v;
}

// ===================== ISR SCR timer =====================
static bool IRAM_ATTR scr_fire_timer_isr(gptimer_handle_t timer,
                                         const gptimer_alarm_event_data_t*,
                                         void *user_ctx)
{
    phase_config_t *phase = (phase_config_t *)user_ctx;

    // Si deshabilitado, no dispares
    if (!phase->enabled) {
        gpio_set_level(phase->output_pin, 0);
        gptimer_stop(timer);
        phase->pulse_high = false;
        return false;
    }

    // ONE-SHOT: subir a HIGH y detener el timer
    gpio_set_level(phase->output_pin, 1);
    phase->pulse_high = true;

    // Timer ya cumplió su función, lo detenemos.
    gptimer_stop(timer);
    return false; // no rearmamos nada aquí
}


// ===================== ISR ZC =====================
static void IRAM_ATTR zero_crossing_isr_handler(void* arg){
    phase_config_t *phase = (phase_config_t *)arg;

    // Tiempo base a 1 MHz (ya creada en init_zc_timebase_1mhz)
    uint64_t now_tick = 0;
    gptimer_get_raw_count(zc_timebase, &now_tick);

    // Nivel actual del pin ZC para saber si es RISING (1) o FALLING (0)
    const int level = gpio_get_level(phase->zc_pin);

    if (level) {
        // ---------- RISING EDGE ----------
        // Debounce solo para rising
        uint64_t last = last_zc_rise_tick[phase->phase_index];
        if ((uint64_t)(now_tick - last) < (uint64_t)ZC_RISE_DEBOUNCE_US) return;
        last_zc_rise_tick[phase->phase_index] = now_tick;

        // Asegura que el gate esté en LOW al ZC
        gpio_set_level(phase->output_pin, 0);
        phase->pulse_high = false;

        if (phase->enabled) {
            // Calcula y limita el retardo desde tu ZC detectado
            uint32_t d = phase->delay_us;
#ifdef MAX_DELAY_FROM_ZC_US
            if (d < MIN_DELAY_US)         d = MIN_DELAY_US;
            if (d > MAX_DELAY_FROM_ZC_US) d = MAX_DELAY_FROM_ZC_US;
#else
            if (d < MIN_DELAY_US) d = MIN_DELAY_US;
            if (d > MAX_DELAY_US) d = MAX_DELAY_US;
#endif

            // Cancela cualquier programación previa y arma ONE-SHOT para encender HIGH
            gptimer_stop(phase->timer);

            gptimer_alarm_config_t alarm = {
                .alarm_count  = (uint64_t)d,
                .reload_count = 0,
                .flags = {0}
            };
            gptimer_set_alarm_action(phase->timer, &alarm);
            gptimer_set_raw_count(phase->timer, 0);
            gptimer_start(phase->timer);
        } else {
            // Si está deshabilitada la fase, garantiza timer parado y salida en LOW
            gptimer_stop(phase->timer);
        }

    } else {
        // ---------- FALLING EDGE ----------
        // Debounce solo para falling
        uint64_t last = last_zc_fall_tick[phase->phase_index];
        if ((uint64_t)(now_tick - last) < (uint64_t)ZC_FALL_DEBOUNCE_US) return;
        last_zc_fall_tick[phase->phase_index] = now_tick;

        // FALLING: apaga el gate y cancela el temporizador pendiente.
        // Con esto, si el retardo 'd' era largo (p.ej. 7.8 ms), evitamos
        // que el timer dispare el gate después del falling de este pulso ZC.
        gpio_set_level(phase->output_pin, 0);
        phase->pulse_high = false;
        gptimer_stop(phase->timer);
    }
}



// ===================== Tareas =====================
void button_control_task(void*){
    esp_task_wdt_add(NULL);
    const TickType_t period = pdMS_TO_TICKS(20);
    for(;;){ esp_task_wdt_reset(); read_buttons(); vTaskDelay(period); }
}

void dynamic_control_task(void*)
{
    // --- WDT en esta tarea ---
    esp_task_wdt_add(NULL);

    // ----------- (1) Inicialización del filtro -----------
    // Suavizamos la primera lectura llenando el buffer
    for (int i = 0; i < FILTER_SIZE; i++) {
        reading_buffer[i] = ads1115_read_raw_protected();
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // Delay base a partir del ZC (en us); arranca neutro (≈ medio ciclo)
    uint32_t new_delay_from_zc_us = (MIN_DELAY_US + MAX_DELAY_FROM_ZC_US) / 2;

    for (;;) {
        esp_task_wdt_reset();

        // ----------- (2) Lectura del potenciómetro -----------
        g_pot_raw  = ads1115_read_raw_protected();
        int32_t       pot_filt = get_i2c_filtered_value(g_pot_raw);
        int32_t bus_mV = 0;
        int32_t shunt_uV = 0;
        if (ina) {
            // Opción A: lectura directa sin esperar (rápida, suficiente para telemetría periódica)
            bool ok_bus   = ina->readBusVoltage_mV(bus_mV,   /*wait_ready=*/false);
            bool ok_shunt = ina->readShuntMicroVolts(shunt_uV, /*wait_ready=*/false);

            if (ok_bus && ok_shunt) {
                // Si quieres ver en consola:
                // printf("[INA226] Bus=%ld mV  Shunt=%ld uV\n", (long)bus_mV, (long)shunt_uV);
            }
        }
        // Limitar a rango declarado
        if (pot_filt < POT_MIN_COUNTS) pot_filt = POT_MIN_COUNTS;
        if (pot_filt > POT_MAX_COUNTS) pot_filt = POT_MAX_COUNTS;

        // ----------- (3) Mapeo lineal invertido -----------
        // pot bajo  -> delay alto (más cerca de 180°)
        // pot alto  -> delay bajo (más cerca de 0°)
        const int32_t span_counts = (POT_MAX_COUNTS - POT_MIN_COUNTS);
        const uint32_t span_delay = (MAX_DELAY_FROM_ZC_US - MIN_DELAY_US);

        uint32_t k = (span_counts > 0) ? (uint32_t)(pot_filt - POT_MIN_COUNTS) : 0;

        // new_delay = MAX - (k/span_counts)*span_delay
        // hacemos la cuenta en 64 bits para evitar overflow
        uint32_t mapped = (span_counts > 0)
            ? (uint32_t)(( (uint64_t)k * (uint64_t)span_delay ) / (uint64_t)span_counts)
            : 0;

        new_delay_from_zc_us = (MAX_DELAY_FROM_ZC_US > mapped)
            ? (MAX_DELAY_FROM_ZC_US - mapped)
            : MIN_DELAY_US;

        // Clamps defensivos (por ruido o errores de config)
        if (new_delay_from_zc_us < MIN_DELAY_US)         new_delay_from_zc_us = MIN_DELAY_US;
        if (new_delay_from_zc_us > MAX_DELAY_FROM_ZC_US) new_delay_from_zc_us = MAX_DELAY_FROM_ZC_US;

        // ----------- (4) Aplicar al plan de disparo -----------
        if (scr_enabled) {
            // el mismo ángulo para las 3 fases (puedes diferenciar si quieres)
            for (int i = 0; i < NUM_PHASES; i++) {
                phases[i].delay_us = new_delay_from_zc_us;
            }
            // habilitar/deshabilitar fases según “pot_filt”
            update_phases_based_on_potentiometer(pot_filt);
        } else {
            // si SCR deshabilitados, garantizamos salidas en LOW y timers parados
            for (int i = 0; i < NUM_PHASES; i++) {
                phases[i].enabled = false;
                gpio_set_level(phases[i].output_pin, 0);
                gptimer_stop(phases[i].timer);
            }
        }

        // ----------- (5) Periodicidad -----------
        vTaskDelay(pdMS_TO_TICKS(DYNAMIC_CTRL_PERIOD_MS));
    }
}


// Monitor: única tarea que imprime
void system_health_monitor(void*){
    printf("[HEALTH] Logger RAM activo. Sin persistencia.\n");
    static uint32_t sample = 0;
    for(;;){
        uint8_t pb = 0;
        bool ok_btn = mcp23017_read_register_protected(MCP23017_GPIOB, &pb);
        int b0 = ok_btn ? !(pb & 0x01) : -1;
        int b1 = ok_btn ? !(pb & 0x02) : -1;

        int32_t pot_raw    = ads1115_read_raw_protected();
        int32_t pot_filt   = get_i2c_filtered_value(pot_raw);
        int32_t pot_diff49 = ads1115_read_small_signal_49_protected();
        float pot_diff49_mv = ads1115_raw_to_mv(pot_diff49);

        printf("[HEALTH %lu] ok=%d rawB=0x%02X | B0=%d B1=%d | START=%d ENABLE=%d FORWARD=%d REVERSE=%d | SCR=%d | POT_RAW=%ld POT_FILT=%ld POT_DIFF49=%ld | delayA=%lu delayB=%lu delayC=%lu\n",
               (unsigned long)sample++,
               (int)ok_btn, (unsigned)pb,
               b0, b1,
               (int)a0_on, (int)a1_on, (int)a2_on, (int)a3_on,
               (int)scr_enabled,
               (long)pot_raw, (long)pot_filt, (long)pot_diff49,
               (unsigned long)phases[0].delay_us,
               (unsigned long)phases[1].delay_us,
               (unsigned long)phases[2].delay_us);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ===================== Timebase 1 MHz =====================
static void init_zc_timebase_1mhz(){
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
void initialize_phase(phase_config_t *phase, int timer_idx){
    gpio_config_t zc_config = {
        .pin_bit_mask = (1ULL << phase->zc_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,    // típico con colector abierto a GND
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE      // <- ANTES era POSEDGE
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
    phase->pulse_high = false;

    gpio_isr_handler_add(phase->zc_pin, zero_crossing_isr_handler, (void*)phase);

    LOGI("BOOT","Fase %c ZC=%d SCR=%d timer=%d",
         (phase==&phases[0]?'A':(phase==&phases[1]?'B':'C')),
         (int)phase->zc_pin, (int)phase->output_pin, timer_idx);
}

// ===================== I2C init / Enables =====================
void i2c_master_init(){
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

void initialize_mcp_enables(){
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
extern "C" void app_main(void){
    esp_log_level_set("*", ESP_LOG_WARN);

    // WDT
    esp_task_wdt_config_t twdt_config = { .timeout_ms = 60000, .idle_core_mask = 0, .trigger_panic = true };
    esp_task_wdt_init(&twdt_config);

    // Logger RAM
    log_q = xQueueCreate(64, sizeof(LogMsg));
    xTaskCreate(logger_task, "Logger", 4096, NULL, 2, NULL);

    // Mutex
    i2c_mutex    = xSemaphoreCreateMutex();
    system_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL || system_mutex == NULL) { printf("[HEALTH] ERROR creando mutex\n"); return; }

    

    initialize_mcp_enables();
    i2c_master_init();
    init_mcp23017();


    adc0 = new ADS1115(I2C_PORT, ADS1115_ADDR_1, i2c_mutex);
    adc1 = new ADS1115(I2C_PORT, ADS1115_ADDR_2, i2c_mutex);
    adc0->begin();
    adc1->begin();

    ina = new INA226(I2C_PORT, INA226_ADDR, i2c_mutex);
    if (!ina->begin(INA226::Avg::AVG_64,
                    INA226::Ct::CT_1100us,
                    INA226::Ct::CT_1100us,
                    INA226::Mode::SHUNT_BUS_CONT)) {
        printf("[INA226] begin() FAIL\n");
    }

    esp_err_t gi = gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3);
    if (gi != ESP_OK && gi != ESP_ERR_INVALID_STATE) { printf("GPIO ISR err=%d\n", gi); return; }
    init_zc_timebase_1mhz();

    initialize_phase(&phases[0], 0);
    initialize_phase(&phases[1], 1);
    initialize_phase(&phases[2], 2);

    xTaskCreate(dynamic_control_task, "DynamicControl", 8192, NULL, 5, NULL);
    xTaskCreate(button_control_task,  "ButtonControl",  6144, NULL, 6, NULL);
    xTaskCreate(system_health_monitor,"HealthMonitor",  4096, NULL, 1, NULL);

    esp_task_wdt_add(NULL);

    // === Wi-Fi y OTA por navegador ===
    //esp_err_t w = wifi_init_sta_safe("SmartLabs", "20120415H");
    //if (w != ESP_OK){
    //    printf("[OTA] WiFi no disponible, err=%d\n", w);
    //} else {
        //esp_err_t h = http_ota_start(80);
        //if (h != ESP_OK) printf("[OTA] HTTP OTA fallo: %d\n", h);
    //}

    for(;;){ esp_task_wdt_reset(); vTaskDelay(pdMS_TO_TICKS(1000)); }
}
