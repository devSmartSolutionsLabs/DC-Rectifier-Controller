#include <cmath>
#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_intr_alloc.h"
#include "esp_err.h"
#include "esp_task_wdt.h"

constexpr uint32_t ZC_PULSE_WIDTH_US = 20;
constexpr uint32_t DEBOUNCE_TIME_US = 700;
constexpr uint32_t MIN_DELAY_US = 6800;
constexpr uint32_t MAX_DELAY_US = 8310;

constexpr gpio_num_t I2C_SDA_PIN = GPIO_NUM_5;
constexpr gpio_num_t I2C_SCL_PIN = GPIO_NUM_4;
constexpr i2c_port_t I2C_PORT = I2C_NUM_0;

constexpr uint8_t ADS1115_ADDR_1 = 0x48;
constexpr uint8_t ADS1115_ADDR_2 = 0x49;
constexpr uint8_t MCP23017_ADDR = 0x27;

// Registros MCP23017 
constexpr uint8_t MCP23017_IODIRA = 0x00;
constexpr uint8_t MCP23017_IODIRB = 0x01;
constexpr uint8_t MCP23017_GPIOA = 0x12;
constexpr uint8_t MCP23017_GPIOB = 0x13;
constexpr uint8_t MCP23017_GPPUB = 0x0D;

constexpr uint8_t ADS1115_REG_CONVERSION = 0x00;
constexpr uint8_t ADS1115_REG_CONFIG = 0x01;

// Configuraciones ADS1115
constexpr uint16_t ADS1115_CONFIG_START = 0xC1C3;
constexpr uint16_t ADS1115_CONFIG_DIFF_0_1 = 0xC583;

constexpr int32_t I2C_MAX_VALUE = 32767;
#define FILTER_SIZE 16

// ✅ MUTEX para protección I2C y variables compartidas
static SemaphoreHandle_t i2c_mutex = NULL;
static SemaphoreHandle_t system_mutex = NULL;

typedef struct {
    gpio_num_t zc_pin;
    gpio_num_t output_pin;
    volatile uint32_t delay_us;
    volatile uint64_t last_zc_time;
    gptimer_handle_t timer;
    volatile bool enabled;
} phase_config_t;

#define NUM_PHASES 3
constexpr int32_t RAW_V_MIN = 15799; 
constexpr int32_t RAW_V_MAX = 17799;
constexpr int32_t ONE_PHASE_THRESHOLD = 16999;
constexpr int32_t TWO_PHASE_THRESHOLD = 17499;

phase_config_t phases[NUM_PHASES] = {
    { .zc_pin = GPIO_NUM_38, .output_pin = GPIO_NUM_48, .delay_us = MAX_DELAY_US, .last_zc_time = 0, .timer = NULL, .enabled = false },
    { .zc_pin = GPIO_NUM_21, .output_pin = GPIO_NUM_47, .delay_us = MAX_DELAY_US, .last_zc_time = 0, .timer = NULL, .enabled = false },
    { .zc_pin = GPIO_NUM_14, .output_pin = GPIO_NUM_13, .delay_us = MAX_DELAY_US, .last_zc_time = 0, .timer = NULL, .enabled = false }
};

static int32_t reading_buffer[FILTER_SIZE] = {0};
static int buffer_index = 0;
static float voltage_buffer[FILTER_SIZE] = {0};
static int voltage_index = 0;

// Variables para el control de botones y relés (PROTEGIDAS)
static bool system_enabled = false;
static bool system_ready = false;
static bool relay_a1_state = false;
static uint32_t button_start_press_time = 0;
static bool button_start_pressed = false;
static uint32_t system_activation_time = 0;

// Variables para RMS
static float current_rms_voltage = 0.0f;
static uint32_t last_rms_measurement = 0;
static const uint32_t RMS_MEASUREMENT_INTERVAL = 500;
static const uint32_t DISPLAY_INTERVAL = 500;
static uint32_t last_display_time = 0;

// ✅ FUNCIONES PARA ACCESO SEGURO A VARIABLES COMPARTIDAS
bool get_system_enabled() {
    bool value;
    if (xSemaphoreTake(system_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        value = system_enabled;
        xSemaphoreGive(system_mutex);
        return value;
    }
    return false;
}

void set_system_enabled(bool value) {
    if (xSemaphoreTake(system_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        system_enabled = value;
        xSemaphoreGive(system_mutex);
    }
}

bool get_system_ready() {
    bool value;
    if (xSemaphoreTake(system_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        value = system_ready;
        xSemaphoreGive(system_mutex);
        return value;
    }
    return false;
}

void set_system_ready(bool value) {
    if (xSemaphoreTake(system_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        system_ready = value;
        xSemaphoreGive(system_mutex);
    }
}

bool get_relay_a1_state() {
    bool value;
    if (xSemaphoreTake(system_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        value = relay_a1_state;
        xSemaphoreGive(system_mutex);
        return value;
    }
    return false;
}

void set_relay_a1_state(bool value) {
    if (xSemaphoreTake(system_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        relay_a1_state = value;
        xSemaphoreGive(system_mutex);
    }
}

// 🔧 FUNCIONES I2C ORIGINALES
bool mcp23017_write_register(uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MCP23017_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        printf("Error escribiendo MCP23017 reg 0x%02X: %d\n", reg, ret);
        return false;
    }
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
    
    if (ret != ESP_OK) {
        printf("Error leyendo MCP23017 reg 0x%02X: %d\n", reg, ret);
        return false;
    }
    return true;
}

int32_t ads1115_read_raw() {
    uint8_t data[2];
    
    uint8_t config_buf[3] = {
        ADS1115_REG_CONFIG, 
        (uint8_t)(ADS1115_CONFIG_START >> 8), 
        (uint8_t)(ADS1115_CONFIG_START & 0xFF)
    };
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR_1 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, config_buf, 3, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        printf("Error escribiendo configuración ADS1115 (0x48): %d\n", ret);
        return 0;
    }
    
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
    
    if (ret != ESP_OK) {
        printf("Error leyendo conversión ADS1115 (0x48): %d\n", ret);
        return 0;
    }
    
    int16_t raw_value = (data[0] << 8) | data[1];
    return raw_value;
}

float ads1115_read_differential_rms() {
    const int SAMPLES = 10; // ✅ REDUCIDO de 83 a 10 para menos carga
    int64_t sum_squares = 0;
    int samples_ok = 0;
    
    uint8_t config_buf[3] = {
        ADS1115_REG_CONFIG, 
        (uint8_t)(ADS1115_CONFIG_DIFF_0_1 >> 8),
        (uint8_t)(ADS1115_CONFIG_DIFF_0_1 & 0xFF)
    };
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR_2 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, config_buf, 3, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        printf("Error configurando ADS1115 (0x49) para RMS: %d\n", ret);
        return -1.0f;
    }
    
    vTaskDelay(pdMS_TO_TICKS(15));
    
    for (int i = 0; i < SAMPLES; i++) {
        // ✅ ALIMENTAR WATCHDOG CADA 10 MUESTRAS
        if (i % 10 == 0) {
            esp_task_wdt_reset();
        }
        
        uint8_t data[2];
        
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (ADS1115_ADDR_2 << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, ADS1115_REG_CONVERSION, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (ADS1115_ADDR_2 << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        
        ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50)); // ✅ Reducido timeout
        i2c_cmd_link_delete(cmd);
        
        if (ret != ESP_OK) {
            continue;
        }
        
        int16_t raw_value = (int16_t)((data[0] << 8) | data[1]);
        
        if (raw_value == 0 || raw_value == -1 || raw_value == 32767 || raw_value == -32768) {
            continue;
        }
        
        float instant_voltage = (raw_value * 0.256f) / 32768.0f;
        float instant_voltage_mv = instant_voltage * 1000.0f;
        
        if (instant_voltage_mv < -500.0f || instant_voltage_mv > 500.0f) {
            continue;
        }
        
        sum_squares += (int64_t)(instant_voltage_mv * instant_voltage_mv);
        samples_ok++;
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    if (samples_ok < SAMPLES * 0.5) {
        printf("Advertencia RMS: Solo %d/%d muestras válidas\n", samples_ok, SAMPLES);
        return -1.0f;
    }
    
    float mean_square = (float)sum_squares / samples_ok;
    
    if (mean_square < 0) {
        return -1.0f;
    }
    
    float rms_voltage = sqrtf(mean_square);
    return rms_voltage;
}

// 🔒 FUNCIONES PROTEGIDAS CON MUTEX
bool mcp23017_write_register_protected(uint8_t reg, uint8_t value) {
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bool result = mcp23017_write_register(reg, value);
        xSemaphoreGive(i2c_mutex);
        return result;
    }
    printf("Timeout escribiendo MCP23017\n");
    return false;
}

bool mcp23017_read_register_protected(uint8_t reg, uint8_t *value) {
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bool result = mcp23017_read_register(reg, value);
        xSemaphoreGive(i2c_mutex);
        return result;
    }
    printf("Timeout leyendo MCP23017\n");
    return false;
}

int32_t ads1115_read_raw_protected() {
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        int32_t result = ads1115_read_raw();
        xSemaphoreGive(i2c_mutex);
        return result;
    }
    printf("Timeout leyendo ADS1115 (0x48)\n");
    return 0;
}

float ads1115_read_differential_rms_protected() {
    return 0;
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(150)) == pdTRUE) {
        float result = ads1115_read_differential_rms();
        xSemaphoreGive(i2c_mutex);
        return result;
    }
    printf("Timeout leyendo ADS1115 (0x49) RMS\n");
    return -1.0f;
}

void init_mcp23017() {
    mcp23017_write_register_protected(MCP23017_IODIRA, 0x00);
    mcp23017_write_register_protected(MCP23017_IODIRB, 0x03);
    mcp23017_write_register_protected(MCP23017_GPPUB, 0x03);
    mcp23017_write_register_protected(MCP23017_GPIOA, 0x00);
    
    printf("MCP23017 inicializado - A0,A1: Salidas, B0,B1: Entradas con pull-up\n");
}

void update_relays() {
    uint8_t relay_state = 0x00;
    
    if (get_system_enabled()) {
        relay_state |= 0x01;
    }
    
    if (get_relay_a1_state()) {
        relay_state |= 0x02;
    }
    
    if (get_system_ready()) {
        relay_state |= 0x08;
    }
    
    mcp23017_write_register_protected(MCP23017_GPIOA, relay_state);
    
    printf("Relés: A0=%s, A1=%s, A3=%s\n", 
           get_system_enabled() ? "ON" : "OFF", 
           get_relay_a1_state() ? "ON" : "OFF",
           get_system_ready() ? "ON" : "OFF");
}

void update_phases_based_on_potentiometer(int32_t filtered_value) {
    if (!get_system_ready()) {
        for (int i = 0; i < NUM_PHASES; i++) {
            phases[i].enabled = false;
        }
        return;
    }
    
    if(get_relay_a1_state()){
        if (filtered_value <= ONE_PHASE_THRESHOLD) {
            phases[0].enabled = false;
            phases[1].enabled = false;
            phases[2].enabled = true;
            printf("MODO 1 FASE (B) - Valor: %ld\n", filtered_value);
        } 
        else if (filtered_value > ONE_PHASE_THRESHOLD && filtered_value <= TWO_PHASE_THRESHOLD ){
            phases[0].enabled = false;
            phases[1].enabled = true;
            phases[2].enabled = true;
            printf("MODO 2 FASES (B y C) - Valor: %ld\n", filtered_value);
        }
        else {
            phases[0].enabled = true;
            phases[1].enabled = true;
            phases[2].enabled = true;
            printf("MODO 3 FASES (A, B y C) - Valor: %ld\n", filtered_value);
        }
    }
    else {
        phases[0].enabled = false;
        phases[1].enabled = true;
        phases[2].enabled = true;
        printf("MODO REVERSA - 2 FASES (B y C) - Valor: %ld\n", filtered_value);
    }
}

void read_buttons() {
    uint8_t port_b_value;
    
    if (!mcp23017_read_register_protected(MCP23017_GPIOB, &port_b_value)) {
        return;
    }
    
    bool button_b0 = !(port_b_value & 0x01);
    bool button_b1 = !(port_b_value & 0x02);
    
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Lógica del botón START (B0)
    if (button_b0 && !button_start_pressed) {
        button_start_pressed = true;
        button_start_press_time = current_time;
        printf("Boton START presionado - contando 3 segundos...\n");
    } 
    else if (button_b0 && button_start_pressed) {
        uint32_t pressed_time = current_time - button_start_press_time;
        
        if (pressed_time >= 3000 && !get_system_enabled()) {
            set_system_enabled(true);
            set_system_ready(false);
            system_activation_time = current_time;
            printf(">>> Sistema ACTIVADO (manteniendo START)\n");
            printf(">>> Esperando 2 segundos para habilitar pulsos...\n");
            update_relays();
        }
        else if (pressed_time < 3000) {
            if (pressed_time % 1000 == 0) {
                printf("Manteniendo START... %lu segundos\n", (3000 - pressed_time) / 1000);
            }
        }
    }
    else if (!button_b0 && button_start_pressed) {
        button_start_pressed = false;
        
        if (get_system_enabled()) {
            printf("=== INICIANDO SECUENCIA DE APAGADO SEGURO ===\n");
            set_system_ready(false);
            
            for (int i = 0; i < NUM_PHASES; i++) {
                phases[i].enabled = false;
                gpio_set_level(phases[i].output_pin, 0);
            }
            
            printf(">>> PASO 1: Pulsos DESHABILITADOS - A3 OFF, SCRs apagados\n");
            update_relays();
            
            printf(">>> PASO 2: Esperando 2 segundos antes de apagar potencia...\n");
            vTaskDelay(pdMS_TO_TICKS(2000));
            
            set_system_enabled(false);
            update_relays();
            
            printf(">>> PASO 3: Potencia APAGADA - A0 OFF\n");
            printf("<<< SECUENCIA DE APAGADO COMPLETADA\n");
        } else {
            printf("Boton START liberado (sin activar sistema)\n");
        }
    }
    
    static bool last_system_ready = false;
    bool current_system_ready = get_system_ready();
    if (get_system_enabled() && !current_system_ready) {
        uint32_t current_time_check = esp_timer_get_time() / 1000;
        if (current_time_check - system_activation_time >= 2000) {
            set_system_ready(true);
            printf(">>> SISTEMA LISTO - Pulsos habilitados después de 2 segundos\n");
            update_relays();
        }
    }
    
    if (current_system_ready != last_system_ready) {
        update_relays();
        last_system_ready = current_system_ready;
    }
    
    // Lógica del botón DIRECCION (B1)
    if (get_system_enabled()) {
        bool new_relay_a1_state = !button_b1;
        if (new_relay_a1_state != get_relay_a1_state()) {
            set_relay_a1_state(new_relay_a1_state);
            update_relays();
        }
    } else {
        if (get_relay_a1_state()) {
            set_relay_a1_state(false);
            update_relays();
        }
    }
}

int32_t get_i2c_filtered_value(int32_t raw_value) {
    int32_t current_reading = raw_value;
    
    if (current_reading == 0 && buffer_index > 0) 
        return reading_buffer[buffer_index - 1];
    
    reading_buffer[buffer_index] = current_reading;
    buffer_index = (buffer_index + 1) % FILTER_SIZE;
    
    int64_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += reading_buffer[i];
    }
    
    int32_t filtered_value = (int32_t)(sum / FILTER_SIZE);
    
    if (filtered_value < RAW_V_MIN) return RAW_V_MIN;
    if (filtered_value > RAW_V_MAX) return RAW_V_MAX;
    
    return filtered_value;
}

static bool IRAM_ATTR scr_fire_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    phase_config_t *phase = (phase_config_t *)user_ctx;
    
    if (!phase->enabled) {
        gptimer_stop(timer);
        return false;
    }
    
    int zc_state = gpio_get_level(phase->zc_pin);
    
    if (zc_state == 0) {
        gpio_set_level(phase->output_pin, 1);
    } else {
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = 2,
            .reload_count = 0,
            .flags = {0}
        };
        
        gptimer_set_alarm_action(timer, &alarm_config);
        gptimer_set_raw_count(timer, 0);
        gptimer_start(timer);
        gpio_set_level(phase->output_pin, 0);
    }
    
    return false;
}

static void IRAM_ATTR zero_crossing_isr_handler(void* arg) {
    phase_config_t *phase = (phase_config_t *)arg;
    
    uint64_t current_time = esp_timer_get_time();
    
    if (current_time - phase->last_zc_time < DEBOUNCE_TIME_US) { 
        return;
    }
    
    phase->last_zc_time = current_time;
    
    gpio_set_level(phase->output_pin, 0);
    gptimer_stop(phase->timer);
    
    if (phase->enabled) {
        uint32_t current_delay = phase->delay_us;
        
        if (current_delay < MIN_DELAY_US) current_delay = MIN_DELAY_US;
        if (current_delay > MAX_DELAY_US) current_delay = MAX_DELAY_US;
        
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = current_delay,
            .reload_count = 0,
            .flags = {0}
        };
        
        gptimer_set_alarm_action(phase->timer, &alarm_config);
        gptimer_set_raw_count(phase->timer, 0);
        gptimer_start(phase->timer);
    }
}

void button_control_task(void* arg) {
    printf("Tarea de control de botones iniciada\n");
    esp_task_wdt_add(NULL); // ✅ REGISTRAR CON WATCHDOG
    
    while (1) {
        esp_task_wdt_reset(); // ✅ ALIMENTAR WATCHDOG
        read_buttons();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void dynamic_control_task(void* arg) {
    uint32_t delay_range = MAX_DELAY_US - MIN_DELAY_US;
    
    printf("Tarea de control dinámico iniciada.\n");
    esp_task_wdt_add(NULL); // ✅ REGISTRAR CON WATCHDOG
    
    // Inicializar buffers
    for (int i = 0; i < FILTER_SIZE; i++) {
        reading_buffer[i] = ads1115_read_raw_protected();
        voltage_buffer[i] = 0.0f;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    while (1) {
        esp_task_wdt_reset(); // ✅ ALIMENTAR WATCHDOG
        
        uint32_t current_time = esp_timer_get_time() / 1000;
        
        // Leer potenciómetro
        int32_t raw_value = ads1115_read_raw_protected();
        int32_t i2c_value_filtered = get_i2c_filtered_value(raw_value);
        
        if (i2c_value_filtered < RAW_V_MIN) i2c_value_filtered = RAW_V_MIN;
        if (i2c_value_filtered > RAW_V_MAX) i2c_value_filtered = RAW_V_MAX;
        
        // Medición RMS cada 500ms
        if (current_time - last_rms_measurement >= RMS_MEASUREMENT_INTERVAL) {
            float new_rms = 0.0;
            //float new_rms = ads1115_read_differential_rms_protected();
            if (new_rms >= 0) {
                current_rms_voltage = new_rms;
            }
            last_rms_measurement = current_time;
        }
        
        // Filtrar voltaje RMS
        voltage_buffer[voltage_index] = current_rms_voltage;
        voltage_index = (voltage_index + 1) % FILTER_SIZE;
        
        float voltage_sum = 0;
        int valid_voltage_samples = 0;
        for (int i = 0; i < FILTER_SIZE; i++) {
            if (voltage_buffer[i] >= 0) {
                voltage_sum += voltage_buffer[i];
                valid_voltage_samples++;
            }
        }
        
        float diff_voltage_filtered = (valid_voltage_samples > 0) ? voltage_sum / valid_voltage_samples : 0.0f;
        
        // Cálculo del delay
        float ratio_saturated = 0.0;
        if(get_relay_a1_state()){
            if(i2c_value_filtered <= ONE_PHASE_THRESHOLD){
                ratio_saturated = (float)(i2c_value_filtered - RAW_V_MIN) / (ONE_PHASE_THRESHOLD - RAW_V_MIN);
            }
            else if(i2c_value_filtered <= TWO_PHASE_THRESHOLD && i2c_value_filtered > ONE_PHASE_THRESHOLD ){
                ratio_saturated = (float)(35 + i2c_value_filtered - ONE_PHASE_THRESHOLD) / (TWO_PHASE_THRESHOLD - ONE_PHASE_THRESHOLD);
            }
            else if(i2c_value_filtered > TWO_PHASE_THRESHOLD ){
                ratio_saturated = (float)(200 + i2c_value_filtered - TWO_PHASE_THRESHOLD) / (RAW_V_MAX - TWO_PHASE_THRESHOLD);
            }
        }
        else {
            ratio_saturated = (float)(i2c_value_filtered - RAW_V_MIN) / (RAW_V_MAX - RAW_V_MIN);
        }
        if (ratio_saturated < 0.0f) ratio_saturated = 0.0f;
        if (ratio_saturated > 1.0f) ratio_saturated = 1.0f;
        
        uint32_t new_delay_base = (uint32_t)((1.0f - ratio_saturated) * delay_range) + MIN_DELAY_US;
        
        // Control SCRs
        if (get_system_enabled()) {
            phases[0].delay_us = new_delay_base;
            phases[1].delay_us = new_delay_base;
            phases[2].delay_us = new_delay_base;
            
            update_phases_based_on_potentiometer(i2c_value_filtered);
        }
        
        // Mostrar valores cada 500ms
        if (current_time - last_display_time >= DISPLAY_INTERVAL) {
            if (get_system_enabled()) {
                if (get_system_ready()) {
                    printf("Sistema ACTIVO - POT: RAW=%ld FILT=%ld DELAY=%luus | VOLT_RMS: %.2fmV\n", 
                           raw_value, i2c_value_filtered, new_delay_base, diff_voltage_filtered);
                } else {
                    printf("Sistema ACTIVO (Esperando...) - POT: RAW=%ld FILT=%ld DELAY=%luus | VOLT_RMS: %.2fmV\n", 
                           raw_value, i2c_value_filtered, new_delay_base, diff_voltage_filtered);
                }
            } else {
                printf("Sistema INACTIVO - POT: RAW=%ld FILT=%ld | VOLT_RMS: %.2fmV\n", 
                       raw_value, i2c_value_filtered, diff_voltage_filtered);
            }
            last_display_time = current_time;
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

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
        .flags = {
            .intr_shared = false,
            .allow_pd = false,
            .backup_before_sleep = false
        }
    };
    
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &phase->timer));
    gptimer_event_callbacks_t cbs = {.on_alarm = scr_fire_timer_isr};
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(phase->timer, &cbs, phase));
    ESP_ERROR_CHECK(gptimer_enable(phase->timer));
    
    gpio_isr_handler_add(phase->zc_pin, zero_crossing_isr_handler, (void*)phase);
    
    printf("Fase %c - ZC Pin:%d, SCR Pin:%d, Timer ID:%d, Estado: %s\n", 
           (phase == &phases[0] ? 'A' : (phase == &phases[1] ? 'B' : 'C')), 
           phase->zc_pin, phase->output_pin, timer_idx,
           phase->enabled ? "HABILITADO" : "DESHABILITADO");
}

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {.clk_speed = 100000},
        .clk_flags = 0
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
    printf("I2C Master inicializado en SDA:%d, SCL:%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
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
    
    printf("Pines 15 y 41 configurados como HIGH para habilitar MCP23017s.\n");
}

extern "C" void app_main(void) {
    // ✅ CORREGIDO: Watchdog para ESP-IDF 5.x
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 30000,  // 30 segundos
        .idle_core_mask = 0,  // No vigilar cores inactivos  
        .trigger_panic = true // Provocar panic en timeout
    };
    esp_task_wdt_init(&twdt_config);

    // ✅ CREAR MUTEX I2C y SYSTEM
    i2c_mutex = xSemaphoreCreateMutex();
    system_mutex = xSemaphoreCreateMutex();
    
    if (i2c_mutex == NULL || system_mutex == NULL) {
        printf("Error: No se pudieron crear mutex\n");
        return;
    }
    
    initialize_mcp_enables(); 
    i2c_master_init();
    init_mcp23017();
    
    printf("=============================================\n");
    printf("Control de Potencia Trifasico (ESTABILIZADO)\n");
    printf("Watchdog habilitado - Stack aumentado\n");
    printf("Mutex para variables compartidas\n");
    printf("=============================================\n");
    
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM); // ✅ INTERRUPCIONES EN IRAM
    
    initialize_phase(&phases[0], 0);
    initialize_phase(&phases[1], 1);
    initialize_phase(&phases[2], 2);
    
    // ✅ STACK AUMENTADO a 6144 bytes
    xTaskCreate(dynamic_control_task, "DynamicControl", 6144, NULL, 5, NULL);
    xTaskCreate(button_control_task, "ButtonControl", 4096, NULL, 6, NULL);
    
    // ✅ REGISTRAR TAREA PRINCIPAL CON WATCHDOG
    esp_task_wdt_add(NULL);
    
    while (1) {
        esp_task_wdt_reset(); // ✅ ALIMENTAR WATCHDOG
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}