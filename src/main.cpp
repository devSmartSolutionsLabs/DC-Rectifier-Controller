#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/gptimer.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_intr_alloc.h"
#include "esp_err.h"

constexpr uint32_t ZC_PULSE_WIDTH_US = 20;
constexpr uint32_t DEBOUNCE_TIME_US = 700;
constexpr uint32_t MIN_DELAY_US = 6800;
constexpr uint32_t MAX_DELAY_US = 8310;

constexpr gpio_num_t I2C_SDA_PIN = GPIO_NUM_5;
constexpr gpio_num_t I2C_SCL_PIN = GPIO_NUM_4;
constexpr i2c_port_t I2C_PORT = I2C_NUM_0;

constexpr uint8_t ADS1115_ADDR = 0x48;
constexpr uint8_t MCP23017_ADDR = 0x27;

// Registros MCP23017 
constexpr uint8_t MCP23017_IODIRA = 0x00;
constexpr uint8_t MCP23017_IODIRB = 0x01;
constexpr uint8_t MCP23017_GPIOA = 0x12;
constexpr uint8_t MCP23017_GPIOB = 0x13;
constexpr uint8_t MCP23017_GPPUB = 0x0D;

constexpr uint8_t ADS1115_REG_CONVERSION = 0x00;
constexpr uint8_t ADS1115_REG_CONFIG = 0x01;
constexpr uint16_t ADS1115_CONFIG_START = 0xC1C3;

constexpr int32_t I2C_MAX_VALUE = 32767;
#define FILTER_SIZE 16 // x

typedef struct {
    gpio_num_t zc_pin;
    gpio_num_t output_pin;
    volatile uint32_t delay_us;
    volatile uint64_t last_zc_time;
    gptimer_handle_t timer;
    volatile bool enabled;
} phase_config_t;

#define NUM_PHASES 3
constexpr int32_t RAW_V_MIN = 15699; 
constexpr int32_t RAW_V_MAX = 17799;
constexpr int32_t ONE_PHASE_THRESHOLD = 16999;
constexpr int32_t TWO_PHASE_THRESHOLD = 17499; // Umbral para 2 fases vs 3 fases

phase_config_t phases[NUM_PHASES] = {
    { .zc_pin = GPIO_NUM_38, .output_pin = GPIO_NUM_48, .delay_us = MAX_DELAY_US, .last_zc_time = 0, .timer = NULL, .enabled = false },
    { .zc_pin = GPIO_NUM_21, .output_pin = GPIO_NUM_47, .delay_us = MAX_DELAY_US, .last_zc_time = 0, .timer = NULL, .enabled = false },
    { .zc_pin = GPIO_NUM_14, .output_pin = GPIO_NUM_13, .delay_us = MAX_DELAY_US, .last_zc_time = 0, .timer = NULL, .enabled = false }
};

static int32_t reading_buffer[FILTER_SIZE] = {0};
static int buffer_index = 0;

// Variables para el control de botones y relés
static volatile bool system_enabled = false;
static volatile bool system_ready = false; // Nueva variable: sistema listo para enviar pulsos
static volatile bool relay_a1_state = false;
static uint32_t button_start_press_time = 0;
static bool button_start_pressed = false;
static uint32_t system_activation_time = 0; // Tiempo cuando se activó el sistema

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
        //printf("Error escribiendo MCP23017 reg 0x%02X: %d\n", ret);
        return false;
    }
    return true;
}

bool mcp23017_read_register(uint8_t reg, uint8_t *value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MCP23017_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd); // Repeated start
    i2c_master_write_byte(cmd, (MCP23017_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        //printf("Error leyendo MCP23017 reg 0x%02X: %d\n", ret);
        return false;
    }
    return true;
}

void init_mcp23017() {
    // Configurar Puerto A como salidas (A0, A1, A3 para relés)
    mcp23017_write_register(MCP23017_IODIRA, 0x00); // Todos como salidas
    
    // Configurar Puerto B como entradas (B0, B1 para botones)
    mcp23017_write_register(MCP23017_IODIRB, 0x03); // B0 y B1 como entradas
    
    // Habilitar pull-ups en Puerto B
    mcp23017_write_register(MCP23017_GPPUB, 0x03); // Pull-ups en B0 y B1
    
    // Apagar todos los relés inicialmente
    mcp23017_write_register(MCP23017_GPIOA, 0x00);
    
    printf("MCP23017 inicializado - A0,A1: Salidas, B0,B1: Entradas con pull-up\n");
}

void update_relays() {
    uint8_t relay_state = 0x00;
    
    if (system_enabled) {
        relay_state |= 0x01; // Encender A0 (potencia principal)
    }
    
    if (relay_a1_state) {
        relay_state |= 0x02; // Encender A1 (dirección)
    }
    
    // ⚡ A3 se enciende SOLO cuando system_ready (pulsos habilitados)
    if (system_ready) {
        relay_state |= 0x08; // Encender A3 (habilitación física de pulsos)
        printf(">>> RELE A3 ENCENDIDO - Pulsos habilitados físicamente\n");
    } else {
        printf(">>> RELE A3 APAGADO - Pulsos deshabilitados físicamente\n");
    }
    
    mcp23017_write_register(MCP23017_GPIOA, relay_state);
    printf("Relés actualizados: A0=%s, A1=%s, A3=%s\n", 
           system_enabled ? "ON" : "OFF", 
           relay_a1_state ? "ON" : "OFF",
           system_ready ? "ON" : "OFF");
}

void enable_phase_control(bool enable) {
    printf("=== enable_phase_control(%s) ===\n", enable ? "true" : "false");
    
    for (int i = 0; i < NUM_PHASES; i++) {
        phases[i].enabled = enable;
        if (enable) {
            printf("Fase %c HABILITADA - ZC Pin:%d, SCR Pin:%d\n", 
                   'A' + i, phases[i].zc_pin, phases[i].output_pin);
        } else {
            printf("Fase %c DESHABILITADA\n", 'A' + i);
        }
    }
    printf("===============================\n");
}

// Nueva función para habilitar fases según el valor del potenciómetro
void update_phases_based_on_potentiometer(int32_t filtered_value) {
    if (!system_ready) {  // Cambiado de system_enabled a system_ready
        // Si el sistema no está listo, deshabilitar todas las fases
        for (int i = 0; i < NUM_PHASES; i++) {
            phases[i].enabled = false;
        }
        return;
    }
    
    // Lógica de habilitación de fases según el valor del potenciómetro
    if (filtered_value <= ONE_PHASE_THRESHOLD) {
        // 15999 - 16999: Solo fase B
        phases[0].enabled = false; // Fase A deshabilitada
        phases[1].enabled = true;  // Fase B habilitada
        phases[2].enabled = false; // Fase C deshabilitada
        printf("MODO 1 FASE (B) - Valor: %ld\n", filtered_value);
    } 
    else if (filtered_value > ONE_PHASE_THRESHOLD && filtered_value <= TWO_PHASE_THRESHOLD ){
        // 17000 - 17199: Fases B y C
        phases[0].enabled = false; // Fase A deshabilitada
        phases[1].enabled = true;  // Fase B habilitada
        phases[2].enabled = true;  // Fase C habilitada
        printf("MODO 2 FASES (B y C) - Valor: %ld\n", filtered_value);
    }
    else {
        // 17200 - 17599: Las 3 fases
        phases[0].enabled = true;  // Fase A habilitada
        phases[1].enabled = true;  // Fase B habilitada
        phases[2].enabled = true;  // Fase C habilitada
        printf("MODO 3 FASES (A, B y C) - Valor: %ld\n", filtered_value);
    }

    // prueba con todos encendidos
}

void read_buttons() {
    uint8_t port_b_value;
    if (!mcp23017_read_register(MCP23017_GPIOB, &port_b_value)) {
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
        
        if (pressed_time >= 3000 && !system_enabled) {
            system_enabled = true;
            system_ready = false;
            system_activation_time = current_time;
            printf(">>> Sistema ACTIVADO (manteniendo START)\n");
            printf(">>> Esperando 2 segundos para habilitar pulsos...\n");
            update_relays(); // Actualizar relés (A0 ON, A3 OFF por ahora)
        }
        else if (pressed_time < 3000) {
            if (pressed_time % 1000 == 0) {
                printf("Manteniendo START... %lu segundos\n", (3000 - pressed_time) / 1000);
            }
        }
    }
    else if (!button_b0 && button_start_pressed) {
        // ⚡ APAGADO SEGURO: Primero deshabilitar pulsos (A3), luego potencia (A0)
        button_start_pressed = false;
        
        if (system_enabled) {
            printf("=== INICIANDO APAGADO SEGURO ===\n");
            
            // 1. DESHABILITAR PULSOS INMEDIATAMENTE (A3 OFF)
            system_ready = false;
            
            // 2. Deshabilitar todas las fases de software
            for (int i = 0; i < NUM_PHASES; i++) {
                phases[i].enabled = false;
                gpio_set_level(phases[i].output_pin, 0); // Apagar SCRs
            }
            
            printf(">>> Pulsos DESHABILITADOS - A3 OFF, SCRs apagados\n");
            
            // 3. Actualizar relés inmediatamente (A3 se apaga aquí)
            update_relays();
            
            // 4. DESHABILITAR SISTEMA COMPLETO después de deshabilitar pulsos
            system_enabled = false;
            
            printf("<<< Sistema DESACTIVADO (apagado seguro completado)\n");
        } else {
            printf("Boton START liberado (sin activar sistema)\n");
        }
    }
    
    // ⚡ VERIFICAR SI system_ready CAMBIÓ Y ACTUALIZAR RELÉS
    static bool last_system_ready = false;
    if (system_enabled && !system_ready) {
        uint32_t current_time_check = esp_timer_get_time() / 1000;
        if (current_time_check - system_activation_time >= 2000) {
            system_ready = true;
            printf(">>> SISTEMA LISTO - Pulsos habilitados después de 2 segundos\n");
            update_relays(); // ⚡ ACTUALIZAR RELÉS PARA ENCENDER A3
        }
    }
    
    // ⚡ ACTUALIZAR RELÉS SI system_ready CAMBIÓ
    if (system_ready != last_system_ready) {
        update_relays();
        last_system_ready = system_ready;
    }
    
    // Lógica del botón DIRECCION (B1)
    if (system_enabled) {
        bool new_relay_a1_state = !button_b1;
        if (new_relay_a1_state != relay_a1_state) {
            relay_a1_state = new_relay_a1_state;
            update_relays();
        }
    } else {
        if (relay_a1_state) {
            relay_a1_state = false;
            update_relays();
        }
    }
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
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, config_buf, 3, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        printf("Error escribiendo configuración ADS1115: %d\n", ret);
        return 0;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, ADS1115_REG_CONVERSION, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        printf("Error leyendo conversión ADS1115: %d\n", ret);
        return 0;
    }
    
    int16_t raw_value = (data[0] << 8) | data[1];
    return raw_value;
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
        gptimer_stop(timer);  // ⬅️ IMPORTANTE: detener timer si no está habilitado
        return false;
    }
    
    // ⚡ LECTURA DIGITAL DIRECTA del pin ZC
    // gpio_get_level() lee el estado ACTUAL del pin GPIO
    int zc_state = gpio_get_level(phase->zc_pin);
    
    if (zc_state == 0) {
        // ZC inactivo - ENCENDER SCR seguro
        gpio_set_level(phase->output_pin, 1);
    } else {
        // ZC todavía activo - REPROGRAMAR timer
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = 10,  // ⬅️ Esperar 50us (más que el ancho de tu pulso ZC)
            .reload_count = 0,
            .flags = {0}
        };
        
        gptimer_set_alarm_action(timer, &alarm_config);
        gptimer_set_raw_count(timer, 0);
        gptimer_start(timer);
        
        // ⚡ Asegurar que SCR esté APAGADO mientras ZC esté activo
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
    
    // 1. APAGAR SCR inmediatamente
    gpio_set_level(phase->output_pin, 0);
    
    // 2. DETENER timer actual
    gptimer_stop(phase->timer);
    
    // 3. Programar NUEVO timer con el delay actual
    if (phase->enabled) {
        uint32_t current_delay = phase->delay_us;
        
        // Asegurar límites
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
    
    while (1) {
        read_buttons();
        vTaskDelay(pdMS_TO_TICKS(100)); // Leer botones cada 50ms
    }
}

void dynamic_control_task(void* arg) {
    const uint32_t ADC_MAP_RANGE = RAW_V_MAX - RAW_V_MIN;
    uint32_t delay_range = MAX_DELAY_US - MIN_DELAY_US;
    
    printf("Tarea de control dinámico iniciada.\n");
    
    for (int i = 0; i < FILTER_SIZE; i++) {
        reading_buffer[i] = ads1115_read_raw();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    while (1) {
        // Siempre leer el potenciómetro para mostrar valores
        int32_t raw_value = ads1115_read_raw();
        int32_t i2c_value_filtered = get_i2c_filtered_value(raw_value);

        if (i2c_value_filtered < RAW_V_MIN) i2c_value_filtered = RAW_V_MIN;
        if (i2c_value_filtered > RAW_V_MAX) i2c_value_filtered = RAW_V_MAX;

        
        float ratio_saturated = 0.0;

        if(i2c_value_filtered <= ONE_PHASE_THRESHOLD){
            ratio_saturated = (float)(i2c_value_filtered - RAW_V_MIN) / (ONE_PHASE_THRESHOLD - RAW_V_MIN);
        }
        else if(i2c_value_filtered <= TWO_PHASE_THRESHOLD && i2c_value_filtered > ONE_PHASE_THRESHOLD ){
            ratio_saturated = (float)(35 + i2c_value_filtered - ONE_PHASE_THRESHOLD) / (TWO_PHASE_THRESHOLD - ONE_PHASE_THRESHOLD);
        }
        else if(i2c_value_filtered > TWO_PHASE_THRESHOLD ){
            ratio_saturated = (float)(200 + i2c_value_filtered - TWO_PHASE_THRESHOLD) / (RAW_V_MAX - TWO_PHASE_THRESHOLD);
        }
        
        
        if (ratio_saturated < 0.0f) ratio_saturated = 0.0f;
        if (ratio_saturated > 1.0f) ratio_saturated = 1.0f;

        

        uint32_t new_delay_base = (uint32_t)((1.0f - ratio_saturated) * delay_range) + MIN_DELAY_US;
        
        // Solo actualizar delays si el sistema está activado
        if (system_enabled) {
            phases[0].delay_us = new_delay_base;
            phases[1].delay_us = new_delay_base;
            phases[2].delay_us = new_delay_base;
            
            // Actualizar qué fases están habilitadas según el potenciómetro
            update_phases_based_on_potentiometer(i2c_value_filtered);
            
            if (system_ready) {
                printf("Sistema ACTIVO - RAW: %ld\tFILT: %ld\tDELAY: %lu us\n", 
                       raw_value, i2c_value_filtered, new_delay_base);
            } else {
                printf("Sistema ACTIVO (Esperando...) - RAW: %ld\tFILT: %ld\tDELAY: %lu us\n", 
                       raw_value, i2c_value_filtered, new_delay_base);
            }
        } else {
            printf("Sistema INACTIVO - RAW: %ld\tFILT: %ld (Pulsos deshabilitados)\n", 
                   raw_value, i2c_value_filtered);
        }
        
        vTaskDelay(pdMS_TO_TICKS(200)); 
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
        .flags = {0}
    };
    
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &phase->timer));
    gptimer_event_callbacks_t cbs = {.on_alarm = scr_fire_timer_isr};
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(phase->timer, &cbs, phase));
    ESP_ERROR_CHECK(gptimer_enable(phase->timer));
    
    // Configurar la interrupción ZC - SIEMPRE HABILITADA
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
    initialize_mcp_enables(); 
    i2c_master_init();
    init_mcp23017();
    
    printf("=============================================\n");
    printf("Control de Potencia Trifasico (ADS1115 + MCP23017)\n");
    printf("Sistema INICIA DESHABILITADO\n");
    printf("START: Mantener 3 segundos para ACTIVAR, soltar para DESACTIVAR\n");
    printf("Sistema espera 2 segundos después de activarse para habilitar pulsos\n");
    printf("B1: DIRECCION (controla Rele A1 cuando sistema activo)\n");
    printf("A0: Rele Sistema, A1: Rele Direccion\n");
    printf("Modos de operación:\n");
    printf("- 15999-16999: 1 FASE (B)\n");
    printf("- 17000-17199: 2 FASES (B y C)\n");
    printf("- 17200-17599: 3 FASES (A, B y C)\n");
    printf("Interrupciones ZC SIEMPRE HABILITADAS\n");
    printf("=============================================\n");
    
    gpio_install_isr_service(0);
    initialize_phase(&phases[0], 0);
    initialize_phase(&phases[1], 1);
    initialize_phase(&phases[2], 2);
    
    xTaskCreate(dynamic_control_task, "DynamicControl", 4096, NULL, 5, NULL);
    xTaskCreate(button_control_task, "ButtonControl", 4096, NULL, 6, NULL);
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}