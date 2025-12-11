// main.cpp — ESP-IDF v5.5.1 — Sistema Completo Rectificador Integrado
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_task_wdt.h" 
#include <cmath>
#include <algorithm> // Necesario para la función std::sort

#include "ads1115.hpp"
#include "mcp23017.hpp"
#include "ina226.hpp"

static const char* TAG = "RECTIFICADOR";

// === Configuración SCR ===
static constexpr uint32_t PULSE_US       = 500;     // ANCHO DEL PULSO: 50 us
static constexpr uint32_t DEBOUNCE_US    = 200;    // anti-rebote ZC
static constexpr uint32_t DEFAULT_SEMI_PERIOD_US = 8333; // 60Hz

// Pines 
static const gpio_num_t ZC_PIN[3]  = { GPIO_NUM_38, GPIO_NUM_21, GPIO_NUM_14 };
static const gpio_num_t SCR_PIN[3] = { GPIO_NUM_48, GPIO_NUM_47, GPIO_NUM_13 };

// === Configuración I2C ===
static constexpr i2c_port_t I2C_PORT = I2C_NUM_0;
static constexpr i2c_port_t I2C_PORT_1 = I2C_NUM_1;
static constexpr gpio_num_t I2C_SDA = GPIO_NUM_5;
static constexpr gpio_num_t I2C_SCL = GPIO_NUM_4;
static constexpr gpio_num_t I2C1_SDA = GPIO_NUM_43;
static constexpr gpio_num_t I2C1_SCL = GPIO_NUM_44;
static constexpr uint32_t   I2C_HZ  = 400000;

// === CONSTANTES GLOBALES DE MAPEO (Refactorizadas) ===
static constexpr float MAX_CURRENT_A    = 5000.0f;      // Corriente máxima total
static constexpr float CURRENT_STEP_A   = 5.0f;         // Paso de corriente deseado (5A)
static constexpr float DELAY_STEP_US    = 4.0f;         // Paso de delay deseado (1.0 us/punto)

// Constantes Derivadas
static constexpr float NUM_POINTS_F     = MAX_CURRENT_A / CURRENT_STEP_A; // 5000A / 5A = 1000.0f
static constexpr float DELAY_RANGE_US_F = NUM_POINTS_F * DELAY_STEP_US;   // 1000.0f * 1.0f = 1000.0f

// Límites de Potenciómetro
static constexpr float POT_MIN_MV       = 400.0f;  // NUEVO: Mínimo mapeado
static constexpr float POT_MAX_MV       = 4000.0f;  // NUEVO: Máximo mapeado
static constexpr float MV_RANGE         = POT_MAX_MV - POT_MIN_MV; // 2000 mV de rango útil

// Nuevo límite superior de seguridad
static constexpr uint32_t SAFE_MAX_DELAY_US = 8320; // Hard cap para el delay máximo (seguridad)

// === LÍMITES DE VALIDACIÓN DE FRECUENCIA (NUEVOS) ===
// 60.5 Hz (Período más corto)
static constexpr uint32_t MIN_PERIOD_VALID_US = 8264; 
// 59.5 Hz (Período más largo)
static constexpr uint32_t MAX_PERIOD_VALID_US = 8404;
// ===================================================

// El DELAY MAX y MIN AHORA SERÁN CALCULADOS DINÁMICAMENTE.
static volatile uint32_t g_scr_delay_us = DEFAULT_SEMI_PERIOD_US; // Valor inicial
static volatile uint32_t g_current_delay_max_us = DEFAULT_SEMI_PERIOD_US;
static volatile uint32_t g_current_delay_min_us = (uint32_t)(DEFAULT_SEMI_PERIOD_US - DELAY_RANGE_US_F);

// === Variables Globales del Sistema ===
static volatile bool g_system_started = false;
static volatile bool g_scr_enabled = false;
static volatile uint32_t g_pulse_count[3] = {0, 0, 0};
static volatile float g_pot_mv = 0.0f; // Almacena el último valor del potenciómetro

// === Variables de Medición de Periodo Adaptativo (MÍNIMO HISTÓRICO) ===
// Inicializado a un valor alto para que la primera medición válida lo reemplace.
static volatile uint32_t g_semi_period_measured_us = 8400; 
// =====================================================================

// === Estados del Sistema ===
static bool a0_on = false;  // Relay principal
static bool a1_on = false;  // Relay dirección
static bool a2_on = false;  // Forward  
static bool a3_on = false;  // Reverse

// Botones y máquina de estados
static uint64_t start_hold_ms = 0;
static bool last_b0 = false;
static bool step1_done = false;
static bool step2_done = false;
static bool shutting_down = false;

// === Instancias ===
static ADS1115* g_ads = nullptr;
static MCP23017* g_mcp = nullptr;
static INA226* g_ina = nullptr;

// === Estructura por Fase ===
typedef struct {
    gptimer_handle_t timer;
    int              idx;
    volatile uint32_t last_rise_us; // Usado también para R-to-R period calculation
    volatile uint32_t last_fall_us;
    volatile bool     next_is_high;
    volatile bool     enabled;
} phase_t;

static phase_t ph[3];

// === Mutex Global ===
static SemaphoreHandle_t g_i2c_mutex = nullptr;

// === Helpers ===
static inline uint32_t now_us() { return (uint32_t)esp_timer_get_time(); }
static inline uint64_t now_ms() { return now_us() / 1000ULL; }

// === Control MCP23017 ===
static void control_relays() {
    if (!g_mcp) return;
    
    uint8_t relay_state = 0x00;
    if (a0_on) relay_state |= 0x01;
    if (a1_on) relay_state |= 0x02;
    if (a2_on) relay_state |= 0x04;
    if (a3_on) relay_state |= 0x08;
    
    g_mcp->write_port_a(relay_state);
    
    ESP_LOGI(TAG, "Relays: A0=%s A1=%s A2=%s A3=%s", 
             a0_on?"ON":"OFF", a1_on?"ON":"OFF", a2_on?"ON":"OFF", a3_on?"ON":"OFF");
}

static void set_direction(bool forward) {
    a2_on = forward;    // Forward
    a3_on = !forward;   // Reverse
}

// === Lectura de Botones ===
static void read_buttons() {
    // La suscripción al WDT (esp_task_wdt_add) ha sido movida a button_task.
    if (!g_mcp) return;
    
    bool b0_pressed = false, b1_pressed = false;
    g_mcp->digital_read(8, b0_pressed);  // B0 - Start
    g_mcp->digital_read(9, b1_pressed);  // B1 - Direction
    
    b0_pressed = !b0_pressed;  // Active low
    b1_pressed = !b1_pressed;
    
    const uint64_t now = now_ms();
    static uint64_t last_read = 0;
    uint64_t dt = (last_read == 0) ? 0 : (now - last_read);
    last_read = now;
    
    // Máquina de estados de arranque
    if (b0_pressed) {
        if (!last_b0) {
            ESP_LOGI(TAG, "Start pressed");
        }
        
        start_hold_ms += dt;
        
        if (!step1_done && start_hold_ms >= 2000) {
            a0_on = true;
            g_scr_enabled = true;
            step1_done = true;
            ESP_LOGI(TAG, "STEP1: A0 ON + SCR habilitados");
            control_relays();
        }
        
        if (!step2_done && start_hold_ms >= 3000) {
            a1_on = true;
            set_direction(!b1_pressed);  // Invertido según tu lógica
            step2_done = true;
            ESP_LOGI(TAG, "STEP2: A1 ON + Dirección fijada");
            control_relays();
        }
    } else {
        if (last_b0 && (a0_on || a1_on || g_scr_enabled)) {
            // Iniciar secuencia de apagado
            a1_on = false;
            g_scr_enabled = false;
            shutting_down = true;
            ESP_LOGI(TAG, "Iniciando apagado...");
            control_relays();
        }
        start_hold_ms = 0;
    }
    
    last_b0 = b0_pressed;
    
    // Apagado escalonado
    if (shutting_down) {
        static uint64_t shutdown_start = 0;
        if (shutdown_start == 0) shutdown_start = now_ms();
        
        if (now_ms() - shutdown_start >= 1000) {
            a0_on = false;
            a2_on = false;
            a3_on = false;
            step1_done = step2_done = false;
            shutting_down = false;
            ESP_LOGI(TAG, "Apagado completado");
            control_relays();
        }
    }
    // ESP_ERROR_CHECK(esp_task_wdt_reset()); // Se mueve a button_task
}

// === Lectura de Potenciómetro y Mapeo Adaptativo ===
static void update_potentiometer() {
    if (!g_ads) return;
    
    // --- PARTE 1: FILTRO DE MEDIANA (Elimina picos de ruido) ---
    const int NUM_SAMPLES = 11; 
    float samples[NUM_SAMPLES];
    bool success = true;

    for (int i = 0; i < NUM_SAMPLES; ++i) {
        if (g_ads->singleShotMV(ADS1115::Mux::DIFF_0_1,
                               ADS1115::PGA::FS_6V144,
                               ADS1115::DataRate::SPS_64, 
                               samples[i])) {
            // SPS_64 toma ~15ms por muestra
        } else {
            // Log de error I2C
            ESP_LOGE(TAG, "ADS1115 I2C FAILED: Fallo en muestra %d. Lectura congelada.", i);
            success = false;
            break; 
        }
    }

    if (!success) {
        // Si falla la lectura, no actualizamos g_scr_delay_us ni g_pot_mv para mantener el último valor estable.
        return; 
    }
    
    // Aplicar Filtro de Mediana
    std::sort(samples, samples + NUM_SAMPLES);
    float v_mediana = samples[NUM_SAMPLES / 2]; // El valor central (limpio de picos)
    
    // --- PARTE 2: MEDIA MÓVIL EXPONENCIAL (EMA) (Suaviza el drift lento) ---
    static float v_ema = 0.0f; 
    static bool v_ema_initialized = false;
    const float ALPHA = 0.5f; 
    
    if (!v_ema_initialized) { 
        v_ema = v_mediana;
        v_ema_initialized = true;
    } else {
        v_ema = (ALPHA * v_mediana) + ((1.0f - ALPHA) * v_ema);
    }
    
    float mv = v_ema; // Usamos el valor suavizado para el control
    
    // ALMACENAR VALOR EMA
    g_pot_mv = mv; 

    // ------------------------------------------------------------------------------------------
    // === LÓGICA DE MAPEO ADAPTATIVO Y SEGURO (MÍNIMO HISTÓRICO) ===
    
    // 1. Obtener el Mínimo Histórico del Semi-Período
    // dynamic_semi_period es ahora el mínimo absoluto visto hasta el momento.
    uint32_t dynamic_semi_period = g_semi_period_measured_us; 
    
    if (dynamic_semi_period < MIN_PERIOD_VALID_US || dynamic_semi_period > MAX_PERIOD_VALID_US) { 
        // Si el MÍNIMO HISTÓRICO es inválido, usar el valor de seguridad por defecto.
        dynamic_semi_period = DEFAULT_SEMI_PERIOD_US;
    }

    // Aplicar el hard cap de seguridad (8320 us) sobre el Mínimo Histórico
    uint32_t dynamic_delay_max;
    if (dynamic_semi_period > SAFE_MAX_DELAY_US) {
        dynamic_delay_max = SAFE_MAX_DELAY_US;
    } else {
        dynamic_delay_max = dynamic_semi_period;
    }
    
    uint32_t dynamic_delay_min = (uint32_t)(dynamic_delay_max - DELAY_RANGE_US_F);

    // Actualizar las variables globales usadas en la tarea de monitoreo
    g_current_delay_max_us = dynamic_delay_max;
    g_current_delay_min_us = dynamic_delay_min;

    // 2. Aplicar límites al voltaje
    if (mv < POT_MIN_MV) mv = POT_MIN_MV;
    if (mv > POT_MAX_MV) mv = POT_MAX_MV;
    
    // 3. Normalizar el voltaje al rango [0, 1]
    // Si mv es POT_MIN_MV (2000 mV), normalized = 0
    // Si mv es POT_MAX_MV (4000 mV), normalized = 1
    float normalized = (mv - POT_MIN_MV) / MV_RANGE;
    
    // 4. Convertir a Punto Discreto (P de 0 a 999)
    float point_float = normalized * NUM_POINTS_F;
    uint32_t current_point = (uint32_t)floorf(point_float);
    
    if (current_point >= (uint32_t)NUM_POINTS_F) {
        current_point = (uint32_t)NUM_POINTS_F - 1; 
    }
    
    // 5. Mapeo Invertido y Discreto a Delay (usando el dynamic_delay_max capado)
    // new_delay = DELAY_MAX_US - (current_point * DELAY_STEP_US)
    uint32_t new_delay = (uint32_t)(dynamic_delay_max - ((float)current_point * DELAY_STEP_US));
    
    // 6. Asegurar límites finales
    if (new_delay < dynamic_delay_min) new_delay = dynamic_delay_min;
    if (new_delay > dynamic_delay_max) new_delay = dynamic_delay_max;
    
    // ALMACENAR VALOR FINAL DEL DELAY (usado por la ISR)
    g_scr_delay_us = new_delay;
}

// === Monitoreo INA226 (solo lectura, sin control) ===
static void monitor_ina226() {
    if (!g_ina) return;
    
    int32_t bus_mv = 0, shunt_uv = 0;
    
    if (g_ina->readBusVoltage_mV(bus_mv, true) && 
        g_ina->readShuntMicroVolts(shunt_uv, true)) {
        
        // Log solo cada 5 segundos (movido a monitor_task para evitar redundancia)
    }
}

// === ISR GPTimer ===
static bool IRAM_ATTR on_alarm(gptimer_handle_t timer,
                               const gptimer_alarm_event_data_t* edata,
                               void* user_ctx) {
    phase_t* p = (phase_t*)user_ctx;

    if (!g_scr_enabled || !p->enabled) {
        gpio_set_level(SCR_PIN[p->idx], 0);
        return true;
    }

    if (p->next_is_high) {
        // Disparo del Pulso (HIGH)
        gpio_set_level(SCR_PIN[p->idx], 1);
        
        // ******* PULSO SOSTENIDO: NO SE REPROGRAMA EL APAGADO *******
        // La alarma sonó (t = Delay). El pulso comienza y se mantendrá HIGH.
        p->next_is_high = false; 
    } else {
        // Esta sección NO DEBERÍA ejecutarse en modo pulso sostenido.
        
        // Apagado del Pulso (LOW)
        gpio_set_level(SCR_PIN[p->idx], 0); 
        
        // Deshabilitar futuras alarmas
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = edata->alarm_value + 0x7FFFFFFF,
            .reload_count = 0,
            .flags = { .auto_reload_on_alarm = false }
        };
        gptimer_set_alarm_action(p->timer, &alarm_config);
        
        // Incrementar contador de pulsos
        uint32_t temp = g_pulse_count[p->idx];
        g_pulse_count[p->idx] = temp + 1;
    }
    return true;
}

// === ISR ZC (AMBOS FLANCOS) ===
static void IRAM_ATTR zc_isr(void* arg) {
    const int idx = (int)(intptr_t)arg;
    
    phase_t* p = &ph[idx];

    // Leer nivel actual del pin ZC
    int current_level = gpio_get_level(ZC_PIN[idx]);

    if (!g_scr_enabled || !p->enabled) {
        gpio_set_level(SCR_PIN[idx], 0);
        return;
    }

    uint32_t t = now_us();
    
    // --- Lógica de Debounce (aplicada a ambos flancos) ---
    bool is_rising_edge = (current_level == 1);
    
    uint32_t last_rise_t_for_period = p->last_rise_us; // Capturar el tiempo anterior antes de la actualización

    if (is_rising_edge) {
        if (t - p->last_rise_us < DEBOUNCE_US) {
            // Eliminado el log WARN
            return; 
        }
        p->last_rise_us = t; // Actualiza con el nuevo tiempo para el próximo debounce y periodo
    } else {
        if (t - p->last_fall_us < DEBOUNCE_US) {
            // Eliminado el log WARN
            return; 
        }
        p->last_fall_us = t;
    }
    
    // === MEDICIÓN DEL PERÍODO (RISING-TO-RISING / 2) Y TRACKING MÍNIMO HISTÓRICO ===
    if (is_rising_edge && last_rise_t_for_period != 0) {
        uint32_t full_period_T = t - last_rise_t_for_period; 
        uint32_t semi_period_T_half = full_period_T / 2;

        if (semi_period_T_half >= MIN_PERIOD_VALID_US && semi_period_T_half <= MAX_PERIOD_VALID_US) {
            if (semi_period_T_half < g_semi_period_measured_us) {
                g_semi_period_measured_us = semi_period_T_half;
            }
        }
    }

    // *** Lógica Unificada para programar el Pulso en CADA ZC ***
    
    // 1. APAGADO FÍSICO INMEDIATO Y CANCELACIÓN DE ALARMA PENDIENTE
    gpio_set_level(SCR_PIN[idx], 0); // <--- ESTO APAGA EL PULSO SOSTENIDO DEL SEMICICLO ANTERIOR
    
    // CORRECCIÓN CLAVE 2: AUMENTAR EL CONTADOR DE PULSOS AQUÍ (Semiciclo ZC Procesado)
    uint32_t temp = g_pulse_count[idx];
    g_pulse_count[idx] = temp + 1;
    
    // Detener el timer cancela cualquier alarma de disparo que pudiera estar pendiente.
    gptimer_stop(p->timer);
    
    // 2. Reiniciar Timer para contar el retardo desde el ZC
    gptimer_set_raw_count(p->timer, 0);
    gptimer_start(p->timer);
    
    uint32_t delay = g_scr_delay_us;

    // --- Inhibición del pulso en Delay_MAX ---
    if (delay >= g_current_delay_max_us) { 
        p->next_is_high = false;
        return; 
    }
    
    // 3. Programar alarma para DISPARO (el pulso se mantendrá HIGH hasta el próximo ZC)
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = (uint64_t)delay, // El contador inicia en 0, la alarma es el delay
        .reload_count = 0,
        .flags = { .auto_reload_on_alarm = false }
    };
    gptimer_set_alarm_action(p->timer, &alarm_config);
    p->next_is_high = true; // El próximo evento del timer es HIGH (el inicio del pulso sostenido)
}

// === Tareas ===
static void button_task(void* arg) {
    // LLAMADA UNICA: Suscribir la tarea al WDT
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL)); 
    ESP_LOGI(TAG, "Tarea botones iniciada");
    
    for (;;) {
        // LLAMADA REPETIDA: Resetear el WDT para indicar actividad
        esp_task_wdt_reset();
        read_buttons();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void control_task(void* arg) {
    // LLAMADA UNICA: Suscribir la tarea al WDT
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_LOGI(TAG, "Tarea control iniciada");
    
    for (;;) {
        // LLAMADA REPETIDA: Resetear el WDT para indicar actividad
        esp_task_wdt_reset();
        update_potentiometer();
        monitor_ina226();
        vTaskDelay(pdMS_TO_TICKS(100)); // Controla el delay cada 100 ms
    }
}

static void monitor_task(void* arg) {
    // LLAMADA UNICA: Suscribir la tarea al WDT
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_LOGI(TAG, "Tarea monitoreo iniciada");
    
    while (true) {
        // LLAMADA REPETIDA: Resetear el WDT para indicar actividad
        esp_task_wdt_reset();
        
        // Log consolidado (PULSOS + POT/DELAY) - Solo cada 500 ms (SOLICITADO)
        
        // Cálculo de Corriente para el Log (Inversión del mapeo)
        // Usamos g_current_delay_max_us en lugar de la constante
        uint32_t current_point = (uint32_t)floorf(((float)g_current_delay_max_us - g_scr_delay_us) / DELAY_STEP_US);
        
        // Asegurar límites del punto de 0 a (NUM_POINTS - 1)
        if (current_point >= (uint32_t)NUM_POINTS_F) current_point = (uint32_t)NUM_POINTS_F - 1;
        
        // Corriente = Punto * CURRENT_STEP_A
        uint32_t current_amps = (uint32_t)floorf((float)current_point * CURRENT_STEP_A);

        // Periodo Medido: mostrar el valor del mínimo histórico usado como base para Max Delay
        uint32_t measured_period_log = g_semi_period_measured_us;
        
        ESP_LOGI(TAG, "Monitor: Pulsos: A=%lu B=%lu C=%lu | Pot: %.0f mV | Delay: %lu us | Corriente: %lu A | Max Delay: %lu us | Periodo Minimo Historico (T/2): %lu us | SCR=%s",
                 (unsigned long)g_pulse_count[0],
                 (unsigned long)g_pulse_count[1], 
                 (unsigned long)g_pulse_count[2],
                 (double)g_pot_mv, // Imprime el valor EMA suavizado
                 (unsigned long)g_scr_delay_us,
                 (unsigned long)current_amps, // Imprime la corriente discreta
                 (unsigned long)g_current_delay_max_us, // Imprime el DELAY_MAX DINÁMICO
                 (unsigned long)measured_period_log, // NUEVO: Imprime el periodo mínimo histórico
                 g_scr_enabled ? "ON" : "OFF");
        
        vTaskDelay(pdMS_TO_TICKS(500)); // Log cada 500 ms
    }
}

// === Habilitación de Fases basada en Potenciómetro ===
static void update_phases_enable() {
    // Si g_scr_enabled cambia, actualiza las fases
    static bool last_scr_enabled = false;
    
    if (g_scr_enabled != last_scr_enabled) {
        if (g_scr_enabled) {
             ESP_LOGI(TAG, "Habilitando todas las fases SCR.");
             for (int i = 0; i < 3; i++) {
                 ph[i].enabled = true;
             }
        } else {
             ESP_LOGI(TAG, "Deshabilitando todas las fases SCR.");
             for (int i = 0; i < 3; i++) {
                 ph[i].enabled = false;
                 gpio_set_level(SCR_PIN[i], 0);
                 // No detenemos el timer, solo lo deshabilitamos de la ISR
             }
        }
        last_scr_enabled = g_scr_enabled;
    }
    
    // Lógica para habilitar/deshabilitar fases individuales si fuera necesario
    // (Actual: todas habilitadas si g_scr_enabled es true)
}

// === Inicialización de Fase ===
static void init_phase(int i, gpio_num_t zc, gpio_num_t scr) {
    // SCR como salida
    gpio_config_t outc = {
        .pin_bit_mask = (1ULL << scr),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&outc));
    
    gpio_set_level(scr, 0);

    // ZC como entrada con detección de AMBOS flancos
    gpio_config_t inc = {
        .pin_bit_mask = (1ULL << zc),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&inc));

    // Verificar configuración del pin ZC
    ESP_LOGI(TAG, "Configurando ZC: pin=%d, pullup=ENABLED, intr=ANYEDGE", zc);
    
    // Leer nivel inicial
    int initial_level = gpio_get_level(zc);
    ESP_LOGI(TAG, "Nivel inicial ZC pin %d: %d", zc, initial_level);

    // Configurar Timer para la fase
    // GPTimer para ESP-IDF v5.5.1
     gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
        .intr_priority = 1,
        .flags = {
            .intr_shared = false,
            .allow_pd = false,
            .backup_before_sleep = false
        }
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &ph[i].timer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = on_alarm,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(ph[i].timer, &cbs, &ph[i]));
    ESP_ERROR_CHECK(gptimer_enable(ph[i].timer));
    ESP_ERROR_CHECK(gptimer_start(ph[i].timer)); // Dejar el timer corriendo

    ph[i].idx = i;
    ph[i].last_rise_us = 0;
    ph[i].last_fall_us = 0;
    ph[i].next_is_high = false;
    ph[i].enabled = false;
    //ph[i].timer = 100;

    static bool isr_installed = false;
    if (!isr_installed) {
        ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
        isr_installed = true;
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(zc, zc_isr, (void*)(intptr_t)i));

    ESP_LOGI(TAG, "Fase %d lista: ZC=%d SCR=%d. Intr instalada.", i, (int)zc, (int)scr);
}

// === Inicialización I2C ===
static void i2c_init() {
    // Puerto 0 para ADS1115 y MCP23017
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_HZ,
        },
        .clk_flags = 0,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    // Puerto 1 para INA226
    i2c_config_t i2c_config1 = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C1_SDA,
        .scl_io_num = I2C1_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 100000, // INA226 típicamente a 100kHz
        },
        .clk_flags = 0,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT_1, &i2c_config1));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT_1, I2C_MODE_MASTER, 0, 0, 0));
}

// === Inicialización de pines de habilitación MCP23017 ===
static void initialize_mcp_enables() {
    const gpio_num_t pin_15 = GPIO_NUM_15;
    const gpio_num_t pin_41 = GPIO_NUM_41;

    gpio_config_t io_config = {
        .pin_bit_mask = (1ULL << pin_15) | (1ULL << pin_41),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_config));

    gpio_set_level(pin_15, 1);  // Sacar MCP23017 del reset
    gpio_set_level(pin_41, 1);   // Otro pin de habilitación
    
    ESP_LOGI(TAG, "Enables MCP: GPIO15=1 GPIO41=1");
}

extern "C" void app_main(void) {
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "=== INICIANDO SISTEMA COMPLETO ESP-IDF v5.5.1 ===");

    // El watchdog se configura automáticamente, solo reconfiguramos el timeout.
    
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 60000,  // 60 segundos
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    if (esp_task_wdt_reconfigure(&wdt_config) != ESP_OK) {
        ESP_LOGW(TAG, "Error reconfigurando WDT. Usando configuración por defecto.");
    } else {
        ESP_LOGI(TAG, "Watchdog configurado a 60 segundos");
    }

    // Agregar la tarea principal al watchdog
    if (esp_task_wdt_add(NULL) != ESP_OK) {
        ESP_LOGW(TAG, "Error agregando tarea principal al WDT.");
    }


    // ¡PRIMERO habilitar el MCP23017!
    initialize_mcp_enables();

    // Mutex
    g_i2c_mutex = xSemaphoreCreateMutex();
    if (g_i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Error creando mutex I2C");
        return;
    }

    // I2C
    i2c_init();

    // Dispositivos I2C
    g_ads = new ADS1115(I2C_PORT, 0x48, g_i2c_mutex);
    if (!g_ads->begin()) {
        ESP_LOGE(TAG, "Error inicializando ADS1115");
    }

    g_mcp = new MCP23017(I2C_PORT, 0x27);
    if (g_mcp->begin()) {
        // Configurar pines MCP23017
        g_mcp->pin_mode(0, 0);  // A0 - OUTPUT
        g_mcp->pin_mode(1, 0);  // A1 - OUTPUT  
        g_mcp->pin_mode(2, 0);  // A2 - OUTPUT
        g_mcp->pin_mode(3, 0);  // A3 - OUTPUT
        g_mcp->pin_mode(8, 1);  // B0 - INPUT (Start)
        g_mcp->pin_mode(9, 1);  // B1 - INPUT (Direction)
        g_mcp->pin_pullup(8, true);
        g_mcp->pin_pullup(9, true);
        
        control_relays(); // Estado inicial
        ESP_LOGI(TAG, "MCP23017 configurado");
    } else {
        ESP_LOGE(TAG, "Error inicializando MCP23017");
    }

    g_ina = new INA226(I2C_PORT_1, 0x40, g_i2c_mutex);
    if (g_ina->begin()) {
        ESP_LOGI(TAG, "INA226 configurado (solo monitoreo)");
    } else {
        ESP_LOGW(TAG, "INA226 no detectado, continuando sin monitoreo de corriente");
    }

    // Fases SCR
    for (int i = 0; i < 3; ++i) {
        init_phase(i, ZC_PIN[i], SCR_PIN[i]);
    }

    // Tareas
    if (xTaskCreate(button_task, "buttons", 4096, nullptr, 6, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Error creando tarea botones");
    }
    if (xTaskCreate(control_task, "control", 8192, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Error creando tarea control");
    }
    if (xTaskCreate(monitor_task, "monitor", 4096, nullptr, 1, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Error creando tarea monitoreo");
    }

    ESP_LOGI(TAG, "Sistema listo. Esperando comando START...");
    ESP_LOGI(TAG, "Mapeo Pot: %.0f mV-%.0f mV -> 0A-%.0fA (%.0f pasos, %.1f us/paso) | Max Delay Seguro: %lu us",
             POT_MIN_MV, POT_MAX_MV, MAX_CURRENT_A, NUM_POINTS_F, DELAY_STEP_US, SAFE_MAX_DELAY_US);

    // Loop principal
    while (true) {
        esp_task_wdt_reset();
        update_phases_enable();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}