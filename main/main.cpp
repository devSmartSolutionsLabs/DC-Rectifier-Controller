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

// Componentes de Red y OTA
#include "WifiManager.hpp"
#include "GitHubClient.hpp"
#include "PortalWeb.hpp"
#include "esp_ota_ops.h" // Necesario para consultar la descripción de la app
#include "LoggerFS.hpp"
#include "esp_sntp.h" // Necesario para la hora de Lima

#include "driver/uart.h"
#include "driver/usb_serial_jtag.h" // Asegúrate de incluir esta cabecera
#include "CommandManager.hpp"

#define CURRENT_VERSION "3.0.3"
float g_corriente_actual = 0.0f;
int g_potenciometro_mv = 0;
bool g_scr_activo = false;
static const char* TAG = "RECTIFICADOR";
// Instancia del portal
static PortalWeb g_portal;
// Global en main.cpp
volatile bool g_is_wifi_scanning = false; // Aquí sí se define y se inicializa
// === Configuración SCR ===
static constexpr uint32_t PULSE_US       = 700;     // ANCHO DEL PULSO: 700 us
static constexpr uint32_t DEBOUNCE_US    = 2500;    // anti-rebote ZC (2.5 ms)
static constexpr uint32_t DEFAULT_SEMI_PERIOD_US = 8333; // 60Hz

// Pines 
static const gpio_num_t ZC_PIN[3]  = { GPIO_NUM_38, GPIO_NUM_21, GPIO_NUM_43 };
static const gpio_num_t SCR_PIN[3] = { GPIO_NUM_48, GPIO_NUM_47, GPIO_NUM_44 };

// === Configuración I2C ÚNICA (Consolidada) ===
static constexpr i2c_port_t I2C_PORT = I2C_NUM_0;
static constexpr gpio_num_t I2C_SDA  = GPIO_NUM_5;
static constexpr gpio_num_t I2C_SCL  = GPIO_NUM_4;
static constexpr uint32_t   I2C_HZ   = 400000;

// === CONSTANTES GLOBALES DE MAPEO (Refactorizadas) ===
static constexpr float MAX_CURRENT_A    = 5000.0f;      // Corriente máxima total
static constexpr float CURRENT_STEP_A   = 5.0f;         // Paso de corriente deseado (5A)
static constexpr float DELAY_STEP_US    = 5.75f;         // Paso de delay deseado (6.0 us/punto)

// Constantes Derivadas
static constexpr float NUM_POINTS_F     = MAX_CURRENT_A / CURRENT_STEP_A; // 5000A / 5A = 1000.0f
static constexpr float DELAY_RANGE_US_F = NUM_POINTS_F * DELAY_STEP_US;   // 1000.0f * 6.0f = 6000.0f

// Límites de Potenciómetro
static constexpr float POT_MIN_MV       = 200.0f;  // Mínimo mapeado
static constexpr float POT_MAX_MV       = 4100.0f;  // Máximo mapeado
static constexpr float MV_RANGE         = POT_MAX_MV - POT_MIN_MV; 

// Nuevo límite superior de seguridad
static constexpr uint32_t SAFE_MAX_DELAY_US = 8320; // Hard cap para el delay máximo (seguridad)

// === LÍMITES DE VALIDACIÓN DE FRECUENCIA ===
static constexpr uint32_t MIN_PERIOD_VALID_US = 8264; // 60.5 Hz (Período más corto)
static constexpr uint32_t MAX_PERIOD_VALID_US = 8404; // 59.5 Hz (Período más largo)
// ===================================================

// El DELAY MAX y MIN AHORA SERÁN CALCULADOS DINÁMICAMENTE.
static volatile uint32_t g_scr_delay_us = DEFAULT_SEMI_PERIOD_US; // Valor inicial
static volatile uint32_t g_current_delay_max_us = DEFAULT_SEMI_PERIOD_US;
static volatile uint32_t g_current_delay_min_us = (uint32_t)(DEFAULT_SEMI_PERIOD_US - DELAY_RANGE_US_F);

// === Variables Globales del Sistema ===
static volatile bool g_system_started = false;
extern "C" {
    volatile bool g_scr_enabled = false;
}
static volatile uint32_t g_pulse_count[3] = {0, 0, 0};
static volatile float g_pot_mv = 0.0f; // Almacena el último valor del potenciómetro

// === Variables de Medición de Periodo Adaptativo (MÍNIMO HISTÓRICO) ===
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
MCP23017* g_mcp_2 = nullptr;  // El nuevo para la SD y otros usos (0x20)
static INA226* g_ina = nullptr;

// Instancia Global del Logger
LoggerFS g_logger("/sd");

RectStatus obtener_estado_actual() {
    RectStatus status;
    // Mapeamos la dirección según los relays de dirección
    status.direction = a2_on ? RectDirection::FORWARD : RectDirection::REVERSE;
    status.current   = g_corriente_actual;
    status.voltage   = (float)g_potenciometro_mv; 
    status.temp      = 0; // Placeholder por ahora
    return status;
}
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
            g_logger.registrarEstructurado(
                RectEvent::BTN_START_PRESS, 
                "START PRESSED", 
                "Arranque confirmado"
            );
        }
        
        start_hold_ms += dt;
        
        if (!step1_done && start_hold_ms >= 2000) {
            a0_on = true;
            g_scr_enabled = true;
            step1_done = true;
            ESP_LOGI(TAG, "STEP1: A0 ON + SCR habilitados");
            g_logger.registrarEstructurado(RectEvent::PROCESS_START, "SCR_ON", "Etapa 1: Relays y SCR habilitados");
            control_relays();
        }
        
        if (!step2_done && start_hold_ms >= 3000) {
            a1_on = true;
            set_direction(!b1_pressed);  // Invertido según tu lógica
            step2_done = true;
            ESP_LOGI(TAG, "STEP2: A1 ON + Dirección fijada");
            if(b1_pressed)  g_logger.registrarEstructurado(RectEvent::PROCESS_START, "FORWARD", "Etapa 2: Direccion fijada");
            else g_logger.registrarEstructurado(RectEvent::PROCESS_START, "REVERSE", "Etapa 2: Direccion fijada");
            control_relays();
        }
    } else {
        if (last_b0){
        g_logger.registrarEstructurado(RectEvent::BTN_START_RELEASE, "-", "Boton START liberado");        
            if ((a0_on || a1_on || g_scr_enabled)) {
                // Iniciar secuencia de apagado
                a1_on = false;
                g_scr_enabled = false;
                shutting_down = true;
                ESP_LOGI(TAG, "Iniciando apagado...");
                g_logger.registrarEstructurado(RectEvent::PROCESS_STOP, "SHUTDOWN", "Iniciando secuencia de parada");
                control_relays();
            }
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

void i2c_scanner() {
    ESP_LOGI("SCANNER", "Iniciando escaneo del bus I2C...");
    int encontrados = 0;
    for (int i = 1; i < 127; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(10));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            ESP_LOGI("SCANNER", "Dispositivo encontrado en direccion: 0x%02X", i);
            encontrados++;
        }
    }
    if (encontrados == 0) ESP_LOGW("SCANNER", "No se encontraron dispositivos I2C.");
}

// === Lectura de Potenciómetro y Mapeo Adaptativo ===
static void update_potentiometer() {
    if (g_is_wifi_scanning) {
        // No hacemos nada. El sistema usará el último valor de 'delay' guardado.
        return; 
    }

    if (!g_ads || !g_i2c_mutex) return;

    if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        const int NUM_SAMPLES = 7; // Reducimos muestras para no saturar el bus
        float samples[NUM_SAMPLES];
        bool success = true;

        for (int i = 0; i < NUM_SAMPLES; ++i) {
            if (!g_ads->singleShotMV(ADS1115::Mux::DIFF_0_1, ADS1115::PGA::FS_6V144, ADS1115::DataRate::SPS_64, samples[i])) {
                success = false;
                break;
            }
        }
        
        xSemaphoreGive(g_i2c_mutex); // Liberar bus

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
        uint32_t dynamic_semi_period = g_semi_period_measured_us; 
        
        // Aplicamos límites de validación de frecuencia para el fallback
        if (dynamic_semi_period < MIN_PERIOD_VALID_US || dynamic_semi_period > MAX_PERIOD_VALID_US) { 
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
        float normalized = (mv - POT_MIN_MV) / MV_RANGE;
        
        // *** 4. COMPENSACIÓN INVERSA POR RAÍZ CUADRADA (Square Root Compensation) ***
        // Propiedad: Genera un cambio RÁPIDO al inicio (bajo V_POT) y LENTO al final (alto V_POT),
        // lo que da mayor resolución de control en la zona de alta corriente.
        float compensated_factor = sqrtf(normalized); 
        
        // Convertir el valor compensado (0 a 1) a un punto de delay (0 a 999)
        float point_float = compensated_factor * NUM_POINTS_F;
        uint32_t current_point = (uint32_t)floorf(point_float);
        
        if (current_point >= (uint32_t)NUM_POINTS_F) {
            current_point = (uint32_t)NUM_POINTS_F - 1; 
        }
        
        // 5. Mapeo Invertido y Discreto a Delay (usando el dynamic_delay_max capado)
        // new_delay = DELAY_MAX - (current_point_compensado * DELAY_STEP_US)
        uint32_t new_delay = (uint32_t)(dynamic_delay_max - ((float)current_point * DELAY_STEP_US));
        
        // 6. Asegurar límites finales
        if (new_delay < dynamic_delay_min) new_delay = dynamic_delay_min;
        if (new_delay > dynamic_delay_max) new_delay = dynamic_delay_max;
        
        // ALMACENAR VALOR FINAL DEL DELAY (usado por la ISR)
        g_scr_delay_us = new_delay;
    }
    else {
        ESP_LOGW(TAG, "I2C ocupado, saltando lectura de pot.");
    }
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
        
        // ******* PULSO DE ANCHO FIJO (PULSE_US) *******
        // Programar la alarma para apagar el pulso después de PULSE_US
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = edata->alarm_value + PULSE_US,
            .reload_count = 0,
            .flags = {
                .auto_reload_on_alarm = false
            }
        };
        gptimer_set_alarm_action(p->timer, &alarm_config);
        p->next_is_high = false; // El próximo evento apaga el pulso
        
    } else {
        // Apagado del Pulso (LOW) - Activado por la alarma de PULSE_US
        gpio_set_level(SCR_PIN[p->idx], 0); 
        
        // Deshabilitar futuras alarmas (hasta el próximo ZC)
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = edata->alarm_value + 0x7FFFFFFF,
            .reload_count = 0,
            .flags = { .auto_reload_on_alarm = false }
        };
        gptimer_set_alarm_action(p->timer, &alarm_config);
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
    // SEGURIDAD ZC: APAGA CUALQUIER PULSO ACTIVO (ya sea corto o extendido).
    gpio_set_level(SCR_PIN[idx], 0); 
    
    // CORRECCIÓN: AUMENTAR EL CONTADOR DE PULSOS AQUÍ (Semiciclo ZC Procesado)
    uint32_t temp = g_pulse_count[idx];
    g_pulse_count[idx] = temp + 1;
    
    // Detener el timer cancela cualquier alarma de disparo o apagado que pudiera estar pendiente.
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
    
    // 3. Programar alarma para DISPARO (que activará la alarma de apagado de 50 us en on_alarm)
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = (uint64_t)delay, // El contador inicia en 0, la alarma es el delay
        .reload_count = 0,
        .flags = { .auto_reload_on_alarm = false }
    }
    ;
    gptimer_set_alarm_action(p->timer, &alarm_config);
    p->next_is_high = true; // El próximo evento del timer es HIGH (el inicio del pulso)
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
        vTaskDelay(pdMS_TO_TICKS(100));
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
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL)); 
    while (true) {
        esp_task_wdt_reset();
        
        // Cálculo de Corriente para el Log y la Telemetría
        uint32_t current_point = (uint32_t)floorf(((float)g_current_delay_max_us - g_scr_delay_us) / DELAY_STEP_US);
        if (current_point >= (uint32_t)NUM_POINTS_F) current_point = (uint32_t)NUM_POINTS_F - 1;
        
        float current_amps = (float)current_point * CURRENT_STEP_A;

        // TELEMETRÍA: Actualizar variables globales para el WebSocket
        g_corriente_actual = current_amps;
        g_scr_activo = g_scr_enabled;

        ESP_LOGI(TAG, "Monitor: Pulsos: A=%lu B=%lu C=%lu | Pot: %.0f mV | Delay: %lu us | Corriente: %.0f A | SCR=%s",
                 (unsigned long)g_pulse_count[0], (unsigned long)g_pulse_count[1], (unsigned long)g_pulse_count[2],
                 (double)g_pot_mv, (unsigned long)g_scr_delay_us, (double)current_amps, g_scr_enabled ? "ON" : "OFF");
        
        vTaskDelay(pdMS_TO_TICKS(5000)); 
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
        .intr_priority = 0,
        .flags = {
            .intr_shared = true, // solo por agregar el wifi
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

void iniciar_sincronizacion_tiempo() {
    ESP_LOGI(TAG, "Configurando SNTP para Lima, Peru...");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL); // Usar esp_sntp_...
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "south-america.pool.ntp.org");
    esp_sntp_init(); // Usar esp_sntp_...

    // Lima no tiene horario de verano, es UTC-5 fijo
    setenv("TZ", "PET5", 1); 
    tzset();
}

extern "C" void app_main(void) {
    const esp_app_desc_t *app_desc = esp_app_get_description();    
    ESP_LOGI("SISTEMA", "##########################################");
    ESP_LOGI("SISTEMA", " PROYECTO: %s", app_desc->project_name);
    ESP_LOGI("SISTEMA", " VERSION : %s", app_desc->version); // Aquí saldrá "3.0.1"
    ESP_LOGI("SISTEMA", " COMPILADO: %s %s", app_desc->date, app_desc->time);
    ESP_LOGI("SISTEMA", "##########################################");
    
    // 4. HARDWARE: MCP23017 Y BUS I2C
    initialize_mcp_enables();
    i2c_init();

    g_mcp_2 = new MCP23017(I2C_PORT, 0x25); // El nuevo MCP en dirección 0x20
    if (g_mcp_2->begin()) {
        ESP_LOGI("MCP_2", "Segundo MCP detectado en 0x20");
        // Aquí configuras el pin GPB5 para el Chip Select de la SD
        g_mcp_2->pin_mode(13, 0); // CS - Pin 13 es GPB5, modo 0 es OUTPUT
        g_mcp_2->pin_mode(14, 1); // CD - Pin 14 es GPB6, modo 0 es INPUT
        
        g_mcp_2->digital_write(13, 0); // CS en alto (deseleccionado)
        g_mcp_2->pin_pullup(14, true);
        
    } else {
        ESP_LOGE("MCP_2", "No se encontró el segundo MCP");
    }
    ///////////////
    vTaskDelay(pdMS_TO_TICKS(50));
    long last_t = WifiManager::get_last_time();
    if (last_t > 1700000000) { // Si es una fecha válida post-2023
        struct timeval tv = { .tv_sec = last_t };
        settimeofday(&tv, NULL);
        ESP_LOGI("TIME", "Reloj recuperado de NVS");
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    if (g_logger.begin()) {
        ESP_LOGI("MAIN", "Escribiendo log de prueba...");
        //g_logger.registrarEstructurado(RectEvent::BOOT, "3.0.3-SD", "Prueba de escritura manual");
    } else {
        ESP_LOGE(TAG, "ERROR: Tarjeta SD no lista. Revisa GPB6.");
    }
    
    struct stat st;
    if (stat("/sd", &st) == 0) {
        ESP_LOGI("VERIFICACION", "La ruta /sd existe y es accesible.");
    } else {
        ESP_LOGE("VERIFICACION", "La ruta /sd NO existe en el VFS.");
    }

    // Primero inicializar los componentes de red/memoria
    WifiManager::init();

    iniciar_sincronizacion_tiempo();
    // Registro inicial: BOOT
    g_logger.registrarEstructurado(
                RectEvent::BOOT,            // Esto grabará 0x0100
                "v" CURRENT_VERSION,        // Valor: "v3.0.3"
                "Arranque del sistema"      // Nota
            );

    if (WifiManager::connect_saved()) {
        ESP_LOGI(TAG, "Intentando conectar a red guardada...");
        
        bool conectado = false;
        for(int i = 0; i < 150; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
            if (WifiManager::is_connected()) {
                conectado = true;
                break;
            }
            if (WifiManager::should_fallback()) break;
        }

        if (conectado) {
            // En lugar de registrar inmediatamente, podrías usar un flag
            // o imprimir un log simple primero para ver si sobrevive
            ESP_LOGI(TAG, "Conexión exitosa. IP: %s", WifiManager::get_ip().c_str());
            vTaskDelay(pdMS_TO_TICKS(500));
            // Intenta registrar con una nota estática (ocupa menos stack)
            g_logger.registrarEstructurado(RectEvent::NET_SSID, WifiManager::get_ssid(), "Conectado");
            vTaskDelay(pdMS_TO_TICKS(500));
            g_logger.registrarEstructurado(RectEvent::NET_IP, WifiManager::get_ip(), "DHCP OK");
        }
        else{
            vTaskDelay(pdMS_TO_TICKS(500));
            g_logger.registrarEstructurado(RectEvent::ERR_SYSTEM, "WIFI_TIMEOUT", "Fallo conexion a red guardada");
        }
    }

    // Si no hay red guardada, o si fallaron los reintentos:
    if (!WifiManager::is_connected()) {
        ESP_LOGW(TAG, "Abriendo modo configuracion (Portal)");
        // REGISTRO: Modo Punto de Acceso activo
        g_logger.registrarEstructurado(
                RectEvent::NET_AP_START,    // Definir como 0x0700
                "192.168.4.1",             // Valor: La IP del portal
                "Portal AP Activo"         // Nota
            );
        WifiManager::start_ap();
    }
    
    if (g_portal.start() == ESP_OK) {
        ESP_LOGI(TAG, "Servidor Web iniciado en: %s", 
                WifiManager::is_connected() ? WifiManager::get_ip().c_str() : "192.168.4.1");
    }

    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "=== INICIANDO SISTEMA COMPLETO ESP-IDF v5.5.1 ===");

    // 2. CONFIGURACIÓN DEL WATCHDOG (WDT)
    
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
    
    // 3. Inicializa el driver del puerto nativo para permitir lectura
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .tx_buffer_size = 256,
        .rx_buffer_size = 256,
    };
    usb_serial_jtag_driver_install(&usb_serial_jtag_config);

    
    // Mutex
    g_i2c_mutex = xSemaphoreCreateMutex();
    if (g_i2c_mutex == NULL) {
        // Categoría 0x05 (Errores) | Evento 0x0503 (Error de Recurso/Mutex)
        g_logger.registrarEstructurado(
            RectEvent::ERR_SYSTEM,      // Definir como 0x0503
            "I2C_MUTEX",               // Valor: El recurso que falló
            "Fallo creacion Mutex I2C" // Nota
        );
        ESP_LOGE(TAG, "Error creando mutex I2C");
        return;
    }

    // 5. DISPOSITIVOS I2C
    g_ads = new ADS1115(I2C_PORT, 0x48, g_i2c_mutex);
    if (!g_ads->begin()) {
        ESP_LOGE(TAG, "Error inicializando ADS1115");
        // Categoría 0x05 (Errores) | Evento 0x0501 (Error de Dispositivo I2C)
        g_logger.registrarEstructurado(
            RectEvent::ERR_I2C,         // Definir como 0x0501
            "ADS1115",                 // Valor: ID del chip no detectado
            "Sensor no detectado"      // Nota
        );
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
        g_logger.registrarEstructurado(RectEvent::ERR_I2C, "MCP23017", "Fallo de inicializacion");
    }
    
    g_ina = new INA226(I2C_PORT, 0x40, g_i2c_mutex);
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
    g_logger.registrarEstructurado(RectEvent::CONFIG_CHANGE,   // Esto grabará 0x0600 (o el sub-ID que definas)
                                    "READY",                    // El valor específico
                                    "Hardware Listo - Esperando Operario" // La nota descriptiva
            );

    //ESP_LOGI(TAG, "Mapeo Pot: %.0f mV-%.0f mV -> 0A-%.0fA (%.0f pasos, %.1f us/paso) | Max Delay Seguro: %lu us",
    //         POT_MIN_MV, POT_MAX_MV, MAX_CURRENT_A, NUM_POINTS_F, DELAY_STEP_US, SAFE_MAX_DELAY_US);

    // Loop principal

    uint32_t loop_counter = 0;
    static std::string acumulador_serie = "";
    while (true) {
        esp_task_wdt_reset();
        update_phases_enable();
        if (loop_counter++ >= 10) {
            update_phases_enable();
            g_scr_activo = g_scr_enabled;
            loop_counter = 0; 
        }

        // LECTURA POR PUERTO USB NATIVO (Pines 19 y 20)
        uint8_t n_buf[64];
        // Usamos un timeout de 0 para no bloquear el resto del sistema
        int len = usb_serial_jtag_read_bytes(n_buf, sizeof(n_buf), 0);
        
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = (char)n_buf[i];
                
                // Procesar al recibir Enter
                if (c == '\n' || c == '\r') {
                    if (!acumulador_serie.empty()) {
                        std::string respuesta = CommandManager::execute(acumulador_serie);
                        // Usamos printf, que ya está ruteado al USB nativo por defecto
                        printf("\r\n%s\r\n> ", respuesta.c_str());
                        fflush(stdout); 
                        acumulador_serie.clear();
                    }
                } else {
                    acumulador_serie += c;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}