// main.cpp — ESP-IDF v5.5.1 — SCR gate desde ZC con GPTimer 1 MHz
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char* TAG = "SCR";

// === Ajustes ===
static constexpr uint32_t DELAY_US       = 4000;  // retardo desde ZC hasta flanco de gate
static constexpr uint32_t PULSE_US       = 100;   // ancho del gate
static constexpr uint32_t DEBOUNCE_US    = 200;   // anti-rebote ZC
static constexpr int32_t  DELAY_TRIM_US  = -10;   // compensación de offset medido (+/- corrige)

// Pines (tus asignaciones)
static const gpio_num_t ZC_PIN[3]  = { GPIO_NUM_38, GPIO_NUM_21, GPIO_NUM_14 };
static const gpio_num_t SCR_PIN[3] = { GPIO_NUM_48, GPIO_NUM_47, GPIO_NUM_13 };

// Estado por fase
typedef struct {
    gptimer_handle_t timer;          // temporizador libre a 1 MHz
    int              idx;            // índice 0/1/2
    volatile uint32_t last_rise_us;  // para debounce por soft (us)
    volatile bool     next_is_high;  // alterna HIGH -> LOW dentro del ciclo
} phase_t;

static phase_t ph[3];

// Helpers de tiempo
static inline uint32_t now_us() { return (uint32_t)esp_timer_get_time(); }

// ISR de alarma: genera el pulso (HIGH y luego LOW) en SCR_PIN[idx]
static bool IRAM_ATTR on_alarm(gptimer_handle_t t,
                               const gptimer_alarm_event_data_t* edata,
                               void* user)
{
    phase_t* p = (phase_t*)user;

    if (p->next_is_high) {
        // Subir gate
        gpio_set_level(SCR_PIN[p->idx], 1);

        // Programar fin de pulso a +PULSE_US
        gptimer_alarm_config_t a2 = {
            .alarm_count  = edata->count_value + PULSE_US,
            .reload_count = 0,
            .flags = { .auto_reload_on_alarm = false }
        };
        gptimer_set_alarm_action(t, &a2);
        p->next_is_high = false;
    } else {
        // Bajar gate y mandar la siguiente alarma "lejos" (sin auto-reload)
        gpio_set_level(SCR_PIN[p->idx], 0);
        gptimer_alarm_config_t aclr = {
            .alarm_count  = edata->count_value + 0x7FFFFFFF, // sin próximas
            .reload_count = 0,
            .flags = { .auto_reload_on_alarm = false }
        };
        gptimer_set_alarm_action(t, &aclr);
    }
    return true; // ISR handled
}

// ISR de ZC (solo flanco de subida): agenda el pulso a (DELAY_US + TRIM)
static void IRAM_ATTR zc_isr(void* arg)
{
    const int idx = (int)(intptr_t)arg;

    // Debounce simple en us
    uint32_t t = now_us();
    uint32_t last = ph[idx].last_rise_us;
    if ((uint32_t)(t - last) < DEBOUNCE_US) return;
    ph[idx].last_rise_us = t;

    // Fuerza gate en LOW al cruce por cero
    gpio_set_level(SCR_PIN[idx], 0);

    // Lee contador del GPT de esta fase y programa la próxima alarma
    uint64_t cur = 0;
    gptimer_get_raw_count(ph[idx].timer, &cur);

    int32_t eff_delay = (int32_t)DELAY_US + (int32_t)DELAY_TRIM_US;
    if (eff_delay < 0) eff_delay = 0;

    gptimer_alarm_config_t a1 = {
        .alarm_count  = cur + (uint32_t)eff_delay,
        .reload_count = 0,
        .flags = { .auto_reload_on_alarm = false }
    };
    gptimer_set_alarm_action(ph[idx].timer, &a1);
    ph[idx].next_is_high = true;
}

// Inicializa una fase: SCR salida, ZC entrada con interrupción, GPTimer 1 MHz libre
static void init_phase(int i, gpio_num_t zc, gpio_num_t scr)
{
    // SCR como salida con pulldown (evita flancos espurios al boot)
    gpio_config_t outc = {
        .pin_bit_mask = 1ULL << scr,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&outc));
    gpio_set_level(scr, 0);

    // ZC como entrada + pullup + interrupción SOLO flanco de subida
    gpio_config_t inc = {
        .pin_bit_mask = 1ULL << zc,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&inc));

    // GPTimer 1 MHz, libre y corriendo siempre (menor jitter)
    gptimer_config_t tc = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 tick = 1 us
        .intr_priority = 1
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&tc, &ph[i].timer));
    gptimer_event_callbacks_t cbs = { .on_alarm = on_alarm };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(ph[i].timer, &cbs, &ph[i]));
    ESP_ERROR_CHECK(gptimer_enable(ph[i].timer));
    ESP_ERROR_CHECK(gptimer_start(ph[i].timer)); // siempre ON

    ph[i].idx = i;
    ph[i].last_rise_us = 0;
    ph[i].next_is_high = false;

    // Instala servicio de ISR GPIO una sola vez
    static bool isr_installed = false;
    if (!isr_installed) {
        esp_err_t gi = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        if (gi != ESP_OK && gi != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(gi);
        isr_installed = true;
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(zc, zc_isr, (void*)(intptr_t)i));

    ESP_LOGI(TAG, "Fase %d lista: ZC=%d SCR=%d", i, (int)zc, (int)scr);
}

extern "C" void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);

    for (int i = 0; i < 3; ++i) init_phase(i, ZC_PIN[i], SCR_PIN[i]);

    ESP_LOGI(TAG, "DELAY=%lu us  PULSE=%lu us  TRIM=%ld us  => efectivo ≈ %ld us",
             (unsigned long)DELAY_US,
             (unsigned long)PULSE_US,
             (long)DELAY_TRIM_US,
             (long)((int32_t)DELAY_US + (int32_t)DELAY_TRIM_US));

    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
}
