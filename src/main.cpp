#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/mcpwm.h>
#include <esp_rom_sys.h>
#include <Adafruit_ADS1X15.h>
#include "Calibracion.hpp"
#include "Sensores.hpp"
#include "TTL.hpp"
#include "MCP23017_IO.hpp"
#include "GlobalVars.hpp"
#include "mutexDebug.hpp"
#include "esp_task_wdt.h"

// ================== CONFIGURACIÓN GENERAL ==================
#define SCR_PULSE_WIDTH_US 400    // duración fija del pulso
#define NUM_FASES          3
#define SEMI_PERIOD_US     8333   // duración semionda 60 Hz

// ================== VARIABLES GLOBALES ==================
QueueHandle_t zcQueues[NUM_FASES] = {NULL, NULL, NULL};
Sensores sensores;

// Pines
const int zcPins[NUM_FASES]  = {38, 47, 14};
const int scrPins[NUM_FASES] = {48, 21, 13};

// Variables de control
volatile uint32_t lastZCTime[NUM_FASES] = {0};
volatile uint32_t pulseCount[NUM_FASES] = {0};


// scrDelayUs se declara en GlobalVars.hpp → solo se asigna aquí
// extern uint32_t scrDelayUs;   // ya viene del header
// Inicialización en setup()
mcpwm_unit_t mcpwmUnits[NUM_FASES]  = {MCPWM_UNIT_0, MCPWM_UNIT_1, MCPWM_UNIT_0};
mcpwm_timer_t mcpwmTimers[NUM_FASES] = {MCPWM_TIMER_0, MCPWM_TIMER_0, MCPWM_TIMER_1};

// ================== FUNCIONES ==================
void IRAM_ATTR forceTurnOffSCR(uint8_t dev) {
    gpio_set_level((gpio_num_t)scrPins[dev], 0);
}

// ---------- CONFIGURAR MCPWM PARA UNA FASE ----------
void setupMCPWMForSCR(uint8_t dev) {
    // Asignar pin de salida al MCPWM
    if (dev == 0) mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, scrPins[dev]);
    if (dev == 1) mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, scrPins[dev]);
    if (dev == 2) mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, scrPins[dev]);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 120;               // Hz (dummy, control manual)
    pwm_config.cmpr_a = 0;                    // duty inicial
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(mcpwmUnits[dev], mcpwmTimers[dev], &pwm_config);
    mcpwm_set_duty(mcpwmUnits[dev], mcpwmTimers[dev], MCPWM_OPR_A, 0);
    mcpwm_set_duty_type(mcpwmUnits[dev], mcpwmTimers[dev], MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    gpio_set_level((gpio_num_t)scrPins[dev], 0);
}

// ---------- ISR DE CRUCE POR CERO ----------
void IRAM_ATTR zcISR_Generic(void* arg) {
    uint8_t dev = (uint8_t)(intptr_t)arg;
    if (!systemStarted || !scrEnabled[dev]) return;

    uint32_t now = micros();
    if (now - lastZCTime[dev] < 1000) return;  // anti-rebote
    lastZCTime[dev] = now;

    gpio_set_level((gpio_num_t)scrPins[dev], 0);  // LOW inmediato
    uint32_t delay = scrDelayUs;
    if (delay >= SEMI_PERIOD_US) return;

    // Esperar retardo y disparar SCR con pulso de anchura fija
    esp_rom_delay_us(delay);
    gpio_set_level((gpio_num_t)scrPins[dev], 1);
    esp_rom_delay_us(SCR_PULSE_WIDTH_US);
    gpio_set_level((gpio_num_t)scrPins[dev], 0);

    pulseCount[dev]++;
}

// ---------- FUNCIONES VARIAS ----------
void enableWatchdog() {
    esp_task_wdt_init(30, false);
    esp_task_wdt_add(xTaskGetCurrentTaskHandle());
}

void resetWatchdog() {
    esp_task_wdt_reset();
}

// ================== TASK DE MONITOREO ==================
void controlTaskGeneric(void* param) {
    uint8_t idx = (uint8_t)(intptr_t)param;
    esp_task_wdt_add(NULL);

    Serial.printf("[MONITOR %c] iniciado\n", 'A' + idx);
    while (true) {
        esp_task_wdt_reset();
        static uint32_t lastLog = 0;
        if (millis() - lastLog > 10000) {
            Serial.printf("[FASE %c] Pulsos: %lu\n", 'A' + idx, pulseCount[idx]);
            lastLog = millis();
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

// ================== POTENCIÓMETRO ==================
void updateWaveBasedControl() {
    float rawV = sensores.getVoltage();
    const float POT_MIN_V = 2.30;
    const float POT_MAX_V = 4.00;
    float potNorm = constrain((rawV - POT_MIN_V) / (POT_MAX_V - POT_MIN_V), 0.0f, 1.0f);
    scrDelayUs = 8100 - (uint32_t)(potNorm * (8100 - 7500));
    scrDelayUs = constrain(scrDelayUs, 7500, 8100);

    static float lastV = 0;
    if (fabs(rawV - lastV) > 0.05) {
        Serial.printf("🎛 Pot: %.2f V → delay %lu µs\n", rawV, scrDelayUs);
        lastV = rawV;
    }
}

// ================== SETUP ==================
void setup() {
    Serial.begin(115200);
    pinMode(15,OUTPUT);
    pinMode(41,OUTPUT);
    digitalWrite(15,HIGH);
    digitalWrite(41,HIGH);
    delay(500);
    Serial.println("=== INICIANDO SISTEMA MCPWM ===");

    enableWatchdog();
    Wire.begin(5, 4);
    Wire.setClock(100000);

    if (i2cMutex == NULL) {
        i2cMutex = xSemaphoreCreateMutex();
        giveI2CMutex();
    }

    sensores.begin();
    ioController.begin(5, 4);
    ioController.setRelay(1, true);
    Serial.println("⚙️ Dirección DIRECTA (Relé 2 ON)");

    // Configurar ISR de cruces y MCPWM
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    for (int i = 0; i < NUM_FASES; i++) {
        pinMode(zcPins[i], INPUT_PULLDOWN);
        pinMode(scrPins[i], OUTPUT);
        digitalWrite(scrPins[i], LOW);
        setupMCPWMForSCR(i);
        gpio_set_intr_type((gpio_num_t)zcPins[i], GPIO_INTR_POSEDGE);
        gpio_isr_handler_add((gpio_num_t)zcPins[i], zcISR_Generic, (void*)i);
    }

    scrDelayUs = 8100;  // inicialización global

    xTaskCreate(controlTaskGeneric, "ctrlA", 4096, (void*)0, 3, NULL);
    xTaskCreate(controlTaskGeneric, "ctrlB", 4096, (void*)1, 3, NULL);
    xTaskCreate(controlTaskGeneric, "ctrlC", 4096, (void*)2, 3, NULL);

    Serial.println("✅ MCPWM configurado - sistema listo");
}

// ================== LOOP ==================
void loop() {
    resetWatchdog();
    updateWaveBasedControl();

    if (!systemStarted) {
        systemStarted = true;
        ioController.setRelay(0, true);
        Serial.println("✅ SISTEMA INICIADO - CONTROL POR MCPWM");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    resetWatchdog();
}
