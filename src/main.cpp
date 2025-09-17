#include <Arduino.h>
#include <driver/gpio.h>
#include <Adafruit_ADS1X15.h>
#include "Calibracion.hpp"
#include "Sensores.hpp"
#include "TTL.hpp"
#include "MCP23017_IO.hpp"  // ← AÑADIR ESTA LÍNEA


#define DEBOUNCE_TIME_US 1000

// Declarar la constante como extern para que esté disponible en TTL.cpp
extern const int NUM_DEVICES = 3;      // ← AÑADIR ESTA LÍNEA
extern const int SEMI_PERIOD_US = 8333;  // ← AÑADIR ESTA LÍNEA

QueueHandle_t zcQueues[NUM_DEVICES] = {NULL, NULL, NULL};
esp_timer_handle_t fireTimers[NUM_DEVICES];

Adafruit_ADS1115 adsLow;   // 0x48
Adafruit_ADS1115 adsHigh;  // 0x49

const int zcPins[NUM_DEVICES] = {38, 47, 11};
const int scrPins[NUM_DEVICES] = {48, 21, 12};

// Variables para control y medición
volatile uint32_t lastZCTime[NUM_DEVICES] = {0};
volatile bool scrActive[NUM_DEVICES] = {false};
volatile uint32_t pulseStartTime[NUM_DEVICES] = {0};
volatile uint32_t pulseDuration[NUM_DEVICES] = {0};
volatile uint32_t pulseCount[NUM_DEVICES] = {0};
volatile uint32_t zcCount[NUM_DEVICES] = {0};

// Variable compartida para el porcentaje del potenciómetro (0-100%)
volatile int potPercentage = 50;
volatile float potVoltage = 0;
volatile float CurrentSensorVoltage = 0;

// Variables para MCP23017 ← AÑADIR ESTAS VARIABLES
bool ioControlEnabled = false;
unsigned long lastInputCheckTime = 0;
const unsigned long INPUT_CHECK_INTERVAL = 100; // 100ms entre lecturas

// FUNCIÓN FORZADA PARA APAGAR SCR
void IRAM_ATTR forceTurnOffSCR(uint8_t dev) {
    digitalWrite(scrPins[dev], LOW);
    scrActive[dev] = false;
    if (pulseStartTime[dev] > 0) {
        pulseDuration[dev] = micros() - pulseStartTime[dev];
        pulseStartTime[dev] = 0;
    }
}

// ✅ ISRs OPTIMIZADAS
void IRAM_ATTR zcISR_FaseA(void* arg) {
    uint32_t now = micros();
    if (now - lastZCTime[0] > DEBOUNCE_TIME_US) {
        lastZCTime[0] = now;
        zcCount[0]++;
        digitalWrite(scrPins[0], LOW);
        scrActive[0] = false;
        static uint8_t dev = 0;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(zcQueues[0], &dev, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
    }
}

void IRAM_ATTR zcISR_FaseB(void* arg) {
    uint32_t now = micros();
    if (now - lastZCTime[1] > DEBOUNCE_TIME_US) {
        lastZCTime[1] = now;
        zcCount[1]++;
        digitalWrite(scrPins[1], LOW);
        scrActive[1] = false;
        static uint8_t dev = 1;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(zcQueues[1], &dev, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
    }
}

void IRAM_ATTR zcISR_FaseC(void* arg) {
    uint32_t now = micros();
    if (now - lastZCTime[2] > DEBOUNCE_TIME_US) {
        lastZCTime[2] = now;
        zcCount[2]++;
        digitalWrite(scrPins[2], LOW);
        scrActive[2] = false;
        static uint8_t dev = 2;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(zcQueues[2], &dev, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
    }
}

// Callback de timer - ENCENDER SCR
void IRAM_ATTR timerCallback(void* arg) {
    uint8_t dev = (uint8_t)(intptr_t)arg;
    digitalWrite(scrPins[dev], HIGH);
    scrActive[dev] = true;
    pulseStartTime[dev] = micros();
    pulseCount[dev]++;
}

// Tarea dedicada para leer el potenciómetro
void VoltageReadTask(void* param) {
    while (true) {
        readPotenciometerSafe();
        readCurrentSafe();
        vTaskDelay(POT_READ_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

// ✅ Tareas de CONTROL
void controlTaskFaseA(void* param) {
    uint8_t dev;
    uint32_t delay_us = 0;
    uint32_t lastZC = 0;
    
    while (true) {
        if (xQueueReceive(zcQueues[0], &dev, portMAX_DELAY) == pdTRUE) {
            delay_us = map(potPercentage, 0, 100, SEMI_PERIOD_US, 100);
            if (delay_us < SEMI_PERIOD_US) {
                esp_timer_start_once(fireTimers[0], delay_us);
            }
            if (zcCount[0] > lastZC) {
                lastZC = zcCount[0];
                float firingAngle = (delay_us * 180.0) / SEMI_PERIOD_US;
                Serial.printf("[FASE A] ZC#%lu | Pot: %d%% | Angle: %.1f°\n", 
                             zcCount[0], potPercentage, firingAngle);
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void controlTaskFaseB(void* param) {
    uint8_t dev;
    uint32_t delay_us = 0;
    uint32_t lastZC = 0;
    
    while (true) {
        if (xQueueReceive(zcQueues[1], &dev, portMAX_DELAY) == pdTRUE) {
            delay_us = map(potPercentage, 0, 100, SEMI_PERIOD_US, 100);
            if (delay_us < SEMI_PERIOD_US) {
                esp_timer_start_once(fireTimers[1], delay_us);
            }
            if (zcCount[1] > lastZC) {
                lastZC = zcCount[1];
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void controlTaskFaseC(void* param) {
    uint8_t dev;
    uint32_t delay_us = 0;
    uint32_t lastZC = 0;
    
    while (true) {
        if (xQueueReceive(zcQueues[2], &dev, portMAX_DELAY) == pdTRUE) {
            delay_us = map(potPercentage, 0, 100, SEMI_PERIOD_US, 100);
            if (delay_us < SEMI_PERIOD_US) {
                esp_timer_start_once(fireTimers[2], delay_us);
            }
            if (zcCount[2] > lastZC) {
                lastZC = zcCount[2];
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// Función para verificar entradas digitales ← AÑADIR ESTA FUNCIÓN
void checkDigitalInputs() {
    if (!ioControlEnabled) return;
    
    unsigned long currentTime = millis();
    if (currentTime - lastInputCheckTime >= INPUT_CHECK_INTERVAL) {
        lastInputCheckTime = currentTime;
        
        // Leer y procesar entradas si está en modo monitor
        if (ioController.isMonitoring()) {
            uint8_t currentInputs = ioController.readGPIOB();  // ← CAMBIAR readAllInputs() por readGPIOA()
            
            // Detectar cambios
            static uint8_t lastInputState = 0;
            if (currentInputs != lastInputState) {
                Serial.println(ioController.getInputsStatus());
                lastInputState = currentInputs;
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Sistema iniciado - Disparo en CADA zero crossing");
    #if defined(ESP32)
    EEPROM.begin(512);
    #endif
    
    // Cargar calibración al inicio
    loadCalibration();
    
    // Inicializar I2C
    Wire.begin(5, 4);
    Wire.setTimeout(100);
    
    // Inicializar ADS1115
    if (!adsLow.begin(0x48, &Wire)) {
        Serial.println("Error ADS1115 0x48!");
        while (1);
    }
    if (!adsHigh.begin(0x49, &Wire)) {
        Serial.println("Error ADS1115 0x49!");
        while (1);
    }
    adsLow.setGain(GAIN_ONE);  // ±0.256 V
    adsHigh.setGain(GAIN_ONE);     // ±4.096 V
    adsHigh.setDataRate(RATE_ADS1115_8SPS);
    adsLow.setDataRate(RATE_ADS1115_8SPS);
    
    // Configurar pines
    for (int i = 0; i < NUM_DEVICES; i++) {
        pinMode(zcPins[i], INPUT_PULLDOWN);
        pinMode(scrPins[i], OUTPUT);
        digitalWrite(scrPins[i], LOW);
    }

    // ✅ Colas más grandes para no perder ZCs
    for (int i = 0; i < NUM_DEVICES; i++) {
        zcQueues[i] = xQueueCreate(10, sizeof(uint8_t));
    }

    // Crear timers de disparo
    for (int i = 0; i < NUM_DEVICES; i++) {
        esp_timer_create_args_t fireArgs = {
            .callback = &timerCallback,
            .arg = (void*)(intptr_t)i,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "fireTimer",
            .skip_unhandled_events = false
        };
        esp_timer_create(&fireArgs, &fireTimers[i]);
    }

    // ✅ Configurar ISRs con máxima prioridad
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    gpio_set_intr_type((gpio_num_t)zcPins[0], GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)zcPins[0], zcISR_FaseA, NULL);
    gpio_set_intr_type((gpio_num_t)zcPins[1], GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)zcPins[1], zcISR_FaseB, NULL);
    gpio_set_intr_type((gpio_num_t)zcPins[2], GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)zcPins[2], zcISR_FaseC, NULL);

    // ✅ Tareas con prioridades altas
    xTaskCreate(controlTaskFaseA, "ctrlA", 4096, NULL, 5, NULL);
    xTaskCreate(controlTaskFaseB, "ctrlB", 4096, NULL, 5, NULL);
    xTaskCreate(controlTaskFaseC, "ctrlC", 4096, NULL, 5, NULL);
    xTaskCreate(VoltageReadTask, "VoltageReadTask", 4096, NULL, 1, NULL);

    // Inicializar MCP23017 - FORMA CORRECTA
    if (ioController.begin(5, 4)) {  // ← Usar los pines correctos SDA=5, SCL=4
        ioControlEnabled = true;
        Serial.println("Control MCP23017 habilitado");
        Serial.println("Puerto A: Entradas digitales (0-7) con pull-up");
        Serial.println("Puerto B: Salidas para relés (8-15)");
        
        // Opcional: secuencia de prueba al inicio
        // ioController.testSequence(200);
    } else {
        Serial.println("Control MCP23017 deshabilitado");
    }
}

void loop() {
    processSerialCommands();
    
    // Verificación de seguridad
    for (int i = 0; i < NUM_DEVICES; i++) {
        if (scrActive[i] && micros() - pulseStartTime[i] > SEMI_PERIOD_US * 1.2) {
            Serial.printf("[SAFETY] Fase %c apagada\n", 'A' + i);
            forceTurnOffSCR(i);
        }
    }
    
    // Verificar entradas digitales ← AÑADIR ESTA LÍNEA
    checkDigitalInputs();
    
    // Estadísticas normales (solo fuera del modo calibración)
    static uint32_t lastStatsTime = 0;
    if (millis() - lastStatsTime > 500 && !calibrationMode) {
        lastStatsTime = millis();
        Serial.printf("Pot: %d%% | Voltage: %.2f mV |  RealVoltage: %.2f mV | RealCorriente: %.1f A ||  Current: %.2f mV\n", 
                     potPercentage, potVoltage ,potVoltage/50.5-5.12, (potVoltage/50.5-5.12)*100, CurrentSensorVoltage);
    }
    
    vTaskDelay(50 / portTICK_PERIOD_MS);
}