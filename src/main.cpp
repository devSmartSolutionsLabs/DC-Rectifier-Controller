#include <Arduino.h>
#include <driver/gpio.h>
#include <Adafruit_ADS1X15.h>
#include "Calibracion.hpp"
#include "Sensores.hpp"
#include "TTL.hpp"
#include "MCP23017_IO.hpp"
#include "GlobalVars.hpp"
#include "mutexDebug.hpp"

// ================== Variables Globales ==================
volatile bool interruptsEnabled = true;
volatile bool pwmGenerationEnabled = true;

QueueHandle_t zcQueues[NUM_DEVICES] = {NULL, NULL, NULL};
esp_timer_handle_t fireTimers[NUM_DEVICES];

// Instancias ADS
Adafruit_ADS1115 adsLow;   // 0x48
Adafruit_ADS1115 adsHigh;  // 0x49

// Pines
const int zcPins[NUM_DEVICES]  = {38, 47, 14};
const int scrPins[NUM_DEVICES] = {48, 21, 13};

// Control y medición
volatile uint32_t lastZCTime[NUM_DEVICES] = {0};
volatile bool scrActive[NUM_DEVICES] = {false};
volatile uint32_t pulseStartTime[NUM_DEVICES] = {0};
volatile uint32_t pulseDuration[NUM_DEVICES] = {0};
volatile uint32_t pulseCount[NUM_DEVICES] = {0};
volatile uint32_t zcCount[NUM_DEVICES] = {0};

// Botón START
int startButtonPin = -1;
bool startButtonDetected = false;
bool lastStartButtonState = false;
uint32_t startButtonPressTime = 0;
bool startCounting = false;
// ================== WRAPPERS PARA I2C MANAGER ==================
float readADSSafe(uint8_t address, uint8_t channel) {
    Adafruit_ADS1115* ads = nullptr;
    if (address == 0x48) ads = &adsLow;
    else if (address == 0x49) ads = &adsHigh;
    else return 0.0;

    float result = 0.0;
    if (i2cMutex != nullptr) {
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            result = readAveraged(*ads, channel, false, 0.1875, 10);
            xSemaphoreGive(i2cMutex);
        } else {
            Serial.println("[WARN] No se pudo tomar mutex I2C para ADS");
        }
    }
    return result;
}


uint8_t readMCP23017Safe(uint8_t address, uint8_t reg) {
    if (reg == 0x13) {  // GPIOB
        return ioController.readGPIOB();
    }
    // Si necesitas otro registro, agrega más condiciones
    return 0xFF;  // default si registro no soportado
}
// ================== FUNCIONES ==================

void IRAM_ATTR forceTurnOffSCR(uint8_t dev) {
    gpio_set_level((gpio_num_t)scrPins[dev], 0);
    scrActive[dev] = false;
    if (pulseStartTime[dev] > 0) {
        pulseDuration[dev] = micros() - pulseStartTime[dev];
        pulseStartTime[dev] = 0;
    }
}

void updateSCREnabledStates(int percentage) {
    static int lastPercentage = -1;
    if (percentage == lastPercentage) return;
    lastPercentage = percentage;

    bool newEnabledStates[NUM_DEVICES];
    int newActiveCount = 0;

    if (percentage <= SCR_1_PHASE_THRESHOLD) {
        newEnabledStates[0] = true;
        newEnabledStates[1] = false;
        newEnabledStates[2] = false;
        newActiveCount = 1;
        if (percentage > 0) Serial.printf("[SCR] Modo 1-FASE (A) - %d%%\n", percentage);
    } else if (percentage <= SCR_2_PHASE_THRESHOLD) {
        newEnabledStates[0] = true;
        newEnabledStates[1] = true;
        newEnabledStates[2] = false;
        newActiveCount = 2;
        Serial.printf("[SCR] Modo 2-FASES (A+B) - %d%%\n", percentage);
    } else {
        newEnabledStates[0] = true;
        newEnabledStates[1] = true;
        newEnabledStates[2] = true;
        newActiveCount = 3;
        Serial.printf("[SCR] Modo 3-FASES (A+B+C) - %d%%\n", percentage);
    }

    for (int i = 0; i < NUM_DEVICES; i++) {
        scrEnabled[i] = newEnabledStates[i];
        if (!scrEnabled[i] && scrActive[i]) {
            forceTurnOffSCR(i);
        }
    }
    activeSCRsCount = newActiveCount;
}

// ISR zero crossing (fase A/B/C)
void IRAM_ATTR zcISR_FaseA(void* arg) {
    if (!systemStarted || !scrEnabled[0]) return;
    uint32_t now = micros();
    if (now - lastZCTime[0] > DEBOUNCE_TIME_US) {
        lastZCTime[0] = now;
        zcCount[0]++;
        gpio_set_level((gpio_num_t)scrPins[0], 0);
        scrActive[0] = false;
        uint32_t currentPot = potPercentage;
        if (currentPot >= 95) {
            gpio_set_level((gpio_num_t)scrPins[0], 1);
            scrActive[0] = true;
            pulseStartTime[0] = micros();
            pulseCount[0]++;
        } else {
            uint8_t dev = 0;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(zcQueues[0], &dev, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
        }
    }
}
void IRAM_ATTR zcISR_FaseB(void* arg) {
    if (!systemStarted || !scrEnabled[1]) return;
    uint32_t now = micros();
    if (now - lastZCTime[1] > DEBOUNCE_TIME_US) {
        lastZCTime[1] = now;
        zcCount[1]++;
        gpio_set_level((gpio_num_t)scrPins[1], 0);
        scrActive[1] = false;
        uint32_t currentPot = potPercentage;
        if (currentPot >= 95) {
            gpio_set_level((gpio_num_t)scrPins[1], 1);
            scrActive[1] = true;
            pulseStartTime[1] = micros();
            pulseCount[1]++;
        } else {
            uint8_t dev = 1;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(zcQueues[1], &dev, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
        }
    }
}
void IRAM_ATTR zcISR_FaseC(void* arg) {
    if (!systemStarted || !scrEnabled[2]) return;
    uint32_t now = micros();
    if (now - lastZCTime[2] > DEBOUNCE_TIME_US) {
        lastZCTime[2] = now;
        zcCount[2]++;
        gpio_set_level((gpio_num_t)scrPins[2], 0);
        scrActive[2] = false;
        uint32_t currentPot = potPercentage;
        if (currentPot >= 95) {
            gpio_set_level((gpio_num_t)scrPins[2], 1);
            scrActive[2] = true;
            pulseStartTime[2] = micros();
            pulseCount[2]++;
        } else {
            uint8_t dev = 2;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(zcQueues[2], &dev, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
        }
    }
}

// Timer callback
void IRAM_ATTR timerCallback(void* arg) {
    if (!systemStarted) return;
    uint8_t dev = (uint8_t)(intptr_t)arg;
    gpio_set_level((gpio_num_t)scrPins[dev], 1);
    scrActive[dev] = true;
    pulseStartTime[dev] = micros();
    pulseCount[dev]++;
}

// ========== I2C MANAGER ==========
bool requestMCP23017Read(uint8_t reg, uint8_t* result) {
    if (i2cQueue == NULL) return false;
    I2CRequest req;
    req.device = DEV_MCP23017;
    req.isWrite = false;
    req.address = 0x27;
    req.reg = reg;
    req.resultB = result;
    return (xQueueSend(i2cQueue, &req, pdMS_TO_TICKS(50)) == pdTRUE);
}
void i2cManagerTask(void *pvParameters) {
    I2CRequest req;
    for (;;) {
        if (xQueueReceive(i2cQueue, &req, portMAX_DELAY) == pdTRUE) {
            switch (req.device) {
                case DEV_ADS1115:
                    if (!req.isWrite && req.resultF) {
                        *req.resultF = readADSSafe(req.address, req.reg);
                    }
                    break;
                case DEV_MCP23017:
                    if (!req.isWrite && req.resultB) {
                        *req.resultB = readMCP23017Safe(req.address, req.reg);
                    }
                    break;
            }
        }
    }
}

// ========== LECTURA DIGITAL + BOTÓN START ==========
void processInputChanges(uint8_t inputs) {
    static uint32_t lastPrintTime = 0;  // Control de impresión

    if (!startButtonDetected) {
        for (int pin = 0; pin < 8; pin++) {
            bool pinState = (inputs & (1 << pin)) == 0; // true si presionado
            if (pinState) {
                startButtonPin = pin;
                startButtonDetected = true;
                Serial.printf("🎯 BOTÓN START detectado en pin: %d\n", pin);
                break;
            }
        }
    }

    if (startButtonDetected) {
        bool currentStartState = (inputs & (1 << startButtonPin)) != 0;

        // Imprimir solo cada 1 segundo
        if (millis() - lastPrintTime >= 1000) {
            Serial.printf("   [START BTN] Estado actual: %s\n",
                          currentStartState ? "🚨 PRESIONADO" : "⬜ SUELTO");
            lastPrintTime = millis();
        }

        if (currentStartState && !lastStartButtonState) {
            startButtonPressTime = millis();
            startCounting = true;
            Serial.println("▶️ BOTÓN START PRESIONADO - Iniciando conteo...");
        }

        if (!currentStartState && lastStartButtonState) {
            startCounting = false;
            if (systemStarted) {
                systemStarted = false;
                ioController.setRelay(0, false);
                ioController.setRelay(4, false);
                for (int i = 0; i < NUM_DEVICES; i++) forceTurnOffSCR(i);
                Serial.println("⏹️ BOTÓN START LIBERADO - Sistema DETENIDO");
            } else {
                Serial.println("⏹️ BOTÓN START LIBERADO - Conteo cancelado");
            }
        }

        if (startCounting && !systemStarted) {
            uint32_t elapsed = millis() - startButtonPressTime;
            if (elapsed >= START_DELAY_MS) {
                systemStarted = true;
                startCounting = false;
                ioController.setRelay(0, true);
                ioController.setRelay(4, true);
                Serial.println("🚀 SISTEMA INICIADO - Delay completado");
            }
        }

        lastStartButtonState = currentStartState;
    }
}


// ================== DIGITAL INPUT TASK ==================
void digitalInputTask(void *pvParameters) {
    uint8_t inputs = 0;
    uint32_t lastDebugTime = 0;

    while (true) {
        if (ioController.isInitialized()) {
            inputs = ioController.readGPIOB();
            processInputChanges(inputs);
        }

        // Debug general cada 1 segundo
        if (millis() - lastDebugTime > 1000) {
            Serial.println("=== DEBUG MCP23017 ===");
            Serial.printf("Sistema iniciado: %s\n", systemStarted ? "SI" : "NO");
            //ioController.debugInputs(); // solo si quieres depuración visual
            lastDebugTime = millis();
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);  // <--- lectura cada 500ms
    }
}

// ================== CONTROL TASKS ==================
void controlTaskFaseA(void* param) {
    uint8_t dev;
    uint32_t delay_us;
    uint32_t currentPot;
    while (true) {
        if (xQueueReceive(zcQueues[0], &dev, portMAX_DELAY) == pdTRUE) {
            if (!systemStarted) { forceTurnOffSCR(0); continue; }
            currentPot = potPercentage;
            delay_us = map(currentPot, 0, 100, SEMI_PERIOD_US, 0);
            if (delay_us <= 10) {
                gpio_set_level((gpio_num_t)scrPins[0], 1);
                scrActive[0] = true;
                pulseStartTime[0] = micros();
                pulseCount[0]++;
            } else if (delay_us < SEMI_PERIOD_US) {
                esp_timer_start_once(fireTimers[0], delay_us);
            }
        }
    }
}
void controlTaskFaseB(void* param) {
    uint8_t dev;
    uint32_t delay_us;
    uint32_t currentPot;
    while (true) {
        if (xQueueReceive(zcQueues[1], &dev, portMAX_DELAY) == pdTRUE) {
            if (!systemStarted) { forceTurnOffSCR(1); continue; }
            currentPot = potPercentage;
            delay_us = map(currentPot, 0, 100, SEMI_PERIOD_US, 0);
            if (delay_us <= 10) {
                gpio_set_level((gpio_num_t)scrPins[1], 1);
                scrActive[1] = true;
                pulseStartTime[1] = micros();
                pulseCount[1]++;
            } else if (delay_us < SEMI_PERIOD_US) {
                esp_timer_start_once(fireTimers[1], delay_us);
            }
        }
    }
}
void controlTaskFaseC(void* param) {
    uint8_t dev;
    uint32_t delay_us;
    uint32_t currentPot;
    while (true) {
        if (xQueueReceive(zcQueues[2], &dev, portMAX_DELAY) == pdTRUE) {
            if (!systemStarted) { forceTurnOffSCR(2); continue; }
            currentPot = potPercentage;
            delay_us = map(currentPot, 0, 100, SEMI_PERIOD_US, 0);
            if (delay_us <= 10) {
                gpio_set_level((gpio_num_t)scrPins[2], 1);
                scrActive[2] = true;
                pulseStartTime[2] = micros();
                pulseCount[2]++;
            } else if (delay_us < SEMI_PERIOD_US) {
                esp_timer_start_once(fireTimers[2], delay_us);
            }
        }
    }
}

void adsReadTask(void* pvParameters) {
    while (true) {
        readPotenciometerSafe();
        readCurrentSafe();
        //readAllChannelsSafe(); // si quieres leer todos los canales juntos
        vTaskDelay(2000 / portTICK_PERIOD_MS); // lee cada 100 ms
    }
}
// ================== SETUP ==================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== INICIANDO SISTEMA ===");

    // --- Mutex I2C ---
    if (i2cMutex == NULL) {
        i2cMutex = xSemaphoreCreateMutex();
        Serial.print("✅ i2cMutex creado: ");
        Serial.println(i2cMutex != nullptr ? "SI" : "NO");
    }

    // --- Cola I2C ---
    i2cQueue = xQueueCreate(10, sizeof(I2CRequest));
    Serial.print("✅ i2cQueue creada: ");
    Serial.println(i2cQueue != nullptr ? "SI" : "NO");

    // --- Configuración de pines y zcQueues ---
    for (int i = 0; i < NUM_DEVICES; i++) {
        pinMode(zcPins[i], INPUT_PULLDOWN);
        pinMode(scrPins[i], OUTPUT);
        digitalWrite(scrPins[i], LOW);
        zcQueues[i] = xQueueCreate(10, sizeof(uint8_t));
        Serial.printf("✅ zcQueue[%d] creada: %s\n", i, zcQueues[i] != nullptr ? "SI" : "NO");
    }

    Wire.begin(5, 4);
    if (!adsLow.begin(0x48)) {
        Serial.println("❌ ERROR inicializando ADS1115 LOW");
    }
    if (!adsHigh.begin(0x49)) {
        Serial.println("❌ ERROR inicializando ADS1115 HIGH");
    }

    // --- ISR Zero Crossing ---
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    gpio_set_intr_type((gpio_num_t)zcPins[0], GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)zcPins[0], zcISR_FaseA, NULL);
    gpio_set_intr_type((gpio_num_t)zcPins[1], GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)zcPins[1], zcISR_FaseB, NULL);
    gpio_set_intr_type((gpio_num_t)zcPins[2], GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)zcPins[2], zcISR_FaseC, NULL);

    // --- Timers SCR ---
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

    // --- Tareas ---
    xTaskCreate(i2cManagerTask, "I2C Manager", 4096, NULL, 2, NULL);
    xTaskCreate(digitalInputTask, "inputTask", 4096, NULL, 1, NULL);
    xTaskCreate(controlTaskFaseA, "ctrlA", 4096, NULL, 3, NULL);
    xTaskCreate(controlTaskFaseB, "ctrlB", 4096, NULL, 3, NULL);
    xTaskCreate(controlTaskFaseC, "ctrlC", 4096, NULL, 3, NULL);
    xTaskCreate(adsReadTask, "ADS Read Task", 4096, NULL, 2, NULL);

    // --- Inicialización MCP23017 ---
    bool mcpInit = ioController.begin(5,4); // Ajusta parámetros si tu begin requiere SDA/SCL
    Serial.printf("✅ MCP23017 inicializado: %s\n", mcpInit ? "SI" : "NO");

    Serial.println("=== SETUP COMPLETADO ===");
}
// ================== LOOP ==================
void loop() {
    processSerialCommands();
    updateSCREnabledStates(potPercentage);
    readPotenciometerSafe();   // <--- agregar esto
    readCurrentSafe();         // <--- y esto si quieres leer corriente
    vTaskDelay(50 / portTICK_PERIOD_MS);
}
