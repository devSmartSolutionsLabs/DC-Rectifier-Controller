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

QueueHandle_t zcQueues[NUM_DEVICES] = {NULL, NULL, NULL};
esp_timer_handle_t fireTimers[NUM_DEVICES];

Sensores sensores;

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
    if (address == 0x48) ads = &sensores.getADSLow();
    else if (address == 0x49) ads = &sensores.getADSHigh();
    else return 0.0;

    float result = 0.0;
    if (i2cMutex != nullptr) {
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            result = sensores.readAveragedWrapper(*ads, channel, false, 0.1875, 10);
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
            Serial.printf("🔧 i2cManager: Procesando request - Device: %d, Reg: 0x%02X\n", 
                         req.device, req.reg);
            
            switch (req.device) {
                case DEV_ADS1115:
                    if (!req.isWrite && req.resultF) {
                        if (takeI2CMutex(100, "i2cManager-ADS")) {
                            *req.resultF = readADSSafe(req.address, req.reg);
                            giveI2CMutex("i2cManager-ADS");
                        }
                    }
                    break;
                case DEV_MCP23017:
                    if (!req.isWrite && req.resultB) {
                        if (takeI2CMutex(100, "i2cManager-MCP")) {
                            *req.resultB = readMCP23017Safe(req.address, req.reg);
                            giveI2CMutex("i2cManager-MCP");
                        }
                    }
                    break;
            }
            
            Serial.printf("🔧 i2cManager: Request completado\n");
        }
    }
}

void handleInputChange(uint8_t inputNumber, bool state) {
    Serial.printf("🎯 handleInputChange: Entrada %d -> %s\n", 
                 inputNumber + 1, 
                 state ? "ACTIVA" : "INACTIVA");
    
    // Ejemplo de acciones según la entrada
    switch(inputNumber) {
        case 0: // Entrada 1
            if (state) {
                Serial.println("✅ Entrada 1 activada - Acción específica");
                // ioController.setRelay(0, true); // Ejemplo: activar relé 0
            } else {
                Serial.println("❌ Entrada 1 desactivada");
                // ioController.setRelay(0, false);
            }
            break;
            
        case 1: // Entrada 2
            if (state) {
                Serial.println("✅ Entrada 2 activada - Alternar relé");
                // ioController.toggleRelay(1);
            }
            break;
            
        case 2: // Entrada 3
            // Tu lógica para entrada 3
            break;
            
        case 3: // Entrada 4
            // Tu lógica para entrada 4
            break;
            
        // ... añadir más casos según necesites
            
        default:
            Serial.printf("⚠️ Entrada %d no configurada\n", inputNumber + 1);
            break;
    }
}

// ========== LECTURA DIGITAL + BOTÓN START ==========
void processInputChanges(uint8_t currentStates) {
    static uint8_t lastStates = 0xFF;
    static uint32_t lastDebounceTime = 0;
    
    if (currentStates == lastStates) return;
    
    // Debounce simple
    if (millis() - lastDebounceTime < 50) return;
    
    lastDebounceTime = millis();
    
    // Detectar cambios
    for (int i = 0; i < 8; i++) {
        bool currentState = (currentStates & (1 << i)) == 0;
        bool lastState = (lastStates & (1 << i)) == 0;
        
        if (currentState != lastState) {
            Serial.printf("🔄 Entrada %d: %s -> %s\n", 
                         i + 1, 
                         lastState ? "ACTIVA" : "INACTIVA",
                         currentState ? "ACTIVA" : "INACTIVA");
            
            // Aquí tu lógica de acciones para cambios de entrada
            handleInputChange(i, currentState);
        }
    }
    
    lastStates = currentStates;
}



// ================== DIGITAL INPUT TASK CORREGIDA ==================
void digitalInputTask(void* parameter) {
    const TickType_t xFrequency = pdMS_TO_TICKS(50);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    uint32_t totalFails = 0;
    uint32_t lastSuccessTime = millis();
    uint32_t lastErrorLog = 0;

    while (true) {
        uint8_t inputStates = 0xFF;
        bool success = false;

        // ⚡ INTENTAR LECTURA DIRECTA CON MUTEX
        if (takeI2CMutex(25)) { // Timeout de 25ms
            uint32_t readStart = micros();
            
            // LECTURA DIRECTA ULTRA RÁPIDA
            Wire.beginTransmission(MCP23017_ADDRESS);
            Wire.write(0x13); // GPIOB
            if (Wire.endTransmission() == 0) {
                uint8_t bytes = Wire.requestFrom((uint16_t)MCP23017_ADDRESS, (uint8_t)1);
                if (bytes > 0 && Wire.available()) {
                    inputStates = Wire.read();
                    success = true;
                    totalFails = 0;
                    lastSuccessTime = millis();
                }
            }
            
            giveI2CMutex();
            
            if (success) {
                processInputChanges(inputStates);
            } else {
                totalFails++;
            }
        } else {
            totalFails++;
        }

        // 📊 LOGGING INTELIGENTE
        if (totalFails > 0) {
            if (millis() - lastErrorLog > 3000) { // Log cada 3 segundos máximo
                Serial.printf("⚠️ [Input] %lu fallos, último éxito hace %lums\n", 
                             totalFails, millis() - lastSuccessTime);
                lastErrorLog = millis();
                
                // 🎯 DEBUG DEL MUTEX CUANDO HAY FALLOS
                if (i2cMutex != NULL) {
                    UBaseType_t mutexCount = uxSemaphoreGetCount(i2cMutex);
                    TaskHandle_t mutexHolder = xSemaphoreGetMutexHolder(i2cMutex);
                    
                    if (mutexHolder != NULL) {
                        Serial.printf("🔍 Mutex bloqueado por: %s\n", pcTaskGetName(mutexHolder));
                    } else {
                        Serial.printf("🔍 Mutex count: %d, Holder: NINGUNO\n", mutexCount);
                    }
                }
            }
            
            // 🚨 ESTRATEGIA DE RECUPERACIÓN
            if (totalFails >= 20) {
                Serial.println("🚨 RESETEO DE EMERGENCIA - Reiniciando I2C");
                
                // FORZAR LIBERACIÓN DEL MUTEX
                while (xSemaphoreTake(i2cMutex, 0) == pdTRUE) {
                    xSemaphoreGive(i2cMutex);
                }
                
                // REINICIAR I2C
                Wire.end();
                delay(50);
                Wire.begin(5, 4);
                Wire.setClock(100000);
                
                totalFails = 0;
                vTaskDelay(pdMS_TO_TICKS(200)); // Pausa larga para recuperación
            }
            else if (totalFails >= 5) {
                vTaskDelay(pdMS_TO_TICKS(10)); // Pausa corta
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
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

// ================== ADS READ TASK ULTRA OPTIMIZADA ==================
void adsReadTask(void* parameter) {
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // ⚡ SOLO 1 VEZ POR SEGUNDO
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (true) {
        // ⚡ LECTURA MÍNIMA
        bool success = sensores.readAllSensors(nullptr, 0);
        
        if (!success) {
            Serial.println("❌ Fallo lectura ADS - Skipping ciclo");
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ================== SETUP ==================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== INICIANDO SISTEMA ===");

    // --- HABILITAR MCP23017 ---
    Serial.println("🔌 Habilitando MCP23017...");
    pinMode(15, OUTPUT);
    digitalWrite(15, HIGH); 
    delay(1000);
    Serial.println("✅ MCP23017 habilitado");

    // --- INICIALIZAR I2C PRIMERO ---
    Wire.begin(5, 4);
    Wire.setClock(100000);

    // --- Mutex I2C ---
    if (i2cMutex == NULL) {
        i2cMutex = xSemaphoreCreateMutex();
        Serial.print("✅ i2cMutex creado: ");
        Serial.println(i2cMutex != nullptr ? "SI" : "NO");
        
        // Dar el mutex inicialmente
        giveI2CMutex();
    }

    // --- Configuración de pines SCR ---
    for (int i = 0; i < NUM_DEVICES; i++) {
        pinMode(zcPins[i], INPUT_PULLDOWN);
        pinMode(scrPins[i], OUTPUT);
        digitalWrite(scrPins[i], LOW);
        zcQueues[i] = xQueueCreate(10, sizeof(uint8_t));
        Serial.printf("✅ zcQueue[%d] creada: %s\n", i, zcQueues[i] != nullptr ? "SI" : "NO");
    }

    // --- Inicializar Sensores ---
    Serial.println("🔍 Inicializando sensores...");
    if (!sensores.begin()) {
        Serial.println("❌ ERROR inicializando Sensores (ADS1115)");
    }

    // --- Inicializar MCP23017 ---
    Serial.println("🔍 Inicializando MCP23017...");
    bool mcpInit = ioController.begin(5,4);
    Serial.printf("✅ MCP23017 inicializado: %s\n", mcpInit ? "SI" : "NO");

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

    // --- Tareas PRIMERO (sin I2C Manager inicialmente) ---
    xTaskCreate(digitalInputTask, "inputTask", 4096, NULL, 1, NULL);
    xTaskCreate(controlTaskFaseA, "ctrlA", 4096, NULL, 3, NULL);
    xTaskCreate(controlTaskFaseB, "ctrlB", 4096, NULL, 3, NULL);
    xTaskCreate(controlTaskFaseC, "ctrlC", 4096, NULL, 3, NULL);

    // --- Cola I2C AL FINAL ---
    i2cQueue = xQueueCreate(5, sizeof(I2CRequest)); // Reducir tamaño
    Serial.print("✅ i2cQueue creada: ");
    Serial.println(i2cQueue != nullptr ? "SI" : "NO");

    // --- Tareas I2C DESPUÉS de crear la cola ---
    xTaskCreate(i2cManagerTask, "I2C Manager", 4096, NULL, 2, NULL);
    xTaskCreate(adsReadTask, "ADS Read Task", 4096, NULL, 3, NULL);

    Serial.println("=== SETUP COMPLETADO ===");
    
    // Verificar estado final
    UBaseType_t mutexCount = uxSemaphoreGetCount(i2cMutex);
    Serial.printf("🔍 Mutex disponibles al final: %d\n", mutexCount);
}
// ================== LOOP ==================
void loop() {
    processSerialCommands();
    updateSCREnabledStates(sensores.getPotPercentage());
    vTaskDelay(50 / portTICK_PERIOD_MS);
}
