#include <Arduino.h>
#include <driver/gpio.h>
#include <Adafruit_ADS1X15.h>
#include "Calibracion.hpp"
#include "Sensores.hpp"
#include "TTL.hpp"
#include "MCP23017_IO.hpp"
#include "GlobalVars.hpp"
#include "mutexDebug.hpp"
#include "esp_task_wdt.h"

// ================== Variables Globales ==================
QueueHandle_t zcQueues[NUM_DEVICES] = {NULL, NULL, NULL};
esp_timer_handle_t fireTimers[NUM_DEVICES];

Sensores sensores;

// Pines
const int zcPins[NUM_DEVICES]  = {38, 47, 14};
const int scrPins[NUM_DEVICES] = {48, 21, 13};
// ================== TIMERS PARA APAGADO RETARDADO ==================
esp_timer_handle_t turnOffTimers[NUM_DEVICES];

// ================== VARIABLES PARA CONTROL POR ONDAS COMPLETAS ==================
volatile uint32_t zcPairCount[NUM_DEVICES] = {0};        // Contador de PARES de ZC
volatile uint32_t wavesToSkip[NUM_DEVICES] = {0};        // Ondas completas a saltar
volatile bool expectingSecondZC[NUM_DEVICES] = {false};  // Esperando 2do ZC del par
const uint32_t SAFE_MAX_DELAY = 8100;      // ⚡ Cambiado a 8300
const uint32_t USEFUL_MIN_DELAY = 7500;    // ⚡ Cambiado a 7000
const uint32_t ABSOLUTE_MAX_DELAY = 8110;  // ⚡ Cambiado a 8300
const uint32_t ABSOLUTE_MIN_DELAY = 7500;  // ⚡ Cambiado a 7000

// Control y medición
volatile uint32_t lastZCTime[NUM_DEVICES] = {0};
volatile bool scrActive[NUM_DEVICES] = {false};
volatile uint32_t pulseStartTime[NUM_DEVICES] = {0};
volatile uint32_t pulseDuration[NUM_DEVICES] = {0};
volatile uint32_t pulseCount[NUM_DEVICES] = {0};
volatile uint32_t zcCount[NUM_DEVICES] = {0};

// Para capturar delay en momento del zero crossing
volatile uint32_t currentPhaseDelays[NUM_DEVICES] = {8200, 8200, 8200};

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
    bool newEnabledStates[NUM_DEVICES];
    for (int i = 0; i < NUM_DEVICES; i++) {
        scrEnabled[i] = true;
    }
}

// ================== TIMER CALLBACK - NUEVA FUNCIÓN ==================
void IRAM_ATTR fireTimerCallback(void* arg) {
    uint8_t dev = (uint8_t)(intptr_t)arg;
    
    // ⚡ ESTO SE EJECUTA AUTOMÁTICAMENTE DESPUÉS DEL DELAY PROGRAMADO
    // SIN BLOQUEAR NINGUNA TASK
    if (scrEnabled[dev] && systemStarted) {
        gpio_set_level((gpio_num_t)scrPins[dev], 1);
        scrActive[dev] = true;
        pulseStartTime[dev] = micros();
        pulseCount[dev]++;
        
        // Debug del primer disparo con timer
        static bool firstFire[3] = {true, true, true};
        if (firstFire[dev]) {
            Serial.printf("[TIMER%c] Primer disparo con timer HW\n", 'A' + dev);
            firstFire[dev] = false;
        }
    }
}


// ================== CALLBACK PARA APAGADO RETARDADO ==================
void IRAM_ATTR turnOffTimerCallback(void* arg) {
    uint8_t dev = (uint8_t)(intptr_t)arg;
    gpio_set_level((gpio_num_t)scrPins[dev], 0);
    scrActive[dev] = false;
    
    if (pulseStartTime[dev] > 0) {
        pulseDuration[dev] = micros() - pulseStartTime[dev];
        pulseStartTime[dev] = 0;
    }
}

// ================== ISR MODIFICADA - APAGADO RETARDADO ==================
void IRAM_ATTR zcISR_Generic(void* arg) {
    uint8_t dev = (uint8_t)(intptr_t)arg;
    
    if (!systemStarted || !scrEnabled[dev]) return;
    
    uint32_t now = micros();
    if (now - lastZCTime[dev] > 1800) {
        lastZCTime[dev] = now;
        zcCount[dev]++;
        
        // 1. ⚡ PROGRAMAR APAGADO EN 300µs (NO APAGAR INMEDIATAMENTE)
        if (scrActive[dev]) {
            esp_timer_start_once(turnOffTimers[dev], 300); // ⚡ Esperar 300µs antes de apagar
        }
        
        // 2. VERIFICACIÓN BÁSICA
        if (scrDelayUs >= ABSOLUTE_MAX_DELAY) {
            return; // APAGADO COMPLETO
        }
        
        // 3. LÓGICA DE CONTROL (IGUAL QUE ANTES)
        static uint32_t waveCount[NUM_DEVICES] = {0, 0, 0};
        uint32_t wavesToSkipCurrent = wavesToSkip[dev];
        
        bool shouldFire = false;
        
        if (wavesToSkipCurrent == 0) {
            shouldFire = true;
        } else if (wavesToSkipCurrent >= 50) {
            shouldFire = (waveCount[dev] % (wavesToSkipCurrent + 1) == 0);
        } else {
            uint32_t totalCycleLength = wavesToSkipCurrent + 2;
            uint32_t cyclePosition = waveCount[dev] % totalCycleLength;
            shouldFire = (cyclePosition < 2);
        }
        
        waveCount[dev]++;
        
        // 4. PROGRAMAR DISPARO (IGUAL QUE ANTES)
        if (shouldFire && scrDelayUs < ABSOLUTE_MAX_DELAY) {
            esp_timer_start_once(fireTimers[dev], scrDelayUs);
        }
        
        if (waveCount[dev] > 1000000) {
            waveCount[dev] = 0;
        }
    }
}

// ================== WATCHDOG CONFIGURATION ==================
void enableWatchdog() {
    esp_task_wdt_init(30, false); // ⚡ Volver a 30 segundos (ahora será estable)
    esp_task_wdt_add(xTaskGetCurrentTaskHandle());
}

void resetWatchdog() {
    esp_task_wdt_reset();
}

// ================== CONTROL TASK SIMPLIFICADA - SOLO MONITOREO ==================
void controlTaskGeneric(void* param) {
    uint8_t queueIndex = (uint8_t)(intptr_t)param;
    
    esp_task_wdt_add(NULL);
    
    Serial.printf("[MONITOR%c] Task de monitoreo iniciada\n", 'A' + queueIndex);
    
    while (true) {
        esp_task_wdt_reset();
        
        // ⚡ ESTA TASK AHORA SOLO MONITOREA - NO HACE ESPERAS ACTIVAS
        // El disparo lo hace el timer de hardware automáticamente
        
        // Debug del contador cada 10 segundos
        static uint32_t lastCountLog = 0;
        if (millis() - lastCountLog > 10000) {
            Serial.printf("[FASE%c] Pulsos totales: %lu\n", 'A' + queueIndex, pulseCount[queueIndex]);
            lastCountLog = millis();
        }
        
        // ⚡ PAUSA LARGA - LA CPU ESTÁ LIBRE!
        vTaskDelay(2000 / portTICK_PERIOD_MS); // 2 segundos
        esp_task_wdt_reset();
    }
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
    
    esp_task_wdt_add(NULL);
    
    for (;;) {
        esp_task_wdt_reset();
            
        if (xQueueReceive(i2cQueue, &req, pdMS_TO_TICKS(200)) == pdTRUE) {
            esp_task_wdt_reset();
            
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
            
            esp_task_wdt_reset();
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS);
        esp_task_wdt_reset();
    }
}

void handleInputChange(uint8_t inputNumber, bool state) {
    switch(inputNumber) {
        case 0: // START
            if (state) {
                Serial.println("🎯 START presionado, iniciando conteo 3s...");
                startRequested = true;
                startRequestTime = millis();
            } else {
                Serial.println("🛑 START liberado, apagando sistema");
                startRequested = false;
                systemStarted = false;
                for (int i = 0; i < NUM_DEVICES; i++) {
                    forceTurnOffSCR(i);
                }
                ioController.setRelay(0, false);
            }
            break;
        
        case 1: // Botón DIRECCIÓN
            if (state) {
                direction = false;
                ioController.setRelay(1, false);
                Serial.println("🔄 Dirección: INVERSA (Relé 2 OFF)");
            } else {
                direction = true;
                ioController.setRelay(1, true);
                Serial.println("🔄 Dirección: DIRECTA (Relé 2 ON)");
            }
            break;

        default:
            Serial.printf("Entrada %d cambio a %s\n", 
                          inputNumber+1, state ? "ACTIVA":"INACTIVA");
            break;
    }
}

// ================== VARIABLE GLOBAL PARA ESTADO DE INPUTS ==================
static uint8_t lastKnownInputStates = 0xFF;

void processInputChanges(uint8_t currentStates) {
    static uint8_t lastStates = 0xFF;
    static uint32_t lastDebounceTime = 0;
    
    if (currentStates == 0xFF) {
        currentStates = lastKnownInputStates;
    } else {
        lastKnownInputStates = currentStates;
    }
    
    if (currentStates == lastStates) return;
    if (millis() - lastDebounceTime < 100) return;
    lastDebounceTime = millis();
    
    for (int i = 0; i < 8; i++) {
        bool currentState = (currentStates & (1 << i)) == 0;
        bool lastState = (lastStates & (1 << i)) == 0;
        
        if (currentState != lastState) {
            Serial.printf("🔘 Entrada %d: %s -> %s\n", 
                         i, lastState ? "ACTIVO" : "INACTIVO", 
                         currentState ? "ACTIVO" : "INACTIVO");
            handleInputChange(i, currentState);
        }
    }
    lastStates = currentStates;
}

// ================== TASKS OPTIMIZADAS ==================
void adsReadTask(void* parameter) {
    const TickType_t xFrequency = pdMS_TO_TICKS(3000); // ⚡ Cada 3 segundos
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    esp_task_wdt_add(NULL);
    
    while (true) {
        esp_task_wdt_reset();
        
        sensores.readAllSensors(nullptr, 0);
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        esp_task_wdt_reset();
    }
}

void digitalInputTask(void* parameter) {
    const TickType_t xFrequency = pdMS_TO_TICKS(1500); // ⚡ Cada 1.5 segundos
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    esp_task_wdt_add(NULL);
    
    while (true) {
        esp_task_wdt_reset();
        
        uint8_t inputStates = ioController.readAllInputs();
        processInputChanges(inputStates);
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        esp_task_wdt_reset();
    }
}

// ================== CONTROL POTENCIÓMETRO ==================
uint32_t calculateWavesToSkip(int potPercentage) {
    // Tabla progresiva de skipeo de ondas
    return 0;
    if (potPercentage == 0) return 50;      // 0% → saltar 50 ondas (mínima corriente)
    else if (potPercentage <= 5) return 2; // 1-5% → saltar 20 ondas
    else if (potPercentage <= 10) return 2;// 6-10% → saltar 15 ondas
    else if (potPercentage <= 15) return 2;// 11-15% → saltar 12 ondas
    else if (potPercentage <= 20) return 2;// 16-20% → saltar 10 ondas
    else if (potPercentage <= 25) return 2; // 21-25% → saltar 8 ondas
    else if (potPercentage <= 30) return 2; // 26-30% → saltar 6 ondas
    else if (potPercentage <= 35) return 2; // 31-35% → saltar 5 ondas
    else if (potPercentage <= 40) return 2; // 36-40% → saltar 4 ondas
    else if (potPercentage <= 45) return 2; // 41-45% → saltar 3 ondas
    else if (potPercentage <= 50) return 2; // 46-50% → saltar 2 ondas
    else if (potPercentage <= 60) return 2; // 51-60% → saltar 1 onda
    else return 0;                          // 61-100% → no saltar ondas
}

void updateWaveBasedControl() {
    const float POT_MIN_V = 2.30;  // ⚡ Mínimo útil
    const float POT_MAX_V = 4.00;  // ⚡ Máximo útil
    const float alpha = 0.6;

    float rawV = sensores.getVoltage();
    
    // ⚡ MAPEO DIRECTO 2.00V - 4.00V → 0% - 100%
    float potNorm = 0.0;
    
    if (rawV < POT_MIN_V) {
        // Por debajo de 2.00V → 0% (mínima potencia)
        potNorm = 0.0;
    } else if (rawV > POT_MAX_V) {
        // Por encima de 4.00V → 100% (máxima potencia)
        potNorm = 1.0;
    } else {
        // Entre 2.00V - 4.00V → Mapeo lineal
        potNorm = (rawV - POT_MIN_V) / (POT_MAX_V - POT_MIN_V);
    }
    
    // Aplicar filtro solo al valor normalizado para suavizar
    filteredPotVoltage = alpha * potNorm + (1 - alpha) * filteredPotVoltage;
    float filteredNorm = filteredPotVoltage;
    
    int potPercentage = (int)(filteredNorm * 100);

    // ⚡ CALCULAR SKIP DE ONDAS BASADO EN POTENCIÓMETRO
    uint32_t skipCount = calculateWavesToSkip(potPercentage);
    
    for (int dev = 0; dev < NUM_DEVICES; dev++) {
        if (scrEnabled[dev]) {
            wavesToSkip[dev] = skipCount;
        }
    }

    // CÁLCULO DE DELAY
    scrDelayUs = SAFE_MAX_DELAY - (uint32_t)(filteredNorm * (SAFE_MAX_DELAY - USEFUL_MIN_DELAY));
    scrDelayUs = constrain(scrDelayUs, USEFUL_MIN_DELAY, SAFE_MAX_DELAY);

    // APAGADO COMPLETO solo si está muy por debajo del mínimo
    if (rawV < 1.0) {
        scrDelayUs = ABSOLUTE_MAX_DELAY;
        for (int dev = 0; dev < NUM_DEVICES; dev++) {
            wavesToSkip[dev] = 100;
        }
    }

    updateSCREnabledStates(potPercentage);

    // ⚡ LOG MEJORADO CON INFO DEL MAPEO
    static int lastLoggedPercentage = -1;
    static uint32_t lastSkipCount = 0;
    static float lastRawV = -1.0;
    
    if (potPercentage != lastLoggedPercentage || skipCount != lastSkipCount || abs(rawV - lastRawV) > 0.1) {
        if (rawV < POT_MIN_V) {
            Serial.printf("🔻 Raw: %.2fV → Pot: %d%% (FUERA DE RANGO - MÍNIMO)\n", rawV, potPercentage);
        } else if (rawV > POT_MAX_V) {
            Serial.printf("🔺 Raw: %.2fV → Pot: %d%% (FUERA DE RANGO - MÁXIMO)\n", rawV, potPercentage);
        } else {
            if (skipCount > 0) {
                Serial.printf("🎛️  Raw: %.2fV → Pot: %d%% → Delay: %luµs | 2 pulsos / %lu ondas\n", 
                             rawV, potPercentage, scrDelayUs, skipCount + 2);
            } else {
                Serial.printf("🎛️  Raw: %.2fV → Pot: %d%% → Delay: %luµs | CONTINUA\n", 
                             rawV, potPercentage, scrDelayUs);
            }
        }
        lastLoggedPercentage = potPercentage;
        lastSkipCount = skipCount;
        lastRawV = rawV;
    }
}

// ================== SETUP CON TIMERS HW ==================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== INICIANDO SISTEMA CON TIMERS HW ===");

    enableWatchdog();

    pinMode(15, OUTPUT);
    digitalWrite(15, HIGH); 
    delay(1000);

    Wire.begin(5, 4);
    Wire.setClock(100000);

    if (i2cMutex == NULL) {
        i2cMutex = xSemaphoreCreateMutex();
        giveI2CMutex();
    }

    for (int i = 0; i < NUM_DEVICES; i++) {
        pinMode(zcPins[i], INPUT_PULLDOWN);
        pinMode(scrPins[i], OUTPUT);
        digitalWrite(scrPins[i], LOW);
        zcQueues[i] = xQueueCreate(5, sizeof(uint8_t)); // ⚡ Cola más pequeña (ya no es crítica)
        currentPhaseDelays[i] = 8300;
        
        // Timer para disparo
        esp_timer_create_args_t fireTimerArgs = {
            .callback = &fireTimerCallback,
            .arg = (void*)(intptr_t)i,
            .dispatch_method = ESP_TIMER_ISR,
            .name = "SCR_Fire_Timer",
            .skip_unhandled_events = true
        };
        esp_timer_create(&fireTimerArgs, &fireTimers[i]);
        
        // ⚡ Timer para apagado retardado
        esp_timer_create_args_t turnOffTimerArgs = {
            .callback = &turnOffTimerCallback,
            .arg = (void*)(intptr_t)i,
            .dispatch_method = ESP_TIMER_ISR,
            .name = "SCR_TurnOff_Timer", 
            .skip_unhandled_events = true
        };
        esp_timer_create(&turnOffTimerArgs, &turnOffTimers[i]);
        
    }

    sensores.begin();
    ioController.begin(5,4);
    direction = true;
    ioController.setRelay(1, true);

    Serial.println("⚙️ Estado inicial: Dirección DIRECTA (Relé 2 ON)");

    // Configurar ISRs
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    
    gpio_set_intr_type((gpio_num_t)zcPins[0], GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)zcPins[0], zcISR_Generic, (void*)0);
    
    gpio_set_intr_type((gpio_num_t)zcPins[1], GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)zcPins[1], zcISR_Generic, (void*)1);
    
    gpio_set_intr_type((gpio_num_t)zcPins[2], GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)zcPins[2], zcISR_Generic, (void*)2);

    // CREAR TASKS
    xTaskCreate(digitalInputTask, "inputTask", 4096, NULL, 1, NULL);
    xTaskCreate(controlTaskGeneric, "ctrlA", 4096, (void*)0, 3, NULL); // ⚡ Prioridad más baja
    xTaskCreate(controlTaskGeneric, "ctrlB", 4096, (void*)1, 3, NULL);
    xTaskCreate(controlTaskGeneric, "ctrlC", 4096, (void*)2, 3, NULL);

    i2cQueue = xQueueCreate(5, sizeof(I2CRequest));
    xTaskCreate(i2cManagerTask, "I2C Manager", 4096, NULL, 2, NULL);
    xTaskCreate(adsReadTask, "ADS Read Task", 4096, NULL, 2, NULL); // ⚡ Prioridad más baja

    Serial.println("=== SETUP COMPLETADO - TIMERS HW ACTIVOS ===");
}

// ================== LOOP OPTIMIZADO ==================
void loop() {
    resetWatchdog();
    
    processSerialCommands();
    updateWaveBasedControl();

    // START
    if (startRequested && !systemStarted) {
        if (millis() - startRequestTime >= 3000) {
            systemStarted = true;
            startRequested = false;
            ioController.setRelay(0, true);
            Serial.println("✅✅✅ SISTEMA INICIADO - TIMERS HW ACTIVOS ✅✅✅");
        }
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS); // ⚡ 1 SEGUNDO - CPU LIBRE
    resetWatchdog();
}