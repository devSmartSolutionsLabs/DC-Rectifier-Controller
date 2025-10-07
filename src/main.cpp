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

// ================== VARIABLES PARA CONTROL POR ONDAS COMPLETAS ==================
volatile uint32_t zcPairCount[NUM_DEVICES] = {0};        // Contador de PARES de ZC
volatile uint32_t wavesToSkip[NUM_DEVICES] = {0};        // Ondas completas a saltar
volatile bool expectingSecondZC[NUM_DEVICES] = {false};  // Esperando 2do ZC del par
const uint32_t SAFE_MAX_DELAY = 8000;      // Límite seguro superior
const uint32_t USEFUL_MIN_DELAY = 7400;    // ← NUEVO: Límite inferior útil
const uint32_t ABSOLUTE_MAX_DELAY = 8100;  // Para apagado completo
const uint32_t ABSOLUTE_MIN_DELAY = 4000;  // Mínimo absoluto (pero no lo usaremos en el rango ú

// Control y medición
volatile uint32_t lastZCTime[NUM_DEVICES] = {0};
volatile bool scrActive[NUM_DEVICES] = {false};
volatile uint32_t pulseStartTime[NUM_DEVICES] = {0};
volatile uint32_t pulseDuration[NUM_DEVICES] = {0};
volatile uint32_t pulseCount[NUM_DEVICES] = {0};
volatile uint32_t zcCount[NUM_DEVICES] = {0};

// Para capturar delay en momento del zero crossing
volatile uint32_t currentPhaseDelays[NUM_DEVICES] = {8300, 8300, 8300};

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

// Estructura para pasar datos de ISR a task
typedef struct {
    uint8_t dev;
    uint32_t delay_us;
} ZCEvent_t;

void IRAM_ATTR zcISR_Generic(void* arg) {
    uint8_t dev = (uint8_t)(intptr_t)arg;
    
    if (!systemStarted || !scrEnabled[dev]) return;
    
    uint32_t now = micros();
    if (now - lastZCTime[dev] > 2) {
        lastZCTime[dev] = now;
        zcCount[dev]++;
        
        // Apagar SCR
        gpio_set_level((gpio_num_t)scrPins[dev], 0);
        scrActive[dev] = false;
        
        // VERIFICACIÓN DE SEGURIDAD MEJORADA
        if (scrDelayUs >= ABSOLUTE_MAX_DELAY) {
            return; // APAGADO COMPLETO - no disparar
        }
        
        // Asegurar que el delay está en zona segura
        uint32_t safeDelay = scrDelayUs;
        if (safeDelay > SAFE_MAX_DELAY) {
            safeDelay = SAFE_MAX_DELAY;
        }
        
        // Contador de semiondas
        static uint32_t semiWaveCount[NUM_DEVICES] = {0};
        semiWaveCount[dev]++;
        
        // Lógica de control por ondas
        uint32_t completeWavesToSkip = wavesToSkip[dev];
        uint32_t totalCycleLength = (completeWavesToSkip + 1) * 2;
        uint32_t positionInCycle = semiWaveCount[dev] % totalCycleLength;
        
        // Conducir solo en las primeras 2 semiondas del ciclo
        if (positionInCycle < 2 && safeDelay < SAFE_MAX_DELAY) {
            if (safeDelay <= 100) {
                // Disparo inmediato
                gpio_set_level((gpio_num_t)scrPins[dev], 1);
                scrActive[dev] = true;
                pulseStartTime[dev] = micros();
                pulseCount[dev]++;
            } else {
                // Disparo con delay seguro
                uint32_t targetTime = now + safeDelay;
                while (micros() < targetTime) {
                    asm volatile ("nop");
                }
                gpio_set_level((gpio_num_t)scrPins[dev], 1);
                scrActive[dev] = true;
                pulseStartTime[dev] = micros();
                pulseCount[dev]++;
            }
        }
        
        // Reset preventivo
        if (semiWaveCount[dev] > 1000000) {
            semiWaveCount[dev] = 0;
        }
    }
}

// Timer callback
void IRAM_ATTR timerCallback(void* arg) {
    if (!systemStarted) return;
    uint8_t dev = (uint8_t)(intptr_t)arg;
    
    if (scrEnabled[dev]) {
        gpio_set_level((gpio_num_t)scrPins[dev], 1);
        scrActive[dev] = true;
        pulseStartTime[dev] = micros();
        pulseCount[dev]++;
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
    for (;;) {
        if (xQueueReceive(i2cQueue, &req, portMAX_DELAY) == pdTRUE) {
            switch (req.device) {
                case DEV_ADS1115:
                    if (!req.isWrite && req.resultF) {
                        if (takeI2CMutex(300, "i2cManager-ADS")) {
                            *req.resultF = readADSSafe(req.address, req.reg);
                            giveI2CMutex("i2cManager-ADS");
                        }
                    }
                    break;
                case DEV_MCP23017:
                    if (!req.isWrite && req.resultB) {
                        if (takeI2CMutex(300, "i2cManager-MCP")) {
                            *req.resultB = readMCP23017Safe(req.address, req.reg);
                            giveI2CMutex("i2cManager-MCP");
                        }
                    }
                    break;
            }
        }
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
                // Apagar todos los SCRs
                for (int i = 0; i < NUM_DEVICES; i++) {
                    forceTurnOffSCR(i);
                }
                ioController.setRelay(0, false);
            }
            break;
        
        case 1: // Botón DIRECCIÓN
            if (state) {
                direction = false;  // Presionado = INVERSA
                ioController.setRelay(1, false);   // Relé 2 OFF
                Serial.println("🔄 Dirección: INVERSA (Relé 2 OFF)");
            } else {
                direction = true;  // Suelto = DIRECTA
                ioController.setRelay(1, true);    // Relé 2 ON
                Serial.println("🔄 Dirección: DIRECTA (Relé 2 ON)");
            }
            break;

        default:
            Serial.printf("Entrada %d cambio a %s\n", 
                          inputNumber+1, state ? "ACTIVA":"INACTIVA");
            break;
    }
}

// ========== LECTURA DIGITAL ==========
void processInputChanges(uint8_t currentStates) {
    static uint8_t lastStates = 0xFF;
    static uint32_t lastDebounceTime = 0;
    
    if (currentStates == lastStates) return;
    if (millis() - lastDebounceTime < 50) return;
    lastDebounceTime = millis();
    
    for (int i = 0; i < 8; i++) {
        bool currentState = (currentStates & (1 << i)) == 0;
        bool lastState = (lastStates & (1 << i)) == 0;
        if (currentState != lastState) {
            handleInputChange(i, currentState);
        }
    }
    lastStates = currentStates;
}

// ================== DIGITAL INPUT TASK ==================
void digitalInputTask(void* parameter) {
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        uint8_t inputStates = ioController.readAllInputs();
        processInputChanges(inputStates);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ================== CONTROL TASKS CORREGIDAS ==================
void controlTaskFaseA(void* param) {
    uint8_t dev;
    while (true) {
        if (xQueueReceive(zcQueues[0], &dev, portMAX_DELAY) == pdTRUE) {
            if (!systemStarted) { 
                forceTurnOffSCR(dev); 
                continue; 
            }
            
            // Usar EXACTAMENTE el mismo scrDelayUs
            uint32_t currentDelay = scrDelayUs;
            
            if (currentDelay >= 8300) {  
                continue;
            }
            else if (currentDelay < SEMI_PERIOD_US && currentDelay > 100) {
                esp_timer_stop(fireTimers[dev]);
                esp_timer_start_once(fireTimers[dev], currentDelay);
            }
        }
    }
}

void controlTaskFaseB(void* param) {
    uint8_t dev;
    while (true) {
        if (xQueueReceive(zcQueues[1], &dev, portMAX_DELAY) == pdTRUE) {
            if (!systemStarted) { 
                forceTurnOffSCR(dev); 
                continue; 
            }
            
            // Usar EXACTAMENTE el mismo scrDelayUs
            uint32_t currentDelay = scrDelayUs;
            
            if (currentDelay >= 8300) {  
                continue;
            }
            else if (currentDelay < SEMI_PERIOD_US && currentDelay > 100) {
                esp_timer_stop(fireTimers[dev]);
                esp_timer_start_once(fireTimers[dev], currentDelay);
            }
        }
    }
}

void controlTaskFaseC(void* param) {
    uint8_t dev;
    while (true) {
        if (xQueueReceive(zcQueues[2], &dev, portMAX_DELAY) == pdTRUE) {
            if (!systemStarted) { 
                forceTurnOffSCR(dev); 
                continue; 
            }
            
            // Usar EXACTAMENTE el mismo scrDelayUs
            uint32_t currentDelay = scrDelayUs;
            
            if (currentDelay >= 8300) {  
                continue;
            }
            else if (currentDelay < SEMI_PERIOD_US && currentDelay > 100) {
                esp_timer_stop(fireTimers[dev]);
                esp_timer_start_once(fireTimers[dev], currentDelay);
            }
        }
    }
}

// ================== ADS READ TASK ==================
void adsReadTask(void* parameter) {
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        bool success = sensores.readAllSensors(nullptr, 0);
        if (!success) {
            if (verboseLog) {
                Serial.println("❌ Fallo lectura ADS - Skipping ciclo");
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


// ================== CÁLCULO PROGRESIVO DE ONDAS SKIPPEADAS ==================
uint32_t calculateWavesToSkip(int potPercentage) {
    // Tabla progresiva de skipeo de ondas
    if (potPercentage == 0) return 50;      // 0% → saltar 50 ondas (mínima corriente)
    else if (potPercentage <= 5) return 20; // 1-5% → saltar 20 ondas
    else if (potPercentage <= 10) return 19;// 6-10% → saltar 15 ondas
    else if (potPercentage <= 15) return 18;// 11-15% → saltar 12 ondas
    else if (potPercentage <= 20) return 17;// 16-20% → saltar 10 ondas
    else if (potPercentage <= 25) return 16; // 21-25% → saltar 8 ondas
    else if (potPercentage <= 30) return 15; // 26-30% → saltar 6 ondas
    else if (potPercentage <= 35) return 14; // 31-35% → saltar 5 ondas
    else if (potPercentage <= 40) return 13; // 36-40% → saltar 4 ondas
    else if (potPercentage <= 45) return 12; // 41-45% → saltar 3 ondas
    else if (potPercentage <= 50) return 11; // 46-50% → saltar 2 ondas
    else if (potPercentage <= 60) return 10; // 51-60% → saltar 1 onda
    else return 0;                          // 61-100% → no saltar ondas
}

void updateWaveBasedControl() {
    const float POT_MAX_V = 5.0;
    const float alpha = 0.15;

    // 1. LECTURA Y FILTRADO
    float rawV = sensores.getVoltage();
    rawV = constrain(rawV, 0.0, POT_MAX_V);
    filteredPotVoltage = alpha * rawV + (1 - alpha) * filteredPotVoltage;
    float vFiltered = filteredPotVoltage;
    float potNorm = vFiltered / POT_MAX_V;
    int potPercentage = (int)(potNorm * 100);

    // 2. CONTROL POR ONDAS (se mantiene igual)
    uint32_t skipCount = calculateWavesToSkip(potPercentage);
    
    for (int dev = 0; dev < NUM_DEVICES; dev++) {
        if (scrEnabled[dev]) {
            wavesToSkip[dev] = skipCount;
        }
    }

    // 3. CÁLCULO DE DELAY EN RANGO ÚTIL 8200-7500µs
    // Mapeo: 0V → 8200µs, 5V → 7500µs
    scrDelayUs = SAFE_MAX_DELAY - (uint32_t)(potNorm * (SAFE_MAX_DELAY - USEFUL_MIN_DELAY));

    // 4. LÍMITES DEL RANGO ÚTIL
    if (scrDelayUs > SAFE_MAX_DELAY) scrDelayUs = SAFE_MAX_DELAY;
    if (scrDelayUs < USEFUL_MIN_DELAY) scrDelayUs = USEFUL_MIN_DELAY;

    // 5. APAGADO COMPLETO
    if (vFiltered < 0.1) scrDelayUs = ABSOLUTE_MAX_DELAY;

    // 6. ACTUALIZAR SCR
    updateSCREnabledStates(potPercentage);
}

// ================== SETUP CORREGIDO ==================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== INICIANDO SISTEMA ===");

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
        zcQueues[i] = xQueueCreate(10, sizeof(ZCEvent_t)); // Cambiado a ZCEvent_t
        currentPhaseDelays[i] = 8300; // Inicializar con apagado
    }

    sensores.begin();
    ioController.begin(5,4);
    direction = true;                  // Por defecto: DIRECTA
    ioController.setRelay(1, true);    // Encender relé 2 (dirección directa)

    Serial.println("⚙️ Estado inicial: Dirección DIRECTA (Relé 2 ON)");

    // Configurar ISRs genéricas
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    
    // Fase A - dev = 0
    gpio_set_intr_type((gpio_num_t)zcPins[0], GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)zcPins[0], zcISR_Generic, (void*)0);
    
    // Fase B - dev = 1  
    gpio_set_intr_type((gpio_num_t)zcPins[1], GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)zcPins[1], zcISR_Generic, (void*)1);
    
    // Fase C - dev = 2
    gpio_set_intr_type((gpio_num_t)zcPins[2], GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)zcPins[2], zcISR_Generic, (void*)2);
    
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

    xTaskCreate(digitalInputTask, "inputTask", 4096, NULL, 1, NULL);
    xTaskCreate(controlTaskFaseA, "ctrlA", 4096, NULL, 3, NULL);
    xTaskCreate(controlTaskFaseB, "ctrlB", 4096, NULL, 3, NULL);
    xTaskCreate(controlTaskFaseC, "ctrlC", 4096, NULL, 3, NULL);

    i2cQueue = xQueueCreate(5, sizeof(I2CRequest));
    xTaskCreate(i2cManagerTask, "I2C Manager", 4096, NULL, 2, NULL);
    xTaskCreate(adsReadTask, "ADS Read Task", 4096, NULL, 3, NULL);

    Serial.println("=== SETUP COMPLETADO ===");
}

// ================== LOOP CON LOG PROGRESIVO ==================
void loop() {
    processSerialCommands();
    updateWaveBasedControl();

    // START con retardo de 3 segundos
    if (startRequested && !systemStarted) {
        if (millis() - startRequestTime >= 3000) {
            systemStarted = true;
            startRequested = false;
            ioController.setRelay(0, true);
            Serial.println("✅ Sistema iniciado, relé ON");
        }
    }

    // LOG periódico
    static uint32_t lastLog = 0;
    static uint32_t lastSkipLog = 0;
    if (millis() - lastLog > 500) {
        lastLog = millis();
        
        int potPercentage = (int)((filteredPotVoltage / 5.0) * 100);
        uint32_t currentSkip = wavesToSkip[0];
        
        Serial.printf("📊 Pot: %d%% (%.2fV) | ", potPercentage, filteredPotVoltage);
        
        if (currentSkip > 0) {
            Serial.printf("🌊 Conducir cada %lu ondas | ", currentSkip + 1);
        } else {
            Serial.printf("⚡ Conducción continua | ");
        }
        
        Serial.printf("Delay: %luµs\n", scrDelayUs);
        
        Serial.printf("   🔧 Fases: A[%s] B[%s] C[%s]", 
            scrEnabled[0] ? "ON" : "OFF", 
            scrEnabled[1] ? "ON" : "OFF", 
            scrEnabled[2] ? "ON" : "OFF");

        // Log de corrientes si está habilitado
        if (verboseLog) {
            Serial.printf(" | Corrientes: A=%.2fA B=%.2fA C=%.2fA",
                         sensores.getCurrent(0), sensores.getCurrent(1), sensores.getCurrent(2));
        }
        Serial.println();
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
}