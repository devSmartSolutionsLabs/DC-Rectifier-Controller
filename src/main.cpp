#include <Arduino.h>
#include <driver/gpio.h>
#include <Adafruit_ADS1X15.h>
#include "Calibracion.hpp"
#include "Sensores.hpp"
#include "TTL.hpp"
#include "MCP23017_IO.hpp"  // ← AÑADIR ESTA LÍNEA
#include "GlobalVars.hpp"



// Agregar estas variables globales
volatile bool interruptsEnabled = true;
volatile bool pwmGenerationEnabled = true;


QueueHandle_t zcQueues[NUM_DEVICES] = {NULL, NULL, NULL};
esp_timer_handle_t fireTimers[NUM_DEVICES];

Adafruit_ADS1115 adsLow;   // 0x48
Adafruit_ADS1115 adsHigh;  // 0x49

const int zcPins[NUM_DEVICES] = {38, 47, 14};
const int scrPins[NUM_DEVICES] = {48, 21, 13}; // 11 miso

// Variables para control y medición
volatile uint32_t lastZCTime[NUM_DEVICES] = {0};
volatile bool scrActive[NUM_DEVICES] = {false};
volatile uint32_t pulseStartTime[NUM_DEVICES] = {0};
volatile uint32_t pulseDuration[NUM_DEVICES] = {0};
volatile uint32_t pulseCount[NUM_DEVICES] = {0};
volatile uint32_t zcCount[NUM_DEVICES] = {0};


// Variables para MCP23017 ← AÑADIR ESTAS VARIABLES

unsigned long lastInputCheckTime = 0;
const unsigned long INPUT_CHECK_INTERVAL = 100; // 100ms entre lecturas


void IRAM_ATTR startDelayCallback(void* arg) {
    if (startRequested) {
        systemStarted = true;
        ioController.setRelay(0, true);  // Relé 1 ON
        startRequested = false;
        Serial.println("Sistema INICIADO - Relé 1 activado");
        Serial.println("Delay de inicio completado");
    }
}

// FUNCIÓN FORZADA PARA APAGAR SCR
void IRAM_ATTR forceTurnOffSCR(uint8_t dev) {
    GPIO.out_w1tc = (1 << scrPins[dev]);  
    //digitalWrite(scrPins[dev], LOW);
    scrActive[dev] = false;
    if (pulseStartTime[dev] > 0) {
        pulseDuration[dev] = micros() - pulseStartTime[dev];
        pulseStartTime[dev] = 0;
    }
}

// ✅ ISRs OPTIMIZADAS
void IRAM_ATTR zcISR_FaseA(void* arg) {
    if (!systemStarted) return;  // ← NO procesar si sistema deshabilitado
    
    uint32_t now = micros();
    if (now - lastZCTime[0] > DEBOUNCE_TIME_US) {
        lastZCTime[0] = now;
        zcCount[0]++;
        GPIO.out_w1tc = (1 << scrPins[0]);  
        //digitalWrite(scrPins[0], LOW);
        scrActive[0] = false;
        static uint8_t dev = 0;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(zcQueues[0], &dev, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
    }
}

void IRAM_ATTR zcISR_FaseB(void* arg) {
    if (!systemStarted) return;  // ← NO procesar si sistema deshabilitado

    uint32_t now = micros();
    if (now - lastZCTime[1] > DEBOUNCE_TIME_US) {
        lastZCTime[1] = now;
        zcCount[1]++;
        GPIO.out_w1tc = (1 << scrPins[1]);  
        //digitalWrite(scrPins[1], LOW);
        scrActive[1] = false;
        static uint8_t dev = 1;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(zcQueues[1], &dev, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
    }
}

void IRAM_ATTR zcISR_FaseC(void* arg) {
    if (!systemStarted) return;  // ← NO procesar si sistema deshabilitado
    
    uint32_t now = micros();
    if (now - lastZCTime[2] > DEBOUNCE_TIME_US) {
        lastZCTime[2] = now;
        zcCount[2]++;
        GPIO.out_w1tc = (1 << scrPins[2]);  
        //digitalWrite(scrPins[2], LOW);
        scrActive[2] = false;
        static uint8_t dev = 2;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(zcQueues[2], &dev, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
    }
}

// Callback de timer - ENCENDER SCR
void IRAM_ATTR timerCallback(void* arg) {
    if (!systemStarted) return;  // ← No disparar si sistema deshabilitado
    
    uint8_t dev = (uint8_t)(intptr_t)arg;
    GPIO.out_w1ts = (1 << scrPins[dev]);  
    //digitalWrite(scrPins[dev], HIGH);
    scrActive[dev] = true;
    pulseStartTime[dev] = micros();
    pulseCount[dev]++;
}

// Tarea dedicada para leer el potenciómetro
void VoltageReadTask(void* param) {
    while (true) {
        readPotenciometerSafe();
        readCurrentSafe();
        readAllChannelsSafe(); // Leer todos los canales
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
            if (!systemStarted) {
                forceTurnOffSCR(0);
                continue;
            }
            
            delay_us = map(potPercentage, 0, 100, SEMI_PERIOD_US, 1);
            if (delay_us < SEMI_PERIOD_US) {
                esp_timer_start_once(fireTimers[0], delay_us);
            }
            
            // ✅ SOLO IMPRIMIR EN MODO TEST
            if (testMode && zcCount[0] > lastZC) {
                lastZC = zcCount[0];
                float firingAngle = (delay_us * 180.0) / SEMI_PERIOD_US;
                //Serial.printf("[FASE A] ZC#%lu | Pot: %d%% | Angle: %.1f°", 
                  //           zcCount[0], potPercentage, firingAngle);
                //Serial.print("                                           \r");  // <-- sobrescribe en la misma línea             
                //Serial.println();
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
            if (!systemStarted) {
                forceTurnOffSCR(1);
                continue;
            }
            
            delay_us = map(potPercentage, 0, 100, SEMI_PERIOD_US, 1);
            if (delay_us < SEMI_PERIOD_US) {
                esp_timer_start_once(fireTimers[1], delay_us);
            }
            
            // ✅ SOLO IMPRIMIR EN MODO TEST
            if (testMode && zcCount[1] > lastZC) {
                lastZC = zcCount[1];
                Serial.printf("[FASE B] ZC#%lu | Pot: %d%%\n", zcCount[1], potPercentage);
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
            if (!systemStarted) {
                forceTurnOffSCR(2);
                continue;
            }
            
            delay_us = map(potPercentage, 0, 100, SEMI_PERIOD_US, 1);
            if (delay_us < SEMI_PERIOD_US) {
                esp_timer_start_once(fireTimers[2], delay_us);
            }
            
            // ✅ SOLO IMPRIMIR EN MODO TEST
            if (testMode && zcCount[2] > lastZC) {
                lastZC = zcCount[2];
                Serial.printf("[FASE C] ZC#%lu | Pot: %d%%\n", zcCount[2], potPercentage);
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void processInputChanges(uint8_t inputStates) {
    // Leer botones (invertidos por pull-up)
    bool startButton = !(inputStates & 0x01);    // B0 - START
    bool stopButton = !(inputStates & 0x02);     // B1 - STOP  
    bool directionSelector = !(inputStates & 0x04); // B2 - DIR

    // ✅ PROCESAR BOTÓN START (B0) CON DELAY
    static bool lastStartState = false;
    if (startButton && !lastStartState) {  // Flanco de subida (presionado)
        if (!systemStarted && !startRequested) {
            // SOLICITAR INICIO CON DELAY
            startRequested = true;
            startRequestTime = millis();
            esp_timer_start_once(startDelayTimer, START_DELAY_MS * 1000);
            
            Serial.printf("Solicitud de inicio recibida. Iniciando en %d segundos...\n", 
                         START_DELAY_MS / 1000);
            Serial.println("Mantenga presionado STOP para cancelar");
        }
    }
    lastStartState = startButton;

    // ✅ PROCESAR BOTÓN STOP (B1) - CANCELAR INICIO O DETENER
    static bool lastStopState = false;
    if (stopButton && !lastStopState) {  // Flanco de subida (presionado)
        // CANCELAR INICIO PENDIENTE
        if (startRequested) {
            startRequested = false;
            esp_timer_stop(startDelayTimer);
            Serial.println("Inicio cancelado por usuario");
        }
        
        // DETENER SISTEMA SI ESTÁ ACTIVO
        if (systemStarted) {
            systemStarted = false;
            ioController.setRelay(0, false);  // Relé 1 OFF
            
            // Apagar todos los SCRs inmediatamente
            for (int i = 0; i < NUM_DEVICES; i++) {
                forceTurnOffSCR(i);
            }
            
            Serial.println("Sistema DETENIDO - Relé 1 desactivado");
        }
    }
    lastStopState = stopButton;

    // ✅ PROCESAR SELECTOR DIRECCIÓN (B2) - SIEMPRE
    static bool lastDirection = false;
    if (directionSelector != lastDirection) {
        direction = directionSelector;
        ioController.setRelay(1, direction);
        
        if (systemStarted) {
            Serial.printf("[SISTEMA ACTIVO] Dirección: %s\n", direction ? "DIR-A" : "DIR-B");
        } else if (startRequested) {
            Serial.printf("[INICIO PENDIENTE] Dirección: %s\n", direction ? "DIR-A" : "DIR-B");
        } else {
            Serial.printf("[SISTEMA INACTIVO] Dirección: %s\n", direction ? "DIR-A" : "DIR-B");
        }
        
        lastDirection = directionSelector;
    }
}

void checkDigitalInputs() {
    if (!ioControlEnabled) return;
    
    static unsigned long lastDebounceTime = 0;
    static uint8_t lastStableState = 0;
    
    if (millis() - lastInputCheckTime >= INPUT_CHECK_INTERVAL) {
        lastInputCheckTime = millis();
        
        uint8_t currentInputs = ioController.readGPIOB();
        
        // Detectar cambios con debounce
        if (currentInputs != lastStableState) {
            if (millis() - lastDebounceTime > ioController.getDebounceTime()) {
                lastDebounceTime = millis();
                lastStableState = currentInputs;
                
                // Procesar cambios de estado
                processInputChanges(currentInputs);
                
                // Logging si está en modo monitor
                if (ioController.isMonitoring()) {
                    Serial.println(ioController.getInputsStatus());
                }
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
        GPIO.out_w1tc = (1 << scrPins[i]);  
        //digitalWrite(scrPins[i], LOW);
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

    esp_timer_create_args_t startDelayArgs = {
        .callback = &startDelayCallback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "startDelayTimer",
        .skip_unhandled_events = false
    };
    esp_timer_create(&startDelayArgs, &startDelayTimer);

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
    
    // Verificar entradas digitales y procesar botones
    checkDigitalInputs();
    
    // Solo procesar zero crossing si el sistema está iniciado
    if (!systemStarted) {
        for (int i = 0; i < NUM_DEVICES; i++) {
            forceTurnOffSCR(i); // Asegurar que todos los SCRs estén apagados
        }
    }
    
    // Estadísticas normales (solo fuera del modo calibración)
    static uint32_t lastStatsTime = 0;
    // En el loop principal de main.cpp:
    if (millis() - lastStatsTime > 1000 && !calibrationMode) {
        lastStatsTime = millis();
        
        // ✅ SOLO IMPRIMIR EN MODO TEST
        if (testMode) {
        Serial.print("                          \r");  // <-- sobrescribe en la misma línea
        Serial.println("=== VALORES DE CANALES ===");
        
        // Información del sistema
        Serial.printf("Sistema: %s | Pot: %d%% | Dirección: %s\n", 
                    systemStarted ? "ACTIVO" : "INACTIVO",
                    potPercentage,
                    direction ? "DIR-A" : "DIR-B");
        
        // Canales single-ended (ADS 0x48)
        Serial.println("Single-ended (0x48):");
        Serial.printf("  CH0(Pot): %.2f mV | CH1: %.2f mV | CH2: %.2f mV | CH3: %.2f mV\n", 
                    adcChannels[0], adcChannels[1], adcChannels[2], adcChannels[3]);
        
        // Canales diferenciales (ADS 0x49)
        Serial.println("Diferenciales (0x49):");
        Serial.printf("  DIF0-1(Corriente): %.2f mV | DIF2-3: %.2f mV\n", 
                    adcChannels[4], adcChannels[5]);
        
        Serial.println("------------------------");
    }
    }

    // Mostrar cuenta regresiva si hay inicio pendiente
    if (startRequested) {
        static uint32_t lastCountdownTime = 0;
        if (millis() - lastCountdownTime > 1000) {  // Actualizar cada segundo
            lastCountdownTime = millis();
            uint32_t elapsed = millis() - startRequestTime;
            uint32_t remaining = START_DELAY_MS - elapsed;
            
            if (remaining > 0) {
                Serial.printf("Iniciando en: %d segundos\n", remaining / 1000);
            }
        }
    }
    
    vTaskDelay(50 / portTICK_PERIOD_MS);
}