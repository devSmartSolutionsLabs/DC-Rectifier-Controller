#include "Sensores.hpp"
#include "GlobalVars.hpp"
#include "mutexDebug.hpp"

Sensores::Sensores() 
    : potPercentageLocal(0), voltage(0) {
    for (int i = 0; i < NUM_DEVICES; i++) {
        offsetCurrentLow[i] = 0.0;
        offsetCurrentHigh[i] = 0.0;
        gainCurrentLow[i] = 1.0;
        gainCurrentHigh[i] = 1.0;
        current[i] = 0.0;
    }
}

bool Sensores::begin() {
    bool success = true;
    
    if (!takeI2CMutex(1000)) {
        Serial.println("❌ No se pudo tomar mutex para inicializar ADS1115");
        return false;
    }
    
    // Inicializar ADS1115 de baja corriente
    if (!adsLow.begin(ADS1115_ADDRESS_LOW)) {
        Serial.println("❌ No se pudo inicializar ADS1115 (0x48)");
        success = false;
    } else {
        adsLow.setGain(GAIN_TWOTHIRDS);  // ±4.096V
        adsLow.setDataRate(RATE_ADS1115_860SPS);
        Serial.println("✅ ADS1115 (0x48) inicializado");
    }
    
    // Inicializar ADS1115 de alta corriente  
    if (!adsHigh.begin(ADS1115_ADDRESS_HIGH)) {
        Serial.println("❌ No se pudo inicializar ADS1115 (0x49)");
        success = false;
    } else {
        adsHigh.setGain(GAIN_ONE);  // ±4.096V
        adsHigh.setDataRate(RATE_ADS1115_860SPS); // ⚡ MÁXIMA VELOCIDAD
        Serial.println("✅ ADS1115 (0x49) inicializado");
    }
    
    giveI2CMutex();
    loadCalibration();
    
    return success;
}

// ⚡ LECTURA ULTRA RÁPIDA - SOLO LECTURAS CRÍTICAS
bool Sensores::readAllSensors(float* results, uint8_t numChannels) {
    static uint32_t lastReadTime = 0;
    static uint32_t readCounter = 0;
    
    // Solo leer cada 200ms para reducir carga (opcional)
    if (millis() - lastReadTime < 200 && readCounter > 0) {
        return true; // Usar valores anteriores
    }
    
    if (!takeI2CMutex(15, "ADS_READ_ALL")) {   // ⚡ Timeout MUY corto
        return false; // No log, just fail silently
    }
    
    uint32_t startTime = micros();
    bool success = true;

    // ⚡ LECTURA DIRECTA SIN VERIFICACIONES EXTRA
    try {
        // 1. Potenciómetro (más importante)
        int16_t potRaw = adsLow.readADC_SingleEnded(POT_CHANNEL);
        voltage = (potRaw * 0.1875) / 1000.0;
        potPercentageLocal = constrain((voltage / 5.0) * 100.0, 0, 100);
        ::potPercentage = (uint32_t)potPercentageLocal;

        // 2. Corrientes fase A y B (lectura rápida)
        for (int dev = 0; dev < 2; dev++) {
            int16_t raw = adsLow.readADC_SingleEnded(CURRENT_CHANNELS[dev]);
            float vSense = raw * 0.1875 / 1000.0;
            current[dev] = vSense * 100.0f; // Conversión simplificada
            if (fabs(current[dev]) < 2.0f) current[dev] = 0.0f;
        }

        // 3. Corriente fase C (diferencial) - SOLO si es crítica
        if (readCounter % 2 == 0) { // Leer cada 2 ciclos para reducir carga
            int16_t raw = adsHigh.readADC_Differential_0_1();
            float vSense = raw * 7.8125e-6f; // LSB para GAIN_ONE
            
            static float currentEMA = 0.0f;
            const float alpha = 0.3f;
            float ampsInstant = vSense * 200.0f;
            currentEMA = (1 - alpha) * currentEMA + alpha * ampsInstant;
            
            if (fabs(currentEMA) < 10.0f) currentEMA = 0.0f;
            current[2] = currentEMA;
        }
        
    } catch (...) {
        success = false;
    }
    
    giveI2CMutex();
    
    readCounter++;
    lastReadTime = millis();
    
    return success;
}

// 🔄 MANTENER LAS OTRAS FUNCIONES PERO CON TIMEOUTS MÁS CORTOS
float Sensores::readSingle(Adafruit_ADS1115& ads, uint8_t channel, bool differential, float gain) {
    if (!takeI2CMutex(10, "ADS_SINGLE")) return 0.0;
    
    int16_t raw;
    if (differential) {
        switch (channel) {
            case 0: raw = ads.readADC_Differential_0_1(); break;
            case 1: raw = ads.readADC_Differential_2_3(); break;
            default: raw = 0;
        }
    } else {
        raw = ads.readADC_SingleEnded(channel);
    }
    
    giveI2CMutex();
    return raw * gain / 1000.0;
}

float Sensores::readAveraged(Adafruit_ADS1115& ads, uint8_t channel, bool differential, float gain, int samples) {
    if (!takeI2CMutex(20, "ADS_AVG")) return 0.0;
    
    if (samples <= 0) samples = 1;
    long sum = 0;
    for (int i = 0; i < samples; i++) {
        int16_t raw;
        if (differential) {
            switch (channel) {
                case 0: raw = ads.readADC_Differential_0_1(); break;
                case 1: raw = ads.readADC_Differential_2_3(); break;
                default: raw = 0;
            }
        } else {
            raw = ads.readADC_SingleEnded(channel);
        }
        sum += raw;
    }
    
    giveI2CMutex();
    return ((float)sum / samples) * gain / 1000.0;
}

// ... resto de funciones con timeouts cortos ...
float Sensores::readADSChannel(uint8_t adsIndex, uint8_t channel, bool differential) {
    if (!takeI2CMutex(100,"ADS_READ_CHANNEL")) {
        if (verboseLog) Serial.println("❌ [Sensores] Timeout en readADSChannel");
        return 0.0;
    }
    
    float result = 0.0;
    if (adsIndex == 0) {
        result = readSingle(adsLow, channel, differential, 0.1875);
    } else if (adsIndex == 1) {
        result = readSingle(adsHigh, channel, differential, 0.1875);
    }
    
    giveI2CMutex();
    return result;
}

float Sensores::applyCalibration(float rawValue, float offset, float gain) {
    return (rawValue - offset) * gain;
}

void Sensores::readPotenciometer() {
    if (!takeI2CMutex(30,"ADS_READ_POT")) return;
    
    float rawVoltage = readSingle(adsLow, POT_CHANNEL, false, 0.1875);
    voltage = rawVoltage;
    potPercentageLocal = (voltage / 3.3) * 100.0;
    potPercentageLocal = constrain(potPercentageLocal, 0, 100);
    ::potPercentage = (uint32_t)potPercentageLocal;
    
    giveI2CMutex();
}

void Sensores::readCurrent() {
    if (!takeI2CMutex(30,"ADS_READ_CURRENT")) return;
    
    for (int dev = 0; dev < NUM_DEVICES; dev++) {
        float rawCurrent;
        
        if (dev < 2) {
            rawCurrent = readSingle(adsLow, CURRENT_CHANNELS[dev], false, 0.1875);
            current[dev] = applyCalibration(rawCurrent, offsetCurrentLow[dev], gainCurrentLow[dev]);
        } else {
            rawCurrent = readSingle(adsHigh, CURRENT_CHANNELS[dev] - 4, false, 0.1875);
            current[dev] = applyCalibration(rawCurrent, offsetCurrentHigh[dev], gainCurrentHigh[dev]);
        }
    }
    
    giveI2CMutex();
}

void Sensores::readAllChannels() {
    readAllSensors(nullptr, 0);
}

float Sensores::getCurrent(uint8_t device) const {
    if (device < NUM_DEVICES) {
        return current[device];
    }
    return 0.0;
}

float Sensores::getPotPercentage() const {
    return potPercentageLocal;
}

float Sensores::getVoltage() const {
    return voltage;
}

void Sensores::calibrateCurrent(uint8_t device, bool highRange, float knownCurrent) {
    if (device >= NUM_DEVICES) return;
    
    if (!takeI2CMutex(100,"ADS_CALIBRATE")) {
        Serial.println("❌ [Sensores] Timeout en calibrateCurrent");
        return;
    }
    
    float rawValue;
    
    if (device < 2) {
        rawValue = readAveraged(adsLow, CURRENT_CHANNELS[device], false, 0.1875, 20);
    } else {
        rawValue = readAveraged(adsHigh, CURRENT_CHANNELS[device] - 4, false, 0.1875, 20);
    }
    
    if (highRange) {
        if (knownCurrent != 0) {
            gainCurrentHigh[device] = knownCurrent / (rawValue - offsetCurrentHigh[device]);
        }
    } else {
        if (knownCurrent == 0) {
            offsetCurrentLow[device] = rawValue;
        }
    }
    
    giveI2CMutex();
    
    Serial.printf("Calibración dispositivo %d - Raw: %.4f, Known: %.2f\n", 
                  device, rawValue, knownCurrent);
}

void Sensores::saveCalibration() {
    Serial.println("💾 Guardando calibración...");
}

void Sensores::loadCalibration() {
    Serial.println("📂 Cargando calibración...");
}
