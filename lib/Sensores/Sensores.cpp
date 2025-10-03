#include "Sensores.hpp"
#include "GlobalVars.hpp"
#include "mutexDebug.hpp"

Sensores::Sensores() 
    : potPercentageLocal(0), voltage(0) {
    // Inicializar offsets y gains
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
    
    // TOMAR el mutex ANTES de inicializar los ADS1115
    if (!takeI2CMutex(1000)) {
        Serial.println("❌ No se pudo tomar mutex para inicializar ADS1115");
        return false;
    }
    
    // Inicializar ADS1115 de baja corriente
    if (!adsLow.begin(ADS1115_ADDRESS_LOW)) {
        Serial.println("❌ No se pudo inicializar ADS1115 (0x48)");
        success = false;
    } else {
        adsLow.setGain(GAIN_ONE);  // ±4.096V
        adsLow.setDataRate(RATE_ADS1115_860SPS); // ⚡ MÁXIMA VELOCIDAD
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
    
    // LIBERAR el mutex después de inicializar
    giveI2CMutex();
    
    // Cargar calibración
    loadCalibration();
    
    return success;
}

// ⚡ LECTURA RÁPIDA SIN PROMEDIO (para uso en tiempo real)
float Sensores::readSingle(Adafruit_ADS1115& ads, uint8_t channel, bool differential, float gain) {
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
    return raw * gain / 1000.0;  // Convertir a voltios
}

// 📊 LECTURA CON PROMEDIO (para calibración y mediciones precisas)
float Sensores::readAveraged(Adafruit_ADS1115& ads, uint8_t channel, bool differential, float gain, int samples) {
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
        if (i < samples - 1) delay(1); // Pequeño delay entre lecturas, pero no después de la última
    }
    
    float average = (float)sum / samples;
    return average * gain / 1000.0;  // Convertir a voltios
}

// 🎯 LECTURA OPTIMIZADA PARA EL SISTEMA EN TIEMPO REAL
bool Sensores::readAllSensors(float* results, uint8_t numChannels) {
    // ⚡ VERIFICAR SI EL MUTEX ESTÁ BLOQUEADO
    if (xSemaphoreGetMutexHolder(i2cMutex) != NULL) {
        Serial.println("🚨 ERROR: Mutex ya estaba tomado en readAllSensors!");
        return false;
    }
    
    if (!takeI2CMutex(10)) { // ⚡ Timeout MUY corto
        Serial.println("❌ Timeout crítico en readAllSensors");
        return false;
    }
    
    uint32_t startTime = micros();
    bool success = true;

    // ⚡ LECTURA CON PROTECCIÓN MÁXIMA
    bool lowOK = false, highOK = false;
    
    // Verificar dispositivos rápidamente
    Wire.beginTransmission(ADS1115_ADDRESS_LOW);
    lowOK = (Wire.endTransmission() == 0);
    
    Wire.beginTransmission(ADS1115_ADDRESS_HIGH);  
    highOK = (Wire.endTransmission() == 0);
    
    if (lowOK) {
        // Solo leer si el dispositivo responde
        int16_t potRaw = adsLow.readADC_SingleEnded(POT_CHANNEL);
        voltage = (potRaw * 0.1875) / 1000.0;
        potPercentageLocal = constrain((voltage / 3.3) * 100.0, 0, 100);
        ::potPercentage = (uint32_t)potPercentageLocal;
    } else {
        success = false;
    }

    if (lowOK || highOK) {
        for (int dev = 0; dev < NUM_DEVICES; dev++) {
            int16_t currentRaw = 0;
            bool devOK = false;
            
            if (dev < 2 && lowOK) {
                currentRaw = adsLow.readADC_SingleEnded(CURRENT_CHANNELS[dev]);
                devOK = true;
            } else if (dev >= 2 && highOK) {
                currentRaw = adsHigh.readADC_SingleEnded(CURRENT_CHANNELS[dev] - 4);
                devOK = true;
            }
            
            if (devOK) {
                float currentVoltage = (currentRaw * 0.1875) / 1000.0;
                current[dev] = (currentVoltage - offsetCurrentLow[dev]) * gainCurrentLow[dev];
            } else {
                current[dev] = 0.0;
            }
        }
    } else {
        success = false;
    }

    giveI2CMutex();
    
    uint32_t duration = micros() - startTime;
    if (duration > 3000) {
        Serial.printf("⚠️ Sensores lentos: %luμs\n", duration);
    }
    
    return success;
}

float Sensores::readADSChannel(uint8_t adsIndex, uint8_t channel, bool differential) {
    if (!takeI2CMutex(50)) {
        Serial.println("❌ [Sensores] Timeout en readADSChannel");
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

// 📈 MÉTODOS ORIGINALES (optimizados)
void Sensores::readPotenciometer() {
    // Ahora se hace en readAllSensors, mantener por compatibilidad
    if (!takeI2CMutex(30)) return;
    
    float rawVoltage = readSingle(adsLow, POT_CHANNEL, false, 0.1875);
    voltage = rawVoltage;
    potPercentageLocal = (voltage / 3.3) * 100.0;
    potPercentageLocal = constrain(potPercentageLocal, 0, 100);
    ::potPercentage = (uint32_t)potPercentageLocal;
    
    giveI2CMutex();
}

void Sensores::readCurrent() {
    // Ahora se hace en readAllSensors, mantener por compatibilidad
    if (!takeI2CMutex(30)) return;
    
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
    // ⚡ USAR EL NUEVO MÉTODO OPTIMIZADO
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
    
    if (!takeI2CMutex(100)) {
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
        // Calcular nuevo gain
        if (knownCurrent != 0) {
            gainCurrentHigh[device] = knownCurrent / (rawValue - offsetCurrentHigh[device]);
        }
    } else {
        // Para calibración de offset, knownCurrent debería ser 0
        if (knownCurrent == 0) {
            offsetCurrentLow[device] = rawValue;
        }
    }
    
    giveI2CMutex();
    
    Serial.printf("Calibración dispositivo %d - Raw: %.4f, Known: %.2f\n", 
                  device, rawValue, knownCurrent);
}

void Sensores::saveCalibration() {
    // Implementar guardado en EEPROM o SPIFFS
    Serial.println("💾 Guardando calibración...");
}

void Sensores::loadCalibration() {
    // Implementar carga desde EEPROM o SPIFFS  
    Serial.println("📂 Cargando calibración...");
}