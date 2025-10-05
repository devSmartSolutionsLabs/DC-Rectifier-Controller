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
        adsLow.setGain(GAIN_TWOTHIRDS);  // ±4.096V
        adsLow.setDataRate(RATE_ADS1115_860SPS); // ⚡ MÁXIMA VELOCIDAD
        Serial.println("✅ ADS1115 (0x48) inicializado");
    }
    
    // Inicializar ADS1115 de alta corriente  
    if (!adsHigh.begin(ADS1115_ADDRESS_HIGH)) {
        Serial.println("❌ No se pudo inicializar ADS1115 (0x49)");
        success = false;
    } else {
        adsHigh.setGain(GAIN_ONE);  // ±4.096V
        adsHigh.setDataRate(RATE_ADS1115_128SPS); // ⚡ MÁXIMA VELOCIDAD
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
        if (i < samples - 1) delay(1); // Pequeño delay entre lecturas
    }
    
    float average = (float)sum / samples;
    return average * gain / 1000.0;  // Convertir a voltios
}

// 🎯 LECTURA OPTIMIZADA PARA EL SISTEMA EN TIEMPO REAL
bool Sensores::readAllSensors(float* results, uint8_t numChannels) {
     if (!takeI2CMutex(500,"ADS_READ_ALL")) {   // espera hasta 500ms
        if(verboseLog) Serial.println("❌ No se pudo tomar mutex en readAllSensors");
        return false;
    }
    
    uint32_t startTime = micros();
    bool success = true;

    bool lowOK = false, highOK = false;
    
    // Verificar dispositivos rápidamente
    Wire.beginTransmission(ADS1115_ADDRESS_LOW);
    lowOK = (Wire.endTransmission() == 0);
    
    Wire.beginTransmission(ADS1115_ADDRESS_HIGH);  
    highOK = (Wire.endTransmission() == 0);
    
    if (lowOK) {
        int16_t potRaw = adsLow.readADC_SingleEnded(POT_CHANNEL);
        voltage = (potRaw * 0.1875) / 1000.0;
        potPercentageLocal = constrain((voltage / 5.0) * 100.0, 0, 100);
        ::potPercentage = (uint32_t)potPercentageLocal;
    } else {
        success = false;
    }

    if (lowOK || highOK) {
        for (int dev = 0; dev < NUM_DEVICES; dev++) {
            int16_t raw = 0;
            bool devOK = false;

            if (dev < 2 && lowOK) {                          // los dos primeros single-ended en 0x48
                raw = adsLow.readADC_SingleEnded(CURRENT_CHANNELS[dev]);
                devOK = true;
            } 
            
            else if (dev == 2 && highOK) {  // Corriente real en diferencial 0–1 de 0x49
                // Leer una única conversión diferencial estable
                int16_t raw = adsHigh.readADC_Differential_0_1();

                // LSB para GAIN_ONE (±4.096 V) = 125 µV/bit
                const float LSB_V = 7.8125e-6f;
                float vSense = raw * LSB_V;  // en voltios

                // Calibración empírica: 215 mV = 3300 A → 1 V = 15348 A
                float ampsInstant = vSense * 200.0f;

                // Filtro EMA (suavizado)
                static float currentEMA = 0.0f;
                const float alpha = 0.12f;  // 0.1 = más suave, 0.3 = más rápido
                currentEMA = (1 - alpha) * currentEMA + alpha * ampsInstant;

                // Zona muerta y límites
                if (fabs(currentEMA) < 10.0f) currentEMA = 0.0f;
                currentEMA = constrain(currentEMA, 0.0f, 6000.0f);

                current[dev] = currentEMA;

                Serial.printf("[ADS49] raw=%d | %.6f mV | %.1f A\n",
                                raw, vSense*1000, current[dev]);
                

                devOK = true;
            }
        }
    }            
        else {
        success = false;
    }

    giveI2CMutex();
    
    uint32_t duration = micros() - startTime;
    if (duration > 3000) {
        if (verboseLog) Serial.printf("⚠️ Sensores lentos: %luμs\n", duration);
    }
    
    return success;
}

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
