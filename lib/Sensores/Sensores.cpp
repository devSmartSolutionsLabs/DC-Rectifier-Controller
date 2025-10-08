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
        adsHigh.setGain(GAIN_SIXTEEN);  // ±0.256 V
        adsHigh.setDataRate(RATE_ADS1115_128SPS); // ⚡ MÁXIMA VELOCIDAD
        Serial.println("✅ ADS1115 (0x49) inicializado");
    }
    
    giveI2CMutex();
    loadCalibration();
    
    return success;
}

// ⚡ LECTURA ULTRA RÁPIDA - SOLO LECTURAS CRÍTICAS
bool Sensores::readAllSensors(float* results, uint8_t numChannels) {
    if (!takeI2CMutex(200, "ADS_READ_ALL")) { 
        if (verboseLog) Serial.println("❌ No se pudo tomar mutex en readAllSensors");
        return false;
    }
    
    uint32_t startTime = micros();
    bool success = true;

    // 1. LEER POTENCIÓMETRO (canal single-ended en ADS 0x48)
    int16_t potRaw = adsLow.readADC_SingleEnded(POT_CHANNEL);
    voltage = (potRaw * 0.1875) / 1000.0; // Convertir a voltios
    potPercentageLocal = constrain((voltage / 5.0) * 100.0, 0, 100);
    ::potPercentage = (uint32_t)potPercentageLocal;

    // 2. ⚡ LEER CORRIENTE - CANAL DIFERENCIAL 0-1 EN ADS 0x49
    int16_t currentRaw = adsHigh.readADC_Differential_0_1();
    if (abs(currentRaw) > 6400) {
        Serial.printf("⚠️ Lectura fuera de rango: %d\n", currentRaw);
        currentRaw = constrain(currentRaw, -6400, 6400);   // O bien descarta: raw = prevRaw;
    }
    
    // Conversión a amperios
    // ADS1115 con GAIN_ONE: ±0.256V, LSB = 125µV
    constexpr float LSB = 7.8125e-6f; // voltios por bit
    float voltageDiff = currentRaw * LSB;
    
    // ⚡ CALIBRAR ESTE FACTOR SEGÚN TU SENSOR DE CORRIENTE
    // Ejemplo: Si usas sensor de 30A/1V, factor = 30
    // Ejemplo: Si usas shunt, calcular según resistencia
    const float CURRENT_SENSITIVITY = 20000.00000f;  // 20 A/mV = 20000 A/V
    float currentInstant = voltageDiff* CURRENT_SENSITIVITY;
    
    // Aplicar filtro para suavizar lectura
    static float currentFiltered[NUM_DEVICES] = {0, 0, 0};
    const float alpha = 0.5; // Factor de filtrado
    
    currentFiltered[2] = (1 - alpha) * currentFiltered[2] + alpha * currentInstant;
    
    // Eliminar ruido cerca de cero
    if (fabs(currentFiltered[2]) < 0.5) {
        currentFiltered[2] = 0.0;
    }
    
    // Asignar a variable global (fase C)
    current[2] = currentFiltered[2];
    // Asignar a variable global (fase C)

    // 🔹 Mostrar resultado en monitor serie
    Serial.printf("ADC= %d | ΔV= %.6f V | Corriente= %.2f A\n", 
                currentRaw, voltageDiff, current[2]);

    // 3. LEER OTRAS CORRIENTES SI ES NECESARIO
    // (Mantener las lecturas existentes para fases A y B si las tienes)
    for (int dev = 0; dev < 2; dev++) {
        // Tus lecturas existentes para fases A y B
        int16_t raw = adsLow.readADC_SingleEnded(CURRENT_CHANNELS[dev]);
        float vSense = raw * 0.1875 / 1000.0;
        current[dev] = vSense * 100.0f; // Ajustar según tu calibración
        if (fabs(current[dev]) < 2.0f) current[dev] = 0.0f;
    }

    giveI2CMutex();
    
    uint32_t duration = micros() - startTime;
    if (duration > 5000 && verboseLog) {
        Serial.printf("[SENSORES] Lectura tomó %luμs\n", duration);
    }
    
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
