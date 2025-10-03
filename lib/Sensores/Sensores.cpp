#include <Arduino.h>
#include "Sensores.hpp"
#include "GlobalVars.hpp"

// Constantes para el potenciómetro
const int maxPotVoltage = 4200;
const int minPotVoltage = 1;
// Variables específicas de sensores
volatile int potPercentage = 0;
volatile float potVoltage = 0;
volatile float CurrentSensorVoltage = 0;
volatile float adcChannels[6] = {0};

// ==================================================
// Lectura promedio del ADS1115 con mutex
// ==================================================

float readAveraged(Adafruit_ADS1115 &ads, uint8_t channel, bool differential, float lsb_mV, int samples) {
    float sum = 0;

  
    if (!takeI2CMutex(50)) {
        Serial.printf("[ADS DEBUG] ❌ No se pudo tomar mutex para channel %d\n", channel);
        return 0.0;
    }

    for (int i = 0; i < samples; i++) {
        int16_t raw = 0;
        if (differential) {
            if (channel == 0) raw = ads.readADC_Differential_0_1();
            else if (channel == 1) raw = ads.readADC_Differential_2_3();
        } else {
            raw = ads.readADC_SingleEnded(channel);
        }
        sum += raw * lsb_mV;
    }

    giveI2CMutex();

    return sum / samples;
}


// ==================================================
// Lecturas específicas
// ==================================================
void readCurrentSafe() {
    static uint32_t lastReadTime = 0;

    if (millis() - lastReadTime >= POT_READ_INTERVAL_MS) {
        lastReadTime = millis();

        float measured = readAveraged(adsHigh, 0, true, 0.125, 2) - offsetCurrent;
        float calibrated = applyCalibration(measured, rawPointsDiff[0], realPointsDiff[0], pointCountDiff[0]);
        CurrentSensorVoltage = calibrated;
        adcChannels[4] = calibrated;
    }
}

void readPotenciometerSafe() {
    static uint32_t lastReadTime = 0;

    if (millis() - lastReadTime >= POT_READ_INTERVAL_MS) {
        lastReadTime = millis();

        if (!potManualControl) {
            float measured = readAveraged(adsLow, 0, false, 0.1875, 2);
            float calibrated = applyCalibration(measured, rawPointsSingle[0], realPointsSingle[0], pointCountSingle[0]);
            potVoltage = calibrated;

            int percentage = map(potVoltage, minPotVoltage, maxPotVoltage, 0, 100);
            percentage = constrain(percentage, 0, 100);
            potPercentage = percentage;
            adcChannels[0] = calibrated;
        } else {
            potPercentage = manualPotPercentage;
            potVoltage = map(potPercentage, 0, 100, minPotVoltage, maxPotVoltage);
            adcChannels[0] = potVoltage;
        }
    }
}

void readAllChannelsSafe() {
    static uint32_t lastReadTime = 0;

    if (millis() - lastReadTime >= POT_READ_INTERVAL_MS) {
        lastReadTime = millis();

        Serial.println("[ADS DEBUG] Intentando tomar mutex I2C para leer todos los canales...");
        if (takeI2CMutex(50)) {
            Serial.println("[ADS DEBUG] ✅ Mutex tomado para leer todos los canales");

            // 4 canales single-ended (ADS1115 0x48)
            for (int i = 0; i < 4; i++) {
                int16_t raw = adsLow.readADC_SingleEnded(i);
                float measured = raw * 0.1875;
                adcChannels[i] = applyCalibration(measured, rawPointsSingle[i], realPointsSingle[i], pointCountSingle[i]);
                Serial.printf("[ADS DEBUG] Single-ended channel %d: raw=%d, measured=%.3f\n", i, raw, adcChannels[i]);
            }

            // 2 canales diferenciales (ADS1115 0x49)
            for (int i = 0; i < 2; i++) {
                int16_t raw = (i == 0) ? adsHigh.readADC_Differential_0_1()
                                       : adsHigh.readADC_Differential_2_3();
                float measured = raw * 0.125;
                adcChannels[i + 4] = applyCalibration(measured, rawPointsDiff[i], realPointsDiff[i], pointCountDiff[i]);
                Serial.printf("[ADS DEBUG] Differential channel %d: raw=%d, measured=%.3f\n", i, raw, adcChannels[i + 4]);
            }

            giveI2CMutex();
            Serial.println("[ADS DEBUG] 🔓 Mutex liberado después de leer todos los canales");
        } else {
            Serial.println("[ADS DEBUG] ❌ No se pudo tomar mutex para leer todos los canales");
        }
    }
}


float readRawSingleEnded(uint8_t channel) {
    if (channel > 3) return 0.0;
    float result = 0;
    if (takeI2CMutex(30)) {
        result = adsLow.readADC_SingleEnded(channel) * 0.1875;
        giveI2CMutex();
    }
    return result;
}

float readRawDifferential(uint8_t pairIndex) {
    if (pairIndex > 1) return 0.0;
    float result = 0;
    if (takeI2CMutex(30)) {
        if (pairIndex == 0) result = adsHigh.readADC_Differential_0_1() * 0.125;
        else result = adsHigh.readADC_Differential_2_3() * 0.125;
        giveI2CMutex();
    }
    return result;
}
