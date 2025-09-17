#include "Sensores.hpp"

// Constantes para el potenciómetro
const int maxPotVoltage = 3600;
const int minPotVoltage = 100;
const int POT_READ_INTERVAL_MS = 330;

// Implementación de funciones de lectura de sensores
float readAveraged(Adafruit_ADS1115 &ads, uint8_t pairIndex, float lsb_mV, int samples) {
    long sum = 0;
    for (int i = 0; i < samples; i++) {
        int16_t raw;
        if (pairIndex == 0) raw = ads.readADC_Differential_0_1();
        else                raw = ads.readADC_Differential_2_3();
        sum += raw;
    }
    float avgRaw = (float)sum / samples;
    return avgRaw * lsb_mV;
}

void readCurrentSafe() {
    static uint32_t lastReadTime = 0;

    if (millis() - lastReadTime >= POT_READ_INTERVAL_MS) {
        lastReadTime = millis();

        // Amperímetro: ADS 0x49, diferencial 0-1, ±256mV (0.0078125 mV/LSB)
        float measured = readAveraged(adsHigh, 0, 0.125, 4); - offsetCurrent; 
        float calibrated = applyCalibration(measured, rawPointsCurrent, realPointsCurrent, pointCountCurrent);

        CurrentSensorVoltage = calibrated;
    }
}

void readPotenciometerSafe() {
    static uint32_t lastReadTime = 0;

    if (millis() - lastReadTime >= POT_READ_INTERVAL_MS) {
        lastReadTime = millis();

        // Voltímetro: ADS 0x48, diferencial 2-3, ±4.096V (0.125 mV/LSB)
        float measured = readAveraged(adsLow, 0, 0.125, 4); - offsetVoltage;
        float calibrated = applyCalibration(measured, rawPointsVoltage, realPointsVoltage, pointCountVoltage);

        potVoltage = calibrated;
        int percentage = map((int)(calibrated), minPotVoltage, maxPotVoltage, 0, 100);
        potPercentage = constrain(percentage, 0, 100);
    }
}

float readRawCurrentUncalibrated() {
    // Amperímetro en ADS 0x49, diferencial 0-1, ±256mV
    return readAveraged(adsHigh, 0, 0.125, 4);
}

float readRawVoltageUncalibrated() {
    // Voltímetro en ADS 0x48, diferencial 2-3, ±4.096V
    return readAveraged(adsLow, 0, 0.125, 4);
}

float readRawVoltagePort0() {
    // Voltímetro en ADS 0x48, diferencial 0-1, ±4.096V
    return readAveraged(adsLow, 0, 0.125, 4);
}

float readRawCurrentPort1() {
    // Amperímetro en ADS 0x49, diferencial 2-3, ±256mV
    return readAveraged(adsHigh, 0, 0.125, 4);
}
