#include <Arduino.h>
#include "Sensores.hpp"

// Constantes para el potenciómetro
const int maxPotVoltage = 3600;
const int minPotVoltage = 100;
const int POT_READ_INTERVAL_MS = 330;

// Variables específicas de sensores
volatile int potPercentage = 50;
volatile float potVoltage = 0;
volatile float CurrentSensorVoltage = 0;

// Array para todos los canales (0-3: single-ended, 4-5: differential)
volatile float adcChannels[6] = {0};

// Implementación de funciones de lectura de sensores
float readAveraged(Adafruit_ADS1115 &ads, uint8_t channel, bool differential, float lsb_mV, int samples) {
    long sum = 0;
    for (int i = 0; i < samples; i++) {
        int16_t raw;
        if (differential) {
            if (channel == 0) raw = ads.readADC_Differential_0_1();
            else raw = ads.readADC_Differential_2_3();
        } else {
            raw = ads.readADC_SingleEnded(channel);
        }
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
        float measured = readAveraged(adsHigh, 0, true, 0.0078125, 4) - offsetCurrent; 
        
        // Usar calibración para canal diferencial 0
        float calibrated = applyCalibration(measured, rawPointsDiff[0], realPointsDiff[0], pointCountDiff[0]);

        CurrentSensorVoltage = calibrated;
        adcChannels[4] = calibrated; // Canal 4 es diferencial 0-1
    }
}

void readPotenciometerSafe() {
    static uint32_t lastReadTime = 0;

    if (millis() - lastReadTime >= POT_READ_INTERVAL_MS) {
        lastReadTime = millis();

        // Solo leer el potenciómetro físico si no estamos en control manual
        if (!potManualControl) {
            // Potenciómetro: ADS 0x48, single-ended canal 0, ±4.096V (0.125 mV/LSB)
            float measured = readAveraged(adsLow, 0, false, 0.125, 4) - offsetVoltage;
            
            // Usar calibración para canal single-ended 0
            float calibrated = applyCalibration(measured, rawPointsSingle[0], realPointsSingle[0], pointCountSingle[0]);

            potVoltage = calibrated;
            int percentage = map((int)(calibrated), minPotVoltage, maxPotVoltage, 0, 100);
            potPercentage = constrain(percentage, 0, 100);
            adcChannels[0] = calibrated; // Canal 0 es single-ended
        }
        // Si estamos en control manual, usar el valor establecido por serial
        else {
            potPercentage = manualPotPercentage;
            // Opcional: calcular un voltage simulado para mostrar
            potVoltage = map(potPercentage, 0, 100, minPotVoltage, maxPotVoltage);
            adcChannels[0] = potVoltage;
        }
    }
}

void readAllChannelsSafe() {
    static uint32_t lastReadTime = 0;

    if (millis() - lastReadTime >= POT_READ_INTERVAL_MS) {
        lastReadTime = millis();

        // Leer los 4 canales single-ended del ADS 0x48
        for (int i = 0; i < 4; i++) {
            float measured = readAveraged(adsLow, i, false, 0.125, 2);
            // Aplicar calibración específica para cada canal
            adcChannels[i] = applyCalibration(measured, rawPointsSingle[i], realPointsSingle[i], pointCountSingle[i]);
        }

        // Leer los 2 canales diferenciales del ADS 0x49
        for (int i = 0; i < 2; i++) {
            float measured = readAveraged(adsHigh, i, true,  0.125, 2);
            // Aplicar calibración específica para cada canal
            adcChannels[i + 4] = applyCalibration(measured, rawPointsDiff[i], realPointsDiff[i], pointCountDiff[i]);
        }
    }
}

float readRawSingleEnded(uint8_t channel) {
    if (channel > 3) return 0.0;
    return readAveraged(adsLow, channel, false, 0.125, 4);
}

float readRawDifferential(uint8_t pairIndex) {
    if (pairIndex > 1) return 0.0;
    return readAveraged(adsHigh, pairIndex, true,  0.125, 4);
}