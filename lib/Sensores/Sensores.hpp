#ifndef SENSORES_HPP
#define SENSORES_HPP

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include "Calibracion.hpp"
#include "GlobalVars.hpp"

extern Adafruit_ADS1115 adsLow;    // 0x48 - 4 canales single-ended
extern Adafruit_ADS1115 adsHigh;   // 0x49 - 2 canales diferenciales

// Variables para potenciómetro
extern volatile int potPercentage;
extern volatile float potVoltage;
extern volatile float CurrentSensorVoltage;

// Constantes
extern const int maxPotVoltage;
extern const int minPotVoltage;
extern const int POT_READ_INTERVAL_MS;

// Nuevas variables para múltiples canales
extern volatile float adcChannels[6]; // 4 single-ended + 2 differential

// Funciones para lectura de sensores
float readAveraged(Adafruit_ADS1115 &ads, uint8_t channel, bool differential, float lsb_mV, int samples = 4);
void readCurrentSafe();
void readPotenciometerSafe();
void readAllChannelsSafe();

// Funciones para lectura raw de cada canal
float readRawSingleEnded(uint8_t channel);
float readRawDifferential(uint8_t pairIndex);

#endif