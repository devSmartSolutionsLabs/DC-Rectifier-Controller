#ifndef SENSORES_HPP
#define SENSORES_HPP

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include "Calibracion.hpp"

extern Adafruit_ADS1115 adsLow;
extern Adafruit_ADS1115 adsHigh;

// Variables compartidas
extern volatile int potPercentage;
extern volatile float potVoltage;
extern volatile float CurrentSensorVoltage;

// Constantes para el potenciómetro
extern const int maxPotVoltage;
extern const int minPotVoltage;
extern const int POT_READ_INTERVAL_MS;

// Funciones para lectura de sensores
float readAveraged(Adafruit_ADS1115 &ads, uint8_t pairIndex, float lsb_mV, int samples = 4);
void readCurrentSafe();
void readPotenciometerSafe();
float readRawCurrentUncalibrated();
float readRawVoltageUncalibrated();
float readRawVoltagePort0();
float readRawCurrentPort1();

#endif