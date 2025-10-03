#ifndef SENSORES_HPP
#define SENSORES_HPP

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include "Calibracion.hpp"
#include "GlobalVars.hpp"

extern Adafruit_ADS1115 adsLow;   // 0x48
extern Adafruit_ADS1115 adsHigh;  // 0x49
extern SemaphoreHandle_t i2cMutex;
const int POT_READ_INTERVAL_MS = 100;

extern volatile int potPercentage;
extern volatile float potVoltage;
extern volatile float CurrentSensorVoltage;
extern volatile float adcChannels[6];

// Funciones de sensores
void readPotenciometerSafe();
void readCurrentSafe();
void readAllChannelsSafe();
float readRawSingleEnded(uint8_t channel);
float readRawDifferential(uint8_t pairIndex);
float readAveraged(Adafruit_ADS1115 &ads, uint8_t channel, bool differential, float lsb_mV, int samples);


// Mutex helpers (implementados en main.cpp)
bool takeI2CMutex(uint32_t timeoutMs);
void giveI2CMutex();

#endif