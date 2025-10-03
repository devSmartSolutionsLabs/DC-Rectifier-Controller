#ifndef SENSORES_HPP
#define SENSORES_HPP

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include "GlobalVars.hpp"  // ✅ Ahora incluye todas las constantes
#include "mutexDebug.hpp"  // ✅ Para las funciones de mutex

class Sensores {
private:
    Adafruit_ADS1115 adsLow;
    Adafruit_ADS1115 adsHigh;
    
    // Variables de calibración
    float offsetCurrentLow[NUM_DEVICES];
    float offsetCurrentHigh[NUM_DEVICES];
    float gainCurrentLow[NUM_DEVICES];
    float gainCurrentHigh[NUM_DEVICES];
    
    // Variables de medición
    float current[NUM_DEVICES];
    float potPercentageLocal;  // Variable local
    float voltage;
    
    // Métodos privados
    float readAveraged(Adafruit_ADS1115& ads, uint8_t channel, bool differential, float gain, int samples);
    float readSingle(Adafruit_ADS1115& ads, uint8_t channel, bool differential, float gain); // ⚡ NUEVO
    float applyCalibration(float rawValue, float offset, float gain);
    
public:
    Sensores();
    bool begin();
    
    // ⚡ MÉTODOS OPTIMIZADOS NUEVOS
    bool readAllSensors(float* results = nullptr, uint8_t numChannels = 0);
    float readADSChannel(uint8_t adsIndex, uint8_t channel, bool differential = false);
    
    // Métodos de lectura (existentes - compatibilidad)
    void readPotenciometer();
    void readCurrent();
    void readAllChannels();
    
    // Getters
    float getCurrent(uint8_t device) const;
    float getPotPercentage() const;
    float getVoltage() const;
    
    // Métodos para acceso externo a ADS
    Adafruit_ADS1115& getADSLow() { return adsLow; }
    Adafruit_ADS1115& getADSHigh() { return adsHigh; }
    
    // Wrapper para función legacy
    float readAveragedWrapper(Adafruit_ADS1115& ads, uint8_t channel, bool differential, float gain, int samples) {
        return readAveraged(ads, channel, differential, gain, samples);
    }
    
    // Métodos de calibración
    void calibrateCurrent(uint8_t device, bool highRange, float knownCurrent);
    void saveCalibration();
    void loadCalibration();
};

extern Sensores sensores;

#endif