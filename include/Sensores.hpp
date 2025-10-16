#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"

// Forward minimal para mantener compatibilidad de firmas
class Adafruit_ADS1115 {};

// Direcciones I2C por defecto
#ifndef ADS_LOW_ADDR
#define ADS_LOW_ADDR  0x48
#endif
#ifndef ADS_HIGH_ADDR
#define ADS_HIGH_ADDR 0x49
#endif

class Sensores {
public:
    Sensores();

    // === Inicialización ===
    bool begin(int sda_pin = 5, int scl_pin = 4, uint32_t clk_hz = 100000);

    // === Lecturas de alto nivel (compatibles con tu código) ===
    void  readAllSensors(float* out, size_t n);
    float getVoltage();

    // Mantengo firma “Arduino” para compatibilidad (internamente se ignora el objeto)
    float readAveragedWrapper(Adafruit_ADS1115& ads, uint8_t channel, bool differential,
                              float multiplier_mV, uint8_t samples);

    // Nueva versión explícita por dirección
    float readAveraged(uint8_t i2c_addr, uint8_t channel, bool differential,
                       float multiplier_mV, uint8_t samples);

    // === Accesores dummy para compatibilidad (tu código los llama) ===
    Adafruit_ADS1115& getADSLow();
    Adafruit_ADS1115& getADSHigh();

    // === Lecturas RAW (si las usas en debug) ===
    int16_t readSingleEndedRaw(uint8_t i2c_addr, uint8_t channel);
    int16_t readDifferential01Raw(uint8_t i2c_addr);
    int16_t readDifferential23Raw(uint8_t i2c_addr);

private:
    // Helpers I2C (ESP-IDF)
    bool writeReg16(uint8_t i2c_addr, uint8_t reg, uint16_t value_be);
    bool readReg16 (uint8_t i2c_addr, uint8_t reg, uint16_t* out_be);
    bool ensureDev(uint8_t addr, i2c_master_dev_handle_t& out);

    // ADS1115 helpers
    bool  startSingleShot(uint8_t addr, uint16_t config_be);
    bool  waitConversionReady(uint8_t addr, uint32_t timeout_ms);
    float convertRawToVoltage(int16_t raw, float mv_per_bit);

private:
    static constexpr const char* TAG = "SENSORES";

    // Bus maestro y devices
    i2c_master_bus_handle_t   bus_   = nullptr;
    i2c_master_dev_handle_t   dev48_ = nullptr;
    i2c_master_dev_handle_t   dev49_ = nullptr;

    // Dummies para compatibilidad de interfaz
    Adafruit_ADS1115 dummyLow_;
    Adafruit_ADS1115 dummyHigh_;

    // Cache de último voltaje leído (potenciómetro)
    float lastVoltage_ = 0.0f;

    // Pines configurados
    int sda_ = 5;
    int scl_ = 4;
    uint32_t clk_ = 100000;
};
