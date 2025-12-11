#pragma once
#include <stdint.h>
#include "driver/i2c.h"
#include "freertos/semphr.h"

class ADS1115 {
public:
    // Direcciones I2C típicas: 0x48..0x4B
    explicit ADS1115(i2c_port_t port, uint8_t i2c_addr, SemaphoreHandle_t opt_mutex = nullptr);

    // ----- enums de configuración -----
    enum class Mux : uint8_t {
        DIFF_0_1 = 0b000, DIFF_0_3 = 0b001, DIFF_1_3 = 0b010, DIFF_2_3 = 0b011,
        AIN0_GND = 0b100, AIN1_GND = 0b101, AIN2_GND = 0b110, AIN3_GND = 0b111
    };
    enum class PGA : uint8_t { // Full-Scale Range
        FS_6V144 = 0b000, // ±6.144 V  (LSB 187.5µV)  NO exceder VDD
        FS_4V096 = 0b001, // ±4.096 V  (125.0µV)
        FS_2V048 = 0b010, // ±2.048 V  (62.5µV)
        FS_1V024 = 0b011, // ±1.024 V  (31.25µV)
        FS_0V512 = 0b100, // ±0.512 V  (15.625µV)
        FS_0V256 = 0b101  // ±0.256 V  (7.8125µV)  MÁXIMA ganancia
    };
    enum class DataRate : uint8_t {
        SPS_8=0b000, SPS_16=0b001, SPS_32=0b010, SPS_64=0b011,
        SPS_128=0b100, SPS_250=0b101, SPS_475=0b110, SPS_860=0b111
    };
    enum class Mode : uint8_t { CONTINUOUS=0, SINGLE_SHOT=1 };

    // ----- init opcional (solo por simetría) -----
    bool begin(); // no toca el bus, solo valida dirección

    // ----- API principal -----
    bool singleShot(Mux mux, PGA pga, DataRate dr, int16_t& raw);                 // bloqueante con polling OS
    bool singleShotMV(Mux mux, PGA pga, DataRate dr, float& mv);                  // como arriba pero en mV
    bool startContinuous(Mux mux, PGA pga, DataRate dr);                          // configura y deja corriendo
    bool readContinuous(int16_t& raw);                                            // lee último valor
    bool stopContinuous();                                                        // cambia a single-shot inerte

    // Conversión helper
    static float lsb_uV(PGA pga);     // tamaño de LSB en microvoltios
    static float fsr_mV(PGA pga);     // FSR en mV

private:
    // Registros
    static constexpr uint8_t REG_CONVERSION = 0x00;
    static constexpr uint8_t REG_CONFIG     = 0x01;
    static constexpr uint8_t REG_LO_THRESH  = 0x02;
    static constexpr uint8_t REG_HI_THRESH  = 0x03;

    i2c_port_t port_;
    uint8_t addr_;
    SemaphoreHandle_t mtx_; // puede ser null

    // Bajo nivel
    bool writeConfig(uint16_t cfg);
    bool readConfig(uint16_t& cfg);
    bool readConversion(int16_t& raw);
    bool waitOSReady(uint16_t cfg);   // timeout derivado de DR

    // Utilidades
    static uint16_t makeConfig(Mux mux, PGA pga, DataRate dr, Mode mode, bool start);
    static uint32_t drTimeoutMs(DataRate dr); // tiempo de conversión teórico + margen

    // Mutex RAII
    struct Lock {
        SemaphoreHandle_t m;
        bool locked=false;
        Lock(SemaphoreHandle_t mm): m(mm){ if(m) locked = xSemaphoreTake(m, pdMS_TO_TICKS(100))==pdTRUE; }
        ~Lock(){ if(m && locked) xSemaphoreGive(m); }
    };
};
