#pragma once
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdint.h>

/**
 * INA226 driver (ESP-IDF)
 * - Lee tensión de bus (mV) y shunt (µV)
 * - Usa promedio interno (AVG) del chip
 * - Opción de esperar "Conversion Ready" antes de leer
 * - I2C protegido opcionalmente con SemaphoreHandle_t
 */
class INA226 {
public:
    // Direcciones de registro
    enum Reg : uint8_t {
        REG_CONFIG       = 0x00,
        REG_SHUNT_V      = 0x01, // 2.5 µV/LSB (signed)
        REG_BUS_V        = 0x02, // 1.25 mV/LSB (unsigned)
        REG_POWER        = 0x03,
        REG_CURRENT      = 0x04,
        REG_CALIB        = 0x05,
        REG_MASK_ENABLE  = 0x06,
        REG_ALERT_LIMIT  = 0x07,
    };

    // Bits de configuración
    enum class Avg : uint8_t {
        AVG_1    = 0, // 1 sample
        AVG_4    = 1,
        AVG_16   = 2,
        AVG_64   = 3,
        AVG_128  = 4,
        AVG_256  = 5,
        AVG_512  = 6,
        AVG_1024 = 7
    };

    // Conversion time codes (ver hoja de datos)
    enum class Ct : uint8_t {
        CT_140us  = 0,
        CT_204us  = 1,
        CT_332us  = 2,
        CT_588us  = 3,
        CT_1100us = 4,
        CT_2116us = 5,
        CT_4156us = 6,
        CT_8244us = 7
    };

    // Modo (3 bits)
    enum class Mode : uint8_t {
        POWER_DOWN             = 0,
        SHUNT_TRIG             = 1,
        BUS_TRIG               = 2,
        SHUNT_BUS_TRIG         = 3,
        ADC_OFF                = 4,
        SHUNT_CONT             = 5,
        BUS_CONT               = 6,
        SHUNT_BUS_CONT         = 7, // recomendado
    };

    INA226(i2c_port_t port, uint8_t addr7, SemaphoreHandle_t i2c_mutex = nullptr);

    // Inicializa: aplica AVG, CT y modo. Devuelve true si OK.
    bool begin(Avg avg = Avg::AVG_16, Ct vbus_ct = Ct::CT_1100us, Ct vsh_ct = Ct::CT_1100us,
               Mode mode = Mode::SHUNT_BUS_CONT);

    // Cambia promedio / tiempos / modo en caliente (opcional)
    bool configure(Avg avg, Ct vbus_ct, Ct vsh_ct, Mode mode);

    // Lee bus (mV) y shunt (µV). Si wait_ready=true, espera CNVR antes de leer.
    bool readBusVoltage_mV(int32_t& out_mV, bool wait_ready = false, uint32_t timeout_us = 20000);
    bool readShuntMicroVolts(int32_t& out_uV, bool wait_ready = false, uint32_t timeout_us = 20000);

    // Utilidad: espera bit Conversion Ready (CNVR) en REG_MASK_ENABLE
    bool waitConversionReady(uint32_t timeout_us);

    // Acceso crudo (por si lo necesitas)
    bool readReg16(Reg reg, uint16_t& be_val);
    bool writeReg16(Reg reg, uint16_t be_val);

private:
    i2c_port_t       _port;
    uint8_t          _addr;         // 7-bit
    SemaphoreHandle_t _mtx;         // puede ser nullptr
    uint16_t         _config_cached = 0; // opcional, para debug/lecturas rápidas

    bool lock(uint32_t ms = 100);
    void unlock();
    bool i2cWrite16(Reg reg, uint16_t be_val);
    bool i2cRead16(Reg reg, uint16_t& be_val);

    // Helpers para armar CONFIG (big-endian)
    static inline uint16_t makeConfig(Avg avg, Ct vbus_ct, Ct vsh_ct, Mode mode) {
        // CONFIG: [RST:1][AVG:3][VBUSCT:3][VSHCT:3][MODE:3]
        uint16_t v = 0;
        v |= (static_cast<uint16_t>(avg)    & 0x7) << 9;
        v |= (static_cast<uint16_t>(vbus_ct)& 0x7) << 6;
        v |= (static_cast<uint16_t>(vsh_ct) & 0x7) << 3;
        v |= (static_cast<uint16_t>(mode)   & 0x7) << 0;
        return v;
    }
};
