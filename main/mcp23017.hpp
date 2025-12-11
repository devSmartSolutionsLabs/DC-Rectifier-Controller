#pragma once
#include "driver/i2c.h"
#include "esp_log.h"

class MCP23017 {
private:
    i2c_port_t m_port;
    uint8_t m_addr;
    static const char* TAG;

    // Registros MCP23017
    enum Registers {
        IODIRA = 0x00, IODIRB = 0x01,
        IPOLA = 0x02, IPOLB = 0x03,
        GPINTENA = 0x04, GPINTENB = 0x05,
        DEFVALA = 0x06, DEFVALB = 0x07,
        INTCONA = 0x08, INTCONB = 0x09,
        IOCONA = 0x0A, IOCONB = 0x0B,
        GPPUA = 0x0C, GPPUB = 0x0D,
        INTFA = 0x0E, INTFB = 0x0F,
        INTCAPA = 0x10, INTCAPB = 0x11,
        GPIOA = 0x12, GPIOB = 0x13,
        OLATA = 0x14, OLATB = 0x15
    };

    bool write_register(uint8_t reg, uint8_t value);
    bool read_register(uint8_t reg, uint8_t& value);

public:
    MCP23017(i2c_port_t port, uint8_t addr = 0x20);
    ~MCP23017();

    bool begin();
    
    // Configuración de pines
    bool pin_mode(uint8_t pin, uint8_t mode); // 0-15, mode: 0=OUTPUT, 1=INPUT
    bool pin_pullup(uint8_t pin, bool enable);
    
    // Escritura/Lectura
    bool digital_write(uint8_t pin, bool level);
    bool digital_read(uint8_t pin, bool& level);
    
    // Escritura de puertos completos
    bool write_port_a(uint8_t value);
    bool write_port_b(uint8_t value);
    bool read_port_a(uint8_t& value);
    bool read_port_b(uint8_t& value);
    
    // Utilidades
    bool test_connection();
};