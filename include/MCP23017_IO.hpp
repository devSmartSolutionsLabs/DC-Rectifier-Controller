#pragma once
#include <cstdint>
#include <cstring>
#include <string>

extern "C" {
  #include "driver/i2c_master.h"
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "esp_timer.h"
}

// ======= Registros MCP23017 =======
#ifndef MCP23017_ADDRESS
#define MCP23017_ADDRESS 0x27
#endif

#define IODIRA  0x00
#define IODIRB  0x01
#define IPOLA   0x02
#define IPOLB   0x03
#define GPINTENA 0x04
#define GPINTENB 0x05
#define DEFVALA 0x06
#define DEFVALB 0x07
#define INTCONA 0x08
#define INTCONB 0x09
#define IOCON   0x0A
#define GPPUA   0x0C
#define GPPUB   0x0D
#define INTFA   0x0E
#define INTFB   0x0F
#define INTCAPA 0x10
#define INTCAPB 0x11
#define GPIOA   0x12
#define GPIOB   0x13
#define OLATA   0x14
#define OLATB   0x15

// Máximos del diseño
#define MAX_RELAYS   8
#define MAX_INPUTS   8

// ==== Variables/mutex externos (siguen tu proyecto) ====
extern bool verboseLog;                  // definido en GlobalVars.cpp
extern "C" bool takeI2CMutex(uint32_t ms, const char* holder);
extern "C" void giveI2CMutex(const char* holder);

// =======================================================

class MCP23017_IO {
public:
    explicit MCP23017_IO(uint8_t address = MCP23017_ADDRESS);

    // --- Inicialización ---
    // Opción A (recomendada): pasar bus ya creado por tu app (compartido)
    bool begin(i2c_master_bus_handle_t bus, uint8_t address = MCP23017_ADDRESS);

    // Opción B: crear bus interno (si quieres mantener firma previa)
    // Nota: mejor usa begin(bus, addr). Esta crea un bus propio a 100kHz.
    bool begin(uint8_t sdaPin, uint8_t sclPin, uint8_t address = MCP23017_ADDRESS);

    bool isInitialized();

    // --- Lectura/Escritura segura ---
    uint8_t readRegisterSafe(uint8_t reg);
    bool    writeRegisterSafe(uint8_t reg, uint8_t value);

    // --- Lectura/Escritura simple ---
    uint8_t readRegister(uint8_t reg);
    void    writeRegister(uint8_t reg, uint8_t value);

    // --- Configuración de puertos ---
    void setPortADirection(uint8_t dir);
    void setPortBDirection(uint8_t dir);
    void setPullupsA(uint8_t mask);
    void setPullupsB(uint8_t mask);

    // --- Relés (Puerto A) ---
    void writeGPIOA(uint8_t value);
    void writePinA(uint8_t pin, bool state);
    void setRelay(uint8_t relayNum, bool state);
    bool getRelayState(uint8_t relayNum);
    void toggleRelay(uint8_t relayNum);
    void setAllRelays(bool state);
    std::string getRelaysStatus();

    // --- Entradas (Puerto B) ---
    uint8_t readGPIOB();
    bool readPinB(uint8_t pin);
    uint8_t readAllInputs();
    bool readInput(uint8_t inputNum);
    std::string getInputsStatus();
    void enableInputPullups(bool enable);

    // --- Herramientas/Debug ---
    void debugInputs();
    void setDebounceTime(uint16_t ms);
    void startInputMonitoring();
    void stopInputMonitoring();
    bool isMonitoring();
    void testSequence(uint16_t delayMs);

private:
    // Helpers I2C (v2)
    bool devWriteReg(uint8_t reg, const uint8_t* data, size_t len);
    bool devReadReg(uint8_t reg, uint8_t* data, size_t len);

private:
    uint8_t  _addr;
    bool     initialized;

    // Estado
    uint8_t  relayStates;
    uint8_t  lastInputStates;
    uint16_t debounceTime;
    bool     monitoringEnabled;
    uint64_t lastReadTime;

    // I2C-IDF v2
    i2c_master_bus_handle_t _bus = nullptr;
    i2c_master_dev_handle_t _dev = nullptr;
    bool owns_bus = false; // si creamos bus interno en begin(sda,scl)
};

// ==== Instancia global, como en tu proyecto original ====
extern MCP23017_IO ioController;
