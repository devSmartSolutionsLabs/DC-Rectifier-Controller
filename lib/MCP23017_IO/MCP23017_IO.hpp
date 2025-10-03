#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "GlobalVars.hpp"

// Dirección por defecto del MCP23017
#define MCP23017_ADDRESS 0x27

// Máximos de pines
#define MAX_RELAYS 8
#define MAX_INPUTS 8

// Registros del MCP23017
#define IODIRA 0x00
#define IODIRB 0x01
#define GPPUA  0x0C
#define GPPUB  0x0D
#define GPIOA  0x12
#define GPIOB  0x13
#define OLATA  0x14
#define OLATB  0x15

class MCP23017_IO {
public:
    MCP23017_IO(uint8_t address = MCP23017_ADDRESS);

    // Inicialización
    bool begin(uint8_t sda, uint8_t scl, uint8_t address = MCP23017_ADDRESS);
    bool isInitialized();

    // Relés (Puerto A)
    void setRelay(uint8_t relayNum, bool state);
    bool getRelayState(uint8_t relayNum);
    void toggleRelay(uint8_t relayNum);
    void setAllRelays(bool state);
    String getRelaysStatus();
    void testSequence(uint16_t delayMs = 200);

    // Entradas (Puerto B)
    bool readPinB(uint8_t pin);
    uint8_t readAllInputs();
    bool readInput(uint8_t inputNum);
    String getInputsStatus();
    void enableInputPullups(bool enable);
    
    // Debug
    void debugInputs();

    // Configuración
    void setDebounceTime(uint16_t ms);
    void startInputMonitoring();
    void stopInputMonitoring();
    bool isMonitoring();
    uint8_t readGPIOB();

private:
    uint8_t _addr;
    bool initialized;
    uint8_t relayStates;
    uint8_t lastInputStates;
    uint16_t debounceTime;
    bool monitoringEnabled;
    uint32_t lastReadTime;

    // Métodos internos I2C
    uint8_t readRegister(uint8_t reg);
    uint8_t readRegisterSafe(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);
    bool writeRegisterSafe(uint8_t reg, uint8_t value); // ✅ AGREGAR esta función

    // Configuración de puertos
    void setPortADirection(uint8_t dir);
    void setPortBDirection(uint8_t dir);
    void setPullupsA(uint8_t mask);
    void setPullupsB(uint8_t mask);
    void writeGPIOA(uint8_t value);
    void writePinA(uint8_t pin, bool state);

    bool tryReadInputs(uint8_t* result);
};
 
// Instancia global
extern MCP23017_IO ioController;