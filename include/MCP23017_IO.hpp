#ifndef MCP23017_IO_HPP
#define MCP23017_IO_HPP

#include <Arduino.h>
#include <Wire.h>

// Dirección I2C del MCP23017
#define MCP23017_ADDRESS 0x27

// Número máximo de puertos
#define MAX_RELAYS 8
#define MAX_INPUTS 8

class MCP23017_IO {
public:
    MCP23017_IO(uint8_t address = MCP23017_ADDRESS);
    
    // Configuración básica
    bool begin(uint8_t sda = 5, uint8_t scl = 4);
    
    // Configuración de puertos
    void setPortADirection(uint8_t dir);  // Puerto A como SALIDAS (relés)
    void setPortBDirection(uint8_t dir);  // Puerto B como ENTRADAS (digitales)
    void setPullupsA(uint8_t mask);       // Pull-ups para Puerto A (normalmente no)
    void setPullupsB(uint8_t mask);       // Pull-ups para Puerto B (SÍ para entradas)
    
    // Control de Relés (Puerto A - Salidas)
    void writeGPIOA(uint8_t value);      // ← CAMBIAR a GPIOA
    void writePinA(uint8_t pin, bool state); // ← CAMBIAR a PinA
    void setRelay(uint8_t relayNum, bool state);
    bool getRelayState(uint8_t relayNum);
    void toggleRelay(uint8_t relayNum);
    void setAllRelays(bool state);
    void testSequence(uint16_t delayMs = 500);
    String getRelaysStatus();
    
    // Lectura de Entradas (Puerto B - Entradas) ← CAMBIAR
    bool readPinB(uint8_t pin);          // ← CAMBIAR a PinB
    uint8_t readGPIOB();                 // ← CAMBIAR a GPIOB
    String getInputsStatus();
    void setDebounceTime(uint16_t ms);
    
    // Monitoreo continuo
    void startInputMonitoring();
    void stopInputMonitoring();
    bool isMonitoring();

    // Funciones adicionales
    uint8_t readAllInputs();
    bool readInput(uint8_t inputNum);
    void enableInputPullups(bool enable);

private:
    uint8_t _addr;
    bool initialized;
    uint8_t relayStates;      // Estado de relés (Puerto A)
    uint8_t lastInputStates;  // Último estado de entradas (Puerto B)
    uint16_t debounceTime;
    bool monitoringEnabled;
    unsigned long lastReadTime;

    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
};

extern MCP23017_IO ioController;

#endif