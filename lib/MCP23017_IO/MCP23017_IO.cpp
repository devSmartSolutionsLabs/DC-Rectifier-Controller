#include <Arduino.h>
#include "MCP23017_IO.hpp"

// MCP23017 register addresses
#define IODIRA   0x00
#define IODIRB   0x01
#define GPPUA    0x0C
#define GPPUB    0x0D
#define GPIOA    0x12
#define GPIOB    0x13
#define OLATA    0x14
#define OLATB    0x15

MCP23017_IO ioController(MCP23017_ADDRESS);

MCP23017_IO::MCP23017_IO(uint8_t address) : 
    _addr(address), 
    initialized(false), 
    relayStates(0),
    lastInputStates(0),
    debounceTime(50),
    monitoringEnabled(false),
    lastReadTime(0) {}

bool MCP23017_IO::begin(uint8_t sda, uint8_t scl) {
    Wire.begin(sda, scl);
    
    pinMode(15,OUTPUT);
    digitalWrite(15,HIGH);
    // Verificar comunicación con el dispositivo
    Wire.beginTransmission(_addr);
    if (Wire.endTransmission() != 0) {
        Serial.printf("Error: No se pudo encontrar MCP23017 en dirección 0x%02X\n", _addr);
        initialized = false;
        return false;
    }
    
    // Configurar Puerto A como SALIDAS (0-7) para relés
    setPortADirection(0x00);  // Todos como salidas
    setPullupsA(0x00);        // Sin pull-ups en salidas
    
    // Configurar Puerto B como ENTRADAS (8-15) con pull-ups
    setPortBDirection(0xFF);  // Todos como entradas
    setPullupsB(0xFF);        // Pull-ups habilitados para todos
    
    writeGPIOA(0x00);         // Inicialmente todos los relés en LOW
    
    // Leer estado inicial de entradas
    lastInputStates = readGPIOB();
    
    initialized = true;
    Serial.printf("MCP23017 inicializado correctamente en dirección 0x%02X\n", _addr);
    Serial.println("Puerto A: Salidas para relés (0-7)");
    Serial.println("Puerto B: Entradas digitales (8-15) con pull-up");
    return true;
}

void MCP23017_IO::setPortADirection(uint8_t dir) { 
    writeRegister(IODIRA, dir); 
}

void MCP23017_IO::setPortBDirection(uint8_t dir) { 
    writeRegister(IODIRB, dir); 
}

void MCP23017_IO::setPullupsA(uint8_t mask) { 
    writeRegister(GPPUA, mask); 
}

void MCP23017_IO::setPullupsB(uint8_t mask) { 
    writeRegister(GPPUB, mask); 
}

// --- Control de Relés (Puerto A - Salidas) ---
void MCP23017_IO::writeGPIOA(uint8_t value) {
    writeRegister(GPIOA, value);
    relayStates = value;
}

void MCP23017_IO::writePinA(uint8_t pin, bool state) {
    if (pin > 7) return;
    
    uint8_t currentState = readRegister(GPIOA);
    if (state) {
        currentState |= (1 << pin);
    } else {
        currentState &= ~(1 << pin);
    }
    writeGPIOA(currentState);
}

void MCP23017_IO::setRelay(uint8_t relayNum, bool state) {
    if (!initialized || relayNum >= MAX_RELAYS) return;
    
    writePinA(relayNum, state);
    Serial.printf("Relé %d %s\n", relayNum + 1, state ? "ENCENDIDO" : "APAGADO");
}

bool MCP23017_IO::getRelayState(uint8_t relayNum) {
    if (!initialized || relayNum >= MAX_RELAYS) return false;
    return (relayStates & (1 << relayNum)) != 0;
}

void MCP23017_IO::toggleRelay(uint8_t relayNum) {
    if (!initialized || relayNum >= MAX_RELAYS) return;
    setRelay(relayNum, !getRelayState(relayNum));
}

void MCP23017_IO::setAllRelays(bool state) {
    if (!initialized) return;
    
    writeGPIOA(state ? 0xFF : 0x00);
    Serial.printf("Todos los relés %s\n", state ? "ENCENDIDOS" : "APAGADOS");
}

void MCP23017_IO::testSequence(uint16_t delayMs) {
    if (!initialized) return;
    
    Serial.println("Iniciando secuencia de prueba de relés...");
    
    // Apagar todos primero
    setAllRelays(false);
    delay(1000);
    
    // Encender uno por uno
    for (int i = 0; i < MAX_RELAYS; i++) {
        setRelay(i, true);
        delay(delayMs);
    }
    
    // Apagar uno por uno
    for (int i = 0; i < MAX_RELAYS; i++) {
        setRelay(i, false);
        delay(delayMs);
    }
    
    Serial.println("Secuencia de prueba completada");
}

String MCP23017_IO::getRelaysStatus() {
    String status = "Estado Relés: ";
    for (int i = 0; i < MAX_RELAYS; i++) {
        status += String(i + 1) + ":" + (getRelayState(i) ? "ON " : "OFF ");
    }
    return status;
}

// --- Lectura de Entradas (Puerto B - Entradas) ---
bool MCP23017_IO::readPinB(uint8_t pin) {
    if (!initialized || pin >= MAX_INPUTS) return false;
    uint8_t value = readRegister(GPIOB);
    return (value & (1 << pin)) == 0;  // LOW = activo (pull-up)
}

uint8_t MCP23017_IO::readGPIOB() {
    if (!initialized) return 0;
    return readRegister(GPIOB);
}

String MCP23017_IO::getInputsStatus() {
    String status = "Entradas: ";
    uint8_t inputs = readGPIOB();
    
    for (int i = 0; i < MAX_INPUTS; i++) {
        bool state = (inputs & (1 << i)) == 0;  // Invertido por pull-up
        status += String(i + 1) + ":" + (state ? "ACT " : "INACT ");
    }
    return status;
}

void MCP23017_IO::setDebounceTime(uint16_t ms) {
    debounceTime = ms;
    Serial.printf("Tiempo debounce configurado: %d ms\n", ms);
}

void MCP23017_IO::startInputMonitoring() {
    monitoringEnabled = true;
    Serial.println("Monitor de entradas ACTIVADO");
    Serial.println("Enviar 'INPUT_MONITOR' nuevamente para detener");
}

void MCP23017_IO::stopInputMonitoring() {
    monitoringEnabled = false;
    Serial.println("Monitor de entradas DESACTIVADO");
}

bool MCP23017_IO::isMonitoring() {
    return monitoringEnabled;
}

// --- Funciones adicionales ---
uint8_t MCP23017_IO::readAllInputs() {
    return readGPIOB();
}

bool MCP23017_IO::readInput(uint8_t inputNum) {
    return readPinB(inputNum);
}

void MCP23017_IO::enableInputPullups(bool enable) {
    setPullupsB(enable ? 0xFF : 0x00);
    Serial.printf("Pull-ups %s\n", enable ? "HABILITADOS" : "DESHABILITADOS");
}

// --- Funciones privadas de bajo nivel ---
void MCP23017_IO::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t MCP23017_IO::readRegister(uint8_t reg) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0xFF;
    Wire.requestFrom(_addr, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}