#include "MCP23017_IO.hpp"
#include "GlobalVars.hpp" // Para i2cMutex

// Instancia global
MCP23017_IO ioController(MCP23017_ADDRESS);

// Constructor
MCP23017_IO::MCP23017_IO(uint8_t address) :
    _addr(address),
    initialized(false),
    relayStates(0),
    lastInputStates(0),
    debounceTime(50),
    monitoringEnabled(false),
    lastReadTime(0)
{}

// --- Inicialización ---
bool MCP23017_IO::begin(uint8_t sda, uint8_t scl) {
    Serial.printf("[MCP23017] Inicializando en SDA:%d, SCL:%d, Addr:0x%02X\n", sda, scl, _addr);
    pinMode(15,OUTPUT);
    pinMode(41,OUTPUT);
    digitalWrite(15,HIGH);
    digitalWrite(41,HIGH);
    delay(200);

    Wire.begin(sda, scl);
    Wire.setTimeout(250);
    delay(200);

    // Verificación de dispositivo
    bool found = false;
    for (int i = 0; i < 3; i++) {
        Wire.beginTransmission(_addr);
        uint8_t error = Wire.endTransmission();
        if (error == 0) {
            found = true;
            Serial.printf("✅ [MCP23017] Encontrado en intento %d\n", i+1);
            break;
        }
        delay(100);
    }
    if (!found) {
        Serial.println("❌ [MCP23017] No responde");
        return false;
    }

    // Configuración con mutex
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        setPortADirection(0x00);  // Relés = salida
        setPullupsA(0x00);
        writeGPIOA(0x00);

        setPortBDirection(0xFF);  // Entradas = entrada
        setPullupsB(0xFF);        // Pull-ups activados

        lastInputStates = readGPIOB();

        xSemaphoreGive(i2cMutex);
    } else {
        Serial.println("❌ Timeout mutex en configuración");
        return false;
    }

    initialized = true;
    Serial.println("✅ [MCP23017] Inicializado correctamente");
    return true;
}

// --- Estado ---
bool MCP23017_IO::isInitialized() {
    return initialized;
}

// --- Lectura/escritura I2C segura ---
uint8_t MCP23017_IO::readRegister(uint8_t reg) {
    uint8_t val = 0xFF;
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return 0xFF;
    Wire.requestFrom(_addr, (uint8_t)1);
    if (Wire.available()) val = Wire.read();
    return val;
}

uint8_t MCP23017_IO::readRegisterSafe(uint8_t reg) {
    uint8_t result = 0xFF;
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        Serial.println("❌ Timeout mutex en readRegisterSafe");
        return result;
    }
    result = readRegister(reg);
    xSemaphoreGive(i2cMutex);
    return result;
}

void MCP23017_IO::writeRegister(uint8_t reg, uint8_t value) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        Serial.println("❌ Timeout mutex en writeRegister");
        return;
    }
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
    xSemaphoreGive(i2cMutex);
}

// --- Configuración de puertos ---
void MCP23017_IO::setPortADirection(uint8_t dir) { writeRegister(IODIRA, dir); }
void MCP23017_IO::setPortBDirection(uint8_t dir) { writeRegister(IODIRB, dir); }
void MCP23017_IO::setPullupsA(uint8_t mask) { writeRegister(GPPUA, mask); }
void MCP23017_IO::setPullupsB(uint8_t mask) { writeRegister(GPPUB, mask); }

// --- Relés (Puerto A) ---
void MCP23017_IO::writeGPIOA(uint8_t value) {
    writeRegister(GPIOA, value);
    relayStates = value;
}

void MCP23017_IO::writePinA(uint8_t pin, bool state) {
    if (!initialized || pin > 7) return;
    uint8_t curr = readRegisterSafe(GPIOA);
    if (state) curr |= (1 << pin);
    else curr &= ~(1 << pin);
    writeGPIOA(curr);
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
    setRelay(relayNum, !getRelayState(relayNum));
}

void MCP23017_IO::setAllRelays(bool state) {
    if (!initialized) return;
    writeGPIOA(state ? 0xFF : 0x00);
    Serial.printf("Todos los relés %s\n", state ? "ENCENDIDOS" : "APAGADOS");
}

String MCP23017_IO::getRelaysStatus() {
    String s = "Estado Relés: ";
    for (int i = 0; i < MAX_RELAYS; i++) s += String(i+1) + ":" + (getRelayState(i) ? "ON " : "OFF ");
    return s;
}

// --- Entradas (Puerto B) ---
uint8_t MCP23017_IO::readGPIOB() { return readRegisterSafe(GPIOB); }

bool MCP23017_IO::readPinB(uint8_t pin) {
    if (!initialized || pin >= MAX_INPUTS) return false;
    return (readGPIOB() & (1 << pin)) == 0;
}

uint8_t MCP23017_IO::readAllInputs() { return readGPIOB(); }

bool MCP23017_IO::readInput(uint8_t inputNum) { return readPinB(inputNum); }

String MCP23017_IO::getInputsStatus() {
    String s = "Entradas: ";
    uint8_t vals = readGPIOB();
    for (int i = 0; i < MAX_INPUTS; i++) {
        bool state = (vals & (1 << i)) == 0;
        s += String(i+1) + ":" + (state ? "ACT " : "INACT ");
    }
    return s;
}

void MCP23017_IO::enableInputPullups(bool enable) {
    setPullupsB(enable ? 0xFF : 0x00);
    Serial.printf("Pull-ups %s\n", enable ? "HABILITADOS" : "DESHABILITADOS");
}

// --- Debug ---
void MCP23017_IO::debugInputs() {
    if (!initialized) {
        Serial.println("[MCP DEBUG] ❌ MCP23017 no inicializado");
        return;
    }

    uint8_t vals = readGPIOB();
    Serial.println("=== [MCP23017 DEBUG] ===");
    Serial.print("GPIOB: ");
    for (int i = 7; i >= 0; i--) Serial.print((vals >> i) & 1);
    Serial.print(" | Hex: 0x"); Serial.println(vals, HEX);

    for (int i = 0; i < MAX_INPUTS; i++) {
        bool state = (vals & (1 << i)) == 0;
        Serial.printf("Pin B%d: %s\n", i, state ? "PRESIONADO" : "LIBRE");
    }
    Serial.println("=========================");
}

// --- Configuración avanzada ---
void MCP23017_IO::setDebounceTime(uint16_t ms) { debounceTime = ms; }
void MCP23017_IO::startInputMonitoring() { monitoringEnabled = true; }
void MCP23017_IO::stopInputMonitoring() { monitoringEnabled = false; }
bool MCP23017_IO::isMonitoring() { return monitoringEnabled; }

void MCP23017_IO::testSequence(uint16_t delayMs) {
    Serial.println("Iniciando secuencia de prueba de relés...");
    setAllRelays(false);
    delay(500);
    for (int i = 0; i < MAX_RELAYS; i++) {
        setRelay(i, true);
        delay(delayMs);
    }
    for (int i = 0; i < MAX_RELAYS; i++) {
        setRelay(i, false);
        delay(delayMs);
    }
    Serial.println("Secuencia completada");
}
