#include "MCP23017_IO.hpp"
#include "GlobalVars.hpp" // Para i2cMutex
#include "mutexDebug.hpp" // Para takeI2CMutex y giveI2CMutex

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

// --- writeRegisterSafe (debe estar primero) ---
bool MCP23017_IO::writeRegisterSafe(uint8_t reg, uint8_t value) {
    if (!takeI2CMutex(50,"MCP_Write")) { // ✅ Timeout más corto
        if (verboseLog) Serial.printf("❌ Timeout mutex en writeRegisterSafe 0x%02X\n", reg);
        return false;
    }
    
    bool success = false;
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(value);
    uint8_t error = Wire.endTransmission();
    
    giveI2CMutex(); // ✅ Liberar inmediatamente
    
    if (error == 0) {
        success = true;
    } else {
        Serial.printf("❌ Error I2C 0x%02X en write 0x%02X\n", error, reg);
    }
    
    return success;
}

// --- Inicialización ---
bool MCP23017_IO::begin(uint8_t sdaPin, uint8_t sclPin, uint8_t address) {
    // Si se proporciona una nueva dirección, actualizarla
    if (address != _addr) {
        _addr = address;
    }
    
    Serial.printf("[MCP23017] Inicializando en SDA:%d, SCL:%d, Addr:0x%02X\n", 
                  sdaPin, sclPin, _addr);
    
    // Verificar si el mutex existe
    if (i2cMutex == nullptr) {
        Serial.println("❌ [MCP23017] ERROR: i2cMutex es NULL");
        return false;
    }
    
    Wire.begin(sdaPin, sclPin);
    Wire.setClock(100000); // 100kHz para mayor estabilidad
    
    // Intentar detectar el dispositivo
    int intentos = 0;
    const int maxIntentos = 5;
    bool deviceFound = false;
    
    while (intentos < maxIntentos) {
        intentos++;
        
        // USAR MUTEX SOLO para la detección
        if (takeI2CMutex(500,"MCP_begin")) {
            Wire.beginTransmission(_addr);
            uint8_t error = Wire.endTransmission();
            giveI2CMutex(); // ✅ LIBERAR INMEDIATAMENTE
            
            if (error == 0) {
                if (verboseLog) Serial.printf("✅ [MCP23017] Encontrado en intento %d\n", intentos);
                deviceFound = true;
                break;
            } else {
                if (verboseLog) Serial.printf("⚠️ [MCP23017] Intento %d falló, error: %d\n", intentos, error);
            }
        } else {
            if (verboseLog) Serial.printf("❌ [MCP23017] Timeout mutex en intento %d\n", intentos);
        }
        
        delay(100);
    }
    
    if (!deviceFound) {
        Serial.println("❌ [MCP23017] No se pudo encontrar el dispositivo");
        return false;
    }
    
    // ✅ CONFIGURACIÓN SIN MUTEX - Usar I2C directamente para evitar bloqueos
    Serial.println("🔧 Configurando MCP23017...");
    
    bool success = true;
    
    // Configurar sin mutex - las funciones de Adafruit manejan su propia sincronización
    Wire.beginTransmission(_addr);
    Wire.write(IODIRA);
    Wire.write(0x00);  // Puerto A como salidas
    if (Wire.endTransmission() != 0) {
        Serial.println("❌ Error configurando IODIRA");
        success = false;
    }
    
    Wire.beginTransmission(_addr);
    Wire.write(GPIOA);
    Wire.write(0x00);   // Puerto A en LOW
    if (Wire.endTransmission() != 0) {
        Serial.println("❌ Error configurando GPIOA");
        success = false;
    }
    
    Wire.beginTransmission(_addr);
    Wire.write(IODIRB);
    Wire.write(0xFF);  // Puerto B como entradas
    if (Wire.endTransmission() != 0) {
        Serial.println("❌ Error configurando IODIRB");
        success = false;
    }
    
    Wire.beginTransmission(_addr);
    Wire.write(GPPUB);
    Wire.write(0xFF);   // Pull-ups habilitados
    if (Wire.endTransmission() != 0) {
        Serial.println("❌ Error configurando GPPUB");
        success = false;
    }
    
    if (success) {
        Serial.printf("✅ [MCP23017] Inicializado correctamente\n");
        initialized = true;
    } else {
        Serial.println("❌ [MCP23017] Error en configuración");
    }
    
    return success;
}

// --- Lectura/escritura I2C ---
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
    
    if (!takeI2CMutex(15,"MCP_Read")) { // ⚡ Reducido a 15ms
        static uint32_t lastLog = 0;
        if (millis() - lastLog > 2000) { // Log cada 2 segundos máximo
            Serial.printf("❌ Timeout mutex en readRegisterSafe 0x%02X\n", reg);
            lastLog = millis();
        }
        return result;
    }
    
    // Operación I2C ultra rápida
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    if (Wire.endTransmission() == 0) {
        Wire.requestFrom(_addr, (uint8_t)1);
        if (Wire.available()) {
            result = Wire.read();
        }
    }
    
    giveI2CMutex();
    return result;
}

void MCP23017_IO::writeRegister(uint8_t reg, uint8_t value) {
    writeRegisterSafe(reg, value); // Reutilizar función segura
}

// --- Estado ---
bool MCP23017_IO::isInitialized() {
    return initialized;
}

// --- Configuración de puertos ---
void MCP23017_IO::setPortADirection(uint8_t dir) { writeRegister(IODIRA, dir); }
void MCP23017_IO::setPortBDirection(uint8_t dir) { writeRegister(IODIRB, dir); }
void MCP23017_IO::setPullupsA(uint8_t mask) { writeRegister(GPPUA, mask); }
void MCP23017_IO::setPullupsB(uint8_t mask) { writeRegister(GPPUB, mask); }

// --- Relés (Puerto A) ---
void MCP23017_IO::writeGPIOA(uint8_t value) {
    if (writeRegisterSafe(GPIOA, value)) {
        relayStates = value;
    }
}


void MCP23017_IO::writePinA(uint8_t pin, bool state) {
    if (!initialized || pin > 7) return;
    
    // Leer estado actual
    uint8_t currentState = readRegisterSafe(GPIOA);
    if (currentState == 0xFF) { // Error en lectura
        Serial.printf("❌ Error leyendo GPIOA para pin %d\n", pin);
        return;
    }
    
    // Modificar bit específico
    if (state) {
        currentState |= (1 << pin);
    } else {
        currentState &= ~(1 << pin);
    }
    
    // Escribir nuevo estado
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
uint8_t MCP23017_IO::readGPIOB() { 
    return readRegisterSafe(GPIOB); 
}

bool MCP23017_IO::readPinB(uint8_t pin) {
    if (!initialized || pin >= MAX_INPUTS) return false;
    uint8_t portValue = readGPIOB();
    return (portValue & (1 << pin)) == 0; // Lógica invertida para entradas
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
        Serial.println("[MCP DEBUG] ❌ No inicializado");
        return;
    }

    uint8_t vals;
    if (!tryReadInputs(&vals)) {
        Serial.println("[MCP DEBUG] ❌ Timeout en lectura");
        return;
    }

    Serial.println("=== [MCP23017 DEBUG] ===");
    Serial.printf("GPIOB: 0x%02X | Bin: ", vals);
    for (int i = 7; i >= 0; i--) Serial.print((vals >> i) & 1);
    Serial.println();
    
    for (int i = 0; i < MAX_INPUTS; i++) {
        bool state = (vals & (1 << i)) == 0;
        Serial.printf("Entrada %d: %s\n", i+1, state ? "ACTIVA" : "INACTIVA");
    }
    Serial.println("========================");
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