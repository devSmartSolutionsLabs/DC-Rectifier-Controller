#include <Arduino.h>
#include "TTL.hpp"
#include "Calibracion.hpp"
#include "Sensores.hpp"
#include "MCP23017_IO.hpp"
#include "GlobalVars.hpp"   // verboseLog, systemStarted, etc.

// =====================================================
// VARIABLES DE CALIBRACIÓN
// =====================================================
float offsetVoltage = 0.0;
float offsetCurrent = 0.0;

int pointCountSingle[4] = {0, 0, 0, 0};
int pointCountDiff[2]   = {0, 0};

float rawPointsSingle[4][10];
float realPointsSingle[4][10];
float rawPointsDiff[2][10];
float realPointsDiff[2][10];

// Estado de calibración
bool calibrationMode           = false;
int currentCalibrationSensor   = 0;
int currentCalibrationChannel  = -1;
int currentCalibrationPort     = -1;

// =====================================================
// FUNCIONES DE CALIBRACIÓN
// =====================================================
void saveCalibration() {
    Serial.println("💾 Calibración guardada (función temporal)");
}

void resetCalibration(int sensorType, int channel) {
    if (sensorType == 1 && channel >= 0 && channel < 4) {
        pointCountSingle[channel] = 0;
        Serial.printf("✅ Calibración single-ended canal %d reseteada\n", channel);
    } 
    else if (sensorType == 2 && channel >= 0 && channel < 2) {
        pointCountDiff[channel] = 0;
        Serial.printf("✅ Calibración diferencial canal %d reseteada\n", channel);
    } 
    else {
        Serial.println("❌ Error: Tipo de sensor o canal inválido");
    }
}

void displayCalibrationHelp() {
    Serial.println("=== MODO CALIBRACIÓN ===");
    Serial.println("CAL_SINGLE_X    - Calibrar canal single-ended X (0-3)");
    Serial.println("CAL_DIFF_X      - Calibrar canal diferencial X (0-1)");
    Serial.println("VOLT            - Calibrar voltímetro (canal 0 single)");
    Serial.println("AMP             - Calibrar amperímetro (canal 0 diferencial)");
    Serial.println("SAVE            - Guardar calibración");
    Serial.println("CANCEL          - Cancelar calibración");
    Serial.println("CAL_STATUS      - Ver calibración actual");
    Serial.println("RESET_SINGLE_X  - Resetear calibración single canal X");
    Serial.println("RESET_DIFF_X    - Resetear calibración diferencial canal X");
    Serial.println("ZERO            - Calibrar offset a cero");
    Serial.println("HELP            - Mostrar esta ayuda");
}

void displayPortSelection() {
    Serial.println("Seleccione puerto diferencial:");
    Serial.println("0 - Diferencial 0-1");
    Serial.println("1 - Diferencial 2-3");
    Serial.println("Envie el número del puerto (0 o 1)");
}

float readCurrentRawValueImmediate() {
    // ⚠️ Temporal: devolver 0 hasta integrar con Sensores
    return 0.0f;
}

// =====================================================
// PROCESAMIENTO DE PUNTOS DE CALIBRACIÓN
// =====================================================
void processCalibrationPoint(String command) {
    if (command == "ZERO") {
        float sumRaw = 0;
        const int readings = 10;
        for (int i = 0; i < readings; i++) {
            sumRaw += readCurrentRawValueImmediate();
            delay(10);
        }

        float currentRaw = sumRaw / readings;
        if (currentCalibrationSensor == 1) {
            offsetVoltage = currentRaw;
            Serial.printf("Offset single canal %d ajustado: %.4f mV\n", 
                          currentCalibrationChannel, offsetVoltage);
        } else {
            offsetCurrent = currentRaw;
            Serial.printf("Offset diferencial canal %d ajustado: %.4f mV\n", 
                          currentCalibrationChannel, offsetCurrent);
        }
        return;
    }

    float realVal = command.toFloat();
    float rawVal  = readCurrentRawValueImmediate();

    if (currentCalibrationSensor == 1) {
        int idx = pointCountSingle[currentCalibrationChannel];
        if (idx < MAX_POINTS) {
            rawPointsSingle[currentCalibrationChannel][idx]  = rawVal;
            realPointsSingle[currentCalibrationChannel][idx] = realVal;
            pointCountSingle[currentCalibrationChannel]++;
            Serial.printf("Punto %d: %.2f V -> %.2f mV\n", idx+1, realVal, rawVal);
        } else {
            Serial.println("❌ Máximo de puntos alcanzado para este canal");
        }
    } 
    else {
        int idx = pointCountDiff[currentCalibrationChannel];
        if (idx < MAX_POINTS) {
            rawPointsDiff[currentCalibrationChannel][idx]  = rawVal;
            realPointsDiff[currentCalibrationChannel][idx] = realVal;
            pointCountDiff[currentCalibrationChannel]++;
            Serial.printf("Punto %d: %.2f mV -> %.2f mV\n", idx+1, realVal, rawVal);
        } else {
            Serial.println("❌ Máximo de puntos alcanzado para este canal");
        }
    }
}

// =====================================================
// COMANDOS DE CONTROL (START/STOP/DIR/POT/STATUS)
// =====================================================
void processControlCommands(String command) {
    if (command == "START") {
        if (!systemStarted && !startRequested) {
            startRequested   = true;
            startRequestTime = millis();
            esp_timer_start_once(startDelayTimer, START_DELAY_MS * 1000);
            Serial.printf("▶️ Inicio en %d segundos...\n", START_DELAY_MS / 1000);
        } 
        else if (systemStarted) {
            Serial.println("⚠️ El sistema ya está iniciado");
        } 
        else {
            Serial.println("⚠️ Ya hay una solicitud pendiente");
        }
    }

    else if (command == "STOP") {
        if (startRequested) {
            startRequested = false;
            esp_timer_stop(startDelayTimer);
            Serial.println("⏹️ Inicio cancelado por consola");
        }

        if (systemStarted) {
            systemStarted = false;
            if (ioControlEnabled) ioController.setRelay(0, false);
            for (int i = 0; i < NUM_DEVICES; i++) forceTurnOffSCR(i);
            Serial.println("⛔ Sistema detenido por consola");
        } else {
            Serial.println("⚠️ El sistema ya estaba detenido");
        }
    }

    else if (command == "DIR_A") {
        direction = true;
        if (ioControlEnabled) ioController.setRelay(1, true);
        Serial.println("🔄 Dirección: DIR-A");
    }

    else if (command == "DIR_B") {
        direction = false;
        if (ioControlEnabled) ioController.setRelay(1, false);
        Serial.println("🔄 Dirección: DIR-B");
    }

    else if (command.startsWith("POT_")) {
        int percentage = command.substring(4).toInt();
        if (percentage >= 0 && percentage <= 100) {
            potManualControl   = true;
            manualPotPercentage = percentage;
            potPercentage       = percentage;
            Serial.printf("🎚️ Potenciómetro: %d%% (modo SERIAL)\n", potPercentage);
        } else {
            Serial.println("❌ Error: % debe estar entre 0 y 100");
        }
    }

    else if (command == "POT_AUTO") {
        potManualControl = false;
        Serial.println("🎚️ Control por potenciómetro físico");
    }

    else if (command == "POT_MANUAL") {
        potManualControl = true;
        Serial.printf("🎚️ Control manual (actual %d%%)\n", potPercentage);
    }

    else if (command == "STATUS") {
        Serial.printf("Sistema: %s | ", systemStarted ? "ACTIVO" : "INACTIVO");
        Serial.printf("Inicio pendiente: %s | ", startRequested ? "SI" : "NO");
        Serial.printf("Dirección: %s | ", direction ? "DIR-A" : "DIR-B");
        Serial.printf("Potenciómetro: %d%% | ", potPercentage);
        Serial.printf("Modo: %s\n", potManualControl ? "SERIAL" : "FÍSICO");
    }
}

// =====================================================
// COMANDOS DE IO (MCP23017)
// =====================================================
void processIOCommands(String command) {
    if (command == "INPUTS_STATUS") {
        Serial.println(ioController.getInputsStatus());
    }
    else if (command == "RELAY_STATUS") {
        Serial.println(ioController.getRelaysStatus());
    }
    else if (command == "RELAY_ALL_ON") {
        ioController.setAllRelays(true);
    }
    else if (command == "RELAY_ALL_OFF") {
        ioController.setAllRelays(false);
    }
    else if (command == "RELAY_TEST") {
        ioController.testSequence(300);
    }
    else if (command.startsWith("RELAY_")) {
        int relayNum = command.substring(6, 7).toInt() - 1;
        String action = command.substring(8);
        if (relayNum >= 0 && relayNum < 8) {
            if (action == "ON") ioController.setRelay(relayNum, true);
            else if (action == "OFF") ioController.setRelay(relayNum, false);
            else if (action == "TOGGLE") ioController.toggleRelay(relayNum);
        }
    }
}

// =====================================================
// COMANDOS POR SERIAL
// =====================================================
void processSerialCommands() {
    if (!Serial.available()) return;

    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();

    // Control log dinámico
    if (command == "LOG_ON")  { verboseLog = true;  Serial.println("🔊 Log detallado ACTIVADO"); return; }
    if (command == "LOG_OFF") { verboseLog = false; Serial.println("🔇 Log detallado DESACTIVADO"); return; }

    // Dependiendo del modo
    if (calibrationMode) {
        if (command == "CANCEL") {
            calibrationMode = false;
            Serial.println("Calibración cancelada");
            return;
        }
        // Otros comandos de calibración se manejan aquí...
    }
    else {
        processControlCommands(command);
        processIOCommands(command);
    }
}
