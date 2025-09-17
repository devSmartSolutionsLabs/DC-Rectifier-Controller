#include <Arduino.h>
#include "TTL.hpp"
#include "Calibracion.hpp"
#include "Sensores.hpp"
#include "MCP23017_IO.hpp"

// Variables para modo calibración
bool calibrationMode = false;
int currentCalibrationSensor = 0;
int currentCalibrationPort = -1;

// Declarar la constante externa
extern const int SEMI_PERIOD_US;
extern const int NUM_DEVICES;

void displayCalibrationHelp() {
  Serial.println("=== MODO CALIBRACIÓN ===");
  Serial.println("Comandos disponibles:");
  Serial.println("VOLT - Calibrar voltímetro (ADS 0x48, ±4.096V)");
  Serial.println("AMP  - Calibrar amperímetro (ADS 0x49, ±256mV)");
  Serial.println("SAVE - Guardar calibración");
  Serial.println("CANCEL - Cancelar calibración");
  Serial.println("STATUS - Ver calibración actual");
  Serial.println("RESET_VOLT - Resetear calibración voltímetro");
  Serial.println("RESET_AMP - Resetear calibración amperímetro");
  Serial.println("ZERO - Calibrar offset a cero");
  Serial.println("HELP - Mostrar esta ayuda");
}

void displayPortSelection() {
  Serial.println("Seleccione puerto diferencial:");
  Serial.println("0 - Diferencial 0-1");
  Serial.println("1 - Diferencial 2-3");
  Serial.println("Envie el número del puerto (0 o 1)");
}

float readCurrentRawValueImmediate() {
  if (currentCalibrationSensor == 1) { // Voltímetro (ADS 0x48)
    if (currentCalibrationPort == 0) {
      return readAveraged(adsLow, 0, 0.125, 2);
    } else {
      return readAveraged(adsLow, 1, 0.125, 2);
    }
  } else { // Amperímetro (ADS 0x49)
    if (currentCalibrationPort == 0) {
      return readAveraged(adsHigh, 0, 0.0078125, 2);
    } else {
      return readAveraged(adsHigh, 1, 0.0078125, 2);
    }
  }
}

void processCalibrationPoint(String command) {
  if (command == "ZERO") {
    // Hacer 10 lecturas y promediar para mejor precisión
    float sumRaw = 0;
    int readings = 10;
    
    for (int i = 0; i < readings; i++) {
      sumRaw += readCurrentRawValueImmediate();
      delay(10); // Pequeña pausa entre lecturas
    }
    
    float currentRaw = sumRaw / readings;
    
    if (currentCalibrationSensor == 1) { // Voltímetro
      offsetVoltage = currentRaw;
      Serial.printf("Offset voltímetro ajustado: %.4f mV (10 lecturas)\n", offsetVoltage);
    } else { // Amperímetro
      offsetCurrent = currentRaw;
      Serial.printf("Offset amperímetro ajustado: %.4f mV (10 lecturas)\n", offsetCurrent);
    }
    return;
  }
  float realVal = command.toFloat();
  float rawVal = readCurrentRawValueImmediate();
  
  if (currentCalibrationSensor == 1) { // Voltímetro
    if (pointCountVoltage < MAX_POINTS) {
      rawPointsVoltage[pointCountVoltage] = rawVal;
      realPointsVoltage[pointCountVoltage] = realVal;
      pointCountVoltage++;
      Serial.printf("Punto %d: %.2f V -> %.2f mV\n", pointCountVoltage, realVal, rawVal);
    } else {
      Serial.println("Error: Máximo de puntos alcanzado para voltímetro");
    }
  } else { // Amperímetro
    if (pointCountCurrent < MAX_POINTS) {
      rawPointsCurrent[pointCountCurrent] = rawVal;
      realPointsCurrent[pointCountCurrent] = realVal;
      pointCountCurrent++;
      Serial.printf("Punto %d: %.2f mV -> %.2f mV\n", pointCountCurrent, realVal, rawVal);
    } else {
      Serial.println("Error: Máximo de puntos alcanzado para amperímetro");
    }
  }
}

// Función para procesar comandos de IO (MCP23017)
void processIOCommands(String command) {
  if (command == "INPUTS_STATUS") {
    Serial.println(ioController.getInputsStatus());
  }
  else if (command == "RELAY_STATUS") {
    Serial.println(ioController.getRelaysStatus());
  }
  else if (command == "INPUT_MONITOR") {
    if (ioController.isMonitoring()) {
      ioController.stopInputMonitoring();
      Serial.println("Monitor de entradas DESACTIVADO");
    } else {
      ioController.startInputMonitoring();
      Serial.println("Monitor de entradas ACTIVADO");
    }
  }
  else if (command.startsWith("INPUT_DEBOUNCE_MS")) {
    int time = command.substring(17).toInt();
    if (time > 0) {
      ioController.setDebounceTime(time);
      Serial.printf("Tiempo debounce configurado: %d ms\n", time);
    } else {
      Serial.println("Tiempo inválido. Use: INPUT_DEBOUNCE_MS 50");
    }
  }
  else if (command.startsWith("INPUT_") && command.endsWith("_STATUS")) {
    int inputNum = command.substring(6, 7).toInt() - 1;
    if (inputNum >= 0 && inputNum < 8) {
      bool state = ioController.readPinB(inputNum);  // ← CAMBIADO: readPinB en lugar de readInput
      Serial.printf("Entrada %d: %s\n", inputNum + 1, state ? "ACTIVA" : "INACTIVA");
    } else {
      Serial.println("Número de entrada inválido. Use 1-8");
    }
  }
  else if (command == "RELAY_TEST") {
    ioController.testSequence(300);
  }
  else if (command == "RELAY_ALL_ON") {
    ioController.setAllRelays(true);
  }
  else if (command == "RELAY_ALL_OFF") {
    ioController.setAllRelays(false);
  }
  else if (command.startsWith("RELAY_")) {
    // Formato: RELAY_X_ON, RELAY_X_OFF, RELAY_X_TOGGLE
    int relayNum = command.substring(6, 7).toInt() - 1;
    String action = command.substring(8);
    
    if (relayNum >= 0 && relayNum < 8) {
      if (action == "ON") {
        ioController.setRelay(relayNum, true);
      } else if (action == "OFF") {
        ioController.setRelay(relayNum, false);
      } else if (action == "TOGGLE") {
        ioController.toggleRelay(relayNum);
      } else {
        Serial.println("Formato: RELAY_X_ON, RELAY_X_OFF o RELAY_X_TOGGLE");
      }
    } else {
      Serial.println("Número de relé inválido. Use 1-8");
    }
  }
  else if (command == "PULLUPS_ON") {
    ioController.enableInputPullups(true);
  }
  else if (command == "PULLUPS_OFF") {
    ioController.enableInputPullups(false);
  }
  // En la función processIOCommands de TTL.cpp, modifica el manejo de entradas:
  else if (command.startsWith("INPUT_") && command.endsWith("_STATUS")) {
      int inputNum = command.substring(6, 7).toInt() - 1;
      if (inputNum >= 0 && inputNum < 8) {
          bool state = ioController.readPinB(inputNum);  // Ya está correcto
          Serial.printf("Entrada %d: %s\n", inputNum + 1, state ? "ACTIVA" : "INACTIVA");
      } else {
          Serial.println("Número de entrada inválido. Use 1-8");
      }
  }

  else if (command == "TEST_MODE_ON") {
    testMode = true;
    Serial.println("Modo TEST activado - Prints habilitados");
  }
  else if (command == "TEST_MODE_OFF") {
    testMode = false;
      Serial.println("Modo TEST desactivado - Prints deshabilitados");
  }
  else if (command == "TEST_MODE_STATUS") {
    Serial.printf("Modo TEST: %s\n", testMode ? "ACTIVADO" : "DESACTIVADO");
  }

  else if (command == "SYSTEM_STATUS") {
    Serial.printf("Sistema: %s | ", systemStarted ? "ACTIVO" : "INACTIVO");
    Serial.printf("Relé 1: %s | ", ioController.getRelayState(0) ? "ON" : "OFF");
    Serial.printf("Relé 2: %s | ", ioController.getRelayState(1) ? "ON" : "OFF");
    Serial.printf("Dirección: %s\n", direction ? "DIR-A" : "DIR-B");
  }

  else {
    Serial.println("Comando IO no reconocido");
  }
}

void processSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    
    if (command == "CALIBRATE") {
      calibrationMode = true;
      Serial.println("Modo calibración activado");
      Serial.println("ADS 0x48: Voltímetro (±4.096V)");
      Serial.println("ADS 0x49: Amperímetro (±256mV)");
      displayCalibrationHelp();
    }
    else if (command == "HELP") {
      displayCalibrationHelp();
      Serial.println("\n=== COMANDOS IO (MCP23017) ===");
      Serial.println("INPUTS_STATUS - Estado de entradas digitales");
      Serial.println("RELAY_STATUS - Estado de relés");
      Serial.println("INPUT_MONITOR - Monitor continuo de entradas");
      Serial.println("INPUT_DEBOUNCE_MS X - Configurar debounce");
      Serial.println("INPUT_X_STATUS - Estado de entrada específica");
      Serial.println("RELAY_TEST - Secuencia prueba relés");
      Serial.println("RELAY_ALL_ON - Encender todos los relés");
      Serial.println("RELAY_ALL_OFF - Apagar todos los relés");
      Serial.println("RELAY_X_ON - Encender relé X (1-8)");
      Serial.println("RELAY_X_OFF - Apagar relé X (1-8)");
      Serial.println("RELAY_X_TOGGLE - Cambiar estado relé X");
      Serial.println("PULLUPS_ON - Habilitar pull-ups");
      Serial.println("PULLUPS_OFF - Deshabilitar pull-ups");
    }
    else if (command == "VOLT" && calibrationMode) {
      currentCalibrationSensor = 1;
      Serial.println("Calibrando VOLTÍMETRO (ADS 0x48, ±4.096V)");
      Serial.println("Conecte el multimetro y ajuste la carga");
      displayPortSelection();
    }
    else if (command == "AMP" && calibrationMode) {
      currentCalibrationSensor = 2;
      Serial.println("Calibrando AMPERÍMETRO (ADS 0x49, ±256mV)");
      Serial.println("Conecte el multimetro y ajuste la carga");
      displayPortSelection();
    }
    else if ((command == "0" || command == "1") && calibrationMode && currentCalibrationSensor > 0) {
      currentCalibrationPort = command.toInt();
      Serial.printf("Puerto %d seleccionado\n", currentCalibrationPort);
      
      if (currentCalibrationSensor == 1) {
        Serial.println("Ajuste la carga y ingrese el valor de VOLTAJE leído en el multimetro (en V)");
      } else {
        Serial.println("Ajuste la carga y ingrese el valor de CORRIENTE leído en el multimetro (en mV)");
      }
      Serial.println("Ejemplo: 12.34");
      Serial.println("El sistema leerá el valor RAW inmediatamente después");
      Serial.println("Envie 'DONE' cuando termine");
      Serial.println("Envie 'ZERO' para calibrar offset a cero");
      Serial.println("Envie 'BACK' para cambiar de puerto");
    }
    else if (command == "SAVE" && calibrationMode) {
      saveCalibration();
      calibrationMode = false;
      currentCalibrationSensor = 0;
      currentCalibrationPort = -1;
      Serial.println("Calibración guardada exitosamente");
    }
    else if (command == "CANCEL") {
      calibrationMode = false;
      currentCalibrationSensor = 0;
      currentCalibrationPort = -1;
      Serial.println("Calibración cancelada");
    }
    else if (command == "BACK" && calibrationMode && currentCalibrationSensor > 0) {
      currentCalibrationPort = -1;
      displayPortSelection();
    }
    else if (command == "STATUS") {
      Serial.println("=== ESTADO DE CALIBRACIÓN ===");
      
      Serial.println("VOLTÍMETRO (ADS 0x48, ±4.096V):");
      Serial.printf("  Puntos: %d, Offset: %.4f mV\n", pointCountVoltage, offsetVoltage);
      for (int i = 0; i < pointCountVoltage; i++) {
        Serial.printf("    Punto %d: %.2f V -> %.2f mV\n", i, realPointsVoltage[i], rawPointsVoltage[i]);
      }
      
      Serial.println("AMPERÍMETRO (ADS 0x49, ±256mV):");
      Serial.printf("  Puntos: %d, Offset: %.4f mV\n", pointCountCurrent, offsetCurrent);
      for (int i = 0; i < pointCountCurrent; i++) {
        Serial.printf("    Punto %d: %.2f mV -> %.2f mV\n", i, realPointsCurrent[i], rawPointsCurrent[i]);
      }
    }
    else if (command == "RESET_VOLT") {
      resetCalibration(1);
    }
    else if (command == "RESET_AMP") {
      resetCalibration(2);
    }
    else if (command == "ZERO" && calibrationMode && currentCalibrationSensor > 0 && currentCalibrationPort >= 0) {
      processCalibrationPoint(command);
    }
    else if (calibrationMode && currentCalibrationSensor > 0 && currentCalibrationPort >= 0) {
      if (command == "DONE") {
        Serial.printf("Calibración completada para ");
        if (currentCalibrationSensor == 1) {
          Serial.printf("voltímetro. %d puntos.\n", pointCountVoltage);
        } else {
          Serial.printf("amperímetro. %d puntos.\n", pointCountCurrent);
        }
        currentCalibrationPort = -1;
      } else {
        processCalibrationPoint(command);
      }
    }
    // Comandos de monitoreo del sistema
    else if (command == "POT_VALUE") {
      Serial.printf("Potenciómetro: %d%% | Voltage: %.2f mV\n", potPercentage, potVoltage);
    }
    else if (command == "CURRENT_VALUE") {
      Serial.printf("Corriente: %.2f mV\n", CurrentSensorVoltage);
    }
    else if (command == "VOLTAGE_VALUE") {
      Serial.printf("Voltaje: %.2f mV | Real: %.2f V\n", potVoltage, potVoltage / 50.5);
    }
    else if (command == "FREQUENCY") {
      Serial.printf("Frecuencia operación: %.1f Hz\n", 1000000.0 / (2 * SEMI_PERIOD_US));
    }
    // Comandos de seguridad
    else if (command == "EMERGENCY_OFF") {
      for (int i = 0; i < NUM_DEVICES; i++) {
        forceTurnOffSCR(i);
      }
      ioController.setAllRelays(false);
      Serial.println("EMERGENCIA: Todos los SCRs y relés apagados");
    }
    else if (command == "SYSTEM_RESET") {
      Serial.println("Reiniciando sistema...");
      ESP.restart();
    }

    else if (command == "SET_START_DELAY") {
    // Para configurar el delay por serial si quieres
    int delaySec = command.substring(14).toInt();
        if (delaySec > 0) {
            START_DELAY_MS = delaySec * 1000;
            Serial.printf("Delay de inicio configurado: %d segundos\n", delaySec);
        }
    }
    // Procesar comandos de IO (MCP23017)
    else {
      processIOCommands(command);
    }
  }
}