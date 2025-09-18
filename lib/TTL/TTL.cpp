#include <Arduino.h>
#include "TTL.hpp"
#include "Calibracion.hpp"
#include "Sensores.hpp"
#include "MCP23017_IO.hpp"

// Variables para modo calibración
bool calibrationMode = false;
int currentCalibrationSensor = 0; // 0: ninguno, 1: single-ended, 2: diferencial
int currentCalibrationChannel = -1;
int currentCalibrationPort = -1;

void displayCalibrationHelp() {
  Serial.println("=== MODO CALIBRACIÓN ===");
  Serial.println("Comandos disponibles:");
  Serial.println("CAL_SINGLE_X - Calibrar canal single-ended X (0-3)");
  Serial.println("CAL_DIFF_X   - Calibrar canal diferencial X (0-1)");
  Serial.println("VOLT         - Calibrar voltímetro (canal 0 single-ended)");
  Serial.println("AMP          - Calibrar amperímetro (canal 0 diferencial)");
  Serial.println("SAVE         - Guardar calibración");
  Serial.println("CANCEL       - Cancelar calibración");
  Serial.println("CAL_STATUS   - Ver calibración actual");
  Serial.println("RESET_SINGLE_X - Resetear calibración single-ended canal X");
  Serial.println("RESET_DIFF_X   - Resetear calibración diferencial canal X");
  Serial.println("ZERO         - Calibrar offset a cero");
  Serial.println("HELP         - Mostrar esta ayuda");
}

void displayPortSelection() {
  Serial.println("Seleccione puerto diferencial:");
  Serial.println("0 - Diferencial 0-1");
  Serial.println("1 - Diferencial 2-3");
  Serial.println("Envie el número del puerto (0 o 1)");
}

float readCurrentRawValueImmediate() {
  if (currentCalibrationSensor == 1) { // Single-ended (ADS 0x48)
    return readAveraged(adsLow, currentCalibrationChannel, false, 0.125, 2);
  } else { // Diferencial (ADS 0x49)
    if (currentCalibrationPort == 0) {
      return readAveraged(adsHigh, 0, true, 0.125, 2);
    } else {
      return readAveraged(adsHigh, 1, true, 0.125, 2);
    }
  }
}

void processCalibrationPoint(String command) {
  if (command == "ZERO") {
    float sumRaw = 0;
    int readings = 10;
    
    for (int i = 0; i < readings; i++) {
      sumRaw += readCurrentRawValueImmediate();
      delay(10);
    }
    
    float currentRaw = sumRaw / readings;
    
    if (currentCalibrationSensor == 1) { // Single-ended
      offsetVoltage = currentRaw;
      Serial.printf("Offset single-ended canal %d ajustado: %.4f mV (10 lecturas)\n", 
                   currentCalibrationChannel, offsetVoltage);
    } else { // Diferencial
      offsetCurrent = currentRaw;
      Serial.printf("Offset diferencial canal %d ajustado: %.4f mV (10 lecturas)\n", 
                   currentCalibrationChannel, offsetCurrent);
    }
    return;
  }
  
  float realVal = command.toFloat();
  float rawVal = readCurrentRawValueImmediate();
  
  if (currentCalibrationSensor == 1) { // Single-ended
    if (pointCountSingle[currentCalibrationChannel] < MAX_POINTS) {
      int idx = pointCountSingle[currentCalibrationChannel];
      rawPointsSingle[currentCalibrationChannel][idx] = rawVal;
      realPointsSingle[currentCalibrationChannel][idx] = realVal;
      pointCountSingle[currentCalibrationChannel]++;
      Serial.printf("Punto %d: %.2f V -> %.2f mV\n", 
                   pointCountSingle[currentCalibrationChannel], realVal, rawVal);
    } else {
      Serial.println("Error: Máximo de puntos alcanzado para este canal");
    }
  } else { // Diferencial
    if (pointCountDiff[currentCalibrationChannel] < MAX_POINTS) {
      int idx = pointCountDiff[currentCalibrationChannel];
      rawPointsDiff[currentCalibrationChannel][idx] = rawVal;
      realPointsDiff[currentCalibrationChannel][idx] = realVal;
      pointCountDiff[currentCalibrationChannel]++;
      Serial.printf("Punto %d: %.2f mV -> %.2f mV\n", 
                   pointCountDiff[currentCalibrationChannel], realVal, rawVal);
    } else {
      Serial.println("Error: Máximo de puntos alcanzado para este canal");
    }
  }
}

void processControlCommands(String command) {
  if (command == "START") {
    if (!systemStarted && !startRequested) {
      startRequested = true;
      startRequestTime = millis();
      esp_timer_start_once(startDelayTimer, START_DELAY_MS * 1000);
      Serial.printf("Solicitud de inicio recibida. Iniciando en %d segundos...\n", 
                   START_DELAY_MS / 1000);
    } else if (systemStarted) {
      Serial.println("El sistema ya está iniciado");
    } else if (startRequested) {
      Serial.println("Ya hay una solicitud de inicio pendiente");
    }
  }
  else if (command == "STOP") {
    if (startRequested) {
      startRequested = false;
      esp_timer_stop(startDelayTimer);
      Serial.println("Inicio cancelado por consola");
    }
    
    if (systemStarted) {
      systemStarted = false;
      if (ioControlEnabled) {
        ioController.setRelay(0, false);
      }
      
      for (int i = 0; i < NUM_DEVICES; i++) {
        forceTurnOffSCR(i);
      }
      
      Serial.println("Sistema DETENIDO por consola - Relé 1 desactivado");
    } else {
      Serial.println("El sistema ya está detenido");
    }
  }
  else if (command == "DIR_A") {
    direction = true;
    if (ioControlEnabled) {
      ioController.setRelay(1, direction);
    }
    Serial.printf("Dirección configurada: %s\n", direction ? "DIR-A" : "DIR-B");
  }
  else if (command == "DIR_B") {
    direction = false;
    if (ioControlEnabled) {
      ioController.setRelay(1, direction);
    }
    Serial.printf("Dirección configurada: %s\n", direction ? "DIR-A" : "DIR-B");
  }
  else if (command.startsWith("POT_")) {
    int percentage = command.substring(4).toInt();
    if (percentage >= 0 && percentage <= 100) {
      potManualControl = true;
      manualPotPercentage = percentage;
      potPercentage = percentage;
      Serial.printf("Potenciometro configurado manualmente a: %d%%\n", potPercentage);
      Serial.println("Modo: Control por SERIAL (potenciómetro físico deshabilitado)");
    } else {
      Serial.println("Error: El porcentaje debe estar entre 0 y 100");
    }
  }
  else if (command == "POT_AUTO") {
    potManualControl = false;
    Serial.println("Modo: Control por POTENCIÓMETRO FÍSICO");
  }
  else if (command == "POT_MANUAL") {
    potManualControl = true;
    Serial.printf("Modo: Control por SERIAL (valor actual: %d%%)\n", potPercentage);
  }
  else if (command == "STATUS") {
    Serial.printf("Sistema: %s | ", systemStarted ? "ACTIVO" : "INACTIVO");
    Serial.printf("Inicio pendiente: %s | ", startRequested ? "SI" : "NO");
    Serial.printf("Dirección: %s | ", direction ? "DIR-A" : "DIR-B");
    Serial.printf("Potenciometro: %d%% | ", potPercentage);
    Serial.printf("Modo: %s\n", potManualControl ? "SERIAL" : "POT-FÍSICO");
    Serial.printf("Voltaje: %.2f mV | Corriente: %.2f mV\n", potVoltage, CurrentSensorVoltage);
    
    // Mostrar valores de todos los canales
    Serial.println("Canales single-ended (0-3):");
    for (int i = 0; i < 4; i++) {
      Serial.printf("  CH%d: %.2f mV | ", i, adcChannels[i]);
    }
    Serial.println("\nCanales diferenciales (0-1):");
    for (int i = 0; i < 2; i++) {
      Serial.printf("  DIF%d: %.2f mV | ", i, adcChannels[i + 4]);
    }
    Serial.println();
  }
}

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
      bool state = ioController.readPinB(inputNum);
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
  else if (command == "CHANNELS_STATUS") {
    Serial.println("=== ESTADO DE CANALES ===");
    Serial.println("Canales single-ended (ADS 0x48):");
    for (int i = 0; i < 4; i++) {
      Serial.printf("  CH%d: %.2f mV\n", i, adcChannels[i]);
    }
    Serial.println("Canales diferenciales (ADS 0x49):");
    for (int i = 0; i < 2; i++) {
      Serial.printf("  DIF%d: %.2f mV\n", i, adcChannels[i + 4]);
    }
  }
  else if (command == "ALL_CHANNELS") {
      Serial.println("=== TODOS LOS CANALES ===");
      Serial.println("Single-ended (ADS 0x48):");
      for (int i = 0; i < 4; i++) {
          Serial.printf("  CH%d: %.2f mV\n", i, adcChannels[i]);
      }
      Serial.println("Diferenciales (ADS 0x49):");
      for (int i = 0; i < 2; i++) {
          Serial.printf("  DIF%d: %.2f mV\n", i, adcChannels[i + 4]);
      }
  }
  else if (command.startsWith("CH_")) {
      int channel = command.substring(3).toInt();
      if (channel >= 0 && channel < 6) {
          Serial.printf("Canal %d: %.2f mV\n", channel, adcChannels[channel]);
          
          // Información adicional según el canal
          if (channel == 0) {
              Serial.printf("  Equivalente potenciómetro: %d%%\n", potPercentage);
          } else if (channel == 4) {
              Serial.printf("  Valor de corriente: %.2f mV\n", CurrentSensorVoltage);
          }
      } else {
          Serial.println("Error: Canal debe estar entre 0-5");
      }
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
      Serial.println("ADS 0x48: 4 canales single-ended (±4.096V)");
      Serial.println("ADS 0x49: 2 canales diferenciales (±256mV)");
      displayCalibrationHelp();
    }
    else if (command == "HELP") {
      if (calibrationMode) {
        displayCalibrationHelp();
      } else {
        Serial.println("=== COMANDOS DEL SISTEMA ===");
        Serial.println("START - Iniciar sistema (con delay de seguridad)");
        Serial.println("STOP - Detener sistema o cancelar inicio");
        Serial.println("DIR_A - Configurar dirección A");
        Serial.println("DIR_B - Configurar dirección B");
        Serial.println("POT_X - Configurar potenciómetro (0-100%)");
        Serial.println("POT_AUTO - Volver a control por potenciómetro físico");
        Serial.println("POT_MANUAL - Activar control por serial");
        Serial.println("STATUS - Mostrar estado del sistema");
        Serial.println("CHANNELS_STATUS - Mostrar valores de todos los canales");
        Serial.println("CALIBRATE - Entrar en modo calibración");
        Serial.println("TEST_MODE_ON/OFF - Modo test");
        Serial.println("EMERGENCY_OFF - Parada emergencia");
        Serial.println("SYSTEM_RESET - Reiniciar sistema");
      }
      
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
    else if (command.startsWith("CAL_SINGLE_")) {
      int channel = command.substring(11).toInt();
      if (channel >= 0 && channel < 4) {
        calibrationMode = true;
        currentCalibrationSensor = 1;
        currentCalibrationChannel = channel;
        Serial.printf("Calibrando SINGLE-ENDED canal %d (ADS 0x48, ±4.096V)\n", channel);
        Serial.println("Conecte el multímetro y ajuste la carga");
        Serial.println("Ingrese el valor de VOLTAJE leído en el multímetro (en V)");
        Serial.println("Ejemplo: 12.34");
        Serial.println("Envie 'DONE' cuando termine");
        Serial.println("Envie 'ZERO' para calibrar offset a cero");
      } else {
        Serial.println("Error: Canal debe estar entre 0 y 3");
      }
    }
    else if (command.startsWith("CAL_DIFF_")) {
      int channel = command.substring(9).toInt();
      if (channel >= 0 && channel < 2) {
        calibrationMode = true;
        currentCalibrationSensor = 2;
        currentCalibrationChannel = channel;
        Serial.printf("Calibrando DIFERENCIAL canal %d (ADS 0x49, ±256mV)\n", channel);
        Serial.println("Conecte el multímetro y ajuste la carga");
        displayPortSelection();
      } else {
        Serial.println("Error: Canal debe estar entre 0 y 1");
      }
    }
    else if (command == "VOLT" && calibrationMode) {
      calibrationMode = true;
      currentCalibrationSensor = 1;
      currentCalibrationChannel = 0;
      Serial.println("Calibrando VOLTÍMETRO (Single-ended canal 0, ADS 0x48, ±4.096V)");
      Serial.println("Conecte el multímetro y ajuste la carga");
      Serial.println("Ingrese el valor de VOLTAJE leído en el multímetro (en V)");
    }
    else if (command == "AMP" && calibrationMode) {
      calibrationMode = true;
      currentCalibrationSensor = 2;
      currentCalibrationChannel = 0;
      Serial.println("Calibrando AMPERÍMETRO (Diferencial canal 0, ADS 0x49, ±256mV)");
      Serial.println("Conecte el multímetro y ajuste la carga");
      displayPortSelection();
    }
    else if ((command == "0" || command == "1") && calibrationMode && currentCalibrationSensor == 2) {
      currentCalibrationPort = command.toInt();
      Serial.printf("Puerto %d seleccionado\n", currentCalibrationPort);
      Serial.println("Ajuste la carga y ingrese el valor de CORRIENTE leído en el multímetro (en mV)");
    }
    else if (command == "SAVE" && calibrationMode) {
      saveCalibration();
      calibrationMode = false;
      currentCalibrationSensor = 0;
      currentCalibrationChannel = -1;
      currentCalibrationPort = -1;
      Serial.println("Calibración guardada exitosamente");
    }
    else if (command == "CANCEL") {
      calibrationMode = false;
      currentCalibrationSensor = 0;
      currentCalibrationChannel = -1;
      currentCalibrationPort = -1;
      Serial.println("Calibración cancelada");
    }
    else if (command == "BACK" && calibrationMode && currentCalibrationSensor > 0) {
      currentCalibrationPort = -1;
      if (currentCalibrationSensor == 2) {
        displayPortSelection();
      }
    }
    else if (command == "STATUS") {
      if (calibrationMode) {
        Serial.println("=== ESTADO DE CALIBRACIÓN ===");
        if (currentCalibrationSensor == 1) {
          Serial.printf("Calibrando SINGLE-ENDED canal %d\n", currentCalibrationChannel);
          Serial.printf("  Puntos: %d\n", pointCountSingle[currentCalibrationChannel]);
          for (int i = 0; i < pointCountSingle[currentCalibrationChannel]; i++) {
            Serial.printf("    Punto %d: %.2f V -> %.2f mV\n", 
                         i, realPointsSingle[currentCalibrationChannel][i], 
                         rawPointsSingle[currentCalibrationChannel][i]);
          }
        } else if (currentCalibrationSensor == 2) {
          Serial.printf("Calibrando DIFERENCIAL canal %d\n", currentCalibrationChannel);
          Serial.printf("  Puntos: %d\n", pointCountDiff[currentCalibrationChannel]);
          for (int i = 0; i < pointCountDiff[currentCalibrationChannel]; i++) {
            Serial.printf("    Punto %d: %.2f mV -> %.2f mV\n", 
                         i, realPointsDiff[currentCalibrationChannel][i], 
                         rawPointsDiff[currentCalibrationChannel][i]);
          }
        }
      } else {
        processControlCommands(command);
      }
    }
    else if (command == "CAL_STATUS") {
      Serial.println("=== ESTADO DE CALIBRACIÓN COMPLETA ===");
      
      Serial.println("Canales single-ended (ADS 0x48):");
      for (int ch = 0; ch < 4; ch++) {
        Serial.printf("  Canal %d: %d puntos\n", ch, pointCountSingle[ch]);
        for (int i = 0; i < pointCountSingle[ch]; i++) {
          Serial.printf("    Punto %d: %.2f V -> %.2f mV\n", 
                       i, realPointsSingle[ch][i], rawPointsSingle[ch][i]);
        }
      }
      
      Serial.println("Canales diferenciales (ADS 0x49):");
      for (int ch = 0; ch < 2; ch++) {
        Serial.printf("  Canal %d: %d puntos\n", ch, pointCountDiff[ch]);
        for (int i = 0; i < pointCountDiff[ch]; i++) {
          Serial.printf("    Punto %d: %.2f mV -> %.2f mV\n", 
                       i, realPointsDiff[ch][i], rawPointsDiff[ch][i]);
        }
      }
    }
    else if (command.startsWith("RESET_SINGLE_")) {
      int channel = command.substring(13).toInt();
      if (channel >= 0 && channel < 4) {
        resetCalibration(1, channel);
      } else {
        Serial.println("Error: Canal debe estar entre 0 y 3");
      }
    }
    else if (command.startsWith("RESET_DIFF_")) {
      int channel = command.substring(11).toInt();
      if (channel >= 0 && channel < 2) {
        resetCalibration(2, channel);
      } else {
        Serial.println("Error: Canal debe estar entre 0 y 1");
      }
    }
    else if (command == "ZERO" && calibrationMode && currentCalibrationSensor > 0) {
      processCalibrationPoint(command);
    }
    else if (calibrationMode && currentCalibrationSensor > 0) {
      if (command == "DONE") {
        Serial.printf("Calibración completada para ");
        if (currentCalibrationSensor == 1) {
          Serial.printf("single-ended canal %d. %d puntos.\n", 
                       currentCalibrationChannel, pointCountSingle[currentCalibrationChannel]);
        } else {
          Serial.printf("diferencial canal %d. %d puntos.\n", 
                       currentCalibrationChannel, pointCountDiff[currentCalibrationChannel]);
        }
        currentCalibrationPort = -1;
      } else {
        processCalibrationPoint(command);
      }
    }
    else if (command == "POT_VALUE") {
      Serial.printf("Potenciómetro: %d%% | Voltage: %.2f mV\n", potPercentage, potVoltage);
    }
    else if (command == "CURRENT_VALUE") {
      Serial.printf("Corriente: %.2f mV\n", CurrentSensorVoltage);
    }
    else if (command == "VOLTAGE_VALUE") {
      Serial.printf("Voltaje: %.2f mV\n", potVoltage);
    }
    else if (command == "FREQUENCY") {
      Serial.printf("Frecuencia operación: %.1f Hz\n", 1000000.0 / (2 * SEMI_PERIOD_US));
    }
    else if (command == "EMERGENCY_OFF") {
      for (int i = 0; i < NUM_DEVICES; i++) {
        forceTurnOffSCR(i);
      }
      if (ioControlEnabled) {
        ioController.setAllRelays(false);
      }
      Serial.println("EMERGENCIA: Todos los SCRs y relés apagados");
    }
    else if (command == "SYSTEM_RESET") {
      Serial.println("Reiniciando sistema...");
      ESP.restart();
    }
    else if (command == "SET_START_DELAY") {
      int delaySec = command.substring(14).toInt();
      if (delaySec > 0) {
        START_DELAY_MS = delaySec * 1000;
        Serial.printf("Delay de inicio configurado: %d segundos\n", delaySec);
      }
    }
    else if (!calibrationMode) {
      processIOCommands(command);
      processControlCommands(command);
    }
    else {
      Serial.println("Comando no reconocido en modo calibración");
    }
  }
}