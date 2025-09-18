#include <Arduino.h>
#include "Calibracion.hpp"

// --- Configuración EEPROM ---
const int OFFSET_CURRENT_ADDR = 0;
const int OFFSET_VOLTAGE_ADDR = OFFSET_CURRENT_ADDR + 4;
const int POINTS_SINGLE_ADDR = OFFSET_VOLTAGE_ADDR + 4;
const int POINTS_DIFF_ADDR = POINTS_SINGLE_ADDR + (MAX_SINGLE_CHANNELS * MAX_POINTS * 2 * sizeof(float));
const int COUNTS_ADDR = POINTS_DIFF_ADDR + (MAX_DIFF_CHANNELS * MAX_POINTS * 2 * sizeof(float));

// Variables de calibración
float offsetCurrent = 0.0f, offsetVoltage = 0.0f;

// Calibración para canales single-ended
float rawPointsSingle[MAX_SINGLE_CHANNELS][MAX_POINTS];
float realPointsSingle[MAX_SINGLE_CHANNELS][MAX_POINTS];
int pointCountSingle[MAX_SINGLE_CHANNELS] = {0};

// Calibración para canales diferenciales
float rawPointsDiff[MAX_DIFF_CHANNELS][MAX_POINTS];
float realPointsDiff[MAX_DIFF_CHANNELS][MAX_POINTS];
int pointCountDiff[MAX_DIFF_CHANNELS] = {0};

void loadCalibration() {
  // Cargar offsets
  EEPROM.get(OFFSET_CURRENT_ADDR, offsetCurrent);
  EEPROM.get(OFFSET_VOLTAGE_ADDR, offsetVoltage);
  
  // Validar offsets
  if (isnan(offsetCurrent) || isinf(offsetCurrent)) offsetCurrent = 0.0f;
  if (isnan(offsetVoltage) || isinf(offsetVoltage)) offsetVoltage = 0.0f;
  
  // Cargar calibración single-ended
  for (int i = 0; i < MAX_SINGLE_CHANNELS; i++) {
    EEPROM.get(POINTS_SINGLE_ADDR + (i * MAX_POINTS * 2 * sizeof(float)), rawPointsSingle[i]);
    EEPROM.get(POINTS_SINGLE_ADDR + (i * MAX_POINTS * 2 * sizeof(float) + (MAX_POINTS * sizeof(float))), realPointsSingle[i]);
    EEPROM.get(COUNTS_ADDR + (i * sizeof(int)), pointCountSingle[i]);
    
    if (pointCountSingle[i] < 0 || pointCountSingle[i] > MAX_POINTS) pointCountSingle[i] = 0;
  }
  
  // Cargar calibración diferencial
  for (int i = 0; i < MAX_DIFF_CHANNELS; i++) {
    EEPROM.get(POINTS_DIFF_ADDR + (i * MAX_POINTS * 2 * sizeof(float)), rawPointsDiff[i]);
    EEPROM.get(POINTS_DIFF_ADDR + (i * MAX_POINTS * 2 * sizeof(float) + (MAX_POINTS * sizeof(float))), realPointsDiff[i]);
    EEPROM.get(COUNTS_ADDR + (MAX_SINGLE_CHANNELS * sizeof(int)) + (i * sizeof(int)), pointCountDiff[i]);
    
    if (pointCountDiff[i] < 0 || pointCountDiff[i] > MAX_POINTS) pointCountDiff[i] = 0;
  }
}

void saveCalibration() {
  // Guardar offsets
  EEPROM.put(OFFSET_CURRENT_ADDR, offsetCurrent);
  EEPROM.put(OFFSET_VOLTAGE_ADDR, offsetVoltage);
  
  // Guardar calibración single-ended
  for (int i = 0; i < MAX_SINGLE_CHANNELS; i++) {
    EEPROM.put(POINTS_SINGLE_ADDR + (i * MAX_POINTS * 2 * sizeof(float)), rawPointsSingle[i]);
    EEPROM.put(POINTS_SINGLE_ADDR + (i * MAX_POINTS * 2 * sizeof(float) + (MAX_POINTS * sizeof(float))), realPointsSingle[i]);
    EEPROM.put(COUNTS_ADDR + (i * sizeof(int)), pointCountSingle[i]);
  }
  
  // Guardar calibración diferencial
  for (int i = 0; i < MAX_DIFF_CHANNELS; i++) {
    EEPROM.put(POINTS_DIFF_ADDR + (i * MAX_POINTS * 2 * sizeof(float)), rawPointsDiff[i]);
    EEPROM.put(POINTS_DIFF_ADDR + (i * MAX_POINTS * 2 * sizeof(float) + (MAX_POINTS * sizeof(float))), realPointsDiff[i]);
    EEPROM.put(COUNTS_ADDR + (MAX_SINGLE_CHANNELS * sizeof(int)) + (i * sizeof(int)), pointCountDiff[i]);
  }
  
  #if defined(ESP32)
  EEPROM.commit();
  #endif
  
  Serial.println("Calibración guardada en EEPROM");
}

void resetCalibration(int sensorType, int channel) {
  if (sensorType == 1) { // Single-ended
    if (channel >= 0 && channel < MAX_SINGLE_CHANNELS) {
      pointCountSingle[channel] = 0;
      Serial.printf("Calibración single-ended canal %d reseteada\n", channel);
    }
  } else if (sensorType == 2) { // Diferencial
    if (channel >= 0 && channel < MAX_DIFF_CHANNELS) {
      pointCountDiff[channel] = 0;
      Serial.printf("Calibración diferencial canal %d reseteada\n", channel);
    }
  }
}

float applyCalibration(float measured, float *rawPts, float *realPts, int count) {
  if (count == 0) return measured;
  if (count == 1) return realPts[0];
  
  for (int i = 0; i < count - 1; i++) {
    if (measured >= rawPts[i] && measured <= rawPts[i + 1]) {
      float t = (measured - rawPts[i]) / (rawPts[i + 1] - rawPts[i]);
      return realPts[i] + t * (realPts[i + 1] - realPts[i]);
    }
  }
  
  if (measured < rawPts[0]) {
    float slope = (realPts[1] - realPts[0]) / (rawPts[1] - rawPts[0]);
    return realPts[0] + slope * (measured - rawPts[0]);
  } else {
    float slope = (realPts[count - 1] - realPts[count - 2]) /
                  (rawPts[count - 1] - rawPts[count - 2]);
    return realPts[count - 1] + slope * (measured - rawPts[count - 1]);
  }
}