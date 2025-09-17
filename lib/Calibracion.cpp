#include "Calibracion.hpp"

// --- Configuración EEPROM ---
const int OFFSET_CURRENT_ADDR = 0;
const int OFFSET_VOLTAGE_ADDR = OFFSET_CURRENT_ADDR + 4;
const int POINTS_CURRENT_ADDR = OFFSET_VOLTAGE_ADDR + 4;
const int POINTS_VOLTAGE_ADDR = POINTS_CURRENT_ADDR + (MAX_POINTS * 2 * sizeof(float));
const int COUNTS_ADDR = POINTS_VOLTAGE_ADDR + (MAX_POINTS * 2 * sizeof(float));

// Variables de calibración SEPARADAS
float offsetCurrent = 0.0f, offsetVoltage = 0.0f;
float rawPointsCurrent[MAX_POINTS], realPointsCurrent[MAX_POINTS];
float rawPointsVoltage[MAX_POINTS], realPointsVoltage[MAX_POINTS];
int pointCountCurrent = 0, pointCountVoltage = 0;

void loadCalibration() {
  EEPROM.get(OFFSET_CURRENT_ADDR, offsetCurrent);
  EEPROM.get(OFFSET_VOLTAGE_ADDR, offsetVoltage);
  EEPROM.get(POINTS_CURRENT_ADDR, rawPointsCurrent);
  EEPROM.get(POINTS_CURRENT_ADDR + MAX_POINTS * sizeof(float), realPointsCurrent);
  EEPROM.get(POINTS_VOLTAGE_ADDR, rawPointsVoltage);
  EEPROM.get(POINTS_VOLTAGE_ADDR + MAX_POINTS * sizeof(float), realPointsVoltage);
  EEPROM.get(COUNTS_ADDR, pointCountCurrent);
  EEPROM.get(COUNTS_ADDR + sizeof(int), pointCountVoltage);

  if (pointCountCurrent < 0 || pointCountCurrent > MAX_POINTS) pointCountCurrent = 0;
  if (pointCountVoltage < 0 || pointCountVoltage > MAX_POINTS) pointCountVoltage = 0;
}

void saveCalibration() {
  EEPROM.put(OFFSET_CURRENT_ADDR, offsetCurrent);
  EEPROM.put(OFFSET_VOLTAGE_ADDR, offsetVoltage);
  EEPROM.put(POINTS_CURRENT_ADDR, rawPointsCurrent);
  EEPROM.put(POINTS_CURRENT_ADDR + MAX_POINTS * sizeof(float), realPointsCurrent);
  EEPROM.put(POINTS_VOLTAGE_ADDR, rawPointsVoltage);
  EEPROM.put(POINTS_VOLTAGE_ADDR + MAX_POINTS * sizeof(float), realPointsVoltage);
  EEPROM.put(COUNTS_ADDR, pointCountCurrent);
  EEPROM.put(COUNTS_ADDR + sizeof(int), pointCountVoltage);
  
  #if defined(ESP32)
  EEPROM.commit();
  #endif
  
  Serial.println("Calibración guardada en EEPROM");
}

void resetCalibration(int sensor) {
  if (sensor == 1) { // Voltímetro
    pointCountVoltage = 0;
    offsetVoltage = 0.0f;
    Serial.println("Calibración voltímetro reseteada");
  } else if (sensor == 2) { // Amperímetro
    pointCountCurrent = 0;
    offsetCurrent = 0.0f;
    Serial.println("Calibración amperímetro reseteada");
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