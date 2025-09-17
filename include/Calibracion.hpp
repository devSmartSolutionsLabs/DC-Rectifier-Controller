#ifndef CALIBRACION_HPP
#define CALIBRACION_HPP

#include <EEPROM.h>

// --- Configuración EEPROM ---
#define MAX_POINTS 10
extern const int OFFSET_CURRENT_ADDR;
extern const int OFFSET_VOLTAGE_ADDR;
extern const int POINTS_CURRENT_ADDR;
extern const int POINTS_VOLTAGE_ADDR;
extern const int COUNTS_ADDR;

// Variables de calibración SEPARADAS
extern float offsetCurrent, offsetVoltage;
extern float rawPointsCurrent[MAX_POINTS], realPointsCurrent[MAX_POINTS];
extern float rawPointsVoltage[MAX_POINTS], realPointsVoltage[MAX_POINTS];
extern int pointCountCurrent, pointCountVoltage;

// Funciones de calibración
void loadCalibration();
void saveCalibration();
void resetCalibration(int sensor);
float applyCalibration(float measured, float *rawPts, float *realPts, int count);

#endif