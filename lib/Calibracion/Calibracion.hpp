#ifndef CALIBRACION_HPP
#define CALIBRACION_HPP

#include <EEPROM.h>

// --- Configuración EEPROM ---
#define MAX_POINTS 10
#define MAX_SINGLE_CHANNELS 4
#define MAX_DIFF_CHANNELS 2

// Direcciones EEPROM
extern const int OFFSET_CURRENT_ADDR;
extern const int OFFSET_VOLTAGE_ADDR;
extern const int POINTS_SINGLE_ADDR;
extern const int POINTS_DIFF_ADDR;
extern const int COUNTS_ADDR;

// Variables de calibración para múltiples canales
extern float offsetCurrent, offsetVoltage;

// Calibración para canales single-ended (0-3)
extern float rawPointsSingle[MAX_SINGLE_CHANNELS][MAX_POINTS];
extern float realPointsSingle[MAX_SINGLE_CHANNELS][MAX_POINTS];
extern int pointCountSingle[MAX_SINGLE_CHANNELS];

// Calibración para canales diferenciales (0-1)
extern float rawPointsDiff[MAX_DIFF_CHANNELS][MAX_POINTS];
extern float realPointsDiff[MAX_DIFF_CHANNELS][MAX_POINTS];
extern int pointCountDiff[MAX_DIFF_CHANNELS];

// Funciones de calibración
void loadCalibration();
void saveCalibration();
void resetCalibration(int sensorType, int channel);
float applyCalibration(float measured, float *rawPts, float *realPts, int count);

#endif