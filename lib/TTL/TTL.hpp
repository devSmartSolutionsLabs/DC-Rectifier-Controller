#ifndef TTL_HPP
#define TTL_HPP

#include <Arduino.h>

// Variables para modo calibración
extern bool calibrationMode;
extern int currentCalibrationSensor;
extern int currentCalibrationPort;

extern volatile uint32_t START_DELAY_MS ;  // 3 segundos de delay de inicio
// ✅ Agregar declaración extern para testMode
extern volatile bool testMode;
extern volatile bool systemStarted;
extern volatile bool direction;
// Constante compartida desde el archivo principal
extern const int SEMI_PERIOD_US;  // ← AÑADIR ESTA LÍNEA
extern const int NUM_DEVICES;  // ← AÑADIR ESTA LÍNEA

// Funciones de calibración por TTL
void processSerialCommands();
void displayCalibrationHelp();
void displayPortSelection();
float readCurrentRawValueImmediate();
void processCalibrationPoint(String command);

// Declarar la función forceTurnOffSCR para que esté disponible
void forceTurnOffSCR(uint8_t dev);  // ← AÑADIR ESTA LÍNEA

#endif