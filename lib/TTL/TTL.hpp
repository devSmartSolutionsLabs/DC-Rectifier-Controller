#ifndef TTL_HPP
#define TTL_HPP

#include <Arduino.h>
#include "GlobalVars.hpp"

// Variables para modo calibración (solo declaración extern)
extern bool calibrationMode;
extern int currentCalibrationSensor;
extern int currentCalibrationPort;

// Funciones de calibración por TTL
void processSerialCommands();
void displayCalibrationHelp();
void displayPortSelection();
float readCurrentRawValueImmediate();
void processCalibrationPoint(String command);

// Declarar la función forceTurnOffSCR para que esté disponible
void forceTurnOffSCR(uint8_t dev);

// Agregar estas declaraciones que faltan
void processControlCommands(String command);
void processIOCommands(String command);

#endif