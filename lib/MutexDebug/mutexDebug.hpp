#ifndef MUTEXDEBUG_HPP
#define MUTEXDEBUG_HPP

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ================== DECLARACIONES ==================
bool takeI2CMutex(TickType_t timeout = pdMS_TO_TICKS(100), const char* caller = "Unknown");
void giveI2CMutex(const char* caller = "Unknown");
void checkMutexHealth();
void safeMutexTake(const char* caller = "Unknown");
void safeMutexGive(const char* caller = "Unknown");

// Función para debug del mutex holder
void printMutexHolder();

#endif