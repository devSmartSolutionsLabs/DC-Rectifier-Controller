#ifndef MUTEX_DEBUG_HPP
#define MUTEX_DEBUG_HPP

#include "GlobalVars.hpp"


bool takeI2CMutex(TickType_t timeout) {
    if (i2cMutex == NULL) {
        i2cMutex = xSemaphoreCreateMutex();
        if (i2cMutex == NULL) return false;
    }
    
    BaseType_t result = xSemaphoreTake(i2cMutex, timeout);
    return (result == pdTRUE);
}

void giveI2CMutex() {
    if (i2cMutex != NULL) {
        xSemaphoreGive(i2cMutex);
    }
}

void checkMutexHealth() {
    if (i2cMutex == NULL) {
        Serial.println("❌ MUTEX: i2cMutex es NULL");
        return;
    }
    
    UBaseType_t tasksWaiting = uxSemaphoreGetCount(i2cMutex);
    Serial.printf("🔍 MUTEX DEBUG: Semáforos disponibles: %d\n", tasksWaiting);
    
    // Información adicional del mutex
    Serial.printf("🔍 MUTEX: Stack High Water Mark: %u\n", uxTaskGetStackHighWaterMark(NULL));
}

void safeMutexTake(const char* caller) {
    if (i2cMutex == NULL) {
        Serial.printf("❌ MUTEX [%s]: Mutex es NULL\n", caller);
        return;
    }
    
    Serial.printf("🔒 [%s] Intentando tomar mutex...\n", caller);
    uint32_t startTime = millis();
    
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint32_t waitTime = millis() - startTime;
        Serial.printf("✅ [%s] Mutex tomado después de %lu ms\n", caller, waitTime);
    } else {
        Serial.printf("❌ [%s] TIMEOUT mutex después de 100ms\n", caller);
        checkMutexHealth();
    }
}

void safeMutexGive(const char* caller) {
    if (i2cMutex == NULL) {
        Serial.printf("❌ MUTEX [%s]: Mutex es NULL al liberar\n", caller);
        return;
    }
    
    xSemaphoreGive(i2cMutex);
    Serial.printf("🔓 [%s] Mutex liberado\n", caller);
}

#endif