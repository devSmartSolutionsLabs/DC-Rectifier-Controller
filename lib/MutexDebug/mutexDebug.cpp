#include "mutexDebug.hpp"
#include "GlobalVars.hpp"  // Para acceder a i2cMutex

extern SemaphoreHandle_t i2cMutex;
static const char* currentHolder = "None";

bool takeI2CMutex(TickType_t timeout, const char* caller) {
    if (i2cMutex == nullptr) {
        Serial.printf("❌ [%s] takeI2CMutex: i2cMutex es NULL\n", caller);
        return false;
    }
    
    UBaseType_t available = uxSemaphoreGetCount(i2cMutex);
    if (available == 0) {
        Serial.printf("⚠️ [%s] Mutex ocupado por: %s. Esperando...\n", caller, currentHolder);
    }
    
    BaseType_t result = xSemaphoreTake(i2cMutex, timeout);
    
    if (result == pdTRUE) {
        currentHolder = caller;
        // Serial.printf("✅ [%s] Mutex tomado (disponibles: %d)\n", caller, uxSemaphoreGetCount(i2cMutex));
        return true;
    } else {
        Serial.printf("❌ [%s] Timeout después de %d ms. Holder: %s\n", 
                     caller, pdTICKS_TO_MS(timeout), currentHolder);
        return false;
    }
}

void giveI2CMutex(const char* caller) {
    if (i2cMutex != nullptr) {
        currentHolder = "None";
        xSemaphoreGive(i2cMutex);
        // Serial.printf("🔓 [%s] Mutex liberado\n", caller);
    }
}

void printMutexHolder() {
    Serial.printf("🔍 Mutex Holder: %s | Disponibles: %d\n", 
                 currentHolder, uxSemaphoreGetCount(i2cMutex));
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