#pragma once
#include "esp_err.h"

// Arranca servidor HTTP con página de carga y endpoint OTA.
// Devuelve ESP_OK si quedó sirviendo.
esp_err_t http_ota_start(uint16_t port = 80);

// Detiene el servidor si está activo.
void      http_ota_stop();
