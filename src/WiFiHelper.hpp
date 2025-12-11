#pragma once
#include "esp_err.h"

namespace WiFiHelper {
    // SoftAP: el ESP crea una red Wi-Fi propia
    esp_err_t start_softap(const char* ssid,
                           const char* pass = nullptr,
                           uint8_t channel = 6,
                           uint8_t max_conn = 4);

    // STA: el ESP se conecta a tu router (si lo prefieres)
    esp_err_t start_sta(const char* ssid, const char* pass, uint32_t wait_ms = 10000);
}
