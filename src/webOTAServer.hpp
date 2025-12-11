#pragma once
#include "esp_err.h"

class WebOTAServer {
public:
    // start en el puerto dado; expone /update (POST binario)
    static esp_err_t start(uint16_t port = 80);
    static void stop();
};
