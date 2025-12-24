#pragma once

#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <algorithm>

class WebOtaServer {
public:
    WebOtaServer();
    esp_err_t start();

private:
    httpd_handle_t _server = nullptr;
    
    // Handlers estáticos requeridos por el API de ESP-IDF
    static esp_err_t index_get_handler(httpd_req_t *req);
    static esp_err_t update_post_handler(httpd_req_t *req);
};