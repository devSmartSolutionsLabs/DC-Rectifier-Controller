#pragma once

#include <esp_http_server.h>
#include <string>
#include "WifiManager.hpp"
#include "GitHubClient.hpp"

class PortalWeb {
public:
    PortalWeb();
    esp_err_t start();
    void stop();

private:
    httpd_handle_t _server = nullptr;

    // Handlers de las rutas (Endpoints)
    static esp_err_t root_get_handler(httpd_req_t *req);
    static esp_err_t scan_get_handler(httpd_req_t *req);
    static esp_err_t wifi_post_handler(httpd_req_t *req);
    static esp_err_t github_list_handler(httpd_req_t *req);
    static esp_err_t update_execute_handler(httpd_req_t *req);
};