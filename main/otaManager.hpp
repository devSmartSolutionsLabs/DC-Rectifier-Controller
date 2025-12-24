#pragma once
#include <esp_http_server.h>
#include <esp_http_client.h>
#include <string>

class OtaManager {
public:
    OtaManager();
    void start_portal();
    static void check_github_releases(); // Lista versiones
    static void start_ota_update(const std::string& url);

private:
    static esp_err_t http_server_handler(httpd_req_t *req);
    static esp_err_t wifi_config_handler(httpd_req_t *req);
    static esp_err_t github_list_handler(httpd_req_t *req);
};