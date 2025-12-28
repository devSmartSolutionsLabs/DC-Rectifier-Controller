#pragma once

#include <esp_http_server.h>
#include <string>

class PortalWeb {
public:
    PortalWeb();
    esp_err_t start();
    void stop();

private:
    httpd_handle_t _server = nullptr;
};