#include "WebOTAServer.hpp"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "WebOTAServer";
static httpd_handle_t s_server = nullptr;

static esp_err_t update_post_handler(httpd_req_t* req)
{
    esp_ota_handle_t ota_handle = 0;
    const esp_partition_t* update_part = esp_ota_get_next_update_partition(NULL);
    if (!update_part) return ESP_FAIL;

    esp_err_t err = esp_ota_begin(update_part, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ota_begin fail: %s", esp_err_to_name(err));
        return err;
    }

    char buf[4096];
    int received = 0, written = 0;
    while (true) {
        int n = httpd_req_recv(req, buf, sizeof(buf));
        if (n < 0) { err = ESP_FAIL; break; }
        if (n == 0) break; // fin

        err = esp_ota_write(ota_handle, buf, n);
        if (err != ESP_OK) break;
        received += n; written += n;
    }

    if (err == ESP_OK) err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ota_end fail: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return err;
    }

    err = esp_ota_set_boot_partition(update_part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_boot_partition fail: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return err;
    }

    httpd_resp_sendstr(req, "OK. Rebooting...\n");
    ESP_LOGI(TAG, "Update OK (%d bytes). Rebooting", written);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

esp_err_t WebOTAServer::start(uint16_t port){
    if (s_server) return ESP_OK;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = port;
    cfg.uri_match_fn = httpd_uri_match_wildcard;

    esp_err_t err = httpd_start(&s_server, &cfg);
    if (err != ESP_OK) return err;

    httpd_uri_t update = {
        .uri       = "/update",
        .method    = HTTP_POST,
        .handler   = update_post_handler,
        .user_ctx  = nullptr
    };
    httpd_register_uri_handler(s_server, &update);

    ESP_LOGI(TAG, "Web OTA server on :%u (POST /update)", (unsigned)port);
    return ESP_OK;
}

void WebOTAServer::stop(){
    if (s_server){
        httpd_stop(s_server);
        s_server = nullptr;
    }
}
