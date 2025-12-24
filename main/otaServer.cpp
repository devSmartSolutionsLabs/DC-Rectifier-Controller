#include "ota_server.hpp"

static const char *TAG = "WEB_OTA";
extern volatile bool g_scr_enabled; // Referencia a tu variable de main.cpp

// HTML simple embebido en la Flash
static const char* index_html = R"(
<!DOCTYPE html><html><head><title>RECTIFICADOR - OTA</title>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<style>
    body { font-family: Arial; text-align: center; background: #f4f4f4; }
    .card { background: white; padding: 30px; border-radius: 10px; display: inline-block; margin-top: 50px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }
    .btn { background: #007bff; color: white; padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; }
    .warn { color: red; font-weight: bold; }
</style></head>
<body><div class='card'>
    <h2>Actualización de Firmware</h2>
    <p>Seleccione el archivo .bin del proyecto compilado</p>
    <form method='POST' action='/update' enctype='multipart/form-data'>
        <input type='file' name='update' accept='.bin'><br><br>
        <input type='submit' class='btn' value='Iniciar Actualización'>
    </form>
</div></body></html>
)";

WebOtaServer::WebOtaServer() {}

esp_err_t WebOtaServer::start() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 10240; 
    config.max_uri_handlers = 8;

    ESP_LOGI(TAG, "Iniciando servidor en puerto: %d", config.server_port);
    if (httpd_start(&_server, &config) == ESP_OK) {
        httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = index_get_handler };
        httpd_register_uri_handler(_server, &index_uri);

        httpd_uri_t update_uri = { .uri = "/update", .method = HTTP_POST, .handler = update_post_handler };
        httpd_register_uri_handler(_server, &update_uri);
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t WebOtaServer::index_get_handler(httpd_req_t *req) {
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t WebOtaServer::update_post_handler(httpd_req_t *req) {
    if (g_scr_enabled) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ERROR: SCR ACTIVOS - Detenga el equipo.");
        return ESP_FAIL;
    }

    // Buffer de 32KB -> Irá a PSRAM automáticamente
    char *buf = (char *)malloc(32768); 
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Sin memoria PSRAM disponible");
        return ESP_FAIL;
    }

    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    
    ESP_LOGI(TAG, "Iniciando escritura en partición: %s", update_partition->label);
    esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);

    int remaining = req->content_len;
    while (remaining > 0) {
        int received = httpd_req_recv(req, buf, std::min(remaining, 32768));
        if (received <= 0) {
            free(buf);
            esp_ota_abort(update_handle);
            return ESP_FAIL;
        }
        esp_ota_write(update_handle, buf, received);
        remaining -= received;
    }

    esp_ota_end(update_handle);
    esp_ota_set_boot_partition(update_partition);
    
    ESP_LOGI(TAG, "OTA Completo. Reiniciando...");
    httpd_resp_sendstr(req, "<html><body><h1>Exito. Reiniciando equipo...</h1></body></html>");
    
    free(buf);
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
    return ESP_OK;
}