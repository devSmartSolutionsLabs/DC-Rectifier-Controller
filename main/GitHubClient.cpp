#include "GitHubClient.hpp"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "cJSON.h"
#include "esp_heap_caps.h"
#include "esp_crt_bundle.h"

static const char *TAG = "GH_CLIENT";

// URL base de tu repositorio
#define REPO_PATH "devSmartSolutionsLabs/DC-Rectifier-Controller"

extern "C" {
    extern volatile bool g_scr_enabled;
}

static char* github_url_to_save = nullptr;

void GitHubClient::start_ota_from_url(const char* url) {
    if (github_url_to_save != nullptr) free(github_url_to_save);
    github_url_to_save = strdup(url);
    
    // Tarea en Core 1 para proteger el tiempo real del Core 0 (SCR)
    xTaskCreatePinnedToCore(&GitHubClient::ota_task, "ota_task", 10240, NULL, 5, NULL, 1);
}

void GitHubClient::ota_task(void* pvParameter) {
    ESP_LOGI(TAG, "Iniciando descarga OTA desde: %s", github_url_to_save);

    esp_http_client_config_t config = {};
    config.url = github_url_to_save;
    config.crt_bundle_attach = esp_crt_bundle_attach; 
    config.keep_alive_enable = true;
    config.timeout_ms = 20000;
    
   // CORRECCIÓN AQUÍ: 
    // En ESP-IDF v5.x, las redirecciones se manejan así:
    // Por defecto vienen habilitadas, pero si quieres asegurar:
    // config.disable_auto_redirect = false; 

    config.buffer_size_tx = 4096;
    config.buffer_size = 10240; 

    esp_https_ota_config_t ota_config = {};
    ota_config.http_config = &config;

    esp_err_t ret = esp_https_ota(&ota_config);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Actualización completada con éxito. Reiniciando...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
    } else {
        ESP_LOGE(TAG, "Error durante el proceso OTA: %s", esp_err_to_name(ret));
    }

    if (github_url_to_save) {
        free(github_url_to_save);
        github_url_to_save = nullptr;
    }
    vTaskDelete(NULL);
}

std::vector<ReleaseInfo> GitHubClient::get_releases(const char* repo) {
    std::vector<ReleaseInfo> list;
    
    // Usamos el repo por defecto si no se pasa uno
    const char* target_repo = (repo && strlen(repo) > 0) ? repo : REPO_PATH;
    
    const size_t BUF_SIZE = 32768;
    char* json_buf = (char*)heap_caps_malloc(BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (!json_buf) {
        ESP_LOGE(TAG, "Error: Memoria insuficiente en PSRAM");
        return list;
    }

    esp_http_client_config_t config = {};
    char api_url[150];
    snprintf(api_url, sizeof(api_url), "https://api.github.com/repos/%s/releases", target_repo);
    
    config.url = api_url;
    config.method = HTTP_METHOD_GET;
    config.user_agent = "ESP32-S3-Rectificador-v1";
    config.crt_bundle_attach = esp_crt_bundle_attach;
    config.timeout_ms = 15000;
    config.skip_cert_common_name_check = true; // Ignora el nombre del host
    config.cert_pem = NULL; // No cargamos certificado manual

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Accept", "application/vnd.github.v3+json");

    esp_err_t err = esp_http_client_open(client, 0);
    if (err == ESP_OK) {
        esp_http_client_fetch_headers(client);
        int total_read = 0;
        int read_now = 0;

        while (total_read < (BUF_SIZE - 1)) {
            read_now = esp_http_client_read(client, json_buf + total_read, BUF_SIZE - total_read - 1);
            if (read_now <= 0) break;
            total_read += read_now;
        }
        json_buf[total_read] = '\0';

        cJSON *root = cJSON_Parse(json_buf);
        if (root) {
            cJSON *release = NULL;
            cJSON_ArrayForEach(release, root) {
                cJSON *tag = cJSON_GetObjectItem(release, "tag_name");
                cJSON *assets = cJSON_GetObjectItem(release, "assets");
                
                if (cJSON_IsString(tag) && cJSON_IsArray(assets)) {
                    // Buscamos el primer archivo .bin dentro de los assets
                    cJSON *asset = NULL;
                    cJSON_ArrayForEach(asset, assets) {
                        cJSON *name = cJSON_GetObjectItem(asset, "name");
                        cJSON *bin_url = cJSON_GetObjectItem(asset, "browser_download_url");
                        
                        // Validamos que el nombre termine en .bin para no descargar el código fuente por error
                        if (cJSON_IsString(name) && strstr(name->valuestring, ".bin") && cJSON_IsString(bin_url)) {
                            list.push_back({tag->valuestring, bin_url->valuestring});
                            break; // Solo tomamos el primer binario encontrado por versión
                        }
                    }
                }
            }
            cJSON_Delete(root);
        }
    } else {
        ESP_LOGE(TAG, "Error de red al consultar releases: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    heap_caps_free(json_buf);
    
    ESP_LOGI(TAG, "Se encontraron %d versiones disponibles en GitHub.", (int)list.size());
    return list;
}