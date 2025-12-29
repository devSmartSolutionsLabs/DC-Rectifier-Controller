#include "GitHubClient.hpp"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "cJSON.h"
#include "esp_heap_caps.h"
#include "esp_crt_bundle.h"
#include "esp_ota_ops.h"
#include <sstream>
#include <vector>
#include <cstdlib> // Necesario para atoi
#include "LoggerFS.hpp"
static const char *TAG = "GH_CLIENT";

// URL base de tu repositorio
#define REPO_PATH "devSmartSolutionsLabs/DC-Rectifier-Controller"

extern "C" {
    extern volatile bool g_scr_enabled;
}
extern LoggerFS g_logger;

static char* github_url_to_save = nullptr;


std::string GitHubClient::get_current_version() {
    return esp_app_get_description()->version;
}

bool GitHubClient::is_newer_version(std::string remote, std::string local) {
    // 1. Limpiar la 'v' si existe
    if (!remote.empty() && remote[0] == 'v') remote.erase(0, 1);
    if (!local.empty() && local[0] == 'v') local.erase(0, 1);

    // 2. Si son iguales, no es nueva
    if (remote == local) return false;

    // 3. Trocear las versiones por los puntos manualmente sin excepciones
    auto split = [](const std::string& s) {
        std::vector<int> res;
        std::string part;
        std::stringstream ss(s);
        while (std::getline(ss, part, '.')) {
            // Usamos atoi que es seguro y no lanza excepciones
            res.push_back(atoi(part.c_str())); 
        }
        // Asegurar que siempre tengamos al menos 3 componentes (Major.Minor.Patch)
        while(res.size() < 3) res.push_back(0);
        return res;
    };

    std::vector<int> v_rem = split(remote);
    std::vector<int> v_loc = split(local);

    // 4. Comparar jerárquicamente
    for (size_t i = 0; i < 3; i++) {
        if (v_rem[i] > v_loc[i]) return true;  // La remota es mayor
        if (v_rem[i] < v_loc[i]) return false; // La remota es menor
    }

    return false;
}

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
    config.disable_auto_redirect = false; // Permitir que siga a Amazon S3
    config.max_redirection_count = 5;      // Darle margen de saltos

    config.buffer_size_tx = 4096;
    config.buffer_size = 10240; 

    esp_https_ota_config_t ota_config = {};
    ota_config.http_config = &config;

    g_logger.registrarEstructurado(RectEvent::OTA_START, "v3.0.2", "Descargando desde GitHub");

    esp_err_t ret = esp_https_ota(&ota_config);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Actualización completada con éxito. Reiniciando...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
    } else {
        char err_buf[32];
        snprintf(err_buf, sizeof(err_buf), "Error:0x%X", ret);
        g_logger.registrarEstructurado(RectEvent::ERR_SYSTEM, err_buf, "Fallo en descarga OTA");
        ESP_LOGE(TAG, "Error durante el proceso OTA: %s", esp_err_to_name(ret));
    }

    if (github_url_to_save) {
        free(github_url_to_save);
        github_url_to_save = nullptr;
    }
    vTaskDelete(NULL);
}

std::string GitHubClient::get_releases_json() {
    // 1. Obtener el vector de versiones usando el método que ya tienes
    std::vector<ReleaseInfo> releases = GitHubClient::get_releases(REPO_PATH);
    
    // 2. Crear objeto raíz JSON (un array)
    cJSON *root = cJSON_CreateArray();
    
    for (const auto& rel : releases) {
        cJSON *item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "tag", rel.tag.c_str());
        cJSON_AddStringToObject(item, "bin_url", rel.bin_url.c_str());
        cJSON_AddBoolToObject(item, "new", rel.is_new);
        cJSON_AddItemToArray(root, item);
    }
    
    // 3. Convertir a string y liberar memoria de cJSON
    char *rendered = cJSON_PrintUnformatted(root);
    std::string out(rendered);
    
    cJSON_Delete(root);
    free(rendered);
    
    return out;
}

std::vector<ReleaseInfo> GitHubClient::get_releases(const char* repo) {
    std::vector<ReleaseInfo> list;
    
    // 1. Obtener la versión que el ESP32 tiene grabada actualmente (vía CMake PROJECT_VER)
    const esp_app_desc_t *app_desc = esp_app_get_description();
    std::string current_version = app_desc->version;
    ESP_LOGI(TAG, "Versión local detectada: %s", current_version.c_str());

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
    config.skip_cert_common_name_check = true; 

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
                    std::string remote_tag = tag->valuestring;
                    
                    cJSON *asset = NULL;
                    cJSON_ArrayForEach(asset, assets) {
                        cJSON *name = cJSON_GetObjectItem(asset, "name");
                        cJSON *bin_url = cJSON_GetObjectItem(asset, "browser_download_url");
                        
                        if (cJSON_IsString(name) && strstr(name->valuestring, ".bin") && cJSON_IsString(bin_url)) {
                            
                            // 2. Lógica de comparación: ¿Es diferente a la actual?
                            // Si el tag de GitHub no es igual al que tenemos grabado, es "new"
                            bool is_new = is_newer_version(remote_tag, current_version);
                            
                            // Agregamos a la lista con la info completa
                            list.push_back({remote_tag, bin_url->valuestring, is_new});
                            break; 
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
    
    ESP_LOGI(TAG, "Se encontraron %d versiones. Local: %s", (int)list.size(), current_version.c_str());
    return list;
}