#include "PortalWeb.hpp"
#include "WifiManager.hpp"
#include "GitHubClient.hpp"
#include "LoggerFS.hpp"
#include "esp_log.h"
#include "esp_http_server.h"
#include <string>
#include <algorithm>
#include "esp_app_format.h"
#include "esp_ota_ops.h"
#include "cJSON.h"

static const char *TAG = "PORTAL_WEB";

// Símbolos generados por CMake para el index.html embebido
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

// Variables globales externas (main.cpp)
extern float g_corriente_actual; 
extern int g_potenciometro_mv;   

int g_debug_adc_val = 0;  // Variable global que compartiremos con PortalWeb
int ws_fd = -1; 

extern "C" { extern volatile bool g_scr_enabled; }
extern LoggerFS g_logger; 


// Utilidad para decodificar caracteres especiales del WiFi (espacios, @, etc)
std::string urlDecode(std::string str) {
    std::string res;
    for (size_t i = 0; i < str.length(); ++i) {
        if (str[i] == '+') res += ' ';        
        else if (str[i] == '%' && i + 2 < str.length()) {
            char hex[3] = { str[i+1], str[i+2], 0 };
            res += (char)strtol(hex, nullptr, 16);
            i += 2;
        } 
        else res += str[i];
    }
    return res;
}



PortalWeb::PortalWeb() {}

// Función para enviar datos en tiempo real sin bloquear el servidor
void broadcast_debug_data(httpd_handle_t server) {
    if (ws_fd == -1 || server == NULL) return;

    char json[128];
    // Enviamos un tipo "debug" para procesarlo independientemente en JS
    snprintf(json, sizeof(json), "{\"type\":\"debug\",\"adc\":%d}", g_debug_adc_val);

    httpd_ws_frame_t ws_pkt = {};
    ws_pkt.payload = (uint8_t*)json;
    ws_pkt.len = strlen(json);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    // Envío asíncrono: no espera a que el cliente reciba para seguir procesando
    httpd_ws_send_frame_async(server, ws_fd, &ws_pkt);
}

esp_err_t PortalWeb::start() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.core_id = 0; // El servidor web se queda en el Core 0 con el WiFi
    config.lru_purge_enable = true;
    config.max_uri_handlers = 15; // Suficientes para todos los endpoints
    config.send_wait_timeout = 15;
    config.recv_wait_timeout = 15; // Añadido para estabilidad
    config.stack_size = 10240;
    config.task_priority = 2;      // Mayor prioridad para fluidez del portal

    ESP_LOGI(TAG, "Iniciando Servidor Web...");

    if (httpd_start(&_server, &config) == ESP_OK) {
        
        // --- 1. RUTA RAÍZ (index.html embebido) ---
        static httpd_uri_t uri_root = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = [](httpd_req_t *req) {
                const size_t size = (index_html_end - index_html_start);
                std::string s((const char*)index_html_start, size);
                
                const esp_app_desc_t *app_desc = esp_app_get_description();
                
                auto replace = [&](std::string& target, const std::string& placeholder, const std::string& value) {
                    size_t pos = 0;
                    while((pos = target.find(placeholder, pos)) != std::string::npos) {
                        target.replace(pos, placeholder.length(), value);
                        pos += value.length();
                    }
                };

                replace(s, "{{VERSION}}", app_desc->version);
                replace(s, "{{TITULO_EQUIPO}}", app_desc->project_name);

                httpd_resp_set_type(req, "text/html; charset=utf-8");
                return httpd_resp_send(req, s.c_str(), s.length());
            }
        };
        httpd_register_uri_handler(_server, &uri_root);

        // --- HANDLER WEBSOCKET ---
        static httpd_uri_t uri_ws = {
            .uri = "/ws",
            .method = HTTP_GET,
            .handler = [](httpd_req_t *req) {
                if (req->sess_ctx) return ESP_OK; 
                ws_fd = httpd_req_to_sockfd(req);
                ESP_LOGI("WS", "Cliente conectado: fd=%d", ws_fd);
                return ESP_OK;
            },
            .is_websocket = true,
            .handle_ws_control_frames = false // Silencia warning
        };
        httpd_register_uri_handler(_server, &uri_ws);

        // --- 3. AUDITORÍA: Obtener Logs ---
        static httpd_uri_t uri_get_logs = {
            .uri = "/get-logs",
            .method = HTTP_GET,
            .handler = [](httpd_req_t *req) {
                // Dentro del handler de "/get-logs"
                FILE* f = fopen("/sd/rect_log.csv", "r"); // Debe coincidir con el prefijo /sd
                if (!f) return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No hay logs");
                
                httpd_resp_set_type(req, "text/plain");
                char line[512];
                while (fgets(line, sizeof(line), f)) {
                    httpd_resp_sendstr_chunk(req, line);
                    vTaskDelay(pdMS_TO_TICKS(2));
                }
                fclose(f);
                return httpd_resp_sendstr_chunk(req, NULL);
            }
        };
        httpd_register_uri_handler(_server, &uri_get_logs);

        // --- 4. AUDITORÍA: Borrar Logs ---
        static httpd_uri_t uri_clear_logs = {
            .uri = "/clear-logs",
            .method = HTTP_POST,
            .handler = [](httpd_req_t *req) {
                g_logger.limpiarLog();
                return httpd_resp_sendstr(req, "Historial borrado");
            }
        };
        httpd_register_uri_handler(_server, &uri_clear_logs);

        // --- 5. WIFI: Escanear Redes ---
        static httpd_uri_t uri_scan = {
            .uri = "/scan",
            .method = HTTP_GET,
            .handler = [](httpd_req_t *req) {
                std::string json = WifiManager::scan_to_json();
                httpd_resp_set_type(req, "application/json");
                return httpd_resp_sendstr(req, json.c_str());
            }
        };
        httpd_register_uri_handler(_server, &uri_scan);

        // --- 6. WIFI: Guardar Configuración ---
        static httpd_uri_t uri_setwifi = {
            .uri = "/setwifi",
            .method = HTTP_POST,
            .handler = [](httpd_req_t *req) {
                char buf[256];
                int ret = httpd_req_recv(req, buf, std::min((size_t)req->content_len, sizeof(buf)-1));
                if (ret <= 0) return ESP_FAIL;
                buf[ret] = 0;

                std::string data(buf);
                size_t s_pos = data.find("ssid="), p_pos = data.find("&pass=");
                if (s_pos != std::string::npos && p_pos != std::string::npos) {
                    std::string ssid = urlDecode(data.substr(s_pos + 5, p_pos - (s_pos + 5)));
                    std::string pass = urlDecode(data.substr(p_pos + 6));
                    WifiManager::save_and_reconnect(ssid, pass);
                }
                return httpd_resp_sendstr(req, "Configuración recibida");
            }
        };
        httpd_register_uri_handler(_server, &uri_setwifi);

        // --- 7. OTA: Listar Versiones ---
        static httpd_uri_t uri_list_ota = {
            .uri = "/list-releases",
            .method = HTTP_GET,
            .handler = [](httpd_req_t *req) {
                std::string json = GitHubClient::get_releases_json();
                httpd_resp_set_type(req, "application/json");
                return httpd_resp_sendstr(req, json.c_str());
            }
        };
        httpd_register_uri_handler(_server, &uri_list_ota);

        // --- 8. OTA: Ejecutar Actualización ---
        static httpd_uri_t uri_do_ota = {
            .uri = "/do-update",
            .method = HTTP_GET,
            .handler = [](httpd_req_t *req) {
                char* query = nullptr;
                size_t q_len = httpd_req_get_url_query_len(req) + 1;
                if (q_len > 1) {
                    query = (char*)malloc(q_len);
                    if (httpd_req_get_url_query_str(req, query, q_len) == ESP_OK) {
                        char url[256];
                        if (httpd_query_key_value(query, "url", url, sizeof(url)) == ESP_OK) {
                            std::string dec_url = urlDecode(std::string(url));
                            g_logger.registrarEstructurado(RectEvent::OTA_START, dec_url, "Inicio OTA desde Web");
                            GitHubClient::start_ota_from_url(dec_url.c_str());
                            free(query);
                            return httpd_resp_sendstr(req, "Actualización iniciada");
                        }
                    }
                    free(query);
                }
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "URL inválida");
            }
        };
        httpd_register_uri_handler(_server, &uri_do_ota);

        return ESP_OK;
    }
    return ESP_FAIL;
}

void PortalWeb::stop() {
    if (_server) {
        httpd_stop(_server);
        _server = nullptr;
    }
}