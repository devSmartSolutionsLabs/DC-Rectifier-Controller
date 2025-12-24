#include "PortalWeb.hpp"
#include "WifiManager.hpp"
#include "GitHubClient.hpp"
#include "esp_log.h"
#include "esp_http_server.h"
#include <string>
#include <algorithm>
#include "esp_app_format.h"
#include "esp_ota_ops.h"
static const char *TAG = "PORTAL_WEB";

// Variable externa para evitar accidentes: No actualizar si el SCR está disparando
extern "C" {
    extern volatile bool g_scr_enabled;
}

// HTML y JS con logs integrados para la consola del navegador
static const char* root_html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>S3 Rectificador Pro - v%s</title>
    <meta name='viewport' content='width=device-width, initial-scale=1'>
    <style>
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #121212; color: #e0e0e0; text-align: center; padding: 20px; margin: 0; }
        .container { max-width: 500px; margin: auto; background: #1e1e1e; padding: 25px; border-radius: 15px; box-shadow: 0 10px 30px rgba(0,0,0,0.7); border: 1px solid #333; }
        
        h2 { color: #ffffff; margin-bottom: 5px; text-transform: uppercase; letter-spacing: 2px; }
        .version-tag { display: inline-block; background: #333; color: #00ff00; padding: 4px 12px; border-radius: 20px; font-family: monospace; font-size: 0.9em; border: 1px solid #00ff00; margin-bottom: 20px; }
        
        .section-title { font-size: 1.1em; font-weight: bold; color: #888; margin: 25px 0 15px 0; text-align: left; border-bottom: 1px solid #333; padding-bottom: 5px; }
        
        .btn { background: #007bff; color: white; border: none; padding: 14px; border-radius: 8px; cursor: pointer; width: 100%; margin: 10px 0; font-weight: bold; font-size: 1em; transition: 0.3s; }
        .btn:hover { background: #0056b3; transform: translateY(-2px); }
        .btn:disabled { background: #444; cursor: not-allowed; transform: none; }
        
        input { width: 100%; padding: 14px; margin: 10px 0; border-radius: 8px; border: 1px solid #333; box-sizing: border-box; background: #2a2a2a; color: white; font-size: 1em; }
        
        /* Listado de Redes WiFi */
        #networks { max-height: 220px; overflow-y: auto; background: #161616; border-radius: 8px; margin-bottom: 15px; border: 1px solid #222; }
        .net-item { display: flex; justify-content: space-between; padding: 12px; border-bottom: 1px solid #222; cursor: pointer; transition: 0.2s; }
        .net-item:hover { background: #252525; color: #007bff; }
        
        /* Listado de Releases de GitHub */
        .rel-item { background: #252525; padding: 15px; border-radius: 10px; margin-bottom: 10px; display: flex; justify-content: space-between; align-items: center; border: 1px solid #333; }
        .rel-info { text-align: left; }
        .rel-tag { font-weight: bold; font-size: 1.1em; display: block; }
        .new-badge { background: #00ff00; color: #000; font-size: 0.7em; padding: 2px 6px; border-radius: 4px; font-weight: bold; vertical-align: middle; margin-left: 5px; }
        
        /* Animación para el botón de actualización disponible */
        .update-ready { background: #28a745 !important; border: 2px solid #00ff00 !important; animation: pulse 2s infinite; }
        @keyframes pulse {
            0% { box-shadow: 0 0 0 0 rgba(0, 255, 0, 0.4); }
            70% { box-shadow: 0 0 0 10px rgba(0, 255, 0, 0); }
            100% { box-shadow: 0 0 0 0 rgba(0, 255, 0, 0); }
        }

        .loader { border: 3px solid #f3f3f3; border-top: 3px solid #007bff; border-radius: 50%; width: 20px; height: 20px; animation: spin 1s linear infinite; display: inline-block; vertical-align: middle; margin-right: 10px; }
        @keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }
        .hidden { display: none; }
    </style>
</head>
<body>
    <div class='container'>
        <h2>RECTIFICADOR S3</h2>
        <div class='version-tag'>Sistema: v%s</div>

        <div class='section-title'>CONEXIÓN WIFI</div>
        <button class='btn' id='scan-btn' onclick='scan()'>ESCANEAR REDES</button>
        <div id='networks'></div>
        <input type='text' id='ssid' placeholder='SSID Seleccionado' readonly>
        <input type='password' id='pass' placeholder='Contraseña de la red'>
        <button class='btn' style='background:#28a745' onclick='connect()'>GUARDAR Y CONECTAR</button>

        <div class='section-title'>FIRMWARE GITHUB</div>
        <button class='btn' style='background:#6f42c1' id='gh-btn' onclick='fetchReleases()'>BUSCAR ACTUALIZACIONES</button>
        <div id='releases-list'></div>
    </div>

<script>
    function scan() {
        const btn = document.getElementById('scan-btn');
        btn.disabled = true; btn.innerHTML = "<div class='loader'></div> ESCANEANDO...";
        fetch('/scan').then(r => r.json()).then(data => {
            const netDiv = document.getElementById('networks');
            netDiv.innerHTML = "";
            data.forEach(n => {
                let div = document.createElement('div');
                div.className = 'net-item';
                div.innerHTML = `<span><b>${n.s}</b></span> <span>${n.r} dBm</span>`;
                div.onclick = () => { document.getElementById('ssid').value = n.s; };
                netDiv.appendChild(div);
            });
        }).finally(() => { 
            btn.disabled = false; btn.innerText = "ESCANEAR REDES"; 
        });
    }

    function connect() {
        const s = document.getElementById('ssid').value;
        const p = document.getElementById('pass').value;
        if(!s) return alert("Selecciona una red");
        fetch('/setwifi', { 
            method: 'POST', 
            headers: {'Content-Type': 'application/x-www-form-urlencoded'},
            body: `ssid=${encodeURIComponent(s)}&pass=${encodeURIComponent(p)}` 
        }).then(r => { if(r.ok) alert("Reiniciando para conectar..."); });
    }

    function fetchReleases() {
        const btn = document.getElementById('gh-btn');
        const relDiv = document.getElementById('releases-list');
        btn.disabled = true; btn.innerHTML = "<div class='loader'></div> CONSULTANDO...";
        
        fetch('/list-releases').then(r => r.json()).then(data => {
            relDiv.innerHTML = "";
            if(data.length === 0) relDiv.innerHTML = "<p>No hay releases.</p>";
            data.forEach(rel => {
                let div = document.createElement('div');
                div.className = 'rel-item';
                let btnClass = rel.new ? 'btn update-ready' : 'btn';
                let newBadge = rel.new ? "<span class='new-badge'>NUEVA</span>" : "";
                
                div.innerHTML = `
                    <div class='rel-info'>
                        <span class='rel-tag'>${rel.tag}${newBadge}</span>
                    </div>
                    <button class='${btnClass}' style='width:auto; margin:0;' onclick="doUpdate('${rel.bin_url}')">
                        ${rel.new ? 'ACTUALIZAR' : 'REINSTALAR'}
                    </button>
                `;
                relDiv.appendChild(div);
            });
        }).finally(() => { 
            btn.disabled = false; btn.innerText = "BUSCAR ACTUALIZACIONES"; 
        });
    }

    function doUpdate(url) {
        if(confirm("¿Deseas iniciar la actualización ahora? El sistema se detendrá.")) {
            fetch('/do-update?url=' + encodeURIComponent(url)).then(r => {
                if(r.ok) alert("OTA en curso. El equipo se reiniciará al finalizar.");
                else alert("Error al iniciar OTA. Verifique conexión o SCR.");
            });
        }
    }
</script>
</body>
</html>
)rawliteral";


PortalWeb::PortalWeb() : _server(NULL) {}

esp_err_t PortalWeb::start() {
    ESP_LOGI(TAG, "Configurando servidor para repo devSmartSolutionsLabs...");
    
    // Logs de memoria para diagnóstico de estabilidad
    ESP_LOGI(TAG, "RAM interna libre: %lu bytes", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "PSRAM libre: %lu bytes", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80; 
    config.stack_size = 25600;      // Aumentado para handshake SSL de GitHub y JSON pesado
    config.max_uri_handlers = 12;
    config.lru_purge_enable = true;
    config.max_open_sockets = 7;

    if (httpd_start(&_server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Servidor iniciado. Registrando rutas...");

        // --- RUTA 1: "/" (Panel de Control) ---
        httpd_uri_t uri_root = { .uri = "/", .method = HTTP_GET, .handler = [](httpd_req_t *req){
            const esp_app_desc_t *app_desc = esp_app_get_description();
            
            // Reservamos memoria para el HTML final inyectando la versión
            char* final_html = (char*)heap_caps_malloc(strlen(root_html) + 128, MALLOC_CAP_SPIRAM);
            if(!final_html) return ESP_FAIL;

            // Inyectamos app_desc->version en el %s del HTML
            sprintf(final_html, root_html, app_desc->version);
            
            httpd_resp_send(req, final_html, HTTPD_RESP_USE_STRLEN);
            heap_caps_free(final_html);
            return ESP_OK;
        }, .user_ctx = NULL };
        httpd_register_uri_handler(_server, &uri_root);

        // --- RUTA 2: "/scan" (Escaneo de Redes) ---
        httpd_uri_t uri_scan = { .uri = "/scan", .method = HTTP_GET, .handler = [](httpd_req_t *req){
            ESP_LOGI(TAG, "Petición: GET /scan");
            std::string json = WifiManager::scan_to_json();
            httpd_resp_set_type(req, "application/json");
            return httpd_resp_send(req, json.c_str(), json.length());
        }, .user_ctx = NULL };
        httpd_register_uri_handler(_server, &uri_scan);

        // --- RUTA 3: "/setwifi" (Configuración WiFi - POST) ---
        httpd_uri_t uri_setwifi = { 
            .uri = "/setwifi", 
            .method = HTTP_POST, 
            .handler = [](httpd_req_t *req){
                char buf[256];
                int ret = httpd_req_recv(req, buf, req->content_len);
                if (ret <= 0) return ESP_FAIL;
                buf[ret] = '\0';
                
                std::string data(buf);
                ESP_LOGI(TAG, "Datos POST recibidos: %s", buf);

                size_t ssid_pos = data.find("ssid=");
                size_t pass_pos = data.find("&pass=");
                
                if (ssid_pos != std::string::npos && pass_pos != std::string::npos) {
                    std::string ssid_raw = data.substr(ssid_pos + 5, pass_pos - (ssid_pos + 5));
                    std::string pass_raw = data.substr(pass_pos + 6);

                    // Lambda para decodificación URL (ej: %20 -> espacio)
                    auto decode = [](std::string str) {
                        std::string res;
                        for (size_t i = 0; i < str.length(); ++i) {
                            if (str[i] == '%' && i + 2 < str.length()) {
                                res += (char)std::strtol(str.substr(i + 1, 2).c_str(), nullptr, 16);
                                i += 2;
                            } else if (str[i] == '+') res += ' ';
                            else res += str[i];
                        }
                        return res;
                    };

                    std::string final_ssid = decode(ssid_raw);
                    std::string final_pass = decode(pass_raw);

                    ESP_LOGW(TAG, "Guardando WiFi -> SSID: %s", final_ssid.c_str());

                    httpd_resp_sendstr(req, "Configuracion guardada correctamente. Reiniciando...");
                    
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    WifiManager::save_and_reconnect(final_ssid, final_pass);
                    return ESP_OK;
                }
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Formato invalido");
            }, 
            .user_ctx = NULL 
        };
        httpd_register_uri_handler(_server, &uri_setwifi);

        // --- RUTA 4: "/list-releases" (Consulta Automática al Repo Oficial) ---
        httpd_uri_t uri_list = { .uri = "/list-releases", .method = HTTP_GET, .handler = [](httpd_req_t *req){
            ESP_LOGI(TAG, "Consultando actualizaciones en GitHub...");
            
            // Llamamos con string vacío para usar REPO_PATH por defecto definido en GitHubClient
            auto releases = GitHubClient::get_releases("devSmartSolutionsLabs/DC-Rectifier-Controller");
            
            if (releases.empty()) {
                return httpd_resp_sendstr(req, "[]");
            }

            std::string json = "[";
                for(size_t i = 0; i < releases.size(); ++i) {
                    json += "{\"tag\":\"" + releases[i].tag + 
                            "\",\"bin_url\":\"" + releases[i].bin_url + 
                            "\",\"new\":" + (releases[i].is_new ? "true" : "false") + "}";
                    if(i < releases.size() - 1) json += ",";
                }
                json += "]";
            
            httpd_resp_set_type(req, "application/json");
            return httpd_resp_send(req, json.c_str(), json.length());
        }, .user_ctx = NULL };
        httpd_register_uri_handler(_server, &uri_list);

        // --- RUTA 5: "/do-update" (Ejecución OTA Segura) ---
        httpd_uri_t uri_update = { 
            .uri = "/do-update", 
            .method = HTTP_GET, 
            .handler = [](httpd_req_t *req){
                ESP_LOGI(TAG, "Petición: GET /do-update");

                if (g_scr_enabled) {
                    return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Detenga el SCR antes de actualizar");
                }

                // 1. Obtener la longitud de la query string
                size_t query_len = httpd_req_get_url_query_len(req) + 1;
                if (query_len <= 1) {
                    ESP_LOGE(TAG, "No hay parámetros en la URL");
                    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "URL faltante");
                }

                // 2. Extraer la query string en PSRAM (por su tamaño)
                char* query_str = (char*)heap_caps_malloc(query_len, MALLOC_CAP_SPIRAM);
                if (!query_str) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memoria insuficiente");

                httpd_req_get_url_query_str(req, query_str, query_len);

                // 3. Extraer el valor de "url"
                char* url_buf = (char*)heap_caps_malloc(1024, MALLOC_CAP_SPIRAM); // Buffer de 1KB para la URL
                esp_err_t res = httpd_query_key_value(query_str, "url", url_buf, 1024);
                
                heap_caps_free(query_str); // Liberamos la query inicial

                if (res == ESP_OK) {
                    // Decodificar la URL (quitar %2F, %3A, etc.)
                    auto url_decode = [](char *dst, const char *src) {
                        char a, b;
                        while (*src) {
                            if ((*src == '%') && ((a = src[1]) && (b = src[2])) && (isxdigit(a) && isxdigit(b))) {
                                if (a >= 'a') a -= 'a' - 'A';
                                if (a >= 'A') a -= ('A' - 10); else a -= '0';
                                if (b >= 'a') b -= 'a' - 'A';
                                if (b >= 'A') b -= ('A' - 10); else b -= '0';
                                *dst++ = 16 * a + b;
                                src += 3;
                            } else if (*src == '+') {
                                *dst++ = ' ';
                                src++;
                            } else {
                                *dst++ = *src++;
                            }
                        }
                        *dst++ = '\0';
                    };

                    char* decoded_url = (char*)heap_caps_malloc(1024, MALLOC_CAP_SPIRAM);
                    url_decode(decoded_url, url_buf);
                    heap_caps_free(url_buf);

                    ESP_LOGW(TAG, "Iniciando OTA con URL: %s", decoded_url);
                    
                    // Enviamos respuesta antes de empezar el proceso pesado
                    httpd_resp_sendstr(req, "Actualización iniciada. El equipo se reiniciará al finalizar.");
                    
                    // Lanzamos el OTA
                    GitHubClient::start_ota_from_url(decoded_url);
                    
                    heap_caps_free(decoded_url);
                    return ESP_OK;
                }

                heap_caps_free(url_buf);
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Parametro URL no encontrado");
            }, 
            .user_ctx = NULL 
        };
        
        httpd_register_uri_handler(_server, &uri_update);

        return ESP_OK;
    }
    ESP_LOGE(TAG, "Error al iniciar el servidor HTTP.");
    return ESP_FAIL;
}

void PortalWeb::stop() {
    if (_server) {
        ESP_LOGW(TAG, "Deteniendo servidor.");
        httpd_stop(_server);
        _server = NULL;
    }
}