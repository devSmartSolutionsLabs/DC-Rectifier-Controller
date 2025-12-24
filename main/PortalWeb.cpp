#include "PortalWeb.hpp"
#include "WifiManager.hpp"
#include "GitHubClient.hpp"
#include "esp_log.h"
#include "esp_http_server.h"
#include <string>
#include <algorithm>

static const char *TAG = "PORTAL_WEB";

// Variable externa para evitar accidentes: No actualizar si el SCR está disparando
extern "C" {
    extern volatile bool g_scr_enabled;
}

// HTML y JS con logs integrados para la consola del navegador
static const char* root_html = R"rawliteral(
<!DOCTYPE html><html><head><title>S3 Rectificador Pro</title>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<style>
    body { font-family: sans-serif; background: #1a1a1a; color: white; text-align: center; padding: 20px; }
    .container { max-width: 450px; margin: auto; background: #2d2d2d; padding: 20px; border-radius: 15px; box-shadow: 0 4px 15px rgba(0,0,0,0.5); }
    .btn { background: #007bff; color: white; border: none; padding: 12px; border-radius: 8px; cursor: pointer; width: 100%; margin: 10px 0; font-weight: bold; }
    .btn:disabled { background: #555; }
    .net-item { background: #3d3d3d; margin: 8px 0; padding: 12px; border-radius: 8px; cursor: pointer; text-align: left; transition: 0.3s; }
    .net-item:hover { background: #4d4d4d; border: 1px solid #007bff; }
    input { width: 100%; padding: 12px; margin: 10px 0; border-radius: 8px; border: none; box-sizing: border-box; background: #444; color: white; }
    hr { border: 0; border-top: 1px solid #555; margin: 20px 0; }
</style></head>
<body>
    <div class='container'>
        <h2>RECTIFICADOR S3</h2>
        <p style='color: #aaa; font-size: 0.9em;'>Panel de Control de Conectividad</p>
        
        <div id='wifi-section'>
            <h4>Configuración de Red</h4>
            <button class='btn' id='scan-btn' onclick='scan()'>ESCANEAR REDES WIFI</button>
            <div id='networks' style='max-height: 200px; overflow-y: auto;'></div>
            <input type='text' id='ssid' placeholder='SSID Seleccionado' readonly>
            <input type='password' id='pass' placeholder='Contraseña WiFi'>
            <button class='btn' style='background:#28a745' onclick='connect()'>GUARDAR Y CONECTAR</button>
        </div>

        <hr>

        <div id='ota-section'>
            <h4>Actualización de Firmware</h4>
            <button class='btn' style='background:#6f42c1' onclick='fetchReleases()'>BUSCAR EN GITHUB</button>
            <div id='releases'></div>
        </div>
    </div>

<script>
    function scan() {
        const btn = document.getElementById('scan-btn');
        const netDiv = document.getElementById('networks');
        btn.disabled = true;
        btn.innerText = "ESCANEANDO...";
        netDiv.innerHTML = "";
        console.log("Iniciando escaneo...");

        fetch('/scan').then(r => r.json()).then(data => {
            console.log("Redes encontradas:", data);
            if(data.length === 0) netDiv.innerHTML = "No se encontraron redes.";
            data.forEach(n => {
                let div = document.createElement('div');
                div.className = 'net-item';
                div.innerHTML = `<b>${n.s}</b> <small style='float:right'>${n.r} dBm</small>`;
                div.onclick = () => { document.getElementById('ssid').value = n.s; };
                netDiv.appendChild(div);
            });
        }).catch(err => {
            console.error("Error escaneando:", err);
            alert("Error al obtener lista de redes.");
        }).finally(() => {
            btn.disabled = false;
            btn.innerText = "ESCANEAR REDES WIFI";
        });
    }

    function connect() {
        const s = document.getElementById('ssid').value;
        const p = document.getElementById('pass').value;
        if(!s) return alert("Selecciona una red primero");
        
        console.log("Enviando credenciales para:", s);
        fetch('/setwifi', { 
            method: 'POST', 
            headers: {'Content-Type': 'application/x-www-form-urlencoded'},
            body: `ssid=${encodeURIComponent(s)}&pass=${encodeURIComponent(p)}` 
        }).then(r => {
            if(r.ok) alert("Configuración guardada. El equipo se reiniciará.");
        });
    }

    function fetchReleases() {
        const relDiv = document.getElementById('releases');
        relDiv.innerHTML = "Consultando GitHub...";
        fetch('/list-releases').then(r => r.json()).then(data => {
            let html = "";
            data.forEach(rel => {
                html += `<button class='btn' onclick="doUpdate('${rel.bin_url}')">INSTALAR ${rel.tag}</button>`;
            });
            relDiv.innerHTML = html;
        }).catch(e => { relDiv.innerHTML = "Error al conectar con GitHub"; });
    }

    function doUpdate(url) {
        if(confirm('¿Confirmar actualización? El sistema se detendrá.')) {
            fetch('/do-update?url=' + encodeURIComponent(url)).then(r => {
                if(r.ok) alert("OTA Iniciado. El equipo se reiniciará al finalizar.");
                else alert("Error: El SCR podría estar encendido.");
            });
        }
    }
</script></body></html>
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
            ESP_LOGI(TAG, "Petición: GET /");
            return httpd_resp_send(req, root_html, HTTPD_RESP_USE_STRLEN);
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
                json += "{\"tag\":\"" + releases[i].tag + "\",\"bin_url\":\"" + releases[i].bin_url + "\"}";
                if(i < releases.size() - 1) json += ",";
            }
            json += "]";
            
            httpd_resp_set_type(req, "application/json");
            return httpd_resp_send(req, json.c_str(), json.length());
        }, .user_ctx = NULL };
        httpd_register_uri_handler(_server, &uri_list);

        // --- RUTA 5: "/do-update" (Ejecución OTA Segura) ---
        httpd_uri_t uri_update = { .uri = "/do-update", .method = HTTP_GET, .handler = [](httpd_req_t *req){
            ESP_LOGI(TAG, "Petición: GET /do-update");
            
            if (g_scr_enabled) {
                ESP_LOGE(TAG, "Bloqueo OTA: El rectificador está activo.");
                return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Detenga el SCR antes de actualizar");
            }

            char url_buf[512]; // Buffer ampliado para URLs largas redirigidas de S3
            if (httpd_query_key_value(req->uri, "url", url_buf, sizeof(url_buf)) == ESP_OK) {
                ESP_LOGW(TAG, "Iniciando descarga OTA segura...");
                GitHubClient::start_ota_from_url(url_buf);
                return httpd_resp_sendstr(req, "Iniciando proceso de actualizacion...");
            }
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "URL faltante");
        }, .user_ctx = NULL };
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