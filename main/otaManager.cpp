#include "OtaManager.hpp"
#include "WifiManager.hpp"
#include "esp_ota_ops.h"
#include "esp_https_ota.h"
#include "cJSON.h"

static const char *TAG = "OTA_MGR";
extern volatile bool g_scr_enabled;

// HTML Dinámico (Simplificado para el ejemplo)
const char* ota_html_template = R"(
    <html><head><meta charset='UTF-8'><title>RECTIFICADOR S3</title></head>
    <body>
        <h1>Panel de Control Rectificador</h1>
        <div id='wifi-section'>
            <h3>Configurar WiFi</h3>
            <form action='/setwifi' method='POST'>
                SSID: <input name='ssid'><br>Pass: <input name='pass' type='password'><br>
                <input type='submit' value='Guardar y Conectar'>
            </form>
        </div>
        <div id='github-section'>
            <h3>Versiones en GitHub</h3>
            <button onclick='loadReleases()'>Buscar Actualizaciones</button>
            <ul id='release-list'></ul>
        </div>
        <script>
            function loadReleases() {
                fetch('/list-releases').then(r => r.json()).then(data => {
                    let list = document.getElementById('release-list');
                    data.forEach(rel => {
                        list.innerHTML += `<li>${rel.tag} - <a href='#' onclick="update('${rel.url}')">Instalar</a></li>`;
                    });
                });
            }
            function update(url) { 
                if(confirm('¿Seguro?')) fetch('/do-update?url=' + btoa(url)); 
            }
        </script>
    </body></html>
)";

// Handler para listar versiones de GitHub
esp_err_t OtaManager::github_list_handler(httpd_req_t *req) {
    if(!WifiManager::is_connected()) {
        httpd_resp_sendstr(req, "[]");
        return ESP_OK;
    }

    // Aquí usamos esp_http_client para pedir a: 
    // https://api.github.com/repos/tu_usuario/tu_repo/releases
    // Procesamos el JSON usando cJSON y lo enviamos al cliente web.
    // Usamos PSRAM para el buffer del JSON
    char* json_buffer = (char*)heap_caps_malloc(10240, MALLOC_CAP_SPIRAM);
    // ... lógica de petición HTTP ...
    httpd_resp_sendstr(req, json_buffer);
    free(json_buffer);
    return ESP_OK;
}

void OtaManager::start_ota_update(const std::string& url) {
    if (g_scr_enabled) {
        ESP_LOGE(TAG, "No se puede actualizar con SCR encendidos");
        return;
    }

    esp_http_client_config_t config = {};
    config.url = url.c_str();
    config.cert_pem = NULL; // En producción, poner el Root CA de GitHub
    
    esp_https_ota_config_t ota_config = {};
    ota_config.http_config = &config;

    ESP_LOGI(TAG, "Iniciando descarga OTA...");
    esp_https_ota(&ota_config);
}