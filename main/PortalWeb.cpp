#include "PortalWeb.hpp"
#include "WifiManager.hpp"
#include "GitHubClient.hpp"
#include "esp_log.h"
#include "esp_http_server.h"
#include <string>
#include <algorithm>
#include "esp_app_format.h"
#include "esp_ota_ops.h"
#include "cJSON.h"

static const char *TAG = "PORTAL_WEB";

// Variables globales (main.cpp)
extern float g_corriente_actual; 
extern int g_potenciometro_mv;   
extern bool g_scr_activo;
extern "C" { extern volatile bool g_scr_enabled; }

static int ws_fd = -1; 
static httpd_handle_t g_server_handle = NULL;

// --- HTML CON WEBSOCKET ACTIVO EN TODO MOMENTO ---
static const char* root_html = R"rawliteral(
<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'>
<title>S3 Rectificador Pro - v{{VERSION}}</title>
<style>
    body { font-family: 'Segoe UI', sans-serif; background: #121212; color: #e0e0e0; text-align: center; margin: 0; padding-bottom: 30px; }
    .nav { display: flex; background: #1e1e1e; position: sticky; top: 0; border-bottom: 1px solid #333; z-index: 100; }
    .nav button { flex: 1; padding: 15px; border: none; background: none; color: #888; cursor: pointer; font-weight: bold; font-size: 14px; }
    .nav button.active { color: #00ff00; border-bottom: 3px solid #00ff00; background: #252525; }
    .container { max-width: 500px; margin: 15px auto; padding: 15px; }
    .card { background: #1e1e1e; padding: 20px; border-radius: 15px; border: 1px solid #333; margin-bottom: 15px; }
    .tab-content { display: none; }
    .tab-content.active { display: block; }
    .val-box { display: flex; justify-content: space-around; margin-bottom: 15px; }
    .val-num { font-size: 2.2em; color: #00ff00; font-family: monospace; font-weight: bold; }
    canvas { background: #000; width: 100%; border-radius: 10px; border: 1px solid #444; }
    .btn { background: #007bff; color: white; border: none; padding: 14px; border-radius: 8px; cursor: pointer; width: 100%; margin: 10px 0; font-weight: bold; }
    .btn:hover { background: #0056b3; }
    .btn.update-ready { background: #28a745 !important; animation: pulse 2s infinite; }
    @keyframes pulse { 0% { box-shadow: 0 0 0 0 rgba(0,255,0,0.4); } 70% { box-shadow: 0 0 0 10px rgba(0,255,0,0); } 100% { box-shadow: 0 0 0 0 rgba(0,255,0,0); } }
    #networks { max-height: 180px; overflow-y: auto; background: #161616; margin: 10px 0; border-radius: 8px; text-align: left; border: 1px solid #222; }
    .net-item { padding: 12px; border-bottom: 1px solid #222; cursor: pointer; display: flex; justify-content: space-between; }
    .new-badge { background: #00ff00; color: #000; font-size: 0.7em; padding: 2px 5px; border-radius: 4px; margin-left: 5px; font-weight: bold; }
    hr { border: 0; border-top: 1px solid #333; margin: 20px 0; }
    input { width: 90%; padding: 12px; margin: 5px 0; background: #2a2a2a; color: white; border: 1px solid #444; border-radius: 5px; font-size: 16px; }
    /* Spinner giratorio */
    .spinner {
        border: 3px solid rgba(255, 255, 255, 0.3);
        border-radius: 50%;
        border-top: 3px solid #00ff00;
        width: 16px;
        height: 16px;
        animation: spin 1s linear infinite;
        display: inline-block;
        vertical-align: middle;
        margin-right: 8px;
    }
    @keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }
</style>
</head><body>
    <div class='nav'>
        <button id='t-dash' class='active' onclick="tab('dash')">MONITOR</button>
        <button id='t-sys' onclick="tab('sys')">CONFIG / OTA</button>
    </div>
    <div class='container'>
        <div id='dash' class='tab-content active'><div class='card'>
            <div class='val-box'>
                <div><small style='color:#888'>CORRIENTE</small><div id='v-amp' class='val-num'>0.0A</div></div>
                <div><small style='color:#888'>POTENCIA</small><div id='v-pot' class='val-num'>0mV</div></div>
            </div>
            <canvas id='chart' height='160'></canvas>
            <div id='v-scr' style='margin-top:15px; font-family: monospace;'>ESTADO SCR: ---</div>
        </div></div>

        <div id='sys' class='tab-content'><div class='card'>
            <div style='text-align:left; color:#888; font-weight:bold; font-size:12px;'>WIFI MANAGER</div>
            <button class='btn' onclick='scan()'>BUSCAR REDES</button>
            <div id='networks'></div>
            <input type='text' id='ssid' placeholder='SSID Seleccionado'>
            <input type='password' id='pass' placeholder='Contraseña'>
            <button class='btn' style='background:#28a745' onclick='connect()'>GUARDAR WIFI</button>
            <hr>
            <div style='text-align:left; color:#888; font-weight:bold; font-size:12px;'>ACTUALIZACIÓN GITHUB</div>
            <button class='btn' style='background:#6f42c1' onclick='fetchReleases()'>VERIFICAR VERSIONES</button>
            <div id='releases-list'></div>
        </div></div>
    </div>
<script>
    // Navegación de pestañas
    function tab(id){
        document.querySelectorAll('.tab-content').forEach(t=>t.classList.remove('active'));
        document.querySelectorAll('.nav button').forEach(b=>b.classList.remove('active'));
        document.getElementById(id).classList.add('active');
        document.getElementById('t-'+id).classList.add('active');
    }

    // --- WEBSOCKET REAL-TIME ---
    let dataPoints = Array(60).fill(0);
    let canvas = document.getElementById('chart');
    let ctx = canvas.getContext('2d');
    let ws = new WebSocket(`ws://${location.host}/ws`);

    ws.onmessage = (e) => {
        let d = JSON.parse(e.data);
        document.getElementById('v-amp').innerText = d.amp.toFixed(1) + "A";
        document.getElementById('v-pot').innerText = d.pot + "mV";
        document.getElementById('v-scr').innerText = "ESTADO SCR: " + (d.scr ? "ACTIVO" : "INACTIVO");
        dataPoints.push(d.amp); dataPoints.shift(); 
        draw();
    };

    function draw() {
        ctx.clearRect(0,0,canvas.width,canvas.height);
        ctx.strokeStyle='#00ff00'; ctx.lineWidth=2; ctx.beginPath();
        dataPoints.forEach((p,i)=>{
            let x=(i/59)*canvas.width; 
            let y=canvas.height-(p/5000*canvas.height); // Escala 5000A
            if(i===0)ctx.moveTo(x,y); else ctx.lineTo(x,y);
        }); ctx.stroke();
    }

    // --- WIFI & OTA FUNCTIONS ---
    function scan(){ fetch('/scan').then(r=>r.json()).then(data=>{
        let d=document.getElementById('networks'); d.innerHTML="";
        data.forEach(n=>{ let i=document.createElement('div'); i.className='net-item'; i.innerHTML=`<span>${n.s}</span><span>${n.r}dBm</span>`; i.onclick=()=>document.getElementById('ssid').value=n.s; d.appendChild(i);});
    });}

    function connect(){ 
        const body = `ssid=${encodeURIComponent(document.getElementById('ssid').value)}&pass=${encodeURIComponent(document.getElementById('pass').value)}`;
        fetch('/setwifi',{method:'POST', body: body}).then(()=>alert("Reiniciando equipo..."));
    }

    function fetchReleases(){ fetch('/list-releases').then(r=>r.json()).then(data=>{
        let l=document.getElementById('releases-list'); l.innerHTML="";
        data.forEach(r=>{ let d=document.createElement('div'); d.style="display:flex;justify-content:space-between;padding:12px;background:#252525;margin-top:8px;border-radius:8px;align-items:center;border:1px solid #333;";
            d.innerHTML=`<div style='text-align:left'><b>${r.tag}</b>${r.new?"<span class='new-badge'>NUEVA</span>":""}</div>
                         <button class='btn ${r.new?"update-ready":""}' style='width:auto;margin:0;padding:8px 12px;font-size:12px;' onclick="doUpdate('${r.bin_url}')">${r.new?'ACTUALIZAR':'INSTALAR'}</button>`;
            l.appendChild(d);});
    });}

    function doUpdate(url){ if(confirm("¿Desea iniciar la actualización de firmware?")) location.href='/do-update?url='+encodeURIComponent(url); }
</script></body></html>
)rawliteral";

// --- TAREA DE TELEMETRÍA (INDIFERENTE A LA PESTAÑA ABIERTA) ---
void send_telemetry_task(void* pv) {
    while(1) {
        if (ws_fd != -1 && g_server_handle) {
            cJSON *root = cJSON_CreateObject();
            cJSON_AddNumberToObject(root, "amp", g_corriente_actual);
            cJSON_AddNumberToObject(root, "pot", g_potenciometro_mv);
            cJSON_AddBoolToObject(root, "scr", g_scr_activo);
            char *json = cJSON_PrintUnformatted(root);
            
            httpd_ws_frame_t frame = {}; // Limpia todo a cero primero
                frame.type = HTTPD_WS_TYPE_TEXT;
                frame.payload = (uint8_t*)json;
                frame.len = strlen(json);
                frame.final = true; // El compilador pedía este inicializador
            httpd_ws_send_frame_async(g_server_handle, ws_fd, &frame);
            
            cJSON_Delete(root); 
            free(json);
        }
        vTaskDelay(pdMS_TO_TICKS(200)); // 5Hz para un gráfico suave
    }
}

PortalWeb::PortalWeb() : _server(NULL) {}

esp_err_t PortalWeb::start() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.stack_size = 20480; 
    config.max_uri_handlers = 12;

    if (httpd_start(&_server, &config) == ESP_OK) {
        g_server_handle = _server;

        // Registro de rutas con estructura limpia para evitar warnings
        static httpd_uri_t uri_root = {};
        uri_root.uri = "/"; uri_root.method = HTTP_GET;
        uri_root.handler = [](httpd_req_t *req){
            std::string s = root_html; std::string v = esp_app_get_description()->version;
            size_t p = s.find("{{VERSION}}");
            while(p != std::string::npos){ s.replace(p,11,v); p=s.find("{{VERSION}}",p+v.length()); }
            httpd_resp_set_type(req, "text/html; charset=utf-8");
            return httpd_resp_send(req, s.c_str(), s.length());
        };
        httpd_register_uri_handler(_server, &uri_root);

        static httpd_uri_t uri_ws = {};
        uri_ws.uri = "/ws"; uri_ws.method = HTTP_GET; uri_ws.is_websocket = true;
        uri_ws.handler = [](httpd_req_t *req){ ws_fd = httpd_req_to_sockfd(req); return ESP_OK; };
        httpd_register_uri_handler(_server, &uri_ws);

        static httpd_uri_t uri_scan = {};
        uri_scan.uri = "/scan"; uri_scan.method = HTTP_GET;
        uri_scan.handler = [](httpd_req_t *req){
            std::string j = WifiManager::scan_to_json();
            httpd_resp_set_type(req, "application/json");
            return httpd_resp_send(req, j.c_str(), j.length());
        };
        httpd_register_uri_handler(_server, &uri_scan);

        static httpd_uri_t uri_list = {};
        uri_list.uri = "/list-releases"; uri_list.method = HTTP_GET;
        uri_list.handler = [](httpd_req_t *req){
            auto rels = GitHubClient::get_releases("");
            std::string j = "[";
            for(size_t i=0; i<rels.size(); ++i){
                j += "{\"tag\":\""+rels[i].tag+"\",\"bin_url\":\""+rels[i].bin_url+"\",\"new\":"+(rels[i].is_new?"true":"false")+"}";
                if(i < rels.size()-1) j += ",";
            }
            j += "]";
            httpd_resp_set_type(req, "application/json");
            return httpd_resp_send(req, j.c_str(), j.length());
        };
        httpd_register_uri_handler(_server, &uri_list);

        static httpd_uri_t uri_upd = {};
        uri_upd.uri = "/do-update"; 
        uri_upd.method = HTTP_GET;
        uri_upd.handler = [](httpd_req_t *req){
            if(g_scr_enabled) return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Apague el SCR primero");

            // 1. Obtener la query string completa de la URI
            size_t query_len = httpd_req_get_url_query_len(req) + 1;
            if (query_len <= 1) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "URL faltante");

            char* query_str = (char*)malloc(query_len);
            httpd_req_get_url_query_str(req, query_str, query_len);

            // 2. Extraer el valor del parámetro "url"
            char url_encoded[512];
            if (httpd_query_key_value(query_str, "url", url_encoded, sizeof(url_encoded)) == ESP_OK) {
                
                // 3. Decodificar caracteres (%3A -> :, %2F -> /)
                std::string decoded_url = "";
                for (size_t i = 0; url_encoded[i] != '\0'; i++) {
                    if (url_encoded[i] == '%' && url_encoded[i+1] && url_encoded[i+2]) {
                        char hex[3] = { url_encoded[i+1], url_encoded[i+2], '\0' };
                        decoded_url += (char)strtol(hex, nullptr, 16);
                        i += 2;
                    } else if (url_encoded[i] == '+') {
                        decoded_url += ' ';
                    } else {
                        decoded_url += url_encoded[i];
                    }
                }

                ESP_LOGI("PORTAL_WEB", "Iniciando OTA hacia: %s", decoded_url.c_str());
                
                // 4. Iniciar proceso
                GitHubClient::start_ota_from_url(decoded_url.c_str());
                
                free(query_str);
                return httpd_resp_sendstr(req, "Actualización iniciada correctamente...");
            }

            free(query_str);
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "URL incorrecta o malformada");
        };
        httpd_register_uri_handler(_server, &uri_upd);

        static httpd_uri_t uri_wifi = {};
        uri_wifi.uri = "/setwifi"; uri_wifi.method = HTTP_POST;
        uri_wifi.handler = [](httpd_req_t *req){
            char b[256]; int r = httpd_req_recv(req, b, req->content_len); if(r<=0) return ESP_FAIL; b[r]=0;
            std::string d(b); size_t s_p=d.find("ssid="), p_p=d.find("&pass=");
            if(s_p!=std::string::npos && p_p!=std::string::npos){
                WifiManager::save_and_reconnect(d.substr(s_p+5, p_p-(s_p+5)), d.substr(p_p+6));
            }
            return httpd_resp_sendstr(req, "WiFi configurado");
        };
        httpd_register_uri_handler(_server, &uri_wifi);

        // Iniciar tarea de telemetría en el core secundario
        xTaskCreatePinnedToCore(send_telemetry_task, "tele_ws", 4096, NULL, 3, NULL, 1);
        
        ESP_LOGI(TAG, "Portal Web con WebSockets y OTA listo.");
        return ESP_OK;
    }
    return ESP_FAIL;
}

void PortalWeb::stop() { if(_server){ httpd_stop(_server); _server=NULL; ws_fd=-1; } }