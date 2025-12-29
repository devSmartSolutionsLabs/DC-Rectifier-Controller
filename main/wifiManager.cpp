#include "WifiManager.hpp"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <string.h>


static const char* TAG = "WIFI_MGR";
static int s_retry_num = 0;
#define MAX_RETRIES 5

// Banderas de estado
bool s_connected = false;
bool s_must_fallback = false;
// Cambia esto al principio de wifiManager.cpp
extern volatile bool g_is_wifi_scanning;
// Manejador de eventos de WiFi (Conexión, Desconexión e IP)
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected = false;
        if (s_retry_num < MAX_RETRIES) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "Reintentando conectar al AP... (%d/%d)", s_retry_num, MAX_RETRIES);
        } else {
            ESP_LOGE(TAG, "Fallo de conexión tras %d intentos. Solicitando portal...", MAX_RETRIES);
            s_must_fallback = true;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "IP Obtenida: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        s_connected = true;
        s_must_fallback = false;
    }
}

void WifiManager::init() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    // Se asume que el event loop ya existe o se crea aquí
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }
}

bool WifiManager::is_connected() { return s_connected; }
bool WifiManager::should_fallback() { return s_must_fallback; }

// --- MÉTODO DE ESCANEO PROFESIONAL ---
std::string WifiManager::scan_to_json() {
    g_is_wifi_scanning = true; // Pausamos lecturas del ADS1115 en el loop principal
    ESP_LOGI(TAG, "Iniciando escaneo en modo APSTA para estabilidad...");

    // 1. Cambiamos modo a APSTA (Permite escanear mientras el Portal sigue vivo)
    esp_wifi_set_mode(WIFI_MODE_APSTA);
    vTaskDelay(pdMS_TO_TICKS(100)); // Delay para estabilizar radio

    wifi_scan_config_t scan_config = {};
    scan_config.show_hidden = true;
    scan_config.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    scan_config.scan_time.active.min = 100;
    scan_config.scan_time.active.max = 200;

    // 2. Iniciamos escaneo bloqueante (true) ahora que estamos en modo seguro
    esp_err_t res = esp_wifi_scan_start(&scan_config, true);
    
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al iniciar escaneo: %s", esp_err_to_name(res));
        g_is_wifi_scanning = false;
        return "[]";
    }

    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    ESP_LOGI(TAG, "Redes encontradas: %d", ap_count);

    if (ap_count == 0) {
        g_is_wifi_scanning = false;
        return "[]";
    }

    // 3. Obtener registros usando PSRAM (Ideal para ESP32-S3)
    wifi_ap_record_t *ap_info = (wifi_ap_record_t *)heap_caps_malloc(sizeof(wifi_ap_record_t) * ap_count, MALLOC_CAP_SPIRAM);
    if (!ap_info) {
        ESP_LOGE(TAG, "Error: Sin memoria PSRAM para escaneo");
        g_is_wifi_scanning = false;
        return "[]";
    }

    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_info));

    // 4. Construir el JSON
    std::string json = "[";
    for (int i = 0; i < ap_count; i++) {
        json += "{\"s\":\"" + std::string((char*)ap_info[i].ssid) + "\",\"r\":" + std::to_string(ap_info[i].rssi) + "}";
        if (i < ap_count - 1) json += ",";
    }
    json += "]";

    heap_caps_free(ap_info);
    g_is_wifi_scanning = false; // Reanudamos ADS1115
    return json;
}

void WifiManager::save_and_reconnect(std::string ssid, std::string pass) {
    nvs_handle_t handle;
    if (nvs_open("storage", NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_str(handle, "wifi_ssid", ssid.c_str());
        nvs_set_str(handle, "wifi_pass", pass.c_str());
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "Credenciales guardadas en NVS (storage)");
    }
    ESP_LOGW(TAG, "Credenciales guardadas. Reiniciando en 3 segundos...");
    // Damos tiempo al PortalWeb para terminar la petición HTTP
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();
}

bool WifiManager::connect_saved() {
    nvs_handle_t handle;
    char ssid[32], pass[64];
    size_t s_len = 32, p_len = 64;

    if (nvs_open("storage", NVS_READONLY, &handle) != ESP_OK) {
        ESP_LOGW(TAG, "No se encontro particion de datos WiFi.");
        return false;
    }

    esp_err_t res = nvs_get_str(handle, "wifi_ssid", ssid, &s_len);
    res |= nvs_get_str(handle, "wifi_pass", pass, &p_len);
    nvs_close(handle);

    if (res != ESP_OK) {
        ESP_LOGW(TAG, "Credenciales incompletas en NVS.");
        return false;
    }

    ESP_LOGI(TAG, "Conectando a red guardada: %s", ssid);

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL);

    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, ssid, 32);
    strncpy((char*)wifi_config.sta.password, pass, 64);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_connect();
    
    return true;
}

void WifiManager::start_ap() {
    ESP_LOGI(TAG, "Iniciando SoftAP de configuracion...");
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t ap_config = {};
    strcpy((char*)ap_config.ap.ssid, "RECTIFICADOR_S3_CFG");
    ap_config.ap.channel = 1;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_OPEN;

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    esp_wifi_start();
    ESP_LOGI(TAG, "Portal AP listo en 192.168.4.1");
}

// Añade estos métodos al final de tu archivo wifiManager.cpp

std::string WifiManager::get_ssid() {
    wifi_config_t conf;
    // Obtenemos la configuración actual de la interfaz Station
    esp_err_t res = esp_wifi_get_config(WIFI_IF_STA, &conf);
    if (res == ESP_OK) {
        return std::string((char*)conf.sta.ssid);
    }
    return "Desconocido";
}

std::string WifiManager::get_ip() {
    esp_netif_ip_info_t ip_info;
    // Obtenemos el handle de la interfaz por defecto de la estación
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    
    if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
        char buf[16];
        esp_ip4addr_ntoa(&ip_info.ip, buf, sizeof(buf));
        return std::string(buf);
    }
    return "0.0.0.0";
}

void WifiManager::save_last_time(long timestamp) {
    nvs_handle_t handle;
    if (nvs_open("storage", NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_i64(handle, "last_time", (int64_t)timestamp);
        nvs_commit(handle);
        nvs_close(handle);
    }
}

long WifiManager::get_last_time() {
    nvs_handle_t handle;
    int64_t timestamp = 0;
    if (nvs_open("storage", NVS_READONLY, &handle) == ESP_OK) {
        nvs_get_i64(handle, "last_time", &timestamp);
        nvs_close(handle);
    }
    return (long)timestamp;
}