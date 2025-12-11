#include "WiFiHelper.hpp"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char* TAG = "WiFiHelper";

static esp_err_t ensure_nvs_netif_eventloop() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(esp_netif_init());
    // Si ya existe, devolverá INVALID_STATE y lo ignoramos
    esp_err_t e = esp_event_loop_create_default();
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;
    return ESP_OK;
}

esp_err_t WiFiHelper::start_softap(const char* ssid, const char* pass, uint8_t channel, uint8_t max_conn) {
    ESP_ERROR_CHECK(ensure_nvs_netif_eventloop());

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    wifi_config_t ap = {};
    strncpy((char*)ap.ap.ssid, ssid, sizeof(ap.ap.ssid));
    ap.ap.ssid_len = strlen((const char*)ap.ap.ssid);
    ap.ap.channel = channel;
    ap.ap.max_connection = max_conn;
    ap.ap.authmode = pass && pass[0] ? WIFI_AUTH_WPA_WPA2_PSK : WIFI_AUTH_OPEN;
    if (ap.ap.authmode != WIFI_AUTH_OPEN) {
        strncpy((char*)ap.ap.password, pass, sizeof(ap.ap.password));
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "SoftAP listo. SSID: %s  Pass: %s  Canal: %u",
             (const char*)ap.ap.ssid,
             ap.ap.authmode == WIFI_AUTH_OPEN ? "(OPEN)" : (const char*)ap.ap.password,
             ap.ap.channel);
    ESP_LOGI(TAG, "Conéctate y abre http://192.168.4.1/update (si montas UI) o usa curl");

    return ESP_OK;
}

esp_err_t WiFiHelper::start_sta(const char* ssid, const char* pass, uint32_t wait_ms) {
    ESP_ERROR_CHECK(ensure_nvs_netif_eventloop());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t wc = {};
    strncpy((char*)wc.sta.ssid, ssid, sizeof(wc.sta.ssid));
    strncpy((char*)wc.sta.password, pass, sizeof(wc.sta.password));
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    // Espera best-effort por IP (simple)
    uint32_t t0 = (uint32_t) (esp_timer_get_time()/1000ULL);
    while ((uint32_t)(esp_timer_get_time()/1000ULL) - t0 < wait_ms) {
        vTaskDelay(pdMS_TO_TICKS(200));
        // En producción usar eventos IP_EVENT_STA_GOT_IP
    }

    ESP_LOGI(TAG, "STA iniciado (ver logs de DHCP para IP)");
    return ESP_OK;
}
