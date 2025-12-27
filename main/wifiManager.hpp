#pragma once
#include <string>
#include <vector>
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"

extern volatile bool g_is_wifi_scanning;

class WifiManager {
public:
    static void init();
    static bool connect_saved();
    static void start_ap();
    static std::string scan_to_json();
    static void save_and_reconnect(std::string ssid, std::string pass);
    static bool is_connected();
    static bool should_fallback();
    
    // --- NUEVOS MÉTODOS PARA EL LOGGER ---
    static std::string get_ssid();
    static std::string get_ip();
};