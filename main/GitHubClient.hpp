#pragma once
#include <string>
#include <vector>
#include "esp_err.h"

struct ReleaseInfo {
    std::string tag;
    std::string bin_url;
    bool is_new; // Nueva bandera para el portal
};

class GitHubClient {
public:
    // Obtiene la lista de versiones y decide si son nuevas
    static std::vector<ReleaseInfo> get_releases(const char* repo);
    static std::string get_releases_json();
    // Inicia la tarea de actualización OTA
    static void start_ota_from_url(const char* url);
    
    // Funciones de utilidad de versión (Las que daban error)
    static std::string get_current_version();
    static bool is_newer_version(std::string remote, std::string local);

private:
    static void ota_task(void* pvParameter);
};