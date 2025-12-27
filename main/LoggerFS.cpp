#include "LoggerFS.hpp"
#include "esp_log.h"
#include "esp_spiffs.h" // Driver nativo de ESP-IDF
#include <cstdio>
#include <sys/unistd.h>
#include <sys/stat.h>

static const char* TAG = "LoggerFS";

LoggerFS::LoggerFS(const char* base_path) : _base_path(base_path) {
    _full_path = std::string(base_path) + "/rect_log.csv";
    _old_path  = std::string(base_path) + "/rect_log.old";
}

bool LoggerFS::begin() {
    ESP_LOGI(TAG, "Montando SPIFFS en %s...", _base_path.c_str());

    esp_vfs_spiffs_conf_t conf = {
        .base_path = _base_path.c_str(),
        .partition_label = "storage", // Coincide con tu partitions.csv
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Fallo al montar o formatear SPIFFS");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "No se encontro la particion 'storage'");
        }
        return false;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Particion SPIFFS: Total: %d KB, Usado: %d KB", total/1024, used/1024);
    }

    struct stat st;
    if (stat(_full_path.c_str(), &st) != 0) {
        writeHeader();
    }
    return true;
}

std::string LoggerFS::getLimaTimestamp() {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    char buf[64];
    const char* tag = (timeinfo.tm_year < (2024 - 1900)) ? "[L]" : "[S]";
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return std::string(tag) + " " + std::string(buf);
}

void LoggerFS::writeHeader() {
    FILE* f = fopen(_full_path.c_str(), "w");
    if (f) {
        fprintf(f, "Fecha_Hora,EventID,Dir,Amps,Volts,Temp,Nota\n");
        fclose(f);
    }
}

void LoggerFS::checkRotation() {
    struct stat st;
    if (stat(_full_path.c_str(), &st) == 0) {
        if (st.st_size >= MAX_LOG_SIZE) {
            ESP_LOGW(TAG, "Rotando log...");
            unlink(_old_path.c_str());
            rename(_full_path.c_str(), _old_path.c_str());
            writeHeader();
        }
    }
}

void LoggerFS::registrarEstructurado(RectEvent evento, std::string valor, std::string nota) {
    std::lock_guard<std::mutex> lock(_mutex);
    checkRotation();

    FILE* f = fopen(_full_path.c_str(), "a");
    if (f == NULL) return;

    // Guardamos el ID en Hexadecimal para que sea fácil de filtrar
    fprintf(f, "%s,0x%04X,%s,%s\n", 
            getLimaTimestamp().c_str(),
            static_cast<uint16_t>(evento),
            valor.empty() ? "-" : valor.c_str(),
            nota.empty() ? "-" : nota.c_str());

    fsync(fileno(f));
    fclose(f);
}

void LoggerFS::registrar(RectEvent evento, const RectStatus& status, const std::string& nota) {
    std::lock_guard<std::mutex> lock(_mutex);
    checkRotation();

    FILE* f = fopen(_full_path.c_str(), "a");
    if (f == NULL) return;

    fprintf(f, "%s,%u,%u,%.2f,%.2f,%lu,%s\n", 
            getLimaTimestamp().c_str(),
            static_cast<uint8_t>(evento),
            static_cast<uint8_t>(status.direction),
            status.current,
            status.voltage,
            status.temp,
            nota.c_str());

    fsync(fileno(f));
    fclose(f);
}

void LoggerFS::limpiarLog() {
    std::lock_guard<std::mutex> lock(_mutex);
    
    // 1. Recreamos el encabezado (esto borra el contenido previo)
    writeHeader(); 

    // 2. Registramos el rastro del borrado
    FILE* f = fopen(_full_path.c_str(), "a");
    if (f) {
        // Cambiamos el 0 por 0UL para que coincida con %lu
        fprintf(f, "%s,%u,%u,%.2f,%.2f,%lu,%s\n", 
                getLimaTimestamp().c_str(),
                static_cast<uint8_t>(RectEvent::CONFIG_CHANGE), 
                0,      // Dir (uint8_t)
                0.0,    // Amps (float)
                0.0,    // Volts (float)
                0UL,    // Temp (uint32_t -> requiere UL para %lu)
                "LOG_CLEARED: El historial fue reiniciado por el usuario.");
        
        fsync(fileno(f));
        fclose(f);
    }
    
    ESP_LOGW(TAG, "Historial reiniciado. Se ha dejado rastro de la accion.");
}