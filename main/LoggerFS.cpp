#include "LoggerFS.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstdio>
#include <sys/unistd.h>
#include <sys/stat.h>

static const char* TAG = "LoggerFS";

LoggerFS::LoggerFS(const char* base_path) {
    _full_path = std::string(base_path) + "/rect_log.csv";
    _old_path  = std::string(base_path) + "/rect_log.old";
}

bool LoggerFS::begin() {
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
    // Marcamos si es LOCAL (1970) o SYNC (2024+)
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
            ESP_LOGW(TAG, "Rotando log... limite alcanzado");
            unlink(_old_path.c_str());
            rename(_full_path.c_str(), _old_path.c_str());
            writeHeader();
        }
    }
}

void LoggerFS::registrar(RectEvent evento, const RectStatus& status, const std::string& nota) {
    std::lock_guard<std::mutex> lock(_mutex);
    
    checkRotation(); // Limpieza automática antes de escribir

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
    writeHeader();
    ESP_LOGI(TAG, "Log del sistema truncado y reiniciado.");
}