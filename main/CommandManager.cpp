#include "CommandManager.hpp"
#include <cstdio>
#include "esp_log.h"

extern LoggerFS g_logger;

// Variables externas para reporte de estado
extern float g_corriente_actual;
extern int g_potenciometro_mv;
extern bool g_scr_enabled;

static const char* TAG = "CMD_MGR";

std::string CommandManager::execute(std::string cmd) {
    sanitize(cmd);
    
    if (cmd.empty()) return "";

    ESP_LOGI(TAG, "Ejecutando: %s", cmd.c_str());

    if (cmd == "log.show") {
        return dumpLogs();
    } 
    // Dentro de CommandManager::execute
    else if (cmd == "log.clear") {
        g_logger.limpiarLog(); 
        return "SUCCESS: Historial reiniciado. Registro de borrado generado.";
    }
    else if (cmd == "stats") {
        return getSystemStats();
    }
    else if (cmd == "help") {
        return "\n--- COMANDOS RECTIFICADOR ---\n"
               "log.show  : Muestra logs CSV\n"
               "log.clear : Borra logs\n"
               "stats     : Estado actual\n"
               "-----------------------------\n";
    }

    return "ERROR: Comando '" + cmd + "' no reconocido. Escriba 'help'.";
}

void CommandManager::sanitize(std::string &s) {
    s.erase(std::remove(s.begin(), s.end(), '\n'), s.end());
    s.erase(std::remove(s.begin(), s.end(), '\r'), s.end());
}

std::string CommandManager::getSystemStats() {
    char buf[128];
    snprintf(buf, sizeof(buf), "STATS: Pot:%d mV | Amp:%.1f A | SCR:%s", 
             g_potenciometro_mv, g_corriente_actual, g_scr_enabled ? "ON" : "OFF");
    return std::string(buf);
}

std::string CommandManager::dumpLogs() {
    extern LoggerFS g_logger;
    FILE* f = fopen(g_logger.getFilePath().c_str(), "r");
    if (f == NULL) return "ERROR: No se pudo leer el archivo de logs.";

    std::string res = "\n>>> INICIO LOG CSV <<<\n";
    char line[128];
    while (fgets(line, sizeof(line), f)) {
        res += line;
    }
    res += ">>> FIN LOG CSV <<<\n";
    fclose(f);
    return res;
}