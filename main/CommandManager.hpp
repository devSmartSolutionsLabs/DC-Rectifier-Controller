#ifndef COMMAND_MANAGER_HPP
#define COMMAND_MANAGER_HPP

#include <string>
#include <vector>
#include <algorithm>
#include "LoggerFS.hpp"

class CommandManager {
public:
    /**
     * @brief Procesa un comando de texto y retorna la respuesta.
     * @param cmd Cadena de texto recibida (Serial, WiFi, BT)
     * @return std::string Respuesta para el canal emisor
     */
    static std::string execute(std::string cmd);

private:
    // Métodos internos de procesamiento
    static std::string dumpLogs();
    static std::string getSystemStats();
    static void sanitize(std::string &s);
};

#endif