#ifndef LOGGER_FS_HPP
#define LOGGER_FS_HPP

#include <string>
#include <mutex>
#include <time.h>
#include <algorithm>

enum class RectDirection : uint8_t { FORWARD = 0, REVERSE = 1 };

enum class RectEvent : uint16_t {
    // CATEGORIA 01: SISTEMA
    BOOT            = 0x0100,  // Arranque normal (Power ON)
    BOOT_WDT        = 0x0101,  // Reinicio por Watchdog (Cuelgue)
    BOOT_SOFT       = 0x0102,  // Reinicio por Software (Post-OTA o Config)
    HEARTBEAT       = 0x0103,  // Pulso periódico de vida

    // CATEGORIA 02: PROCESO (POTENCIA)
    BTN_START_PRESS   = 0x0210, // Botón presionado
    BTN_START_RELEASE = 0x0211, // Botón liberado
    PROCESS_START     = 0x0200, // Proceso de potencia realmente activado
    PROCESS_STOP      = 0x0201,  // Proceso de potencia desactivado
    POT_CHANGE      = 0x0202, // Cambio de potenciómetro

    // CATEGORIA 05: ERRORES
    ERR_I2C         = 0x0501,
    ERR_WDT         = 0x0502,
    ERR_SYSTEM      = 0x0503,

    // CATEGORIA 06: CONFIG/USUARIO
    CONFIG_CHANGE  = 0x0600,  // Evento genérico de configuración
    LOG_CLEARED     = 0x0601,
    OTA_START       = 0x0602,

    // CATEGORIA 07: RED
    NET_AP_START    = 0x0700,
    NET_IP          = 0x0701,
    NET_SSID        = 0x0702,
    NET_RSSI        = 0x0703
};

struct RectStatus {
    RectDirection direction;
    float current;
    float voltage;
    uint32_t temp;
};

class LoggerFS {
public:
    explicit LoggerFS(const char* base_path);
    bool begin(); 
    bool is_card_inserted(); // <--- AÑADIR ESTA LÍNEA
    void registrar(RectEvent evento, const RectStatus& status, const std::string& nota = "");
    void registrarEstructurado(RectEvent evento, std::string valor, std::string nota);
    void limpiarLog();
    std::string getFilePath() const { return _full_path; }

private:
    std::string _base_path;
    std::string _full_path;
    std::string _old_path;
    std::mutex _mutex;
    const size_t MAX_LOG_SIZE = 500 * 1024; // 500 KB

    std::string getLimaTimestamp();
    void checkRotation();
    void writeHeader();
};

#endif