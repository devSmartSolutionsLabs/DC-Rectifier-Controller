#ifndef LOGGER_FS_HPP
#define LOGGER_FS_HPP

#include <string>
#include <mutex>
#include <time.h>

enum class RectDirection : uint8_t { FORWARD = 0, REVERSE = 1 };

enum class RectEvent : uint8_t {
    BOOT           = 0x01,
    PROCESS_START  = 0x02,
    PROCESS_STOP   = 0x03,
    INTERRUPTION   = 0x04,
    ERROR_HARDWARE = 0x05,
    CONFIG_CHANGE  = 0x06,
    NETWORK_ST     = 0x07  // Estado de Red (Conexión/Desconexión)
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
    
    // Inicializa el sistema y verifica rotación/encabezados
    bool begin(); 

    void registrar(RectEvent evento, const RectStatus& status, const std::string& nota = "");
    void limpiarLog();
    std::string getFilePath() const { return _full_path; }

private:
    std::string _full_path;
    std::string _old_path;
    std::mutex _mutex;
    const size_t MAX_LOG_SIZE = 500 * 1024; // 500 KB limite

    std::string getLimaTimestamp();
    void checkRotation();
    void writeHeader();
};

#endif