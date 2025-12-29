#include "LoggerFS.hpp"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "mcp23017.hpp"
#include <cstdio>
#include <sys/unistd.h>
#include <sys/stat.h>

static const char* TAG = "LoggerFS";

// Referencia al segundo MCP para control de pines de la SD (dirección 0x25)
extern MCP23017* g_mcp_2; 

#define SD_MOSI     GPIO_NUM_11
#define SD_MISO     GPIO_NUM_13
#define SD_SCK      GPIO_NUM_12
#define SD_CS_INDEX 13 // GPB5
#define SD_CD_INDEX 14 // GPB6

LoggerFS::LoggerFS(const char* base_path) : _base_path(base_path) {
    _full_path = std::string(base_path) + "/rect_log.csv";
    _old_path  = std::string(base_path) + "/rect_log.old";
}

bool LoggerFS::is_card_inserted() {
    return true; // Forzamos true para pruebas
    if (!g_mcp_2) return false;
    bool level = true;
    // Leer pin GPB6 (Card Detect)
    g_mcp_2->digital_read(SD_CD_INDEX, level);
    // Típicamente CD se conecta a GND (LOW) cuando la tarjeta entra
    return (level == false); 
}

bool LoggerFS::begin() {
    if (!is_card_inserted()) {
        ESP_LOGE(TAG, "No se detecta tarjeta SD en GPB6. Abortando montaje.");
        return false;
    }

    ESP_LOGI(TAG, "Iniciando montaje de SD en %s...", _base_path.c_str());

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = false,
        .use_one_fat = false
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST; // Bus SPI2 (Pines 11, 12, 13)
    host.max_freq_khz = 1000; // REDUCIR A 5MHz para estabilidad con CS en MCP23017

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI,
        .miso_io_num = SD_MISO,
        .sclk_io_num = SD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = GPIO_NUM_NC; // CS no es un pin local del ESP32
    slot_config.host_id = (spi_host_device_t)host.slot;

    // 3. Inicializar el bus SPI
    esp_err_t ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return false;

    // --- OPCIONAL: SOLO SI LA TARJETA ESTÁ CORRUPTA ---
    // Si necesitas forzar un formateo manual porque nada funciona, descomenta la siguiente línea:
    //esp_vfs_fat_sdcard_format(_base_path.c_str(), &host, &slot_config);

    sdmmc_card_t* card;
    ret = esp_vfs_fat_sdspi_mount(_base_path.c_str(), &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al montar FatFS: %s", esp_err_to_name(ret));
        return false;
    }

    // ÉXITO: Dejamos el CS en 0 si es bus dedicado para evitar latencias de I2C
    ESP_LOGI(TAG, "SD montada con éxito a 1MHz.");
    sdmmc_card_print_info(stdout, card);

    // 6. Verificar o crear el archivo de logs
    struct stat st;
    if (stat(_full_path.c_str(), &st) != 0) {
        writeHeader();
    }
    
    return true;
}

void LoggerFS::registrarEstructurado(RectEvent evento, std::string valor, std::string nota) {
    if (!is_card_inserted()) return;

    std::lock_guard<std::mutex> lock(_mutex);
    checkRotation();

    // ACTIVAR CS (LOW)
    g_mcp_2->digital_write(SD_CS_INDEX, 0);

    FILE* f = fopen(_full_path.c_str(), "a");
    if (f != NULL) {
        fprintf(f, "%s,0x%04X,%s,%s\n", 
                getLimaTimestamp().c_str(),
                static_cast<uint16_t>(evento),
                valor.empty() ? "-" : valor.c_str(),
                nota.empty() ? "-" : nota.c_str());
        fclose(f);
    }
}

void LoggerFS::limpiarLog() {
    if (!is_card_inserted()) return;
    
    std::lock_guard<std::mutex> lock(_mutex);
    
    writeHeader(); 

    // Registrar el rastro del borrado
    FILE* f = fopen(_full_path.c_str(), "a");
    if (f) {
        fprintf(f, "%s,0x0601,USER,Historial reiniciado por el usuario\n", getLimaTimestamp().c_str());
        fclose(f);
    }
    ESP_LOGW(TAG, "Log en SD reiniciado.");
}

void LoggerFS::writeHeader() {
    FILE* f = fopen(_full_path.c_str(), "w");
    if (f) {
        fprintf(f, "Fecha_Hora,EventID,Valor,Nota\n");
        fclose(f);
    }
}

std::string LoggerFS::getLimaTimestamp() {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    char buf[64];
    // [L] Local (sin sincronizar), [S] Sincronizado por NTP
    const char* tag = (timeinfo.tm_year < (2024 - 1900)) ? "[L]" : "[S]";
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return std::string(tag) + " " + std::string(buf);
}

void LoggerFS::checkRotation() {
    struct stat st;
    if (stat(_full_path.c_str(), &st) == 0 && st.st_size >= MAX_LOG_SIZE) {
        ESP_LOGW(TAG, "Rotando archivo de log en SD...");
        unlink(_old_path.c_str());
        rename(_full_path.c_str(), _old_path.c_str());
        writeHeader();
    }
}

// Método compatible con el formato antiguo (si aún se usa en alguna parte)
void LoggerFS::registrar(RectEvent evento, const RectStatus& status, const std::string& nota) {
    char buf[64];
    snprintf(buf, sizeof(buf), "D:%d|A:%.1f|V:%.1f", 
             static_cast<int>(status.direction), status.current, status.voltage);
    registrarEstructurado(evento, buf, nota);
}