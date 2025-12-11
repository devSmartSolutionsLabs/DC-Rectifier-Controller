#include "ina226.hpp"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/task.h"

INA226::INA226(i2c_port_t port, uint8_t addr7, SemaphoreHandle_t i2c_mutex)
: _port(port), _addr(addr7), _mtx(i2c_mutex) {}

bool INA226::begin(Avg avg, Ct vbus_ct, Ct vsh_ct, Mode mode) {
    return configure(avg, vbus_ct, vsh_ct, mode);
}

bool INA226::configure(Avg avg, Ct vbus_ct, Ct vsh_ct, Mode mode) {
    uint16_t cfg = makeConfig(avg, vbus_ct, vsh_ct, mode);
    uint16_t be = (uint16_t)((cfg >> 8) & 0xFF) | (uint16_t)((cfg & 0xFF) << 8);
    if (!writeReg16(REG_CONFIG, be)) return false;
    _config_cached = be;
    return true;
}

bool INA226::readBusVoltage_mV(int32_t& out_mV, bool wait_ready, uint32_t timeout_us) {
    if (wait_ready && !waitConversionReady(timeout_us)) return false;
    uint16_t raw_be = 0;
    if (!readReg16(REG_BUS_V, raw_be)) return false;
    uint16_t raw = (uint16_t)((raw_be >> 8) | (raw_be << 8));
    out_mV = static_cast<int32_t>(raw) * 125 / 100;
    return true;
}

bool INA226::readShuntMicroVolts(int32_t& out_uV, bool wait_ready, uint32_t timeout_us) {
    if (wait_ready && !waitConversionReady(timeout_us)) return false;
    uint16_t raw_be = 0;
    if (!readReg16(REG_SHUNT_V, raw_be)) return false;
    int16_t raw = (int16_t)((raw_be >> 8) | (raw_be << 8));
    out_uV = static_cast<int32_t>(raw) * 25 / 10;
    return true;
}

bool INA226::waitConversionReady(uint32_t timeout_us) {
    const uint64_t t0 = esp_timer_get_time();
    for (;;) {
        uint16_t mask_be = 0;
        if (!readReg16(REG_MASK_ENABLE, mask_be)) return false;
        uint16_t mask = (uint16_t)((mask_be >> 8) | (mask_be << 8));
        if (mask & (1u << 3)) return true;

        if ((uint32_t)(esp_timer_get_time() - t0) > timeout_us) return false;
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

bool INA226::readReg16(Reg reg, uint16_t& be_val) {
    return i2cRead16(reg, be_val);
}

bool INA226::writeReg16(Reg reg, uint16_t be_val) {
    return i2cWrite16(reg, be_val);
}

bool INA226::lock(uint32_t ms) {
    if (!_mtx) return true;
    return xSemaphoreTake(_mtx, pdMS_TO_TICKS(ms)) == pdTRUE;
}

void INA226::unlock() {
    if (_mtx) xSemaphoreGive(_mtx);
}

bool INA226::i2cWrite16(Reg reg, uint16_t be_val) {
    if (!lock()) return false;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == nullptr) {
        unlock();
        return false;
    }

    uint8_t hi = (uint8_t)(be_val & 0xFF);
    uint8_t lo = (uint8_t)((be_val >> 8) & 0xFF);

    esp_err_t err = ESP_OK;
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, static_cast<uint8_t>(reg), true);
    i2c_master_write_byte(cmd, hi, true);
    i2c_master_write_byte(cmd, lo, true);
    i2c_master_stop(cmd);
    
    err = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    unlock();
    return err == ESP_OK;
}

bool INA226::i2cRead16(Reg reg, uint16_t& be_val) {
    if (!lock()) return false;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == nullptr) {
        unlock();
        return false;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, static_cast<uint8_t>(reg), true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_READ, true);

    uint8_t b0 = 0, b1 = 0;
    i2c_master_read_byte(cmd, &b0, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &b1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    unlock();

    if (err != ESP_OK) return false;

    be_val = ((uint16_t)b0 << 8) | b1;
    return true;
}