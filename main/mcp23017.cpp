#include "mcp23017.hpp"

const char* MCP23017::TAG = "MCP23017";

MCP23017::MCP23017(i2c_port_t port, uint8_t addr) 
    : m_port(port), m_addr(addr) {
}

MCP23017::~MCP23017() {
}

bool MCP23017::begin() {
    // Configurar IOCON para modo bancos secuenciales
    uint8_t iocon_value;
    if (!read_register(IOCONA, iocon_value)) return false;
    
    iocon_value &= ~(1 << 7); // SEQOP = 0 (operación secuencial habilitada)
    iocon_value &= ~(1 << 6); // MIRROR = 0 (INTA/INTB separados)
    
    if (!write_register(IOCONA, iocon_value)) return false;
    
    ESP_LOGI(TAG, "MCP23017 inicializado en addr 0x%02X", m_addr);
    return true;
}

bool MCP23017::write_register(uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (m_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(m_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        // ESP_LOGE(TAG, "Error escritura reg 0x%02X: %d", reg, ret); //(Comentado para reducir logs)
        return false;
    }
    return true;
}

bool MCP23017::read_register(uint8_t reg, uint8_t& value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (m_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd); // Repeated start
    i2c_master_write_byte(cmd, (m_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(m_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        //ESP_LOGE(TAG, "Error lectura reg 0x%02X: %d", reg, ret); //(Comentado para reducir logs)
        return false;
    }
    return true;
}

bool MCP23017::pin_mode(uint8_t pin, uint8_t mode) {
    uint8_t reg = (pin < 8) ? IODIRA : IODIRB;
    uint8_t bit = pin % 8;
    
    uint8_t current;
    if (!read_register(reg, current)) return false;
    
    if (mode == 1) { // INPUT
        current |= (1 << bit);
    } else { // OUTPUT
        current &= ~(1 << bit);
    }
    
    return write_register(reg, current);
}

bool MCP23017::pin_pullup(uint8_t pin, bool enable) {
    uint8_t reg = (pin < 8) ? GPPUA : GPPUB;
    uint8_t bit = pin % 8;
    
    uint8_t current;
    if (!read_register(reg, current)) return false;
    
    if (enable) {
        current |= (1 << bit);
    } else {
        current &= ~(1 << bit);
    }
    
    return write_register(reg, current);
}

bool MCP23017::digital_write(uint8_t pin, bool level) {
    uint8_t reg = (pin < 8) ? GPIOA : GPIOB;
    uint8_t bit = pin % 8;
    
    uint8_t current;
    if (!read_register(reg, current)) return false;
    
    if (level) {
        current |= (1 << bit);
    } else {
        current &= ~(1 << bit);
    }
    
    return write_register(reg, current);
}

bool MCP23017::digital_read(uint8_t pin, bool& level) {
    uint8_t reg = (pin < 8) ? GPIOA : GPIOB;
    uint8_t value;
    
    if (!read_register(reg, value)) return false;
    
    level = (value >> (pin % 8)) & 1;
    return true;
}

bool MCP23017::write_port_a(uint8_t value) {
    return write_register(GPIOA, value);
}

bool MCP23017::write_port_b(uint8_t value) {
    return write_register(GPIOB, value);
}

bool MCP23017::read_port_a(uint8_t& value) {
    return read_register(GPIOA, value);
}

bool MCP23017::read_port_b(uint8_t& value) {
    return read_register(GPIOB, value);
}

bool MCP23017::test_connection() {
    uint8_t value;
    return read_register(IOCONA, value);
}