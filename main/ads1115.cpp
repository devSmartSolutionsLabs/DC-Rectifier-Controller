#include "ADS1115.hpp"
#include "esp_timer.h"
#include "driver/i2c.h"

ADS1115::ADS1115(i2c_port_t port, uint8_t addr, SemaphoreHandle_t opt_mutex)
: port_(port), addr_(addr), mtx_(opt_mutex) {}

bool ADS1115::begin(){ return true; }

// ===== Helpers de conversión =====
float ADS1115::fsr_mV(PGA p){
    switch(p){
        case PGA::FS_6V144: return 6144.0f;
        case PGA::FS_4V096: return 4096.0f;
        case PGA::FS_2V048: return 2048.0f;
        case PGA::FS_1V024: return 1024.0f;
        case PGA::FS_0V512: return 512.0f;
        default:            return 256.0f;
    }
}
float ADS1115::lsb_uV(PGA p){ return fsr_mV(p) * 1000.0f / 32768.0f; }

// ===== Config builder =====
uint16_t ADS1115::makeConfig(Mux mux, PGA pga, DataRate dr, Mode mode, bool start){
    uint16_t cfg = 0;
    // bit15 OS: 1=Start single-shot; en continuo se ignora
    if(start) cfg |= (1u<<15);
    // bits14..12 MUX
    cfg |= (uint16_t(uint8_t(mux)&0x07) << 12);
    // bits11..9 PGA
    cfg |= (uint16_t(uint8_t(pga)&0x07) << 9);
    // bit8 MODE: 1=single-shot, 0=continuous
    cfg |= (uint16_t(mode==Mode::SINGLE_SHOT) << 8);
    // bits7..5 DR
    cfg |= (uint16_t(uint8_t(dr)&0x07) << 5);
    // bits4..0 comparador apagado (00011)
    cfg |= 0x0003;
    return cfg;
}

uint32_t ADS1115::drTimeoutMs(DataRate dr){
    switch(dr){
        case DataRate::SPS_8:   return 130;
        case DataRate::SPS_16:  return 70;
        case DataRate::SPS_32:  return 36;
        case DataRate::SPS_64:  return 20;
        case DataRate::SPS_128: return 10;
        case DataRate::SPS_250: return 6;
        case DataRate::SPS_475: return 3;
        case DataRate::SPS_860: return 2;
        default: return 10;
    }
}

// ===== I2C bajo nivel =====
bool ADS1115::writeConfig(uint16_t cfg){
    Lock L(mtx_);
    uint8_t buf[3] = { REG_CONFIG, (uint8_t)(cfg>>8), (uint8_t)(cfg&0xFF) };
    i2c_cmd_handle_t c = i2c_cmd_link_create();
    i2c_master_start(c);
    i2c_master_write_byte(c, (addr_<<1)|I2C_MASTER_WRITE, true);
    i2c_master_write(c, buf, 3, true);
    i2c_master_stop(c);
    esp_err_t ret = i2c_master_cmd_begin(port_, c, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(c);
    return ret == ESP_OK;
}

bool ADS1115::readConfig(uint16_t& cfg){
    Lock L(mtx_);
    uint8_t h=0,l=0;
    i2c_cmd_handle_t c = i2c_cmd_link_create();
    i2c_master_start(c);
    i2c_master_write_byte(c, (addr_<<1)|I2C_MASTER_WRITE, true);
    i2c_master_write_byte(c, REG_CONFIG, true);
    i2c_master_start(c);
    i2c_master_write_byte(c, (addr_<<1)|I2C_MASTER_READ, true);
    i2c_master_read_byte(c, &h, I2C_MASTER_ACK);
    i2c_master_read_byte(c, &l, I2C_MASTER_NACK);
    i2c_master_stop(c);
    esp_err_t ret = i2c_master_cmd_begin(port_, c, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(c);
    if(ret != ESP_OK) return false;
    cfg = (uint16_t(h)<<8) | l;
    return true;
}

bool ADS1115::readConversion(int16_t& raw){
    Lock L(mtx_);
    uint8_t d[2] = {0,0};
    i2c_cmd_handle_t c = i2c_cmd_link_create();
    i2c_master_start(c);
    i2c_master_write_byte(c, (addr_<<1)|I2C_MASTER_WRITE, true);
    i2c_master_write_byte(c, REG_CONVERSION, true);
    i2c_master_start(c);
    i2c_master_write_byte(c, (addr_<<1)|I2C_MASTER_READ, true);
    i2c_master_read(c, d, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(c);
    esp_err_t ret = i2c_master_cmd_begin(port_, c, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(c);
    if(ret != ESP_OK) return false;
    raw = int16_t((d[0]<<8) | d[1]);
    return true;
}

bool ADS1115::waitOSReady(uint16_t cfg_start){
    uint32_t timeout = drTimeoutMs(static_cast<DataRate>((cfg_start>>5)&0x7));
    uint64_t t0 = esp_timer_get_time()/1000ULL;
    for(;;){
        uint16_t c=0;
        if(!readConfig(c)) return false;
        if(c & 0x8000) return true; // OS=1 listo
        if((esp_timer_get_time()/1000ULL - t0) > timeout) return false;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ===== API de alto nivel =====
bool ADS1115::singleShot(Mux mux, PGA pga, DataRate dr, int16_t& raw){
    uint16_t cfg = makeConfig(mux,pga,dr,Mode::SINGLE_SHOT,true);
    if(!writeConfig(cfg)) return false;
    if(!waitOSReady(cfg)) return false;
    return readConversion(raw);
}

bool ADS1115::singleShotMV(Mux mux, PGA pga, DataRate dr, float& mv){
    int16_t raw;
    if(!singleShot(mux,pga,dr,raw)) return false;
    mv = raw * (fsr_mV(pga) / 32768.0f);
    return true;
}

bool ADS1115::startContinuous(Mux mux, PGA pga, DataRate dr){
    uint16_t cfg = makeConfig(mux,pga,dr,Mode::CONTINUOUS,true);
    return writeConfig(cfg);
}

bool ADS1115::readContinuous(int16_t& raw){
    return readConversion(raw);
}

bool ADS1115::stopContinuous(){
    // Pasar a single-shot sin iniciar conversión
    uint16_t cfg = makeConfig(Mux::AIN0_GND, PGA::FS_6V144, DataRate::SPS_128, Mode::SINGLE_SHOT, false);
    return writeConfig(cfg);
}
