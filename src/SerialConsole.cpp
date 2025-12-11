#include "SerialConsole.hpp"
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include "driver/uart.h"

static inline void to_upper_inplace(char* s){
    for (; *s; ++s) *s = (char)toupper((unsigned char)*s);
}

SerialConsole::SerialConsole()
: SerialConsole(Config{}) {}

SerialConsole::SerialConsole(const Config& cfg)
: cfg_(cfg) {}

void SerialConsole::start(){
    if (task_) return;

    printf("\r\n\r\n[CONSOLE] Configurando para Arduino IDE...\r\n");
    
    // Configuración ESPECÍFICA para Arduino IDE
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Reiniciar UART
    uart_driver_delete(cfg_.uart_num);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    uart_param_config(cfg_.uart_num, &uart_config);
    uart_set_pin(cfg_.uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(cfg_.uart_num, 512, 0, 0, NULL, 0);  // Buffer más grande
    
    printf("[CONSOLE] Listo para Arduino IDE Serial Monitor\r\n");
    printf("ESCRIBE 'HELP' Y PRESIONA ENVIAR\r\n");
    
    xTaskCreate(task_trampoline, cfg_.task_name, cfg_.task_stack, this, cfg_.task_prio, &task_);
}

void SerialConsole::task_trampoline(void* arg){
    reinterpret_cast<SerialConsole*>(arg)->task_loop();
}

void SerialConsole::task_loop(){
    static char line[128];
    int idx = 0;
    
    printf("[CONSOLE] Tarea iniciada - Esperando comandos...\r\n");

    while(1) {
        // Método ESPECÍFICO para Arduino IDE
        uint8_t data[128];
        int len = uart_read_bytes(cfg_.uart_num, data, sizeof(data) - 1, 100 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = data[i];
                
                // DEBUG OBLIGATORIO
                printf("[UART] Char: 0x%02X", (uint8_t)c);
                if (c >= 32 && c <= 126) printf(" '%c'", c);
                printf("\r\n");
                fflush(stdout);  // ← IMPORTANTE para Arduino IDE
                
                // Arduino IDE envía \r\n o \n
                if (c == '\r' || c == '\n') {
                    if (idx > 0) {
                        line[idx] = 0;
                        printf("\r\n[CMD] Procesando: '%s'\r\n", line);
                        process_command(line);
                        idx = 0;
                    }
                } 
                // Caracter normal
                else if (idx < (int)sizeof(line) - 1 && c >= 32 && c <= 126) {
                    line[idx++] = c;
                }
            }
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}


void SerialConsole::stop(){
    if (!task_) return;
    vTaskDelete(task_);
    task_ = nullptr;
}

SerialConsole::Mode SerialConsole::mode() const { return mode_; }
uint32_t SerialConsole::plotterPeriodMs() const { return plotter_period_ms_; }

bool SerialConsole::shouldPrintPlotter(uint64_t now_ms){
    if (mode_ != Mode::PLOT) return false;
    uint64_t last = last_plotter_ms_;
    if ((now_ms - last) >= plotter_period_ms_){
        last_plotter_ms_ = now_ms;
        return true;
    }
    return false;
}

void SerialConsole::printPlotHeaderIfNeeded(){
    if (mode_ == Mode::PLOT && plot_header_needed_){
        printf("raw\tfilt\n");
        plot_header_needed_ = false;
    }
}

bool SerialConsole::uart_read_char(char& c, uint32_t timeout_ms) {
    uint8_t byte;
    int len = uart_read_bytes(cfg_.uart_num, &byte, 1, pdMS_TO_TICKS(timeout_ms));
    if (len > 0) {
        c = (char)byte;
        return true;
    }
    return false;
}

void SerialConsole::process_command(const char* command) {
    char cmd[128];
    strncpy(cmd, command, sizeof(cmd)-1);
    cmd[sizeof(cmd)-1] = 0;
    
    // Trim
    char* p = cmd;
    while (*p == ' ' || *p == '\t') p++;
    size_t L = strlen(p);
    while (L > 0 && (p[L-1] == ' ' || p[L-1] == '\t')) p[--L] = 0;
    
    if (L == 0) return;
    
    // Convertir a mayúsculas
    char up[128];
    strncpy(up, p, sizeof(up)-1);
    up[sizeof(up)-1] = 0;
    to_upper_inplace(up);
    
    printf("[CMD] Ejecutando: '%s'\r\n", up);
    
    if (strcmp(up, "HELP") == 0) {
        printf("[CMD] Comandos disponibles:\r\n");
        printf("  HELP    - Muestra esta ayuda\r\n");
        printf("  PING    - Responde PONG\r\n");
        printf("  MODE LOG- Modo logging\r\n");
        printf("  MODE PLOT- Modo plotter\r\n");
        printf("  MODE OFF - Silenciar\r\n");
        printf("  RATE 100- Cambiar periodo plotter\r\n");
    } 
    else if (strcmp(up, "PING") == 0) {
        printf("[CMD] PONG\r\n");
    }
    else if (strncmp(up, "MODE ", 5) == 0) {
        const char* arg = up + 5;
        if (strstr(arg, "LOG")) {
            set_mode(Mode::LOG);
            printf("[CMD] Modo LOG activado\r\n");
        } else if (strstr(arg, "PLOT")) {
            set_mode(Mode::PLOT);
            printf("[CMD] Modo PLOT activado\r\n");
        } else if (strstr(arg, "OFF")) {
            set_mode(Mode::OFF);
            printf("[CMD] Modo OFF activado\r\n");
        } else {
            printf("[CMD] MODE desconocido. Usa: LOG, PLOT, OFF\r\n");
        }
    }
    else if (strncmp(up, "RATE ", 5) == 0) {
        const char* arg = up + 5;
        int ms = atoi(arg);
        if (ms >= 10 && ms <= 5000) {
            set_rate((uint32_t)ms);
            printf("[CMD] Rate configurado a %d ms\r\n", ms);
        } else {
            printf("[CMD] Rate debe estar entre 10-5000 ms\r\n");
        }
    }
    else {
        printf("[CMD] Comando no reconocido: '%s'\r\n", up);
        printf("[CMD] Escribe HELP para ver comandos\r\n");
    }
    
    fflush(stdout);  // ← IMPORTANTE para Arduino IDE
}

void SerialConsole::print_help(){
    printf(
        "[CMD] HELP:\r\n"
        "  MODE LOG        -> imprime Health/INA\r\n"
        "  MODE PLOT       -> imprime 'raw\\tfilt' para Plotter\r\n"
        "  MODE OFF        -> silencia prints\r\n"
        "  RATE <ms>       -> periodo del plotter (ej. RATE 20)\r\n"
        "  PING            -> eco\r\n"
    );
}

void SerialConsole::set_mode(Mode m){
    mode_ = m;
    if (mode_ == Mode::PLOT) plot_header_needed_ = true;
}

void SerialConsole::set_rate(uint32_t ms){
    plotter_period_ms_ = ms;
}