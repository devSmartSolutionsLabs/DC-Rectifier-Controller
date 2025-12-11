#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" {
#include "driver/uart.h"
#include "esp_vfs_dev.h"
}

class SerialConsole {
public:
    enum class Mode : uint8_t { LOG = 0, PLOT = 1, OFF = 2 };

    struct Config {
        uart_port_t uart_num      = UART_NUM_0;
        int         rx_buffer     = 256;
        int         tx_buffer     = 0;
        uint32_t    plotter_ms    = 50;
        uint32_t    min_plotter   = 5;
        uint32_t    max_plotter   = 2000;
        int         task_stack    = 4096;
        UBaseType_t task_prio     = 5;
        const char* task_name     = "SerialCmd";

        constexpr Config(
            uart_port_t u   = UART_NUM_0,
            int         rx  = 256,
            int         tx  = 0,
            uint32_t    pms = 50,
            uint32_t    pmin= 5,
            uint32_t    pmax= 2000,
            int         stk = 4096,
            UBaseType_t pr  = 5,
            const char* nm  = "SerialCmd"
        )
        : uart_num(u), rx_buffer(rx), tx_buffer(tx),
          plotter_ms(pms), min_plotter(pmin), max_plotter(pmax),
          task_stack(stk), task_prio(pr), task_name(nm) {}
    };

    SerialConsole();
    explicit SerialConsole(const Config&);
    ~SerialConsole() = default;

    void start();
    void stop();

    Mode     mode() const;
    uint32_t plotterPeriodMs() const;

    bool shouldPrintPlotter(uint64_t now_ms);
    void printPlotHeaderIfNeeded();

private:
    static void task_trampoline(void* arg);
    void        task_loop();
    void        process_command(const char* command);
    
    void        print_help();
    void        set_mode(Mode m);
    void        set_rate(uint32_t ms);
    bool        uart_read_char(char& c, uint32_t timeout_ms);

private:
    Config      cfg_;
    TaskHandle_t task_ = nullptr;

    volatile Mode     mode_              = Mode::LOG;
    volatile uint32_t plotter_period_ms_ = 50;
    volatile uint64_t last_plotter_ms_   = 0;
    volatile bool     plot_header_needed_ = false;
};