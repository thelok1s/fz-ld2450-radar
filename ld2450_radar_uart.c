#include "ld2450_radar_uart.h"
#include "ld2450_radar_parser.h"
#include <furi_hal_power.h>
#include <furi_hal_gpio.h>
#include <furi_hal_resources.h>

#ifndef RECORD_EXPANSION
#define RECORD_EXPANSION "expansion"
typedef struct Expansion Expansion;
void expansion_enable(Expansion* instance);
void expansion_disable(Expansion* instance);
#else
#include <expansion/expansion.h>
#endif

#define UART_CH          FuriHalSerialIdUsart
#define DEFAULT_BAUDRATE 256000
#define RX_BUF_SIZE      1024

#define RAW_BUF_SIZE 16

struct LD2450RadarUart {
    FuriThread* thread;
    FuriStreamBuffer* rx_stream;
    FuriHalSerialHandle* serial_handle;
    Expansion* expansion;
    LD2450RadarUartCallback callback;
    void* callback_context;
    LD2450Parser parser;
    uint32_t current_baud;
    volatile bool worker_running;
    volatile uint32_t total_rx_bytes;
    bool otg_enabled_by_us;
    uint8_t last_raw_bytes[RAW_BUF_SIZE];
    uint8_t last_raw_head;
    FuriMutex* raw_mutex;
};

// ISR Callback
static void ld2450_radar_uart_on_irq_rx(
    FuriHalSerialHandle* handle,
    FuriHalSerialRxEvent event,
    void* context) {
    LD2450RadarUart* uart = (LD2450RadarUart*)context;

    if(event & FuriHalSerialRxEventData) {
        uint8_t data = furi_hal_serial_async_rx(handle);
        uart->total_rx_bytes++;
        furi_stream_buffer_send(uart->rx_stream, &data, 1, 0);

        while(furi_hal_serial_async_rx_available(handle)) {
            data = furi_hal_serial_async_rx(handle);
            uart->total_rx_bytes++;
            furi_stream_buffer_send(uart->rx_stream, &data, 1, 0);
        }
    }
}

uint32_t ld2450_radar_uart_get_rx_bytes(LD2450RadarUart* uart) {
    if(!uart) return 0;
    return uart->total_rx_bytes;
}

uint32_t ld2450_radar_uart_get_baudrate(LD2450RadarUart* uart) {
    if(!uart) return DEFAULT_BAUDRATE;
    return uart->current_baud;
}

void ld2450_radar_uart_set_baudrate(LD2450RadarUart* uart, uint32_t baudrate) {
    if(!uart || !uart->serial_handle) return;
    if(uart->current_baud == baudrate) return;

    furi_hal_serial_async_rx_stop(uart->serial_handle);
    furi_hal_serial_set_br(uart->serial_handle, baudrate);
    uart->current_baud = baudrate;

    /* Flush stream buffer and reset parser to prevent mixing old baud garbage */
    furi_stream_buffer_reset(uart->rx_stream);
    ld2450_radar_parser_init(&uart->parser);

    if(furi_mutex_acquire(uart->raw_mutex, 20) == FuriStatusOk) {
        memset(uart->last_raw_bytes, 0, sizeof(uart->last_raw_bytes));
        uart->last_raw_head = 0;
        furi_mutex_release(uart->raw_mutex);
    }

    /* Keep RX pin pulled up so it never floats when switching bauds */
    furi_hal_gpio_init_ex(
        &gpio_usart_rx,
        GpioModeAltFunctionPushPull,
        GpioPullUp,
        GpioSpeedVeryHigh,
        GpioAltFn7USART1);

    furi_hal_serial_async_rx_start(
        uart->serial_handle, ld2450_radar_uart_on_irq_rx, uart, false);
}

void ld2450_radar_uart_get_last_raw(LD2450RadarUart* uart, uint8_t* out_buf, size_t max_len) {
    if(!uart || !out_buf || max_len == 0) return;
    memset(out_buf, 0, max_len);

    if(furi_mutex_acquire(uart->raw_mutex, 20) == FuriStatusOk) {
        size_t count = (max_len < RAW_BUF_SIZE) ? max_len : RAW_BUF_SIZE;
        for(size_t i = 0; i < count; i++) {
            uint8_t idx = (uart->last_raw_head + RAW_BUF_SIZE - count + i) % RAW_BUF_SIZE;
            out_buf[i] = uart->last_raw_bytes[idx];
        }
        furi_mutex_release(uart->raw_mutex);
    }
}

// Worker Thread
static int32_t ld2450_radar_uart_worker(void* context) {
    LD2450RadarUart* uart = (LD2450RadarUart*)context;
    uint8_t data_buf[64];

    while(uart->worker_running) {
        size_t received =
            furi_stream_buffer_receive(uart->rx_stream, data_buf, sizeof(data_buf), 100);

        if(received > 0) {
            if(furi_mutex_acquire(uart->raw_mutex, 20) == FuriStatusOk) {
                for(size_t i = 0; i < received; i++) {
                    uart->last_raw_bytes[uart->last_raw_head] = data_buf[i];
                    uart->last_raw_head = (uart->last_raw_head + 1) % RAW_BUF_SIZE;
                }
                furi_mutex_release(uart->raw_mutex);
            }

            for(size_t i = 0; i < received; i++) {
                LD2450Data sensor_data;
                if(ld2450_radar_parser_push_byte(
                       &uart->parser, data_buf[i], &sensor_data)) {
                    // Packet complete
                    if(uart->callback) {
                        uart->callback(&sensor_data, uart->callback_context);
                    }
                }
            }
        }
    }

    return 0;
}

LD2450RadarUart* ld2450_radar_uart_alloc(void) {
    LD2450RadarUart* uart = malloc(sizeof(LD2450RadarUart));
    furi_check(uart);
    memset(uart, 0, sizeof(LD2450RadarUart));

    uart->current_baud = DEFAULT_BAUDRATE;

    uart->rx_stream = furi_stream_buffer_alloc(RX_BUF_SIZE, 1);
    furi_check(uart->rx_stream);

    uart->raw_mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    furi_check(uart->raw_mutex);

    uart->thread = furi_thread_alloc();
    furi_check(uart->thread);

    furi_thread_set_name(uart->thread, "LD2450RadarUartWorker");
    furi_thread_set_stack_size(uart->thread, 2048);
    furi_thread_set_callback(uart->thread, ld2450_radar_uart_worker);
    furi_thread_set_context(uart->thread, uart);

    ld2450_radar_parser_init(&uart->parser);

    // Enable 5V on GPIO Pin 1 if on battery so LD2450 is powered
    if(!furi_hal_power_is_otg_enabled()) {
        furi_hal_power_enable_otg();
        uart->otg_enabled_by_us = true;
    }

    return uart;
}

void ld2450_radar_uart_free(LD2450RadarUart* uart) {
    furi_assert(uart);
    ld2450_radar_uart_stop(uart);
    furi_thread_free(uart->thread);
    furi_stream_buffer_free(uart->rx_stream);
    furi_mutex_free(uart->raw_mutex);
    free(uart);
}

bool ld2450_radar_uart_start(LD2450RadarUart* uart) {
    furi_assert(uart);
    uart->worker_running = true;

    // First disable expansion service if running so it releases USART
    if(!uart->expansion) {
        uart->expansion = furi_record_open(RECORD_EXPANSION);
        if(uart->expansion) {
            expansion_disable(uart->expansion);
        }
    }

    // Acquire UART and configure with retry
    int attempts = 30;
    while(attempts > 0) {
        uart->serial_handle = furi_hal_serial_control_acquire(UART_CH);
        if(uart->serial_handle) break;
        furi_delay_ms(10);
        attempts--;
    }

    if(!uart->serial_handle) {
        FURI_LOG_E("LD2450Radar", "Failed to acquire UART handle");
        uart->worker_running = false;
        return false;
    }

    furi_hal_serial_init(uart->serial_handle, uart->current_baud);

    /* Pull up Pin 14 (USART RX) so it does not float and pick up 5V switching noise */
    furi_hal_gpio_init_ex(
        &gpio_usart_rx,
        GpioModeAltFunctionPushPull,
        GpioPullUp,
        GpioSpeedVeryHigh,
        GpioAltFn7USART1);

    // Enable RX
    furi_hal_serial_async_rx_start(
        uart->serial_handle, ld2450_radar_uart_on_irq_rx, uart, false);

    furi_thread_start(uart->thread);
    return true;
}

void ld2450_radar_uart_stop(LD2450RadarUart* uart) {
    furi_assert(uart);

    if(uart->worker_running) {
        uart->worker_running = false;

        // Stop RX first
        if(uart->serial_handle) {
            furi_hal_serial_async_rx_stop(uart->serial_handle);
        }

        // Wait for thread to finish
        furi_thread_join(uart->thread);

        // Deinit and release
        if(uart->serial_handle) {
            furi_hal_serial_deinit(uart->serial_handle);
            furi_hal_serial_control_release(uart->serial_handle);
            uart->serial_handle = NULL;
        }

        // Restore expansion service
        if(uart->expansion) {
            expansion_enable(uart->expansion);
            furi_record_close(RECORD_EXPANSION);
            uart->expansion = NULL;
        }

        // Disable 5V OTG if we enabled it
        if(uart->otg_enabled_by_us) {
            furi_hal_power_disable_otg();
            uart->otg_enabled_by_us = false;
        }

        // Give hardware time to settle
        furi_delay_ms(50);
    }
}

void ld2450_radar_uart_set_handle_rx_data_cb(
    LD2450RadarUart* uart,
    LD2450RadarUartCallback callback,
    void* context) {
    uart->callback = callback;
    uart->callback_context = context;
}

static bool ld2450_send_raw(LD2450RadarUart* uart, const uint8_t* data, size_t len) {
    if(!uart || !uart->serial_handle || !data || len == 0) return false;
    furi_hal_serial_tx(uart->serial_handle, data, len);
    furi_hal_serial_tx_wait_complete(uart->serial_handle);
    return true;
}

bool ld2450_radar_uart_send_mode_multi(LD2450RadarUart* uart) {
    uint8_t buf[32];
    size_t len = ld2450_cmd_enable_config(buf, sizeof(buf));
    if(!ld2450_send_raw(uart, buf, len)) return false;
    furi_delay_ms(50);

    len = ld2450_cmd_set_multi_target(buf, sizeof(buf));
    if(!ld2450_send_raw(uart, buf, len)) return false;
    furi_delay_ms(50);

    len = ld2450_cmd_end_config(buf, sizeof(buf));
    return ld2450_send_raw(uart, buf, len);
}

bool ld2450_radar_uart_send_mode_single(LD2450RadarUart* uart) {
    uint8_t buf[32];
    size_t len = ld2450_cmd_enable_config(buf, sizeof(buf));
    if(!ld2450_send_raw(uart, buf, len)) return false;
    furi_delay_ms(50);

    len = ld2450_cmd_set_single_target(buf, sizeof(buf));
    if(!ld2450_send_raw(uart, buf, len)) return false;
    furi_delay_ms(50);

    len = ld2450_cmd_end_config(buf, sizeof(buf));
    return ld2450_send_raw(uart, buf, len);
}

bool ld2450_radar_uart_send_restart(LD2450RadarUart* uart) {
    uint8_t buf[32];
    size_t len = ld2450_cmd_enable_config(buf, sizeof(buf));
    if(!ld2450_send_raw(uart, buf, len)) return false;
    furi_delay_ms(50);

    len = ld2450_cmd_restart(buf, sizeof(buf));
    if(!ld2450_send_raw(uart, buf, len)) return false;
    furi_delay_ms(50);

    len = ld2450_cmd_end_config(buf, sizeof(buf));
    return ld2450_send_raw(uart, buf, len);
}

bool ld2450_radar_uart_send_baudrate(LD2450RadarUart* uart, uint32_t baud) {
    uint8_t buf[32];
    size_t len = ld2450_cmd_enable_config(buf, sizeof(buf));
    if(!ld2450_send_raw(uart, buf, len)) return false;
    furi_delay_ms(30);

    len = ld2450_cmd_set_baudrate(buf, sizeof(buf), baud);
    if(!ld2450_send_raw(uart, buf, len)) return false;
    furi_delay_ms(30);

    len = ld2450_cmd_restart(buf, sizeof(buf));
    if(!ld2450_send_raw(uart, buf, len)) return false;
    furi_delay_ms(30);

    len = ld2450_cmd_end_config(buf, sizeof(buf));
    return ld2450_send_raw(uart, buf, len);
}

bool ld2450_radar_uart_send_end_config(LD2450RadarUart* uart) {
    uint8_t buf[32];
    size_t len = ld2450_cmd_end_config(buf, sizeof(buf));
    return ld2450_send_raw(uart, buf, len);
}

bool ld2450_radar_uart_send_factory_reset(LD2450RadarUart* uart) {
    if(!uart || !uart->serial_handle) return false;

    static const uint32_t try_bauds[] = {
        57600, 256000, 115200, 230400, 460800, 38400, 19200, 9600
    };
    const size_t num_bauds = sizeof(try_bauds) / sizeof(try_bauds[0]);

    uint8_t cfg_buf[32];
    size_t en_len = ld2450_cmd_enable_config(cfg_buf, sizeof(cfg_buf));
    uint8_t reset_buf[32];
    size_t reset_len = ld2450_cmd_factory_reset(reset_buf, sizeof(reset_buf));
    uint8_t rst_buf[32];
    size_t rst_len = ld2450_cmd_restart(rst_buf, sizeof(rst_buf));
    uint8_t end_buf[32];
    size_t end_len = ld2450_cmd_end_config(end_buf, sizeof(end_buf));

    for(size_t i = 0; i < num_bauds; i++) {
        ld2450_radar_uart_set_baudrate(uart, try_bauds[i]);
        furi_delay_ms(20);

        ld2450_send_raw(uart, cfg_buf, en_len);
        furi_delay_ms(20);

        ld2450_send_raw(uart, reset_buf, reset_len);
        furi_delay_ms(20);

        ld2450_send_raw(uart, rst_buf, rst_len);
        furi_delay_ms(20);

        ld2450_send_raw(uart, end_buf, end_len);
        furi_delay_ms(20);
    }

    // Factory reset returns sensor to 256000 baud
    ld2450_radar_uart_set_baudrate(uart, 256000);
    furi_delay_ms(300);
    ld2450_send_raw(uart, end_buf, end_len);

    return true;
}

bool ld2450_radar_uart_force_sensor_baud(LD2450RadarUart* uart, uint32_t target_baud) {
    if(!uart || !uart->serial_handle) return false;

    static const uint32_t try_bauds[] = {
        57600, 256000, 115200, 230400, 460800, 38400, 19200, 9600
    };
    const size_t num_bauds = sizeof(try_bauds) / sizeof(try_bauds[0]);

    uint8_t cfg_buf[32];
    size_t en_len = ld2450_cmd_enable_config(cfg_buf, sizeof(cfg_buf));
    uint8_t baud_buf[32];
    size_t baud_len = ld2450_cmd_set_baudrate(baud_buf, sizeof(baud_buf), target_baud);
    uint8_t rst_buf[32];
    size_t rst_len = ld2450_cmd_restart(rst_buf, sizeof(rst_buf));
    uint8_t end_buf[32];
    size_t end_len = ld2450_cmd_end_config(end_buf, sizeof(end_buf));

    for(size_t i = 0; i < num_bauds; i++) {
        ld2450_radar_uart_set_baudrate(uart, try_bauds[i]);
        furi_delay_ms(20);

        ld2450_send_raw(uart, cfg_buf, en_len);
        furi_delay_ms(20);

        ld2450_send_raw(uart, baud_buf, baud_len);
        furi_delay_ms(20);

        ld2450_send_raw(uart, rst_buf, rst_len);
        furi_delay_ms(20);

        ld2450_send_raw(uart, end_buf, end_len);
        furi_delay_ms(20);
    }

    // Configure Flipper UART to target baud
    ld2450_radar_uart_set_baudrate(uart, target_baud);
    furi_delay_ms(300);
    ld2450_send_raw(uart, end_buf, end_len);

    return true;
}



