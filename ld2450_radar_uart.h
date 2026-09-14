#pragma once

#include <furi.h>
#include <furi_hal.h>
#include "ld2450_radar_parser.h"

typedef struct LD2450RadarUart LD2450RadarUart;

typedef void (*LD2450RadarUartCallback)(LD2450Data* data, void* context);

LD2450RadarUart* ld2450_radar_uart_alloc(void);
void ld2450_radar_uart_free(LD2450RadarUart* uart);

bool ld2450_radar_uart_start(LD2450RadarUart* uart);
void ld2450_radar_uart_stop(LD2450RadarUart* uart);

uint32_t ld2450_radar_uart_get_rx_bytes(LD2450RadarUart* uart);
uint32_t ld2450_radar_uart_get_baudrate(LD2450RadarUart* uart);
void ld2450_radar_uart_set_baudrate(LD2450RadarUart* uart, uint32_t baudrate);

void ld2450_radar_uart_set_handle_rx_data_cb(
    LD2450RadarUart* uart,
    LD2450RadarUartCallback callback,
    void* context);

bool ld2450_radar_uart_send_mode_multi(LD2450RadarUart* uart);
bool ld2450_radar_uart_send_mode_single(LD2450RadarUart* uart);
bool ld2450_radar_uart_send_restart(LD2450RadarUart* uart);
bool ld2450_radar_uart_send_baudrate(LD2450RadarUart* uart, uint32_t baud);
bool ld2450_radar_uart_send_end_config(LD2450RadarUart* uart);
bool ld2450_radar_uart_send_factory_reset(LD2450RadarUart* uart);
bool ld2450_radar_uart_force_sensor_baud(LD2450RadarUart* uart, uint32_t target_baud);

void ld2450_radar_uart_get_last_raw(LD2450RadarUart* uart, uint8_t* out_buf, size_t max_len);
