#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define LD2450_TARGET_COUNT 3
#define LD2450_MAX_TARGETS 3

typedef struct {
    int16_t x_mm;                      /* Lateral position: + right, - left (mm) */
    int16_t y_mm;                      /* Forward distance from sensor (mm) */
    int16_t speed_cm_s;                /* Speed: + moving away, - approaching (cm/s) */
    uint16_t distance_resolution_mm;   /* Distance resolution (mm) */
    bool valid;                        /* True if actively detected */
} LD2450Target;

typedef struct {
    uint8_t target_count;              /* Number of targets reported (1..3) */
    LD2450Target targets[LD2450_MAX_TARGETS];
} LD2450Data;

typedef enum {
    LD2450_PARSER_STATE_WAIT_HEADER,
    LD2450_PARSER_STATE_READ_FRAME,
} LD2450ParserState;

typedef struct {
    LD2450ParserState state;
    uint8_t hdr_match;
    uint8_t fbuf[32];
    uint8_t fpos;
    uint8_t expected_len;
    uint8_t target_count;
} LD2450Parser;

void ld2450_radar_parser_init(LD2450Parser* parser);

bool ld2450_radar_parser_push_byte(
    LD2450Parser* parser,
    uint8_t byte,
    LD2450Data* out_data);

/* Radar control command generators (return byte count written to out_buf) */
size_t ld2450_cmd_enable_config(uint8_t* out_buf, size_t max_len);
size_t ld2450_cmd_end_config(uint8_t* out_buf, size_t max_len);
size_t ld2450_cmd_set_multi_target(uint8_t* out_buf, size_t max_len);
size_t ld2450_cmd_set_single_target(uint8_t* out_buf, size_t max_len);
size_t ld2450_cmd_restart(uint8_t* out_buf, size_t max_len);
size_t ld2450_cmd_factory_reset(uint8_t* out_buf, size_t max_len);
size_t ld2450_cmd_set_baudrate(uint8_t* out_buf, size_t max_len, uint32_t baud);