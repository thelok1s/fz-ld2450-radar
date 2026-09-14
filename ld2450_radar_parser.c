#include "ld2450_radar_parser.h"
#include <string.h>

static int16_t ld2450_read_coord_or_speed(uint8_t lo, uint8_t hi) {
    uint16_t raw = (uint16_t)lo | ((uint16_t)hi << 8);

    if(raw & 0x8000) {
        return (int16_t)(raw & 0x7FFF);
    } else {
        return -(int16_t)(raw & 0x7FFF);
    }
}

static uint16_t ld2450_read_u16_le(uint8_t lo, uint8_t hi) {
    return (uint16_t)lo | ((uint16_t)hi << 8);
}

void ld2450_radar_parser_init(LD2450Parser* parser) {
    memset(parser, 0, sizeof(LD2450Parser));
    parser->state = LD2450_PARSER_STATE_WAIT_HEADER;
}

bool ld2450_radar_parser_push_byte(
    LD2450Parser* parser,
    uint8_t byte,
    LD2450Data* out_data) {
    if(parser->state == LD2450_PARSER_STATE_WAIT_HEADER) {
        /* Stream search for frame header: 0xAA, 0xFF, [0x01..0x03], 0x00 */
        if(parser->hdr_match == 0) {
            if(byte == 0xAA) {
                parser->fbuf[0] = byte;
                parser->hdr_match = 1;
            }
        } else if(parser->hdr_match == 1) {
            if(byte == 0xFF) {
                parser->fbuf[1] = byte;
                parser->hdr_match = 2;
            } else if(byte == 0xAA) {
                parser->hdr_match = 1;
            } else {
                parser->hdr_match = 0;
            }
        } else if(parser->hdr_match == 2) {
            if(byte >= 0x01 && byte <= 0x03) {
                parser->fbuf[2] = byte;
                parser->target_count = byte;
                parser->expected_len = 4 + (byte * 8) + 2;
                parser->hdr_match = 3;
            } else if(byte == 0xAA) {
                parser->hdr_match = 1;
            } else {
                parser->hdr_match = 0;
            }
        } else if(parser->hdr_match == 3) {
            if(byte == 0x00) {
                parser->fbuf[3] = byte;
                parser->fpos = 4;
                parser->state = LD2450_PARSER_STATE_READ_FRAME;
                parser->hdr_match = 0;
            } else if(byte == 0xAA) {
                parser->hdr_match = 1;
            } else {
                parser->hdr_match = 0;
            }
        }
    } else if(parser->state == LD2450_PARSER_STATE_READ_FRAME) {
        parser->fbuf[parser->fpos++] = byte;
        if(parser->fpos >= parser->expected_len) {
            parser->state = LD2450_PARSER_STATE_WAIT_HEADER;
            parser->hdr_match = 0;

            uint8_t ftr1 = parser->fbuf[parser->expected_len - 2];
            uint8_t ftr2 = parser->fbuf[parser->expected_len - 1];

            if(ftr1 == 0x55 && ftr2 == 0xCC) {
                memset(out_data, 0, sizeof(LD2450Data));
                out_data->target_count = parser->target_count;

                for(uint8_t i = 0; i < parser->target_count && i < LD2450_MAX_TARGETS; i++) {
                    const uint8_t* d = &parser->fbuf[4 + (i * 8)];
                    uint16_t rx = ld2450_read_u16_le(d[0], d[1]);
                    uint16_t ry = ld2450_read_u16_le(d[2], d[3]);
                    uint16_t rs = ld2450_read_u16_le(d[4], d[5]);
                    uint16_t rr = ld2450_read_u16_le(d[6], d[7]);

                    if(rx == 0 && ry == 0 && rs == 0) {
                        out_data->targets[i].valid = false;
                    } else {
                        out_data->targets[i].x_mm = ld2450_read_coord_or_speed(d[0], d[1]);
                        out_data->targets[i].y_mm = (int16_t)(ry & 0x7FFF);
                        out_data->targets[i].speed_cm_s = ld2450_read_coord_or_speed(d[4], d[5]);
                        out_data->targets[i].distance_resolution_mm = rr;
                        out_data->targets[i].valid = true;
                    }
                }
                return true;
            } else if(byte == 0xAA) {
                parser->hdr_match = 1;
                parser->fbuf[0] = byte;
            }
        }
    }

    return false;
}

size_t ld2450_cmd_enable_config(uint8_t* out_buf, size_t max_len) {
    static const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
    if(max_len < sizeof(cmd)) return 0;
    memcpy(out_buf, cmd, sizeof(cmd));
    return sizeof(cmd);
}

size_t ld2450_cmd_end_config(uint8_t* out_buf, size_t max_len) {
    static const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
    if(max_len < sizeof(cmd)) return 0;
    memcpy(out_buf, cmd, sizeof(cmd));
    return sizeof(cmd);
}

size_t ld2450_cmd_set_multi_target(uint8_t* out_buf, size_t max_len) {
    static const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01};
    if(max_len < sizeof(cmd)) return 0;
    memcpy(out_buf, cmd, sizeof(cmd));
    return sizeof(cmd);
}

size_t ld2450_cmd_set_single_target(uint8_t* out_buf, size_t max_len) {
    static const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01};
    if(max_len < sizeof(cmd)) return 0;
    memcpy(out_buf, cmd, sizeof(cmd));
    return sizeof(cmd);
}

size_t ld2450_cmd_restart(uint8_t* out_buf, size_t max_len) {
    static const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xA3, 0x00, 0x04, 0x03, 0x02, 0x01};
    if(max_len < sizeof(cmd)) return 0;
    memcpy(out_buf, cmd, sizeof(cmd));
    return sizeof(cmd);
}

size_t ld2450_cmd_factory_reset(uint8_t* out_buf, size_t max_len) {
    static const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xA2, 0x00, 0x04, 0x03, 0x02, 0x01};
    if(max_len < sizeof(cmd)) return 0;
    memcpy(out_buf, cmd, sizeof(cmd));
    return sizeof(cmd);
}

size_t ld2450_cmd_set_baudrate(uint8_t* out_buf, size_t max_len, uint32_t baud) {
    uint8_t baud_idx;
    switch(baud) {
    case 9600:
        baud_idx = 1;
        break;
    case 19200:
        baud_idx = 2;
        break;
    case 38400:
        baud_idx = 3;
        break;
    case 57600:
        baud_idx = 4;
        break;
    case 115200:
        baud_idx = 5;
        break;
    case 230400:
        baud_idx = 6;
        break;
    case 256000:
        baud_idx = 7;
        break;
    case 460800:
        baud_idx = 8;
        break;
    default:
        return 0;
    }
    uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xA1, 0x00, baud_idx, 0x00, 0x04, 0x03, 0x02, 0x01};
    if(max_len < sizeof(cmd)) return 0;
    memcpy(out_buf, cmd, sizeof(cmd));
    return sizeof(cmd);
}