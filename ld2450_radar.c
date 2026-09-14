#include <furi.h>
#include <furi_hal.h>
#include <gui/gui.h>
#include <input/input.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ld2450_radar_uart.h"

#define SCREEN_W 128
#define SCREEN_H 64
#define RADAR_CX 64
#define RADAR_CY 63
#define RADAR_R  53
#define STATUS_Y 62

#define S30 27
#define C30 46
#define S60 46
#define C60 27

static const int32_t RANGES_MM[] = {2000, 4000, 6000, 8000};
#define RANGE_COUNT 4

static const uint32_t SUPPORTED_BAUDRATES[] = {
    256000,
    57600,
    115200,
    230400,
    460800,
    38400,
    19200,
    9600
};
#define BAUDRATE_COUNT ((int)(sizeof(SUPPORTED_BAUDRATES) / sizeof(SUPPORTED_BAUDRATES[0])))

typedef enum {
    LD2450ScreenRadar,
    LD2450ScreenText,
    LD2450ScreenDebug,
    LD2450ScreenMenu,
    LD2450ScreenPinout,
    LD2450ScreenAbout,
} LD2450Screen;

typedef enum {
    LD2450UnitsMetric,
    LD2450UnitsImperial,
} LD2450Units;

typedef enum {
    LD2450BaudModeAuto,
    LD2450BaudModeFixed,
} LD2450BaudMode;

typedef enum {
    LD2450MenuRoot,
    LD2450MenuUart,
    LD2450MenuMisc,
} LD2450Submenu;

typedef enum {
    LD2450PerimeterOff,
    LD2450PerimeterNotification,
    LD2450PerimeterAlert,
    LD2450PerimeterAlarm,
} LD2450PerimeterMode;

#define ROOT_MENU_COUNT 6
#define UART_MENU_COUNT 5
#define MISC_MENU_COUNT 3

typedef struct {
    LD2450Data data;
    bool connected;
    bool uart_init_failed;
    uint32_t last_packet_time;
    uint32_t packet_count;
    uint32_t rx_bytes;
    uint32_t last_rx_bytes_check;
    uint32_t current_baud;
    LD2450BaudMode baud_mode;
    uint8_t range_idx;
    LD2450Units units;
    LD2450Screen screen;
    LD2450Submenu menu_submenu;
    int8_t menu_root_idx;
    int8_t menu_uart_idx;
    int8_t menu_misc_idx;
    uint32_t menu_selected_tick;
    LD2450PerimeterMode perimeter_mode;
    bool perimeter_alarm_active;
    bool debug_enabled;
    char toast_msg[32];
    uint32_t toast_expiry;
    uint8_t last_raw_bytes[8];
} LD2450RadarModel;

typedef struct {
    LD2450RadarUart* uart;
    Gui* gui;
    ViewPort* view_port;
    FuriMessageQueue* event_queue;
    LD2450RadarModel model;
    FuriMutex* model_mutex;
    bool speaker_acquired;
    bool target_in_range_prev[LD2450_MAX_TARGETS];
    uint32_t target_last_seen_tick[LD2450_MAX_TARGETS];
    uint32_t alarm_last_target_tick;
    uint32_t tone_stop_tick;
    uint32_t alert_start_tick;
    uint8_t alert_target;
} LD2450RadarApp;

typedef enum {
    LD2450RadarEventInput,
    LD2450RadarEventDataUpdate,
} LD2450RadarEventType;

typedef struct {
    LD2450RadarEventType type;
    InputEvent input;
} LD2450RadarEvent;

static float mm_to_m(int16_t mm) {
    return (float)mm / 1000.0f;
}

static float mm_to_ft(int16_t mm) {
    return (float)mm / 304.8f;
}

static float cms_to_ms(int16_t cms) {
    return (float)cms / 100.0f;
}

static float cms_to_fts(int16_t cms) {
    return (float)cms / 30.48f;
}

static void set_toast(LD2450RadarModel* model, const char* msg, uint32_t duration_ms) {
    strncpy(model->toast_msg, msg, sizeof(model->toast_msg) - 1);
    model->toast_msg[sizeof(model->toast_msg) - 1] = '\0';
    model->toast_expiry = furi_get_tick() + duration_ms;
}

static void ld2450_radar_uart_callback(LD2450Data* data, void* context) {
    LD2450RadarApp* app = context;

    furi_mutex_acquire(app->model_mutex, FuriWaitForever);
    app->model.data = *data;
    app->model.connected = true;
    app->model.last_packet_time = furi_get_tick();
    app->model.packet_count++;
    furi_mutex_release(app->model_mutex);
}

/* Semicircular arc centred at (cx, cy) with radius r (upper half). */
static void draw_arc(Canvas* canvas, int32_t cx, int32_t cy, int32_t r, bool dotted) {
    int32_t x = 0;
    int32_t y = r;
    int32_t d = 3 - 2 * r;
    int32_t step = 0;

    while(y >= x) {
        bool draw = !dotted || ((step & 3) < 2);
        if(draw) {
            canvas_draw_dot(canvas, cx + x, cy - y);
            canvas_draw_dot(canvas, cx - x, cy - y);
            canvas_draw_dot(canvas, cx + y, cy - x);
            canvas_draw_dot(canvas, cx - y, cy - x);
        }
        step++;
        if(d <= 0) {
            d = d + 4 * x + 6;
        } else {
            d = d + 4 * (x - y) + 10;
            y--;
        }
        x++;
    }
}

/* Dotted Bresenham line */
static void draw_dotted_line(Canvas* canvas, int32_t x0, int32_t y0, int32_t x1, int32_t y1) {
    int32_t dx = (x1 > x0 ? x1 - x0 : x0 - x1);
    int32_t sx = (x0 < x1 ? 1 : -1);
    int32_t dy = -(y1 > y0 ? y1 - y0 : y0 - y1);
    int32_t sy = (y0 < y1 ? 1 : -1);
    int32_t err = dx + dy;
    int32_t tog = 0;

    for(;;) {
        if((tog & 3) < 2 && x0 >= 0 && x0 < SCREEN_W && y0 >= 0 && y0 < SCREEN_H) {
            canvas_draw_dot(canvas, x0, y0);
        }
        tog++;
        if(x0 == x1 && y0 == y1) break;
        int32_t e2 = 2 * err;
        if(e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if(e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

static void ld2450_draw_radar(Canvas* canvas, LD2450RadarModel* model) {
    int32_t max_range_mm = RANGES_MM[model->range_idx];

    /* Horizontal baseline */
    canvas_draw_line(canvas, RADAR_CX - RADAR_R, RADAR_CY, RADAR_CX + RADAR_R, RADAR_CY);

    /* Range arcs: inner dotted, outermost solid */
    int32_t step_mm = (model->range_idx == 0) ? 1000 : 2000;
    for(int32_t r_mm = step_mm; r_mm <= max_range_mm; r_mm += step_mm) {
        int32_t r_px = (r_mm * RADAR_R) / max_range_mm;
        bool is_outer = (r_mm == max_range_mm);
        draw_arc(canvas, RADAR_CX, RADAR_CY, r_px, !is_outer);
    }

    /* Sector divider lines at +-30 deg and +-60 deg */
    draw_dotted_line(canvas, RADAR_CX, RADAR_CY, RADAR_CX - S60, RADAR_CY - C60);
    draw_dotted_line(canvas, RADAR_CX, RADAR_CY, RADAR_CX - S30, RADAR_CY - C30);
    draw_dotted_line(canvas, RADAR_CX, RADAR_CY, RADAR_CX + S30, RADAR_CY - C30);
    draw_dotted_line(canvas, RADAR_CX, RADAR_CY, RADAR_CX + S60, RADAR_CY - C60);

    /* Center line straight ahead */
    canvas_draw_line(canvas, RADAR_CX, RADAR_CY, RADAR_CX, RADAR_CY - RADAR_R);

    /* Sensor position dot */
    canvas_draw_disc(canvas, RADAR_CX, RADAR_CY, 2);

    /* Top-Left Information:
     * Line 1 (Y=7): Range & Baud rate
     * Line 2 (Y=16): Target 1 distance
     */
    canvas_set_font(canvas, FontSecondary);
    char buf[24];

    /* Line 1: Range & Baud */
    if(model->units == LD2450UnitsMetric) {
        snprintf(buf, sizeof(buf), "%dm %luK", (int)(max_range_mm / 1000), model->current_baud / 1000);
    } else {
        snprintf(buf, sizeof(buf), "%dft %luK", (int)roundf(mm_to_ft(max_range_mm)), model->current_baud / 1000);
    }
    canvas_draw_str(canvas, 0, 7, buf);

    /* Line 2: Target 1 */
    if(model->data.target_count > 0 && model->data.targets[0].valid) {
        float d1 = sqrtf((float)model->data.targets[0].x_mm * model->data.targets[0].x_mm +
                         (float)model->data.targets[0].y_mm * model->data.targets[0].y_mm);
        if(model->units == LD2450UnitsMetric) {
            snprintf(buf, sizeof(buf), "1: %.1fm", (double)mm_to_m(d1));
        } else {
            snprintf(buf, sizeof(buf), "1: %.1fft", (double)mm_to_ft(d1));
        }
    } else {
        snprintf(buf, sizeof(buf), "1: ---");
    }
    canvas_draw_str(canvas, 0, 16, buf);

    /* Top-Center: Perimeter Security Badge */
    if(model->perimeter_mode != LD2450PerimeterOff) {
        if(model->perimeter_mode == LD2450PerimeterAlarm) {
            if(model->perimeter_alarm_active) {
                bool blink = ((furi_get_tick() / 250) % 2) == 0;
                if(blink) {
                    canvas_set_color(canvas, ColorBlack);
                    canvas_draw_box(canvas, 42, 0, 44, 9);
                    canvas_set_color(canvas, ColorWhite);
                    canvas_draw_str(canvas, 44, 7, "! ALARM !");
                    canvas_set_color(canvas, ColorBlack);
                } else {
                    canvas_draw_frame(canvas, 42, 0, 44, 9);
                    canvas_draw_str(canvas, 44, 7, "! ALARM !");
                }
            } else {
                canvas_draw_str(canvas, 46, 7, "[ARMED]");
            }
        } else if(model->perimeter_mode == LD2450PerimeterNotification) {
            canvas_draw_str(canvas, 46, 7, "[NOTIF]");
        } else if(model->perimeter_mode == LD2450PerimeterAlert) {
            canvas_draw_str(canvas, 46, 7, "[ALERT]");
        }
    }

    /* Top-Right Information:
     * Line 1 (Y=7): Target 2 distance
     * Line 2 (Y=16): Target 3 distance
     */
    /* Line 1: Target 2 */
    if(model->data.target_count > 1 && model->data.targets[1].valid) {
        float d2 = sqrtf((float)model->data.targets[1].x_mm * model->data.targets[1].x_mm +
                         (float)model->data.targets[1].y_mm * model->data.targets[1].y_mm);
        if(model->units == LD2450UnitsMetric) {
            snprintf(buf, sizeof(buf), "2: %.1fm", (double)mm_to_m(d2));
        } else {
            snprintf(buf, sizeof(buf), "2: %.1fft", (double)mm_to_ft(d2));
        }
    } else {
        snprintf(buf, sizeof(buf), "2: ---");
    }
    int16_t w2 = canvas_string_width(canvas, buf);
    canvas_draw_str(canvas, SCREEN_W - w2 - 1, 7, buf);

    /* Line 2: Target 3 */
    if(model->data.target_count > 2 && model->data.targets[2].valid) {
        float d3 = sqrtf((float)model->data.targets[2].x_mm * model->data.targets[2].x_mm +
                         (float)model->data.targets[2].y_mm * model->data.targets[2].y_mm);
        if(model->units == LD2450UnitsMetric) {
            snprintf(buf, sizeof(buf), "3: %.1fm", (double)mm_to_m(d3));
        } else {
            snprintf(buf, sizeof(buf), "3: %.1fft", (double)mm_to_ft(d3));
        }
    } else {
        snprintf(buf, sizeof(buf), "3: ---");
    }
    int16_t w3 = canvas_string_width(canvas, buf);
    canvas_draw_str(canvas, SCREEN_W - w3 - 1, 16, buf);

    /* Plot targets */
    for(uint8_t i = 0; i < model->data.target_count && i < LD2450_MAX_TARGETS; i++) {
        LD2450Target* t = &model->data.targets[i];
        if(!t->valid) continue;

        int32_t px = RADAR_CX + ((int32_t)t->x_mm * RADAR_R) / max_range_mm;
        int32_t py = RADAR_CY - ((int32_t)t->y_mm * RADAR_R) / max_range_mm;

        if(px < 2) px = 2;
        if(px > SCREEN_W - 3) px = SCREEN_W - 3;
        if(py < 2) py = 2;
        if(py > RADAR_CY - 1) py = RADAR_CY - 1;

        /* Target dot */
        canvas_draw_disc(canvas, px, py, 3);

        /* Target label with 1px border */
        char lbl[3];
        snprintf(lbl, sizeof(lbl), "%d", i + 1);

        int32_t lx = (px > SCREEN_W - 10) ? (px - 8) : (px + 4);
        int32_t ly = (py < 9) ? (py + 9) : (py - 3);

        canvas_set_color(canvas, ColorWhite);
        canvas_draw_box(canvas, lx - 1, ly - 7, 7, 9);
        canvas_set_color(canvas, ColorBlack);
        canvas_draw_str(canvas, lx, ly, lbl);
    }

    /* Floating toast notification if active */
    if(model->toast_expiry > furi_get_tick() && model->toast_msg[0] != '\0') {
        int16_t tw = canvas_string_width(canvas, model->toast_msg);
        int16_t tx = (SCREEN_W - tw) / 2;
        canvas_set_color(canvas, ColorWhite);
        canvas_draw_box(canvas, tx - 3, 50, tw + 6, 12);
        canvas_set_color(canvas, ColorBlack);
        canvas_draw_frame(canvas, tx - 3, 50, tw + 6, 12);
        canvas_draw_str(canvas, tx, 59, model->toast_msg);
    }
}

static void ld2450_draw_text(Canvas* canvas, LD2450RadarModel* model) {
    char buf[48];

    for(uint8_t i = 0; i < LD2450_MAX_TARGETS; i++) {
        LD2450Target* t = &model->data.targets[i];
        uint8_t y_pos1 = 9 + (i * 21);
        uint8_t y_pos2 = y_pos1 + 10;

        if(i < model->data.target_count && t->valid) {
            float dist_val = sqrtf((float)t->x_mm * t->x_mm + (float)t->y_mm * t->y_mm);
            float angle_deg = atan2f((float)t->x_mm, (float)t->y_mm) * (180.0f / (float)M_PI);

            if(model->units == LD2450UnitsMetric) {
                snprintf(
                    buf,
                    sizeof(buf),
                    "T%d: %.1fm  %+d deg  %+.1fm/s",
                    i + 1,
                    (double)mm_to_m(dist_val),
                    (int)roundf(angle_deg),
                    (double)cms_to_ms(t->speed_cm_s));
                canvas_draw_str(canvas, 0, y_pos1, buf);

                snprintf(
                    buf,
                    sizeof(buf),
                    "   X:%.1fm Y:%.1fm Res:%dmm",
                    (double)mm_to_m(t->x_mm),
                    (double)mm_to_m(t->y_mm),
                    t->distance_resolution_mm);
                canvas_draw_str(canvas, 0, y_pos2, buf);
            } else {
                snprintf(
                    buf,
                    sizeof(buf),
                    "T%d: %.1fft %+d deg %+.1fft/s",
                    i + 1,
                    (double)mm_to_ft(dist_val),
                    (int)roundf(angle_deg),
                    (double)cms_to_fts(t->speed_cm_s));
                canvas_draw_str(canvas, 0, y_pos1, buf);

                snprintf(
                    buf,
                    sizeof(buf),
                    "   X:%.1fft Y:%.1fft Res:%dmm",
                    (double)mm_to_ft(t->x_mm),
                    (double)mm_to_ft(t->y_mm),
                    t->distance_resolution_mm);
                canvas_draw_str(canvas, 0, y_pos2, buf);
            }
        } else {
            snprintf(buf, sizeof(buf), "T%d: [No target detected]", i + 1);
            canvas_draw_str(canvas, 0, y_pos1, buf);
        }
    }

    /* Floating toast notification if active */
    if(model->toast_expiry > furi_get_tick() && model->toast_msg[0] != '\0') {
        int16_t tw = canvas_string_width(canvas, model->toast_msg);
        int16_t tx = (SCREEN_W - tw) / 2;
        canvas_set_color(canvas, ColorWhite);
        canvas_draw_box(canvas, tx - 3, 50, tw + 6, 12);
        canvas_set_color(canvas, ColorBlack);
        canvas_draw_frame(canvas, tx - 3, 50, tw + 6, 12);
        canvas_draw_str(canvas, tx, 59, model->toast_msg);
    }
}

static void ld2450_draw_debug(Canvas* canvas, LD2450RadarModel* model) {
    char buf[48];

    canvas_draw_str(canvas, 0, 8, "DEBUG MONITOR");
    snprintf(buf, sizeof(buf), "%luK", model->current_baud / 1000);
    int16_t bw = canvas_string_width(canvas, buf);
    canvas_draw_str(canvas, SCREEN_W - bw - 1, 8, buf);
    canvas_draw_line(canvas, 0, 10, SCREEN_W - 1, 10);

    snprintf(buf, sizeof(buf), "RX: %lu B  Pkts: %lu", model->rx_bytes, model->packet_count);
    canvas_draw_str(canvas, 0, 19, buf);

    snprintf(
        buf,
        sizeof(buf),
        "Raw: %02X%02X %02X%02X %02X%02X %02X%02X",
        model->last_raw_bytes[0],
        model->last_raw_bytes[1],
        model->last_raw_bytes[2],
        model->last_raw_bytes[3],
        model->last_raw_bytes[4],
        model->last_raw_bytes[5],
        model->last_raw_bytes[6],
        model->last_raw_bytes[7]);
    canvas_draw_str(canvas, 0, 28, buf);

    if(model->packet_count > 0) {
        canvas_draw_str(canvas, 0, 38, "* OK! Sensor streaming");
        uint8_t act = 0;
        for(uint8_t i = 0; i < model->data.target_count && i < LD2450_MAX_TARGETS; i++) {
            if(model->data.targets[i].valid) act++;
        }
        snprintf(buf, sizeof(buf), "Active targets: %d", act);
        canvas_draw_str(canvas, 0, 47, buf);
    } else if(model->rx_bytes > 30 && (model->last_raw_bytes[0] == 0x10 || model->last_raw_bytes[0] == 0x08)) {
        canvas_draw_str(canvas, 0, 38, "Sensor detected at 57600!");
        canvas_draw_str(canvas, 0, 47, "UP/DN: 57.6k | OK: Force 256k");
    } else if(model->rx_bytes > 30) {
        canvas_draw_str(canvas, 0, 38, "Bytes seen (baud mismatch)");
        canvas_draw_str(canvas, 0, 47, "UP/DN: Change | OK: Force 256k");
    } else {
        canvas_draw_str(canvas, 0, 38, "No data. Check Pin 1 (5V)");
        canvas_draw_str(canvas, 0, 47, "and Pin 13/14 connections");
    }

    canvas_draw_line(canvas, 0, 52, SCREEN_W - 1, 52);
    if(model->toast_expiry > furi_get_tick() && model->toast_msg[0] != '\0') {
        canvas_draw_str(canvas, 0, STATUS_Y, model->toast_msg);
    } else {
        canvas_draw_str(canvas, 0, STATUS_Y, "OK:Force256k");
        int16_t w = canvas_string_width(canvas, "UP/DN:Baud");
        canvas_draw_str(canvas, SCREEN_W - w - 1, STATUS_Y, "UP/DN:Baud");
    }
}

static void ld2450_draw_pinout(Canvas* canvas, LD2450RadarModel* model) {
    UNUSED(model);
    canvas_draw_str(canvas, 0, 8, "LD2450 WIRING GUIDE");
    canvas_draw_line(canvas, 0, 10, SCREEN_W - 1, 10);

    canvas_draw_str(canvas, 0, 19, "Pin 1 (5V)  -> VCC (5V)");
    canvas_draw_str(canvas, 0, 27, "Pin 8 (GND) -> GND");
    canvas_draw_str(canvas, 0, 35, "Pin 13 (TX) -> RX (Sensor)");
    canvas_draw_str(canvas, 0, 43, "Pin 14 (RX) -> TX (Sensor)");
    canvas_draw_str(canvas, 0, 51, "* Note: Cross TX and RX!");

    canvas_draw_line(canvas, 0, 53, SCREEN_W - 1, 53);
    canvas_draw_str(canvas, 0, STATUS_Y, "Disconnect BLE app");
    int16_t w = canvas_string_width(canvas, "Back:Ret");
    canvas_draw_str(canvas, SCREEN_W - w - 1, STATUS_Y, "Back:Ret");
}

static void ld2450_draw_about(Canvas* canvas, LD2450RadarModel* model) {
    UNUSED(model);
    canvas_draw_str(canvas, 0, 8, "About LD2450 Radar");
    canvas_draw_line(canvas, 0, 10, SCREEN_W - 1, 10);

    canvas_draw_str(canvas, 0, 20, "LD2450 Radar Tracker");
    canvas_draw_str(canvas, 0, 29, "24GHz mmWave Radar");
    canvas_draw_str(canvas, 0, 38, "API 87.1 | 256k Baud");
    canvas_draw_str(canvas, 0, 47, "Multi-target 3D Tracking");

    canvas_draw_line(canvas, 0, 53, SCREEN_W - 1, 53);
    canvas_draw_str(canvas, 0, STATUS_Y, "TownOfBulls/radar");
    int16_t w = canvas_string_width(canvas, "Back:Ret");
    canvas_draw_str(canvas, SCREEN_W - w - 1, STATUS_Y, "Back:Ret");
}

static void ld2450_draw_menu_item(
    Canvas* canvas,
    int y,
    const char* text,
    bool is_selected,
    uint32_t select_tick) {
    if(is_selected) {
        canvas_set_color(canvas, ColorBlack);
        canvas_draw_box(canvas, 0, y - 8, SCREEN_W, 10);
        canvas_set_color(canvas, ColorWhite);

        uint16_t str_w = canvas_string_width(canvas, text);
        const int16_t visible_w = SCREEN_W - 6; /* 122px */
        int32_t offset = 0;

        if(str_w > visible_w) {
            int32_t overflow = str_w - visible_w;
            const uint32_t HOLD_START_MS = 800;
            const uint32_t MS_PER_PX = 35;
            const uint32_t scroll_duration = overflow * MS_PER_PX;
            const uint32_t HOLD_END_MS = 1000;
            const uint32_t total_cycle = HOLD_START_MS + scroll_duration + HOLD_END_MS + 400;

            uint32_t elapsed = furi_get_tick() - select_tick;
            uint32_t phase = elapsed % total_cycle;

            if(phase < HOLD_START_MS) {
                offset = 0;
            } else if(phase < HOLD_START_MS + scroll_duration) {
                offset = (phase - HOLD_START_MS) / MS_PER_PX;
            } else if(phase < HOLD_START_MS + scroll_duration + HOLD_END_MS) {
                offset = overflow;
            } else {
                offset = 0;
            }
        }

        canvas_draw_str(canvas, 3 - offset, y, text);
        canvas_set_color(canvas, ColorBlack);
    } else {
        canvas_draw_str(canvas, 3, y, text);
    }
}

static void ld2450_draw_menu(Canvas* canvas, LD2450RadarModel* model) {
    const char* items[6];
    char item_buf[6][36];
    int item_count = 0;
    int selected = 0;
    const char* header_title = "";

    switch(model->menu_submenu) {
    case LD2450MenuUart:
        header_title = "Settings > UART Cmds";
        item_count = UART_MENU_COUNT;
        selected = model->menu_uart_idx;

        items[0] = "Force 256k (Auto-Sweep)";
        items[1] = "Factory Reset (to 256k)";
        items[2] = "Wake / End BLE Config";
        items[3] = "Force Multi-Target";
        items[4] = "Force Single-Target";
        break;

    case LD2450MenuMisc:
        header_title = "Settings > Misc";
        item_count = MISC_MENU_COUNT;
        selected = model->menu_misc_idx;

        snprintf(
            item_buf[0],
            sizeof(item_buf[0]),
            "Debug Screen: %s",
            model->debug_enabled ? "ON" : "OFF");
        items[0] = item_buf[0];

        items[1] = "Help: Wiring Guide >";
        items[2] = "About LD2450 Radar >";
        break;

    case LD2450MenuRoot:
    default:
        header_title = "Settings";
        item_count = ROOT_MENU_COUNT;
        selected = model->menu_root_idx;

        snprintf(
            item_buf[0],
            sizeof(item_buf[0]),
            "Units: %s",
            (model->units == LD2450UnitsMetric) ? "Metric (m)" : "Imperial (ft)");
        items[0] = item_buf[0];

        if(model->baud_mode == LD2450BaudModeAuto) {
            snprintf(item_buf[1], sizeof(item_buf[1]), "Baud: Auto (%lu)", model->current_baud);
        } else {
            snprintf(item_buf[1], sizeof(item_buf[1]), "Baud: %lu Fixed", model->current_baud);
        }
        items[1] = item_buf[1];

        snprintf(
            item_buf[2],
            sizeof(item_buf[2]),
            "Zoom Range: %dm (%dft)",
            (int)(RANGES_MM[model->range_idx] / 1000),
            (int)roundf(mm_to_ft(RANGES_MM[model->range_idx])));
        items[2] = item_buf[2];

        const char* perim_str = "OFF";
        if(model->perimeter_mode == LD2450PerimeterNotification) {
            perim_str = "Notification";
        } else if(model->perimeter_mode == LD2450PerimeterAlert) {
            perim_str = "Alert";
        } else if(model->perimeter_mode == LD2450PerimeterAlarm) {
            perim_str = "Alarm";
        }
        snprintf(item_buf[3], sizeof(item_buf[3]), "Perimeter: %s", perim_str);
        items[3] = item_buf[3];

        items[4] = "UART Commands >";
        items[5] = "Miscellaneous >";
        break;
    }

    /* Draw Header */
    canvas_draw_str(canvas, 0, 8, header_title);
    canvas_draw_line(canvas, 0, 10, SCREEN_W - 1, 10);

    /* Display items with scrolling window */
    int start_idx = 0;
    if(selected >= 4) {
        start_idx = selected - 3;
    }
    if(start_idx > item_count - 4) {
        start_idx = item_count - 4;
    }
    if(start_idx < 0) start_idx = 0;

    for(int i = 0; i < 4 && (start_idx + i) < item_count; i++) {
        int idx = start_idx + i;
        int y = 20 + (i * 10);

        ld2450_draw_menu_item(
            canvas,
            y,
            items[idx],
            (idx == selected),
            model->menu_selected_tick);
    }

    /* Bottom footer */
    canvas_draw_line(canvas, 0, 53, SCREEN_W - 1, 53);
    if(model->toast_expiry > furi_get_tick() && model->toast_msg[0] != '\0') {
        canvas_draw_str(canvas, 1, STATUS_Y, model->toast_msg);
    } else {
        if(model->menu_submenu == LD2450MenuRoot) {
            canvas_draw_str(canvas, 1, STATUS_Y, "OK:Select");
            int16_t wb = canvas_string_width(canvas, "Back:Radar");
            canvas_draw_str(canvas, SCREEN_W - wb - 1, STATUS_Y, "Back:Radar");
        } else {
            canvas_draw_str(canvas, 1, STATUS_Y, "OK:Execute");
            int16_t wb = canvas_string_width(canvas, "Back:Menu");
            canvas_draw_str(canvas, SCREEN_W - wb - 1, STATUS_Y, "Back:Menu");
        }
    }
}

static void ld2450_radar_draw_callback(Canvas* canvas, void* context) {
    LD2450RadarApp* app = context;

    furi_mutex_acquire(app->model_mutex, FuriWaitForever);
    LD2450RadarModel model = app->model;
    furi_mutex_release(app->model_mutex);

    canvas_clear(canvas);
    canvas_set_font(canvas, FontSecondary);

    if(model.uart_init_failed) {
        canvas_draw_str(canvas, 10, 24, "UART INIT ERROR");
        canvas_draw_str(canvas, 10, 36, "Expansion in use or busy");
        canvas_draw_str(canvas, 10, 48, "Check GPIO Pin 13 & 14");
        return;
    }

    switch(model.screen) {
    case LD2450ScreenRadar:
        ld2450_draw_radar(canvas, &model);
        break;
    case LD2450ScreenText:
        ld2450_draw_text(canvas, &model);
        break;
    case LD2450ScreenDebug:
        ld2450_draw_debug(canvas, &model);
        break;
    case LD2450ScreenMenu:
        ld2450_draw_menu(canvas, &model);
        break;
    case LD2450ScreenPinout:
        ld2450_draw_pinout(canvas, &model);
        break;
    case LD2450ScreenAbout:
        ld2450_draw_about(canvas, &model);
        break;
    }
}

static void ld2450_radar_input_callback(InputEvent* input_event, void* context) {
    LD2450RadarApp* app = context;

    LD2450RadarEvent event = {
        .type = LD2450RadarEventInput,
        .input = *input_event,
    };

    furi_message_queue_put(app->event_queue, &event, 0);
}

static void ld2450_radar_tick_callback(void* context) {
    LD2450RadarApp* app = context;

    LD2450RadarEvent event = {
        .type = LD2450RadarEventDataUpdate,
    };

    furi_message_queue_put(app->event_queue, &event, 0);
}

static void cycle_baud(LD2450RadarApp* app, int direction) {
    furi_mutex_acquire(app->model_mutex, FuriWaitForever);
    uint32_t cur = app->model.current_baud;
    int idx = 0;
    for(int i = 0; i < BAUDRATE_COUNT; i++) {
        if(SUPPORTED_BAUDRATES[i] == cur) {
            idx = i;
            break;
        }
    }
    idx = (idx + direction + BAUDRATE_COUNT) % BAUDRATE_COUNT;
    uint32_t next = SUPPORTED_BAUDRATES[idx];
    ld2450_radar_uart_set_baudrate(app->uart, next);
    app->model.current_baud = next;
    app->model.baud_mode = LD2450BaudModeFixed;
    app->model.last_packet_time = furi_get_tick();
    app->model.last_rx_bytes_check = app->model.rx_bytes;
    char msg[32];
    snprintf(msg, sizeof(msg), "Baud: %lu (Fixed)", next);
    set_toast(&app->model, msg, 1500);
    furi_mutex_release(app->model_mutex);
}

static void menu_change_selection(LD2450RadarModel* model, int delta) {
    switch(model->menu_submenu) {
    case LD2450MenuUart:
        model->menu_uart_idx = (model->menu_uart_idx + delta + UART_MENU_COUNT) % UART_MENU_COUNT;
        break;
    case LD2450MenuMisc:
        model->menu_misc_idx = (model->menu_misc_idx + delta + MISC_MENU_COUNT) % MISC_MENU_COUNT;
        break;
    case LD2450MenuRoot:
    default:
        model->menu_root_idx = (model->menu_root_idx + delta + ROOT_MENU_COUNT) % ROOT_MENU_COUNT;
        break;
    }
    model->menu_selected_tick = furi_get_tick();
}

static void ld2450_perimeter_audio_update(LD2450RadarApp* app) {
    uint32_t now = furi_get_tick();
    int32_t max_range_mm = RANGES_MM[app->model.range_idx];

    bool any_in_range = false;
    bool target_entered[LD2450_MAX_TARGETS] = {false, false, false};

    for(uint8_t i = 0; i < LD2450_MAX_TARGETS; i++) {
        bool in_range = false;
        if(i < app->model.data.target_count && app->model.data.targets[i].valid) {
            float dist = sqrtf((float)app->model.data.targets[i].x_mm * app->model.data.targets[i].x_mm +
                               (float)app->model.data.targets[i].y_mm * app->model.data.targets[i].y_mm);
            if(dist <= (float)max_range_mm) {
                in_range = true;
                any_in_range = true;
            }
        }
        if(in_range) {
            /* Debounce: target considered "entered" if it wasn't seen in the last 1500ms */
            if((now - app->target_last_seen_tick[i] > 1500) || (app->target_last_seen_tick[i] == 0)) {
                target_entered[i] = true;
            }
            app->target_last_seen_tick[i] = now;
        }
        app->target_in_range_prev[i] = in_range;
    }

    if(any_in_range) {
        app->alarm_last_target_tick = now;
    }

    LD2450PerimeterMode mode = app->model.perimeter_mode;

    if(mode == LD2450PerimeterAlarm) {
        /* Alarm sounds as long as targets are present; holds 800ms to avoid stutter */
        bool alarm_firing = any_in_range ||
                            (app->alarm_last_target_tick > 0 && (now - app->alarm_last_target_tick < 800));
        app->model.perimeter_alarm_active = alarm_firing;

        if(alarm_firing) {
            if(!app->speaker_acquired) {
                if(furi_hal_speaker_acquire(10)) {
                    app->speaker_acquired = true;
                }
            }
            if(app->speaker_acquired) {
                /* Alternating two-tone siren every 150ms */
                uint32_t phase = (now / 150) % 2;
                float freq = (phase == 0) ? 880.0f : 1320.0f;
                furi_hal_speaker_start(freq, 0.90f);
            }
        } else {
            if(app->speaker_acquired) {
                furi_hal_speaker_stop();
                furi_hal_speaker_release();
                app->speaker_acquired = false;
            }
        }
    } else if(mode == LD2450PerimeterNotification) {
        app->model.perimeter_alarm_active = false;
        int8_t new_target = -1;
        for(uint8_t i = 0; i < LD2450_MAX_TARGETS; i++) {
            if(target_entered[i]) {
                new_target = i;
                break;
            }
        }

        if(new_target >= 0) {
            if(!app->speaker_acquired) {
                if(furi_hal_speaker_acquire(10)) {
                    app->speaker_acquired = true;
                }
            }
            if(app->speaker_acquired) {
                static const float NOTIF_FREQS[3] = {659.0f, 988.0f, 1318.0f}; /* E5, B5, E6 */
                furi_hal_speaker_start(NOTIF_FREQS[new_target], 0.30f);
                app->tone_stop_tick = now + 90;
            }
        } else if(app->speaker_acquired && now >= app->tone_stop_tick) {
            furi_hal_speaker_stop();
            furi_hal_speaker_release();
            app->speaker_acquired = false;
        }
    } else if(mode == LD2450PerimeterAlert) {
        app->model.perimeter_alarm_active = false;
        int8_t new_target = -1;
        for(uint8_t i = 0; i < LD2450_MAX_TARGETS; i++) {
            if(target_entered[i]) {
                new_target = i;
                break;
            }
        }

        if(new_target >= 0) {
            app->alert_target = new_target;
            app->alert_start_tick = now;
            if(!app->speaker_acquired) {
                if(furi_hal_speaker_acquire(10)) {
                    app->speaker_acquired = true;
                }
            }
        }

        if(app->speaker_acquired && (now < app->alert_start_tick + 240)) {
            static const float ALERT_LOW[3] = {587.0f, 880.0f, 1175.0f};  /* D5, A5, D6 */
            static const float ALERT_HIGH[3] = {784.0f, 1175.0f, 1568.0f}; /* G5, D6, G6 */
            uint32_t elapsed = now - app->alert_start_tick;
            float freq = (elapsed < 120) ? ALERT_LOW[app->alert_target] :
                                          ALERT_HIGH[app->alert_target];
            furi_hal_speaker_start(freq, 0.85f);
        } else if(app->speaker_acquired) {
            furi_hal_speaker_stop();
            furi_hal_speaker_release();
            app->speaker_acquired = false;
        }
    } else {
        /* Perimeter Off */
        app->model.perimeter_alarm_active = false;
        if(app->speaker_acquired) {
            furi_hal_speaker_stop();
            furi_hal_speaker_release();
            app->speaker_acquired = false;
        }
    }
}

static void handle_menu_action(LD2450RadarApp* app) {
    furi_mutex_acquire(app->model_mutex, FuriWaitForever);
    LD2450Submenu submenu = app->model.menu_submenu;
    int8_t selected = 0;
    switch(submenu) {
    case LD2450MenuUart:
        selected = app->model.menu_uart_idx;
        break;
    case LD2450MenuMisc:
        selected = app->model.menu_misc_idx;
        break;
    case LD2450MenuRoot:
    default:
        selected = app->model.menu_root_idx;
        break;
    }
    furi_mutex_release(app->model_mutex);

    if(submenu == LD2450MenuRoot) {
        switch(selected) {
        case 0: /* Toggle Units */
            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
            app->model.units = (app->model.units == LD2450UnitsMetric) ? LD2450UnitsImperial :
                                                                         LD2450UnitsMetric;
            set_toast(
                &app->model,
                (app->model.units == LD2450UnitsMetric) ? "Units: Metric (m)" : "Units: Imperial (ft)",
                1500);
            furi_mutex_release(app->model_mutex);
            break;

        case 1: /* Cycle Baud in menu */ {
            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
            if(app->model.baud_mode == LD2450BaudModeAuto) {
                app->model.baud_mode = LD2450BaudModeFixed;
                ld2450_radar_uart_set_baudrate(app->uart, 256000);
                app->model.current_baud = 256000;
                set_toast(&app->model, "Baud: 256000 Fixed", 1500);
            } else {
                uint32_t cur = app->model.current_baud;
                int idx = 0;
                for(int i = 0; i < BAUDRATE_COUNT; i++) {
                    if(SUPPORTED_BAUDRATES[i] == cur) {
                        idx = i;
                        break;
                    }
                }
                idx++;
                if(idx >= BAUDRATE_COUNT) {
                    app->model.baud_mode = LD2450BaudModeAuto;
                    set_toast(&app->model, "Baud: Auto-scan", 1500);
                } else {
                    uint32_t next = SUPPORTED_BAUDRATES[idx];
                    ld2450_radar_uart_set_baudrate(app->uart, next);
                    app->model.current_baud = next;
                    char msg[32];
                    snprintf(msg, sizeof(msg), "Baud: %lu Fixed", next);
                    set_toast(&app->model, msg, 1500);
                }
            }
            furi_mutex_release(app->model_mutex);
            break;
        }

        case 2: /* Cycle Zoom Range */
            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
            app->model.range_idx = (app->model.range_idx + 1) % RANGE_COUNT;
            set_toast(&app->model, "Range updated", 1000);
            furi_mutex_release(app->model_mutex);
            break;

        case 3: /* Cycle Perimeter Security Mode */ {
            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
            app->model.perimeter_mode = (app->model.perimeter_mode + 1) % 4;
            const char* toast_txt = "Perimeter: OFF";
            if(app->model.perimeter_mode == LD2450PerimeterNotification) {
                toast_txt = "Perimeter: Notification";
            } else if(app->model.perimeter_mode == LD2450PerimeterAlert) {
                toast_txt = "Perimeter: Alert";
            } else if(app->model.perimeter_mode == LD2450PerimeterAlarm) {
                toast_txt = "Perimeter: Alarm";
            }
            set_toast(&app->model, toast_txt, 1500);
            furi_mutex_release(app->model_mutex);
            break;
        }

        case 4: /* Enter UART Commands Submenu */
            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
            app->model.menu_submenu = LD2450MenuUart;
            app->model.menu_selected_tick = furi_get_tick();
            furi_mutex_release(app->model_mutex);
            break;

        case 5: /* Enter Miscellaneous Submenu */
            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
            app->model.menu_submenu = LD2450MenuMisc;
            app->model.menu_selected_tick = furi_get_tick();
            furi_mutex_release(app->model_mutex);
            break;
        }
    } else if(submenu == LD2450MenuUart) {
        switch(selected) {
        case 0: /* Force 256k (Auto-Sweep) */ {
            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
            set_toast(&app->model, "Sweeping bauds to 256k...", 2500);
            furi_mutex_release(app->model_mutex);

            bool ok = ld2450_radar_uart_force_sensor_baud(app->uart, 256000);

            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
            app->model.current_baud = 256000;
            app->model.baud_mode = LD2450BaudModeAuto;
            app->model.last_packet_time = furi_get_tick();
            app->model.packet_count = 0;
            set_toast(&app->model, ok ? "Done! Set to 256k" : "Sweep failed", 2000);
            furi_mutex_release(app->model_mutex);
            break;
        }

        case 1: /* Factory Reset */ {
            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
            set_toast(&app->model, "Factory resetting sensor...", 2500);
            furi_mutex_release(app->model_mutex);

            bool ok = ld2450_radar_uart_send_factory_reset(app->uart);

            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
            app->model.current_baud = 256000;
            app->model.baud_mode = LD2450BaudModeAuto;
            app->model.last_packet_time = furi_get_tick();
            app->model.packet_count = 0;
            set_toast(&app->model, ok ? "Sensor reset to 256k!" : "Reset failed", 2000);
            furi_mutex_release(app->model_mutex);
            break;
        }

        case 2: /* Send Wake / End BLE Config */ {
            bool ok = ld2450_radar_uart_send_end_config(app->uart);
            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
            set_toast(&app->model, ok ? "TX: Wake / End Config sent" : "TX failed", 2000);
            furi_mutex_release(app->model_mutex);
            break;
        }

        case 3: /* Send Multi-target command */ {
            bool ok = ld2450_radar_uart_send_mode_multi(app->uart);
            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
            set_toast(&app->model, ok ? "TX: Multi-Target sent!" : "TX failed", 2000);
            furi_mutex_release(app->model_mutex);
            break;
        }

        case 4: /* Send Single-target command */ {
            bool ok = ld2450_radar_uart_send_mode_single(app->uart);
            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
            set_toast(&app->model, ok ? "TX: Single-Target sent!" : "TX failed", 2000);
            furi_mutex_release(app->model_mutex);
            break;
        }
        }
    } else if(submenu == LD2450MenuMisc) {
        switch(selected) {
        case 0: /* Toggle Debug Screen */
            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
            app->model.debug_enabled = !app->model.debug_enabled;
            set_toast(
                &app->model,
                app->model.debug_enabled ? "Debug Screen: ON" : "Debug Screen: OFF",
                1500);
            furi_mutex_release(app->model_mutex);
            break;

        case 1: /* Wiring Guide */
            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
            app->model.screen = LD2450ScreenPinout;
            furi_mutex_release(app->model_mutex);
            break;

        case 2: /* About */
            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
            app->model.screen = LD2450ScreenAbout;
            furi_mutex_release(app->model_mutex);
            break;
        }
    }
}

int32_t ld2450_radar_app(void* p) {
    UNUSED(p);

    LD2450RadarApp* app = malloc(sizeof(LD2450RadarApp));
    furi_check(app);
    memset(app, 0, sizeof(LD2450RadarApp));

    furi_delay_ms(100);
    app->model.last_packet_time = furi_get_tick();
    app->model.screen = LD2450ScreenRadar;
    app->model.range_idx = 2; /* 6000mm (6m) default */
    app->model.units = LD2450UnitsMetric;
    app->model.debug_enabled = false;
    app->model.baud_mode = LD2450BaudModeAuto;
    app->model.current_baud = 256000;
    app->model.menu_submenu = LD2450MenuRoot;
    app->model.menu_root_idx = 0;
    app->model.menu_uart_idx = 0;
    app->model.menu_misc_idx = 0;
    app->model.menu_selected_tick = furi_get_tick();

    app->event_queue = furi_message_queue_alloc(16, sizeof(LD2450RadarEvent));
    app->model_mutex = furi_mutex_alloc(FuriMutexTypeNormal);

    app->view_port = view_port_alloc();
    view_port_draw_callback_set(app->view_port, ld2450_radar_draw_callback, app);
    view_port_input_callback_set(app->view_port, ld2450_radar_input_callback, app);

    app->gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(app->gui, app->view_port, GuiLayerFullscreen);

    app->uart = ld2450_radar_uart_alloc();
    ld2450_radar_uart_set_handle_rx_data_cb(app->uart, ld2450_radar_uart_callback, app);

    bool init_success = ld2450_radar_uart_start(app->uart);

    if(!init_success) {
        furi_mutex_acquire(app->model_mutex, FuriWaitForever);
        app->model.uart_init_failed = true;
        furi_mutex_release(app->model_mutex);
    } else {
        /* Wake sensor up in case it was left in BLE configuration mode */
        furi_delay_ms(50);
        ld2450_radar_uart_send_end_config(app->uart);
    }

    FuriTimer* timer = furi_timer_alloc(ld2450_radar_tick_callback, FuriTimerTypePeriodic, app);
    furi_timer_start(timer, 50);

    LD2450RadarEvent event;
    bool running = true;

    while(running) {
        if(furi_message_queue_get(app->event_queue, &event, 100) == FuriStatusOk) {
            if(event.type == LD2450RadarEventInput) {
                if(event.input.key == InputKeyBack && event.input.type == InputTypeShort) {
                    furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                    if(app->model.screen == LD2450ScreenMenu && app->model.menu_submenu != LD2450MenuRoot) {
                        app->model.menu_submenu = LD2450MenuRoot;
                        app->model.menu_selected_tick = furi_get_tick();
                        furi_mutex_release(app->model_mutex);
                        view_port_update(app->view_port);
                    } else if(app->model.screen == LD2450ScreenPinout || app->model.screen == LD2450ScreenAbout) {
                        app->model.screen = LD2450ScreenMenu;
                        app->model.menu_submenu = LD2450MenuMisc;
                        app->model.menu_selected_tick = furi_get_tick();
                        furi_mutex_release(app->model_mutex);
                        view_port_update(app->view_port);
                    } else if(app->model.screen != LD2450ScreenRadar) {
                        app->model.screen = LD2450ScreenRadar;
                        furi_mutex_release(app->model_mutex);
                        view_port_update(app->view_port);
                    } else {
                        furi_mutex_release(app->model_mutex);
                        running = false;
                    }
                } else if(event.input.type == InputTypeShort || event.input.type == InputTypeRepeat) {
                    furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                    LD2450Screen cur_screen = app->model.screen;
                    furi_mutex_release(app->model_mutex);

                    if(cur_screen == LD2450ScreenRadar) {
                        if(event.input.key == InputKeyUp) {
                            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                            if(app->model.range_idx > 0) app->model.range_idx--;
                            furi_mutex_release(app->model_mutex);
                        } else if(event.input.key == InputKeyDown) {
                            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                            if(app->model.range_idx < RANGE_COUNT - 1) app->model.range_idx++;
                            furi_mutex_release(app->model_mutex);
                        } else if(event.input.key == InputKeyOk) {
                            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                            app->model.units = (app->model.units == LD2450UnitsMetric) ? LD2450UnitsImperial :
                                                                                         LD2450UnitsMetric;
                            set_toast(
                                &app->model,
                                (app->model.units == LD2450UnitsMetric) ? "Units: Metric (m)" :
                                                                         "Units: Imperial (ft)",
                                1200);
                            furi_mutex_release(app->model_mutex);
                        } else if(event.input.key == InputKeyRight) {
                            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                            app->model.screen = LD2450ScreenText;
                            furi_mutex_release(app->model_mutex);
                        } else if(event.input.key == InputKeyLeft) {
                            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                            app->model.screen = LD2450ScreenMenu;
                            furi_mutex_release(app->model_mutex);
                        }
                    } else if(cur_screen == LD2450ScreenText) {
                        if(event.input.key == InputKeyUp) {
                            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                            if(app->model.range_idx > 0) app->model.range_idx--;
                            furi_mutex_release(app->model_mutex);
                        } else if(event.input.key == InputKeyDown) {
                            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                            if(app->model.range_idx < RANGE_COUNT - 1) app->model.range_idx++;
                            furi_mutex_release(app->model_mutex);
                        } else if(event.input.key == InputKeyOk) {
                            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                            app->model.units = (app->model.units == LD2450UnitsMetric) ? LD2450UnitsImperial :
                                                                                         LD2450UnitsMetric;
                            set_toast(
                                &app->model,
                                (app->model.units == LD2450UnitsMetric) ? "Units: Metric (m)" :
                                                                         "Units: Imperial (ft)",
                                1200);
                            furi_mutex_release(app->model_mutex);
                        } else if(event.input.key == InputKeyRight) {
                            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                            if(app->model.debug_enabled) {
                                app->model.screen = LD2450ScreenDebug;
                            } else {
                                app->model.screen = LD2450ScreenMenu;
                            }
                            furi_mutex_release(app->model_mutex);
                        } else if(event.input.key == InputKeyLeft) {
                            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                            app->model.screen = LD2450ScreenRadar;
                            furi_mutex_release(app->model_mutex);
                        }
                    } else if(cur_screen == LD2450ScreenDebug) {
                        if(event.input.key == InputKeyUp) {
                            cycle_baud(app, -1);
                        } else if(event.input.key == InputKeyDown) {
                            cycle_baud(app, 1);
                        } else if(event.input.key == InputKeyOk) {
                            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                            set_toast(&app->model, "Forcing sensor to 256k...", 2500);
                            furi_mutex_release(app->model_mutex);

                            bool ok = ld2450_radar_uart_force_sensor_baud(app->uart, 256000);

                            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                            app->model.current_baud = 256000;
                            app->model.baud_mode = LD2450BaudModeAuto;
                            app->model.last_packet_time = furi_get_tick();
                            app->model.packet_count = 0;
                            set_toast(&app->model, ok ? "Done! Set to 256k" : "Force failed", 2000);
                            furi_mutex_release(app->model_mutex);
                        } else if(event.input.key == InputKeyRight) {
                            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                            app->model.screen = LD2450ScreenMenu;
                            furi_mutex_release(app->model_mutex);
                        } else if(event.input.key == InputKeyLeft) {
                            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                            app->model.screen = LD2450ScreenText;
                            furi_mutex_release(app->model_mutex);
                        }
                    } else if(cur_screen == LD2450ScreenMenu) {
                        if(event.input.key == InputKeyUp) {
                            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                            menu_change_selection(&app->model, -1);
                            furi_mutex_release(app->model_mutex);
                        } else if(event.input.key == InputKeyDown) {
                            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                            menu_change_selection(&app->model, 1);
                            furi_mutex_release(app->model_mutex);
                        } else if(event.input.key == InputKeyOk) {
                            handle_menu_action(app);
                        } else if(event.input.key == InputKeyLeft) {
                            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                            if(app->model.menu_submenu != LD2450MenuRoot) {
                                app->model.menu_submenu = LD2450MenuRoot;
                                app->model.menu_selected_tick = furi_get_tick();
                            } else {
                                if(app->model.debug_enabled) {
                                    app->model.screen = LD2450ScreenDebug;
                                } else {
                                    app->model.screen = LD2450ScreenText;
                                }
                            }
                            furi_mutex_release(app->model_mutex);
                        } else if(event.input.key == InputKeyRight) {
                            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                            if(app->model.menu_submenu == LD2450MenuRoot) {
                                if(app->model.menu_root_idx == 4) {
                                    app->model.menu_submenu = LD2450MenuUart;
                                    app->model.menu_selected_tick = furi_get_tick();
                                } else if(app->model.menu_root_idx == 5) {
                                    app->model.menu_submenu = LD2450MenuMisc;
                                    app->model.menu_selected_tick = furi_get_tick();
                                } else {
                                    app->model.screen = LD2450ScreenRadar;
                                }
                            } else if(app->model.menu_submenu == LD2450MenuMisc) {
                                if(app->model.menu_misc_idx == 1) {
                                    app->model.screen = LD2450ScreenPinout;
                                } else if(app->model.menu_misc_idx == 2) {
                                    app->model.screen = LD2450ScreenAbout;
                                }
                            }
                            furi_mutex_release(app->model_mutex);
                        }
                    } else if(cur_screen == LD2450ScreenPinout || cur_screen == LD2450ScreenAbout) {
                        if(event.input.key == InputKeyOk || event.input.key == InputKeyLeft || event.input.key == InputKeyRight) {
                            furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                            app->model.screen = LD2450ScreenMenu;
                            app->model.menu_submenu = LD2450MenuMisc;
                            app->model.menu_selected_tick = furi_get_tick();
                            furi_mutex_release(app->model_mutex);
                        }
                    }
                    view_port_update(app->view_port);
                }
            } else if(event.type == LD2450RadarEventDataUpdate) {
                furi_mutex_acquire(app->model_mutex, FuriWaitForever);
                uint32_t current_rx = ld2450_radar_uart_get_rx_bytes(app->uart);
                app->model.rx_bytes = current_rx;
                app->model.current_baud = ld2450_radar_uart_get_baudrate(app->uart);
                ld2450_radar_uart_get_last_raw(
                    app->uart, app->model.last_raw_bytes, sizeof(app->model.last_raw_bytes));

                /* Auto-Baud Detection:
                 * ONLY if in auto mode and no valid packets received yet (packet_count == 0)
                 * and bytes are flowing for > 1500 ms, switch to next supported baud rate.
                 * As soon as packets arrive, packet_count becomes > 0 and auto-scanning stops!
                 */
                uint32_t now = furi_get_tick();
                if(app->model.baud_mode == LD2450BaudModeAuto && app->model.packet_count == 0) {
                    if((now - app->model.last_packet_time > 1500) &&
                       (current_rx > app->model.last_rx_bytes_check + 15)) {
                        uint32_t cur = ld2450_radar_uart_get_baudrate(app->uart);
                        int idx = 0;
                        for(int i = 0; i < BAUDRATE_COUNT; i++) {
                            if(SUPPORTED_BAUDRATES[i] == cur) {
                                idx = i;
                                break;
                            }
                        }
                        idx = (idx + 1) % BAUDRATE_COUNT;
                        uint32_t next = SUPPORTED_BAUDRATES[idx];
                        ld2450_radar_uart_set_baudrate(app->uart, next);
                        app->model.current_baud = next;
                        app->model.last_rx_bytes_check = current_rx;
                        app->model.last_packet_time = now;
                        char msg[32];
                        snprintf(msg, sizeof(msg), "Auto-detect: %lu...", next);
                        set_toast(&app->model, msg, 1200);
                    }
                }

                ld2450_perimeter_audio_update(app);

                furi_mutex_release(app->model_mutex);
                view_port_update(app->view_port);
            }
        }
    }

    furi_timer_stop(timer);
    furi_timer_free(timer);

    if(app->speaker_acquired) {
        furi_hal_speaker_stop();
        furi_hal_speaker_release();
        app->speaker_acquired = false;
    }

    ld2450_radar_uart_stop(app->uart);
    ld2450_radar_uart_free(app->uart);

    gui_remove_view_port(app->gui, app->view_port);
    view_port_free(app->view_port);
    furi_record_close(RECORD_GUI);

    furi_mutex_free(app->model_mutex);
    furi_message_queue_free(app->event_queue);
    free(app);

    return 0;
}