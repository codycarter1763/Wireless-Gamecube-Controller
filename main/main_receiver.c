/**
 * main_receiver.c - board plugged into the console.
 *
 * Built when CONFIG_GC_ROLE_RECEIVER=y.
 *
 * Core 1: Joybus target task. Answers console commands from cached state:
 *           0x00 / 0xFF -> 3-byte ID
 *           0x40        -> 8-byte poll reply (also grabs the rumble bit)
 *           0x41 / 0x42 -> 10-byte origin reply
 * Core 0: Wi-Fi, a rumble back-channel task, and a once-a-second stats print.
 */
#include "sdkconfig.h"
#if CONFIG_GC_ROLE_RECEIVER

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "gc_protocol.h"
#include "gc_link.h"
#include "gc_debug.h"
#include "joybus.h"

#define RUMBLE_KEEPALIVE_MS 100
#define RUMBLE_RETRY_MS         5
#define RUMBLE_RETRY_WINDOW_MS  1000

static const char *TAG = "gc_rx";

static volatile uint8_t  s_console_rumble;
static volatile bool     s_need_origin = true;   // until console sends 0x41/0x42

// Command counters for debugging
static volatile uint32_t s_n_id, s_n_poll, s_n_origin, s_n_stale;

// Used until the controller board's real origin arrives. DRAM_ATTR keeps it
// out of flash, so reading it never waits on a cache miss.
static DRAM_ATTR const GC_Origin DEFAULT_ORIGIN = {
    .state = {
        .byte0 = 0x00,
        .byte1 = GC_BYTE1_ALWAYS_ONE,
        .joy_X_val = 128, .joy_Y_val = 128,
        .C_stick_X_val = 128, .C_stick_Y_val = 128,
        .left_trigger_val = 0, .right_trigger_val = 0,
    },
    .reserved = { 0, 0 },
};

static void IRAM_ATTR get_origin(GC_Origin *o)
{
    if (!gc_link_get_origin(o)) *o = DEFAULT_ORIGIN;
}

/* Runs on core 1 with interrupts disabled. Keep it fast and simple. */
static int IRAM_ATTR handle_console_cmd(const uint8_t *cmd, int cmd_len, uint8_t *reply)
{
    const uint8_t c = cmd[0];

    if (c == GC_CMD_ID || c == GC_CMD_RESET) {
        s_n_id++;
        reply[0] = GC_ID_BYTE0;
        reply[1] = GC_ID_BYTE1;
        reply[2] = GC_ID_BYTE2;
        return 3;
    }

    if (c == GC_CMD_POLL && cmd_len >= 3) {
        s_n_poll++;
        // cmd[1] = analog mode. Only mode 3 (the standard 8-byte layout)
        // is handled here, which is what most games request.
        s_console_rumble = cmd[2] & GC_RUMBLE_BIT;

        GC_Payload p;
        if (!gc_link_get_input(&p)) {
            // Link lost or stale: neutral sticks, no buttons pressed.
            s_n_stale++;
            GC_Origin o;
            get_origin(&o);
            p = o.state;
            p.byte0 = 0x00;
            p.byte1 = GC_BYTE1_ALWAYS_ONE;
        }
        // This board manages the need-origin flag, not the real pad.
        p.byte0 = (uint8_t)((p.byte0 & ~GC_BYTE0_NEED_ORIGIN) |
                            (s_need_origin ? GC_BYTE0_NEED_ORIGIN : 0));
        p.byte1 |= GC_BYTE1_ALWAYS_ONE;

        memcpy(reply, &p, sizeof(p));
        return sizeof(p);
    }

    if (c == GC_CMD_ORIGIN || c == GC_CMD_RECALIBRATE) {
        s_n_origin++;
        GC_Origin o;
        get_origin(&o);
        o.state.byte0 &= (uint8_t)~GC_BYTE0_NEED_ORIGIN;
        s_need_origin = false;
        memcpy(reply, &o, sizeof(o));
        return sizeof(o);
    }

    return 0;
}

/*
 * Forward the console's rumble bit to the controller board.
 *
 * The controller's radio may be asleep between wake windows, so a packet
 * can go un-ACKed. Resend every RUMBLE_RETRY_MS until the controller ACKs
 * it, then fall back to a keepalive every RUMBLE_KEEPALIVE_MS. (The
 * controller drops rumble to 0 if it hears nothing for 500 ms.)
 */
static void rumble_task(void *arg)
{
    (void)arg;
    uint8_t last = 0xFF;
    TickType_t last_sent = 0;
    TickType_t unacked_since = 0;
    bool was_unacked = false;
    for (;;) {
        uint8_t r = s_console_rumble;
        TickType_t now = xTaskGetTickCount();
        bool changed   = (r != last);
        bool unacked   = !gc_link_last_send_acked();
        bool keepalive = (now - last_sent) >= pdMS_TO_TICKS(RUMBLE_KEEPALIVE_MS);

        // Retry fast for up to 1 s. If the controller board is off, fall back
        // to keepalives only.
        if (unacked && !was_unacked) unacked_since = now;
        was_unacked = unacked;
        bool retry = unacked && (now - unacked_since) < pdMS_TO_TICKS(RUMBLE_RETRY_WINDOW_MS);

        if (changed || retry || keepalive) {
            gc_link_send_rumble(r);
            last = r;
            last_sent = now;
        }
        vTaskDelay(pdMS_TO_TICKS(RUMBLE_RETRY_MS));
    }
}

static void stats_task(void *arg)
{
    (void)arg;
    uint32_t prev_poll = 0;
    char line[96];
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        GC_Payload p;
        bool fresh = gc_link_get_input(&p);
        GC_Origin o;
        bool have_origin = gc_link_get_origin(&o);
        gc_format_payload(&p, line, sizeof(line));

        uint32_t polls = s_n_poll;
        ESP_LOGI(TAG, "link %s origin %s | %s",
                 fresh ? "OK   " : "STALE", have_origin ? "yes" : "no ", line);
        ESP_LOGI(TAG, "console: id %lu origin %lu poll %lu (%lu/s) stale %lu err %lu | rumble %u",
                 (unsigned long)s_n_id, (unsigned long)s_n_origin,
                 (unsigned long)polls, (unsigned long)(polls - prev_poll),
                 (unsigned long)s_n_stale, (unsigned long)joybus_target_error_count(),
                 s_console_rumble);
        prev_poll = polls;
    }
}

void app_main(void)
{
    uint8_t peer[6] = {0};
    if (!gc_link_parse_mac(CONFIG_GC_PEER_MAC, peer)) {
        ESP_LOGE(TAG, "Bad CONFIG_GC_PEER_MAC '%s'", CONFIG_GC_PEER_MAC);
    }
    if (gc_link_init(GC_ROLE_RECEIVER, peer) != ESP_OK) {
        ESP_LOGW(TAG, "Running without wireless link (console will see neutral input)");
    }

    ESP_ERROR_CHECK(joybus_target_init(CONFIG_GC_DATA_GPIO, handle_console_cmd));

    xTaskCreatePinnedToCore(rumble_task, "gc_rumble", 3072, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(stats_task,  "gc_stats",  4096, NULL, 2, NULL, 0);
}

#endif // CONFIG_GC_ROLE_RECEIVER