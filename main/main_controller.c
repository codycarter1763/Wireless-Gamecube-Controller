/**
 * main_controller.c - board that holds the real GameCube controller.
 *
 * Built when CONFIG_GC_ROLE_CONTROLLER=y.
 *
 * Core 1: poll task. Polls the pad every 1 ms and sends over ESP-NOW only
 *         when something changed:
 *           - a button change is sent immediately
 *           - an analog change (beyond the deadband) is sent at most every
 *             CONFIG_GC_ANALOG_MIN_INTERVAL_MS
 *           - otherwise a keepalive goes out every CONFIG_GC_KEEPALIVE_MS
 * Core 0: Wi-Fi, plus a print task that logs pad state and link stats.
 */
#include "sdkconfig.h"
#if CONFIG_GC_ROLE_CONTROLLER

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_cpu.h"
#include "esp_log.h"
#include "gc_protocol.h"
#include "gc_link.h"
#include "gc_debug.h"
#include "joybus.h"

#define POLL_PERIOD_MS         1
#define ORIGIN_RESEND_US       (500 * 1000)
#define KEEPALIVE_US           (CONFIG_GC_KEEPALIVE_MS * 1000)
#define ANALOG_MIN_INTERVAL_US (CONFIG_GC_ANALOG_MIN_INTERVAL_MS * 1000)

// Allow at least two lost keepalives before the receiver goes stale.
_Static_assert(KEEPALIVE_US * 3 <= GC_LINK_STALE_US,
               "keepalive too slow for the receiver's stale timeout");

#define BYTE0_BUTTONS  0x1F   // Start Y X B A
#define BYTE1_BUTTONS  0x7F   // L R Z D-pad

static const char *TAG = "gc_ctrl";

// Shared with the print task
static portMUX_TYPE      s_lock = portMUX_INITIALIZER_UNLOCKED;
static GC_Payload        s_last_pad;
static volatile uint32_t s_ok, s_bad, s_sent, s_queue_fail;

/* ------------------------------------------------------------------ */
/* Change detection                                                    */
/* ------------------------------------------------------------------ */

static inline int absdiff(uint8_t a, uint8_t b)
{
    return a > b ? a - b : b - a;
}

static bool buttons_changed(const GC_Payload *a, const GC_Payload *b)
{
    return ((a->byte0 ^ b->byte0) & BYTE0_BUTTONS) ||
           ((a->byte1 ^ b->byte1) & BYTE1_BUTTONS);
}

static bool analog_changed(const GC_Payload *a, const GC_Payload *b)
{
    const int db = CONFIG_GC_ANALOG_DEADBAND;
    return absdiff(a->joy_X_val,         b->joy_X_val)         > db ||
           absdiff(a->joy_Y_val,         b->joy_Y_val)         > db ||
           absdiff(a->C_stick_X_val,     b->C_stick_X_val)     > db ||
           absdiff(a->C_stick_Y_val,     b->C_stick_Y_val)     > db ||
           absdiff(a->left_trigger_val,  b->left_trigger_val)  > db ||
           absdiff(a->right_trigger_val, b->right_trigger_val) > db;
}

/* ------------------------------------------------------------------ */
/* Debug                                                               */
/* ------------------------------------------------------------------ */

/* Print what the data line does after an ID command (no logic analyzer needed). */
static void debug_dump(void)
{
    // Check the timing basis: CPU cycles measured against esp_timer over 10 ms.
    int64_t  t0 = esp_timer_get_time();
    uint32_t c0 = esp_cpu_get_cycle_count();
    while (esp_timer_get_time() - t0 < 10000) { }
    uint32_t c1 = esp_cpu_get_cycle_count();
    int64_t  t1 = esp_timer_get_time();
    uint32_t mhz = (uint32_t)((c1 - c0) / (uint32_t)(t1 - t0));

    bool idle_high;
    uint16_t e[64];
    int n = joybus_host_debug_capture(&idle_high, e, 64);

    ESP_LOGW(TAG, "  measured CPU %lu MHz, timing uses %lu cycles/us",
             (unsigned long)mhz, (unsigned long)joybus_cycles_per_us());
    ESP_LOGW(TAG, "  line idle: %s, edges after ID cmd: %d",
             idle_high ? "HIGH (ok)" : "LOW (line held low!)", n);

    char buf[256];
    int len = 0;
    for (int i = 0; i + 1 < n && len < (int)sizeof(buf) - 8; i += 2) {
        unsigned w = e[i + 1] - e[i];
        len += snprintf(buf + len, sizeof(buf) - len, "%u.%u ", w / 10, w % 10);
    }
    ESP_LOGW(TAG, "  low pulse widths (us): %s", n ? buf : "(none)");
}

/* ------------------------------------------------------------------ */
/* Tasks                                                               */
/* ------------------------------------------------------------------ */

static void poll_task(void *arg)
{
    (void)arg;

    // 1. Detect the pad and read its ID. Compare this with GC_ID_BYTE0..2.
    uint8_t id[3];
    int tries = 0;
    while (joybus_host_read_id(id) != ESP_OK) {
        if (tries++ % 20 == 0) {
            ESP_LOGW(TAG, "No controller detected, retrying...");
            debug_dump();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "Controller ID: %02X %02X %02X  (header expects %02X %02X %02X)",
             id[0], id[1], id[2], GC_ID_BYTE0, GC_ID_BYTE1, GC_ID_BYTE2);

    // 2. Read the origin (neutral stick and trigger values).
    vTaskDelay(pdMS_TO_TICKS(1));
    GC_Origin origin;
    while (joybus_host_read_origin(&origin) != ESP_OK) {
        ESP_LOGW(TAG, "Origin read failed, retrying...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    char line[96];
    gc_format_payload(&origin.state, line, sizeof(line));
    ESP_LOGI(TAG, "Origin: %s", line);
    gc_link_send_origin(&origin);
    int64_t last_origin_us = esp_timer_get_time();

    // 3. Poll loop with send-on-change.
    GC_Payload last_sent = {0};
    bool       have_sent = false;
    int64_t    last_send_us = 0;

    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        uint32_t cmd = GC_POLL | (gc_link_get_rumble() ? GC_RUMBLE_BIT : 0);

        GC_Payload pad;
        if (joybus_host_poll(cmd, &pad) == ESP_OK && gc_payload_valid(&pad)) {
            s_ok++;
            portENTER_CRITICAL(&s_lock);
            s_last_pad = pad;
            portEXIT_CRITICAL(&s_lock);

            int64_t now   = esp_timer_get_time();
            int64_t since = now - last_send_us;


            bool send = !have_sent
                     || buttons_changed(&pad, &last_sent)                                   // immediate
                     || (analog_changed(&pad, &last_sent) && since >= ANALOG_MIN_INTERVAL_US)
                     || since >= KEEPALIVE_US;

            if (send) {
                if (gc_link_send_input(&pad) == ESP_OK) {
                    last_sent    = pad;
                    have_sent    = true;
                    last_send_us = now;
                    s_sent++;
                } else {
                    s_queue_fail++;   // TX queue full; retried next poll
                }
            }
        } else {
            s_bad++;
        }

        int64_t now = esp_timer_get_time();
        if (now - last_origin_us > ORIGIN_RESEND_US) {
            gc_link_send_origin(&origin);
            last_origin_us = now;
        }

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}

static void print_task(void *arg)
{
    (void)arg;
    char line[96];
    uint32_t prev_sent = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        GC_Payload pad;
        portENTER_CRITICAL(&s_lock);
        pad = s_last_pad;
        portEXIT_CRITICAL(&s_lock);

        uint32_t acked, not_acked;
        gc_link_tx_stats(&acked, &not_acked);
        uint32_t sent = s_sent;

        gc_format_payload(&pad, line, sizeof(line));
        ESP_LOGI(TAG, "%s | rumble %u", line, gc_link_get_rumble());
        ESP_LOGI(TAG, "polls ok %lu bad %lu | sent %lu/s | acked %lu noack %lu qfail %lu",
                 (unsigned long)s_ok, (unsigned long)s_bad,
                 (unsigned long)(sent - prev_sent),
                 (unsigned long)acked, (unsigned long)not_acked,
                 (unsigned long)s_queue_fail);
        prev_sent = sent;
    }
}

void app_main(void)
{
    uint8_t peer[6] = {0};
    if (!gc_link_parse_mac(CONFIG_GC_PEER_MAC, peer)) {
        ESP_LOGE(TAG, "Bad CONFIG_GC_PEER_MAC '%s'", CONFIG_GC_PEER_MAC);
    }
    if (gc_link_init(GC_ROLE_CONTROLLER, peer) != ESP_OK) {
        ESP_LOGW(TAG, "Running without wireless link (pad reading only)");
    }

    ESP_ERROR_CHECK(joybus_host_init(CONFIG_GC_DATA_GPIO));

    xTaskCreatePinnedToCore(poll_task,  "gc_poll",  4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(print_task, "gc_print", 4096, NULL, 2,  NULL, 0);
}

#endif // CONFIG_GC_ROLE_CONTROLLER