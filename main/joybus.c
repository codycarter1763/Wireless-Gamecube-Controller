/**
 * joybus.c - bit-banged Joybus for ESP32 / ESP32-S3 (dual core).
 *
 * Why bit-banging instead of RMT: in target mode the reply has to start a few
 * microseconds after the console's stop bit. A busy-polling loop on a
 * dedicated core, with interrupts off, hits that deterministically. RMT's
 * "receive done" callback plus re-arming TX does not.
 *
 * Pin handling: the pin is in open-drain mode, so writing 0 pulls the line
 * low and writing 1 releases it to the pull-up. The input still reads the
 * real line level, which lets us see the other side.
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_cpu.h"
#include "esp_rom_sys.h"
#include "esp_attr.h"
#include "soc/soc.h"
#include "soc/gpio_reg.h"
#include "joybus.h"

#define HOST_REPLY_TIMEOUT_US  100    // wait for the pad to start replying
#define BIT_TIMEOUT_US         10     // max time for one bit's edges
#define TARGET_WAIT_US         1000   // idle wait per critical section (target)
#define RESYNC_IDLE_US         20     // line must be high this long to resync
#define REPLY_DELAY_US         2      // gap after console stop bit before reply

#define HOST_STOP_LOW_US       1      // console-style stop bit
#define TARGET_STOP_LOW_US     2      // controller-style stop bit

static uint32_t s_mask;     // GPIO bit mask
static uint32_t s_cc_us;    // CPU cycles per microsecond
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static joybus_target_handler_t s_handler;
static volatile uint32_t s_target_errors;

#define LINE_LOW()   REG_WRITE(GPIO_OUT_W1TC_REG, s_mask)
#define LINE_REL()   REG_WRITE(GPIO_OUT_W1TS_REG, s_mask)
#define LINE_HIGH()  ((REG_READ(GPIO_IN_REG) & s_mask) != 0)

/* ------------------------------------------------------------------ */
/* Low-level timing helpers (all inlined into IRAM callers)           */
/* ------------------------------------------------------------------ */

FORCE_INLINE_ATTR uint32_t now_cc(void)
{
    return (uint32_t)esp_cpu_get_cycle_count();
}

FORCE_INLINE_ATTR void wait_until(uint32_t t)
{
    while ((int32_t)(now_cc() - t) < 0) { }
}

/* Wait for the line to reach `high`. False if `timeout_cc` passes since `start`. */
FORCE_INLINE_ATTR bool wait_level(bool high, uint32_t start, uint32_t timeout_cc)
{
    while (LINE_HIGH() != high) {
        if ((uint32_t)(now_cc() - start) > timeout_cc) return false;
    }
    return true;
}

/* ------------------------------------------------------------------ */
/* Bit-level TX / RX                                                   */
/* ------------------------------------------------------------------ */

static void IRAM_ATTR send_bytes(const uint8_t *buf, int n, uint32_t stop_low_us)
{
    const uint32_t us = s_cc_us;
    uint32_t t = now_cc();

    for (int i = 0; i < n; i++) {
        uint8_t v = buf[i];
        for (int b = 0; b < 8; b++, v <<= 1) {
            uint32_t low = (v & 0x80) ? us : 3 * us;
            LINE_LOW();  t += low;          wait_until(t);
            LINE_REL();  t += 4 * us - low; wait_until(t);
        }
    }
    // stop bit
    LINE_LOW(); t += stop_low_us * us; wait_until(t);
    LINE_REL();
}

/* Wait for a falling edge, sample 2 us later. Returns 0/1, or -1 on timeout. */
static int IRAM_ATTR read_bit(uint32_t timeout_cc)
{
    uint32_t start = now_cc();
    if (!wait_level(true,  start, timeout_cc)) return -1;   // end of previous low
    if (!wait_level(false, start, timeout_cc)) return -1;   // falling edge
    wait_until(now_cc() + 2 * s_cc_us);
    return LINE_HIGH() ? 1 : 0;
}

/**
 * Read n bytes, MSB first.
 * Returns n on success, -1 if the very first bit never started (idle),
 * -2 if it timed out partway through (a real error).
 */
static int IRAM_ATTR read_bytes(uint8_t *buf, int n, uint32_t first_timeout_us)
{
    for (int i = 0; i < n; i++) {
        uint8_t v = 0;
        for (int b = 0; b < 8; b++) {
            bool first = (i == 0 && b == 0);
            int bit = read_bit((first ? first_timeout_us : BIT_TIMEOUT_US) * s_cc_us);
            if (bit < 0) return first ? -1 : -2;
            v = (uint8_t)((v << 1) | bit);
        }
        buf[i] = v;
    }
    return n;
}

/* Consume the console's stop bit and wait for the line to be released. */
static bool IRAM_ATTR read_stop(void)
{
    uint32_t start = now_cc();
    uint32_t to = BIT_TIMEOUT_US * s_cc_us;
    return wait_level(true,  start, to)
        && wait_level(false, start, to)
        && wait_level(true,  start, to);
}

/* After an error, wait until the line has been idle-high for a while. */
static void IRAM_ATTR resync(void)
{
    uint32_t start = now_cc();
    uint32_t idle_start = start;
    while ((uint32_t)(now_cc() - start) < 1000 * s_cc_us) {
        if (!LINE_HIGH()) {
            idle_start = now_cc();
        } else if ((uint32_t)(now_cc() - idle_start) >= RESYNC_IDLE_US * s_cc_us) {
            return;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Pin setup                                                           */
/* ------------------------------------------------------------------ */

static esp_err_t pin_init(int gpio)
{
    if (gpio < 0 || gpio > 31) return ESP_ERR_INVALID_ARG;

    s_mask  = 1u << gpio;
    s_cc_us = esp_rom_get_cpu_ticks_per_us();

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << gpio,
        .mode         = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en   = GPIO_PULLUP_ENABLE,     // weak; real pull-up is external/console
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) return err;

    LINE_REL();
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Host mode                                                           */
/* ------------------------------------------------------------------ */

esp_err_t joybus_host_init(int gpio)
{
    return pin_init(gpio);
}

static esp_err_t host_xfer(const uint8_t *cmd, int cmd_len, uint8_t *reply, int reply_len)
{
    int n;
    portENTER_CRITICAL(&s_mux);
    send_bytes(cmd, cmd_len, HOST_STOP_LOW_US);
    n = read_bytes(reply, reply_len, HOST_REPLY_TIMEOUT_US);
    portEXIT_CRITICAL(&s_mux);
    return (n == reply_len) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t joybus_host_read_id(uint8_t id[3])
{
    const uint8_t cmd = GC_CMD_ID;
    return host_xfer(&cmd, 1, id, 3);
}

esp_err_t joybus_host_read_origin(GC_Origin *out)
{
    const uint8_t cmd = GC_CMD_ORIGIN;
    return host_xfer(&cmd, 1, (uint8_t *)out, sizeof(GC_Origin));
}

esp_err_t joybus_host_poll(uint32_t cmd24, GC_Payload *out)
{
    const uint8_t cmd[3] = {
        (uint8_t)(cmd24 >> 16), (uint8_t)(cmd24 >> 8), (uint8_t)cmd24
    };
    return host_xfer(cmd, 3, (uint8_t *)out, sizeof(GC_Payload));
}

/* ------------------------------------------------------------------ */
/* Debug helpers                                                       */
/* ------------------------------------------------------------------ */

uint32_t joybus_cycles_per_us(void)
{
    return s_cc_us;
}

int joybus_host_debug_capture(bool *idle_high, uint16_t *edge_t, int max_edges)
{
    const uint8_t cmd = GC_CMD_ID;
    int n = 0;

    portENTER_CRITICAL(&s_mux);
    *idle_high = LINE_HIGH();
    send_bytes(&cmd, 1, HOST_STOP_LOW_US);

    uint32_t t0 = now_cc();
    uint32_t window = 300 * s_cc_us;
    bool level = LINE_HIGH();
    while ((uint32_t)(now_cc() - t0) < window && n < max_edges) {
        bool l = LINE_HIGH();
        if (l != level) {
            level = l;
            edge_t[n++] = (uint16_t)(((now_cc() - t0) * 10u) / s_cc_us);
        }
    }
    portEXIT_CRITICAL(&s_mux);
    return n;
}

/* ------------------------------------------------------------------ */
/* Target mode                                                         */
/* ------------------------------------------------------------------ */

/* Command length in bytes, or -1 if unknown. (No switch: avoids a flash jump table.) */
FORCE_INLINE_ATTR int cmd_length(uint8_t c)
{
    if (c == GC_CMD_ID || c == GC_CMD_ORIGIN || c == GC_CMD_RESET) return 1;
    if (c == GC_CMD_POLL || c == GC_CMD_RECALIBRATE)              return 3;
    return -1;
}

static void IRAM_ATTR target_task(void *arg)
{
    (void)arg;
    uint8_t cmd[3];
    uint8_t reply[16];

    for (;;) {
        // Interrupts are off for at most ~1 ms of idle wait plus one transfer.
        // Leaving the critical section between waits lets pending interrupts
        // (tick, IPC) run.
        portENTER_CRITICAL(&s_mux);

        int r = read_bytes(cmd, 1, TARGET_WAIT_US);
        if (r == 1) {
            int len = cmd_length(cmd[0]);
            bool ok = (len > 0)
                   && (len == 1 || read_bytes(&cmd[1], len - 1, BIT_TIMEOUT_US) == len - 1)
                   && read_stop();
            if (ok) {
                int n = s_handler(cmd, len, reply);
                if (n > 0 && n <= (int)sizeof(reply)) {
                    wait_until(now_cc() + REPLY_DELAY_US * s_cc_us);
                    send_bytes(reply, n, TARGET_STOP_LOW_US);
                }
            } else {
                s_target_errors++;
                resync();
            }
        } else if (r == -2) {
            s_target_errors++;
            resync();
        }
        // r == -1: nothing arrived within TARGET_WAIT_US, which is normal

        portEXIT_CRITICAL(&s_mux);
    }
}

esp_err_t joybus_target_init(int gpio, joybus_target_handler_t handler)
{
    if (handler == NULL) return ESP_ERR_INVALID_ARG;
    esp_err_t err = pin_init(gpio);
    if (err != ESP_OK) return err;

    s_handler = handler;
    // One below max, so the IPC task (max priority) can still preempt.
    BaseType_t ok = xTaskCreatePinnedToCore(target_task, "joybus_tgt", 4096, NULL,
                                            configMAX_PRIORITIES - 2, NULL, 1);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

uint32_t joybus_target_error_count(void)
{
    return s_target_errors;
}