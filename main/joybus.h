#pragma once
/**
 * joybus.h - Joybus bit layer (bit-banged, cycle-counted).
 *
 * Electrical: one open-drain data line, idle high.
 *   Controller board: needs an external 1k pull-up to 3.3V.
 *   Receiver board:   the console provides the pull-up.
 *
 * Timing: 4 us per bit. A 1 is 1 us low + 3 us high; a 0 is 3 us low + 1 us high.
 *   Console stop bit = 1 us low, controller stop bit = 2 us low.
 *
 * Transfers run with interrupts disabled on the calling core, so call the
 * host functions from a task pinned to core 1 (Wi-Fi lives on core 0).
 * The data GPIO must be 0-31 and must not be a strapping pin.
 */
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "gc_protocol.h"

/* ---------- Host mode (controller board: ESP32 talks to the real pad) ---------- */
esp_err_t joybus_host_init(int gpio);
esp_err_t joybus_host_read_id(uint8_t id[3]);                 // 0x00 -> 3 bytes
esp_err_t joybus_host_read_origin(GC_Origin *out);            // 0x41 -> 10 bytes
esp_err_t joybus_host_poll(uint32_t cmd24, GC_Payload *out);  // 0x40.. -> 8 bytes

/* ---------- Debug helpers ---------- */

/** CPU cycles per microsecond that the bit timing is based on. */
uint32_t joybus_cycles_per_us(void);

/**
 * Debug: record the line level, send 0x00 (ID), then log every edge for
 * ~300 us. edge_t[i] = time of edge i after our stop bit, in 0.1 us units.
 * The first edge is falling, so (edge_t[0], edge_t[1]) is the first low pulse.
 * Returns the number of edges captured.
 */
int joybus_host_debug_capture(bool *idle_high, uint16_t *edge_t, int max_edges);

/* ---------- Target mode (receiver board: ESP32 pretends to be the pad) ---------- */

/**
 * Called once a full console command has been received.
 * Fill `reply` (up to 16 bytes) and return its length, or 0 for no reply.
 * Runs with interrupts disabled: no blocking, no logging, no malloc.
 */
typedef int (*joybus_target_handler_t)(const uint8_t *cmd, int cmd_len, uint8_t *reply);

/** Starts the target task pinned to core 1. It never yields that core. */
esp_err_t joybus_target_init(int gpio, joybus_target_handler_t handler);

/** Count of malformed/partial/unknown console commands (for debugging). */
uint32_t joybus_target_error_count(void);