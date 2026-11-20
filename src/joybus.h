#pragma once
/**
 * joybus.h - Joybus bit layer for nRF52840 (bit-banged, cycle-counted).
 * Port of the ESP32 version in Wireless-Gamecube-Controller.
 *
 * Electrical: one open-drain data line, idle high.
 *   Controller board: needs an external 1k pull-up to 3.3V.
 *   Receiver board:   the console provides the pull-up.
 *
 * Timing: 4 us per bit. A 1 is 1 us low + 3 us high; a 0 is 3 us low + 1 us high.
 *   Console stop bit = 1 us low, controller stop bit = 2 us low.
 *
 * Transfers run with all interrupts masked (PRIMASK) (~100 us for the command plus up to
 * ~330 us for the reply). Call them from thread context, and not while an ESB
 * transmission is in flight: the controller loop polls the pad first and only
 * then hands the result to the radio, so the two never overlap.
 *
 * The data pin must be on port 0 (P0.00 - P0.31).
 */
#include <stdbool.h>
#include <stdint.h>
#include "gc_protocol.h"

/* ---------- Host mode (controller board: nRF52 talks to the real pad) ---------- */

/* All return 0 on success, -ETIMEDOUT if the pad didn't answer completely. */
int joybus_host_init(uint8_t p0_pin);
int joybus_host_read_id(uint8_t id[3]);                 // 0x00 -> 3 bytes
int joybus_host_read_origin(GC_Origin *out);            // 0x41 -> 10 bytes
int joybus_host_poll(uint32_t cmd24, GC_Payload *out);  // 0x40.. -> 8 bytes

/* ---------- Target mode (receiver board: nRF52 pretends to be the pad) ---------- */
/*
 * See joybus_target.c for how this works. In short: the command is received
 * in a zero-latency ISR (interrupts held off ~100 us max), and the reply is
 * played by PWM1 + EasyDMA so the CPU and radio stay free.
 * Uses: PWM1, TIMER3, EGU5/SWI5, one PPI channel, one GPIOTE channel.
 */

struct joybus_target_stats {
	uint32_t polls;     /* 0x40 answered */
	uint32_t origins;   /* 0x41 / 0x42 answered */
	uint32_t ids;       /* 0x00 / 0xFF answered */
	uint32_t errors;    /* edge timeout mid-command */
	uint32_t unknown;   /* unsupported command byte */
	uint32_t late;      /* ISR started too late to read the first bit */
};

int joybus_target_init(uint8_t p0_pin);

/** Re-encode the poll and origin replies. Safe from thread or ISR context
 *  (not from a zero-latency ISR). */
void joybus_target_set_state(const GC_Payload *in, const GC_Origin *origin);

/** Rumble bit from the console's most recent poll. */
uint8_t joybus_target_rumble(void);

void joybus_target_get_stats(struct joybus_target_stats *out, bool reset);
