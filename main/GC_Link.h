#pragma once
/**
 * gc_link - ESP-NOW transport for the wireless GameCube controller.
 *
 * Shared by both boards:
 *   CONTROLLER side: reads the real pad, sends GC_TxPacket, receives rumble.
 *   RECEIVER side:   plugged into the console, receives GC_TxPacket,
 *                    sends GC_RxPacket (rumble) back.
 *
 * Getters are safe to call from an ISR (IRAM + critical-section protected),
 * so the Joybus target code can answer console polls straight from cache.
 */
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "gc_protocol.h"

typedef enum {
    GC_ROLE_CONTROLLER,
    GC_ROLE_RECEIVER,
} gc_role_t;

// Wi-Fi channel both boards use. Must match on both sides.
#define GC_LINK_CHANNEL   1

// Input older than this is treated as stale -> receiver reports neutral.
#define GC_LINK_STALE_US  (100 * 1000)

// Controller drops rumble to 0 if no rumble packet arrives for this long
// (e.g. receiver unplugged), so the motor can't get stuck on.
#define GC_LINK_RUMBLE_TIMEOUT_US  (500 * 1000)

/**
 * Bring up NVS, Wi-Fi (STA, no power save) and ESP-NOW, and register
 * peer_mac as the only accepted peer. Logs this board's MAC at startup,
 * so you can copy it into the other board's config.
 *
 * If peer_mac is all zeros, Wi-Fi still starts and the MAC is still logged,
 * but no peer is added and ESP_ERR_INVALID_ARG is returned. That way each
 * board can be tested on its own before pairing.
 */
esp_err_t gc_link_init(gc_role_t role, const uint8_t peer_mac[6]);

/** Parse "aa:bb:cc:dd:ee:ff". Returns false on bad format. */
bool gc_link_parse_mac(const char *s, uint8_t mac[6]);

/** Delivery stats from the send callback: ACKed by the peer vs not ACKed. */
void gc_link_tx_stats(uint32_t *acked, uint32_t *not_acked);

/** True if the most recently completed send was ACKed by the peer. */
bool gc_link_last_send_acked(void);

/* ---------- Controller side ---------- */
esp_err_t gc_link_send_input(const GC_Payload *p);
esp_err_t gc_link_send_origin(const GC_Origin *o);
uint8_t   gc_link_get_rumble(void);          // last rumble bit; 0 if timed out

/* ---------- Receiver side ---------- */
esp_err_t gc_link_send_rumble(uint8_t on);
bool      gc_link_get_input(GC_Payload *out);   // false if none/stale
bool      gc_link_get_origin(GC_Origin *out);   // false if none received yet