/**
 * gc_link.c - ESP-NOW transport, adapted from the ESP-IDF espnow example.
 *
 * Changes from the example, and why:
 *  - No broadcast/magic-number handshake: each board is given the other's
 *    MAC address, so the two boards are paired at compile time.
 *  - No malloc + queue in the receive callback: the callback copies the
 *    packet into a cached "latest state" under a spinlock. The Joybus side
 *    reads that cache, so a console poll never waits on the radio.
 *  - No CRC16: ESP-NOW frames are already checked by the 802.11 FCS.
 *    Length and source MAC are still checked.
 *
 * Power:
 *  - Receiver: power save off, so it hears every input packet immediately.
 *  - Controller: optional connectionless power save (CONFIG_GC_POWER_SAVE).
 *    The radio listens only inside wake windows; sending still works at
 *    any time.
 */
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "esp_idf_version.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "nvs_flash.h"
#include "gc_link.h"

static const char *TAG = "gc_link";

static gc_role_t    s_role;
static uint8_t      s_peer[ESP_NOW_ETH_ALEN];
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

// Receiver-side cache
static GC_Payload s_input;
static GC_Origin  s_origin;
static bool       s_have_input;
static bool       s_have_origin;
static int64_t    s_last_input_us;
static uint8_t    s_last_seq;

// Controller-side cache
static volatile uint8_t s_rumble;
static volatile int64_t s_last_rumble_rx_us;

// TX sequence counter (controller side)
static uint8_t s_tx_seq;

// Delivery stats (updated in the send callback)
static volatile uint32_t s_tx_acked;
static volatile uint32_t s_tx_not_acked;
static volatile bool     s_last_acked;

/* ------------------------------------------------------------------ */

static bool use_power_save(gc_role_t role)
{
#if CONFIG_GC_POWER_SAVE
    return role == GC_ROLE_CONTROLLER;
#else
    (void)role;
    return false;
#endif
}

static void wifi_init(gc_role_t role)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    // Receiver: never sleep. Controller: modem sleep if power save is on.
    ESP_ERROR_CHECK(esp_wifi_set_ps(use_power_save(role) ? WIFI_PS_MIN_MODEM : WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(GC_LINK_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

/* Called from the Wi-Fi task once a frame is ACKed by the peer or gives up. */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void send_cb(const esp_now_send_info_t *info, esp_now_send_status_t status)
#else
static void send_cb(const uint8_t *mac, esp_now_send_status_t status)
#endif
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
    (void)info;
#else
    (void)mac;
#endif
    bool ok = (status == ESP_NOW_SEND_SUCCESS);
    if (ok) s_tx_acked++; else s_tx_not_acked++;
    s_last_acked = ok;
}

/* Called from the Wi-Fi task. Keep it short: copy and return. */
static void recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (info == NULL || data == NULL) return;
    if (memcmp(info->src_addr, s_peer, ESP_NOW_ETH_ALEN) != 0) return;  // not our pair

    if (s_role == GC_ROLE_RECEIVER && len == sizeof(GC_TxPacket)) {
        const GC_TxPacket *pkt = (const GC_TxPacket *)data;
        int64_t now = esp_timer_get_time();

        portENTER_CRITICAL(&s_lock);
        if (pkt->type == GC_PKT_INPUT) {
            // Drop old/duplicate packets, unless the link went stale
            // (e.g. controller rebooted and its seq restarted at 0).
            bool stale = (now - s_last_input_us) > GC_LINK_STALE_US;
            if (stale || !s_have_input || (int8_t)(pkt->seq - s_last_seq) > 0) {
                s_input         = pkt->data.state;
                s_last_seq      = pkt->seq;
                s_last_input_us = now;
                s_have_input    = true;
            }
        } else if (pkt->type == GC_PKT_ORIGIN) {
            s_origin      = pkt->data;
            s_have_origin = true;
        }
        portEXIT_CRITICAL(&s_lock);

    } else if (s_role == GC_ROLE_CONTROLLER && len == sizeof(GC_RxPacket)) {
        const GC_RxPacket *pkt = (const GC_RxPacket *)data;
        s_rumble = pkt->rumble ? 1 : 0;
        s_last_rumble_rx_us = esp_timer_get_time();
    }
}

esp_err_t gc_link_init(gc_role_t role, const uint8_t peer_mac[6])
{
    s_role = role;
    memcpy(s_peer, peer_mac, ESP_NOW_ETH_ALEN);

    wifi_init(role);

    uint8_t my_mac[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, my_mac));
    ESP_LOGI(TAG, "%s MAC: " MACSTR,
             role == GC_ROLE_CONTROLLER ? "CONTROLLER" : "RECEIVER", MAC2STR(my_mac));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(send_cb));

#if CONFIG_GC_POWER_SAVE
#if !CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE
#warning "CONFIG_GC_POWER_SAVE needs CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE=y to take effect"
#endif
    if (use_power_save(role)) {
        _Static_assert(CONFIG_GC_PS_WAKE_WINDOW_MS < CONFIG_GC_PS_WAKE_INTERVAL_MS,
                       "wake window must be shorter than wake interval");
        ESP_ERROR_CHECK(esp_now_set_wake_window(CONFIG_GC_PS_WAKE_WINDOW_MS));
        ESP_ERROR_CHECK(esp_wifi_connectionless_module_set_wake_interval(CONFIG_GC_PS_WAKE_INTERVAL_MS));
        ESP_LOGI(TAG, "Radio power save on: listen %d ms every %d ms",
                 CONFIG_GC_PS_WAKE_WINDOW_MS, CONFIG_GC_PS_WAKE_INTERVAL_MS);
    }
#endif

    static const uint8_t zero[ESP_NOW_ETH_ALEN] = {0};
    if (memcmp(s_peer, zero, ESP_NOW_ETH_ALEN) == 0) {
        ESP_LOGW(TAG, "Peer MAC not set - link disabled (set CONFIG_GC_PEER_MAC)");
        return ESP_ERR_INVALID_ARG;
    }

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, s_peer, ESP_NOW_ETH_ALEN);
    peer.channel = GC_LINK_CHANNEL;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    return esp_now_add_peer(&peer);
}

bool gc_link_parse_mac(const char *s, uint8_t mac[6])
{
    unsigned v[6];
    if (s == NULL ||
        sscanf(s, "%x:%x:%x:%x:%x:%x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) {
        return false;
    }
    for (int i = 0; i < 6; i++) {
        if (v[i] > 0xFF) return false;
        mac[i] = (uint8_t)v[i];
    }
    return true;
}

void gc_link_tx_stats(uint32_t *acked, uint32_t *not_acked)
{
    *acked     = s_tx_acked;
    *not_acked = s_tx_not_acked;
}

bool gc_link_last_send_acked(void)
{
    return s_last_acked;
}

/* ---------- Controller side ---------- */

static esp_err_t send_tx_packet(uint8_t type, const GC_Origin *data)
{
    GC_TxPacket pkt;
    pkt.type = type;
    pkt.seq  = s_tx_seq++;
    pkt.data = *data;
    // If the TX queue is full this returns an error. The caller retries on
    // the next poll.
    return esp_now_send(s_peer, (const uint8_t *)&pkt, sizeof(pkt));
}

esp_err_t gc_link_send_input(const GC_Payload *p)
{
    GC_Origin wrap = {0};
    wrap.state = *p;
    return send_tx_packet(GC_PKT_INPUT, &wrap);
}

esp_err_t gc_link_send_origin(const GC_Origin *o)
{
    return send_tx_packet(GC_PKT_ORIGIN, o);
}

uint8_t gc_link_get_rumble(void)
{
    if (esp_timer_get_time() - s_last_rumble_rx_us > GC_LINK_RUMBLE_TIMEOUT_US) {
        return 0;   // receiver gone quiet: never leave the motor running
    }
    return s_rumble;
}

/* ---------- Receiver side ---------- */

esp_err_t gc_link_send_rumble(uint8_t on)
{
    GC_RxPacket pkt = { .rumble = on ? 1 : 0 };
    return esp_now_send(s_peer, (const uint8_t *)&pkt, sizeof(pkt));
}

bool IRAM_ATTR gc_link_get_input(GC_Payload *out)
{
    bool fresh;
    portENTER_CRITICAL_SAFE(&s_lock);
    *out  = s_input;
    fresh = s_have_input && (esp_timer_get_time() - s_last_input_us) < GC_LINK_STALE_US;
    portEXIT_CRITICAL_SAFE(&s_lock);
    return fresh;
}

bool IRAM_ATTR gc_link_get_origin(GC_Origin *out)
{
    bool have;
    portENTER_CRITICAL_SAFE(&s_lock);
    *out = s_origin;
    have = s_have_origin;
    portEXIT_CRITICAL_SAFE(&s_lock);
    return have;
}