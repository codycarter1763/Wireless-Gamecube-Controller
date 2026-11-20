/*
 * PTX = controller board: holds the real GameCube controller.
 *
 * Loop, every CONFIG_APP_POLL_INTERVAL_US:
 *   1. Poll the pad over Joybus (interrupts locked, ~370 us), with the
 *      rumble bit taken from the latest ACK payload.
 *   2. Send the result to the receiver as a GC_TxPacket over ESB and wait
 *      for the ACK (~270 us). The ACK carries GC_RxPacket (rumble) back.
 * Steps 1 and 2 never overlap, so Joybus timing and the radio don't fight.
 *
 * The origin (neutral stick/trigger values) is sent at startup and then
 * every 500 ms, so a receiver that powers up late still gets it.
 * If the pad stops answering, we go back to detecting it.
 */
#include <esb.h>
#include <string.h>
#include <zephyr/kernel.h>
#include "link.h"
#include "gc_protocol.h"
#include "gc_debug.h"
#include "joybus.h"

#define ORIGIN_RESEND_MS    500
#define RUMBLE_TIMEOUT_MS   500   /* no ACK payload for this long -> rumble off */
#define PAD_LOST_POLLS      100   /* consecutive bad polls -> re-detect pad */

struct ptx_stats {
	/* Joybus */
	uint32_t polls_ok;
	uint32_t polls_bad;
	/* Radio */
	uint32_t sent;
	uint32_t ok;
	uint32_t failed;     /* all retries used up, no ACK */
	uint32_t timeouts;   /* no ESB event at all (should stay 0) */
	uint32_t retries;    /* extra attempts beyond the first */
	uint32_t rtt_min;
	uint32_t rtt_max;
	uint64_t rtt_sum;
};

static struct ptx_stats stats;
static volatile uint32_t t_start;
static volatile bool last_failed;

/* Rumble from the receiver, via ACK payloads. */
static volatile uint8_t s_rumble;
static volatile int64_t s_rumble_ms = -RUMBLE_TIMEOUT_MS;

static K_SEM_DEFINE(tx_done, 0, 1);
static K_SEM_DEFINE(tick, 0, 1);

static void tick_fn(struct k_timer *t)
{
	ARG_UNUSED(t);
	k_sem_give(&tick);
}
static K_TIMER_DEFINE(poll_timer, tick_fn, NULL);

static void stats_reset(void)
{
	memset(&stats, 0, sizeof(stats));
	stats.rtt_min = UINT32_MAX;
}

/* ------------------------------------------------------------------ */
/* Radio                                                               */
/* ------------------------------------------------------------------ */

/* Runs in the ESB event ISR. Keep it short: no printk here. */
void app_esb_handler(const struct esb_evt *event)
{
	static struct esb_payload rx;

	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS: {
		uint32_t rtt = us_now_isr() - t_start;

		dbg_pin_set(false);
		stats.ok++;
		stats.retries += event->tx_attempts ? event->tx_attempts - 1 : 0;
		stats.rtt_sum += rtt;
		stats.rtt_min = MIN(stats.rtt_min, rtt);
		stats.rtt_max = MAX(stats.rtt_max, rtt);
		last_failed = false;
		if ((stats.ok % 100) == 0) {
			led_toggle(); /* ~5 Hz blink = pad + radio both working */
		}
		k_sem_give(&tx_done);
		break;
	}
	case ESB_EVENT_TX_FAILED:
		dbg_pin_set(false);
		stats.failed++;
		last_failed = true;
		k_sem_give(&tx_done);
		break;
	case ESB_EVENT_RX_RECEIVED:
		/* ACK payload from the receiver. Don't touch tx_done here: this
		 * event can arrive separately from TX_SUCCESS. */
		while (esb_read_rx_payload(&rx) == 0) {
			if (rx.length >= sizeof(GC_RxPacket)) {
				GC_RxPacket r;

				memcpy(&r, rx.data, sizeof(r));
				s_rumble = r.rumble ? 1 : 0;
				s_rumble_ms = k_uptime_get();
			}
		}
		break;
	default:
		break;
	}
}

static uint8_t current_rumble(void)
{
	return (k_uptime_get() - s_rumble_ms < RUMBLE_TIMEOUT_MS) ? s_rumble : 0;
}

/* Send one packet and wait for the ACK (or failure). */
static void radio_send(const GC_TxPacket *pkt)
{
	struct esb_payload tx = {
		.pipe = 0,
		.noack = false,
		.length = sizeof(*pkt),
	};

	if (last_failed) {
		/* A failed packet stays in the TX FIFO; drop it, we only ever
		 * want the newest controller state on air. */
		esb_flush_tx();
		last_failed = false;
	}

	memcpy(tx.data, pkt, sizeof(*pkt));

	dbg_pin_set(true);
	t_start = us_now();
	int err = esb_write_payload(&tx);

	if (err) {
		dbg_pin_set(false);
		esb_flush_tx();
		return;
	}
	stats.sent++;
	if (k_sem_take(&tx_done, K_MSEC(10)) != 0) {
		stats.timeouts++;
		esb_flush_tx();
	}
}

/* ------------------------------------------------------------------ */
/* Pad                                                                 */
/* ------------------------------------------------------------------ */

/* Block until a pad answers, then read its origin. */
static void detect_pad(GC_Origin *origin)
{
	uint8_t id[3];
	int tries = 0;
	char line[96];

	while (joybus_host_read_id(id) != 0) {
		if (tries++ % 20 == 0) {
			printk("No controller detected on P0.%02d, retrying...\n",
			       CONFIG_GC_DATA_PIN);
		}
		k_msleep(100);
	}
	printk("Controller ID: %02X %02X %02X  (header expects %02X %02X %02X)\n",
	       id[0], id[1], id[2], GC_ID_BYTE0, GC_ID_BYTE1, GC_ID_BYTE2);

	k_msleep(1);
	while (joybus_host_read_origin(origin) != 0) {
		printk("Origin read failed, retrying...\n");
		k_msleep(100);
	}
	gc_format_payload(&origin->state, line, sizeof(line));
	printk("Origin: %s\n", line);
}

static void print_stats(const GC_Payload *pad)
{
	struct ptx_stats s;
	char line[96];
	unsigned int key = irq_lock();

	s = stats;
	stats_reset();
	irq_unlock(key);

	uint32_t avg = s.ok ? (uint32_t)(s.rtt_sum / s.ok) : 0;

	gc_format_payload(pad, line, sizeof(line));
	printk("%s | rumble %u\n", line, current_rumble());
	printk("  polls ok %u bad %u | radio sent %u ok %u fail %u timeout %u retries %u"
	       " | RTT us min %u avg %u max %u\n",
	       s.polls_ok, s.polls_bad, s.sent, s.ok, s.failed, s.timeouts, s.retries,
	       s.ok ? s.rtt_min : 0, avg, s.rtt_max);
}

/* ------------------------------------------------------------------ */
/* Main loop                                                           */
/* ------------------------------------------------------------------ */

void role_run(void)
{
	GC_TxPacket pkt;
	GC_Origin origin;
	GC_Payload pad = {0};
	uint8_t seq = 0;
	int bad_streak = 0;

	stats_reset();

	if (joybus_host_init(CONFIG_GC_DATA_PIN) != 0) {
		printk("Bad CONFIG_GC_DATA_PIN %d\n", CONFIG_GC_DATA_PIN);
		return;
	}

	for (;;) {
		/* (Re)detect the pad and announce its origin. */
		detect_pad(&origin);
		pkt.type = GC_PKT_ORIGIN;
		pkt.seq = seq++;
		pkt.data = origin;
		radio_send(&pkt);

		int64_t last_origin_ms = k_uptime_get();
		int64_t next_print = last_origin_ms + 1000;

		bad_streak = 0;
		k_timer_start(&poll_timer, K_USEC(CONFIG_APP_POLL_INTERVAL_US),
			      K_USEC(CONFIG_APP_POLL_INTERVAL_US));

		while (bad_streak < PAD_LOST_POLLS) {
			k_sem_take(&tick, K_FOREVER);

			/* 1. Joybus */
			uint32_t cmd = GC_POLL | (current_rumble() ? GC_RUMBLE_BIT : 0);
			GC_Payload p;

			if (joybus_host_poll(cmd, &p) == 0 && gc_payload_valid(&p)) {
				stats.polls_ok++;
				bad_streak = 0;
				pad = p;

				/* 2. Radio */
				pkt.type = GC_PKT_INPUT;
				pkt.seq = seq++;
				memset(&pkt.data, 0, sizeof(pkt.data));
				pkt.data.state = pad;
				radio_send(&pkt);
			} else {
				stats.polls_bad++;
				bad_streak++;
			}

			int64_t now = k_uptime_get();

			if (now - last_origin_ms >= ORIGIN_RESEND_MS) {
				pkt.type = GC_PKT_ORIGIN;
				pkt.seq = seq++;
				pkt.data = origin;
				radio_send(&pkt);
				last_origin_ms = now;
			}

			if (now >= next_print) {
				next_print += 1000;
				print_stats(&pad);
			}
		}

		k_timer_stop(&poll_timer);
		printk("Controller lost (%d bad polls in a row), re-detecting\n", PAD_LOST_POLLS);
	}
}
