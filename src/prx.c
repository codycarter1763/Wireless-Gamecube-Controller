/*
 * PRX = receiver board, plugged into the console.
 *
 * - ESB: receives GC_TxPacket from the controller board. Every new input or
 *   origin is handed to the Joybus target, which pre-encodes the replies.
 * - Joybus target: answers the console's ID / origin / poll commands from
 *   those pre-encoded replies (see joybus_target.c).
 * - Rumble: the bit from the console's latest poll goes back to the
 *   controller board in the ESB ACK payload (GC_RxPacket).
 * - If the controller board goes quiet for 100 ms, the console gets neutral
 *   input instead of the last state, so nothing sticks held down.
 */
#include <esb.h>
#include <string.h>
#include <zephyr/kernel.h>
#include "link.h"
#include "gc_protocol.h"
#include "gc_debug.h"
#include "joybus.h"

#define INPUT_STALE_MS  100

struct prx_stats {
	uint32_t rx;
	uint32_t inputs;
	uint32_t origins;
	uint32_t missed;     /* sequence gaps = packets the controller gave up on */
	int32_t rssi_sum;    /* ESB reports RSSI as a positive number, -dBm */
	uint32_t ack_queue_full;
};

static struct prx_stats stats;
static uint8_t expected_seq;
static bool have_seq;

/* Latest state from the controller board. Written in the ESB ISR. */
static GC_Payload s_input;
static int64_t s_input_ms = -INPUT_STALE_MS;
static GC_Origin s_origin = {
	.state = {
		.byte1 = GC_BYTE1_ALWAYS_ONE,
		.joy_X_val = 128, .joy_Y_val = 128,
		.C_stick_X_val = 128, .C_stick_Y_val = 128,
	},
};
static bool s_have_origin;
static bool s_stale = true;

/* ------------------------------------------------------------------ */
/* Getters                                                             */
/* ------------------------------------------------------------------ */

bool gc_rx_get_input(GC_Payload *out)
{
	unsigned int key = irq_lock();
	bool fresh = (k_uptime_get() - s_input_ms) < INPUT_STALE_MS;

	*out = s_input;
	irq_unlock(key);
	return fresh;
}

bool gc_rx_get_origin(GC_Origin *out)
{
	unsigned int key = irq_lock();
	bool have = s_have_origin;

	*out = s_origin;
	irq_unlock(key);
	return have;
}

/* Neutral = origin sticks/triggers, no buttons. */
static GC_Payload neutral_input(void)
{
	GC_Payload n = s_origin.state;

	n.byte0 = 0;
	n.byte1 = GC_BYTE1_ALWAYS_ONE;
	return n;
}

/* ------------------------------------------------------------------ */
/* Radio                                                               */
/* ------------------------------------------------------------------ */

static void queue_ack(void)
{
	struct esb_payload ack = {
		.pipe = 0,
		.length = sizeof(GC_RxPacket),
	};
	GC_RxPacket r = { .rumble = joybus_target_rumble() };

	memcpy(ack.data, &r, sizeof(r));
	if (esb_write_payload(&ack) == -ENOMEM) {
		/* Keep only the newest ACK payload queued. */
		stats.ack_queue_full++;
		esb_flush_tx();
		esb_write_payload(&ack);
	}
}

/* Runs in the ESB event ISR. Keep it short: no printk here. */
void app_esb_handler(const struct esb_evt *event)
{
	static struct esb_payload rx;

	if (event->evt_id != ESB_EVENT_RX_RECEIVED) {
		return; /* TX_SUCCESS here just means an ACK payload went out */
	}

	while (esb_read_rx_payload(&rx) == 0) {
		GC_TxPacket pkt;

		dbg_pin_set(true);
		if (rx.length < sizeof(pkt)) {
			dbg_pin_set(false);
			continue;
		}
		memcpy(&pkt, rx.data, sizeof(pkt));

		if (have_seq && pkt.seq != expected_seq) {
			stats.missed += (uint8_t)(pkt.seq - expected_seq);
		}
		expected_seq = pkt.seq + 1;
		have_seq = true;

		if (pkt.type == GC_PKT_INPUT) {
			s_input = pkt.data.state;
			s_input_ms = k_uptime_get();
			s_stale = false;
			stats.inputs++;
			joybus_target_set_state(&s_input, &s_origin);
		} else if (pkt.type == GC_PKT_ORIGIN) {
			s_origin = pkt.data;
			s_have_origin = true;
			stats.origins++;

			GC_Payload cur = s_stale ? neutral_input() : s_input;

			joybus_target_set_state(&cur, &s_origin);
		}

		stats.rx++;
		stats.rssi_sum += rx.rssi;
		if ((stats.rx % 100) == 0) {
			led_toggle();
		}

		/* Goes out with the ACK for the *next* packet. */
		queue_ack();
		dbg_pin_set(false);
	}
}

#if defined(CONFIG_GC_RX_NO_SLEEP)
/* Lowest-priority spinner: keeps the CPU out of WFI idle (see Kconfig). */
static void spin_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	for (;;) {
		__NOP();
	}
}
K_THREAD_DEFINE(no_sleep, 256, spin_fn, NULL, NULL, NULL,
		K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
#endif

/* Every 20 ms: if the controller board went quiet, switch to neutral. */
static void stale_fn(struct k_timer *t)
{
	ARG_UNUSED(t);
	unsigned int key = irq_lock();

	if (!s_stale && (k_uptime_get() - s_input_ms) >= INPUT_STALE_MS) {
		GC_Payload n = neutral_input();

		s_stale = true;
		joybus_target_set_state(&n, &s_origin);
	}
	irq_unlock(key);
}
static K_TIMER_DEFINE(stale_timer, stale_fn, NULL);

/* ------------------------------------------------------------------ */
/* Main loop                                                           */
/* ------------------------------------------------------------------ */

static void print_stats(void)
{
	struct prx_stats s;
	struct joybus_target_stats j;
	unsigned int key = irq_lock();

	s = stats;
	memset(&stats, 0, sizeof(stats));
	irq_unlock(key);
	joybus_target_get_stats(&j, true);

	GC_Payload in;
	GC_Origin org;
	bool fresh = gc_rx_get_input(&in);
	bool have_org = gc_rx_get_origin(&org);
	char line[96];

	gc_format_payload(&in, line, sizeof(line));
	printk("%s | %s | rumble %u\n", line, fresh ? "live" : "STALE->neutral",
	       joybus_target_rumble());
	printk("  radio rx %u (input %u origin %u) missed %u rssi -%d dBm | origin %s\n",
	       s.rx, s.inputs, s.origins, s.missed,
	       s.rx ? (int)(s.rssi_sum / (int32_t)s.rx) : 0,
	       have_org ? "ok" : "not yet");
	printk("  console polls %u origin %u id %u | errors %u unknown %u late %u\n",
	       j.polls, j.origins, j.ids, j.errors, j.unknown, j.late);
}

void role_run(void)
{
	int err;

	err = joybus_target_init(CONFIG_GC_DATA_PIN);
	if (err) {
		printk("joybus_target_init failed: %d\n", err);
		return;
	}

	/* Preload one ACK payload so the very first ACK carries data. */
	queue_ack();

	err = esb_start_rx();
	if (err) {
		printk("esb_start_rx failed: %d\n", err);
		return;
	}

	k_timer_start(&stale_timer, K_MSEC(20), K_MSEC(20));

	while (1) {
		k_msleep(1000);
		print_stats();
	}
}
