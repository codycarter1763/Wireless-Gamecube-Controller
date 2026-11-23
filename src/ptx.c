/*
 * PTX = controller board: holds the real GameCube controller.
 *
 * Each poll tick:
 *   1. Poll the pad over Joybus (interrupts masked, ~370 us), with the
 *      rumble bit taken from the latest ACK payload.
 *   2. If the input changed (or the keepalive is due), send it to the
 *      receiver as a GC_TxPacket over ESB and wait for the ACK, which
 *      carries GC_RxPacket (rumble) back.
 * Steps 1 and 2 never overlap, so Joybus timing and the radio don't fight.
 *
 * Power states (all timed from the last real activity: a button change or a
 * stick/trigger movement of at least CONFIG_APP_ACTIVITY_THRESH):
 *   ACTIVE  HF crystal kept on so a send never waits for it. Adaptive poll
 *           rate: every CONFIG_APP_POLL_INTERVAL_US (1 kHz) while the input
 *           is changing; after CONFIG_APP_FAST_HOLD_MS with no change, every
 *           CONFIG_APP_POLL_SLOW_US (250 Hz). The first change switches
 *           straight back to fast.
 *   DOZE    after CONFIG_APP_DOZE_S: poll every CONFIG_APP_DOZE_POLL_MS, HF
 *           crystal on only while sending, status LED off. Any activity
 *           returns to ACTIVE immediately.
 *   OFF     after CONFIG_APP_SLEEP_MIN: pad power (VCC) cut, nRF System OFF.
 *           A reset (or CONFIG_GC_WAKE_PIN) boots it again.
 * A button held down without moving is not "activity", but it is still
 * sent every keepalive, so the console never sees it released.
 *
 * Link loss: after CONFIG_APP_LINK_LOSS_MS with no acknowledged packet at
 * all, the controller stops sending on input changes and just probes every
 * CONFIG_APP_LINK_PROBE_MS with the HF crystal off in between. The first acknowledged probe restores the link (and resends the
 * origin). No receiver for CONFIG_APP_LINK_LOST_SLEEP_MIN -> power off.
 *
 * OTA: holding L + R + Z + A for CONFIG_APP_OTA_HOLD_MS reboots into the
 * Adafruit bootloader's Bluetooth DFU mode (GPREGRET = 0xA8). The LED
 * flickers while the combo is counting and stays on once it triggers.
 *
 * The origin (neutral stick/trigger values) is sent at startup and then
 * every 500 ms, so a receiver that powers up late still gets it.
 * If the pad stops answering, we go back to detecting it.
 */
#include <esb.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/sys/poweroff.h>
#include <hal/nrf_gpio.h>
#include "link.h"
#include "gc_protocol.h"
#include "gc_debug.h"
#include "joybus.h"
#include "gc_logic.h"

#define ORIGIN_RESEND_MS    500
#define RUMBLE_TIMEOUT_MS   500   /* no ACK payload for this long -> rumble off */
#define PAD_LOST_POLLS      100   /* consecutive bad polls -> re-detect pad */
#define LED_PERIOD_MS       2000  /* status pulse while connected */
#define LED_PULSE_MS        20
#define LED_SEARCH_PERIOD_MS 1000 /* blink while searching for the receiver */
#define LED_SEARCH_PULSE_MS 50

/* Adafruit nRF52 bootloader: GPREGRET value meaning "start in BLE OTA DFU
 * mode, entered by soft reset" (DFU_MAGIC_OTA_RESET in its src/main.c). */
#define DFU_MAGIC_OTA_RESET 0xA8
#define OTA_FEEDBACK_MS     1000   /* start flickering the LED after this */

enum pwr_state { PWR_ACTIVE, PWR_DOZE };

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
	uint32_t link_lost;  /* times the link was declared lost */
};

static struct ptx_stats stats;
static volatile uint32_t t_start;
static volatile bool last_failed;
static volatile bool last_ok;

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
/* Pad power (VCC switch on Pro Micro nRF52840 / nice!nano boards)      */
/* ------------------------------------------------------------------ */

static void vcc_set(bool on)
{
#if CONFIG_GC_VCC_CTRL_PIN >= 0
	const uint32_t pin = NRF_GPIO_PIN_MAP(0, CONFIG_GC_VCC_CTRL_PIN);
	bool level = IS_ENABLED(CONFIG_GC_VCC_ON_HIGH) ? on : !on;

	nrf_gpio_pin_write(pin, level);
	nrf_gpio_cfg_output(pin);
#else
	ARG_UNUSED(on);
#endif
}

/* ------------------------------------------------------------------ */
/* HF crystal: the radio needs it; it costs a few hundred uA while on   */
/* ------------------------------------------------------------------ */

static struct onoff_manager *s_hf_mgr;
static struct onoff_client s_hf_cli;
static bool s_hf_on;

static void hfclk_on(void)
{
	int res;

	if (s_hf_on) {
		return;
	}
	if (!s_hf_mgr) {
		s_hf_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	}
	sys_notify_init_spinwait(&s_hf_cli.notify);
	if (onoff_request(s_hf_mgr, &s_hf_cli) < 0) {
		return;
	}
	/* Crystal start-up, typically a few hundred us. */
	while (sys_notify_fetch_result(&s_hf_cli.notify, &res) == -EAGAIN) {
	}
	s_hf_on = true;
}

static void hfclk_off(void)
{
	if (s_hf_on) {
		onoff_release(s_hf_mgr);
		s_hf_on = false;
	}
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
		last_ok = true;
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

/* Send one packet and wait for the ACK (or failure). Switches the HF crystal
 * on if needed; while dozing or searching, the caller turns it off again.
 * Returns true if the receiver acknowledged it. */
static bool radio_send(const GC_TxPacket *pkt)
{
	struct esb_payload tx = {
		.pipe = 0,
		.noack = false,
		.length = sizeof(*pkt),
	};

	hfclk_on();

	if (last_failed) {
		/* A failed packet stays in the TX FIFO; drop it, we only ever
		 * want the newest controller state on air. */
		esb_flush_tx();
		last_failed = false;
	}

	memcpy(tx.data, pkt, sizeof(*pkt));

	last_ok = false;
	dbg_pin_set(true);
	t_start = us_now();
	int err = esb_write_payload(&tx);

	if (err) {
		dbg_pin_set(false);
		esb_flush_tx();
		return false;
	}
	stats.sent++;
	if (k_sem_take(&tx_done, K_MSEC(10)) != 0) {
		stats.timeouts++;
		esb_flush_tx();
		return false;
	}
	return last_ok;
}

/* ------------------------------------------------------------------ */
/* Pad                                                                 */
/* ------------------------------------------------------------------ */

static void power_off(const char *why);

/* Block until a pad answers, then read its origin. With no pad for the
 * whole sleep timeout, power off instead of retrying forever. */
static void detect_pad(GC_Origin *origin)
{
	uint8_t id[3];
	int tries = 0;
	char line[96];
	int64_t start = k_uptime_get();

	while (joybus_host_read_id(id) != 0) {
		if (tries++ % 20 == 0) {
			printk("No controller detected on P0.%02d, retrying...\n",
			       CONFIG_GC_DATA_PIN);
		}
		if (CONFIG_APP_SLEEP_MIN > 0 &&
		    k_uptime_get() - start >= (int64_t)CONFIG_APP_SLEEP_MIN * 60 * 1000) {
			power_off("No controller found");
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

/* Cut pad power and put the nRF in System OFF. Does not return: the next
 * thing that happens is a fresh boot from reset (or the wake pin). */
static void power_off(const char *why)
{
	printk("%s: powering off. Press reset to wake.\n", why);
	k_msleep(20);   /* let the message out, if a console is attached */

	k_timer_stop(&poll_timer);
	esb_disable();
	hfclk_off();
	led_set(false);

	/* Release the Joybus pin before cutting VCC, so the pad isn't
	 * back-powered through the pin's pull-up. */
	nrf_gpio_cfg_default(NRF_GPIO_PIN_MAP(0, CONFIG_GC_DATA_PIN));
	vcc_set(false);   /* GPIO state is kept through System OFF */

#if CONFIG_GC_WAKE_PIN >= 0
	nrf_gpio_cfg_sense_input(NRF_GPIO_PIN_MAP(0, CONFIG_GC_WAKE_PIN),
				 NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
#endif
	sys_poweroff();
}

/* Reboot into the bootloader's Bluetooth OTA mode. Does not return. */
static void enter_ota(void)
{
	printk("OTA combo held: rebooting into the Bluetooth bootloader "
	       "(advertises as \"AdaDFU\")\n");
	k_msleep(20);   /* let the message out, if a console is attached */

	k_timer_stop(&poll_timer);
	esb_disable();
	hfclk_off();
	led_set(true);
	nrf_gpio_cfg_default(NRF_GPIO_PIN_MAP(0, CONFIG_GC_DATA_PIN));

	/* GPREGRET survives a soft reset; the bootloader reads and clears it.
	 * (Zephyr's sys_reboot() on nRF ignores its argument, so write it
	 * ourselves and reset directly.) */
	NRF_POWER->GPREGRET = DFU_MAGIC_OTA_RESET;
	NVIC_SystemReset();
}

static void print_stats(const GC_Payload *pad, enum pwr_state st, bool fast, bool link_up)
{
	struct ptx_stats s;
	char line[96];
	unsigned int key = irq_lock();

	s = stats;
	stats_reset();
	irq_unlock(key);

	uint32_t avg = s.ok ? (uint32_t)(s.rtt_sum / s.ok) : 0;

	gc_format_payload(pad, line, sizeof(line));
	printk("%s | rumble %u | %s | %s\n", line, current_rumble(),
	       st == PWR_DOZE ? "doze" : (fast ? "active, fast poll" : "active, slow poll"),
	       link_up ? "linked" : "SEARCHING for receiver");
	printk("  polls ok %u bad %u | radio sent %u ok %u fail %u timeout %u retries %u"
	       " | RTT us min %u avg %u max %u | link lost %u\n",
	       s.polls_ok, s.polls_bad, s.sent, s.ok, s.failed, s.timeouts, s.retries,
	       s.ok ? s.rtt_min : 0, avg, s.rtt_max, s.link_lost);
}

static void set_poll_rate(enum pwr_state st, bool fast)
{
	k_timeout_t period;

	if (st == PWR_DOZE) {
		period = K_MSEC(CONFIG_APP_DOZE_POLL_MS);
	} else if (fast) {
		period = K_USEC(CONFIG_APP_POLL_INTERVAL_US);
	} else {
		period = K_USEC(CONFIG_APP_POLL_SLOW_US);
	}
	/* Restarting also re-times the next tick from now, so switching to
	 * fast takes effect within one fast interval. */
	k_timer_start(&poll_timer, period, period);
}

/* ------------------------------------------------------------------ */
/* Main loop                                                           */
/* ------------------------------------------------------------------ */

void role_run(void)
{
	GC_TxPacket pkt;
	GC_Origin origin;
	GC_Payload pad = {0};
	GC_Payload last_sent = {0};
	GC_Payload act_ref = {0};
	uint8_t seq = 0;

	stats_reset();
	vcc_set(true);          /* pad power on (it's on by default anyway) */
	k_msleep(10);           /* let the pad power up */

	if (joybus_host_init(CONFIG_GC_DATA_PIN) != 0) {
		printk("Bad CONFIG_GC_DATA_PIN %d\n", CONFIG_GC_DATA_PIN);
		return;
	}

	for (;;) {
		/* (Re)detect the pad and announce its origin. */
		detect_pad(&origin);

		enum pwr_state st = PWR_ACTIVE;
		int64_t now = k_uptime_get();
		int64_t last_activity = now;
		int64_t last_send = now;
		int64_t last_origin_ms = now;
		int64_t next_print = now + 1000;
		int64_t next_led = now;
		int bad_streak = 0;
		struct gc_link link;     /* assume up until sends start failing */
		struct gc_rate rate;     /* adaptive poll rate (ACTIVE only) */
		int64_t combo_since = -1; /* OTA combo held since, or -1 */

		gc_link_init(&link, now);

		hfclk_on();
		pkt.type = GC_PKT_ORIGIN;
		pkt.seq = seq++;
		pkt.data = origin;
		radio_send(&pkt);
		act_ref = origin.state;
		gc_rate_init(&rate, &origin.state, now);
		set_poll_rate(st, rate.fast);

		while (bad_streak < PAD_LOST_POLLS) {
			k_sem_take(&tick, K_FOREVER);
			now = k_uptime_get();

			/* 1. Joybus */
			uint32_t cmd = GC_POLL | (current_rumble() ? GC_RUMBLE_BIT : 0);
			GC_Payload p;

			if (joybus_host_poll(cmd, &p) != 0 || !gc_payload_valid(&p)) {
				stats.polls_bad++;
				bad_streak++;
				continue;
			}
			stats.polls_ok++;
			bad_streak = 0;
			pad = p;

			/* Adaptive poll rate: a real change means someone's moving;
			 * keep polling fast for a while after (see gc_rate_update).
			 * The rate only applies while ACTIVE. */
			if (gc_rate_update(&rate, &pad, now, CONFIG_APP_FAST_THRESH,
					   CONFIG_APP_FAST_HOLD_MS) && st == PWR_ACTIVE) {
				set_poll_rate(st, rate.fast);
			}

			/* Activity: a button change, or a real stick/trigger move. */
			if (gc_buttons_changed(&pad, &act_ref) ||
			    gc_analog_moved(&pad, &act_ref, CONFIG_APP_ACTIVITY_THRESH)) {
				act_ref = pad;
				last_activity = now;
				if (st == PWR_DOZE) {
					st = PWR_ACTIVE;
					gc_rate_init(&rate, &pad, now);
					hfclk_on();
					set_poll_rate(st, rate.fast);
				}
			}

			/* 2. Radio. Linked: only when something changed, the
			 * keepalive is due, or rumble is on (so "rumble off" arrives
			 * quickly). Link lost: only a probe every LINK_PROBE_MS. */
			bool changed = gc_input_changed(&pad, &last_sent,
							CONFIG_APP_ANALOG_DEADBAND);

			if (gc_link_want_send(&link, now, changed, current_rumble(),
					      last_send, CONFIG_APP_KEEPALIVE_MS,
					      CONFIG_APP_LINK_PROBE_MS)) {
				pkt.type = GC_PKT_INPUT;
				pkt.seq = seq++;
				memset(&pkt.data, 0, sizeof(pkt.data));
				pkt.data.state = pad;

				bool ok = radio_send(&pkt);

				last_sent = pad;
				last_send = now;

				switch (gc_link_on_send(&link, now, ok, CONFIG_APP_LINK_LOSS_MS)) {
				case GC_LINK_RESTORED:
					/* Back: resend the origin right away, in
					 * case the receiver was restarted. */
					if (st == PWR_ACTIVE) {
						hfclk_on();
					}
					last_origin_ms = 0;
					break;
				case GC_LINK_LOST:
					stats.link_lost++;
					break;
				default:
					break;
				}
			}

			if (link.up && now - last_origin_ms >= ORIGIN_RESEND_MS) {
				pkt.type = GC_PKT_ORIGIN;
				pkt.seq = seq++;
				pkt.data = origin;
				radio_send(&pkt);
				last_origin_ms = now;
			}

			/* Dozing or searching: crystal only on while sending. */
			if (st == PWR_DOZE || !link.up) {
				hfclk_off();
			}

			/* OTA combo: L + R + Z + A held continuously. */
			bool ota_feedback = false;
			bool was_held = combo_since >= 0;
			int64_t held = gc_combo_hold(&combo_since,
						     IS_ENABLED(CONFIG_APP_OTA_COMBO) &&
						     gc_ota_combo_pressed(&pad), now);

			if (held >= 0) {
#if defined(CONFIG_APP_OTA_HOLD_MS)
				if (held >= CONFIG_APP_OTA_HOLD_MS) {
					enter_ota();
				}
#endif
				if (held >= OTA_FEEDBACK_MS) {
					/* Flicker (~5 Hz) so you know it's counting. */
					led_set(((now / 100) & 1) != 0);
					ota_feedback = true;
				}
			} else if (was_held) {
				led_set(false);
			}

			/* Status LED (off while dozing): short pulse every 2 s while
			 * linked, longer blink every 1 s while searching. */
			if (IS_ENABLED(CONFIG_APP_STATUS_LED) && st == PWR_ACTIVE && !ota_feedback) {
				int64_t period = link.up ? LED_PERIOD_MS : LED_SEARCH_PERIOD_MS;
				int64_t pulse = link.up ? LED_PULSE_MS : LED_SEARCH_PULSE_MS;

				if (now >= next_led) {
					led_set(true);
					next_led = now + period;
				} else if (now >= next_led - period + pulse) {
					led_set(false);
				}
			}

			/* Idle timers. */
			int64_t idle = now - last_activity;

			if (CONFIG_APP_SLEEP_MIN > 0 &&
			    idle >= (int64_t)CONFIG_APP_SLEEP_MIN * 60 * 1000) {
				power_off("No activity");
			}
			if (!link.up && CONFIG_APP_LINK_LOST_SLEEP_MIN > 0 &&
			    now - link.lost_at >= (int64_t)CONFIG_APP_LINK_LOST_SLEEP_MIN * 60 * 1000) {
				power_off("No receiver");
			}
			if (st == PWR_ACTIVE && CONFIG_APP_DOZE_S > 0 &&
			    idle >= (int64_t)CONFIG_APP_DOZE_S * 1000) {
				st = PWR_DOZE;
				led_set(false);
				hfclk_off();
				set_poll_rate(st, rate.fast);
			}

			if (now >= next_print) {
				next_print += 1000;
				print_stats(&pad, st, rate.fast, link.up);
			}
		}

		k_timer_stop(&poll_timer);
		printk("Controller lost (%d bad polls in a row), re-detecting\n", PAD_LOST_POLLS);
	}
}
