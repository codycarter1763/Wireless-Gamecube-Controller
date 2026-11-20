/**
 * joybus_target.c - Joybus target for nRF52840 (receiver board pretends to
 * be the controller).
 *
 * Why this differs from the ESP32 version
 * ---------------------------------------
 * On the ESP32, a whole core busy-polls the line with interrupts off. Here we
 * have one core, and ESB has a hard interrupt deadline: when a packet arrives,
 * its radio ISR must set up the ACK before the radio finishes turning around
 * (130 us with normal ramp-up). So the CPU may only be held for well under
 * 130 us at a time. The work is split:
 *
 *   1. Detect:  a GPIOTE channel in event mode fires IN[n] on the command's
 *               first falling edge. PPI routes it, with no CPU involved, to
 *               - TIMER3 CAPTURE[0], and disables its own channel, so CC[0]
 *                 keeps the FIRST edge even if the ISR starts a little late
 *               - EGU5 TRIGGER[0], which raises our zero-latency ISR
 *   2. Receive: the ISR (running from RAM) measures each low pulse of the
 *               command and its stop bit, with interrupts masked (~121 us
 *               for a 3-byte poll at the console's 5 us bit time).
 *   3. Reply:   PWM1 plays a pre-encoded reply out of RAM via EasyDMA, one PWM
 *               period (4 us) per Joybus bit. The ISR just starts it and
 *               returns, so the radio is free during the ~270 us reply.
 *   4. Re-arm:  when the PWM stops, its ISR turns the GPIOTE event back on.
 *
 * The GPIOTE channel is only in event mode while waiting for a command, and
 * is switched off before the PWM drives the pin, so the two peripherals never
 * own the pin at the same time.
 *
 * Lessons from bring-up, so they don't get undone:
 *   - GPIO SENSE/PORT is a low-power wake-up path and fired ~10 us late.
 *     GPIOTE IN events are synchronized to 16 MHz and fire on time.
 *   - From flash, this ISR's code is usually out of the instruction cache
 *     (it runs every few ms) and the misses cost ~1.7 us. It runs from RAM.
 *   - Waking from WFI delays the ISR; the receiver keeps the CPU awake
 *     (CONFIG_GC_RX_NO_SLEEP) and in constant-latency mode.
 *   - The console's bits are 5 us long and the line rings. Bits are decoded
 *     by low-pulse WIDTH, and the edge loop does nothing but read the pin and
 *     the cycle counter: extra work between bits (~1.5 us at one point) made
 *     every next edge be seen late and dropped short 1 bits.
 *
 * Reply buffers are encoded ahead of time by joybus_target_set_state()
 * (called from the ESB ISR on every new input), triple-buffered so the
 * buffer the PWM is reading is never rewritten.
 *
 * Electrical: the console provides the pull-up. The pin is open drain;
 * the PWM output goes through the pin's S0D1 drive setting, so a PWM "high"
 * just releases the line.
 */
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <helpers/nrfx_gppi.h>
#include <nrfx_gpiote.h>
#include <zephyr/drivers/gpio/gpio_nrf.h>
#include "joybus.h"
#include "joybus_priv.h"

/* Bit decoding: a console 1 is low ~1.1-1.6 us, a 0 is low ~3.1-4.2 us.
 * Pulses shorter than GLITCH are ringing and are ignored. If the ISR starts
 * after the first bit's low has already ended (line high on entry), that bit
 * must have been a 1 - as long as we got here before FIRST_LATEST, when even
 * a 0 could have ended. */
#define CC_PER_US          64U                      /* nRF52840: 64 MHz */
#define BIT_THRESH_CC      (23U * CC_PER_US / 10U)  /* < 2.3 us low = 1 */
#define GLITCH_CC          (25U * CC_PER_US / 100U) /* < 0.25 us = ringing */
#define FIRST_LATEST_CC    (29U * CC_PER_US / 10U)
#define LOW_TIMEOUT_CC     (8U * CC_PER_US)         /* max wait for an edge */
#define REPLY_DELAY_CC     (2U * CC_PER_US)         /* stop bit -> reply */

BUILD_ASSERT(CC_PER_US == 64U, "nRF52840 core clock is 64 MHz");

/* PWM: 16 MHz, 64 ticks = 4 us per Joybus bit. With bit 15 clear, each
 * value means "low for N ticks, then high" (Zephyr pwm_nrfx: inverted). */
#define PWM_TOP            64
#define PWM_ONE            16     /* 1 us low, 3 us high */
#define PWM_ZERO           48     /* 3 us low, 1 us high */
#define PWM_STOP           32     /* controller stop bit: 2 us low */
#define PWM_IDLE           0      /* high for the whole period */

#define VALS(nbytes)       ((nbytes) * 8 + 2)   /* bits + stop + idle */

#define JB_PWM             NRF_PWM1
#define JB_PWM_IRQn        PWM1_IRQn
#define JB_EGU             NRF_EGU5
#define JB_EGU_IRQn        SWI5_EGU5_IRQn
#define JB_TIMER           NRF_TIMER3

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */

struct reply_set {
	uint16_t poll[VALS(sizeof(GC_Payload))];
	uint16_t origin[VALS(sizeof(GC_Origin))];
};

static struct reply_set s_sets[3];
static uint16_t s_id_reply[VALS(3)];
static volatile uint8_t s_pub;           /* newest complete set */
static volatile uint8_t s_play = 0xFF;   /* set the PWM is reading, or 0xFF */

static volatile uint8_t s_rumble;
static volatile bool s_rearm_pending;
static uint8_t s_pin;
static uint8_t s_gpiote_ch;
static nrfx_gppi_group_handle_t s_grp;   /* holds the timestamp channel */

static struct joybus_target_stats s_stats;

/* ------------------------------------------------------------------ */
/* Reply buffers                                                       */
/* ------------------------------------------------------------------ */

static void encode(uint16_t *dst, const uint8_t *src, int n)
{
	for (int i = 0; i < n; i++) {
		uint8_t v = src[i];

		for (int b = 0; b < 8; b++, v <<= 1) {
			*dst++ = (v & 0x80) ? PWM_ONE : PWM_ZERO;
		}
	}
	*dst++ = PWM_STOP;
	*dst = PWM_IDLE;
}

void joybus_target_set_state(const GC_Payload *in, const GC_Origin *origin)
{
	/* irq_lock keeps two kernel-level writers apart; it does not block the
	 * zero-latency ISR, which is fine: we never write the set it may pick
	 * (s_pub) or the one the PWM is playing (s_play). */
	unsigned int key = irq_lock();
	uint8_t pub = s_pub;
	uint8_t play = s_play;
	uint8_t idx = 0;

	while (idx == pub || idx == play) {
		idx++;
	}
	encode(s_sets[idx].poll, (const uint8_t *)in, sizeof(*in));
	encode(s_sets[idx].origin, (const uint8_t *)origin, sizeof(*origin));
	__DMB();
	s_pub = idx;
	irq_unlock(key);
}

uint8_t joybus_target_rumble(void)
{
	return s_rumble;
}

void joybus_target_get_stats(struct joybus_target_stats *out, bool reset)
{
	*out = s_stats;
	if (reset) {
		memset(&s_stats, 0, sizeof(s_stats));
	}
}

/* ------------------------------------------------------------------ */
/* Arm / disarm the start-of-command trigger                           */
/* ------------------------------------------------------------------ */

static ALWAYS_INLINE void disarm(void)
{
	NRF_GPIOTE->CONFIG[s_gpiote_ch] = GPIOTE_CONFIG_MODE_Disabled << GPIOTE_CONFIG_MODE_Pos;
	NRF_GPIOTE->EVENTS_IN[s_gpiote_ch] = 0;
}

/* Arm only while the line is idle high; otherwise the 1 ms service timer
 * retries. Never called from the zero-latency ISR. */
static void rearm(void)
{
	unsigned int key = irq_lock();

	if (JB_LINE_HIGH()) {
		/* Re-enable the timestamp channel; the first falling edge of the
		 * next command disables it again, so CC[0] keeps that edge. */
		nrfx_gppi_group_enable(s_grp);
		NRF_GPIOTE->EVENTS_IN[s_gpiote_ch] = 0;
		NRF_GPIOTE->CONFIG[s_gpiote_ch] =
			(GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
			((uint32_t)s_pin << GPIOTE_CONFIG_PSEL_Pos) |
			(0UL << GPIOTE_CONFIG_PORT_Pos) |
			(GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos);
		s_rearm_pending = false;
	} else {
		s_rearm_pending = true;
	}
	irq_unlock(key);
}

static void service_fn(struct k_timer *t)
{
	ARG_UNUSED(t);
	if (s_rearm_pending && s_play == 0xFF) {
		rearm();
	}
}
static K_TIMER_DEFINE(service_timer, service_fn, NULL);

/* ------------------------------------------------------------------ */
/* Command receive (zero-latency ISR, runs from RAM)                   */
/* ------------------------------------------------------------------ */

/* Lean edge wait: a pin read, a cycle-counter read and a compare. */
static ALWAYS_INLINE bool wait_lvl(bool high, uint32_t start)
{
	while (JB_LINE_HIGH() != high) {
		if ((uint32_t)(DWT->CYCCNT - start) > LOW_TIMEOUT_CC) {
			return false;
		}
	}
	return true;
}

/* Measure the next real low pulse: width in cycles (0 on timeout) and the
 * time of its rise. A short low after a rise (ringing) is skipped; a short
 * high inside a low just restarts the fall time a little later. */
static ALWAYS_INLINE uint32_t next_low(uint32_t *t_rise)
{
	uint32_t start = DWT->CYCCNT;

	for (;;) {
		if (!wait_lvl(false, start)) {
			return 0;
		}
		uint32_t tf = DWT->CYCCNT;

		if (!wait_lvl(true, tf)) {
			return 0;
		}
		uint32_t tr = DWT->CYCCNT;
		uint32_t w = tr - tf;

		if (w >= GLITCH_CC) {
			*t_rise = tr;
			return w;
		}
	}
}

static ALWAYS_INLINE int cmd_length(uint8_t c)
{
	if (c == GC_CMD_ID || c == GC_CMD_ORIGIN || c == GC_CMD_RESET) {
		return 1;
	}
	if (c == GC_CMD_POLL || c == GC_CMD_RECALIBRATE) {
		return 3;
	}
	return -1;
}

static ALWAYS_INLINE void start_reply(uint16_t *vals, uint16_t count)
{
	JB_PWM->EVENTS_STOPPED = 0;
	JB_PWM->EVENTS_SEQEND[0] = 0;
	JB_PWM->SEQ[0].PTR = (uint32_t)vals;
	JB_PWM->SEQ[0].CNT = count;
	JB_PWM->TASKS_SEQSTART[0] = 1;
}

/*
 * A plain void function works as a direct, zero-latency ISR (no kernel calls,
 * no rescheduling). Every helper it uses is ALWAYS_INLINE, so it all lives in
 * RAM with it.
 */
static __ramfunc void jb_target_isr(void)
{
	uint32_t key = jb_crit_enter();
	uint8_t cmd[3];
	uint32_t tr;
	int len;

	JB_EGU->EVENTS_TRIGGERED[0] = 0;
	disarm();

	/* Where was the first falling edge, in CPU cycles? TIMER3 runs at
	 * 16 MHz (4 CPU cycles per tick) and captured it in CC[0] via PPI. */
	JB_TIMER->TASKS_CAPTURE[1] = 1;
	uint32_t ago_cc = (JB_TIMER->CC[1] - JB_TIMER->CC[0]) * 4;
	uint32_t t0 = DWT->CYCCNT - ago_cc;

	/* First bit, before anything else. If the line is still low, time the
	 * low from the captured edge; if it's already high, it was a short
	 * low, i.e. a 1 (only valid if we're not too late). */
	uint8_t v;

	if (JB_LINE_HIGH()) {
		if (ago_cc > FIRST_LATEST_CC) {
			s_stats.late++;
			goto fail;
		}
		v = 1;
	} else {
		if (!wait_lvl(true, t0)) {
			s_stats.errors++;
			goto fail;
		}
		v = ((DWT->CYCCNT - t0) < BIT_THRESH_CC);
	}

	for (int b = 1; b < 8; b++) {
		uint32_t w = next_low(&tr);

		if (w == 0) {
			s_stats.errors++;
			goto fail;
		}
		v = (uint8_t)((v << 1) | (w < BIT_THRESH_CC));
	}
	cmd[0] = v;

	len = cmd_length(cmd[0]);
	if (len < 0) {
		s_stats.unknown++;
		goto fail;
	}
	for (int i = 1; i < len; i++) {
		v = 0;
		for (int b = 0; b < 8; b++) {
			uint32_t w = next_low(&tr);

			if (w == 0) {
				s_stats.errors++;
				goto fail;
			}
			v = (uint8_t)((v << 1) | (w < BIT_THRESH_CC));
		}
		cmd[i] = v;
	}

	/* Console stop bit: one more short low, then the line goes idle. If no
	 * stop low shows up but the line is idle high, the command bits were
	 * all good, so reply anyway. */
	uint32_t t_stop;
	uint32_t sstart = DWT->CYCCNT;

	if (!wait_lvl(false, sstart)) {
		if (!JB_LINE_HIGH()) {
			s_stats.errors++;
			goto fail;
		}
		t_stop = DWT->CYCCNT;
	} else {
		uint32_t tf = DWT->CYCCNT;

		if (!wait_lvl(true, tf)) {
			s_stats.errors++;
			goto fail;
		}
		t_stop = DWT->CYCCNT;
	}

	/* Pick the reply. */
	uint16_t *vals;
	uint16_t count;
	uint8_t set = s_pub;

	switch (cmd[0]) {
	case GC_CMD_POLL:
		vals = s_sets[set].poll;
		count = ARRAY_SIZE(s_sets[set].poll);
		s_rumble = cmd[2] & GC_RUMBLE_BIT;
		s_stats.polls++;
		break;
	case GC_CMD_ORIGIN:
	case GC_CMD_RECALIBRATE:
		vals = s_sets[set].origin;
		count = ARRAY_SIZE(s_sets[set].origin);
		s_stats.origins++;
		break;
	default: /* GC_CMD_ID, GC_CMD_RESET */
		vals = s_id_reply;
		count = ARRAY_SIZE(s_id_reply);
		s_stats.ids++;
		break;
	}
	s_play = set;

	jb_wait_until(t_stop + REPLY_DELAY_CC);
	start_reply(vals, count);
	/* PWM1's STOPPED interrupt re-arms once the reply is out. */
	jb_crit_exit(key);
	return;

fail:
	/* Garbage or a command we don't know: no reply. The service timer
	 * re-arms once the line is idle. */
	s_rearm_pending = true;
	jb_crit_exit(key);
}

static void pwm_isr(const void *arg)
{
	ARG_UNUSED(arg);
	if (JB_PWM->EVENTS_STOPPED) {
		JB_PWM->EVENTS_STOPPED = 0;
		s_play = 0xFF;
		rearm();
	}
}

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

int joybus_target_init(uint8_t p0_pin)
{
	static const uint8_t id[3] = { GC_ID_BYTE0, GC_ID_BYTE1, GC_ID_BYTE2 };
	const GC_Payload neutral = {
		.byte1 = GC_BYTE1_ALWAYS_ONE,
		.joy_X_val = 128, .joy_Y_val = 128,
		.C_stick_X_val = 128, .C_stick_Y_val = 128,
	};
	const GC_Origin neutral_origin = { .state = neutral };
	nrfx_gppi_handle_t ppi_ts, ppi_irq;
	int err;

	err = jb_pin_init(p0_pin);
	if (err) {
		return err;
	}
	s_pin = p0_pin;

	/* Constant-latency mode: keeps the CPU's clock and regulators ready
	 * while idle, so an interrupt starts quickly. Costs some idle current,
	 * which doesn't matter on the console-powered receiver. */
	NRF_POWER->TASKS_CONSTLAT = 1;

	/* Replies: ID is fixed; poll/origin start neutral until the
	 * controller board sends real data. */
	encode(s_id_reply, id, sizeof(id));
	joybus_target_set_state(&neutral, &neutral_origin);

	/* Borrow one GPIOTE channel from Zephyr's GPIO driver, so its own
	 * channel use can't collide with ours. Its interrupt stays off; the
	 * event only feeds PPI. */
	nrfx_gpiote_t *gpiote = gpio_nrf_gpiote_by_port_get(DEVICE_DT_GET(DT_NODELABEL(gpio0)));

	err = nrfx_gpiote_channel_alloc(gpiote, &s_gpiote_ch);
	if (err) {
		return err;
	}
	NRF_GPIOTE->INTENCLR = BIT(s_gpiote_ch);
	disarm();

	/* TIMER3: free-running 16 MHz timestamp counter. */
	JB_TIMER->TASKS_STOP = 1;
	JB_TIMER->MODE = TIMER_MODE_MODE_Timer;
	JB_TIMER->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
	JB_TIMER->PRESCALER = 0;
	JB_TIMER->TASKS_CLEAR = 1;
	JB_TIMER->TASKS_START = 1;

	/* PPI, all hardware:
	 *   timestamp: IN -> TIMER3 CAPTURE[0], fork -> disable its own group
	 *   irq:       IN -> EGU5 TRIGGER[0] (raises the ISR) */
	uint32_t ev = (uint32_t)&NRF_GPIOTE->EVENTS_IN[s_gpiote_ch];

	err = nrfx_gppi_group_alloc(0, &s_grp);
	if (err) {
		return err;
	}
	err = nrfx_gppi_conn_alloc(ev, (uint32_t)&JB_TIMER->TASKS_CAPTURE[0], &ppi_ts);
	if (err) {
		return err;
	}
	err = nrfx_gppi_ep_attach(nrfx_gppi_group_task_dis_addr(s_grp), ppi_ts);
	if (err) {
		return err;
	}
	nrfx_gppi_group_ch_add(s_grp, ppi_ts);
	nrfx_gppi_conn_enable(ppi_ts);

	err = nrfx_gppi_conn_alloc(ev, (uint32_t)&JB_EGU->TASKS_TRIGGER[0], &ppi_irq);
	if (err) {
		return err;
	}
	nrfx_gppi_conn_enable(ppi_irq);

	/* EGU5 -> zero-latency ISR. */
	JB_EGU->EVENTS_TRIGGERED[0] = 0;
	JB_EGU->INTENSET = EGU_INTENSET_TRIGGERED0_Msk;
	IRQ_DIRECT_CONNECT(SWI5_EGU5_IRQn, 0, jb_target_isr, IRQ_ZERO_LATENCY);
	irq_enable(JB_EGU_IRQn);

	/* PWM1: reply generator on the data pin. */
	JB_PWM->ENABLE = 0;
	JB_PWM->PSEL.OUT[0] = p0_pin;               /* port 0, connected */
	JB_PWM->PSEL.OUT[1] = 0xFFFFFFFF;
	JB_PWM->PSEL.OUT[2] = 0xFFFFFFFF;
	JB_PWM->PSEL.OUT[3] = 0xFFFFFFFF;
	JB_PWM->MODE = PWM_MODE_UPDOWN_Up;
	JB_PWM->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_1;  /* 16 MHz */
	JB_PWM->COUNTERTOP = PWM_TOP;
	JB_PWM->LOOP = 0;
	JB_PWM->DECODER = (PWM_DECODER_LOAD_Common << PWM_DECODER_LOAD_Pos) |
			  (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
	JB_PWM->SEQ[0].REFRESH = 0;
	JB_PWM->SEQ[0].ENDDELAY = 0;
	JB_PWM->SEQ[1].PTR = (uint32_t)s_id_reply;    /* unused, but valid */
	JB_PWM->SEQ[1].CNT = 0;
	/* SEQEND fires when the last value (the idle period) starts; STOP
	 * then takes effect at the end of that period. */
	JB_PWM->SHORTS = PWM_SHORTS_SEQEND0_STOP_Msk;
	JB_PWM->INTENSET = PWM_INTENSET_STOPPED_Msk;
	JB_PWM->ENABLE = 1;
	IRQ_CONNECT(PWM1_IRQn, 2, pwm_isr, NULL, 0);
	irq_enable(JB_PWM_IRQn);

	k_timer_start(&service_timer, K_MSEC(1), K_MSEC(1));
	rearm();
	return 0;
}
