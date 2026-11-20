/**
 * joybus.c - bit-banged Joybus host for nRF52840 (controller board).
 *
 * Same approach as the ESP32 version: busy-wait on a cycle counter with
 * interrupts masked, so every edge lands where it should. The counter only
 * runs while the CPU runs, which is fine because we never sleep mid-transfer.
 *
 * Pin handling: open drain. OUTCLR pulls the line low, OUTSET releases it to
 * the pull-up, and IN still reads the real line level so we can see the pad.
 */
#include <errno.h>
#include "joybus.h"
#include "joybus_priv.h"

#define HOST_REPLY_TIMEOUT_US  100    /* wait for the pad to start replying */
#define HOST_STOP_LOW_US       1      /* console-style stop bit */

/* ------------------------------------------------------------------ */
/* Bit-level TX / RX                                                   */
/* ------------------------------------------------------------------ */

static void send_bytes(const uint8_t *buf, int n, uint32_t stop_low_us)
{
	const uint32_t us = jb_cc_us;
	uint32_t t = jb_now();

	for (int i = 0; i < n; i++) {
		uint8_t v = buf[i];

		for (int b = 0; b < 8; b++, v <<= 1) {
			uint32_t low = (v & 0x80) ? us : 3 * us;

			JB_LINE_LOW();
			t += low;
			jb_wait_until(t);
			JB_LINE_REL();
			t += 4 * us - low;
			jb_wait_until(t);
		}
	}
	/* stop bit */
	JB_LINE_LOW();
	t += stop_low_us * us;
	jb_wait_until(t);
	JB_LINE_REL();
}

/**
 * Read n bytes, MSB first.
 * Returns n on success, -1 if the very first bit never started (idle),
 * -2 if it timed out partway through (a real error).
 */
static int read_bytes(uint8_t *buf, int n, uint32_t first_timeout_us)
{
	for (int i = 0; i < n; i++) {
		uint8_t v = 0;

		for (int b = 0; b < 8; b++) {
			bool first = (i == 0 && b == 0);
			int bit = jb_read_bit((first ? first_timeout_us : JB_BIT_TIMEOUT_US) *
					      jb_cc_us);

			if (bit < 0) {
				return first ? -1 : -2;
			}
			v = (uint8_t)((v << 1) | bit);
		}
		buf[i] = v;
	}
	return n;
}

/* ------------------------------------------------------------------ */
/* Host mode                                                           */
/* ------------------------------------------------------------------ */

int joybus_host_init(uint8_t p0_pin)
{
	return jb_pin_init(p0_pin);
}

static int host_xfer(const uint8_t *cmd, int cmd_len, uint8_t *reply, int reply_len)
{
	uint32_t key = jb_crit_enter();

	send_bytes(cmd, cmd_len, HOST_STOP_LOW_US);
	int n = read_bytes(reply, reply_len, HOST_REPLY_TIMEOUT_US);

	jb_crit_exit(key);
	return (n == reply_len) ? 0 : -ETIMEDOUT;
}

int joybus_host_read_id(uint8_t id[3])
{
	const uint8_t cmd = GC_CMD_ID;

	return host_xfer(&cmd, 1, id, 3);
}

int joybus_host_read_origin(GC_Origin *out)
{
	const uint8_t cmd = GC_CMD_ORIGIN;

	return host_xfer(&cmd, 1, (uint8_t *)out, sizeof(GC_Origin));
}

int joybus_host_poll(uint32_t cmd24, GC_Payload *out)
{
	const uint8_t cmd[3] = {
		(uint8_t)(cmd24 >> 16), (uint8_t)(cmd24 >> 8), (uint8_t)cmd24
	};

	return host_xfer(cmd, 3, (uint8_t *)out, sizeof(GC_Payload));
}
