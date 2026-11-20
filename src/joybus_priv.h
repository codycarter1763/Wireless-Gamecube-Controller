#pragma once
/**
 * joybus_priv.h - bit-timing helpers shared by joybus.c (host) and
 * joybus_target.c (target). Each .c file that includes this gets its own
 * copy of jb_mask / jb_cc_us and must call jb_pin_init().
 *
 * Timing base is the Cortex-M4 DWT cycle counter (64 MHz on nRF52840).
 */
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <hal/nrf_gpio.h>
#include <nrfx.h>

#define JB_BIT_TIMEOUT_US  10     /* max time for one bit's edges */

static uint32_t jb_mask;     /* bit mask within NRF_P0 */
static uint32_t jb_cc_us;    /* CPU cycles per microsecond */

#define JB_LINE_LOW()   (NRF_P0->OUTCLR = jb_mask)
#define JB_LINE_REL()   (NRF_P0->OUTSET = jb_mask)
#define JB_LINE_HIGH()  ((NRF_P0->IN & jb_mask) != 0)

static ALWAYS_INLINE uint32_t jb_now(void)
{
	return DWT->CYCCNT;
}

static ALWAYS_INLINE void jb_wait_until(uint32_t t)
{
	while ((int32_t)(jb_now() - t) < 0) {
	}
}

/* Wait for the line to reach `high`. False if `timeout_cc` passes since `start`. */
static ALWAYS_INLINE bool jb_wait_level(bool high, uint32_t start, uint32_t timeout_cc)
{
	while (JB_LINE_HIGH() != high) {
		if ((uint32_t)(jb_now() - start) > timeout_cc) {
			return false;
		}
	}
	return true;
}

/* Wait for the end of the current low, then the next falling edge, and
 * sample 2 us later. Returns 0/1, or -1 on timeout. */
static ALWAYS_INLINE int jb_read_bit(uint32_t timeout_cc)
{
	uint32_t start = jb_now();

	if (!jb_wait_level(true, start, timeout_cc)) {
		return -1;
	}
	if (!jb_wait_level(false, start, timeout_cc)) {
		return -1;
	}
	jb_wait_until(jb_now() + 2 * jb_cc_us);
	return JB_LINE_HIGH() ? 1 : 0;
}

/* Mask *all* interrupts, including Zephyr zero-latency IRQs that irq_lock()
 * leaves enabled. */
static ALWAYS_INLINE uint32_t jb_crit_enter(void)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	return primask;
}

static ALWAYS_INLINE void jb_crit_exit(uint32_t primask)
{
	__set_PRIMASK(primask);
}

/* Start the DWT cycle counter and set the pin to open drain, released. */
static inline int jb_pin_init(uint8_t p0_pin)
{
	if (p0_pin > 31) {
		return -EINVAL;
	}
	jb_mask = BIT(p0_pin);
	jb_cc_us = SystemCoreClock / 1000000U;   /* 64 on nRF52840 */

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	/* Release first so the line doesn't glitch low while we switch modes. */
	NRF_P0->OUTSET = jb_mask;
	nrf_gpio_cfg(NRF_GPIO_PIN_MAP(0, p0_pin),
		     NRF_GPIO_PIN_DIR_OUTPUT,
		     NRF_GPIO_PIN_INPUT_CONNECT,
		     NRF_GPIO_PIN_PULLUP,        /* weak; real pull-up is external/console */
		     NRF_GPIO_PIN_S0D1,          /* open drain */
		     NRF_GPIO_PIN_NOSENSE);
	return 0;
}
