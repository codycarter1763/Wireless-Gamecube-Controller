/*
 * ESB link test for the wireless GameCube controller.
 * Board: promicro_nrf52840/nrf52840/uf2. Role is chosen at build time
 * (ptx.conf / prx.conf).
 */
#include <zephyr/kernel.h>
#include <zephyr/fatal.h>
#include <zephyr/drivers/gpio.h>
#include <hal/nrf_gpio.h>
#include <nrfx.h>
#include "link.h"

#define DBG_PIN NRF_GPIO_PIN_MAP(0, CONFIG_APP_DEBUG_PIN)

/* TIMER4 as a free-running 1 MHz microsecond counter for latency stats.
 * ESB itself uses TIMER2. CC[0] is read from threads, CC[1] from the ESB
 * event ISR, so the two contexts never fight over one capture register. */
#define US_TIMER NRF_TIMER4

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

void led_toggle(void)
{
	gpio_pin_toggle_dt(&led);
}

void dbg_pin_set(bool on)
{
	if (on) {
		nrf_gpio_pin_set(DBG_PIN);
	} else {
		nrf_gpio_pin_clear(DBG_PIN);
	}
}

uint32_t us_now(void)
{
	US_TIMER->TASKS_CAPTURE[0] = 1;
	return US_TIMER->CC[0];
}

uint32_t us_now_isr(void)
{
	US_TIMER->TASKS_CAPTURE[1] = 1;
	return US_TIMER->CC[1];
}

static void us_timer_init(void)
{
	US_TIMER->TASKS_STOP = 1;
	US_TIMER->MODE = TIMER_MODE_MODE_Timer;
	US_TIMER->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
	US_TIMER->PRESCALER = 4; /* 16 MHz / 2^4 = 1 MHz */
	US_TIMER->TASKS_CLEAR = 1;
	US_TIMER->TASKS_START = 1;
}

/*
 * Crash handler. USB is dead once this runs, so the serial port vanishes;
 * the LED says what happened instead: N fast blinks, a pause, repeat.
 *   1 blink  = CPU exception (bad pointer, bus fault, ...)
 *   2 blinks = spurious interrupt
 *   3 blinks = stack overflow
 *   4 blinks = kernel oops,  5 blinks = kernel panic
 * Replug USB to recover. (A quick replug can count as a double-tap reset
 * and land in the bootloader; just replug again.)
 */
void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	ARG_UNUSED(esf);
	const uint32_t pin = NRF_GPIO_PIN_MAP(0, DT_GPIO_PIN(DT_ALIAS(led0), gpios));
	unsigned int blinks = MIN(reason, 4U) + 1;

	(void)irq_lock();
	nrf_gpio_cfg_output(pin);
	for (;;) {
		for (unsigned int i = 0; i < blinks; i++) {
			nrf_gpio_pin_set(pin);
			k_busy_wait(120000);
			nrf_gpio_pin_clear(pin);
			k_busy_wait(180000);
		}
		k_busy_wait(900000);
	}
}

int main(void)
{
	int err;

	gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	nrf_gpio_cfg_output(DBG_PIN);
	dbg_pin_set(false);
	us_timer_init();

	err = radio_init(IS_ENABLED(CONFIG_APP_ROLE_PRX));
	if (err) {
		/* Fast blink forever = radio init failed. */
		while (1) {
			printk("ESB init failed: %d\n", err);
			led_toggle();
			k_msleep(100);
		}
	}

	printk("ESB link test: %s, ch %d, retx %d x %d us\n",
	       IS_ENABLED(CONFIG_APP_ROLE_PRX) ? "PRX (console side)" : "PTX (controller side)",
	       CONFIG_APP_RF_CHANNEL, CONFIG_APP_RETRANSMIT_COUNT,
	       CONFIG_APP_RETRANSMIT_DELAY_US);

	role_run();
	return 0;
}
