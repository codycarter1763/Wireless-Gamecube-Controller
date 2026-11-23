/*
 * ESB setup shared by both roles. Both boards must agree on everything
 * here: address, channel (prj.conf), bitrate, protocol, ramp-up.
 */
#include <esb.h>
#include <zephyr/kernel.h>
#include "link.h"



int radio_init(bool prx)
{
	/* Private address so we don't talk to stray nRF24/ESB devices.
	 * Full address = 4-byte base + 1-byte pipe prefix. */
	static const uint8_t base_addr_0[4] = {0x47, 0x43, 0x4C, 0x4B}; /* "GCLK" */
	static const uint8_t prefixes[1] = {0xA5};

	struct esb_config config = ESB_DEFAULT_CONFIG;
	int err;

	config.protocol = ESB_PROTOCOL_ESB_DPL;   /* dynamic payload length */
	config.mode = prx ? ESB_MODE_PRX : ESB_MODE_PTX;
	config.event_handler = app_esb_handler;
	config.bitrate = ESB_BITRATE_2MBPS;
	config.crc = ESB_CRC_16BIT;
	config.tx_output_power = CONFIG_APP_TX_POWER;   /* dBm */
	config.retransmit_delay = CONFIG_APP_RETRANSMIT_DELAY_US;
	config.retransmit_count = CONFIG_APP_RETRANSMIT_COUNT;
	config.tx_mode = ESB_TXMODE_AUTO;         /* esb_write_payload() sends now */
	config.selective_auto_ack = true;
	/* Normal 130 us ramp-up, NOT fast (40 us). ESB's radio ISR must run
	 * within the ramp-up time after each packet, and on the receiver the
	 * Joybus target can hold interrupts off for ~100 us while it reads a
	 * console command. 130 us leaves room; 40 us would not. */
	config.use_fast_ramp_up = false;

	err = esb_init(&config);
	if (err) {
		return err;
	}
	err = esb_set_rf_channel(CONFIG_APP_RF_CHANNEL);
	if (err) {
		return err;
	}
	err = esb_set_base_address_0(base_addr_0);
	if (err) {
		return err;
	}
	/* One prefix = only pipe 0 enabled. */
	return esb_set_prefixes(prefixes, ARRAY_SIZE(prefixes));
}
