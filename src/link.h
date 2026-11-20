/*
 * Glue shared by main.c, radio.c and the role files (ptx.c / prx.c).
 * Packet formats live in gc_protocol.h.
 */
#ifndef LINK_H_
#define LINK_H_

#include <stdbool.h>
#include <stdint.h>
#include "gc_protocol.h"

/* radio.c; app_esb_handler() is defined by the role file (ptx.c / prx.c) */
struct esb_evt;
void app_esb_handler(const struct esb_evt *event);
int radio_init(bool prx);

/* main.c helpers */
void led_toggle(void);
void dbg_pin_set(bool on);
uint32_t us_now(void);      /* thread context */
uint32_t us_now_isr(void);  /* ESB event (ISR) context */

/* Role entry point: runs forever, prints stats once a second. */
void role_run(void);

/* Receiver side (prx.c): latest state received from the controller board. */
bool gc_rx_get_input(GC_Payload *out);    /* false if none or stale (>100 ms) */
bool gc_rx_get_origin(GC_Origin *out);    /* false if none received yet */

#endif
