/*
 * Pure decision logic shared by the controller (ptx.c) and receiver (prx.c).
 *
 * Nothing in here touches hardware, Zephyr kernel objects or the clock:
 * callers pass in "now" and the current input, and get back a decision.
 * That keeps it unit-testable on a PC (tests/unit, native_sim).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "gc_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Input comparison                                                    */
/* ------------------------------------------------------------------ */

/* Any analog value (sticks, triggers) differs by at least `thresh`. */
bool gc_analog_moved(const GC_Payload *a, const GC_Payload *b, int thresh);

/* Any real button differs (status bits like ALWAYS_ONE are ignored). */
bool gc_buttons_changed(const GC_Payload *a, const GC_Payload *b);

/* Raw button/status bytes differ, or analog moved by at least `thresh`. */
bool gc_input_changed(const GC_Payload *a, const GC_Payload *b, int thresh);

/* Neutral input for a given origin: origin sticks/triggers, no buttons. */
GC_Payload gc_neutral_input(const GC_Origin *origin);

/* ------------------------------------------------------------------ */
/* Adaptive poll rate (controller)                                     */
/* ------------------------------------------------------------------ */

struct gc_rate {
	GC_Payload ref;        /* input at the last real change */
	int64_t last_change;   /* ms */
	bool fast;
};

/* Start fast, with `ref` as the resting reference. */
void gc_rate_init(struct gc_rate *r, const GC_Payload *ref, int64_t now);

/* Feed one poll. A change of at least `thresh` from the reference (not from
 * the previous poll, so noise around a resting value never counts) goes
 * fast; `hold_ms` without one goes slow. Returns true if `fast` changed. */
bool gc_rate_update(struct gc_rate *r, const GC_Payload *pad, int64_t now,
		    int thresh, int64_t hold_ms);

/* ------------------------------------------------------------------ */
/* Link state (controller)                                             */
/* ------------------------------------------------------------------ */

struct gc_link {
	bool up;
	int64_t last_ack;      /* ms of the last acknowledged packet */
	int64_t lost_at;       /* ms the link was declared lost */
	int64_t last_probe;    /* ms of the last send while lost */
};

enum gc_link_event {
	GC_LINK_NONE,
	GC_LINK_LOST,          /* just declared lost */
	GC_LINK_RESTORED,      /* first ACK after being lost */
};

/* Starts "up", as if acknowledged at `now`. */
void gc_link_init(struct gc_link *l, int64_t now);

/* Should we send this tick? Linked: on change, rumble, or keepalive due.
 * Lost: only a probe every `probe_ms`. */
bool gc_link_want_send(const struct gc_link *l, int64_t now, bool changed,
		       bool rumble, int64_t last_send, int64_t keepalive_ms,
		       int64_t probe_ms);

/* Record the result of a send. Lost after `loss_ms` with no ACK at all. */
enum gc_link_event gc_link_on_send(struct gc_link *l, int64_t now, bool acked,
				   int64_t loss_ms);

/* ------------------------------------------------------------------ */
/* Sequence filter (receiver)                                          */
/* ------------------------------------------------------------------ */

/* Packets this far behind in a row -> assume the controller restarted. */
#define GC_SEQ_BEHIND_RESYNC 8

struct gc_seq {
	uint8_t expected;
	bool have;
	uint8_t behind_run;    /* consecutive packets behind `expected` */
};

enum gc_seq_result {
	GC_SEQ_ACCEPT,
	GC_SEQ_DUP,            /* behind expected: duplicate or out of order */
};

void gc_seq_init(struct gc_seq *s);

/* Classify one received packet. On ACCEPT, *missed gets the number of
 * sequence numbers skipped (packets the controller gave up on); on DUP it
 * is set to 0. Origin packets always resync, because the controller sends
 * one first after every (re)start, when its count starts over. */
enum gc_seq_result gc_seq_check(struct gc_seq *s, uint8_t seq, bool is_origin,
				uint32_t *missed);

/* ------------------------------------------------------------------ */
/* OTA combo (controller): L + R + Z + A                               */
/* ------------------------------------------------------------------ */

bool gc_ota_combo_pressed(const GC_Payload *pad);

/* Track how long the combo has been held. `*since` is -1 when not held.
 * Returns the hold time in ms, or -1 if the combo isn't held right now. */
int64_t gc_combo_hold(int64_t *since, bool pressed, int64_t now);

#ifdef __cplusplus
}
#endif
