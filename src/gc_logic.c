/* Pure decision logic, see gc_logic.h. No hardware, no kernel calls. */
#include <stdlib.h>
#include "gc_logic.h"

#define BYTE0_BUTTONS (START_BTN | Y_BTN | X_BTN | B_BTN | A_BTN)
#define BYTE1_BUTTONS (L_BTN | R_BTN | Z_BTN | D_UP_BTN | D_DOWN_BTN | \
		       D_RIGHT_BTN | D_LEFT_BTN)

#define OTA_BYTE0_MASK (A_BTN)
#define OTA_BYTE1_MASK (L_BTN | R_BTN | Z_BTN)

/* ------------------------------------------------------------------ */
/* Input comparison                                                    */
/* ------------------------------------------------------------------ */

bool gc_analog_moved(const GC_Payload *a, const GC_Payload *b, int thresh)
{
	const uint8_t *pa = &a->joy_X_val;
	const uint8_t *pb = &b->joy_X_val;

	for (int i = 0; i < 6; i++) {
		if (abs((int)pa[i] - (int)pb[i]) >= thresh) {
			return true;
		}
	}
	return false;
}

bool gc_buttons_changed(const GC_Payload *a, const GC_Payload *b)
{
	return ((a->byte0 ^ b->byte0) & BYTE0_BUTTONS) ||
	       ((a->byte1 ^ b->byte1) & BYTE1_BUTTONS);
}

bool gc_input_changed(const GC_Payload *a, const GC_Payload *b, int thresh)
{
	return a->byte0 != b->byte0 || a->byte1 != b->byte1 ||
	       gc_analog_moved(a, b, thresh);
}

GC_Payload gc_neutral_input(const GC_Origin *origin)
{
	GC_Payload n = origin->state;

	n.byte0 = 0;
	n.byte1 = GC_BYTE1_ALWAYS_ONE;
	return n;
}

/* ------------------------------------------------------------------ */
/* Adaptive poll rate                                                  */
/* ------------------------------------------------------------------ */

void gc_rate_init(struct gc_rate *r, const GC_Payload *ref, int64_t now)
{
	r->ref = *ref;
	r->last_change = now;
	r->fast = true;
}

bool gc_rate_update(struct gc_rate *r, const GC_Payload *pad, int64_t now,
		    int thresh, int64_t hold_ms)
{
	if (gc_input_changed(pad, &r->ref, thresh)) {
		r->ref = *pad;
		r->last_change = now;
		if (!r->fast) {
			r->fast = true;
			return true;
		}
	} else if (r->fast && now - r->last_change >= hold_ms) {
		r->fast = false;
		return true;
	}
	return false;
}

/* ------------------------------------------------------------------ */
/* Link state                                                          */
/* ------------------------------------------------------------------ */

void gc_link_init(struct gc_link *l, int64_t now)
{
	l->up = true;
	l->last_ack = now;
	l->lost_at = 0;
	l->last_probe = 0;
}

bool gc_link_want_send(const struct gc_link *l, int64_t now, bool changed,
		       bool rumble, int64_t last_send, int64_t keepalive_ms,
		       int64_t probe_ms)
{
	if (l->up) {
		return changed || rumble || now - last_send >= keepalive_ms;
	}
	return now - l->last_probe >= probe_ms;
}

enum gc_link_event gc_link_on_send(struct gc_link *l, int64_t now, bool acked,
				   int64_t loss_ms)
{
	l->last_probe = now;

	if (acked) {
		l->last_ack = now;
		if (!l->up) {
			l->up = true;
			return GC_LINK_RESTORED;
		}
	} else if (l->up && now - l->last_ack >= loss_ms) {
		l->up = false;
		l->lost_at = now;
		return GC_LINK_LOST;
	}
	return GC_LINK_NONE;
}

/* ------------------------------------------------------------------ */
/* Sequence filter                                                     */
/* ------------------------------------------------------------------ */

void gc_seq_init(struct gc_seq *s)
{
	s->expected = 0;
	s->have = false;
	s->behind_run = 0;
}

enum gc_seq_result gc_seq_check(struct gc_seq *s, uint8_t seq, bool is_origin,
				uint32_t *missed)
{
	/* Distance ahead of what we expect, modulo 256. 128..255 = behind. */
	uint8_t gap = (uint8_t)(seq - s->expected);
	bool behind = s->have && gap >= 128;

	*missed = 0;
	if (behind && !is_origin && ++s->behind_run < GC_SEQ_BEHIND_RESYNC) {
		return GC_SEQ_DUP;
	}
	if (s->have && !behind) {
		*missed = gap;
	}
	s->behind_run = 0;
	s->expected = (uint8_t)(seq + 1);
	s->have = true;
	return GC_SEQ_ACCEPT;
}

/* ------------------------------------------------------------------ */
/* OTA combo                                                           */
/* ------------------------------------------------------------------ */

bool gc_ota_combo_pressed(const GC_Payload *pad)
{
	return (pad->byte0 & OTA_BYTE0_MASK) == OTA_BYTE0_MASK &&
	       (pad->byte1 & OTA_BYTE1_MASK) == OTA_BYTE1_MASK;
}

int64_t gc_combo_hold(int64_t *since, bool pressed, int64_t now)
{
	if (!pressed) {
		*since = -1;
		return -1;
	}
	if (*since < 0) {
		*since = now;
	}
	return now - *since;
}
