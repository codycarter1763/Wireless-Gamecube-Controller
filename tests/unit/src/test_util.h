#pragma once
#include "gc_protocol.h"

/* A pad at rest: no buttons, sticks centred, triggers released. */
static inline GC_Payload pad_rest(void)
{
	return (GC_Payload){
		.byte1 = GC_BYTE1_ALWAYS_ONE,
		.joy_X_val = 128, .joy_Y_val = 128,
		.C_stick_X_val = 128, .C_stick_Y_val = 128,
		.left_trigger_val = 20, .right_trigger_val = 20,
	};
}

/* Same, with buttons pressed (byte0 mask, byte1 mask). */
static inline GC_Payload pad_buttons(uint8_t b0, uint8_t b1)
{
	GC_Payload p = pad_rest();

	p.byte0 |= b0;
	p.byte1 |= b1;
	return p;
}

/* Analog axes in wire order, indexable 0..5. */
static inline uint8_t *pad_axis(GC_Payload *p, int i)
{
	return &p->joy_X_val + i;
}
