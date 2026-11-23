/* Input comparison, neutral input and the protocol's payload sanity check. */
#include <zephyr/ztest.h>
#include "gc_logic.h"
#include "test_util.h"

ZTEST_SUITE(input, NULL, NULL, NULL, NULL, NULL);

ZTEST(input, test_identical_is_unchanged)
{
	GC_Payload a = pad_rest(), b = pad_rest();

	zassert_false(gc_analog_moved(&a, &b, 1));
	zassert_false(gc_buttons_changed(&a, &b));
	zassert_false(gc_input_changed(&a, &b, 1));
}

ZTEST(input, test_analog_threshold_boundary_every_axis)
{
	for (int axis = 0; axis < 6; axis++) {
		for (int dir = -1; dir <= 1; dir += 2) {
			GC_Payload a = pad_rest(), b = pad_rest();

			*pad_axis(&b, axis) = (uint8_t)(*pad_axis(&a, axis) + dir * 2);
			zassert_false(gc_analog_moved(&a, &b, 3),
				      "axis %d dir %d: 2 < thresh 3", axis, dir);

			*pad_axis(&b, axis) = (uint8_t)(*pad_axis(&a, axis) + dir * 3);
			zassert_true(gc_analog_moved(&a, &b, 3),
				     "axis %d dir %d: 3 >= thresh 3", axis, dir);
		}
	}
}

ZTEST(input, test_analog_full_range_no_overflow)
{
	GC_Payload a = pad_rest(), b = pad_rest();

	a.joy_X_val = 0;
	b.joy_X_val = 255;
	zassert_true(gc_analog_moved(&a, &b, 255));
	zassert_true(gc_analog_moved(&b, &a, 255));
}

ZTEST(input, test_every_button_detected)
{
	const uint8_t b0[] = { START_BTN, Y_BTN, X_BTN, B_BTN, A_BTN };
	const uint8_t b1[] = { L_BTN, R_BTN, Z_BTN, D_UP_BTN, D_DOWN_BTN,
			       D_RIGHT_BTN, D_LEFT_BTN };
	GC_Payload rest = pad_rest();

	for (size_t i = 0; i < ARRAY_SIZE(b0); i++) {
		GC_Payload p = pad_buttons(b0[i], 0);

		zassert_true(gc_buttons_changed(&rest, &p), "byte0 bit 0x%02x", b0[i]);
	}
	for (size_t i = 0; i < ARRAY_SIZE(b1); i++) {
		GC_Payload p = pad_buttons(0, b1[i]);

		zassert_true(gc_buttons_changed(&rest, &p), "byte1 bit 0x%02x", b1[i]);
	}
}

ZTEST(input, test_status_bits_are_not_buttons)
{
	GC_Payload a = pad_rest(), b = pad_rest();

	b.byte0 |= GC_BYTE0_NEED_ORIGIN;
	b.byte1 &= ~GC_BYTE1_ALWAYS_ONE;
	zassert_false(gc_buttons_changed(&a, &b),
		      "status bits don't count as activity");
	zassert_true(gc_input_changed(&a, &b, 1),
		     "but they are still a change worth sending");
}

ZTEST(input, test_neutral_input_from_origin)
{
	GC_Origin o = { .state = pad_buttons(A_BTN | START_BTN, L_BTN) };

	o.state.joy_X_val = 120;
	o.state.right_trigger_val = 33;

	GC_Payload n = gc_neutral_input(&o);

	zassert_equal(n.byte0, 0, "no buttons");
	zassert_equal(n.byte1, GC_BYTE1_ALWAYS_ONE, "only the always-one bit");
	zassert_equal(n.joy_X_val, 120, "sticks from the origin");
	zassert_equal(n.right_trigger_val, 33, "triggers from the origin");
}

ZTEST(input, test_payload_valid)
{
	GC_Payload p = pad_rest();

	zassert_true(gc_payload_valid(&p));

	p.byte1 &= ~GC_BYTE1_ALWAYS_ONE;
	zassert_false(gc_payload_valid(&p), "always-one bit missing");

	p = pad_buttons(0, D_UP_BTN | D_DOWN_BTN);
	zassert_false(gc_payload_valid(&p), "up + down at once");

	p = pad_buttons(0, D_LEFT_BTN | D_RIGHT_BTN);
	zassert_false(gc_payload_valid(&p), "left + right at once");

	p = pad_buttons(0, D_UP_BTN | D_LEFT_BTN);
	zassert_true(gc_payload_valid(&p), "diagonal is fine");
}
