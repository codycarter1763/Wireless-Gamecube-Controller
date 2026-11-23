/* Adaptive poll rate: fast while input changes, slow after a quiet spell. */
#include <zephyr/ztest.h>
#include "gc_logic.h"
#include "test_util.h"

#define THRESH  3     /* CONFIG_APP_FAST_THRESH default */
#define HOLD    100   /* CONFIG_APP_FAST_HOLD_MS default */

static struct gc_rate rate;
static GC_Payload rest;

static void before(void *f)
{
	ARG_UNUSED(f);
	rest = pad_rest();
	gc_rate_init(&rate, &rest, 0);
}

ZTEST_SUITE(rate, NULL, NULL, before, NULL, NULL);

ZTEST(rate, test_starts_fast)
{
	zassert_true(rate.fast);
}

ZTEST(rate, test_goes_slow_exactly_at_hold_and_reports_once)
{
	zassert_false(gc_rate_update(&rate, &rest, HOLD - 1, THRESH, HOLD));
	zassert_true(rate.fast);

	zassert_true(gc_rate_update(&rate, &rest, HOLD, THRESH, HOLD),
		     "switch reported");
	zassert_false(rate.fast);

	zassert_false(gc_rate_update(&rate, &rest, HOLD + 50, THRESH, HOLD),
		      "already slow: no second report");
}

ZTEST(rate, test_button_press_goes_fast_immediately)
{
	gc_rate_update(&rate, &rest, HOLD, THRESH, HOLD);
	zassert_false(rate.fast);

	GC_Payload a = pad_buttons(A_BTN, 0);

	zassert_true(gc_rate_update(&rate, &a, HOLD + 1, THRESH, HOLD));
	zassert_true(rate.fast);
}

ZTEST(rate, test_change_restarts_hold_timer)
{
	GC_Payload moved = rest;

	moved.joy_X_val += THRESH;
	gc_rate_update(&rate, &moved, 80, THRESH, HOLD);

	zassert_false(gc_rate_update(&rate, &moved, 80 + HOLD - 1, THRESH, HOLD),
		      "hold counts from the last change, not from start");
	zassert_true(gc_rate_update(&rate, &moved, 80 + HOLD, THRESH, HOLD));
}

/* Regression: a resting stick jitters by a count or two. The first version
 * compared each poll against the previous one, so noise kept a resting pad
 * on the fast rate. It must settle to slow and stay there. */
ZTEST(rate, test_sensor_noise_settles_to_slow)
{
	int64_t t = 0;
	int flips = 0;

	for (int i = 0; i < 1000; i++, t += 4) {
		GC_Payload p = rest;

		p.joy_X_val += (i % 3) - 1;          /* -1, 0, +1 */
		p.C_stick_Y_val += (i & 1) ? 2 : 0;  /* 0, +2 */
		flips += gc_rate_update(&rate, &p, t, THRESH, HOLD);
	}
	zassert_false(rate.fast, "noise below the threshold must not keep it fast");
	zassert_equal(flips, 1, "exactly one fast->slow switch, no flapping");
}

/* ...but a slow, deliberate move (1 count per poll) must still be seen:
 * against the reference it adds up past the threshold. */
ZTEST(rate, test_slow_creep_is_detected)
{
	gc_rate_update(&rate, &rest, HOLD, THRESH, HOLD);
	zassert_false(rate.fast);

	GC_Payload p = rest;
	bool went_fast = false;

	for (int i = 1; i <= THRESH; i++) {
		p.left_trigger_val = rest.left_trigger_val + i;
		went_fast |= gc_rate_update(&rate, &p, HOLD + i * 4, THRESH, HOLD);
	}
	zassert_true(went_fast);
	zassert_true(rate.fast);
}
