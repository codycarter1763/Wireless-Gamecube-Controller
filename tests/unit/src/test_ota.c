/* OTA combo (L + R + Z + A) detection and hold timing. */
#include <zephyr/ztest.h>
#include "gc_logic.h"
#include "test_util.h"

#define COMBO_B0 (A_BTN)
#define COMBO_B1 (L_BTN | R_BTN | Z_BTN)

ZTEST_SUITE(ota, NULL, NULL, NULL, NULL, NULL);

ZTEST(ota, test_combo_exact)
{
	GC_Payload p = pad_buttons(COMBO_B0, COMBO_B1);

	zassert_true(gc_ota_combo_pressed(&p));
}

ZTEST(ota, test_combo_with_extra_buttons)
{
	GC_Payload p = pad_buttons(COMBO_B0 | B_BTN, COMBO_B1 | D_UP_BTN);

	zassert_true(gc_ota_combo_pressed(&p), "extra buttons don't block it");
}

ZTEST(ota, test_combo_missing_any_button)
{
	const uint8_t b1_parts[] = { L_BTN, R_BTN, Z_BTN };

	GC_Payload no_a = pad_buttons(0, COMBO_B1);

	zassert_false(gc_ota_combo_pressed(&no_a), "A missing");

	for (size_t i = 0; i < ARRAY_SIZE(b1_parts); i++) {
		GC_Payload p = pad_buttons(COMBO_B0, COMBO_B1 & ~b1_parts[i]);

		zassert_false(gc_ota_combo_pressed(&p), "missing 0x%02x", b1_parts[i]);
	}

	GC_Payload rest = pad_rest();

	zassert_false(gc_ota_combo_pressed(&rest));
}

ZTEST(ota, test_hold_timing)
{
	int64_t since = -1;

	zassert_equal(gc_combo_hold(&since, false, 0), -1, "not held");
	zassert_equal(since, -1);

	zassert_equal(gc_combo_hold(&since, true, 1000), 0, "just pressed");
	zassert_equal(gc_combo_hold(&since, true, 3500), 2500);
	zassert_equal(gc_combo_hold(&since, true, 6000), 5000, "5 s: trigger");
}

ZTEST(ota, test_release_restarts_count)
{
	int64_t since = -1;

	gc_combo_hold(&since, true, 0);
	gc_combo_hold(&since, true, 4900);
	zassert_equal(gc_combo_hold(&since, false, 4901), -1, "let go at 4.9 s");
	zassert_equal(since, -1);
	zassert_equal(gc_combo_hold(&since, true, 5000), 0, "starts over from 0");
}
