/* Controller link state: send policy, loss detection, probing, recovery. */
#include <zephyr/ztest.h>
#include "gc_logic.h"

#define KEEPALIVE 20     /* CONFIG_APP_KEEPALIVE_MS */
#define PROBE     100    /* CONFIG_APP_LINK_PROBE_MS */
#define LOSS      1000   /* CONFIG_APP_LINK_LOSS_MS */

static struct gc_link link;

static void before(void *f)
{
	ARG_UNUSED(f);
	gc_link_init(&link, 0);
}

ZTEST_SUITE(link, NULL, NULL, before, NULL, NULL);

static bool want(int64_t now, bool changed, bool rumble, int64_t last_send)
{
	return gc_link_want_send(&link, now, changed, rumble, last_send,
				 KEEPALIVE, PROBE);
}

ZTEST(link, test_linked_send_policy)
{
	zassert_true(link.up);
	zassert_false(want(10, false, false, 0), "nothing to say yet");
	zassert_true(want(10, true, false, 0), "input changed");
	zassert_true(want(10, false, true, 0), "rumble on: keep sending");
	zassert_false(want(KEEPALIVE - 1, false, false, 0));
	zassert_true(want(KEEPALIVE, false, false, 0), "keepalive due");
}

ZTEST(link, test_failures_shorter_than_loss_time_keep_link)
{
	for (int64_t t = 1; t < LOSS; t += 7) {
		zassert_equal(gc_link_on_send(&link, t, false, LOSS), GC_LINK_NONE);
	}
	zassert_true(link.up, "a burst of failures under 1 s is not a loss");
}

ZTEST(link, test_lost_after_loss_time_reported_once)
{
	zassert_equal(gc_link_on_send(&link, LOSS, false, LOSS), GC_LINK_LOST);
	zassert_false(link.up);
	zassert_equal(link.lost_at, LOSS);
	zassert_equal(gc_link_on_send(&link, LOSS + 100, false, LOSS), GC_LINK_NONE,
		      "already lost: no second event");
}

ZTEST(link, test_ack_resets_loss_timer)
{
	gc_link_on_send(&link, 900, true, LOSS);
	zassert_equal(gc_link_on_send(&link, 1500, false, LOSS), GC_LINK_NONE,
		      "only 600 ms since the last ACK");
	zassert_equal(gc_link_on_send(&link, 1900, false, LOSS), GC_LINK_LOST);
}

ZTEST(link, test_lost_only_probes)
{
	gc_link_on_send(&link, LOSS, false, LOSS);   /* lost, probe at 1000 */

	zassert_false(want(LOSS + 50, true, true, LOSS),
		      "changes and rumble don't send while lost");
	zassert_false(want(LOSS + PROBE - 1, false, false, LOSS));
	zassert_true(want(LOSS + PROBE, false, false, LOSS), "probe due");
}

ZTEST(link, test_probe_ack_restores)
{
	gc_link_on_send(&link, LOSS, false, LOSS);
	gc_link_on_send(&link, LOSS + PROBE, false, LOSS);

	zassert_equal(gc_link_on_send(&link, LOSS + 2 * PROBE, true, LOSS),
		      GC_LINK_RESTORED);
	zassert_true(link.up);
	zassert_equal(gc_link_on_send(&link, LOSS + 2 * PROBE + 1, true, LOSS),
		      GC_LINK_NONE, "restored only once");
}
