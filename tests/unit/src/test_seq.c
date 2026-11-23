/* Receiver sequence filter: missed-packet counting, duplicate rejection,
 * wraparound and resync. */
#include <zephyr/ztest.h>
#include "gc_logic.h"

static struct gc_seq seq;
static uint32_t missed;

static void before(void *f)
{
	ARG_UNUSED(f);
	gc_seq_init(&seq);
	missed = 0xDEAD;
}

ZTEST_SUITE(seq, NULL, NULL, before, NULL, NULL);

static enum gc_seq_result input(uint8_t s)
{
	return gc_seq_check(&seq, s, false, &missed);
}

static enum gc_seq_result origin(uint8_t s)
{
	return gc_seq_check(&seq, s, true, &missed);
}

ZTEST(seq, test_first_packet_any_seq_accepted)
{
	zassert_equal(input(200), GC_SEQ_ACCEPT);
	zassert_equal(missed, 0, "nothing to compare against yet");
	zassert_equal(seq.expected, 201);
}

ZTEST(seq, test_in_order_and_gaps)
{
	input(10);
	zassert_equal(input(11), GC_SEQ_ACCEPT);
	zassert_equal(missed, 0);
	zassert_equal(input(14), GC_SEQ_ACCEPT);
	zassert_equal(missed, 2, "12 and 13 were lost");
}

ZTEST(seq, test_duplicate_and_older_rejected)
{
	input(10);
	input(11);
	zassert_equal(input(11), GC_SEQ_DUP, "same seq again");
	zassert_equal(missed, 0);
	zassert_equal(input(5), GC_SEQ_DUP, "older");
	zassert_equal(seq.expected, 12, "rejects don't move the window");
}

ZTEST(seq, test_wraparound)
{
	input(254);
	zassert_equal(input(255), GC_SEQ_ACCEPT);
	zassert_equal(input(0), GC_SEQ_ACCEPT);
	zassert_equal(missed, 0, "255 -> 0 is in order");
	zassert_equal(input(3), GC_SEQ_ACCEPT);
	zassert_equal(missed, 2);
	zassert_equal(input(250), GC_SEQ_DUP, "250 is behind 4 across the wrap");
}

ZTEST(seq, test_ahead_behind_boundary)
{
	input(0);                    /* expected = 1 */
	zassert_equal(input(128), GC_SEQ_ACCEPT, "127 ahead: accepted");
	zassert_equal(missed, 127);

	gc_seq_init(&seq);
	input(0);                    /* expected = 1 */
	zassert_equal(input(129), GC_SEQ_DUP, "128 ahead counts as behind");
}

/* Regression: a retry whose ACK was lost used to arrive as a "new" packet
 * 255 behind, show up as "missed 256" and briefly replay an old input. */
ZTEST(seq, test_retry_duplicate_counts_nothing)
{
	uint32_t total = 0;
	const uint8_t rx[] = { 10, 11, 11, 12, 12, 12, 13 };
	int accepted = 0;

	for (size_t i = 0; i < ARRAY_SIZE(rx); i++) {
		if (input(rx[i]) == GC_SEQ_ACCEPT) {
			accepted++;
			total += missed;
		}
	}
	zassert_equal(accepted, 4, "10, 11, 12, 13 once each");
	zassert_equal(total, 0, "no phantom missed packets");
}

ZTEST(seq, test_resync_after_run_of_behind)
{
	input(100);                  /* expected = 101 */
	for (int i = 1; i < GC_SEQ_BEHIND_RESYNC; i++) {
		zassert_equal(input((uint8_t)(i)), GC_SEQ_DUP, "behind #%d", i);
	}
	zassert_equal(input(GC_SEQ_BEHIND_RESYNC), GC_SEQ_ACCEPT,
		      "8th behind in a row: controller restarted, resync");
	zassert_equal(missed, 0);
	zassert_equal(input(GC_SEQ_BEHIND_RESYNC + 1), GC_SEQ_ACCEPT);
	zassert_equal(missed, 0);
}

ZTEST(seq, test_behind_run_resets_on_accept)
{
	input(100);
	for (int i = 0; i < GC_SEQ_BEHIND_RESYNC - 1; i++) {
		input(50);
	}
	input(101);                  /* good packet clears the run */
	zassert_equal(input(50), GC_SEQ_DUP, "run started over, still a dup");
}

ZTEST(seq, test_origin_resyncs_immediately)
{
	input(100);
	zassert_equal(origin(0), GC_SEQ_ACCEPT, "controller rebooted");
	zassert_equal(missed, 0);
	zassert_equal(input(1), GC_SEQ_ACCEPT);
	zassert_equal(missed, 0);
}
