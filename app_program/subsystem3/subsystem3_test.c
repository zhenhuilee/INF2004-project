/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem3/TEST_LOG.md and the
 *	INT- result in testing/integration/TEST_LOG.md.
 */

#include "subsystem3.h"
#include "../subsystem2/motion.h"
#include "../test_report.h"

#if APP_TEST

LOCAL APP_STATS read_stats(void)
{
	APP_STATS	s;

	tk_loc_mtx(mtx_stats, TMO_FEVR);
	s = app_stats;
	tk_unl_mtx(mtx_stats);
	return s;
}

EXPORT void subsystem3_test(void)
{
	APP_STATS	before, after;
	motion_status_t	st;

	TEST_INIT();

	/* INT-01 (integration, IF-01): samples from subsystem 2 arrive.
	   At one sample per 300 ms, 2 s should bring at least 5. */
	before = read_stats();
	tk_dly_tsk(2000);
	after = read_stats();
	TEST_CHECK("INT-01", after.received >= before.received + 5,
		   "samples flow from subsystem 2 to subsystem 3");

	/* S3-U01: the statistics are self-consistent */
	TEST_CHECK("S3-U01",
		   after.received > 0 && after.min <= after.max &&
		   after.min >= 0 && after.max <= 100,
		   "min <= max, both within 0..100");

	/* S3-IR01: line-tracking drive and steering command is accepted */
	ER drv_er = motion_drive(80, 30);
	tk_dly_tsk(250);
	motion_get_status(&st);
	TEST_CHECK("S3-IR01", drv_er == E_OK && st.mode != MOTION_IDLE,
		   "motion_drive accepts line-tracking steering command");
	(void)motion_drive(0, 0);

	/* S3-IR02: forward offset + 90-deg left turn completes cleanly */
	int32_t fwd_seq = motion_move_forward(85, 80);
	ER fwd_er = (fwd_seq > 0) ? motion_wait_done(fwd_seq, 2000) : E_PAR;
	int32_t turn_seq = (fwd_er == E_OK) ? motion_turn_left(90, MOTION_DEFAULT_SPEED) : -1;
	ER turn_er = (turn_seq > 0) ? motion_wait_done(turn_seq, 3000) : E_PAR;
	TEST_CHECK("S3-IR02", fwd_er == E_OK && turn_er == E_OK,
		   "forward offset + 90-deg left turn sequence completes");

	TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */