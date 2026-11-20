/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.
 *
 *	These run with the wheels OFF the ground (car on a stand).
 */

#include "subsystem2.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem2_test(void)
{
	T_RSEM		rsem;
	T_RMPF		rmpf;
#if S2_BRINGUP != 1
	motion_status_t	st;
	int32_t		seq;
#endif
	UW		s;
	INT		v, bad = 0;

	TEST_INIT();

	/* ---- EXAMPLE producer tests (delete with the example block) ---- */
	for (s = 0; s < 40; s++) {
		v = s2_make_value(s);
		if (v < 0 || v > 100) bad++;
	}
	TEST_CHECK("S2-U01", bad == 0, "sample values stay within 0..100");
	TEST_CHECK("S2-U02",
		   tk_ref_sem(sem_slots, &rsem) == E_OK &&
		   rsem.semcnt >= 0 && rsem.semcnt <= S2_POOL_COUNT &&
		   tk_ref_mpf(mpf_samples, &rmpf) == E_OK &&
		   rmpf.frbcnt <= S2_POOL_COUNT,
		   "free slots and free blocks within pool size");

#if S2_BRINGUP != 1
	/* ---- Motion control ------------------------------------------- */

	/* S2-U03: the motion task accepts a command and reports it done */
	seq = motion_stop();
	TEST_CHECK("S2-U03", seq > 0 && motion_wait_done(seq, 200) == E_OK,
		   "stop() is accepted and completes within 200 ms");

	/* S2-U04: at rest the encoders see no impossible transitions */
	(void)tk_dly_tsk(1000);
	motion_get_status(&st);
	TEST_CHECK("S2-U04", st.mode == MOTION_IDLE && st.enc_errors == 0U,
		   "idle with zero encoder errors");

	/* S2-U05: a 100 mm move makes both wheels count FORWARD, by about
	   the same amount (wheels in the air) */
	{
		int32_t	l0 = st.enc_count[0];
		int32_t	r0 = st.enc_count[1];
		int32_t	dl, dr;

		seq = motion_move_forward(100, MOTION_DEFAULT_SPEED);
		TEST_CHECK("S2-U05a", motion_wait_done(seq, 3000) == E_OK,
			   "100 mm move finishes within 3 s");
		motion_get_status(&st);
		dl = st.enc_count[0] - l0;
		dr = st.enc_count[1] - r0;
		TEST_CHECK("S2-U05b", dl > 0 && dr > 0,
			   "both encoders count up when moving forward");
		TEST_CHECK("S2-U05c", (dl - dr) < (dl / 10) && (dr - dl) < (dr / 10),
			   "left and right counts within 10% of each other");
	}

	/* S2-U06: a failed command's error code is not mistaken for success */
	TEST_CHECK("S2-U06", motion_wait_done(-1, 0) == -1,
		   "wait_done returns the error of a failed command");

	(void)motion_stop();
#endif

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
