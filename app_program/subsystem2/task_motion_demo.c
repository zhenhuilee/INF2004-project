/*
 *	task_motion_demo.c
 *	Subsystem 2 -- Motion control.  Demo / accuracy-test task, built
 *	only with S2_BRINGUP == 2.  It uses the public API exactly as the
 *	other subsystems will, so it also shows them how to call it.
 *
 *	DEMO_MODE selects what the car does after RST (3 s delay first):
 *	  0  demo, repeats: forward 500, left 90, forward 500, right 90,
 *	     back 500, U-turn 180, with a pause between moves
 *	  1  distance test:  forward DEMO_TEST_DIST_MM once, then wait
 *	  2  360 turn test:  turn left 360 once, then wait
 *	  3  90 left test:   turn left 90 once, then wait
 *	  4  90 right test:  turn right 90 once, then wait
 *	In modes 1-4 the car stops and does nothing more until RST, so
 *	there is time to measure.  The distance each wheel travelled
 *	according to its encoder is printed for the test log.
 */

#include "subsystem2.h"
#include "motion_config.h"

#if S2_BRINGUP == 2

#ifndef DEMO_MODE
#define DEMO_MODE		(2)
#endif

#define DEMO_DIST_MM		(500)
#define DEMO_TEST_DIST_MM	(1000)
#define DEMO_SPEED		(MOTION_DEFAULT_SPEED)
#define DEMO_PAUSE_MS		(3000)
#define DEMO_TIMEOUT_MS		(15000)

#define DEMO_PI			(3.14159265f)
#define DEMO_MM_PER_TICK	((DEMO_PI * MOTION_WHEEL_DIAMETER_MM * MOTION_DIST_SCALE) \
				 / MOTION_TICKS_PER_REV)

static int32_t
demo_mm (int32_t ticks)
{
	float	mm = (float)ticks * DEMO_MM_PER_TICK;

	return (mm < 0.0f) ? (int32_t)(mm - 0.5f) : (int32_t)(mm + 0.5f);
}

static void
demo_step (const char *p_name, int32_t seq)
{
	motion_status_t	st;
	ER		er;

	er = (ER)motion_wait_done(seq, DEMO_TIMEOUT_MS);
	motion_get_status(&st);
	tm_printf((UB*)"[S2-DEMO] %s: %s  odometer=%d mm  counts L=%d R=%d\n",
		  p_name, (er == E_OK) ? "done" : "TIMEOUT",
		  st.odometer_mm, st.enc_count[0], st.enc_count[1]);
	(void)tk_dly_tsk(DEMO_PAUSE_MS);
}

/* One move for an accuracy test, then report how far each wheel went */
static void
demo_test_once (const char *p_name, int32_t seq)
{
	motion_status_t	before;
	motion_status_t	after;
	ER		er;

	motion_get_status(&before);
	er = (ER)motion_wait_done(seq, DEMO_TIMEOUT_MS);
	(void)tk_dly_tsk(1000);		/* let the car settle after braking */
	motion_get_status(&after);

	tm_printf((UB*)"[S2-TEST] %s: %s  wheel L=%d mm  R=%d mm  (encoder)\n",
		  p_name, (er == E_OK) ? "done" : "TIMEOUT",
		  demo_mm(after.enc_count[0] - before.enc_count[0]),
		  demo_mm(after.enc_count[1] - before.enc_count[1]));
	tm_printf((UB*)"[S2-TEST] finished - measure now, then press RST for the next run\n");
}

void
motion_demo_task (INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;

	(void)tk_dly_tsk(DEMO_PAUSE_MS);	/* time to put the car down */

#if DEMO_MODE == 1
	demo_test_once("forward 1000", motion_move_forward(DEMO_TEST_DIST_MM, DEMO_SPEED));
#elif DEMO_MODE == 2
	demo_test_once("left 360", motion_turn_left(360, DEMO_SPEED));
#elif DEMO_MODE == 3
	demo_test_once("left 90", motion_turn_left(90, DEMO_SPEED));
#elif DEMO_MODE == 4
	demo_test_once("right 90", motion_turn_right(90, DEMO_SPEED));
#else
	for (;;)
	{
		demo_step("forward 500", motion_move_forward(DEMO_DIST_MM, DEMO_SPEED));
		demo_step("left 90",     motion_turn_left(90, DEMO_SPEED));
		demo_step("forward 500", motion_move_forward(DEMO_DIST_MM, DEMO_SPEED));
		demo_step("right 90",    motion_turn_right(90, DEMO_SPEED));
		demo_step("back 500",    motion_move_backward(DEMO_DIST_MM, DEMO_SPEED));
		demo_step("u-turn 180",  motion_turn_left(180, DEMO_SPEED));
	}
#endif

	for (;;)
	{
		(void)tk_slp_tsk(TMO_FEVR);	/* test done: wait for RST */
	}
}

#endif /* S2_BRINGUP == 2 */