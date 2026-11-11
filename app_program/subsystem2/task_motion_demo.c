/*
 *	task_motion_demo.c
 *	Subsystem 2 -- Motion control.  Demo / accuracy-test task, built
 *	only with S2_BRINGUP == 2.  It uses the public API exactly as the
 *	other subsystems will, so it also shows them how to call it.
 *
 *	Sequence (repeats): forward 500 mm, turn left 90, forward 500 mm,
 *	turn right 90, back 500 mm, U-turn (180), with a pause between moves
 *	so you can measure the car's position.
 */

#include "subsystem2.h"

#if S2_BRINGUP == 2

#define DEMO_DIST_MM		(500)
#define DEMO_SPEED		(100)
#define DEMO_PAUSE_MS		(3000)
#define DEMO_TIMEOUT_MS		(10000)

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

void
motion_demo_task (INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;

	(void)tk_dly_tsk(DEMO_PAUSE_MS);	/* time to put the car down */

	for (;;)
	{
		demo_step("forward 500", motion_move_forward(DEMO_DIST_MM, DEMO_SPEED));
		demo_step("left 90",     motion_turn_left(90, DEMO_SPEED));
		demo_step("forward 500", motion_move_forward(DEMO_DIST_MM, DEMO_SPEED));
		demo_step("right 90",    motion_turn_right(90, DEMO_SPEED));
		demo_step("back 500",    motion_move_backward(DEMO_DIST_MM, DEMO_SPEED));
		demo_step("u-turn 180",  motion_turn_left(180, DEMO_SPEED));
	}
}

#endif /* S2_BRINGUP == 2 */
