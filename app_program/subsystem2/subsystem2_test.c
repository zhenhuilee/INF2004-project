/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.
 *
 *	Automatic tests (S2-U01..U05) need no motors.
 *
 *	Motor tests (S2-M01..M03) MOVE THE CAR.  They are OFF by default so
 *	a test image never drives off a desk.  Put the car on the floor with
 *	a metre of clear space, set S2_TEST_MOTORS to 1, rebuild with
 *	make TEST=1, and save the console output to testing/subsystem2/logs/.
 *	They print CSV lines tagged [TUNE] / [ACC] for the PID tuning report
 *	and the motion accuracy evaluation.
 */

#include "subsystem2.h"
#include "../test_report.h"

#if APP_TEST

#define S2_TEST_MOTORS	0

#if S2_TEST_MOTORS
/* S2-M01: step response of both wheels at a given speed (PID tuning) */
static void tune_step(INT speed_mm_s, INT ms)
{
	MOTION_STATUS st;
	INT t;

	tm_printf((UB*)"[TUNE] kp=%d ki=%d kd=%d sp=%d\n",
		  S2_KP, S2_KI, S2_KD, speed_mm_s);
	tm_printf((UB*)"[TUNE] t_ms,sp,v_l,v_r,duty_l,duty_r\n");
	motion_set_speeds(speed_mm_s, speed_mm_s);
	for (t = 0; t < ms; t += S2_CTRL_MS) {
		tk_dly_tsk(S2_CTRL_MS);
		motion_get_status(&st);
		tm_printf((UB*)"[TUNE] %d,%d,%d,%d,%d,%d\n", t, speed_mm_s,
			  st.speed_l, st.speed_r, st.duty_l, st.duty_r);
	}
	stop();
	tk_dly_tsk(500);
}

/* S2-M02: repeat a distance move n times, print where it ended */
static void accuracy_distance(UW mm, INT runs)
{
	MOTION_STATUS st;
	INT i;

	for (i = 0; i < runs; i++) {
		motion_reset_distance();
		tk_dly_tsk(2 * S2_CTRL_MS);
		moveForward(mm);
		motion_wait_done(10000);
		tk_dly_tsk(500);		/* let it roll to a halt */
		motion_get_status(&st);
		tm_printf((UB*)"[ACC] fwd target=%d run=%d enc_mm=%d "
			  "ticks_l=%d ticks_r=%d err=%d  (measure the real "
			  "distance with a tape)\n", (INT)mm, i, st.distance_mm,
			  st.ticks_l, st.ticks_r, st.error);
		moveBackward(mm);		/* come back for the next run */
		motion_wait_done(10000);
		tk_dly_tsk(500);
	}
}

/* S2-M03: repeat a turn; measure the heading change with a protractor */
static void accuracy_turn(UW deg, INT runs)
{
	INT i;

	for (i = 0; i < runs; i++) {
		turnLeft(deg);
		motion_wait_done(10000);
		tk_dly_tsk(1000);
		tm_printf((UB*)"[ACC] turnLeft target=%d run=%d (measure it)\n",
			  (INT)deg, i);
	}
}
#endif /* S2_TEST_MOTORS */

EXPORT void subsystem2_test(void)
{
	S2_PID	pid = {0};
	INT	out;

	TEST_INIT();

	/* S2-U01: distance -> ticks matches the geometry.  1000 mm must be
	   within one tick of 1000 / (circumference / ticks per rev). */
	{
		UW ticks = s2_mm_to_ticks(1000);
		INT back = s2_ticks_to_mm((INT)ticks);
		TEST_CHECK("S2-U01", back >= 995 && back <= 1005,
			   "1000 mm -> ticks -> mm round-trips within 5 mm");
	}

	/* S2-U02: a 180 deg turn needs twice the ticks of a 90 deg turn */
	{
		INT t90 = (INT)s2_turn_to_ticks(90);
		INT t180 = (INT)s2_turn_to_ticks(180);
		TEST_CHECK("S2-U02", t90 > 0 && (t180 - 2 * t90) >= -1 &&
			   (t180 - 2 * t90) <= 1,
			   "turn ticks scale linearly with angle");
	}

	/* S2-U03: PID pushes the right way and is bounded */
	out = s2_pid_step(&pid, 200, 0);
	TEST_CHECK("S2-U03", out > 0, "PID output positive when too slow");
	{
		INT i;
		for (i = 0; i < 1000; i++) out = s2_pid_step(&pid, 200, 0);
		TEST_CHECK("S2-U04",
			   out <= (S2_KP * 200) / 256 + S2_I_LIMIT + 1,
			   "integral anti-windup keeps output bounded");
	}

	/* S2-U05: invalid arguments are rejected, nothing starts moving */
	TEST_CHECK("S2-U05",
		   moveForward(0) == E_PAR && turnLeft(0) == E_PAR &&
		   motion_is_done(),
		   "zero distance / angle rejected, still idle");

#if S2_TEST_MOTORS
	tune_step(150, 2000);
	tune_step(250, 2000);
	accuracy_distance(500, 10);
	accuracy_distance(1000, 10);
	accuracy_turn(90, 10);
	accuracy_turn(180, 10);
#endif

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
