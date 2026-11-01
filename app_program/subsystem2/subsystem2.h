/*
 *	subsystem2.h
 *	Subsystem 2 -- Motion control (Buddy 2).  Student ID: <student ID>.
 *
 *	PUBLIC interface: the motion API other subsystems call, plus the
 *	hardware / tuning constants.  Other subsystems include it as
 *	"../subsystem2/subsystem2.h".
 *
 *	Units everywhere: millimetres, mm/s, degrees, milliseconds.
 *	No floating point is used (the build is soft-float on a Cortex-M0+).
 *
 *	Peripheral ownership (record in TEAM.md section 3):
 *	  PWM slice 5 (GP10 + GP11), motor direction GPIOs, encoder GPIOs
 *	  are driven ONLY by motion_task.  Every other task uses the API below.
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"

/* ====================================================================
 *  1. Hardware: pins.   Claim them in TEAM.md section 2 first!
 *     Change these to match your wiring.
 * ==================================================================== */
#define S2_PIN_L_PWM	10	/* left  motor enable/PWM  (PWM slice 5, ch A) */
#define S2_PIN_R_PWM	11	/* right motor enable/PWM  (PWM slice 5, ch B) */
				/* NOTE: both PWM pins must share a slice
				   (GP10/GP11, GP8/GP9, GP12/GP13 ...). */
#define S2_PIN_L_IN1	6	/* left  H-bridge direction pins */
#define S2_PIN_L_IN2	7
#define S2_PIN_R_IN1	8	/* right H-bridge direction pins */
#define S2_PIN_R_IN2	9
#define S2_PIN_L_ENC	12	/* left  wheel encoder (digital output) */
#define S2_PIN_R_ENC	13	/* right wheel encoder */

/* If a wheel turns the wrong way for a positive duty, flip it here */
#define S2_L_INVERT	0
#define S2_R_INVERT	0

/* 1: IN1=IN2=1 short-brakes at stop (TB6612/L298N); 0: coast (both low) */
#define S2_BRAKE_ON_STOP	1

#define S2_PWM_HZ	10000	/* PWM frequency; assumes 125 MHz system clock */

/* ====================================================================
 *  2. Geometry.  MEASURE these on your car, then calibrate (section 4).
 * ==================================================================== */
#define S2_WHEEL_DIAM_MM	65	/* wheel diameter */
#define S2_TRACK_MM		130	/* centre-to-centre distance of wheels */
#define S2_ENC_SLOTS		20	/* slots (pulses) per wheel revolution */
#define S2_TICKS_PER_REV	(S2_ENC_SLOTS * 2)	/* both edges counted */

/* Wheel circumference in micrometres (pi * D, integer) */
#define S2_CIRC_UM		((S2_WHEEL_DIAM_MM * 3141593UL) / 1000UL)

/* ====================================================================
 *  3. Timing and tuning.
 * ==================================================================== */
#define S2_PRI_MOTION		14	/* priority band 14-15 (TEAM.md) */
#define S2_ENC_POLL_MS		1	/* encoder GPIO sampled every 1 ms */
#define S2_ENC_DEBOUNCE_MS	3	/* ignore edges closer than this */
#define S2_CTRL_MS		20	/* control loop period */
#define S2_SPD_WIN		5	/* speed = ticks over WIN*CTRL_MS (100 ms) */

#define S2_SPEED_MAX		400	/* mm/s at 100 % duty (calibrate!) */
#define S2_SPEED_CRUISE		200	/* default speed for distance moves */
#define S2_SPEED_MIN		60	/* creep speed at the end of a move */
#define S2_TURN_SPEED		120	/* wheel speed while turning, mm/s */
#define S2_RAMP_MM		40	/* start slowing this far from target */
#define S2_DUTY_MIN		350	/* permille needed to start the wheel */
#define S2_DUTY_MAX		1000

/* PID (fixed point: gain/256, evaluated once per S2_CTRL_MS).
   Duty (permille) = feed-forward + (KP*err + KI*sum(err) - KD*dmeas)/256 */
#define S2_KP			128	/* 0.50 */
#define S2_KI			24	/* 0.09 per tick */
#define S2_KD			0
#define S2_I_LIMIT		400	/* integral term limited to +/-400 permille */

/* Straight-line / turn synchronisation: mm/s of correction per encoder
   tick of difference between the two wheels. */
#define S2_KSYNC		6
#define S2_SYNC_MAX		60	/* max correction, mm/s */

#define S2_STALL_MS		1000	/* no ticks for this long => abort */

/* ====================================================================
 *  4. Calibration trims (percent).  Adjust after the accuracy test.
 *     e.g. car goes 950 mm for a 1000 mm command -> S2_DIST_CAL = 105.
 * ==================================================================== */
#define S2_DIST_CAL		100
#define S2_TURN_CAL		100

/* ====================================================================
 *  5. PUBLIC API  (call from any task; NON-BLOCKING unless noted)
 * ==================================================================== */

/* Distance and turns.  A new command replaces the one in progress.
   Returns E_OK, or E_PAR for a zero / absurd argument. */
IMPORT ER moveForward(UW distance_mm);
IMPORT ER moveBackward(UW distance_mm);
IMPORT ER turnLeft(UW angle_deg);	/* pivot on the spot, anticlockwise */
IMPORT ER turnRight(UW angle_deg);	/* pivot on the spot, clockwise */
IMPORT ER stop(void);			/* stop now, cancels the command */

/* Continuous closed-loop wheel speeds (mm/s, negative = reverse), for
   line-following.  No distance limit; call stop() to end it. */
IMPORT ER motion_set_speeds(INT left_mm_s, INT right_mm_s);

/* Speed used by moveForward/moveBackward (default S2_SPEED_CRUISE) */
IMPORT void motion_set_cruise(UW mm_s);

/* Progress of the last distance/turn command */
IMPORT BOOL motion_is_done(void);		/* TRUE when idle / finished */
IMPORT ER   motion_wait_done(TMO tmout);	/* BLOCKS; E_TMOUT if not done */

typedef struct {
	INT	speed_l, speed_r;	/* measured wheel speed, mm/s */
	INT	duty_l, duty_r;		/* last PWM duty, permille (-1000..1000) */
	INT	ticks_l, ticks_r;	/* signed encoder ticks since reset */
	INT	distance_mm;		/* mean signed distance since reset */
	INT	state;			/* S2_ST_xxx */
	INT	error;			/* 0, or S2_ERR_xxx of the last command */
} MOTION_STATUS;

#define S2_ST_IDLE	0
#define S2_ST_SPEED	1
#define S2_ST_MOVING	2
#define S2_ST_TURNING	3
#define S2_ERR_NONE	0
#define S2_ERR_STALL	1	/* wheels not turning: aborted */

IMPORT void motion_get_status(MOTION_STATUS *st);
IMPORT void motion_reset_distance(void);

/* ====================================================================
 *  6. Private to subsystem 2 (shared between its own .c files)
 * ==================================================================== */
#define S2_FLG_DONE	0x01	/* event flag bit: command finished */
IMPORT ID	tid_motion;
IMPORT ID	cyc_motion;
IMPORT ID	mtx_motion;
IMPORT ID	flg_motion;
IMPORT void	motion_task(INT stacd, void *exinf);
IMPORT void	motion_cyclic_handler(void *exinf);

/* drv_motor.c -- owned by motion_task */
#define S2_LEFT		0
#define S2_RIGHT	1
IMPORT ER	motor_init(void);
IMPORT void	motor_set(INT side, INT duty_permille);	/* -1000..1000 */

/* drv_encoder.c -- polled from the cyclic handler */
IMPORT void	enc_init(void);
IMPORT void	enc_poll(void);			/* call every S2_ENC_POLL_MS */
IMPORT UW	enc_raw(INT side);		/* edges counted since boot */

/* Pure helpers (also used by the unit tests) */
IMPORT INT	s2_clamp(INT v, INT lo, INT hi);
IMPORT UW	s2_mm_to_ticks(UW mm);
IMPORT UW	s2_turn_to_ticks(UW deg);	/* ticks per wheel for a pivot */
IMPORT INT	s2_ticks_to_mm(INT ticks);

typedef struct { INT integ; INT prev_meas; } S2_PID;
IMPORT INT	s2_pid_step(S2_PID *p, INT setpoint, INT meas);

#endif /* SUBSYSTEM2_H */
