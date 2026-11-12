/*
 *	motion_config.h
 *	Subsystem 2 -- Motion control.  Calibration and tuning constants.
 *
 *	Every value marked MEASURE comes from your own car: see GUIDE.md
 *	steps 4-6.  Record the values you settle on in the PID tuning
 *	report, with the test that justified each one.
 */

#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

/* ---- Task timing ---------------------------------------------------- */
#define MOTION_PERIOD_MS	(10U)	/* control loop: 100 Hz */
#define MOTION_PRI		(14)	/* S2 band 14-15 (TEAM.md) */
#define MOTION_CMD_QUEUE	(8U)	/* commands that can wait in the queue */
#define MOTION_SEND_TMO_MS	(10)	/* how long a caller waits if it is full */
#define MOTION_DRIVE_TIMEOUT_MS	(500U)	/* drive() watchdog */
#define MOTION_DEBUG_EVERY	(20U)	/* print every N loops (200 ms) */

/* ---- Geometry (MEASURE) --------------------------------------------- */
#define MOTION_WHEEL_DIAMETER_MM	(62.0f) /*new*/
#define MOTION_TRACK_WIDTH_MM		(96.0f)  
#define MOTION_TICKS_PER_REV		(1279.0f)  
#define MOTION_DIST_SCALE		(1.06f)   /* fine-tune after the 1 m test */
#define MOTION_TURN_SCALE		(1.125f)   /* fine-tune after the 360 deg test */

/* ---- Speed PID, one per wheel (output: duty in per-mille) ------------ */
#define MOTION_KF		(1.15f)	  /* feed-forward: duty per mm/s */
#define MOTION_DEADBAND		(20.0f)  /* duty needed just to start turning */
#define MOTION_KP		(1.64f)	  /* duty per (mm/s of error) */
#define MOTION_KI		(4.93f)	  /* duty per mm of accumulated error */
#define MOTION_KD		(0.0f)	  /* duty per (mm/s^2); start at 0 */
#define MOTION_I_LIMIT		(300.0f)  /* anti wind-up clamp on the I term */
#define MOTION_SPEED_ALPHA	(0.4f)	  /* speed filter: weight of newest sample */

/* ---- Keeping straight / turning on the spot ------------------------- */
#define MOTION_KP_SYNC		(3.0f)	  /* mm/s of correction per mm of wheel mismatch */
#define MOTION_SYNC_LIMIT	(80.0f)	  /* max correction, mm/s */

/* ---- Speed profile for moves and turns ------------------------------ */
#define MOTION_ACCEL		(800.0f)  /* mm/s^2 ramp-up */
#define MOTION_DECEL_GAIN	(4.0f)	  /* slow-down: v = gain x remaining mm */
#define MOTION_MIN_SPEED	(50.0f)	  /* mm/s, so the car still creeps to the end */
#define MOTION_MAX_SPEED	(600.0f)  /* mm/s, requests are clamped to this */
#define MOTION_STOP_TOL_MM	(1.0f)	  /* "arrived" when this close */

#endif /* MOTION_CONFIG_H */
