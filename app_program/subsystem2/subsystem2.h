/*
 *	subsystem2.h
 *	Subsystem 2 -- Motion control (Buddy 2).  Student ID: <student ID>.
 *
 *	Public interface of subsystem 2.  Other subsystems include it as
 *	"../subsystem2/subsystem2.h" and use the motion_xxx() API declared
 *	in motion.h (IF-06 in TEAM.md).
 *
 *	Owns: GP8-GP11 (motor PWM, slices 4 and 5), GP7/GP28/GP27/GP26
 *	(encoders), IO_IRQ_BANK0 (GPIO interrupt).
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"
#include "motion.h"

/* ---------------------------------------------------------------------
 *  Build switches for bring-up and debugging (see GUIDE.md)
 *    S2_BRINGUP 0  normal operation
 *               1  open-loop motor + encoder check (no PID)
 *               2  normal operation + a demo task that drives a square
 *    S2_DEBUG   1  print the motion state every 200 ms
 * --------------------------------------------------------------------- */
#ifndef S2_BRINGUP
#define S2_BRINGUP		(1)
#endif
#define S2_BRINGUP_DUTY		(0)	/* per-mille, for S2_BRINGUP 1 */
#ifndef S2_DEBUG
#define S2_DEBUG		(1)
#endif

#define S2_PRI_DEMO		(15)	/* demo task, band 14-15 */

/* =====================================================================
 *  EXAMPLE PRODUCER (from the template).  Kept only because the example
 *  subsystem 3 still uses these objects.  When subsystem 3 no longer
 *  includes this header for them, delete this block, task_producer.c and
 *  the producer lines in subsystem2.c and subsystem2_test.c.
 * ===================================================================== */
#define S2_PRI_PRODUCER		15
#define S2_PERIOD_MS		300

typedef struct {
	T_MSG	hdr;
	UW	seq;
	UW	time_ms;
	INT	value;
	INT	prc;
} SAMPLE_MSG;

#define S2_POOL_COUNT	4

IMPORT ID	sem_slots;
IMPORT ID	mpf_samples;

IMPORT void	producer_task(INT stacd, void *exinf);
IMPORT INT	s2_make_value(UW seq);
/* ===================== end of example block ========================= */

#if S2_BRINGUP == 2
void	motion_demo_task(INT stacd, void *exinf);
#endif

#endif /* SUBSYSTEM2_H */
