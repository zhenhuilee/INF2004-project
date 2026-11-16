/*
 *	drv_motor.h
 *	Subsystem 2 -- Motion control.  DC motor driver for the Robo Pico
 *	on-board H-bridge (Motor 1 = GP8/GP9, Motor 2 = GP10/GP11).
 *
 *	Only the motion task (motion.c) may call these functions: it is the
 *	owning task of PWM slices 4 and 5 (TEAM.md section 3).
 *
 *	Each motor has two driver inputs (A and B).  The bridge works like:
 *	    A = PWM, B = 0   -> motor turns one way, speed set by duty
 *	    A = 0,   B = PWM -> motor turns the other way
 *	    A = 0,   B = 0   -> coast (free-wheel)
 *	    A = 1,   B = 1   -> brake (motor terminals shorted)
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdint.h>

/* ---------------------------------------------------------------------
 *  Wiring.  Motor A (Robo Pico "Motor 1") is assumed to be the LEFT
 *  wheel.  If your Motor A is on the right, swap the two blocks.
 * --------------------------------------------------------------------- */
#define MOTOR_LEFT_PIN_POS	(9U)	/* M1B, motor wire "+" */
#define MOTOR_LEFT_PIN_NEG	(8U)	/* M1A, motor wire "-" */
#define MOTOR_RIGHT_PIN_POS	(11U)	/* M2B, motor wire "+" */
#define MOTOR_RIGHT_PIN_NEG	(10U)	/* M2A, motor wire "-" */

/* Set to 1 if that wheel turns BACKWARDS for a positive duty.  The two
   motors are mounted mirror-image, so with identical wiring one of them
   normally needs inverting.  Check it with bring-up step 2 (GUIDE.md). */
#define MOTOR_LEFT_INVERT	(0U)
#define MOTOR_RIGHT_INVERT	(1U)

/* PWM: 125 MHz system clock / (TOP + 1) = 20 kHz, above hearing range */
#define MOTOR_PWM_TOP		(6249U)

/* Duty is given in per-mille: -1000 (full reverse) .. +1000 (full fwd) */
#define MOTOR_DUTY_MAX		(1000)

typedef enum
{
	MOTOR_LEFT = 0,
	MOTOR_RIGHT,
	MOTOR_COUNT
} motor_id_t;

void	motor_init(void);
void	motor_set_duty(motor_id_t id, int32_t duty_permille);
void	motor_brake(motor_id_t id);
void	motor_coast(motor_id_t id);

#endif /* DRV_MOTOR_H */
