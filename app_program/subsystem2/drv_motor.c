/*
 *	drv_motor.c
 *	Subsystem 2 -- dual H-bridge motor driver (PWM + 2 direction pins
 *	per motor).  Works with L298N / TB6612 style boards.
 *	Call ONLY from motion_task (it owns PWM slice 5 and these GPIOs).
 */

#include "subsystem2.h"

#define PWM_WRAP	(125000000UL / S2_PWM_HZ - 1)	/* divider = 1.0 */

typedef struct { UINT pwm, in1, in2; BOOL invert; } MOTOR;

static const MOTOR motors[2] = {
	{ S2_PIN_L_PWM, S2_PIN_L_IN1, S2_PIN_L_IN2, S2_L_INVERT },
	{ S2_PIN_R_PWM, S2_PIN_R_IN1, S2_PIN_R_IN2, S2_R_INVERT },
};

EXPORT ER motor_init(void)
{
	INT i;

	for (i = 0; i < 2; i++) {
		gpio_set_pin(motors[i].in1, GPIO_MODE_OUT);
		gpio_set_pin(motors[i].in2, GPIO_MODE_OUT);
		gpio_set_val(motors[i].in1, 0);
		gpio_set_val(motors[i].in2, 0);
		pwm_set_pin(motors[i].pwm);
		pwm_set_wrap(motors[i].pwm, PWM_WRAP);
		pwm_set_cc(motors[i].pwm, 0);
	}
	/* Both PWM pins share a slice: enabling one enables both */
	pwm_set_enabled(motors[0].pwm, TRUE);
	return E_OK;
}

/* duty: -1000 (full reverse) .. +1000 (full forward), 0 = stop */
EXPORT void motor_set(INT side, INT duty)
{
	const MOTOR *m = &motors[side];
	INT mag;
	BOOL fwd;

	duty = s2_clamp(duty, -1000, 1000);
	fwd  = (duty >= 0) ^ m->invert;
	mag  = (duty >= 0) ? duty : -duty;

	if (mag == 0) {
#if S2_BRAKE_ON_STOP
		gpio_set_val(m->in1, 1);
		gpio_set_val(m->in2, 1);
#else
		gpio_set_val(m->in1, 0);
		gpio_set_val(m->in2, 0);
#endif
	} else {
		gpio_set_val(m->in1, fwd ? 1 : 0);
		gpio_set_val(m->in2, fwd ? 0 : 1);
	}
	pwm_set_cc(m->pwm, (UW)(((UW)mag * PWM_WRAP) / 1000UL));
}
