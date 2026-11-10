/*
 *	drv_motor.c
 *	Subsystem 2 -- Motion control.  DC motor driver (PWM, 20 kHz).
 *
 *	Uses the board helpers in <bsp/libbsp.h> (pwm_set_pin, pwm_set_wrap,
 *	pwm_set_cc, pwm_set_enabled) plus two register writes the helpers do
 *	not cover: taking the PWM block out of reset and setting the clock
 *	divider.
 */

#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include "drv_motor.h"

/* RP2040 register details not provided by sysdef.h */
#define REG_ALIAS_CLR_OFFSET	(0x3000U)	/* atomic bit-clear alias */
#define PWM_DIV_INT_1		(0x10U)		/* divider 1.0 (8.4 fixed) */
#define PWM_SLICE_OF(pin)	(((pin) >> 1U) & 0x07U)
#define PWM_SLICE_DIV(slice)	(PWM_BASE + PWM_CHx_DIV + ((slice) * 0x14U))

/* Duty 1000 must give a pin that is high all the time: CC > TOP */
#define MOTOR_CC_FULL_ON	(MOTOR_PWM_TOP + 1U)

typedef struct
{
	uint32_t	pin_pos;
	uint32_t	pin_neg;
	uint32_t	b_invert;
} motor_hw_t;

static const motor_hw_t g_motor_hw[MOTOR_COUNT] =
{
	{ MOTOR_LEFT_PIN_POS,  MOTOR_LEFT_PIN_NEG,  MOTOR_LEFT_INVERT  },
	{ MOTOR_RIGHT_PIN_POS, MOTOR_RIGHT_PIN_NEG, MOTOR_RIGHT_INVERT }
};

/*
 * The kernel leaves the PWM block in reset unless its physical-timer
 * option is on (config.h USE_PTMR).  Release it here, but only if it is
 * still in reset: resetting it again would stop any PWM another subsystem
 * (e.g. the ultrasonic servo) has already started.
 */
static void
motor_pwm_unreset (void)
{
	if ((in_w(RESETS_RESET) & RESETS_RESET_PWM) != 0U)
	{
		out_w(RESETS_RESET + REG_ALIAS_CLR_OFFSET, RESETS_RESET_PWM);
		while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0U)
		{
			/* wait: takes a few clock cycles */
		}
	}
}

static uint32_t
motor_duty_to_cc (uint32_t duty_permille)
{
	if (duty_permille >= (uint32_t)MOTOR_DUTY_MAX)
	{
		return MOTOR_CC_FULL_ON;
	}
	return (duty_permille * (MOTOR_PWM_TOP + 1U)) / (uint32_t)MOTOR_DUTY_MAX;
}

void
motor_init (void)
{
	uint32_t	idx;

	motor_pwm_unreset();

	for (idx = 0U; idx < (uint32_t)MOTOR_COUNT; idx++)
	{
		const motor_hw_t * const p_hw = &g_motor_hw[idx];

		/* Both pins of a motor share one PWM slice (slice = pin / 2) */
		out_w(PWM_SLICE_DIV(PWM_SLICE_OF(p_hw->pin_pos)), PWM_DIV_INT_1);
		(void)pwm_set_wrap(p_hw->pin_pos, MOTOR_PWM_TOP);
		(void)pwm_set_cc(p_hw->pin_pos, 0U);
		(void)pwm_set_cc(p_hw->pin_neg, 0U);
		(void)pwm_set_pin(p_hw->pin_pos);
		(void)pwm_set_pin(p_hw->pin_neg);
		(void)pwm_set_enabled(p_hw->pin_pos, TRUE);
	}
}

void
motor_set_duty (motor_id_t id, int32_t duty_permille)
{
	const motor_hw_t	*p_hw;
	uint32_t		cc;
	int32_t			duty = duty_permille;

	if ((uint32_t)id >= (uint32_t)MOTOR_COUNT)
	{
		return;
	}
	p_hw = &g_motor_hw[id];

	if (p_hw->b_invert != 0U)
	{
		duty = -duty;
	}

	if (duty >= 0)
	{
		cc = motor_duty_to_cc((uint32_t)duty);
		(void)pwm_set_cc(p_hw->pin_neg, 0U);
		(void)pwm_set_cc(p_hw->pin_pos, cc);
	}
	else
	{
		cc = motor_duty_to_cc((uint32_t)(-duty));
		(void)pwm_set_cc(p_hw->pin_pos, 0U);
		(void)pwm_set_cc(p_hw->pin_neg, cc);
	}
}

void
motor_brake (motor_id_t id)
{
	if ((uint32_t)id < (uint32_t)MOTOR_COUNT)
	{
		(void)pwm_set_cc(g_motor_hw[id].pin_pos, MOTOR_CC_FULL_ON);
		(void)pwm_set_cc(g_motor_hw[id].pin_neg, MOTOR_CC_FULL_ON);
	}
}

void
motor_coast (motor_id_t id)
{
	if ((uint32_t)id < (uint32_t)MOTOR_COUNT)
	{
		(void)pwm_set_cc(g_motor_hw[id].pin_pos, 0U);
		(void)pwm_set_cc(g_motor_hw[id].pin_neg, 0U);
	}
}
