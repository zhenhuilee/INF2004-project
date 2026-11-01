#include "drv_servo.h"
#include "subsystem5_config.h"

#include <stdint.h>

/* Servo pulse range */
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

/* GP8 = PWM slice 4, channel A */
#define SERVO_PWM_BASE PWM_CH4_BASE

#define REG32(addr) (*(volatile uint32_t *)(addr))

void servo_init(void)
{
    /* Select PWM function for GP8 */
    REG32(GPIO_CTRL(S5_SERVO_PIN)) =
        GPIO_CTRL_FUNCSEL_PWM;

    /* Reset PWM slice 4 counter */
    REG32(SERVO_PWM_BASE + PWM_CHx_CTR) = 0;

    /* Clock divider: 125 MHz / 125 = 1 MHz */
    REG32(SERVO_PWM_BASE + PWM_CHx_DIV) = (125u << 4);

    /* 20,000 counts gives 50 Hz */
    REG32(SERVO_PWM_BASE + PWM_CHx_TOP) = 19999;

    /* Enable PWM slice 4 */
    REG32(PWM_EN) |= (1u << 4);

    servo_set_angle(90);
}

void servo_set_angle(int angle)
{
    uint32_t pulse;

    if (angle < 0)
        angle = 0;

    if (angle > 180)
        angle = 180;

    pulse = SERVO_MIN_US +
        ((SERVO_MAX_US - SERVO_MIN_US) *
        (uint32_t)angle / 180u);

    /* GP8 is channel A: lower 16 bits of CC */
    REG32(SERVO_PWM_BASE + PWM_CHx_CC) = pulse;
}
