#include "subsystem5.h"
#include "subsystem5_config.h"
#include "drv_servo.h"
#include <stdint.h>

#define REG32(addr) (*(volatile uint32_t *)(addr))

/* GP14 = PWM slice 7, channel A */
#define SERVO_PWM_BASE PWM_CH7_BASE

#define SERVO_MIN_US 500u
#define SERVO_MAX_US 2500u

void servo_init(void)
{
    /* Configure GP14 for PWM */
    REG32(GPIO_CTRL(S5_SERVO_PIN)) = GPIO_CTRL_FUNCSEL_PWM;

    /* Reset PWM counter */
    REG32(SERVO_PWM_BASE + PWM_CHx_CTR) = 0;

    /*
     * System clock: 125 MHz
     * Divider: 125
     * PWM clock: 1 MHz
     */
    REG32(SERVO_PWM_BASE + PWM_CHx_DIV) = (125u << 4);

    /*
     * 20,000 counts = 20 ms period
     * 50 Hz servo frequency
     */
    REG32(SERVO_PWM_BASE + PWM_CHx_TOP) = 19999u;

    /* Initial pulse width: 1500 us (90 degrees) */
    REG32(SERVO_PWM_BASE + PWM_CHx_CC) = 1500u;

    /* Enable PWM slice 7 */
    REG32(PWM_EN) |= (1u << 7);

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
        ((uint32_t)angle * (SERVO_MAX_US - SERVO_MIN_US)) / 180u;

    REG32(SERVO_PWM_BASE + PWM_CHx_CC) = pulse;
}
