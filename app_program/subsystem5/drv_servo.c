#include "drv_servo.h"
#include "subsystem5_config.h"

#include <stdint.h>

/* RP2040 register addresses */
#define SIO_BASE_ADDR       0xD0000000UL
#define IO_BANK0_BASE_ADDR  0x40014000UL
#define PADS_BANK0_BASE_ADDR 0x4001C000UL
#define PWM_BASE_ADDR       0x40050000UL
#define RESETS_BASE_ADDR    0x4000C000UL

#define GPIO_CTRL(gpio) \
    (IO_BANK0_BASE_ADDR + 0x04 + ((gpio) * 8))

#define PAD_GPIO(gpio) \
    (PADS_BANK0_BASE_ADDR + 0x04 + ((gpio) * 4))

#define PWM_CH7_CSR  (PWM_BASE_ADDR + 0x70)
#define PWM_CH7_DIV  (PWM_BASE_ADDR + 0x74)
#define PWM_CH7_TOP  (PWM_BASE_ADDR + 0x78)
#define PWM_CH7_CC   (PWM_BASE_ADDR + 0x7C)

#define REG32(addr) (*(volatile uint32_t *)(addr))

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

void servo_init(void)
{
    /* Enable PWM function on the servo GPIO */
    REG32(GPIO_CTRL(S5_SERVO_PIN)) = 4;

    /* Enable PWM peripheral slice 3 (GP6/GP7) or slice 4 (GP8/GP9) */
    /* GP8 uses PWM slice 4, channel A */
    REG32(RESETS_BASE_ADDR + 0x0C) &= ~(1u << 14);

    while ((REG32(RESETS_BASE_ADDR + 0x08) & (1u << 14)) == 0)
    {
        /* Wait for PWM peripheral reset to complete */
    }

    REG32(PWM_CH7_DIV) = 125u << 4;
    REG32(PWM_CH7_TOP) = 19999;
    REG32(PWM_CH7_CSR) = 1;

    servo_set_angle(90);
}

void servo_set_angle(int angle)
{
    uint32_t pulse;

    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    pulse = SERVO_MIN_US +
        ((SERVO_MAX_US - SERVO_MIN_US) * (uint32_t)angle / 180u);

    REG32(PWM_CH7_CC) = pulse;
}
