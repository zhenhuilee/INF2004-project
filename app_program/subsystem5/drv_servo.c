#include "subsystem5.h"
#include "subsystem5_config.h"
#include "drv_servo.h"
#include <stdint.h>

#define S5_REG32(addr) (*(volatile uint32_t *)(addr))

/* RP2040 hardware register addresses */
#define S5_RESETS_BASE       0x4000c000u
#define S5_RESETS_RESET      (S5_RESETS_BASE + 0x00u)
#define S5_RESETS_RESET_CLR  (S5_RESETS_BASE + 0x3000u)
#define S5_RESETS_DONE       (S5_RESETS_BASE + 0x08u)

#define S5_RESET_IO_BANK0    (1u << 5)
#define S5_RESET_PADS_BANK0  (1u << 6)
#define S5_RESET_PWM         (1u << 14)

#define S5_IO_BANK0_BASE     0x40014000u
#define S5_PADS_BANK0_BASE   0x4001c000u
#define S5_PWM_BASE          0x40050000u

#define S5_GPIO_CTRL(pin)    (S5_IO_BANK0_BASE + 0x04u + ((uint32_t)(pin) * 8u))
#define S5_GPIO_FUNC_PWM     4u

#define S5_PAD_GPIO(pin)     (S5_PADS_BANK0_BASE + 0x04u + ((uint32_t)(pin) * 4u))
#define S5_PAD_OD_BIT        (1u << 7)
#define S5_PAD_IE_BIT        (1u << 6)

/* Slice 7 corresponds to GP14 (channel A) and GP15 (channel B) */
#define S5_PWM_SLICE         7u
#define S5_PWM_CH7_BASE      (S5_PWM_BASE + (S5_PWM_SLICE * 0x14u))
#define S5_PWM_CSR           (S5_PWM_CH7_BASE + 0x00u)
#define S5_PWM_DIV           (S5_PWM_CH7_BASE + 0x04u)
#define S5_PWM_CTR           (S5_PWM_CH7_BASE + 0x08u)
#define S5_PWM_CC            (S5_PWM_CH7_BASE + 0x0cu)
#define S5_PWM_TOP           (S5_PWM_CH7_BASE + 0x10u)
#define S5_PWM_EN            (S5_PWM_BASE + 0xa0u)

#define SERVO_MIN_US         500u
#define SERVO_MAX_US         2500u

static void servo_apply_pulse_us(uint32_t pulse_us)
{
    uint32_t cc = S5_REG32(S5_PWM_CC);

    if (((uint32_t)S5_SERVO_PIN & 1u) == 0u)
    {
        cc = (cc & 0xffff0000u) | (pulse_us & 0x0000ffffu);
    }
    else
    {
        cc = (cc & 0x0000ffffu) | ((pulse_us & 0x0000ffffu) << 16);
    }

    S5_REG32(S5_PWM_CC) = cc;
}

void servo_init(void)
{
    uint32_t unreset_mask = S5_RESET_IO_BANK0 | S5_RESET_PADS_BANK0 | S5_RESET_PWM;

    /* Clear reset bits for IO_BANK0, PADS_BANK0, and PWM if held in reset */
    S5_REG32(S5_RESETS_RESET_CLR) = unreset_mask;
    while ((S5_REG32(S5_RESETS_DONE) & unreset_mask) != unreset_mask)
    {
        /* Wait for hardware reset release */
    }

    /* Enable output (clear OD) and input enable (set IE) on servo pad */
    uint32_t pad = S5_REG32(S5_PAD_GPIO(S5_SERVO_PIN));
    pad &= ~S5_PAD_OD_BIT;
    pad |= S5_PAD_IE_BIT;
    S5_REG32(S5_PAD_GPIO(S5_SERVO_PIN)) = pad;

    /* Route pin to PWM peripheral (FUNCSEL = 4) */
    S5_REG32(S5_GPIO_CTRL(S5_SERVO_PIN)) = S5_GPIO_FUNC_PWM;

    /* Configure slice 7 for 50 Hz (1 us tick, 20000 ticks = 20 ms) */
    S5_REG32(S5_PWM_EN) &= ~(1u << S5_PWM_SLICE);
    S5_REG32(S5_PWM_CSR) = 0u;
    S5_REG32(S5_PWM_CTR) = 0u;
    S5_REG32(S5_PWM_DIV) = (125u << 4);
    S5_REG32(S5_PWM_TOP) = 19999u;

    servo_apply_pulse_us(1500u);

    S5_REG32(S5_PWM_CSR) = 1u;
    S5_REG32(S5_PWM_EN) |= (1u << S5_PWM_SLICE);

    tm_printf("servo_init: ctrl=0x%x pad=0x%x csr=0x%x cc=0x%x en=0x%x\n",
              (unsigned int)S5_REG32(S5_GPIO_CTRL(S5_SERVO_PIN)),
              (unsigned int)S5_REG32(S5_PAD_GPIO(S5_SERVO_PIN)),
              (unsigned int)S5_REG32(S5_PWM_CSR),
              (unsigned int)S5_REG32(S5_PWM_CC),
              (unsigned int)S5_REG32(S5_PWM_EN));
}

void servo_set_angle(int angle)
{
    uint32_t pulse;

    if (angle < 0)
        angle = 0;

    if (angle > 180)
        angle = 180;

    pulse = SERVO_MIN_US +
        (((uint32_t)angle * (SERVO_MAX_US - SERVO_MIN_US)) / 180u);

    servo_apply_pulse_us(pulse);
}