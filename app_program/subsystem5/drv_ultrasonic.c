#include "drv_ultrasonic.h"
#include "subsystem5_config.h"

#include <stdint.h>

#define SIO_BASE_ADDR 0xD0000000UL
#define IO_BANK0_BASE_ADDR 0x40014000UL

#define GPIO_CTRL(gpio) \
    (IO_BANK0_BASE_ADDR + 0x04 + ((gpio) * 8))

#define REG32(addr) (*(volatile uint32_t *)(addr))

#define SIO_GPIO_IN  (SIO_BASE_ADDR + 0x04)
#define SIO_GPIO_OUT_SET (SIO_BASE_ADDR + 0x14)
#define SIO_GPIO_OUT_CLR (SIO_BASE_ADDR + 0x18)
#define SIO_GPIO_OE_SET (SIO_BASE_ADDR + 0x24)
#define SIO_GPIO_OE_CLR (SIO_BASE_ADDR + 0x28)

static void delay_us(uint32_t us)
{
    volatile uint32_t i;

    while (us--)
    {
        for (i = 0; i < 30; i++)
        {
            __asm volatile ("nop");
        }
    }
}

void ultrasonic_init(void)
{
    REG32(GPIO_CTRL(S5_TRIG_PIN)) = 5;
    REG32(GPIO_CTRL(S5_ECHO_PIN)) = 5;

    REG32(SIO_GPIO_OE_SET) = 1u << S5_TRIG_PIN;
    REG32(SIO_GPIO_OE_CLR) = 1u << S5_ECHO_PIN;

    REG32(SIO_GPIO_OUT_CLR) = 1u << S5_TRIG_PIN;
}

float ultrasonic_get_distance(void)
{
    uint32_t timeout;
    uint32_t pulse_length = 0;
    uint32_t mask = 1u << S5_ECHO_PIN;

    REG32(SIO_GPIO_OUT_CLR) = 1u << S5_TRIG_PIN;
    delay_us(2);

    REG32(SIO_GPIO_OUT_SET) = 1u << S5_TRIG_PIN;
    delay_us(10);
    REG32(SIO_GPIO_OUT_CLR) = 1u << S5_TRIG_PIN;

    timeout = 30000;

    while ((REG32(SIO_GPIO_IN) & mask) == 0)
    {
        if (--timeout == 0)
            return -1.0f;
    }

    timeout = 30000;

    while (REG32(SIO_GPIO_IN) & mask)
    {
        pulse_length++;

        if (--timeout == 0)
            return -1.0f;
    }

    return (float)pulse_length / 58.0f;
}
