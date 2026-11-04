#include "subsystem5.h"
#include "subsystem5_config.h"
#include "drv_ultrasonic.h"
#include <stdint.h>

#define REG32(addr) (*(volatile uint32_t *)(addr))

#define TRIG_MASK (1u << S5_TRIG_PIN)
#define ECHO_MASK (1u << S5_ECHO_PIN)

#define ECHO_TIMEOUT_US 30000u

static uint32_t timer_us(void)
{
    return REG32(TIMER_TIMELR);
}

static void delay_us(uint32_t delay)
{
    uint32_t start = timer_us();

    while ((uint32_t)(timer_us() - start) < delay)
    {
        /* Wait */
    }
}

void ultrasonic_init(void)
{
    REG32(GPIO_CTRL(S5_TRIG_PIN)) = GPIO_CTRL_FUNCSEL_SIO;
    REG32(GPIO_CTRL(S5_ECHO_PIN)) = GPIO_CTRL_FUNCSEL_SIO;

    REG32(GPIO_OE_SET) = TRIG_MASK;
    REG32(GPIO_OE_CLR) = ECHO_MASK;

    REG32(GPIO_OUT_CLR) = TRIG_MASK;
}

float ultrasonic_get_distance(void)
{
    uint32_t start;
    uint32_t pulse_start;
    uint32_t pulse_end;

    REG32(GPIO_OUT_CLR) = TRIG_MASK;
    delay_us(2);

    REG32(GPIO_OUT_SET) = TRIG_MASK;
    delay_us(10);
    REG32(GPIO_OUT_CLR) = TRIG_MASK;

    start = timer_us();

    while ((REG32(GPIO_IN) & ECHO_MASK) == 0)
    {
        if ((uint32_t)(timer_us() - start) > ECHO_TIMEOUT_US)
            return -1.0f;
    }

    pulse_start = timer_us();

    while ((REG32(GPIO_IN) & ECHO_MASK) != 0)
    {
        if ((uint32_t)(timer_us() - pulse_start) > ECHO_TIMEOUT_US)
            return -1.0f;
    }

    pulse_end = timer_us();

    return (float)(pulse_end - pulse_start) / 58.0f;
}
