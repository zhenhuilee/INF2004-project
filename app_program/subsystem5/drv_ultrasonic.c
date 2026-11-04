
#include "subsystem5.h"
#include "subsystem5_config.h"
#include "drv_ultrasonic.h"
#include "../subsystem2/subsystem2.h"
#include <stdint.h>

#define REG32(addr) (*(volatile uint32_t *)(addr))

#define TRIG_MASK (1u << S5_TRIG_PIN)
#define ECHO_MASK (1u << S5_ECHO_PIN)

#define ECHO_TIMEOUT_US 30000u
#define OBSTACLE_DISTANCE_CM 30.0f

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

    /* Keep Buddy 2 stopped until a valid reading is received */
    s2_set_obstacle_status(S2_PATH_UNKNOWN);
}

float ultrasonic_get_distance(void)
{
    uint32_t start;
    uint32_t pulse_start;
    uint32_t pulse_end;
    float distance;

    /* Send ultrasonic trigger pulse */
    REG32(GPIO_OUT_CLR) = TRIG_MASK;
    delay_us(2);

    REG32(GPIO_OUT_SET) = TRIG_MASK;
    delay_us(10);
    REG32(GPIO_OUT_CLR) = TRIG_MASK;

    /* Wait for echo to become HIGH */
    start = timer_us();

    while ((REG32(GPIO_IN) & ECHO_MASK) == 0)
    {
        if ((uint32_t)(timer_us() - start) > ECHO_TIMEOUT_US)
        {
            s2_set_obstacle_status(S2_PATH_UNKNOWN);
            return -1.0f;
        }
    }

    /* Measure echo pulse duration */
    pulse_start = timer_us();

    while ((REG32(GPIO_IN) & ECHO_MASK) != 0)
    {
        if ((uint32_t)(timer_us() - pulse_start) > ECHO_TIMEOUT_US)
        {
            s2_set_obstacle_status(S2_PATH_UNKNOWN);
            return -1.0f;
        }
    }

    pulse_end = timer_us();

    /* Convert echo duration to distance in centimetres */
    distance = (float)(pulse_end - pulse_start) / 58.0f;

    /* Send obstacle status to Buddy 2 */
    if (distance <= OBSTACLE_DISTANCE_CM)
    {
        tm_printf("Obstacle detected: %d cm\n", (INT)distance);

        s2_set_obstacle_status(S2_PATH_OBSTACLE);
    }
    else
    {
        tm_printf("Path clear: %d cm\n", (INT)distance);

        s2_set_obstacle_status(S2_PATH_CLEAR);
    }

    return distance;
}
