#include "drv_ultrasonic.h"
#include "subsystem5.h"

#include "hardware/gpio.h"
#include "pico/time.h"

#define ECHO_TIMEOUT_US 30000

void ultrasonic_init(void)
{
    gpio_init(S5_TRIG_PIN);
    gpio_set_dir(S5_TRIG_PIN, GPIO_OUT);
    gpio_put(S5_TRIG_PIN, 0);

    gpio_init(S5_ECHO_PIN);
    gpio_set_dir(S5_ECHO_PIN, GPIO_IN);
}

float ultrasonic_get_distance(void)
{
    uint32_t start_time;
    uint32_t end_time;

    gpio_put(S5_TRIG_PIN, 0);
    busy_wait_us(2);

    gpio_put(S5_TRIG_PIN, 1);
    busy_wait_us(10);
    gpio_put(S5_TRIG_PIN, 0);

    start_time = time_us_32();

    while (gpio_get(S5_ECHO_PIN) == 0) {
        if ((uint32_t)(time_us_32() - start_time) > ECHO_TIMEOUT_US)
            return -1.0f;
    }

    start_time = time_us_32();

    while (gpio_get(S5_ECHO_PIN) == 1) {
        if ((uint32_t)(time_us_32() - start_time) > ECHO_TIMEOUT_US)
            return -1.0f;
    }

    end_time = time_us_32();

    return (float)(end_time - start_time) / 58.0f;
}
