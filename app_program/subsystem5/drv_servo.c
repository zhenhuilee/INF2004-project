#include "drv_servo.h"
#include "subsystem5.h"

#include <stdint.h>
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define SERVO_MIN_US  500
#define SERVO_MAX_US  2500

static uint servo_slice;

void servo_init(void)
{
    gpio_set_function(S5_SERVO_PIN, GPIO_FUNC_PWM);

    servo_slice = pwm_gpio_to_slice_num(S5_SERVO_PIN);

    pwm_config config = pwm_get_default_config();

    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, 19999);

    pwm_init(servo_slice, &config, true);

    servo_set_angle(90);
}

void servo_set_angle(int angle)
{
    if (angle < 0) {
        angle = 0;
    }

    if (angle > 180) {
        angle = 180;
    }

    uint32_t pulse =
        SERVO_MIN_US +
        ((SERVO_MAX_US - SERVO_MIN_US) * angle / 180);

    pwm_set_gpio_level(S5_SERVO_PIN, pulse);
}