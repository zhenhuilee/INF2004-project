#include "drv_motor.h"

#define PWM_TOP 62499

void motor_init(void)
{
    // Configure direction pins
    gpio_set_pin(MOTOR_A_IN1, GPIO_MODE_OUT);
    gpio_set_pin(MOTOR_A_IN2, GPIO_MODE_OUT);
    gpio_set_pin(MOTOR_B_IN1, GPIO_MODE_OUT);
    gpio_set_pin(MOTOR_B_IN2, GPIO_MODE_OUT);

    // Set direction pins LOW initially
    motor_stop();

    // Configure PWM pins
    pwm_set_pin(MOTOR_A_EN);
    pwm_set_pin(MOTOR_B_EN);

    // Set PWM frequency to approximately 2 kHz
    pwm_set_wrap(MOTOR_A_EN, PWM_TOP);

    // Initially set motor speed to 0
    pwm_set_cc(MOTOR_A_EN, 0);
    pwm_set_cc(MOTOR_B_EN, 0);

    // Enable PWM
    pwm_set_enabled(MOTOR_A_EN, TRUE);
}

void motor_set_speed(UINT speedA, UINT speedB)
{
    if (speedA > 100) speedA = 100;
    if (speedB > 100) speedB = 100;

    pwm_set_cc(MOTOR_A_EN, (PWM_TOP + 1) * speedA / 100);
    pwm_set_cc(MOTOR_B_EN, (PWM_TOP + 1) * speedB / 100);
}

void motor_forward(void)
{
    gpio_set_val(MOTOR_A_IN1, 1);
    gpio_set_val(MOTOR_A_IN2, 0);

    gpio_set_val(MOTOR_B_IN1, 1);
    gpio_set_val(MOTOR_B_IN2, 0);
}

void motor_backward(void)
{
    gpio_set_val(MOTOR_A_IN1, 0);
    gpio_set_val(MOTOR_A_IN2, 1);

    gpio_set_val(MOTOR_B_IN1, 0);
    gpio_set_val(MOTOR_B_IN2, 1);
}

void motor_stop(void)
{
    gpio_set_val(MOTOR_A_IN1, 0);
    gpio_set_val(MOTOR_A_IN2, 0);

    gpio_set_val(MOTOR_B_IN1, 0);
    gpio_set_val(MOTOR_B_IN2, 0);
}

/* Turn right: Motor A forward, Motor B backward */
void motor_turn_right(void)
{
    gpio_set_val(MOTOR_A_IN1, 1);
    gpio_set_val(MOTOR_A_IN2, 0);

    gpio_set_val(MOTOR_B_IN1, 0);
    gpio_set_val(MOTOR_B_IN2, 1);
}

/* Turn left: Motor A backward, Motor B forward */
void motor_turn_left(void)
{
    gpio_set_val(MOTOR_A_IN1, 0);
    gpio_set_val(MOTOR_A_IN2, 1);

    gpio_set_val(MOTOR_B_IN1, 1);
    gpio_set_val(MOTOR_B_IN2, 0);
}