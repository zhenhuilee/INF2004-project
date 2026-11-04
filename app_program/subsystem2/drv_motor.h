#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include "subsystem2.h"

#define MOTOR_A_IN1 9
#define MOTOR_A_IN2 8

#define MOTOR_B_IN1 11
#define MOTOR_B_IN2 10

#define MOTOR_A_EN 12
#define MOTOR_B_EN 13

void motor_init(void);
void motor_set_speed(UINT speedA, UINT speedB);
void motor_forward(void);
void motor_backward(void);
void motor_stop(void);
void motor_turn_right(void);
void motor_turn_left(void);

#endif 