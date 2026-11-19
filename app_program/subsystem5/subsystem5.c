#include "subsystem5.h"
#include "subsystem5_config.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"

#define SCAN_MIN_ANGLE   30
#define SCAN_MAX_ANGLE   150
#define SCAN_STEP_ANGLE  30

/* Minimum valid HC-SR04 distance in cm to reject short/noisy echo glitches */
#define MIN_VALID_CM     2.0f

/* Pace loop so other subsystems and serial monitor run smoothly */
#define LOOP_PACE_MS     150u

static int next_scan_angle(int current, int *step)
{
    int next = current + *step;

    if (next >= SCAN_MAX_ANGLE)
    {
        next = SCAN_MAX_ANGLE;
        *step = -SCAN_STEP_ANGLE;
    }
    else if (next <= SCAN_MIN_ANGLE)
    {
        next = SCAN_MIN_ANGLE;
        *step = SCAN_STEP_ANGLE;
    }

    return next;
}

static void obstacle_scan_task(INT stacd, void *extra)
{
    int angle = SCAN_MIN_ANGLE;
    int step = SCAN_STEP_ANGLE;

    (void)stacd;
    (void)extra;

    ultrasonic_init();
    servo_init();

    tm_printf("\nBuddy 5 Ultrasonic Test Started\n");

    while (1)
    {
        servo_set_angle(angle);

        if (S5_SCAN_DELAY_MS > 0)
        {
            tk_dly_tsk(S5_SCAN_DELAY_MS);
        }

        float distance = ultrasonic_get_distance();

        if (distance < 0.0f)
        {
            tm_printf("Angle %d deg: No echo\n", angle);
        }
        else if (distance < MIN_VALID_CM)
        {
            tm_printf("Angle %d deg: Short echo (%d cm)\n", angle, (int)distance);
        }
        else if (distance > S5_DISTANCE_LIMIT)
        {
            tm_printf("Angle %d deg: Out of range (%d cm)\n", angle, (int)distance);
        }
        else
        {
            tm_printf("Angle %d deg: %d cm\n", angle, (int)distance);
        }

        angle = next_scan_angle(angle, &step);
        tk_dly_tsk(LOOP_PACE_MS);
    }
}

EXPORT ER subsystem5_init(void)
{
    return E_OK;
}

EXPORT ER subsystem5_start(void)
{
    ID tid;

    tid = app_start_task(
        (FP)obstacle_scan_task,
        S5_PRI_MAIN,
        0,
        "obstacle_scan"
    );

    return (tid < E_OK) ? tid : E_OK;
}