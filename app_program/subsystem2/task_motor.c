#include "subsystem2.h"
#include "drv_motor.h"

#define S2_PATH_CLEAR     0
#define S2_PATH_OBSTACLE  1
#define S2_PATH_UNKNOWN   2

static volatile INT obstacle_status = S2_PATH_UNKNOWN;

/* Called by Buddy 5 to update the obstacle status */
EXPORT void s2_set_obstacle_status(INT status)
{
    if (status == S2_PATH_CLEAR ||
        status == S2_PATH_OBSTACLE ||
        status == S2_PATH_UNKNOWN)
    {
        obstacle_status = status;
    }
}

/* Buddy 2 motor control task */
EXPORT void motor_task(INT stacd, void *exinf)
{
    INT current_status;
    INT previous_status = -1;

    (void)stacd;
    (void)exinf;

    motor_init();
    motor_set_speed(50, 50);
    motor_stop();

    tm_printf("Buddy 2 Motor Task Started\n");

    while (1)
    {
        current_status = obstacle_status;

        if (current_status != previous_status)
        {
            if (current_status == S2_PATH_CLEAR)
            {
                tm_printf("Path clear - Moving forward\n");
                motor_forward();
            }
            else if (current_status == S2_PATH_OBSTACLE)
            {
                tm_printf("Obstacle detected - STOP\n");
                motor_stop();
            }
            else
            {
                tm_printf("Path unknown - Staying stopped\n");
                motor_stop();
            }

            previous_status = current_status;
        }

        tk_dly_tsk(100);
    }
}