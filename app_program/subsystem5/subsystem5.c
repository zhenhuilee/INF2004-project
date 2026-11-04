#include "subsystem5.h"
#include "subsystem5_config.h"
#include "drv_ultrasonic.h"

static void obstacle_scan_task(INT stacd, void *extra)
{
    float distance;

    (void)stacd;
    (void)extra;

    ultrasonic_init();

    tm_printf("\nBuddy 5 Ultrasonic Test Started\n");

    while (1)
    {
        distance = ultrasonic_get_distance();

        if (distance < 0.0f)
        {
            tm_printf("No echo received\n");
        }
        else
        {
            tm_printf("Distance: %d cm\n", (int)distance);
        }

        tk_dly_tsk(500);
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
