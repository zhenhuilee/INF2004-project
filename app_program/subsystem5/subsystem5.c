/*
 * subsystem5.c
 * Subsystem 5 -- Ultrasonic Obstacle Sensor Provider
 */

#include "subsystem5.h"
#include "subsystem5_config.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"
#include <stdint.h>
#include <stdbool.h>

static bool g_obstacle_blocked = false;

static void obstacle_scan_task(INT stacd, void *extra)
{
    float distance;

    (void)stacd;
    (void)extra;

    ultrasonic_init();
    servo_init();

    tm_printf("\nSubsystem 5: Ultrasonic Sensor Task Started\n");

    while (1)
    {
        distance = ultrasonic_get_distance();

        if (distance > 0.0f && distance < S5_DISTANCE_LIMIT)
        {
            if (!g_obstacle_blocked) {
                tm_printf("[S5] Obstacle detected at %d cm!\n", (int)distance);
            }
            g_obstacle_blocked = true;
        }
        else
        {
            if (g_obstacle_blocked) {
                tm_printf("[S5] Path clear.\n");
            }
            g_obstacle_blocked = false;
        }

        tk_dly_tsk(100); /* Sample every 100 ms */
    }
}

bool subsystem5_is_blocked(void)
{
    return g_obstacle_blocked;
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