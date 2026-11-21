#include "subsystem5.h"
#include "subsystem5_config.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"

#define S5_OBSTACLE_DETECT_CM 30
#define S5_OBSTACLE_CLEAR_CM 35
#define S5_SCAN_SETTLE_MS     300
#define S5_READING_COUNT     3

static volatile BOOL obstacle_blocked = FALSE;
static volatile INT obstacle_width_mm = 0;

static const INT scan_angles[S5_SCAN_COUNT] =
{
    30, 60, 90, 120, 150
};

/* Convert milliseconds to T-Kernel delay units */
static void s5_delay(INT milliseconds)
{
    tk_dly_tsk(milliseconds);
}

/* Obtain median of three distance readings */
static float median_distance(float a, float b, float c)
{
    float temp;

    if (a > b) {
        temp = a;
        a = b;
        b = temp;
    }

    if (b > c) {
        temp = b;
        b = c;
        c = temp;
    }

    if (a > b) {
        temp = a;
        a = b;
        b = temp;
    }

    return b;
}

/* Take multiple readings at the current servo position */
static float s5_read_stable_distance(void)
{
    float a, b, c;

    a = ultrasonic_get_distance();
    s5_delay(60);

    b = ultrasonic_get_distance();
    s5_delay(60);

    c = ultrasonic_get_distance();

    if (a < 0.0f || b < 0.0f || c < 0.0f) {
        return -1.0f;
    }

    return median_distance(a, b, c);
}

/*
 * Estimate lateral obstacle width.
 *
 * Servo angle 90 degrees is assumed to point straight forward.
 * The table contains sin(angle - 90 degrees), scaled by 1000.
 *
 * This is an approximate scan-width estimate, not a guaranteed
 * physical measurement of the complete obstacle.
 */
static INT s5_estimate_width(void)
{
    static const INT sin_scaled[S5_SCAN_COUNT] =
    {
        -866, -500, 0, 500, 866
    };

    float distance;
    float x;
    float min_x = 100000.0f;
    float max_x = -100000.0f;

    INT valid_points = 0;
    INT i;

    for (i = 0; i < S5_SCAN_COUNT; i++) {

        servo_set_angle(scan_angles[i]);
        s5_delay(S5_SCAN_SETTLE_MS);

        distance = s5_read_stable_distance();

        if (distance < 0.0f ||
            distance > S5_DISTANCE_LIMIT) {
            continue;
        }

        /* Convert cm to mm */
        x = (distance * 10.0f *
             (float)sin_scaled[i]) / 1000.0f;

        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;

        valid_points++;
        
        tm_printf("[S5] Angle:%d Distance:%d cm\n",
                  scan_angles[i], (INT)distance);
    }

    servo_set_angle(90);

    if (valid_points < 2) {
        return 0;
    }

    return (INT)(max_x - min_x);
}

/* Public interface used by Buddy 3 */
EXPORT BOOL subsystem5_is_blocked(void)
{
    return obstacle_blocked;
}

/* Public interface for estimated obstacle width */
EXPORT INT subsystem5_get_obstacle_width(void)
{
    return obstacle_width_mm;
}

static void obstacle_scan_task(INT stacd, void *extra)
{
    float distance;

    (void)stacd;
    (void)extra;

    ultrasonic_init();
    servo_init();

    servo_set_angle(90);
    s5_delay(500);

    tm_printf("\n[S5] Obstacle scanning task started\n");

    while (1) {

        /* Check the forward direction */
        servo_set_angle(90);
        s5_delay(150);

        distance = s5_read_stable_distance();

        if (distance < 0.0f) {

            /* Unknown distance: keep the robot stopped */
            obstacle_blocked = TRUE;

            tm_printf("[S5] No echo - keeping robot stopped\n");
        }
        else if (distance <= S5_OBSTACLE_DETECT_CM) {

            if (!obstacle_blocked) {

                obstacle_blocked = TRUE;

                tm_printf("\n[S5] OBSTACLE DETECTED: %d cm\n",
                          (INT)distance);

                /* Scan only once when a new obstacle is detected */
                obstacle_width_mm = s5_estimate_width();

                tm_printf("[S5] Estimated width: %d mm\n",
                          obstacle_width_mm);
            }
        }
        else if (distance > S5_OBSTACLE_CLEAR_CM) {

            if (obstacle_blocked) {

                obstacle_blocked = FALSE;

                obstacle_width_mm = 0;

                tm_printf("[S5] Forward path clear\n");
            }
        }

        tm_printf("[S5] Forward distance: %d cm | Blocked: %d\n",
                  (INT)distance, (INT)obstacle_blocked);

        s5_delay(200);
    }
}

EXPORT ER subsystem5_init(void)
{
    obstacle_blocked = FALSE;
    obstacle_width_mm = 0;

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
