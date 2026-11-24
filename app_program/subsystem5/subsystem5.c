/*
- subsystem5.c
- Subsystem 5 -- Ultrasonic Obstacle Sensor & Width Estimator
 */

#include "subsystem5.h"
#include "subsystem5_config.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"

#ifndef SCAN_MIN_ANGLE
#define SCAN_MIN_ANGLE      50
#endif

#ifndef SCAN_MAX_ANGLE
#define SCAN_MAX_ANGLE      190
#endif

#ifndef SCAN_STEP_ANGLE
#define SCAN_STEP_ANGLE     5
#endif

#ifndef S5_SCAN_DELAY_MS
#define S5_SCAN_DELAY_MS    25
#endif

#ifndef S5_DISTANCE_LIMIT
#define S5_DISTANCE_LIMIT   25.0f
#endif

#ifndef S5_CLEARANCE_CM
#define S5_CLEARANCE_CM     12.0f
#endif

/* HC-SR04 valid range and background-rejection band */
#define MIN_VALID_CM        2.0f
#define MAX_VALID_CM        400.0f
#define BAND_TOLERANCE_CM   25.0f
#define LOOP_PACE_MS        20 //from 40
#define S5_PI_F             3.14159265f

static bool g_obstacle_blocked = false;
static S5_OBSTACLE_INFO g_obs_info = {0};

static int s5_abs_int(int val)
{
    return (val < 0) ? -val : val;
}

static float s5_fabs(float val)
{
    return (val < 0.0f) ? -val : val;
}

/* tan(x) for small/moderate half-angles (x in radians): x + x^3/3 + 2*x^5/15 */
static float s5_tan_rad(float x)
{
    float x2 = x * x;
    float x3 = x2 * x;
    float x5 = x3 * x2;
    return x + (x3 / 3.0f) + ((2.0f * x5) / 15.0f);
}

static int next_scan_angle(int current, int *step, bool *wrapped_end)
{
    int next = current + *step;
    *wrapped_end = false;

    if (next >= SCAN_MAX_ANGLE)
    {
        next = SCAN_MAX_ANGLE;
        *step = -SCAN_STEP_ANGLE;
        *wrapped_end = true;
    }
    else if (next <= SCAN_MIN_ANGLE)
    {
        next = SCAN_MIN_ANGLE;
        *step = SCAN_STEP_ANGLE;
        *wrapped_end = true;
    }

    return next;
}

static float estimate_width_cm(float dist_cm, float span_deg)
{
    float adjusted_span = span_deg;

    /* Mild compensation for ultrasonic beam cone widening */
    if (adjusted_span > 10.0f)
    {
        adjusted_span -= 8.0f;
    }

    if (dist_cm <= 0.0f || adjusted_span <= 0.0f)
    {
        return 0.0f;
    }

    {
        float half_rad = (adjusted_span * 0.5f) * (S5_PI_F / 180.0f);
        float width = 2.0f * dist_cm * s5_tan_rad(half_rad);
        return (width > 0.0f) ? width : 0.0f;
    }
}

static void obstacle_scan_task(INT stacd, void *extra)
{
    int angle = SCAN_MIN_ANGLE;
    int step = SCAN_STEP_ANGLE;

    bool collecting_sweep = false;
    bool hit_seen = false;
    int first_hit_angle = 0;
    int last_hit_angle = 0;
    float anchor_dist = 0.0f;
    float dist_sum = 0.0f;
    int hit_count = 0;

    (void)stacd;
    (void)extra;

    ultrasonic_init();
    servo_init();

    tm_printf("\nSubsystem 5: Ultrasonic Sensor Task Started\n");

    while (1)
    {
        bool wrapped_end = false;
        float distance;
        bool valid_reading;

        servo_set_angle(angle);

        if (S5_SCAN_DELAY_MS > 0)
        {
            tk_dly_tsk(S5_SCAN_DELAY_MS);
        }

        distance = ultrasonic_get_distance();
        valid_reading = (distance >= MIN_VALID_CM && distance <= MAX_VALID_CM);

        if (valid_reading && distance < S5_DISTANCE_LIMIT)
        {
            g_obstacle_blocked = true;

            if (!collecting_sweep)
            {
                collecting_sweep = true;
                hit_seen = true;
                first_hit_angle = angle;
                last_hit_angle = angle;
                anchor_dist = distance;
                dist_sum = distance;
                hit_count = 1;
            }
            else if (s5_fabs(distance - anchor_dist) <= BAND_TOLERANCE_CM)
            {
                if (!hit_seen)
                {
                    first_hit_angle = angle;
                    hit_seen = true;
                }
                last_hit_angle = angle;
                dist_sum += distance;
                hit_count++;
            }
        }
        else if (valid_reading)
        {
            /* Only clear global blocked flag once obstacle info has been consumed */
            if (!g_obs_info.valid)
            {
                g_obstacle_blocked = false;
            }
        }

        angle = next_scan_angle(angle, &step, &wrapped_end);

        /* Finalize width estimate at sweep turnaround once hits were captured */
        if (wrapped_end && collecting_sweep && hit_count >= 1)
        {
            float avg_dist = dist_sum / (float)hit_count;
            float span = (float)s5_abs_int(last_hit_angle - first_hit_angle);

            /* If only 1 sample hit, assume one step width so bypass still works */
            if (span < (float)SCAN_STEP_ANGLE)
            {
                span = (float)SCAN_STEP_ANGLE;
            }

            g_obs_info.distance_cm      = avg_dist;
            g_obs_info.span_deg         = span;
            g_obs_info.est_width_cm     = estimate_width_cm(avg_dist, span);
            g_obs_info.bypass_offset_cm = g_obs_info.est_width_cm + S5_CLEARANCE_CM;
            g_obs_info.valid            = true;

            tm_printf("[S5] d=%d cm | span=%d deg | est_w=%d cm | offset=%d cm\n",
                      (int)g_obs_info.distance_cm,
                      (int)g_obs_info.span_deg,
                      (int)g_obs_info.est_width_cm,
                      (int)g_obs_info.bypass_offset_cm);

            collecting_sweep = false;
            hit_seen = false;
            dist_sum = 0.0f;
            hit_count = 0;
        }
        else if (wrapped_end && collecting_sweep)
        {
            collecting_sweep = false;
            hit_seen = false;
            dist_sum = 0.0f;
            hit_count = 0;
        }

        tk_dly_tsk(LOOP_PACE_MS);
    }
}

bool subsystem5_is_blocked(void)
{
    return g_obstacle_blocked;
}

bool subsystem5_get_obstacle_info(S5_OBSTACLE_INFO *out_info)
{
    if (out_info == NULL || !g_obs_info.valid)
    {
        return false;
    }
    *out_info = g_obs_info;
    return true;
}

void subsystem5_clear_obstacle_info(void)
{
    g_obs_info.valid = false;
    g_obstacle_blocked = false;
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