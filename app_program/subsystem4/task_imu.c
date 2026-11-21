/*
 * task_imu.c
 * Subsystem 4 -- continuous IMU motion, terrain, hump and pickup detection.
 */

#include "subsystem4.h"
#include "drv_imu.h"
#include "../subsystem2/motion.h"

#define S4_UPHILL_THRESHOLD              -700
#define S4_PEAK_RISE_THRESHOLD           1500
#define S4_DOWNHILL_RISE_THRESHOLD       2200
#define S4_FLAT_RETURN_THRESHOLD         1000
#define S4_CONFIRM_SAMPLES               5

/* Empirical calibration: 4385 Y-axis counts measured for a 10 mm reference hump. */
#define S4_HEIGHT_REFERENCE_COUNTS       4385
#define S4_HEIGHT_REFERENCE_MM           10

#define S4_MOTION_STATIONARY             0
#define S4_MOTION_ACCELERATING           1
#define S4_MOTION_UNKNOWN                0xFF

#define S4_ACCEL_CHANGE_THRESHOLD        500000UL
#define S4_ACCEL_CONFIRM_SAMPLES         1
#define S4_STATIONARY_CONFIRM_SAMPLES    4

/* Pickup is detected from a sustained filtered orientation change.
 * Raw sample deltas are too sensitive to motor vibration. */
#define S4_LIFT_TILT_THRESHOLD_DEG       12
#define S4_LIFT_Y_CHANGE_THRESHOLD       2200
#define S4_LIFT_CONFIRM_SAMPLES          4
#define S4_LIFT_STARTUP_GRACE_SAMPLES   200

#define S4_RUNTIME_FILTER_SAMPLES        10
#define S4_RUNTIME_SAMPLE_DELAY_MS       10

#define S4_TURN_CHANGE_THRESHOLD         3
#define S4_TURN_CONFIRM_SAMPLES          2
#define S4_TURN_END_SAMPLES              3
#define S4_TURN_RATE_REPORT_SAMPLES      5

static S4_IMU_STATUS imu_status;
EXPORT S4_MOTION_STATUS s4_motion_status;

static UB motion_state;
static IMU_DATA previous_motion_sample;
static BOOL motion_sample_valid;
static INT acceleration_count;
static INT stationary_count;

static BOOL lifted_active;
static BOOL safety_stop_latched;
static INT lift_count;
static INT lift_debug_count;

static INT previous_heading;
static BOOL heading_sample_valid;
static BOOL turning_active;
static INT turning_count;
static INT turn_end_count;
static INT turn_rate_report_count;

static INT current_hump_height_mm;
static INT highest_hump_height_mm;

static INT abs_int(INT value)
{
    return (value < 0) ? -value : value;
}

static UW sample_change_squared(IMU_DATA *previous, IMU_DATA *current)
{
    INT dx = current->x - previous->x;
    INT dy = current->y - previous->y;
    INT dz = current->z - previous->z;

    return (UW)(dx * dx) + (UW)(dy * dy) + (UW)(dz * dz);
}

static INT approximate_angle_deg(INT y, INT z)
{
    INT abs_y = abs_int(y);
    INT abs_z = abs_int(z);
    INT angle;

    if (abs_y == 0 && abs_z == 0)
        return 0;

    if (abs_y <= abs_z)
        angle = (45 * abs_y) / abs_z;
    else
        angle = 90 - ((45 * abs_z) / abs_y);

    if (z >= 0)
        return (y >= 0) ? angle : -angle;

    return (y >= 0) ? (180 - angle) : (-180 + angle);
}

static void update_imu_status(S4_TERRAIN_STATE state, BOOL fault)
{
    S4_IMU_STATUS new_status = (S4_IMU_STATUS)(state & S4_TERRAIN_BITS);

    if (fault)
        new_status |= S4_IMU_FAULT_BIT;

    if (new_status != imu_status)
    {
        imu_status = new_status;
        tm_printf(
            (UB*)"imu: STATUS CHANGE = 0x%02X terrain=%d fault=%d\n",
            imu_status,
            imu_status & S4_TERRAIN_BITS,
            (imu_status & S4_IMU_FAULT_BIT) ? 1 : 0
        );
    }
}

static void update_motion_status(UB bit, BOOL active)
{
    S4_MOTION_STATUS new_status = s4_motion_status;

    if (active)
        new_status |= bit;
    else
        new_status &= (S4_MOTION_STATUS)(~bit);

    new_status &= S4_MOTION_BITS;

    if (new_status != s4_motion_status)
    {
        s4_motion_status = new_status;
        tm_printf(
            (UB*)"imu: MOTION STATUS = 0x%02X accelerating=%d turning=%d lifted=%d\n",
            s4_motion_status,
            (s4_motion_status & S4_MOTION_ACCEL_BIT) ? 1 : 0,
            (s4_motion_status & S4_MOTION_TURN_BIT) ? 1 : 0,
            (s4_motion_status & S4_MOTION_LIFT_BIT) ? 1 : 0
        );
    }
}

static void motion_classifier_reset(void)
{
    motion_state = S4_MOTION_UNKNOWN;
    motion_sample_valid = FALSE;
    acceleration_count = 0;
    stationary_count = 0;
}

static void report_motion_state(UB new_state)
{
    if (new_state == motion_state)
        return;

    motion_state = new_state;

    if (new_state == S4_MOTION_ACCELERATING)
    {
        update_motion_status(S4_MOTION_ACCEL_BIT, TRUE);
        tm_printf((UB*)"imu: MOTION = ACCELERATING\n");
    }
    else
    {
        update_motion_status(S4_MOTION_ACCEL_BIT, FALSE);
        tm_printf((UB*)"imu: MOTION = STATIONARY\n");
    }
}

static void update_motion_classifier(IMU_DATA *current)
{
    UW sample_change;

    if (!motion_sample_valid)
    {
        previous_motion_sample = *current;
        motion_sample_valid = TRUE;
        return;
    }

    sample_change = sample_change_squared(&previous_motion_sample, current);
    previous_motion_sample = *current;

    if (sample_change > S4_ACCEL_CHANGE_THRESHOLD)
    {
        acceleration_count++;
        stationary_count = 0;

        if (acceleration_count >= S4_ACCEL_CONFIRM_SAMPLES)
        {
            report_motion_state(S4_MOTION_ACCELERATING);
            acceleration_count = S4_ACCEL_CONFIRM_SAMPLES;
        }
    }
    else
    {
        stationary_count++;
        acceleration_count = 0;

        if (stationary_count >= S4_STATIONARY_CONFIRM_SAMPLES)
        {
            report_motion_state(S4_MOTION_STATIONARY);
            stationary_count = S4_STATIONARY_CONFIRM_SAMPLES;
        }
    }
}

static void lift_detector_reset(void)
{
    lifted_active = FALSE;
    safety_stop_latched = FALSE;
    lift_count = 0;
    lift_debug_count = 0;
    update_motion_status(S4_MOTION_LIFT_BIT, FALSE);
}

static void update_lift_detector(IMU_DATA *current, INT flat_y)
{
    INT tilt_deg;
    INT y_change;
    int32_t stop_result;

    if (safety_stop_latched)
        return;

    tilt_deg = abs_int(approximate_angle_deg(current->y, current->z));
    y_change = abs_int(current->y - flat_y);

    /* Temporary diagnostics: approximately one line per second, plus
     * extra readings when either lift criterion is close to triggering. */
    lift_debug_count++;
    if (lift_debug_count >= 10 ||
        tilt_deg >= S4_LIFT_TILT_THRESHOLD_DEG ||
        y_change >= S4_LIFT_Y_CHANGE_THRESHOLD)
    {
        tm_printf((UB*)"imu: LIFT CHECK tilt=%d y_change=%d confirm=%d/4\n",
                  tilt_deg, y_change, lift_count);
        lift_debug_count = 0;
    }

    if (tilt_deg >= S4_LIFT_TILT_THRESHOLD_DEG &&
        y_change >= S4_LIFT_Y_CHANGE_THRESHOLD)
    {
        lift_count++;

        if (lift_count >= S4_LIFT_CONFIRM_SAMPLES)
        {
            lifted_active = TRUE;
            safety_stop_latched = TRUE;
            lift_count = 0;

            update_motion_status(S4_MOTION_LIFT_BIT, TRUE);
            tm_printf(
                (UB*)"imu: MOTION EVENT = LIFTED tilt=%d y_change=%d\n",
                tilt_deg,
                y_change
            );

            stop_result = motion_stop();
            tm_printf((UB*)"imu: SAFETY STOP REQUEST result=%ld\n",
                      (long)stop_result);
        }
    }
    else
    {
        lift_count = 0;
    }
}

static INT heading_change_deg(INT previous, INT current)
{
    INT change = current - previous;

    if (change > 180)
        change -= 360;
    else if (change < -180)
        change += 360;

    return change;
}

static void turning_detector_reset(void)
{
    previous_heading = 0;
    heading_sample_valid = FALSE;
    turning_active = FALSE;
    turning_count = 0;
    turn_end_count = 0;
    turn_rate_report_count = 0;
}

static void update_turning_detector(IMU_DATA *mag_data)
{
    INT current_heading;
    INT change;
    INT abs_change;
    INT turn_rate;

    current_heading = approximate_angle_deg(mag_data->y, mag_data->x);

    if (!heading_sample_valid)
    {
        previous_heading = current_heading;
        heading_sample_valid = TRUE;
        return;
    }

    change = heading_change_deg(previous_heading, current_heading);
    previous_heading = current_heading;
    abs_change = abs_int(change);
    turn_rate = abs_change * 10;

    if (abs_change >= S4_TURN_CHANGE_THRESHOLD)
    {
        turning_count++;
        turn_end_count = 0;

        if (!turning_active && turning_count >= S4_TURN_CONFIRM_SAMPLES)
        {
            turning_active = TRUE;
            turn_rate_report_count = 0;
            update_motion_status(S4_MOTION_TURN_BIT, TRUE);
            tm_printf((UB*)"imu: MOTION EVENT = TURNING\n");
            tm_printf((UB*)"imu: TURN RATE = %d deg/s\n", turn_rate);
        }
        else if (turning_active)
        {
            turn_rate_report_count++;

            if (turn_rate_report_count >= S4_TURN_RATE_REPORT_SAMPLES)
            {
                tm_printf((UB*)"imu: TURN RATE = %d deg/s\n", turn_rate);
                turn_rate_report_count = 0;
            }
        }

        if (turning_count > S4_TURN_CONFIRM_SAMPLES)
            turning_count = S4_TURN_CONFIRM_SAMPLES;
    }
    else
    {
        turning_count = 0;

        if (turning_active)
        {
            turn_end_count++;

            if (turn_end_count >= S4_TURN_END_SAMPLES)
            {
                turning_active = FALSE;
                turn_end_count = 0;
                turn_rate_report_count = 0;
                update_motion_status(S4_MOTION_TURN_BIT, FALSE);
                tm_printf((UB*)"imu: MOTION EVENT = TURN END\n");
            }
        }
    }
}

static INT estimate_hump_height_mm(INT flat_y, INT minimum_y)
{
    INT excursion = flat_y - minimum_y;

    if (excursion <= 0)
        return 0;

    return (excursion * S4_HEIGHT_REFERENCE_MM +
            (S4_HEIGHT_REFERENCE_COUNTS / 2)) /
           S4_HEIGHT_REFERENCE_COUNTS;
}

static void report_hump_height(INT flat_y, INT minimum_y)
{
    current_hump_height_mm = estimate_hump_height_mm(flat_y, minimum_y);

    if (current_hump_height_mm > highest_hump_height_mm)
        highest_hump_height_mm = current_hump_height_mm;

    tm_printf(
        (UB*)"imu: HUMP HEIGHT = %d mm highest=%d mm\n",
        current_hump_height_mm,
        highest_hump_height_mm
    );
}

EXPORT S4_IMU_STATUS s4_get_imu_status(void)
{
    return imu_status;
}

EXPORT INT s4_get_current_hump_height_mm(void)
{
    return current_hump_height_mm;
}

EXPORT INT s4_get_highest_hump_height_mm(void)
{
    return highest_hump_height_mm;
}

EXPORT BOOL s4_is_safety_stop_latched(void)
{
    return safety_stop_latched;
}

EXPORT void imu_task(INT stacd, void *exinf)
{
    IMU_DATA imu_data;
    IMU_DATA raw_data;
    IMU_DATA mag_data;
    S4_TERRAIN_STATE state = S4_TERRAIN_FLAT;

    INT flat_y;
    INT lowest_y;
    INT y_change;
    INT transition_count = 0;
    INT lift_startup_grace_samples = S4_LIFT_STARTUP_GRACE_SAMPLES;
    INT runtime_samples = 0;
    int32_t runtime_x_sum = 0;
    int32_t runtime_y_sum = 0;
    int32_t runtime_z_sum = 0;

    (void)stacd;
    (void)exinf;

    imu_status = 0xFF;
    s4_motion_status = 0;
    current_hump_height_mm = 0;
    highest_hump_height_mm = 0;

    if (!imu_init())
    {
        update_imu_status(S4_TERRAIN_FLAT, TRUE);
        tm_printf((UB*)"imu: initialisation failed\n");
        return;
    }

    if (!imu_magnetometer_init())
    {
        update_imu_status(S4_TERRAIN_FLAT, TRUE);
        tm_printf((UB*)"imu: magnetometer initialisation failed\n");
        return;
    }

    tm_printf((UB*)"imu: keep robot flat for calibration\n");

    if (!imu_calibrate())
    {
        update_imu_status(S4_TERRAIN_FLAT, TRUE);
        tm_printf((UB*)"imu: calibration failed\n");
        return;
    }

    tk_dly_tsk(500);

    if (!imu_read_raw_filtered(&raw_data))
    {
        update_imu_status(S4_TERRAIN_FLAT, TRUE);
        tm_printf((UB*)"imu: flat baseline capture failed\n");
        return;
    }

    flat_y = raw_data.y;
    lowest_y = flat_y;

    motion_classifier_reset();
    lift_detector_reset();
    turning_detector_reset();
    update_imu_status(S4_TERRAIN_FLAT, FALSE);

    tm_printf((UB*)"imu: BASELINE READY y=%d\n", flat_y);

    while (1)
    {
        if (!imu_read(&raw_data))
        {
            update_imu_status(state, TRUE);
            tk_dly_tsk(S4_RUNTIME_SAMPLE_DELAY_MS);
            continue;
        }

        if (lift_startup_grace_samples > 0)
        {
            lift_startup_grace_samples--;

            if (lift_startup_grace_samples == 0)
                tm_printf((UB*)"imu: PICKUP SAFETY ARMED\n");
        }

        runtime_x_sum += raw_data.x;
        runtime_y_sum += raw_data.y;
        runtime_z_sum += raw_data.z;
        runtime_samples++;

        if (runtime_samples < S4_RUNTIME_FILTER_SAMPLES)
        {
            tk_dly_tsk(S4_RUNTIME_SAMPLE_DELAY_MS);
            continue;
        }

        imu_data.x = (int16_t)(runtime_x_sum / runtime_samples);
        imu_data.y = (int16_t)(runtime_y_sum / runtime_samples);
        imu_data.z = (int16_t)(runtime_z_sum / runtime_samples);

        runtime_x_sum = 0;
        runtime_y_sum = 0;
        runtime_z_sum = 0;
        runtime_samples = 0;

        if (lift_startup_grace_samples == 0)
            update_lift_detector(&imu_data, flat_y);

        if (safety_stop_latched)
        {
            tk_dly_tsk(S4_RUNTIME_SAMPLE_DELAY_MS);
            continue;
        }

        update_motion_classifier(&imu_data);

        if (imu_read_magnetometer(&mag_data))
            update_turning_detector(&mag_data);

        y_change = imu_data.y - flat_y;

        switch (state)
        {
        case S4_TERRAIN_FLAT:
            if (y_change < S4_UPHILL_THRESHOLD)
            {
                transition_count++;

                if (transition_count >= S4_CONFIRM_SAMPLES)
                {
                    state = S4_TERRAIN_UPHILL;
                    lowest_y = imu_data.y;
                    transition_count = 0;
                    update_imu_status(state, FALSE);
                    tm_printf((UB*)"imu: TERRAIN = UPHILL\n");
                }
            }
            else
            {
                transition_count = 0;
            }
            break;

        case S4_TERRAIN_UPHILL:
            if (imu_data.y < lowest_y)
                lowest_y = imu_data.y;

            if ((imu_data.y - lowest_y) > S4_PEAK_RISE_THRESHOLD)
            {
                transition_count++;

                if (transition_count >= S4_CONFIRM_SAMPLES)
                {
                    report_hump_height(flat_y, lowest_y);
                    state = S4_TERRAIN_PEAK;
                    transition_count = 0;
                    update_imu_status(state, FALSE);
                    tm_printf((UB*)"imu: TERRAIN = PEAK\n");
                }
            }
            else
            {
                transition_count = 0;
            }
            break;

        case S4_TERRAIN_PEAK:
            if ((imu_data.y - lowest_y) > S4_DOWNHILL_RISE_THRESHOLD)
            {
                transition_count++;

                if (transition_count >= S4_CONFIRM_SAMPLES)
                {
                    state = S4_TERRAIN_DOWNHILL;
                    transition_count = 0;
                    update_imu_status(state, FALSE);
                    tm_printf((UB*)"imu: TERRAIN = DOWNHILL\n");
                }
            }
            else
            {
                transition_count = 0;
            }
            break;

        case S4_TERRAIN_DOWNHILL:
            if (y_change > -S4_FLAT_RETURN_THRESHOLD &&
                y_change < S4_FLAT_RETURN_THRESHOLD)
            {
                transition_count++;

                if (transition_count >= S4_CONFIRM_SAMPLES)
                {
                    state = S4_TERRAIN_FLAT;
                    lowest_y = flat_y;
                    transition_count = 0;
                    update_imu_status(state, FALSE);
                    tm_printf(
                        (UB*)"imu: TERRAIN = FLAT hump complete highest=%d mm\n",
                        highest_hump_height_mm
                    );
                }
            }
            else
            {
                transition_count = 0;
            }
            break;

        default:
            state = S4_TERRAIN_FLAT;
            lowest_y = flat_y;
            transition_count = 0;
            update_imu_status(state, TRUE);
            break;
        }

        tk_dly_tsk(S4_RUNTIME_SAMPLE_DELAY_MS);
    }
}
