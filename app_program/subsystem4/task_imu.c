/*
 *	task_imu.c
 *	Subsystem 4 -- continuous IMU motion, terrain and tilt detection.
 */

#include "subsystem4.h"
#include "drv_imu.h"
#include "../subsystem2/motion.h"

#define S4_UPHILL_THRESHOLD			-700
#define S4_PEAK_RISE_THRESHOLD			1500
#define S4_DOWNHILL_RISE_THRESHOLD		2200
#define S4_FLAT_RETURN_THRESHOLD		1000
#define S4_CONFIRM_SAMPLES			3
#define S4_PROGRESS_INTERVAL			5

/*
 * Empirical hump-height calibration from three trials using a 10 mm
 * reference hump.  Mean measured Y excursion was 4385 counts.
 */
#define S4_HEIGHT_REFERENCE_COUNTS		4385
#define S4_HEIGHT_REFERENCE_MM			10

/*
 * Encoder + IMU height estimation.
 * motion_get_status() supplies travelled distance in mm.  Relative IMU
 * pitch supplies the slope angle.  Height is accumulated as ds * sin(theta).
 * Integer sine values are scaled by 1000 to avoid floating point.
 */
#define S4_SIN_SCALE				1000
#define S4_MAX_HEIGHT_TILT_DEG			45

#define S4_MOTION_STATIONARY			0
#define S4_MOTION_ACCELERATING			1
#define S4_MOTION_UNKNOWN			0xFF

/*
 * Measured stationary maximum sample change:
 * 34,709.
 *
 * 500,000 provides margin above stationary noise while remaining
 * well below the measured movement maximum of 891,226,609.
 */
#define S4_ACCEL_CHANGE_THRESHOLD		500000UL
#define S4_ACCEL_CONFIRM_SAMPLES		1
#define S4_STATIONARY_CONFIRM_SAMPLES		4

#define S4_LIFT_Y_THRESHOLD			-1800
#define S4_LIFT_CLEAR_THRESHOLD			-700
#define S4_LIFT_CONFIRM_SAMPLES			3
#define S4_LIFT_CLEAR_SAMPLES			3
#define S4_RUNTIME_FILTER_SAMPLES		10
#define S4_RUNTIME_SAMPLE_DELAY_MS		10

#define S4_TURN_CHANGE_THRESHOLD		3
#define S4_TURN_CONFIRM_SAMPLES		2
#define S4_TURN_END_SAMPLES			3
#define S4_TURN_RATE_REPORT_SAMPLES		5

#define S4_MOTION_LIFT_BIT			S4_MOTION_IMPACT_BIT

static S4_IMU_STATUS imu_status;
EXPORT S4_MOTION_STATUS s4_motion_status;

static UB motion_state;
static IMU_DATA previous_motion_sample;
static BOOL motion_sample_valid;
static INT acceleration_count;
static INT stationary_count;

static BOOL lifted_active;
static BOOL hump_complete;
static INT lift_count;
static INT lift_clear_count;

static INT previous_heading;
static BOOL heading_sample_valid;
static BOOL turning_active;
static INT turning_count;
static INT turn_end_count;
static INT turn_rate_report_count;

static INT current_hump_height_mm;
static INT highest_hump_height_mm;

static BOOL encoder_height_active;
static int32_t encoder_height_mm_x1000;
static int32_t encoder_height_peak_mm_x1000;
static int32_t encoder_previous_odometer_mm;

typedef struct
{
	INT min_x;
	INT max_x;
	INT min_y;
	INT max_y;
	INT min_z;
	INT max_z;
	INT samples;
} S4_RAW_RANGE;

static INT abs_int(INT value)
{
	if (value < 0)
	{
		return -value;
	}

	return value;
}

static UW sample_change_squared(
	IMU_DATA *previous,
	IMU_DATA *current
)
{
	INT delta_x;
	INT delta_y;
	INT delta_z;

	UW x_squared;
	UW y_squared;
	UW z_squared;

	delta_x = current->x - previous->x;
	delta_y = current->y - previous->y;
	delta_z = current->z - previous->z;

	x_squared = (UW)(delta_x * delta_x);
	y_squared = (UW)(delta_y * delta_y);
	z_squared = (UW)(delta_z * delta_z);

	return x_squared + y_squared + z_squared;
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
	{
		return;
	}

	motion_state = new_state;

	if (motion_state == S4_MOTION_ACCELERATING)
	{
		update_motion_status(S4_MOTION_ACCEL_BIT, TRUE);

		tm_printf(
			(UB*)"imu: MOTION = ACCELERATING\n"
		);
	}
	else
	{
		update_motion_status(S4_MOTION_ACCEL_BIT, FALSE);

		tm_printf(
			(UB*)"imu: MOTION = STATIONARY\n"
		);
	}
}

static void update_motion_classifier(IMU_DATA *current)
{
	UW sample_change;

	if (!motion_sample_valid)
	{
		previous_motion_sample = *current;
		motion_sample_valid = TRUE;
		acceleration_count = 0;
		stationary_count = 0;
		return;
	}

	sample_change = sample_change_squared(
		&previous_motion_sample,
		current
	);

	previous_motion_sample = *current;

	if (sample_change > S4_ACCEL_CHANGE_THRESHOLD)
	{
		acceleration_count++;
		stationary_count = 0;

		if (acceleration_count >=
		    S4_ACCEL_CONFIRM_SAMPLES)
		{
			report_motion_state(
				S4_MOTION_ACCELERATING
			);

			acceleration_count =
				S4_ACCEL_CONFIRM_SAMPLES;
		}
	}
	else
	{
		stationary_count++;
		acceleration_count = 0;

		if (stationary_count >=
		    S4_STATIONARY_CONFIRM_SAMPLES)
		{
			report_motion_state(
				S4_MOTION_STATIONARY
			);

			stationary_count =
				S4_STATIONARY_CONFIRM_SAMPLES;
		}
	}
}

static void lift_detector_reset(void)
{
	lifted_active = FALSE;
	hump_complete = FALSE;
	lift_count = 0;
	lift_clear_count = 0;
	update_motion_status(S4_MOTION_LIFT_BIT, FALSE);
}

static void update_lift_detector(
	IMU_DATA *current,
	INT flat_y,
	S4_TERRAIN_STATE terrain_state
)
{
	INT y_change;

	if (terrain_state != S4_TERRAIN_FLAT || !hump_complete)
	{
		lift_count = 0;
		return;
	}

	y_change = current->y - flat_y;

	if (!lifted_active)
	{
		if (y_change < S4_LIFT_Y_THRESHOLD)
		{
			lift_count++;

			if (lift_count >= S4_LIFT_CONFIRM_SAMPLES)
			{
				lifted_active = TRUE;
				lift_count = 0;
				lift_clear_count = 0;

				update_motion_status(
					S4_MOTION_LIFT_BIT,
					TRUE
				);

				tm_printf(
					(UB*)"imu: MOTION EVENT = LIFTED "
					"y_change=%d\n",
					y_change
				);

				(void)motion_stop();

				tm_printf(
					(UB*)"imu: SAFETY STOP REQUEST = 1\n"
				);
			}
		}
		else
		{
			lift_count = 0;
		}

		return;
	}

	if (y_change > S4_LIFT_CLEAR_THRESHOLD)
	{
		lift_clear_count++;

		if (lift_clear_count >= S4_LIFT_CLEAR_SAMPLES)
		{
			lifted_active = FALSE;
			lift_count = 0;
			lift_clear_count = 0;

			update_motion_status(
				S4_MOTION_LIFT_BIT,
				FALSE
			);

			tm_printf(
				(UB*)"imu: MOTION EVENT = LIFT CLEARED\n"
			);

			tm_printf(
				(UB*)"imu: SAFETY STOP REQUEST = 0\n"
			);
		}
	}
	else
	{
		lift_clear_count = 0;
	}
}

static INT approximate_angle_deg(INT y, INT z)
{
	INT abs_y;
	INT abs_z;
	INT angle;

	abs_y = abs_int(y);
	abs_z = abs_int(z);

	if (abs_y == 0 && abs_z == 0)
	{
		return 0;
	}

	if (abs_y <= abs_z)
	{
		angle = (45 * abs_y) / abs_z;
	}
	else
	{
		angle =
			90 - ((45 * abs_z) / abs_y);
	}

	if (z >= 0)
	{
		if (y >= 0)
		{
			return angle;
		}

		return -angle;
	}

	if (y >= 0)
	{
		return 180 - angle;
	}

	return -180 + angle;
}

static INT relative_tilt_deg(
	INT y,
	INT z,
	INT flat_angle
)
{
	INT tilt;

	tilt =
		approximate_angle_deg(y, z) -
		flat_angle;

	if (tilt > 180)
	{
		tilt -= 360;
	}
	else if (tilt < -180)
	{
		tilt += 360;
	}

	return tilt;
}

static void print_tilt(
	const char *stage,
	IMU_DATA *raw_data,
	INT flat_angle
)
{
	INT angle;
	INT tilt;

	angle = approximate_angle_deg(
		raw_data->y,
		raw_data->z
	);

	tilt = relative_tilt_deg(
		raw_data->y,
		raw_data->z,
		flat_angle
	);

	tm_printf(
		(UB*)"\n%s TILT\n",
		stage
	);

	tm_printf(
		(UB*)"RAW-G: X=%d Y=%d Z=%d\n",
		raw_data->x,
		raw_data->y,
		raw_data->z
	);

	tm_printf(
		(UB*)"Sensor angle=%d deg\n",
		angle
	);

	tm_printf(
		(UB*)"Relative tilt=%d deg\n",
		tilt
	);
}

static void update_imu_status(
	S4_TERRAIN_STATE state,
	BOOL fault
)
{
	S4_IMU_STATUS new_status;

	new_status =
		(S4_IMU_STATUS)
		(state & S4_TERRAIN_BITS);

	if (fault)
	{
		new_status |= S4_IMU_FAULT_BIT;
	}

	if (new_status != imu_status)
	{
		imu_status = new_status;

		tm_printf(
			(UB*)"imu: STATUS CHANGE = "
			"0x%02X terrain=%d fault=%d\n",
			imu_status,
			imu_status & S4_TERRAIN_BITS,
			(imu_status & S4_IMU_FAULT_BIT) ?
				1 : 0
		);
	}
}

static INT sine_milli(INT angle_deg)
{
	static const int16_t sine_table[] =
	{
		0, 17, 35, 52, 70, 87, 105, 122, 139, 156,
		174, 191, 208, 225, 242, 259, 276, 292, 309, 326,
		342, 358, 375, 391, 407, 423, 438, 454, 469, 485,
		500, 515, 530, 545, 559, 574, 588, 602, 616, 629,
		643, 656, 669, 682, 695, 707
	};

	if (angle_deg < 0)
	{
		angle_deg = -angle_deg;
	}

	if (angle_deg > S4_MAX_HEIGHT_TILT_DEG)
	{
		angle_deg = S4_MAX_HEIGHT_TILT_DEG;
	}

	return sine_table[angle_deg];
}

static void encoder_height_reset(void)
{
	encoder_height_active = FALSE;
	encoder_height_mm_x1000 = 0;
	encoder_height_peak_mm_x1000 = 0;
	encoder_previous_odometer_mm = 0;
}

static void encoder_height_start(void)
{
	motion_status_t status;

	motion_get_status(&status);

	encoder_height_mm_x1000 = 0;
	encoder_height_peak_mm_x1000 = 0;
	encoder_previous_odometer_mm = status.odometer_mm;
	encoder_height_active = TRUE;

	tm_printf(
		(UB*)"imu: ENCODER HEIGHT TRACKING START odometer=%ld mm\n",
		(long)status.odometer_mm
	);
}

static void encoder_height_update(
	IMU_DATA *current,
	INT flat_angle
)
{
	motion_status_t status;
	int32_t distance_mm;
	INT tilt_deg;
	INT sine_value;

	if (!encoder_height_active)
	{
		return;
	}

	motion_get_status(&status);

	distance_mm =
		status.odometer_mm -
		encoder_previous_odometer_mm;

	encoder_previous_odometer_mm =
		status.odometer_mm;

	if (distance_mm <= 0)
	{
		return;
	}

	tilt_deg = relative_tilt_deg(
		current->y,
		current->z,
		flat_angle
	);

	/*
	 * The mounting direction determines the sign of pitch.  Hump height
	 * needs the magnitude of the uphill slope, so sine_milli() uses the
	 * absolute angle.  Only the UPHILL terrain state calls this function.
	 */
	sine_value = sine_milli(tilt_deg);

	encoder_height_mm_x1000 +=
		distance_mm * sine_value;

	if (encoder_height_mm_x1000 >
	    encoder_height_peak_mm_x1000)
	{
		encoder_height_peak_mm_x1000 =
			encoder_height_mm_x1000;
	}
}

static INT encoder_height_get_mm(void)
{
	if (encoder_height_peak_mm_x1000 <= 0)
	{
		return 0;
	}

	return (INT)(
		(encoder_height_peak_mm_x1000 +
		 (S4_SIN_SCALE / 2)) /
		S4_SIN_SCALE
	);
}

static void report_encoder_hump_height(void)
{
	INT encoder_height_mm;

	encoder_height_mm =
		encoder_height_get_mm();

	if (encoder_height_mm > 0)
	{
		tm_printf(
			(UB*)"imu: ENCODER + IMU HUMP HEIGHT = %d mm\n",
			encoder_height_mm
		);

		if (encoder_height_mm > highest_hump_height_mm)
		{
			highest_hump_height_mm =
				encoder_height_mm;
		}
	}
	else
	{
		tm_printf(
			(UB*)"imu: ENCODER + IMU HUMP HEIGHT = unavailable\n"
		);
	}

	tm_printf(
		(UB*)"imu: HIGHEST HUMP = %d mm\n",
		highest_hump_height_mm
	);

	encoder_height_active = FALSE;
}

static INT estimate_hump_height_mm(
	INT flat_y,
	INT minimum_y
)
{
	INT excursion;

	excursion = flat_y - minimum_y;

	if (excursion <= 0)
	{
		return 0;
	}

	return (excursion * S4_HEIGHT_REFERENCE_MM +
		(S4_HEIGHT_REFERENCE_COUNTS / 2)) /
		S4_HEIGHT_REFERENCE_COUNTS;
}

static void report_hump_height(
	INT flat_y,
	INT minimum_y
)
{
	current_hump_height_mm =
		estimate_hump_height_mm(flat_y, minimum_y);

	if (current_hump_height_mm > highest_hump_height_mm)
	{
		highest_hump_height_mm = current_hump_height_mm;
	}

	tm_printf(
		(UB*)"imu: Y-CALIBRATED HUMP HEIGHT = %d mm\n",
		current_hump_height_mm
	);
}

static void raw_range_reset(S4_RAW_RANGE *range)
{
	range->min_x = 32767;
	range->max_x = -32768;
	range->min_y = 32767;
	range->max_y = -32768;
	range->min_z = 32767;
	range->max_z = -32768;
	range->samples = 0;
}

static void raw_range_update(
	S4_RAW_RANGE *range,
	IMU_DATA *data
)
{
	if (data->x < range->min_x)
	{
		range->min_x = data->x;
	}

	if (data->x > range->max_x)
	{
		range->max_x = data->x;
	}

	if (data->y < range->min_y)
	{
		range->min_y = data->y;
	}

	if (data->y > range->max_y)
	{
		range->max_y = data->y;
	}

	if (data->z < range->min_z)
	{
		range->min_z = data->z;
	}

	if (data->z > range->max_z)
	{
		range->max_z = data->z;
	}

	range->samples++;
}

static void raw_range_print(
	const char *stage,
	S4_RAW_RANGE *range
)
{
	if (range->samples == 0)
	{
		tm_printf(
			(UB*)"%s RAW-G RANGE: no samples\n",
			stage
		);

		return;
	}

	tm_printf(
		(UB*)"\n%s RAW-G RANGE\n",
		stage
	);

	tm_printf(
		(UB*)"samples=%d\n",
		range->samples
	);

	tm_printf(
		(UB*)"X: min=%d max=%d\n",
		range->min_x,
		range->max_x
	);

	tm_printf(
		(UB*)"Y: min=%d max=%d\n",
		range->min_y,
		range->max_y
	);

	tm_printf(
		(UB*)"Z: min=%d max=%d\n",
		range->min_z,
		range->max_z
	);
}

static void preparation_countdown(const char *action)
{
	INT i;

	tm_printf((UB*)"\n");

	tm_printf(
		(UB*)"========================================\n"
	);

	tm_printf(
		(UB*)"NEXT ACTION: %s\n",
		action
	);

	tm_printf(
		(UB*)"========================================\n"
	);

	tm_printf(
		(UB*)"DO NOT MOVE YET.\n"
	);

	tm_printf(
		(UB*)"You have 5 seconds to get ready.\n\n"
	);

	for (i = 5; i > 0; i--)
	{
		tm_printf(
			(UB*)"%d...\n",
			i
		);

		tk_dly_tsk(1000);
	}

	tm_printf((UB*)"\n");

	tm_printf(
		(UB*)"========================================\n"
	);

	tm_printf(
		(UB*)">>> GO NOW: %s <<<\n",
		action
	);

	tm_printf(
		(UB*)"MOVE SLOWLY.\n"
	);

	tm_printf(
		(UB*)"STOP AS SOON AS DETECTED APPEARS.\n"
	);

	tm_printf(
		(UB*)"========================================\n\n"
	);
}

static void print_waiting_message(
	S4_TERRAIN_STATE state
)
{
	switch (state)
	{
	case S4_TERRAIN_FLAT:
		tm_printf(
			(UB*)"\n[IMU] DETECTING UPHILL...\n"
		);

		tm_printf(
			(UB*)"[IMU] KEEP MOVING UPHILL\n"
		);
		break;

	case S4_TERRAIN_UPHILL:
		tm_printf(
			(UB*)"\n[IMU] DETECTING PEAK...\n"
		);

		tm_printf(
			(UB*)"[IMU] KEEP MOVING TOWARDS PEAK\n"
		);
		break;

	case S4_TERRAIN_PEAK:
		tm_printf(
			(UB*)"\n[IMU] DETECTING DOWNHILL...\n"
		);

		tm_printf(
			(UB*)"[IMU] KEEP MOVING DOWNHILL\n"
		);
		break;

	case S4_TERRAIN_DOWNHILL:
		tm_printf(
			(UB*)"\n[IMU] DETECTING FLAT...\n"
		);

		tm_printf(
			(UB*)"[IMU] KEEP RETURNING TO FLAT\n"
		);
		break;

	default:
		break;
	}
}

static void motion_verification_test(void)
{
	IMU_DATA data;
	INT sample;

	motion_classifier_reset();

	tm_printf((UB*)"\n");

	tm_printf(
		(UB*)"========================================\n"
	);

	tm_printf(
		(UB*)"MOTION CLASSIFICATION TEST\n"
	);

	tm_printf(
		(UB*)"========================================\n"
	);

	tm_printf(
		(UB*)"STEP 1: STATIONARY\n"
	);

	tm_printf(
		(UB*)"Keep IMU flat and completely still.\n\n"
	);

	for (sample = 0; sample < 20; sample++)
	{
		if (imu_read_filtered(&data))
		{
			update_motion_classifier(&data);
		}

		tk_dly_tsk(100);
	}

	tm_printf((UB*)"\n");

	tm_printf(
		(UB*)"========================================\n"
	);

	tm_printf(
		(UB*)"STEP 2: ACCELERATION\n"
	);

	tm_printf(
		(UB*)"KEEP IMU STILL DURING COUNTDOWN.\n"
	);

	tm_printf(
		(UB*)"ONLY MOVE AFTER GO NOW.\n"
	);

	tm_printf(
		(UB*)"========================================\n\n"
	);

	for (sample = 5; sample > 0; sample--)
	{
		tm_printf(
			(UB*)"%d...\n",
			sample
		);

		tk_dly_tsk(1000);
	}

	tm_printf((UB*)"\n");

	tm_printf(
		(UB*)"========================================\n"
	);

	tm_printf(
		(UB*)">>> GO NOW - SLIDE FORWARD/BACKWARD <<<\n"
	);

	tm_printf(
		(UB*)"CHANGE DIRECTION SEVERAL TIMES\n"
	);

	tm_printf(
		(UB*)"KEEP THE IMU APPROXIMATELY FLAT\n"
	);

	tm_printf(
		(UB*)"========================================\n\n"
	);

	for (sample = 0; sample < 60; sample++)
	{
		if (imu_read_filtered(&data))
		{
			update_motion_classifier(&data);
		}

		tk_dly_tsk(100);
	}

	tm_printf((UB*)"\n");

	tm_printf(
		(UB*)"========================================\n"
	);

	tm_printf(
		(UB*)">>> STOP MOVING NOW <<<\n"
	);

	tm_printf(
		(UB*)"KEEP IMU FLAT AND COMPLETELY STILL\n"
	);

	tm_printf(
		(UB*)"========================================\n\n"
	);

	for (sample = 0; sample < 30; sample++)
	{
		if (imu_read_filtered(&data))
		{
			update_motion_classifier(&data);
		}

		tk_dly_tsk(100);
	}

	tm_printf((UB*)"\n");

	tm_printf(
		(UB*)"========================================\n"
	);

	tm_printf(
		(UB*)"MOTION CLASSIFICATION TEST COMPLETE\n"
	);

	tm_printf(
		(UB*)"========================================\n\n"
	);
}


static INT heading_change_deg(INT previous, INT current)
{
	INT change = current - previous;

	if (change > 180)
	{
		change -= 360;
	}
	else if (change < -180)
	{
		change += 360;
	}

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

	current_heading =
		approximate_angle_deg(
			mag_data->y,
			mag_data->x
		);

	if (!heading_sample_valid)
	{
		previous_heading = current_heading;
		heading_sample_valid = TRUE;
		return;
	}

	change = heading_change_deg(
		previous_heading,
		current_heading
	);

	previous_heading = current_heading;
	abs_change = abs_int(change);
	turn_rate = abs_change * 10;

	if (abs_change >= S4_TURN_CHANGE_THRESHOLD)
	{
		turning_count++;
		turn_end_count = 0;

		if (!turning_active &&
		    turning_count >= S4_TURN_CONFIRM_SAMPLES)
		{
			turning_active = TRUE;
			turn_rate_report_count = 0;
			update_motion_status(S4_MOTION_TURN_BIT, TRUE);

			tm_printf(
				(UB*)"imu: MOTION EVENT = TURNING\n"
			);

			tm_printf(
				(UB*)"imu: TURN RATE = %d deg/s\n",
				turn_rate
			);
		}
		else if (turning_active)
		{
			turn_rate_report_count++;

			if (turn_rate_report_count >=
			    S4_TURN_RATE_REPORT_SAMPLES)
			{
				tm_printf(
					(UB*)"imu: TURN RATE = %d deg/s\n",
					turn_rate
				);

				turn_rate_report_count = 0;
			}
		}

		if (turning_count > S4_TURN_CONFIRM_SAMPLES)
		{
			turning_count = S4_TURN_CONFIRM_SAMPLES;
		}
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

				tm_printf(
					(UB*)"imu: MOTION EVENT = TURN END\n"
				);
			}
		}
	}
}

EXPORT void imu_task(
	INT stacd,
	void *exinf
)
{
	IMU_DATA imu_data;
	IMU_DATA raw_data;
	IMU_DATA mag_data;
	S4_RAW_RANGE raw_range;

	INT flat_y;
	INT y_change;
	INT lowest_y;
	INT flat_angle;

	INT transition_count = 0;
	INT progress_count = 0;
	INT i;
	INT runtime_samples = 0;
	int32_t runtime_x_sum = 0;
	int32_t runtime_y_sum = 0;
	int32_t runtime_z_sum = 0;

	S4_TERRAIN_STATE state =
		S4_TERRAIN_FLAT;

	(void)stacd;
	(void)exinf;

	imu_status = 0xFF;
	s4_motion_status = 0;
	current_hump_height_mm = 0;
	highest_hump_height_mm = 0;
	encoder_height_reset();

	if (!imu_init())
	{
		update_imu_status(
			S4_TERRAIN_FLAT,
			TRUE
		);

		tm_printf(
			(UB*)"imu: initialisation failed\n"
		);

		return;
	}

	if (!imu_magnetometer_init())
	{
		update_imu_status(
			S4_TERRAIN_FLAT,
			TRUE
		);

		tm_printf(
			(UB*)"imu: magnetometer initialisation failed\n"
		);

		return;
	}

	tm_printf((UB*)"\n");

	tm_printf(
		(UB*)"========================================\n"
	);

	tm_printf(
		(UB*)"IMU MOTION + HUMP + TILT TEST\n"
	);

	tm_printf(
		(UB*)"========================================\n"
	);

	tm_printf(
		(UB*)"FIRST: PLACE IMU COMPLETELY FLAT\n"
	);

	tm_printf(
		(UB*)"DO NOT MOVE IT DURING CALIBRATION\n\n"
	);

	tm_printf(
		(UB*)"Calibration starts in 5 seconds.\n"
	);

	tm_printf(
		(UB*)"Use this time to position the IMU.\n"
	);

	tm_printf(
		(UB*)"========================================\n\n"
	);

	for (i = 5; i > 0; i--)
	{
		tm_printf(
			(UB*)"%d...\n",
			i
		);

		tk_dly_tsk(1000);
	}

	tm_printf((UB*)"\n");

	tm_printf(
		(UB*)"========================================\n"
	);

	tm_printf(
		(UB*)">>> CALIBRATION STARTING NOW <<<\n"
	);

	tm_printf(
		(UB*)"DO NOT TOUCH OR MOVE THE IMU.\n"
	);

	tm_printf(
		(UB*)"========================================\n\n"
	);

	if (!imu_calibrate())
	{
		update_imu_status(
			S4_TERRAIN_FLAT,
			TRUE
		);

		tm_printf(
			(UB*)"imu: calibration failed\n"
		);

		return;
	}

	tk_dly_tsk(500);

	if (!imu_read_filtered(&imu_data))
	{
		update_imu_status(
			S4_TERRAIN_FLAT,
			TRUE
		);

		tm_printf(
			(UB*)"imu: flat baseline capture failed\n"
		);

		return;
	}

	flat_y = imu_data.y;
	lowest_y = flat_y;

	update_imu_status(
		S4_TERRAIN_FLAT,
		FALSE
	);

	motion_verification_test();


	/*
	 * Reset after the guided verification so normal operation begins
	 * with a fresh previous sample.
	 */
	motion_classifier_reset();

	raw_range_reset(&raw_range);

	if (!imu_read_raw_filtered(&raw_data))
	{
		update_imu_status(
			S4_TERRAIN_FLAT,
			TRUE
		);

		tm_printf(
			(UB*)"imu: raw flat reference capture failed\n"
		);

		return;
	}

	flat_angle = approximate_angle_deg(
		raw_data.y,
		raw_data.z
	);

	/*
	 * Runtime processing uses averaged raw samples.  Constant calibration
	 * offsets cancel in both terrain changes and consecutive-sample motion
	 * changes, so the raw flat reference is the matching runtime baseline.
	 */
	flat_y = raw_data.y;
	lift_detector_reset();
	turning_detector_reset();

	tm_printf((UB*)"\n");

	tm_printf(
		(UB*)"========================================\n"
	);

	tm_printf(
		(UB*)"RAW-G FLAT REFERENCE\n"
	);

	tm_printf(
		(UB*)"X=%d Y=%d Z=%d\n",
		raw_data.x,
		raw_data.y,
		raw_data.z
	);

	tm_printf(
		(UB*)"Flat sensor angle=%d deg\n",
		flat_angle
	);

	tm_printf(
		(UB*)"Relative tilt=0 deg\n"
	);

	tm_printf(
		(UB*)"========================================\n"
	);

	tm_printf((UB*)"\n");

	tm_printf(
		(UB*)"========================================\n"
	);

	tm_printf(
		(UB*)"CALIBRATION COMPLETE\n"
	);

	tm_printf(
		(UB*)"Flat baseline Y=%d\n",
		flat_y
	);

	tm_printf(
		(UB*)"========================================\n"
	);

	tm_printf(
		(UB*)"\nSTEP 1 OF 4 - UPHILL\n"
	);

	tm_printf(
		(UB*)"Wait during the countdown.\n"
	);

	tm_printf(
		(UB*)"After GO NOW, slowly move UPHILL.\n"
	);

	tm_printf(
		(UB*)"Stop when UPHILL DETECTED appears.\n"
	);

	preparation_countdown(
		"MOVE UPHILL"
	);

	while (1)
	{
		if (!imu_read(&raw_data))
		{
			update_imu_status(
				state,
				TRUE
			);

			tk_dly_tsk(S4_RUNTIME_SAMPLE_DELAY_MS);
			continue;
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

		imu_data.x =
			(int16_t)(runtime_x_sum / runtime_samples);
		imu_data.y =
			(int16_t)(runtime_y_sum / runtime_samples);
		imu_data.z =
			(int16_t)(runtime_z_sum / runtime_samples);

		raw_data = imu_data;

		runtime_x_sum = 0;
		runtime_y_sum = 0;
		runtime_z_sum = 0;
		runtime_samples = 0;

		/*
		 * Averaged samples are used for motion, terrain and lift processing
		 * so short raw-sample spikes do not trigger the lift detector.
		 */
		update_motion_classifier(&imu_data);

		if (imu_read_magnetometer(&mag_data))
		{
			update_turning_detector(&mag_data);
		}

		y_change =
			imu_data.y - flat_y;

		update_lift_detector(
			&imu_data,
			flat_y,
			state
		);

		raw_range_update(
			&raw_range,
			&raw_data
		);

		progress_count++;

			if (progress_count >=
			    S4_PROGRESS_INTERVAL)
			{
				print_waiting_message(
					state
				);

				progress_count = 0;
			}

			if (hump_complete)
			{
				transition_count = 0;
				progress_count = 0;
				continue;
			}

			switch (state)
			{
			case S4_TERRAIN_FLAT:

				if (y_change <
				    S4_UPHILL_THRESHOLD)
				{
					transition_count++;

					if (transition_count >=
					    S4_CONFIRM_SAMPLES)
					{
						raw_range_print(
							"FLAT / START MOVEMENT",
							&raw_range
						);

						print_tilt(
							"UPHILL",
							&raw_data,
							flat_angle
						);

						lowest_y =
							imu_data.y;

						encoder_height_start();

						state =
							S4_TERRAIN_UPHILL;

						transition_count = 0;
						progress_count = 0;

						update_imu_status(
							state,
							FALSE
						);

						raw_range_reset(
							&raw_range
						);

						tm_printf(
							(UB*)"\n"
						);

						tm_printf(
							(UB*)"========================================\n"
						);

						tm_printf(
							(UB*)">>> UPHILL DETECTED <<<\n"
						);

						tm_printf(
							(UB*)"STOP / HOLD THE IMU\n"
						);

						tm_printf(
							(UB*)"DO NOT MOVE DURING NEXT COUNTDOWN\n"
						);

						tm_printf(
							(UB*)"========================================\n"
						);

						tm_printf(
							(UB*)"\nSTEP 2 OF 4 - PEAK\n"
						);

						tm_printf(
							(UB*)"Wait for GO NOW.\n"
						);

						tm_printf(
							(UB*)"Then continue towards PEAK.\n"
						);

						preparation_countdown(
							"CONTINUE TO PEAK"
						);
					}
				}
				else
				{
					transition_count = 0;
				}

				break;

			case S4_TERRAIN_UPHILL:

				encoder_height_update(
					&imu_data,
					flat_angle
				);

				if (imu_data.y < lowest_y)
				{
					lowest_y =
						imu_data.y;
				}

				if ((imu_data.y - lowest_y) >
				    S4_PEAK_RISE_THRESHOLD)
				{
					transition_count++;

					if (transition_count >=
					    S4_CONFIRM_SAMPLES)
					{
						raw_range_print(
							"UPHILL",
							&raw_range
						);

						print_tilt(
							"PEAK",
							&raw_data,
							flat_angle
						);

						report_hump_height(
							flat_y,
							lowest_y
						);

						report_encoder_hump_height();

						state =
							S4_TERRAIN_PEAK;

						transition_count = 0;
						progress_count = 0;

						update_imu_status(
							state,
							FALSE
						);

						raw_range_reset(
							&raw_range
						);

						tm_printf(
							(UB*)"\n"
						);

						tm_printf(
							(UB*)"========================================\n"
						);

						tm_printf(
							(UB*)">>> PEAK DETECTED <<<\n"
						);

						tm_printf(
							(UB*)"STOP / HOLD THE IMU\n"
						);

						tm_printf(
							(UB*)"DO NOT MOVE DURING NEXT COUNTDOWN\n"
						);

						tm_printf(
							(UB*)"========================================\n"
						);

						tm_printf(
							(UB*)"\nSTEP 3 OF 4 - DOWNHILL\n"
						);

						tm_printf(
							(UB*)"Wait for GO NOW.\n"
						);

						tm_printf(
							(UB*)"Then move DOWNHILL.\n"
						);

						preparation_countdown(
							"MOVE DOWNHILL"
						);
					}
				}
				else
				{
					transition_count = 0;
				}

				break;

			case S4_TERRAIN_PEAK:

				if ((imu_data.y - lowest_y) >
				    S4_DOWNHILL_RISE_THRESHOLD)
				{
					transition_count++;

					if (transition_count >=
					    S4_CONFIRM_SAMPLES)
					{
						raw_range_print(
							"PEAK",
							&raw_range
						);

						print_tilt(
							"DOWNHILL",
							&raw_data,
							flat_angle
						);

						state =
							S4_TERRAIN_DOWNHILL;

						transition_count = 0;
						progress_count = 0;

						update_imu_status(
							state,
							FALSE
						);

						raw_range_reset(
							&raw_range
						);

						tm_printf(
							(UB*)"\n"
						);

						tm_printf(
							(UB*)"========================================\n"
						);

						tm_printf(
							(UB*)">>> DOWNHILL DETECTED <<<\n"
						);

						tm_printf(
							(UB*)"STOP / HOLD THE IMU\n"
						);

						tm_printf(
							(UB*)"DO NOT MOVE DURING NEXT COUNTDOWN\n"
						);

						tm_printf(
							(UB*)"========================================\n"
						);

						tm_printf(
							(UB*)"\nSTEP 4 OF 4 - RETURN TO FLAT\n"
						);

						tm_printf(
							(UB*)"Wait for GO NOW.\n"
						);

						tm_printf(
							(UB*)"Then return to original FLAT position.\n"
						);

						preparation_countdown(
							"RETURN TO FLAT"
						);
					}
				}
				else
				{
					transition_count = 0;
				}

				break;

			case S4_TERRAIN_DOWNHILL:

				if (y_change >
				    -S4_FLAT_RETURN_THRESHOLD &&
				    y_change <
				    S4_FLAT_RETURN_THRESHOLD)
				{
					transition_count++;

					if (transition_count >=
					    S4_CONFIRM_SAMPLES)
					{
						raw_range_print(
							"DOWNHILL / RETURN",
							&raw_range
						);

						print_tilt(
							"FINAL FLAT",
							&raw_data,
							flat_angle
						);

						state =
							S4_TERRAIN_FLAT;

						lowest_y =
							flat_y;

						transition_count = 0;
						progress_count = 0;

						update_imu_status(
							state,
							FALSE
						);

						tm_printf(
							(UB*)"\n"
						);

						tm_printf(
							(UB*)"========================================\n"
						);

						tm_printf(
							(UB*)">>> FLAT DETECTED <<<\n"
						);

						tm_printf(
							(UB*)"STOP MOVING\n"
						);

						tm_printf(
							(UB*)"========================================\n"
						);

						hump_complete = TRUE;

						tm_printf(
							(UB*)"HUMP + TILT TEST COMPLETE\n"
						);

						tm_printf(
							(UB*)"TEST SUCCESSFUL\n"
						);

						tm_printf(
							(UB*)"========================================\n\n"
						);

						raw_range_reset(
							&raw_range
						);
					}
				}
				else
				{
					transition_count = 0;
				}

				break;

			default:

				state =
					S4_TERRAIN_FLAT;

				lowest_y = flat_y;
				transition_count = 0;
				progress_count = 0;

				raw_range_reset(
					&raw_range
				);

				update_imu_status(
					state,
					TRUE
				);

				break;
			}

		tk_dly_tsk(S4_RUNTIME_SAMPLE_DELAY_MS);
	}
}