/*
 *	task_imu.c
 *	Subsystem 4 -- continuous IMU motion, terrain and tilt detection.
 */

#include "subsystem4.h"
#include "drv_imu.h"

#define S4_UPHILL_THRESHOLD			-1000
#define S4_PEAK_RISE_THRESHOLD			1500
#define S4_DOWNHILL_RISE_THRESHOLD		3000
#define S4_FLAT_RETURN_THRESHOLD		1000
#define S4_CONFIRM_SAMPLES			3
#define S4_PROGRESS_INTERVAL			5

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

#define S4_IMPACT_AXIS_THRESHOLD		10000U
#define S4_IMPACT_COOLDOWN_SAMPLES		100
#define S4_RUNTIME_FILTER_SAMPLES		10
#define S4_RUNTIME_SAMPLE_DELAY_MS		10

static S4_IMU_STATUS imu_status;

static UB motion_state;
static IMU_DATA previous_motion_sample;
static BOOL motion_sample_valid;
static INT acceleration_count;
static INT stationary_count;

static IMU_DATA previous_impact_sample;
static BOOL impact_sample_valid;
static INT impact_cooldown;

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
		tm_printf(
			(UB*)"imu: MOTION = ACCELERATING\n"
		);
	}
	else
	{
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

static UW axis_change(INT previous, INT current)
{
	INT difference;

	difference = current - previous;

	if (difference < 0)
	{
		difference = -difference;
	}

	return (UW)difference;
}

static void impact_detector_reset(void)
{
	impact_sample_valid = FALSE;
	impact_cooldown = 0;
}

static void update_impact_detector(IMU_DATA *current)
{
	UW delta_x;
	UW delta_y;
	UW delta_z;
	UW max_axis_delta;

	if (!impact_sample_valid)
	{
		previous_impact_sample = *current;
		impact_sample_valid = TRUE;
		return;
	}

	delta_x = axis_change(previous_impact_sample.x, current->x);
	delta_y = axis_change(previous_impact_sample.y, current->y);
	delta_z = axis_change(previous_impact_sample.z, current->z);

	previous_impact_sample = *current;

	max_axis_delta = delta_x;

	if (delta_y > max_axis_delta)
	{
		max_axis_delta = delta_y;
	}

	if (delta_z > max_axis_delta)
	{
		max_axis_delta = delta_z;
	}

	if (impact_cooldown > 0)
	{
		impact_cooldown--;
		return;
	}

	if (max_axis_delta > S4_IMPACT_AXIS_THRESHOLD)
	{
		tm_printf(
			(UB*)"imu: MOTION EVENT = SUDDEN IMPACT "
			"axis_delta=%u\n",
			max_axis_delta
		);

		impact_cooldown = S4_IMPACT_COOLDOWN_SAMPLES;
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

EXPORT void imu_task(
	INT stacd,
	void *exinf
)
{
	IMU_DATA imu_data;
	IMU_DATA raw_data;
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
	impact_detector_reset();

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

		update_impact_detector(&raw_data);

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
		 * The averaged 100 Hz samples preserve the slower motion/terrain
		 * processing while the impact detector sees every raw sample.
		 */
		update_motion_classifier(&imu_data);

		y_change =
			imu_data.y - flat_y;

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