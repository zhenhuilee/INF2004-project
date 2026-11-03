/*
 *	task_imu.c
 *	Subsystem 4 -- continuous IMU terrain and tilt detection.
 */

#include "subsystem4.h"
#include "drv_imu.h"

#define S4_UPHILL_THRESHOLD		-1000
#define S4_PEAK_RISE_THRESHOLD		1500
#define S4_DOWNHILL_RISE_THRESHOLD	3000
#define S4_FLAT_RETURN_THRESHOLD	1000
#define S4_CONFIRM_SAMPLES		3
#define S4_PROGRESS_INTERVAL		5

static S4_IMU_STATUS imu_status;

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
		angle = 90 - ((45 * abs_z) / abs_y);
	}

	if (z >= 0)
	{
		if (y >= 0)
		{
			return angle;
		}
		else
		{
			return -angle;
		}
	}

	if (y >= 0)
	{
		return 180 - angle;
	}

	return -180 + angle;
}

static INT relative_tilt_deg(INT y, INT z, INT flat_angle)
{
	INT tilt;

	tilt = approximate_angle_deg(y, z) - flat_angle;

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

static void print_tilt(const char *stage,
		       IMU_DATA *raw_data,
		       INT flat_angle)
{
	INT angle;
	INT tilt;

	angle = approximate_angle_deg(raw_data->y,
				      raw_data->z);

	tilt = relative_tilt_deg(raw_data->y,
				 raw_data->z,
				 flat_angle);

	tm_printf((UB*)"\n%s TILT\n", stage);

	tm_printf((UB*)"RAW-G: X=%d Y=%d Z=%d\n",
		  raw_data->x,
		  raw_data->y,
		  raw_data->z);

	tm_printf((UB*)"Sensor angle=%d deg\n", angle);
	tm_printf((UB*)"Relative tilt=%d deg\n", tilt);
}

static void update_imu_status(S4_TERRAIN_STATE state, BOOL fault)
{
	S4_IMU_STATUS new_status;

	new_status = (S4_IMU_STATUS)(state & S4_TERRAIN_BITS);

	if (fault)
	{
		new_status |= S4_IMU_FAULT_BIT;
	}

	if (new_status != imu_status)
	{
		imu_status = new_status;

		tm_printf((UB*)"imu: STATUS CHANGE = 0x%02X terrain=%d fault=%d\n",
			  imu_status,
			  imu_status & S4_TERRAIN_BITS,
			  (imu_status & S4_IMU_FAULT_BIT) ? 1 : 0);
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

static void raw_range_update(S4_RAW_RANGE *range, IMU_DATA *data)
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

static void raw_range_print(const char *stage, S4_RAW_RANGE *range)
{
	if (range->samples == 0)
	{
		tm_printf((UB*)"%s RAW-G RANGE: no samples\n", stage);
		return;
	}

	tm_printf((UB*)"\n%s RAW-G RANGE\n", stage);
	tm_printf((UB*)"samples=%d\n", range->samples);

	tm_printf((UB*)"X: min=%d max=%d\n",
		  range->min_x,
		  range->max_x);

	tm_printf((UB*)"Y: min=%d max=%d\n",
		  range->min_y,
		  range->max_y);

	tm_printf((UB*)"Z: min=%d max=%d\n",
		  range->min_z,
		  range->max_z);
}

static void preparation_countdown(const char *action)
{
	INT i;

	tm_printf((UB*)"\n");
	tm_printf((UB*)"========================================\n");
	tm_printf((UB*)"NEXT ACTION: %s\n", action);
	tm_printf((UB*)"========================================\n");
	tm_printf((UB*)"DO NOT MOVE YET.\n");
	tm_printf((UB*)"You have 15 seconds to get ready.\n\n");

	for (i = 15; i > 0; i--)
	{
		tm_printf((UB*)"%d...\n", i);
		tk_dly_tsk(1000);
	}

	tm_printf((UB*)"\n========================================\n");
	tm_printf((UB*)">>> GO NOW: %s <<<\n", action);
	tm_printf((UB*)"MOVE SLOWLY.\n");
	tm_printf((UB*)"STOP AS SOON AS DETECTED APPEARS.\n");
	tm_printf((UB*)"========================================\n\n");
}

static void print_waiting_message(S4_TERRAIN_STATE state)
{
	switch (state)
	{
	case S4_TERRAIN_FLAT:
		tm_printf((UB*)"\n[IMU] DETECTING UPHILL...\n");
		tm_printf((UB*)"[IMU] KEEP MOVING UPHILL\n");
		break;

	case S4_TERRAIN_UPHILL:
		tm_printf((UB*)"\n[IMU] DETECTING PEAK...\n");
		tm_printf((UB*)"[IMU] KEEP MOVING TOWARDS PEAK\n");
		break;

	case S4_TERRAIN_PEAK:
		tm_printf((UB*)"\n[IMU] DETECTING DOWNHILL...\n");
		tm_printf((UB*)"[IMU] KEEP MOVING DOWNHILL\n");
		break;

	case S4_TERRAIN_DOWNHILL:
		tm_printf((UB*)"\n[IMU] DETECTING FLAT...\n");
		tm_printf((UB*)"[IMU] KEEP RETURNING TO FLAT\n");
		break;

	default:
		break;
	}
}

EXPORT void imu_task(INT stacd, void *exinf)
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

	S4_TERRAIN_STATE state = S4_TERRAIN_FLAT;

	(void)stacd;
	(void)exinf;

	imu_status = 0xFF;

	if (!imu_init())
	{
		update_imu_status(S4_TERRAIN_FLAT, TRUE);
		tm_printf((UB*)"imu: initialisation failed\n");
		return;
	}

	tm_printf((UB*)"\n========================================\n");
	tm_printf((UB*)"IMU HUMP + TILT TEST\n");
	tm_printf((UB*)"========================================\n");
	tm_printf((UB*)"FIRST: PLACE ROBOT COMPLETELY FLAT\n");
	tm_printf((UB*)"DO NOT MOVE IT DURING CALIBRATION\n");
	tm_printf((UB*)"\n");
	tm_printf((UB*)"Calibration starts in 15 seconds.\n");
	tm_printf((UB*)"Use this time to position the robot.\n");
	tm_printf((UB*)"========================================\n\n");

	for (i = 15; i > 0; i--)
	{
		tm_printf((UB*)"%d...\n", i);
		tk_dly_tsk(1000);
	}

	tm_printf((UB*)"\n========================================\n");
	tm_printf((UB*)">>> CALIBRATION STARTING NOW <<<\n");
	tm_printf((UB*)"DO NOT TOUCH OR MOVE THE ROBOT.\n");
	tm_printf((UB*)"========================================\n\n");

	if (!imu_calibrate())
	{
		update_imu_status(S4_TERRAIN_FLAT, TRUE);
		tm_printf((UB*)"imu: calibration failed\n");
		return;
	}

	tk_dly_tsk(500);

	if (!imu_read_filtered(&imu_data))
	{
		update_imu_status(S4_TERRAIN_FLAT, TRUE);
		tm_printf((UB*)"imu: flat baseline capture failed\n");
		return;
	}

	flat_y = imu_data.y;
	lowest_y = flat_y;

	update_imu_status(S4_TERRAIN_FLAT, FALSE);
	raw_range_reset(&raw_range);

	if (!imu_read_raw_filtered(&raw_data))
	{
		update_imu_status(S4_TERRAIN_FLAT, TRUE);
		tm_printf((UB*)"imu: raw flat reference capture failed\n");
		return;
	}

	flat_angle = approximate_angle_deg(
		raw_data.y,
		raw_data.z
	);

	tm_printf((UB*)"\n========================================\n");
	tm_printf((UB*)"RAW-G FLAT REFERENCE\n");

	tm_printf((UB*)"X=%d Y=%d Z=%d\n",
		  raw_data.x,
		  raw_data.y,
		  raw_data.z);

	tm_printf((UB*)"Flat sensor angle=%d deg\n",
		  flat_angle);

	tm_printf((UB*)"Relative tilt=0 deg\n");
	tm_printf((UB*)"========================================\n");

	tm_printf((UB*)"\n========================================\n");
	tm_printf((UB*)"CALIBRATION COMPLETE\n");
	tm_printf((UB*)"Flat baseline Y=%d\n", flat_y);
	tm_printf((UB*)"========================================\n");

	tm_printf((UB*)"\nSTEP 1 OF 4 - UPHILL\n");
	tm_printf((UB*)"Wait during the countdown.\n");
	tm_printf((UB*)"After GO NOW, slowly move UPHILL.\n");
	tm_printf((UB*)"Stop when UPHILL DETECTED appears.\n");

	preparation_countdown("MOVE UPHILL");

	while (1)
	{
		if (imu_read_filtered(&imu_data))
		{
			y_change = imu_data.y - flat_y;

			if (imu_read_raw_filtered(&raw_data))
			{
				raw_range_update(
					&raw_range,
					&raw_data
				);
			}

			progress_count++;

			if (progress_count >= S4_PROGRESS_INTERVAL)
			{
				print_waiting_message(state);
				progress_count = 0;
			}

			switch (state)
			{
			case S4_TERRAIN_FLAT:

				if (y_change < S4_UPHILL_THRESHOLD)
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

						lowest_y = imu_data.y;
						state = S4_TERRAIN_UPHILL;

						transition_count = 0;
						progress_count = 0;

						update_imu_status(
							state,
							FALSE
						);

						raw_range_reset(
							&raw_range
						);

						tm_printf((UB*)"\n");
						tm_printf((UB*)"========================================\n");
						tm_printf((UB*)">>> UPHILL DETECTED <<<\n");
						tm_printf((UB*)"STOP / HOLD THE ROBOT\n");
						tm_printf((UB*)"DO NOT MOVE DURING NEXT COUNTDOWN\n");
						tm_printf((UB*)"========================================\n");

						tm_printf((UB*)"\nSTEP 2 OF 4 - PEAK\n");
						tm_printf((UB*)"Wait for GO NOW.\n");
						tm_printf((UB*)"Then continue towards PEAK.\n");

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
					lowest_y = imu_data.y;
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

						state = S4_TERRAIN_PEAK;

						transition_count = 0;
						progress_count = 0;

						update_imu_status(
							state,
							FALSE
						);

						raw_range_reset(
							&raw_range
						);

						tm_printf((UB*)"\n");
						tm_printf((UB*)"========================================\n");
						tm_printf((UB*)">>> PEAK DETECTED <<<\n");
						tm_printf((UB*)"STOP / HOLD THE ROBOT\n");
						tm_printf((UB*)"DO NOT MOVE DURING NEXT COUNTDOWN\n");
						tm_printf((UB*)"========================================\n");

						tm_printf((UB*)"\nSTEP 3 OF 4 - DOWNHILL\n");
						tm_printf((UB*)"Wait for GO NOW.\n");
						tm_printf((UB*)"Then move DOWNHILL.\n");

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

						tm_printf((UB*)"\n");
						tm_printf((UB*)"========================================\n");
						tm_printf((UB*)">>> DOWNHILL DETECTED <<<\n");
						tm_printf((UB*)"STOP / HOLD THE ROBOT\n");
						tm_printf((UB*)"DO NOT MOVE DURING NEXT COUNTDOWN\n");
						tm_printf((UB*)"========================================\n");

						tm_printf((UB*)"\nSTEP 4 OF 4 - RETURN TO FLAT\n");
						tm_printf((UB*)"Wait for GO NOW.\n");
						tm_printf((UB*)"Then return to original FLAT position.\n");

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

						state = S4_TERRAIN_FLAT;
						lowest_y = flat_y;

						transition_count = 0;
						progress_count = 0;

						update_imu_status(
							state,
							FALSE
						);

						tm_printf((UB*)"\n");
						tm_printf((UB*)"========================================\n");
						tm_printf((UB*)">>> FLAT DETECTED <<<\n");
						tm_printf((UB*)"STOP MOVING\n");
						tm_printf((UB*)"========================================\n");
						tm_printf((UB*)"HUMP + TILT TEST COMPLETE\n");
						tm_printf((UB*)"TEST SUCCESSFUL\n");
						tm_printf((UB*)"========================================\n\n");

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

				state = S4_TERRAIN_FLAT;
				lowest_y = flat_y;

				transition_count = 0;
				progress_count = 0;

				raw_range_reset(&raw_range);

				update_imu_status(
					state,
					TRUE
				);

				break;
			}
		}
		else
		{
			update_imu_status(
				state,
				TRUE
			);
		}

		tk_dly_tsk(100);
	}
}