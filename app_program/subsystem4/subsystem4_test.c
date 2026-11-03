/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem4/TEST_LOG.md and the
 *	INT- result in testing/integration/TEST_LOG.md.
 */

#include "subsystem4.h"
#include "drv_imu.h"
#include "../test_report.h"

#if APP_TEST

static INT flat_y;

static void collect_imu_stage(const char *stage)
{
	IMU_DATA imu_data;
	S4_TERRAIN_STATE terrain;

	tm_printf((UB*)"\n[IMU] Place board at: %s\n", stage);
	tm_printf((UB*)"[IMU] Recording starts in 15 seconds...\n");
	tk_dly_tsk(15000);

	for (INT i = 0; i < 10; i++)
	{
		if (imu_read_filtered(&imu_data))
		{
			terrain = s4_classify_terrain(imu_data.y, flat_y);

			tm_printf((UB*)"[IMU][%s] X=%d Y=%d Z=%d STATE=%d\n",
				  stage,
				  imu_data.x,
				  imu_data.y,
				  imu_data.z,
				  terrain);
		}

		tk_dly_tsk(200);
	}
}

EXPORT void subsystem4_test(void)
{
	T_RMBF	rmbf;
	UW	before;
	IMU_DATA imu_data;
	bool baseline_ok;

	TEST_INIT();

	/* S4-U01: the message buffer accepts the documented line length */
	TEST_CHECK("S4-U01",
		   tk_ref_mbf(mbf_summary, &rmbf) == E_OK &&
		   rmbf.maxmsz == S4_SUMMARY_MAX,
		   "message buffer max line = S4_SUMMARY_MAX");

	/* INT-02 (integration, IF-03): summary lines from subsystem 3 reach
	   the monitor.  One line per 5 samples = one per 1.5 s. */
	before = s4_lines_received;
	tk_dly_tsk(3500);
	TEST_CHECK("INT-02", s4_lines_received >= before + 2,
		   "summary lines flow from subsystem 3 to subsystem 4");

	TEST_CHECK("S4-IMU01",
		   imu_init(),
		   "IMU initialises successfully");

	tm_printf((UB*)"\n[IMU] Keep board FLAT on the table\n");
	tm_printf((UB*)"[IMU] Calibrating...\n");

	TEST_CHECK("S4-IMU02",
		   imu_calibrate(),
		   "IMU calibration completes");

	TEST_CHECK("S4-IMU03",
		   imu_read_calibrated(&imu_data),
		   "calibrated IMU X/Y/Z data can be read");

	TEST_CHECK("S4-IMU04",
		   imu_read_filtered(&imu_data),
		   "filtered IMU X/Y/Z data can be read");

	tm_printf((UB*)"\n[IMU] Capturing flat baseline...\n");
	tk_dly_tsk(1000);

	baseline_ok = imu_read_filtered(&imu_data);

	if (baseline_ok)
	{
		flat_y = imu_data.y;
		tm_printf((UB*)"[IMU] FLAT BASELINE Y=%d\n", flat_y);
	}

	TEST_CHECK("S4-IMU05",
		   baseline_ok,
		   "flat baseline captured");

	TEST_CHECK("S4-IMU06",
		   baseline_ok &&
		   s4_classify_terrain(flat_y, flat_y) == S4_TERRAIN_FLAT,
		   "baseline classified as FLAT");

	TEST_CHECK("S4-IMU07",
		   s4_classify_terrain(flat_y - 1500, flat_y) ==
		   S4_TERRAIN_UPHILL,
		   "negative Y change classified as UPHILL");

	TEST_CHECK("S4-IMU08",
		   s4_classify_terrain(flat_y + 550, flat_y) ==
		   S4_TERRAIN_PEAK,
		   "moderate positive Y change classified as PEAK");

	TEST_CHECK("S4-IMU09",
		   s4_classify_terrain(flat_y + 1800, flat_y) ==
		   S4_TERRAIN_DOWNHILL,
		   "large positive Y change classified as DOWNHILL");

	tm_printf((UB*)"\n[IMU] LIVE TERRAIN CLASSIFICATION START\n");
	tm_printf((UB*)"[IMU] Flat baseline Y=%d\n", flat_y);
	tm_printf((UB*)"[IMU] STATE: 0=FLAT 1=UPHILL 2=PEAK 3=DOWNHILL\n");
	tm_printf((UB*)"[IMU] Keep the IMU orientation consistent\n");

	collect_imu_stage("FLAT_START");
	collect_imu_stage("UPHILL");
	collect_imu_stage("PEAK");
	collect_imu_stage("DOWNHILL");
	collect_imu_stage("FLAT_END");

	tm_printf((UB*)"\n[IMU] LIVE TERRAIN CLASSIFICATION COMPLETE\n");

	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */