/*
 * subsystem4_test.c
 * On-target tests for subsystem 4. Built only with: make TEST=1
 */
#include "subsystem4.h"
#include "drv_imu.h"
#include "../test_report.h"

#if APP_TEST

static INT flat_y;

static void collect_terrain_stage(const char *stage)
{
    IMU_DATA data;
    S4_TERRAIN_STATE terrain;
    INT i;

    tm_printf((UB*)"\n[MANUAL] %s - position robot/IMU, wait 5 seconds.\n", stage);
    tk_dly_tsk(5000);

    for (i = 0; i < 10; i++)
    {
        if (imu_read_filtered(&data))
        {
            terrain = s4_classify_terrain(data.y, flat_y);
            tm_printf((UB*)"[MANUAL][%s] X=%d Y=%d Z=%d terrain=%d\n",
                      stage, data.x, data.y, data.z, terrain);
        }
        tk_dly_tsk(200);
    }
}

static void print_manual_tests(void)
{
    tm_printf((UB*)"\n=== SUBSYSTEM 4 MANUAL HARDWARE TESTS ===\n");
    tm_printf((UB*)"[MANUAL] S4-M01 Terrain: FLAT->UPHILL->PEAK->DOWNHILL->FLAT\n");
    tm_printf((UB*)"[MANUAL] S4-M02 Hump height: completed hump reports plausible calibrated height\n");
    tm_printf((UB*)"[MANUAL] S4-M03 Highest hump: multiple humps retain highest measured height\n");
    tm_printf((UB*)"[MANUAL] S4-M04 Acceleration: motion bit0 sets/clears correctly\n");
    tm_printf((UB*)"[MANUAL] S4-M05 Turning: motion bit1 sets/clears correctly\n");
    tm_printf((UB*)"[MANUAL] S4-M06 Pickup: LIFTED + safety-stop request are produced\n");
    tm_printf((UB*)"[MANUAL] S4-M07 Physical pickup stop: wheels actually stop - REQUIRES RETEST\n");
    tm_printf((UB*)"[MANUAL] S4-M08 IMU fault: fault bit/reporting when IMU access fails\n");
}

EXPORT void subsystem4_test(void)
{
    T_RMBF rmbf;
    UW before;
    IMU_DATA data;
    BOOL baseline_ok;
    S4_IMU_STATUS imu_status;
    S4_MOTION_STATUS motion_status;

    TEST_INIT();

    TEST_CHECK("S4-U01",
               tk_ref_mbf(mbf_summary, &rmbf) == E_OK &&
               rmbf.maxmsz == S4_SUMMARY_MAX,
               "message buffer max line = S4_SUMMARY_MAX");

    before = s4_lines_received;
    tk_dly_tsk(3500);
    TEST_CHECK("INT-02", s4_lines_received >= before + 2,
               "summary lines flow from subsystem 3 to subsystem 4");

    TEST_CHECK("S4-IMU01", imu_init(), "IMU initialises successfully");

    tm_printf((UB*)"\n[IMU] Keep robot/IMU flat for calibration.\n");
    TEST_CHECK("S4-IMU02", imu_calibrate(), "IMU calibration completes");
    TEST_CHECK("S4-IMU03", imu_read_calibrated(&data),
               "calibrated IMU X/Y/Z data can be read");
    TEST_CHECK("S4-IMU04", imu_read_filtered(&data),
               "filtered IMU X/Y/Z data can be read");

    tk_dly_tsk(1000);
    baseline_ok = imu_read_filtered(&data);
    if (baseline_ok)
    {
        flat_y = data.y;
        tm_printf((UB*)"[IMU] FLAT BASELINE Y=%d\n", flat_y);
    }

    TEST_CHECK("S4-IMU05", baseline_ok, "flat baseline captured");
    TEST_CHECK("S4-IMU06",
               baseline_ok && s4_classify_terrain(flat_y, flat_y) == S4_TERRAIN_FLAT,
               "baseline classified as FLAT");
    TEST_CHECK("S4-IMU07",
               baseline_ok && s4_classify_terrain(flat_y - 1500, flat_y) == S4_TERRAIN_UPHILL,
               "negative Y change classified as UPHILL");
    TEST_CHECK("S4-IMU08",
               baseline_ok && s4_classify_terrain(flat_y + 550, flat_y) == S4_TERRAIN_PEAK,
               "moderate positive Y change classified as PEAK");
    TEST_CHECK("S4-IMU09",
               baseline_ok && s4_classify_terrain(flat_y + 1800, flat_y) == S4_TERRAIN_DOWNHILL,
               "large positive Y change classified as DOWNHILL");

    imu_status = s4_get_imu_status();
    TEST_CHECK("S4-U03",
               (imu_status & (S4_IMU_STATUS)(~(S4_TERRAIN_BITS | S4_IMU_FAULT_BIT))) == 0,
               "IMU status uses only documented terrain/fault bits");

    motion_status = s4_motion_status;
    TEST_CHECK("S4-U04",
               (motion_status & (S4_MOTION_STATUS)(~S4_MOTION_BITS)) == 0,
               "motion status uses only accelerating/turning/lift bits");

    TEST_CHECK("S4-U05",
               S4_MOTION_ACCEL_BIT == 0x01 && S4_MOTION_TURN_BIT == 0x02 && S4_MOTION_LIFT_BIT == 0x04,
               "motion bits are bit0 acceleration, bit1 turn, bit2 lift");

    TEST_CHECK("S4-U06",
               s4_get_current_hump_height_mm() >= 0 &&
               s4_get_highest_hump_height_mm() >= s4_get_current_hump_height_mm(),
               "hump-height getters return consistent non-negative values");

    if (baseline_ok)
    {
        collect_terrain_stage("FLAT");
        collect_terrain_stage("UPHILL");
        collect_terrain_stage("PEAK");
        collect_terrain_stage("DOWNHILL");
        collect_terrain_stage("FLAT_END");
    }

    print_manual_tests();
    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
