/*
 * subsystem4_test.c
 * On-target Subsystem 4 tests. Build with TEST=1.
 *
 * IMPORTANT:
 * APP_TEST does not start imu_task(), so the complete runtime terrain FSM,
 * hump-height measurement and physical pickup-stop are MANUAL hardware tests.
 * Do not treat the legacy s4_classify_terrain() helper as an FSM test.
 *
 * This file changes TESTS ONLY. It does not change production IMU logic.
 */
#include "subsystem4.h"
#include "drv_imu.h"
#include "../test_report.h"

#if APP_TEST

static void print_runtime_validation(void)
{
    tm_printf((UB*)"\n=== SUBSYSTEM 4 RUNTIME / MANUAL TESTS ===\n");
    tm_printf((UB*)"[MANUAL] S4-IMU06: Confirm flat baseline produces FLAT in normal firmware.\n");
    tm_printf((UB*)"[MANUAL] S4-IMU07: Confirm positive-Y rise produces UPHILL in normal firmware.\n");
    tm_printf((UB*)"[MANUAL] S4-IMU08: Confirm fall from recorded Y maximum produces PEAK.\n");
    tm_printf((UB*)"[MANUAL] S4-IMU09: Confirm further fall produces DOWNHILL.\n");
    tm_printf((UB*)"[MANUAL] S4-M01: Confirm FLAT->UPHILL->PEAK->DOWNHILL->FLAT.\n");
    tm_printf((UB*)"[MANUAL] S4-M02: Measure physical hump and compare reported height.\n");
    tm_printf((UB*)"[MANUAL] S4-M03: Verify highest height across multiple humps.\n");
    tm_printf((UB*)"[MANUAL] S4-M04: Check acceleration bit sets and clears.\n");
    tm_printf((UB*)"[MANUAL] S4-M05: Check turning bit sets and clears.\n");
    tm_printf((UB*)"[MANUAL] S4-M06: Check LIFTED and safety-stop request.\n");
    tm_printf((UB*)"[MANUAL] S4-M07: Film both wheels physically stopping on pickup.\n");
    tm_printf((UB*)"[MANUAL] S4-M08: Verify IMU fault reporting if safely testable.\n");
    tm_printf((UB*)"[MANUAL] Run with normal firmware (TEST=0), save serial logs/video,\n");
    tm_printf((UB*)"[MANUAL] and record actual PASS/FAIL with tested commit ID.\n");
}

EXPORT void subsystem4_test(void)
{
    T_RMBF rmbf;
    UW before;
    IMU_DATA data;
    BOOL baseline_ok;
    INT flat_y = 0;
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

    /*
     * S4-IMU06..09 are NOT automatic tests here: the production terrain FSM
     * is stateful, requires consecutive samples, and runs inside imu_task().
     * APP_TEST disables imu_task(). These IDs are retained as manual tests.
     */

    imu_status = s4_get_imu_status();
    TEST_CHECK("S4-U03",
               (imu_status & (S4_IMU_STATUS)(~(S4_TERRAIN_BITS | S4_IMU_FAULT_BIT))) == 0,
               "IMU status uses only documented terrain/fault bits");

    motion_status = s4_motion_status;
    TEST_CHECK("S4-U04",
               (motion_status & (S4_MOTION_STATUS)(~S4_MOTION_BITS)) == 0,
               "motion status uses only accelerating/turning/lift bits");

    TEST_CHECK("S4-U05",
               S4_MOTION_ACCEL_BIT == 0x01 &&
               S4_MOTION_TURN_BIT == 0x02 &&
               S4_MOTION_LIFT_BIT == 0x04,
               "motion bits: bit0 acceleration, bit1 turn, bit2 lift");

    TEST_CHECK("S4-U06",
               s4_get_current_hump_height_mm() >= 0 &&
               s4_get_highest_hump_height_mm() >= s4_get_current_hump_height_mm(),
               "hump-height getters return consistent non-negative values");

    print_runtime_validation();
    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
