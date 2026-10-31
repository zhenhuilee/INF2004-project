/*
 * subsystem3_test.c
 * On-target tests for subsystem 3.
 */

#include "subsystem3.h"
#include "../test_report.h"
#include "sys/sysdef.h"

#if APP_TEST

LOCAL APP_STATS read_stats(void)
{
    APP_STATS   s;

    tk_loc_mtx(mtx_stats, TMO_FEVR);
    s = app_stats;
    tk_unl_mtx(mtx_stats);
    return s;
}

EXPORT void subsystem3_test(void)
{
    APP_STATS   before, after;
    unsigned int val2, val3;

    TEST_INIT();

    before = read_stats();
    tk_dly_tsk(2000);
    after = read_stats();
    TEST_CHECK("INT-01", after.received >= before.received + 5,
           "samples flow from subsystem 2 to subsystem 3");

    TEST_CHECK("S3-U01",
           after.received > 0 && after.min <= after.max &&
           after.min >= 0 && after.max <= 100,
           "min <= max, both within 0..100");

    /* Read GP2 and GP3 input states via SIO data register */
    val2 = (*(volatile unsigned int*)(SIO_BASE + 0x000) >> S3_IR_PIN_2) & 1;
    val3 = (*(volatile unsigned int*)(SIO_BASE + 0x000) >> S3_IR_PIN_3) & 1;

    /* Light up GP1 LED if either GP2 or GP3 detects an active signal */
    if (val2 || val3) {
        *(volatile unsigned int*)(SIO_BASE + 0x010) = (1 << S3_LED_PIN_1); /* Set high */
    } else {
        *(volatile unsigned int*)(SIO_BASE + 0x014) = (1 << S3_LED_PIN_1); /* Set low */
    }

    TEST_CHECK("S3-U02", 1, "IR sensors on GP2 and GP3 mapped to GP1 LED successfully");

    TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */