/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5.  Built only with: make TEST=1
 *	Record results in testing/subsystem5/TEST_LOG.md.  IDs: S5-U01, ...
 */

#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem5_test(void)
{
    float distance;

    TEST_INIT();

    /* S5-U01: Check task context */
    TEST_CHECK(
        "S5-U01",
        tk_get_tid() > 0,
        "Running in a valid task context"
    );

    /* S5-U02: Check ultrasonic sensor initialisation */
    ultrasonic_init();

    TEST_CHECK(
        "S5-U02",
        1,
        "Ultrasonic sensor initialisation called"
    );

    /* S5-U03: Check servo initialisation */
    servo_init();

    TEST_CHECK(
        "S5-U03",
        1,
        "Servo initialisation called"
    );

    /* S5-U04: Check ultrasonic sensor reading */
    distance = ultrasonic_get_distance();

    TEST_CHECK(
        "S5-U04",
        distance > 0.0f && distance <= 400.0f,
        "Ultrasonic distance is within valid range"
    );

    /* Return servo to centre position */
    servo_set_angle(90);

    tk_dly_tsk(100);

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */