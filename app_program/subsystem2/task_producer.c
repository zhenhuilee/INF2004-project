/*
 *	task_producer.c
 *	Subsystem 2 -- the producer task.
 *
 *	APIs shown: tk_wai_sem, tk_get_mpf, tk_snd_mbx, tk_get_tim,
 *	            tk_dly_tsk, tk_get_prc
 *
 *	Replace s2_make_value() with a real sensor read (ADC, I2C, ...) in
 *	your project.  Keep each peripheral owned by ONE task.
 */

#include "subsystem2.h"
#include "drv_motor.h"
#include "../subsystem3/subsystem3.h"	/* IF-01: mbx_samples */

/* A fake measurement: a triangle wave between 0 and 100 */
EXPORT INT s2_make_value(UW seq)
{
	INT	phase = (INT)(seq % 20);
	return (phase < 10 ? phase : 20 - phase) * 10;
}

EXPORT void producer_task(INT stacd, void *exinf)
{
    UW seq = 0;
    SAMPLE_MSG *msg;
    SYSTIM now;
    ER er;

    (void)stacd;
    (void)exinf;

    /* ==================================
       TEMPORARY MOTOR TEST
       ================================== */

	motor_init();

	/*motor_forward();
	tk_dly_tsk(5000);

	motor_stop();
	tk_dly_tsk(500);

	motor_backward();
	tk_dly_tsk(5000);

	motor_stop();
	tk_dly_tsk(500);

	motor_turn_right();
	tk_dly_tsk(200);

	motor_stop();
	tk_dly_tsk(500);

	motor_turn_left();
	tk_dly_tsk(200);

	motor_stop();*/

    /* ==================================
       ORIGINAL PRODUCER TASK
       ================================== */

    while (1) {
        er = tk_wai_sem(sem_slots, 1, TMO_FEVR);
        CHECK(er, "tk_wai_sem");

        er = tk_get_mpf(mpf_samples, (void **)&msg, TMO_POL);

        if (er < E_OK) {
            CHECK(er, "tk_get_mpf");
            tk_sig_sem(sem_slots, 1);
            tk_dly_tsk(S2_PERIOD_MS);
            continue;
        }

        tk_get_tim(&now);

        msg->seq = seq++;
        msg->time_ms = now.lo;
        msg->value = s2_make_value(msg->seq);

#if TK_SUPPORT_SMP
        msg->prc = tk_get_prc();
#else
        msg->prc = 1;
#endif

        er = tk_snd_mbx(mbx_samples, (T_MSG *)msg);
        CHECK(er, "tk_snd_mbx");

        tk_dly_tsk(S2_PERIOD_MS);
    }
}
