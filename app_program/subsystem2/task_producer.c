/*
 *  task_producer.c
 *  Subsystem 2 -- the producer task.
 */

#include "subsystem2.h"
#include "../subsystem3/subsystem3.h"   /* IF-01: mbx_samples */

/* A fake measurement: a triangle wave between 0 and 100 */
EXPORT INT s2_make_value(UW seq)
{
    INT phase = (INT)(seq % 20);
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
        msg->prc    = tk_get_prc();    /* which core am I on? */
#else
        msg->prc    = 1;
#endif

        /* Send the pointer to Subsystem 3 */
        er = tk_snd_mbx(mbx_samples, (T_MSG *)msg);
        CHECK(er, "tk_snd_mbx");

        /* Wait for the next period */
        tk_dly_tsk(S2_PERIOD_MS);
    }
}