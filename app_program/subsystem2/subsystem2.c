/*
 *	subsystem2.c
 *	Subsystem 2 -- Motion control: creates its objects and starts its tasks.
 */

#include "subsystem2.h"

/* ---- EXAMPLE PRODUCER objects (see subsystem2.h) -------------------- */
EXPORT ID	sem_slots;
EXPORT ID	mpf_samples;

static ER
s2_example_init (void)
{
    T_CSEM  csem = {0};
    T_CMPF  cmpf = {0};
    ER      er;

    /* 1. Initialize Subsystem 2 Motion Control (Motors, Encoders, Command Buffer) */
    er = motion_init();
    if (er < E_OK) {
        return er;
    }

    /* 2. Semaphore: counter of free "slots" */
    csem.sematr  = TA_TFIFO;
    csem.isemcnt = S2_POOL_COUNT;
    csem.maxsem  = S2_POOL_COUNT;
    sem_slots = tk_cre_sem(&csem);
    if (sem_slots < E_OK) {
        return sem_slots;
    }

    /* 3. Fixed-size memory pool */
    cmpf.mpfatr = TA_TFIFO | TA_RNG3;
    cmpf.mpfcnt = S2_POOL_COUNT;
    cmpf.blfsz  = sizeof(SAMPLE_MSG);
    mpf_samples = tk_cre_mpf(&cmpf);
    return (mpf_samples < E_OK) ? mpf_samples : E_OK;
}
/* ---- end of example ------------------------------------------------- */

EXPORT ER subsystem2_init(void)
{
	ER	er;

	er = (ER)motion_init();
	if (er < E_OK)
	{
		return er;
	}
	return s2_example_init();
}

EXPORT ER subsystem2_start(void)
{
    ID tid_producer, tid_motion;

    /* Start the sample producer task on core 2 (TP_PRC2) */
    tid_producer = app_start_task(producer_task, S2_PRI_PRODUCER, TP_PRC2,
                "producer_task");
    if (tid_producer < E_OK) {
        return tid_producer;
    }

    /* Start the motion control task */
    tid_motion = motion_start();
    return (tid_motion < E_OK) ? tid_motion : E_OK;
}