/*
 *	subsystem2.c
 *	Subsystem 2 -- Motion control: creates its kernel objects and
 *	starts its task.
 *
 *	APIs shown: tk_cre_mtx, tk_cre_flg, tk_cre_cyc, tk_sta_cyc
 */

#include "subsystem2.h"

EXPORT ID	tid_motion;
EXPORT ID	cyc_motion;
EXPORT ID	mtx_motion;	/* protects the command block in task_motion.c */
EXPORT ID	flg_motion;	/* S2_FLG_DONE: set when a command finishes */

EXPORT ER subsystem2_init(void)
{
	T_CMTX	cmtx = {0};
	T_CFLG	cflg = {0};
	T_CCYC	ccyc = {0};
	ER	er;

	/* Hardware set-up happens here, while only usermain() is running,
	   so the 1 ms handler can never sample an unconfigured pin. */
	er = motor_init();
	if (er < E_OK) return er;
	enc_init();

	cmtx.mtxatr = TA_INHERIT;	/* no priority inversion for callers */
	mtx_motion = tk_cre_mtx(&cmtx);
	if (mtx_motion < E_OK) return mtx_motion;

	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = S2_FLG_DONE;	/* idle at boot = "done" */
	flg_motion = tk_cre_flg(&cflg);
	if (flg_motion < E_OK) return flg_motion;

	/* 1 ms tick: samples the encoders, wakes motion_task every
	   S2_CTRL_MS.  Created stopped; started in subsystem2_start(). */
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)motion_cyclic_handler;
	ccyc.cyctim = S2_ENC_POLL_MS;
	ccyc.cycphs = S2_ENC_POLL_MS;
	cyc_motion = tk_cre_cyc(&ccyc);
	return (cyc_motion < E_OK) ? cyc_motion : E_OK;
}

EXPORT ER subsystem2_start(void)
{
	tid_motion = app_start_task(motion_task, S2_PRI_MOTION, 0,
				    "motion_task");
	if (tid_motion < E_OK) return tid_motion;
	return tk_sta_cyc(cyc_motion);
}
