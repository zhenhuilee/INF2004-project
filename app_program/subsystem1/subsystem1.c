/*
 *	subsystem1.c
 *	Subsystem 1 -- LED heartbeat: creates its objects and starts its task.
 *
 *	APIs shown: tk_cre_cyc, tk_sta_cyc
 */

#include "subsystem1.h"

EXPORT ID	tid_led;	/* IF-05, see subsystem1.h */
EXPORT ID	cyc_led;
EXPORT ID mbf_buddy1_cmd;
EXPORT ID mbf_buddy1_tx;

EXPORT ER subsystem1_init(void)
{
	T_CCYC	ccyc = {0};
	T_CMBF cmbf = {0};

	/* Cyclic handler: once started, the kernel calls led_cyclic_handler
	   every S1_TICK_MS.  Without TA_STA it is created stopped; it is
	   started in subsystem1_start(), once the task it wakes exists. */
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)led_cyclic_handler;
	ccyc.cyctim = S1_TICK_MS;	/* period, ms */
	ccyc.cycphs = S1_TICK_MS;	/* delay before the first call, ms */
	// cyc_led = tk_cre_cyc(&ccyc);
	// return (cyc_led < E_OK) ? cyc_led : E_OK;
	cyc_led = tk_cre_cyc(&ccyc);
	if (cyc_led < E_OK) {
		return cyc_led;
	}

	/* Create Buddy 1 command buffer */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz = S1_CMD_BUFFER_SIZE;
	cmbf.maxmsz = S1_CMD_MAX_SIZE;

	mbf_buddy1_cmd = tk_cre_mbf(&cmbf);
if (mbf_buddy1_cmd < E_OK) {
    return mbf_buddy1_cmd;
}

	/* Create outgoing MQTT telemetry buffer */
	T_CMBF tx_cmbf = {0};
	tx_cmbf.mbfatr = TA_TFIFO;
	tx_cmbf.bufsz = S1_TX_BUFFER_SIZE;
	tx_cmbf.maxmsz = S1_TX_MAX_SIZE;

	mbf_buddy1_tx = tk_cre_mbf(&tx_cmbf);
	if (mbf_buddy1_tx < E_OK) {
		return mbf_buddy1_tx;
	}

	return E_OK;
}

EXPORT ER subsystem1_start(void)
{
	tid_led = app_start_task(led_task, S1_PRI_LED, 0, "led_task");
	if (tid_led < E_OK) {
		return tid_led;
	}
	return tk_sta_cyc(cyc_led);	/* start the cyclic handler now */
}
