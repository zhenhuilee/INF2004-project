/*
 * subsystem3.c
 * Subsystem 3 -- Consumer initialization and task starter.
 */

#include "subsystem3.h"
#include "sys/sysdef.h"

EXPORT ID       mbx_samples;    /* IF-01 */
EXPORT ID       mtx_stats;      /* IF-04 */
EXPORT APP_STATS    app_stats;

EXPORT ER subsystem3_init(void)
{
    T_CMBX  cmbx = {0};
    T_CMTX  cmtx = {0};

    cmbx.mbxatr = TA_TFIFO | TA_MFIFO;  
    mbx_samples = tk_cre_mbx(&cmbx);
    if (mbx_samples < E_OK) {
        return mbx_samples;
    }

    cmtx.mtxatr = TA_INHERIT;
    mtx_stats = tk_cre_mtx(&cmtx);
    if (mtx_stats < E_OK) {
        return mtx_stats;
    }

    #define PADS_BANK0_BASE 0x4001c000

	/* Inside subsystem3_init: */
	// Set function 5 (SIO) for GPIOs
	*(volatile unsigned int*)(IO_BANK0_BASE + 0x004 + (S3_IR_PIN_LEFT * 8))   = 5;
	*(volatile unsigned int*)(IO_BANK0_BASE + 0x004 + (S3_IR_PIN_CENTER * 8)) = 5;
	*(volatile unsigned int*)(IO_BANK0_BASE + 0x004 + (S3_IR_PIN_RIGHT * 8))  = 5;

	// Enable input (IE bit = 7th bit, i.e., 1 << 6) and disable output override on pads
	*(volatile unsigned int*)(PADS_BANK0_BASE + 0x004 + (S3_IR_PIN_LEFT * 4))   = (1 << 6); // IE=1, OD=0
	*(volatile unsigned int*)(PADS_BANK0_BASE + 0x004 + (S3_IR_PIN_CENTER * 4)) = (1 << 6);
	*(volatile unsigned int*)(PADS_BANK0_BASE + 0x004 + (S3_IR_PIN_RIGHT * 4))  = (1 << 6);

    return E_OK;
}

EXPORT ER subsystem3_start(void)
{
    ID tid = app_start_task(consumer_task, S3_PRI_CONSUMER, TP_PRC1,
                "consumer_task");
    return (tid < E_OK) ? tid : E_OK;
}