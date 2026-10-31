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

    /* Configure GPIO hardware registers safely via micro T-Kernel base definitions */
    /* Set GP2 and GP3 as inputs */
    *(volatile unsigned int*)(IO_BANK0_BASE + 0x004 + (S3_IR_PIN_2 * 8)) = 5;
    *(volatile unsigned int*)(IO_BANK0_BASE + 0x004 + (S3_IR_PIN_3 * 8)) = 5;

    /* Set GP1 as output LED */
    *(volatile unsigned int*)(IO_BANK0_BASE + 0x004 + (S3_LED_PIN_1 * 8)) = 5;
    *(volatile unsigned int*)(SIO_BASE + 0x004) |= (1 << S3_LED_PIN_1); /* output enable */

    return E_OK;
}

EXPORT ER subsystem3_start(void)
{
    ID tid = app_start_task(consumer_task, S3_PRI_CONSUMER, TP_PRC1,
                "consumer_task");
    return (tid < E_OK) ? tid : E_OK;
}