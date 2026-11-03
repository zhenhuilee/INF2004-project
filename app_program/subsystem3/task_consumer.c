/*
 * task_consumer.c
 * Subsystem 3 -- the consumer task.
 */

#include "subsystem3.h"
#include "../subsystem2/subsystem2.h"   /* IF-01/02: SAMPLE_MSG, pool, slots */
#include "../subsystem4/subsystem4.h"   /* IF-03: mbf_summary, flg_events */
#include "sys/sysdef.h"

EXPORT void consumer_task(INT stacd, void *exinf)
{
    SAMPLE_MSG  *msg;
    UB          text[S4_SUMMARY_MAX];   /* tm_sprintf adds a '\0' */
    UB          debug_buf[90];          /* Buffer for formatted debug output */
    INT         len;
    UW          count;
    ER          er;
    
    unsigned int left, center, right;

    (void)stacd; (void)exinf;

    tm_putstring((const UB*)"[S3] Consumer task started. Monitoring sensors...\n");

    while (1) {
        /* 1. Read the 3 IR sensors via SIO input register (offset 0x004) */
        left   = (*(volatile unsigned int*)(SIO_BASE + 0x004) >> S3_IR_PIN_LEFT) & 1;
        center = (*(volatile unsigned int*)(SIO_BASE + 0x004) >> S3_IR_PIN_CENTER) & 1; //we want black line
        right  = (*(volatile unsigned int*)(SIO_BASE + 0x004) >> S3_IR_PIN_RIGHT) & 1;

        /* 2. 2-Sensor Line Tracking Logic & LED Control */
        if (left || center) {
            *(volatile unsigned int*)(SIO_BASE + 0x010) = (1 << S3_LED_PIN_1); /* Set LED high */
        } else {
            *(volatile unsigned int*)(SIO_BASE + 0x014) = (1 << S3_LED_PIN_1); /* Set LED low */
        }

        /* 3. Wait for the next sample from Subsystem 2 */
        er = tk_rcv_mbx(mbx_samples, (T_MSG **)&msg, TMO_FEVR);
        if (er < E_OK) {
            CHECK(er, "tk_rcv_mbx");
            continue;
        }

        /* 4. Update shared statistics and instruction/history state (IF-04) */
        tk_loc_mtx(mtx_stats, TMO_FEVR);
        
        if (app_stats.received == 0 || msg->value < app_stats.min) {
            app_stats.min = msg->value;
        }
        if (app_stats.received == 0 || msg->value > app_stats.max) {
            app_stats.max = msg->value;
        }
        app_stats.sum += msg->value;
        count = ++app_stats.received;

        /* Shift and update barcode history with Right sensor */
        app_stats.barcode_history = (UB)((app_stats.barcode_history << 1) | (right & 1));

        /* Determine instruction based on 2-sensor state */
        if (!left && center) {
            app_stats.last_instruction = S3_INST_STRAIGHT;
        } else if (left && !center) {
            app_stats.last_instruction = S3_INST_RIGHT; 
        } else if (!center) { 
            app_stats.last_instruction = S3_INST_LEFT;  
        } else {
            app_stats.last_instruction = S3_INST_NONE;  //both just like in the air or smt
        }

        /* Capture local copies of stats for safe printing outside mutex lock */
        UW cur_received = app_stats.received;
        UB cur_inst = app_stats.last_instruction;
        UB cur_history = app_stats.barcode_history;

        tk_unl_mtx(mtx_stats);

        /* Map instruction code to descriptive text string */
        const UB *inst_str;
        switch (cur_inst) {
            case S3_INST_LEFT:     inst_str = (const UB*)"drifting left"; break;
            case S3_INST_RIGHT:    inst_str = (const UB*)"drifting right"; break;
            case S3_INST_STRAIGHT: inst_str = (const UB*)"straight"; break;
            default:               inst_str = (const UB*)"off track"; break;
        }

        /* 5. PRINT OUT REAL-TIME OUTPUTS TO CONSOLE */
        tm_sprintf(debug_buf, (const UB*)"[S3-OUT] L:%u C:%u R:%u | LED:%u | Val:%d | Inst: %s | Hist:0x%02X | Total:%u\n",
                   left, center, right, 
                   (left || center), 
                   msg->value, 
                   inst_str, 
                   cur_history, 
                   cur_received);
        tm_putstring(debug_buf);

        /* 6. Every S3_BATCH_SIZE samples, send summary to Subsystem 4 (IF-03) */
        if (count % S3_BATCH_SIZE == 0) {
            len = tm_sprintf(text, (const UB*)"n=%u last=%d inst=%u core=%d",
                     count, msg->value, cur_inst, msg->prc - 1);

            er = tk_snd_mbf(mbf_summary, text, len, TMO_POL);
            if (er == E_OK) {
                tk_set_flg(flg_events, FLG_BATCH);
            }
        }

        /* 7. Release sample memory block and slot semaphore back to Subsystem 2 */
        tk_rel_mpf(mpf_samples, msg);
        tk_sig_sem(sem_slots, 1);
    }
}