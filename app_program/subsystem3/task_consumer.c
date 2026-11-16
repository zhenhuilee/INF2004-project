/*
 * task_consumer.c
 * Subsystem 3 -- the consumer task.
 */

#include "subsystem3.h"
#include "../subsystem2/subsystem2.h"   /* IF-01/02: SAMPLE_MSG, pool, slots */
#include "../subsystem4/subsystem4.h"   /* IF-03: mbf_summary, flg_events */
#include "sys/sysdef.h"
#include "../subsystem2/motion.h"

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
        /* 1. Read the 3 IR sensors */
        left   = (*(volatile unsigned int*)(SIO_BASE + 0x004) >> S3_IR_PIN_LEFT) & 1;
        center = (*(volatile unsigned int*)(SIO_BASE + 0x004) >> S3_IR_PIN_CENTER) & 1;
        right  = (*(volatile unsigned int*)(SIO_BASE + 0x004) >> S3_IR_PIN_RIGHT) & 1;

        /* 2. Line Tracking & Motor Control via Subsystem 2 API */
        int32_t base_speed = 150;  /* mm/s forward speed */
        int32_t steer_term = 0;    /* positive steers left, negative steers right */

        if (left && !center) {
            // Drifting right off the line -> steer left
            steer_term = 50; 
        } else if (!left && center) {
            // Drifting left off the line -> steer right
            steer_term = -50;
        } else {
            // Centered on track -> go straight
            steer_term = 0;
        }

        /* Send continuous drive command immediately */
        (void)motion_drive(base_speed, steer_term);

        /* --- LIVE IR MONITORING PRINT STATEMENT --- */
        tm_sprintf(debug_buf, (const UB*)"[IR-DEBUG] L:%u C:%u R:%u | Steer:%ld\n", 
                   left, center, right, (long)steer_term);
        tm_putstring(debug_buf);

        /* 3. Check for a sample message with a 20 ms timeout (non-blocking for steering) */
        er = tk_rcv_mbx(mbx_samples, (T_MSG **)&msg, 20);
        
        if (er == E_OK) {
            /* --- Sample received: update shared statistics (IF-04) --- */
            tk_loc_mtx(mtx_stats, TMO_FEVR);
            
            if (app_stats.received == 0 || msg->value < app_stats.min) {
                app_stats.min = msg->value;
            }
            if (app_stats.received == 0 || msg->value > app_stats.max) {
                app_stats.max = msg->value;
            }
            app_stats.sum += msg->value;
            count = ++app_stats.received;

            app_stats.barcode_history = (UB)((app_stats.barcode_history << 1) | (right & 1));

            if (!left && center) {
                app_stats.last_instruction = S3_INST_STRAIGHT;
            } else if (left && !center) {
                app_stats.last_instruction = S3_INST_RIGHT; 
            } else if (!center) { 
                app_stats.last_instruction = S3_INST_LEFT;  
            } else {
                app_stats.last_instruction = S3_INST_NONE;  
            }

            tk_unl_mtx(mtx_stats);

            /* Release sample memory block and slot semaphore back to Subsystem 2 */
            tk_rel_mpf(mpf_samples, msg);
            tk_sig_sem(sem_slots, 1);
        }
    }
}