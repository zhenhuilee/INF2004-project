/*
 * task_consumer.c
 * Subsystem 3 -- Consumer task with non-stuttering obstacle guard.
 */

#include "subsystem3.h"
#include "../subsystem2/subsystem2.h"   
#include "../subsystem4/subsystem4.h"   
#include "../subsystem5/subsystem5.h"   /* Subsystem 5 API */
#include "sys/sysdef.h"
#include "../subsystem2/motion.h"
#include <stdbool.h>

EXPORT void consumer_task(INT stacd, void *exinf)
{
    SAMPLE_MSG  *msg;
    UB          debug_buf[90];          
    UW          count;
    ER          er;
    
    unsigned int left, center, right;
    bool was_blocked = false;

    (void)stacd; (void)exinf;

    tm_putstring((const UB*)"[S3] Consumer task started. Monitoring sensors...\n");

    while (1) {
        /* 1. Check Subsystem 5 obstacle state */
        if (subsystem5_is_blocked()) {
            if (!was_blocked) {
                /* Issue stop command EXACTLY ONCE when first blocked */
                (void)motion_stop();
                tm_putstring((const UB*)"[S3] Motors stopped due to obstacle.\n");
                was_blocked = true;
            }
            
            /* Skip line tracking and motor driving while blocked */
            tk_dly_tsk(50);
            continue;
        }

        /* If we just cleared up, reset the flag */
        if (was_blocked) {
            tm_putstring((const UB*)"[S3] Path clear. Resuming movement.\n");
            was_blocked = false;
        }

        /* 2. Read the 3 IR sensors */
        left   = (*(volatile unsigned int*)(SIO_BASE + 0x004) >> S3_IR_PIN_LEFT) & 1;
        center = (*(volatile unsigned int*)(SIO_BASE + 0x004) >> S3_IR_PIN_CENTER) & 1;
        right  = (*(volatile unsigned int*)(SIO_BASE + 0x004) >> S3_IR_PIN_RIGHT) & 1;

        /* 3. Line Tracking & Motor Control via Subsystem 2 API */
        int32_t base_speed = 150;  /* mm/s forward speed */
        int32_t steer_term = 0;    

        if (left && !center) {
            steer_term = -50; //left og 50
        } else if (!left && center) {
            steer_term = 50; //right og -50
        } else {
            steer_term = 0; //straight
        }

        /* Send continuous drive command safely without constant overrides */
        (void)motion_drive(base_speed, steer_term);

        /* --- LIVE IR MONITORING PRINT STATEMENT --- */
        tm_sprintf(debug_buf, (const UB*)"[IR-DEBUG] L:%u C:%u R:%u | Steer:%ld\n", 
                   left, center, right, (long)steer_term);
        tm_putstring(debug_buf);

        /* 4. Check for a sample message with a 20 ms timeout */
        er = tk_rcv_mbx(mbx_samples, (T_MSG **)&msg, 20);
        
        if (er == E_OK) {
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

            tk_rel_mpf(mpf_samples, msg);
            tk_sig_sem(sem_slots, 1);
        }
    }
}