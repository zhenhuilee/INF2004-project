/*
 *	task_consumer.c
 *	Subsystem 3 -- Consumer task with barcode pass logger and obstacle bypass.
 */

#include "subsystem3.h"
#include "../subsystem2/subsystem2.h"   
#include "../subsystem4/subsystem4.h"   
#include "../subsystem5/subsystem5.h"   /* Subsystem 5 API */
#include "sys/sysdef.h"
#include "../subsystem2/motion.h"
#include <stdbool.h>

/* RP2040 1 MHz free-running raw lower timer word */
#define S5_TIMER_TIMELR (*(volatile const uint32_t *)0x40054028u)

/* Minimum edge duration to count as real (filters out tiny noise glitches) */
#define BC_MIN_EDGE_US      5000u
/* Quiet time on white (1) that marks the end of a barcode pass */
#define BC_PASS_END_US      120000u
/* Max transitions stored per barcode pass */
#define BC_MAX_STEPS        32u
/* Minimum edges required to treat a pass as a barcode rather than a single blip */
#define BC_MIN_BARCODE_EDGES 4u

/* Bypass tuning (verify steer sign on your chassis before full runs) */
#define BYPASS_SPEED_MM_S     80
#define BYPASS_STEER_OUT      (-45)
#define BYPASS_STEER_BACK     (45)
#define BYPASS_TURN_MS        550u
#define BYPASS_EXTRA_BERTH_MM 80u     /* Extra width margin for a slightly wider berth */
#define BYPASS_MIN_DRIVE_MS   450u
#define BYPASS_MAX_DRIVE_MS   3200u
#define BYPASS_MAX_REACQ_MS   3500u
#define BYPASS_MIN_DWELL_MS   1500u   /* Minimum stop/dwell time while measuring width */

typedef enum {
    BYPASS_NONE = 0,
    BYPASS_WAITING_SWEEP,
    BYPASS_TURN_OUT,
    BYPASS_DRIVE_PAST,
    BYPASS_TURN_BACK,
    BYPASS_REACQUIRE_LINE
} bypass_state_t;

/* 1 = barcode detected, 0 = not detected */
static uint8_t barcode_detected = 0u;

static void bc_track_pass(unsigned int right_val)
{
    static uint8_t      active = 0u;
    static unsigned int prev_val = 1u;
    static uint32_t     last_edge_us = 0u;
    static uint32_t     pass_start_us = 0u;
    static uint32_t     last_activity_us = 0u;
    static uint32_t     bits_seen = 0u;
    static uint8_t      edge_cnt = 0u;
    static uint8_t      step_cnt = 0u;
    static uint8_t      step_state[BC_MAX_STEPS];
    static uint32_t     step_dur_us[BC_MAX_STEPS];

    uint32_t now_us = S5_TIMER_TIMELR;
    unsigned int cur = (right_val & 1u);

    /* Start a barcode pass as soon as black (0) appears */
    if (!active)
    {
        if (cur == 0u)
        {
            active = 1u;
            prev_val = 0u;
            last_edge_us = now_us;
            pass_start_us = now_us;
            last_activity_us = now_us;
            bits_seen = 0u;
            edge_cnt = 1u;
            step_cnt = 0u;

            tm_printf("[BC-PASS] START black(0) at %lu us\n", (unsigned long)now_us);
        }
        return;
    }

    /* Detect state change on R */
    if (cur != prev_val)
    {
        uint32_t dur_us = now_us - last_edge_us;

        /* Ignore ultra-short glitches */
        if (dur_us >= BC_MIN_EDGE_US)
        {
            if (step_cnt < BC_MAX_STEPS)
            {
                step_state[step_cnt] = (uint8_t)prev_val;
                step_dur_us[step_cnt] = dur_us;
                step_cnt++;
            }

            bits_seen = (bits_seen << 1) | cur;
            edge_cnt++;
            last_edge_us = now_us;
            last_activity_us = now_us;
        }
        prev_val = cur;
    }
    else if (cur == 0u)
    {
        /* Still on black (0) */
        last_activity_us = now_us;
    }
    else
    {
        /* On white (1): finish pass once quiet for BC_PASS_END_US */
        if ((now_us - last_activity_us) >= BC_PASS_END_US)
        {
            uint32_t end_white_us = (now_us - last_edge_us) - BC_PASS_END_US;
            uint32_t total_pass_ms = (last_activity_us - pass_start_us) / 1000u;

            if (edge_cnt >= BC_MIN_BARCODE_EDGES)
            {
                barcode_detected = 1u;
                tm_printf("barcode found (barcode_detected=%u)\n", (unsigned int)barcode_detected);
            }
            else
            {
                barcode_detected = 0u;
                tm_printf("barcode_detected=%u\n", (unsigned int)barcode_detected);
            }

            tm_printf("[BC-PASS] END: dur=%lu ms | edges=%u | bits=0x%lx | detected=%u\n",
                      (unsigned long)total_pass_ms,
                      (unsigned int)edge_cnt,
                      (unsigned long)bits_seen,
                      (unsigned int)barcode_detected);

            for (uint8_t i = 0u; i < step_cnt; i++)
            {
                tm_printf("  seg[%u]: val=%u held=%lu us\n",
                          (unsigned int)i,
                          (unsigned int)step_state[i],
                          (unsigned long)step_dur_us[i]);
            }
            if (end_white_us >= BC_MIN_EDGE_US && step_cnt < BC_MAX_STEPS)
            {
                tm_printf("  seg[%u]: val=1 held=%lu us (trailing)\n",
                          (unsigned int)step_cnt,
                          (unsigned long)end_white_us);
            }

            active = 0u;
            prev_val = 1u;
        }
    }
}

EXPORT void consumer_task(INT stacd, void *exinf)
{
    SAMPLE_MSG  *msg;
    UB          debug_buf[90];          
    UW          count;
    ER          er;
    
    unsigned int left, center, right;
    bool was_blocked = false;
    bool got_width = false;
    S5_OBSTACLE_INFO obs_info = {0};
    bypass_state_t bypass_state = BYPASS_NONE;
    uint32_t step_start_us = 0u;
    uint32_t drive_past_ms = 500u;
    uint32_t last_junc_ms = 0u;

	(void)stacd; (void)exinf;

    while (1) {
        uint32_t now_us = S5_TIMER_TIMELR;

        /* Buddy 4 pickup detection */
        if (s4_is_pickup_detected())
        {
            (void)motion_stop();
            tk_dly_tsk(25);
            continue;
        }

        /* 1. Check Subsystem 5 obstacle state or active bypass maneuver */
        if (subsystem5_is_blocked() || bypass_state != BYPASS_NONE) {
            if (!was_blocked && bypass_state == BYPASS_NONE) {
                (void)motion_stop();
                tm_putstring((const UB*)"[S3] Obstacle detected. Stopping longer & measuring width...\n");
                was_blocked = true;
                got_width = false;
                bypass_state = BYPASS_WAITING_SWEEP;
                step_start_us = now_us;
            }

            if (bypass_state == BYPASS_WAITING_SWEEP) {
                (void)motion_stop();

                if (!got_width && subsystem5_get_obstacle_info(&obs_info)) {
                    got_width = true;
                }

                /* Stay stopped for at least BYPASS_MIN_DWELL_MS and until obstacle width is read */
                if (got_width && (((now_us - step_start_us) / 1000u) >= BYPASS_MIN_DWELL_MS)) {
                    float width_cm = (obs_info.est_width_cm > 0.0f) ? obs_info.est_width_cm : obs_info.bypass_offset_cm;
                    uint32_t width_mm = (width_cm > 0.0f) ? (uint32_t)(width_cm * 10.0f) : 0u;
                    uint32_t offset_mm = (uint32_t)(obs_info.bypass_offset_cm * 10.0f);
                    uint32_t clearance_mm = ((width_mm > offset_mm) ? width_mm : offset_mm) + BYPASS_EXTRA_BERTH_MM;

                    drive_past_ms = (clearance_mm * 1000u) / (uint32_t)BYPASS_SPEED_MM_S;
                    if (drive_past_ms < BYPASS_MIN_DRIVE_MS) {
                        drive_past_ms = BYPASS_MIN_DRIVE_MS;
                    }
                    if (drive_past_ms > BYPASS_MAX_DRIVE_MS) {
                        drive_past_ms = BYPASS_MAX_DRIVE_MS;
                    }

                    tm_printf("[S3] Width~%d cm (+%u mm berth) -> straight=%lu ms, turn=%lu ms\n",
                              (int)obs_info.est_width_cm,
                              (unsigned int)BYPASS_EXTRA_BERTH_MM,
                              (unsigned long)drive_past_ms,
                              (unsigned long)BYPASS_TURN_MS);

                    bypass_state = BYPASS_TURN_OUT;
                    step_start_us = now_us;
                }
                tk_dly_tsk(25);
                continue;
            }

            if (bypass_state == BYPASS_TURN_OUT) {
                (void)motion_drive(BYPASS_SPEED_MM_S, BYPASS_STEER_OUT);
                if (((now_us - step_start_us) / 1000u) >= BYPASS_TURN_MS) {
                    bypass_state = BYPASS_DRIVE_PAST;
                    step_start_us = now_us;
                }
                tk_dly_tsk(20);
                continue;
            }

            if (bypass_state == BYPASS_DRIVE_PAST) {
                (void)motion_drive(BYPASS_SPEED_MM_S, 0);
                if (((now_us - step_start_us) / 1000u) >= drive_past_ms) {
                    bypass_state = BYPASS_TURN_BACK;
                    step_start_us = now_us;
                }
                tk_dly_tsk(20);
                continue;
            }

            if (bypass_state == BYPASS_TURN_BACK) {
                (void)motion_drive(BYPASS_SPEED_MM_S, BYPASS_STEER_BACK);
                if (((now_us - step_start_us) / 1000u) >= BYPASS_TURN_MS) {
                    bypass_state = BYPASS_REACQUIRE_LINE;
                    step_start_us = now_us;
                    tm_putstring((const UB*)"[S3] Reacquiring line (curving left)...\n");
                }
                tk_dly_tsk(20);
                continue;
            }

            if (bypass_state == BYPASS_REACQUIRE_LINE) {
                left   = (*(volatile unsigned int*)(SIO_BASE + 0x004) >> S3_IR_PIN_LEFT) & 1;
                center = (*(volatile unsigned int*)(SIO_BASE + 0x004) >> S3_IR_PIN_CENTER) & 1;
                right  = (*(volatile unsigned int*)(SIO_BASE + 0x004) >> S3_IR_PIN_RIGHT) & 1;

                bool found_line = (!left || !center || !right);
                bool timed_out  = (((now_us - step_start_us) / 1000u) >= BYPASS_MAX_REACQ_MS);

                /* Black = 0: rejoin line tracking once any IR sensor sees black, or on timeout */
                if (found_line || timed_out) {
                    subsystem5_clear_obstacle_info();
                    was_blocked = false;
                    got_width = false;
                    bypass_state = BYPASS_NONE;
                    tm_printf("[S3] Bypass %s (L:%u C:%u R:%u). Resuming line tracking.\n",
                              found_line ? "rejoined line" : "timed out",
                              left, center, right);
                    continue;
                }

                /* Gentle left curve toward the line instead of driving straight parallel */
                (void)motion_drive(BYPASS_SPEED_MM_S, 20);
                tk_dly_tsk(20);
                continue;
            }


        }

        if (was_blocked) {
            tm_putstring((const UB*)"[S3] Path clear. Resuming movement.\n");
            was_blocked = false;
            got_width = false;
        }

        /* 2. Read the 3 IR sensors */
        left   = (*(volatile unsigned int*)(SIO_BASE + 0x004) >> S3_IR_PIN_LEFT) & 1;
        center = (*(volatile unsigned int*)(SIO_BASE + 0x004) >> S3_IR_PIN_CENTER) & 1;
        right  = (*(volatile unsigned int*)(SIO_BASE + 0x004) >> S3_IR_PIN_RIGHT) & 1;

        bc_track_pass(right); /* barcode pass tracker */

        /* 3. Line Tracking & Motor Control via Subsystem 2 API (slower speed) */
        int32_t base_speed = 80;   /* mm/s forward speed */
        int32_t steer_term = 0;
        uint32_t now_ms = now_us / 1000u;

        if (left && center && ((now_ms - last_junc_ms) >= 600u || last_junc_ms == 0u)) {
            last_junc_ms = now_ms;

            if (barcode_detected) {
                tm_sprintf(debug_buf, (const UB*)"[IR-DEBUG] [JUNCTION] L:%u C:%u R:%u | Barcode=1 -> TurnLeft90\n",
                           left, center, right);
                tm_putstring(debug_buf);

                /* Drive forward a few cm so the robot pivots right on the junction */
                int32_t fwd_seq = motion_move_forward(90, base_speed);
                if (fwd_seq > 0) {
                    (void)motion_wait_done(fwd_seq, 2000);
                }

                int32_t turn_seq = motion_turn_left(90, MOTION_DEFAULT_SPEED);
                if (turn_seq > 0) {
                    (void)motion_wait_done(turn_seq, 3000);
                    last_junc_ms = S5_TIMER_TIMELR / 1000u;
                }

                /* Consume detection so it only triggers once */
                barcode_detected = 0u;
            } else {
                tm_sprintf(debug_buf, (const UB*)"[IR-DEBUG] [JUNCTION] L:%u C:%u R:%u | Barcode=0 -> Skip Left Turn\n",
                           left, center, right);
                tm_putstring(debug_buf);

                /* Continue straight through the junction when no barcode was detected */
                (void)motion_drive(base_speed, 0);
            }
        } else {
            if (left && !center) {
                steer_term = -30; //right
            } else if (!left && center) {
                steer_term = 30;  //left
            } else {
                steer_term = 0;
            }

            /* Send continuous drive command safely without constant overrides */
            (void)motion_drive(base_speed, steer_term);

            /* --- LIVE IR MONITORING PRINT STATEMENT (ONLY ON BLACK DETECTION) --- */
            if (!left || !center || !right) {
                tm_sprintf(debug_buf, (const UB*)"[IR-DEBUG] L:%u C:%u R:%u | Steer:%ld\n",
                           left, center, right, (long)steer_term);
                tm_putstring(debug_buf);
            }
        }

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