#include "subsystem3.h"
#include "../subsystem2/subsystem2.h"   
#include "../subsystem4/subsystem4.h"   
#include "../subsystem5/subsystem5.h"   
#include "sys/sysdef.h"
#include "../subsystem2/motion.h"
#include <stdbool.h>
#include <stdint.h>

#define S5_TIMER_TIMELR (*(volatile const uint32_t *)0x40054028u)

#define BC_MIN_EDGE_US       2500u
#define BC_PASS_END_US       120000u
#define BC_MAX_STEPS         64u
#define BC_MIN_BARCODE_EDGES 9u

#define BYPASS_SPEED_MM_S     80
#define BYPASS_STEER_OUT      (-45)
#define BYPASS_STEER_BACK     (45)
#define BYPASS_TURN_MS        550u
#define BYPASS_EXTRA_BERTH_MM 80u
#define BYPASS_MIN_DRIVE_MS   450u
#define BYPASS_MAX_DRIVE_MS   3200u
#define BYPASS_MAX_REACQ_MS   3500u
#define BYPASS_MIN_DWELL_MS   3500u

typedef enum {
    BYPASS_NONE = 0,
    BYPASS_WAITING_SWEEP,
    BYPASS_BACK_UP,       
    BYPASS_TURN_OUT,
    BYPASS_DRIVE_PAST,
    BYPASS_TURN_BACK,
    BYPASS_REACQUIRE_LINE
} bypass_state_t;
typedef enum {
    BC_ACT_NONE = 0,
    BC_ACT_FORWARD,
    BC_ACT_U_TURN,
    BC_ACT_LEFT,
    BC_ACT_RIGHT
} bc_action_t;

typedef struct {
    char     letter;
    uint16_t code9;
} code39_alpha_t;

static const code39_alpha_t CODE39_ALPHA[26] = {
    { 'A', 0x109u }, { 'B', 0x049u }, { 'C', 0x148u }, { 'D', 0x019u },
    { 'E', 0x118u }, { 'F', 0x058u }, { 'G', 0x00Du }, { 'H', 0x10Cu },
    { 'I', 0x04Cu }, { 'J', 0x01Cu }, { 'K', 0x103u }, { 'L', 0x043u },
    { 'M', 0x142u }, { 'N', 0x013u }, { 'O', 0x112u }, { 'P', 0x052u },
    { 'Q', 0x007u }, { 'R', 0x106u }, { 'S', 0x046u }, { 'T', 0x016u },
    { 'U', 0x181u }, { 'V', 0x0C1u }, { 'W', 0x1C0u }, { 'X', 0x091u },
    { 'Y', 0x190u }, { 'Z', 0x0D0u }
};

static uint8_t barcode_detected = 0u;
static uint8_t bc_pass_array[BC_MAX_STEPS] = {0};
static uint8_t bc_pass_len = 0u;
static char stored_barcode_char = '\0';
static bc_action_t char_action_map[26] = {BC_ACT_NONE};

static bool str_starts_with(const char *s, const char *prefix, uint32_t len)
{
    for (uint32_t i = 0u; i < len; i++) {
        if (s[i] == '\0' || s[i] != prefix[i]) {
            return false;
        }
    }
    return true;
}

static const char *find_char_ptr(const char *s, char target)
{
    while (*s != '\0') {
        if (*s == target) {
            return s;
        }
        s++;
    }
    return 0;
}

void s3_set_char_action(char letter, bc_action_t action)
{
    if (letter >= 'a' && letter <= 'z') {
        letter = (char)(letter - 'a' + 'A');
    }
    if (letter >= 'A' && letter <= 'Z') {
        char_action_map[(uint32_t)(letter - 'A')] = action;
    }
}

void s3_clear_action_map(void)
{
    for (uint32_t i = 0u; i < 26u; i++) {
        char_action_map[i] = BC_ACT_NONE;
    }
}

void s3_apply_mqtt_mapping(const char *payload)
{
    if (!payload) return;

    s3_clear_action_map();

    const char *p = payload;
    while (*p != '\0') {
        if (p[0] == 'F' && p[1] == '=') {
            if (p[2] >= 'A' && p[2] <= 'Z') s3_set_char_action(p[2], BC_ACT_FORWARD);
        } else if (p[0] == 'U' && p[1] == '=') {
            if (p[2] >= 'A' && p[2] <= 'Z') s3_set_char_action(p[2], BC_ACT_U_TURN);
        } else if (p[0] == 'L' && p[1] == '=') {
            if (p[2] >= 'A' && p[2] <= 'Z') s3_set_char_action(p[2], BC_ACT_LEFT);
        } else if (p[0] == 'R' && p[1] == '=') {
            if (p[2] >= 'A' && p[2] <= 'Z') s3_set_char_action(p[2], BC_ACT_RIGHT);
        } else if (str_starts_with(p, "\"forward\"", 9u)) {
            const char *q = find_char_ptr(p + 9, ':');
            if (q) {
                const char *c = find_char_ptr(q, '"');
                if (c && c[1] >= 'A' && c[1] <= 'Z') s3_set_char_action(c[1], BC_ACT_FORWARD);
            }
        } else if (str_starts_with(p, "\"uturn\"", 7u)) {
            const char *q = find_char_ptr(p + 7, ':');
            if (q) {
                const char *c = find_char_ptr(q, '"');
                if (c && c[1] >= 'A' && c[1] <= 'Z') s3_set_char_action(c[1], BC_ACT_U_TURN);
            }
        } else if (str_starts_with(p, "\"left\"", 6u)) {
            const char *q = find_char_ptr(p + 6, ':');
            if (q) {
                const char *c = find_char_ptr(q, '"');
                if (c && c[1] >= 'A' && c[1] <= 'Z') s3_set_char_action(c[1], BC_ACT_LEFT);
            }
        } else if (str_starts_with(p, "\"right\"", 7u)) {
            const char *q = find_char_ptr(p + 7, ':');
            if (q) {
                const char *c = find_char_ptr(q, '"');
                if (c && c[1] >= 'A' && c[1] <= 'Z') s3_set_char_action(c[1], BC_ACT_RIGHT);
            }
        }
        p++;
    }
}

// Ratio-based Code 39 decoder: resilient to line-following jitters and speed variations
static char bc_decode_code39(const uint32_t *durs, uint8_t count)
{
    if (count < 18u) {
        return '\0';
    }

    // Slide across edge windows to locate the data character block
    for (uint8_t start_idx = 0u; start_idx + 9u <= count; start_idx++) {
        uint32_t work[9];
        bool valid = true;

        for (uint8_t i = 0u; i < 9u; i++) {
            work[i] = durs[start_idx + i];
            if (work[i] < BC_MIN_EDGE_US) {
                valid = false;
                break;
            }
        }
        if (!valid) continue;

        // Find the shortest edge in this window to establish the baseline "narrow" stripe width
        uint32_t min_dur = work[0];
        for (uint8_t i = 1u; i < 9u; i++) {
            if (work[i] < min_dur) {
                min_dur = work[i];
            }
        }

        // Code 39 wide-to-narrow ratio is nominally 2.0 to 3.0. 
        // Anything >= 1.6 times the minimum duration is classified as Wide (1), 
        // else Narrow (0). This ratio method completely cancels out speed jitters.
        uint16_t computed_code = 0u;
        for (uint8_t i = 0u; i < 9u; i++) {
            computed_code <<= 1;
            if (work[i] >= (min_dur * 16u / 10u)) {
                computed_code |= 1u;
            }
        }

        // Check if it matches 'A' (0x109u)
        if (computed_code == 0x109u) {
            return 'A';
        }

        // Match against any other A-Z lookup entry
        for (uint8_t i = 0u; i < 26u; i++) {
            if (CODE39_ALPHA[i].code9 == computed_code) {
                return CODE39_ALPHA[i].letter;
            }
        }
    }

    return '\0';
}

static void bc_print_only_barcode(char ch, const uint32_t *durs, uint8_t count)
{
    if (ch >= 'A' && ch <= 'Z') {
        tm_printf("Decoded Character: %c\n", ch);
    } else {
        tm_printf("Decoded Character: Unknown / Unmatched\n");
    }

    tm_printf("Stripe Durations (us): [");
    for (uint8_t i = 0u; i < count; i++) {
        tm_printf("%u%s", (unsigned int)durs[i], (i + 1u < count) ? "," : "");
    }
    tm_printf("]\n");
}

static void bc_track_pass(unsigned int right_val)
{
    static uint8_t      active = 0u;
    static unsigned int prev_val = 1u;
    static uint32_t     last_edge_us = 0u;
    static uint32_t     last_activity_us = 0u;
    static uint8_t      edge_cnt = 0u;
    static uint8_t      step_cnt = 0u;
    static uint8_t      step_state[BC_MAX_STEPS];
    static uint32_t     step_dur_us[BC_MAX_STEPS];

    uint32_t now_us = S5_TIMER_TIMELR;
    unsigned int cur = (right_val & 1u);

    if (!active)
    {
        if (cur == 0u)
        {
            active = 1u;
            prev_val = 0u;
            last_edge_us = now_us;
            last_activity_us = now_us;
            edge_cnt = 1u;
            step_cnt = 0u;
        }
        return;
    }

    if (cur != prev_val)
    {
        uint32_t dur_us = now_us - last_edge_us;

        if (dur_us >= BC_MIN_EDGE_US)
        {
            if (step_cnt < BC_MAX_STEPS)
            {
                step_state[step_cnt] = (uint8_t)prev_val;
                step_dur_us[step_cnt] = dur_us;
                step_cnt++;
            }

            edge_cnt++;
            last_edge_us = now_us;
            last_activity_us = now_us;
        }
        prev_val = cur;
    }
    else if (cur == 0u)
    {
        last_activity_us = now_us;
    }
    else
    {
        if ((now_us - last_activity_us) >= BC_PASS_END_US)
        {
            if (edge_cnt >= BC_MIN_BARCODE_EDGES)
            {
                barcode_detected = 1u;
                bc_pass_len = step_cnt;
                for (uint8_t i = 0u; i < step_cnt; i++)
                {
                    bc_pass_array[i] = step_state[i];
                }

                char decoded = bc_decode_code39(step_dur_us, step_cnt);
                if (decoded >= 'A' && decoded <= 'Z')
                {
                    stored_barcode_char = decoded;
                }
                
                bc_print_only_barcode(decoded, step_dur_us, step_cnt);
            }
            else
            {
                barcode_detected = 0u;
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

        if (s4_is_pickup_detected())
        {
            (void)motion_stop();
            tk_dly_tsk(25);
            continue;
        }

        if (subsystem5_is_blocked() || bypass_state != BYPASS_NONE) {
            if (!was_blocked && bypass_state == BYPASS_NONE) {
                (void)motion_stop();
                tm_putstring((const UB*)"[S3] Obstacle detected. Scanning and backing up...\n");
                was_blocked = true;
                got_width = false;
                bypass_state = BYPASS_WAITING_SWEEP;
                step_start_us = now_us;
            }

            // Step 1: Scan / Dwell
            if (bypass_state == BYPASS_WAITING_SWEEP) {
                (void)motion_stop();

                S5_OBSTACLE_INFO sample_obs;
                if (subsystem5_get_obstacle_info(&sample_obs)) {
                    if (sample_obs.est_width_cm > 0.0f) {
                        obs_info = sample_obs;
                        got_width = true;
                    }
                }

                if (((now_us - step_start_us) / 1000u) >= BYPASS_MIN_DWELL_MS) {
                    float width_cm = (obs_info.est_width_cm > 0.0f) ? obs_info.est_width_cm : obs_info.bypass_offset_cm;
                    uint32_t width_mm = (width_cm > 0.0f) ? (uint32_t)(width_cm * 10.0f) : 0u;
                    uint32_t offset_mm = (uint32_t)(obs_info.bypass_offset_cm * 10.0f);
                    uint32_t clearance_mm = ((width_mm > offset_mm) ? width_mm : offset_mm) + BYPASS_EXTRA_BERTH_MM;

                    drive_past_ms = (clearance_mm * 1000u) / (uint32_t)BYPASS_SPEED_MM_S;
                    if (drive_past_ms < 800u) {
                        drive_past_ms = 800u;
                    }
                    if (drive_past_ms > BYPASS_MAX_DRIVE_MS) {
                        drive_past_ms = BYPASS_MAX_DRIVE_MS;
                    }

                    // Transition to backing up before turning out
                    bypass_state = BYPASS_BACK_UP;
                    step_start_us = now_us;
                }
                tk_dly_tsk(25);
                continue;
            }

            // Step 2: Back up slightly to clear the object safely
            if (bypass_state == BYPASS_BACK_UP) {
                // Negative speed (-60) moves the robot backward
                (void)motion_drive(-60, 0); 
                if (((now_us - step_start_us) / 1000u) >= 500u) { // Back up for 500 ms
                    bypass_state = BYPASS_TURN_OUT;
                    step_start_us = now_us;
                }
                tk_dly_tsk(20);
                continue;
            }

            // Step 3: Turn Right sharply to step out of the obstacle path
            if (bypass_state == BYPASS_TURN_OUT) {
                (void)motion_drive(BYPASS_SPEED_MM_S, 50); 
                if (((now_us - step_start_us) / 1000u) >= 700u) { 
                    bypass_state = BYPASS_DRIVE_PAST;
                    step_start_us = now_us;
                }
                tk_dly_tsk(20);
                continue;
            }

            // Step 4: Drive Straight parallel to the obstacle
            if (bypass_state == BYPASS_DRIVE_PAST) {
                (void)motion_drive(BYPASS_SPEED_MM_S, 0);
                if (((now_us - step_start_us) / 1000u) >= drive_past_ms) {
                    bypass_state = BYPASS_TURN_BACK;
                    step_start_us = now_us;
                }
                tk_dly_tsk(20);
                continue;
            }

            // Step 5: Turn Left back toward the track line
            if (bypass_state == BYPASS_TURN_BACK) {
                (void)motion_drive(BYPASS_SPEED_MM_S, -80); 
                if (((now_us - step_start_us) / 1000u) >= 700u) {
                    bypass_state = BYPASS_REACQUIRE_LINE;
                    step_start_us = now_us;
                }
                tk_dly_tsk(20);
                continue;
            }

            // Step 6: Actively scan IR sensors until the track line is re-acquired
            if (bypass_state == BYPASS_REACQUIRE_LINE) {
                left   = (*(volatile unsigned int*)(SIO_BASE + 0x004) >> S3_IR_PIN_LEFT) & 1;
                center = (*(volatile unsigned int*)(SIO_BASE + 0x004) >> S3_IR_PIN_CENTER) & 1;
                right  = (*(volatile unsigned int*)(SIO_BASE + 0x004) >> S3_IR_PIN_RIGHT) & 1;

                bool found_line = (!left || !center || !right);
                bool timed_out  = (((now_us - step_start_us) / 1000u) >= BYPASS_MAX_REACQ_MS);

                if (found_line || timed_out) {
                    subsystem5_clear_obstacle_info();
                    was_blocked = false;
                    got_width = false;
                    bypass_state = BYPASS_NONE;
                    continue;
                }

                (void)motion_drive(BYPASS_SPEED_MM_S, -20);
                tk_dly_tsk(20);
                continue;
            }
        }

        if (was_blocked) {
            was_blocked = false;
            got_width = false;
        }

        left   = (*(volatile unsigned int*)(SIO_BASE + 0x004) >> S3_IR_PIN_LEFT) & 1;
        center = (*(volatile unsigned int*)(SIO_BASE + 0x004) >> S3_IR_PIN_CENTER) & 1;
        right  = (*(volatile unsigned int*)(SIO_BASE + 0x004) >> S3_IR_PIN_RIGHT) & 1;

        bc_track_pass(right);

        int32_t base_speed = 60;
        int32_t steer_term = 0;
        uint32_t now_ms = now_us / 1000u;

        if (left && center && ((now_ms - last_junc_ms) >= 600u || last_junc_ms == 0u)) {
            last_junc_ms = now_ms;

            if (stored_barcode_char == 'A') {
                bc_action_t action = char_action_map[0];

                switch (action) {
                    case BC_ACT_FORWARD:
                        {
                            int32_t fwd_seq = motion_move_forward(90, base_speed);
                            if (fwd_seq > 0) { (void)motion_wait_done(fwd_seq, 2000); }
                        }
                        break;
                    case BC_ACT_LEFT:
                        {
                            int32_t turn_seq = motion_turn_left(90, MOTION_DEFAULT_SPEED);
                            if (turn_seq > 0) { (void)motion_wait_done(turn_seq, 3000); }
                        }
                        break;
                    case BC_ACT_RIGHT:
                        {
                            int32_t turn_seq = motion_turn_right(90, MOTION_DEFAULT_SPEED);
                            if (turn_seq > 0) { (void)motion_wait_done(turn_seq, 3000); }
                        }
                        break;
                    case BC_ACT_U_TURN:
                        {
                            int32_t turn_seq = motion_turn_left(180, MOTION_DEFAULT_SPEED);
                            if (turn_seq > 0) { (void)motion_wait_done(turn_seq, 4000); }
                        }
                        break;
                    case BC_ACT_NONE:
                    default:
                        {
                            int32_t fwd_seq = motion_move_forward(90, base_speed);
                            if (fwd_seq > 0) { (void)motion_wait_done(fwd_seq, 2000); }

                            int32_t turn_seq = motion_turn_left(90, MOTION_DEFAULT_SPEED);
                            if (turn_seq > 0) { (void)motion_wait_done(turn_seq, 3000); }
                        }
                        break;
                }

                stored_barcode_char = '\0';
            } else {
                (void)motion_drive(base_speed, 0);
                stored_barcode_char = '\0';
            }
        } else {
            if (left && !center) {
                steer_term = -40;
            } else if (!left && center) {
                steer_term = 40;
            } else {
                steer_term = 0;
            }

            (void)motion_drive(base_speed, steer_term);
        }

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