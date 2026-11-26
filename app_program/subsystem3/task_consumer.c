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
#define BC_MAX_STRIPE_US     250000u
#define BC_PASS_END_US       120000u
#define BC_MAX_STEPS         64u
#define BC_MIN_BARCODE_EDGES 9u
#define BC_UTURN_ANGLE_DEG   190
#define CODE39_ASTERISK_CODE 0x094u

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

/* Four single-character variables mapped to the four junction movements */
static char bc_char_left    = 'A';
static char bc_char_forward = 'B';
static char bc_char_right   = 'C';
static char bc_char_uturn   = 'D';

static char last_diag_ch = '?';
static float last_diag_err = -1.0f;
static float last_diag_margin = -1.0f;
static int last_matched_start = -1;
static bool last_matched_reversed = false;
static bool last_matched_after_star = false;
static uint32_t last_matched_win[9] = {0};

static char norm_upper_alpha(char ch)
{
    if (ch >= 'a' && ch <= 'z') {
        return (char)(ch - 'a' + 'A');
    }
    if (ch >= 'A' && ch <= 'Z') {
        return ch;
    }
    return '\0';
}

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
    char ch = norm_upper_alpha(letter);
    if (ch == '\0') return;

    switch (action) {
        case BC_ACT_LEFT:
            bc_char_left = ch;
            break;
        case BC_ACT_FORWARD:
            bc_char_forward = ch;
            break;
        case BC_ACT_RIGHT:
            bc_char_right = ch;
            break;
        case BC_ACT_U_TURN:
            bc_char_uturn = ch;
            break;
        case BC_ACT_NONE:
        default:
            if (bc_char_left == ch)    bc_char_left = '\0';
            if (bc_char_forward == ch) bc_char_forward = '\0';
            if (bc_char_right == ch)   bc_char_right = '\0';
            if (bc_char_uturn == ch)   bc_char_uturn = '\0';
            break;
    }
}

void s3_clear_action_map(void)
{
    bc_char_left    = '\0';
    bc_char_forward = '\0';
    bc_char_right   = '\0';
    bc_char_uturn   = '\0';
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

static bool bc_normalize_window(const uint32_t widths[9], float out_norm[9])
{
    uint32_t sum = 0u;
    uint32_t min_w = widths[0];
    uint32_t max_w = widths[0];

    for (int i = 0; i < 9; i++) {
        if (widths[i] < BC_MIN_EDGE_US || widths[i] > BC_MAX_STRIPE_US) {
            return false;
        }
        sum += widths[i];
        if (widths[i] < min_w) min_w = widths[i];
        if (widths[i] > max_w) max_w = widths[i];
    }

    if (sum == 0u || max_w < (min_w * 4u) / 3u) {
        return false;
    }

    float mean = (float)sum / 9.0f;
    for (int i = 0; i < 9; i++) {
        out_norm[i] = (float)widths[i] / mean;
    }
    return true;
}

static float bc_score_against_mask(const float norm_widths[9], uint16_t code9, float wide_ratio)
{
    float w_n = 9.0f / (6.0f + 3.0f * wide_ratio);
    float w_w = w_n * wide_ratio;
    float err_sum = 0.0f;

    for (int i = 0; i < 9; i++) {
        bool is_wide = ((code9 >> (8 - i)) & 1u) != 0u;
        float expected = is_wide ? w_w : w_n;
        float diff = norm_widths[i] - expected;
        err_sum += diff * diff;
    }
    return err_sum / 9.0f;
}

static void bc_eval_candidate_order(const uint32_t raw_win[9],
                                  bool reversed,
                                  float wide_ratio,
                                  char *out_ch,
                                  float *out_best_err,
                                  float *out_second_err)
{
    uint32_t ordered[9];
    for (int i = 0; i < 9; i++) {
        ordered[i] = reversed ? raw_win[8 - i] : raw_win[i];
    }

    *out_ch = '\0';
    *out_best_err = 1e9f;
    *out_second_err = 1e9f;

    float norm[9];
    if (!bc_normalize_window(ordered, norm)) {
        return;
    }

    for (uint8_t i = 0u; i < 26u; i++) {
        float err = bc_score_against_mask(norm, CODE39_ALPHA[i].code9, wide_ratio);
        if (err < *out_best_err) {
            *out_second_err = *out_best_err;
            *out_best_err = err;
            *out_ch = CODE39_ALPHA[i].letter;
        } else if (err < *out_second_err) {
            *out_second_err = err;
        }
    }
}

static float bc_score_star_window(const uint32_t raw_win[9], bool reversed, float wide_ratio)
{
    uint32_t ordered[9];
    for (int i = 0; i < 9; i++) {
        ordered[i] = reversed ? raw_win[8 - i] : raw_win[i];
    }

    float norm[9];
    if (!bc_normalize_window(ordered, norm)) {
        return 1e9f;
    }

    return bc_score_against_mask(norm, CODE39_ASTERISK_CODE, wide_ratio);
}

// Soft MSE Code 39 decoder across sliding 9-element windows (forward + reverse, with * framing)
static char bc_decode_code39(const uint32_t *durs, uint8_t count)
{
    last_diag_ch = '?';
    last_diag_err = -1.0f;
    last_diag_margin = -1.0f;
    last_matched_start = -1;
    last_matched_reversed = false;
    last_matched_after_star = false;

    if (count < 9u) {
        return '\0';
    }

    const float wide_ratio = 2.2f;
    const float max_err    = 0.09f;
    const float min_margin = 0.005f;

    char best_overall_ch = '\0';
    float best_overall_err = 1e9f;
    bool best_is_framed = false;

    for (uint8_t start_idx = 0u; (uint8_t)(start_idx + 9u) <= count; start_idx++) {
        const uint32_t *raw_win = &durs[start_idx];

        char fwd_ch = '\0';
        float fwd_err = 1e9f, fwd_second = 1e9f;
        bc_eval_candidate_order(raw_win, false, wide_ratio, &fwd_ch, &fwd_err, &fwd_second);

        char rev_ch = '\0';
        float rev_err = 1e9f, rev_second = 1e9f;
        bc_eval_candidate_order(raw_win, true, wide_ratio, &rev_ch, &rev_err, &rev_second);

        bool use_rev = (rev_err < fwd_err);
        char cand_ch = use_rev ? rev_ch : fwd_ch;
        float cand_err = use_rev ? rev_err : fwd_err;
        float cand_second = use_rev ? rev_second : fwd_second;

        if (cand_err < 1e8f && (best_overall_err > cand_err || last_diag_err < 0.0f)) {
            last_diag_ch = cand_ch;
            last_diag_err = cand_err;
            last_diag_margin = (cand_second < 1e8f) ? (cand_second - cand_err) : 0.0f;
        }

        if (cand_ch != '\0' &&
            cand_err <= max_err &&
            (cand_second - cand_err) >= min_margin) {
            bool has_leading_star = false;
            if ((uint8_t)(start_idx + 18u) <= count) {
                float star_err = bc_score_star_window(&durs[start_idx], use_rev, wide_ratio);
                if (star_err <= (max_err + 0.03f)) {
                    has_leading_star = true;
                }
            }

            bool has_preceding_star = false;
            if (start_idx >= 9u) {
                float prev_star_err = bc_score_star_window(&durs[start_idx - 9u], use_rev, wide_ratio);
                if (prev_star_err <= (max_err + 0.03f)) {
                    has_preceding_star = true;
                }
            }

            /* Prefer a character window adjacent to a '*' start/stop symbol over unframed slices */
            bool is_framed = (has_preceding_star || (has_leading_star && ((uint8_t)(start_idx + 18u) <= count)));
            bool take_candidate = false;

            if (has_preceding_star && ((uint8_t)(start_idx + 9u) <= count)) {
                if (!best_is_framed || cand_err < best_overall_err) {
                    take_candidate = true;
                }
            } else if (!best_is_framed && cand_err < best_overall_err) {
                take_candidate = true;
            }

            if (take_candidate) {
                best_overall_err = cand_err;
                best_overall_ch = cand_ch;
                best_is_framed = is_framed;
                last_matched_start = (int)start_idx;
                last_matched_reversed = use_rev;
                last_matched_after_star = has_preceding_star;
                for (int k = 0; k < 9; k++) {
                    last_matched_win[k] = raw_win[k];
                }
            }
        }
    }

    return best_overall_ch;
}

static void bc_print_only_barcode(char ch, const uint32_t *durs, uint8_t count)
{
    if (ch >= 'A' && ch <= 'Z') {
        tm_printf("Decoded Character: %c (dir=%s star=%d start_idx=%d win=[",
                  ch,
                  last_matched_reversed ? "REV" : "FWD",
                  last_matched_after_star ? 1 : 0,
                  last_matched_start);
        for (int k = 0; k < 9; k++) {
            tm_printf("%u%s", (unsigned int)last_matched_win[k], (k < 8) ? "," : "");
        }
        tm_printf("])\n");
    } else if (last_diag_err >= 0.0f) {
        int err_milli = (int)(last_diag_err * 1000.0f);
        int margin_milli = (int)(last_diag_margin * 1000.0f);
        tm_printf("Decoded Character: Unknown / Unmatched (closest=%c err=%d.%03d margin=%d.%03d)\n",
                  last_diag_ch,
                  err_milli / 1000, err_milli % 1000,
                  margin_milli / 1000, margin_milli % 1000);
    } else {
        tm_printf("Decoded Character: Unknown / Unmatched (no valid 9-stripe window <=250ms)\n");
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

                    bypass_state = BYPASS_BACK_UP;
                    step_start_us = now_us;
                }
                tk_dly_tsk(25);
                continue;
            }

            // Step 2: Back up slightly to clear the object safely
            if (bypass_state == BYPASS_BACK_UP) {
                (void)motion_drive(-60, 0);
                if (((now_us - step_start_us) / 1000u) >= 500u) {
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

            // Step 5: Turn Left more sharply back toward the track line
            if (bypass_state == BYPASS_TURN_BACK) {
                (void)motion_drive(BYPASS_SPEED_MM_S, -75);
                if (((now_us - step_start_us) / 1000u) >= 850u) {
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

                (void)motion_drive(BYPASS_SPEED_MM_S, -35);
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

        int32_t base_speed = 80;
        int32_t steer_term = 0;
        uint32_t now_ms = now_us / 1000u;

        if (left && center && ((now_ms - last_junc_ms) >= 600u || last_junc_ms == 0u)) {
            last_junc_ms = now_ms;
            char bc = norm_upper_alpha(stored_barcode_char);

            if (bc != '\0' && bc == bc_char_left) {
                int32_t turn_seq = motion_turn_left(90, MOTION_DEFAULT_SPEED);
                if (turn_seq > 0) { (void)motion_wait_done(turn_seq, 3000); }
            } else if (bc != '\0' && bc == bc_char_forward) {
                int32_t fwd_seq = motion_move_forward(90, base_speed);
                if (fwd_seq > 0) { (void)motion_wait_done(fwd_seq, 2000); }
            } else if (bc != '\0' && bc == bc_char_right) {
                int32_t turn_seq = motion_turn_right(90, MOTION_DEFAULT_SPEED);
                if (turn_seq > 0) { (void)motion_wait_done(turn_seq, 3000); }
            } else if (bc != '\0' && bc == bc_char_uturn) {
                int32_t turn_seq = motion_turn_left(BC_UTURN_ANGLE_DEG, MOTION_DEFAULT_SPEED);
                if (turn_seq > 0) { (void)motion_wait_done(turn_seq, 4000); }
            } else {
                int32_t fwd_seq = motion_move_forward(90, base_speed);
                if (fwd_seq > 0) { (void)motion_wait_done(fwd_seq, 2000); }
            }

            stored_barcode_char = '\0';
        } else {
            if (left && !center) {
                steer_term = -30;
            } else if (!left && center) {
                steer_term = 30;
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