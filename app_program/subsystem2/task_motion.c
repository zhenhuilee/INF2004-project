/*
 *	task_motion.c
 *	Subsystem 2 -- the motion task: speed estimation, wheel PID,
 *	straight-line correction, distance moves, pivot turns, and the
 *	public motion API.
 *
 *	Data flow every S2_CTRL_MS (20 ms):
 *	  encoders (polled 1 ms) -> signed ticks -> wheel speed (mm/s)
 *	  command -> wheel speed setpoints (+ sync correction, + end ramp)
 *	  setpoint vs measured -> PID -> duty -> motor_set()
 *
 *	APIs shown: tk_slp_tsk, tk_wup_tsk, tk_loc_mtx, tk_unl_mtx,
 *	            tk_set_flg, tk_clr_flg, tk_wai_flg
 *
 *	Integer maths only.
 */

#include "subsystem2.h"

/* ------------------------------------------------------------------ */
/*  Pure helpers                                                       */
/* ------------------------------------------------------------------ */

EXPORT INT s2_clamp(INT v, INT lo, INT hi)
{
	return (v < lo) ? lo : (v > hi) ? hi : v;
}

/* mm of wheel travel -> encoder ticks (rounded) */
EXPORT UW s2_mm_to_ticks(UW mm)
{
	/* ticks = mm * ticks_per_rev / circumference, with the calibration
	   percentage; arranged so nothing overflows 32 bits up to 20 m */
	UW num = mm * S2_TICKS_PER_REV * S2_DIST_CAL * 10UL;
	return (num + S2_CIRC_UM / 2) / S2_CIRC_UM;
}

/* Pivot turn: each wheel travels pi*T*angle/360, so
   ticks = (T/D) * (angle/360) * ticks_per_rev   (pi cancels). */
EXPORT UW s2_turn_to_ticks(UW deg)
{
	UW num = deg * S2_TRACK_MM * S2_TICKS_PER_REV * S2_TURN_CAL;
	UW den = 360UL * S2_WHEEL_DIAM_MM * 100UL;
	return (num + den / 2) / den;
}

/* signed ticks -> mm (inverse of s2_mm_to_ticks) */
EXPORT INT s2_ticks_to_mm(INT ticks)
{
	INT mm = ((INT)S2_CIRC_UM * ticks) / (INT)(S2_TICKS_PER_REV * 1000);
	return mm * 100 / S2_DIST_CAL;
}

/* PID on wheel speed.  Returns a duty CORRECTION in permille.
   Integral is clamped (anti-windup); derivative acts on the measurement
   so a setpoint step does not kick the output. */
EXPORT INT s2_pid_step(S2_PID *p, INT sp, INT meas)
{
	INT err = sp - meas;
	INT ilim = (S2_KI > 0) ? (S2_I_LIMIT * 256) / S2_KI : 0;
	INT u;

	p->integ = s2_clamp(p->integ + err, -ilim, ilim);
	u = (S2_KP * err + S2_KI * p->integ - S2_KD * (meas - p->prev_meas)) / 256;
	p->prev_meas = meas;
	return u;
}

/* Feed-forward: the duty that roughly gives the wanted speed */
static INT feed_forward(INT sp)
{
	INT mag = (sp < 0) ? -sp : sp;
	INT duty;

	if (mag == 0) return 0;
	duty = S2_DUTY_MIN + mag * (S2_DUTY_MAX - S2_DUTY_MIN) / S2_SPEED_MAX;
	duty = s2_clamp(duty, 0, S2_DUTY_MAX);
	return (sp < 0) ? -duty : duty;
}

/* ------------------------------------------------------------------ */
/*  Command block (written by the API, read by motion_task)           */
/* ------------------------------------------------------------------ */

enum { M_IDLE, M_SPEED, M_DIST, M_TURN };

typedef struct {
	INT	mode;
	INT	sp_l, sp_r;		/* M_SPEED setpoints */
	INT	sl, sr;			/* M_DIST/M_TURN: wheel directions +/-1 */
	UW	target;			/* ticks per wheel */
	UW	ramp;			/* ticks over which to slow down */
	INT	speed;			/* cruise speed for this command */
	UW	seq;			/* bumped on every new command */
} CMD;

static CMD		cmd;		/* protected by mtx_motion */
static UW		cruise_mm_s = S2_SPEED_CRUISE;

/* Published status, written by motion_task, read lock-free.
   Each word is atomic; fields may be from adjacent ticks (fine). */
static volatile MOTION_STATUS	pub;

/* Install a new command: bump the sequence number so the task restarts */
static ER submit_new(CMD c)
{
	ER er = tk_loc_mtx(mtx_motion, TMO_FEVR);
	if (er < E_OK) return er;
	c.seq = cmd.seq + 1;
	cmd = c;
	tk_unl_mtx(mtx_motion);
	/* Not done any more: the task sets the flag again on completion */
	tk_clr_flg(flg_motion, ~(UINT)S2_FLG_DONE);
	return E_OK;
}

static ER distance_cmd(UW mm, INT dir)
{
	CMD c = {0};

	if (mm == 0 || mm > 20000) return E_PAR;
	c.mode   = M_DIST;
	c.sl = c.sr = dir;
	c.target = s2_mm_to_ticks(mm);
	c.ramp   = s2_mm_to_ticks(S2_RAMP_MM);
	c.speed  = (INT)cruise_mm_s;
	return submit_new(c);
}

static ER turn_cmd(UW deg, INT sl, INT sr)
{
	CMD c = {0};

	if (deg == 0 || deg > 3600) return E_PAR;
	c.mode   = M_TURN;
	c.sl     = sl;
	c.sr     = sr;
	c.target = s2_turn_to_ticks(deg);
	c.ramp   = s2_turn_to_ticks(25);	/* slow down over the last 25 deg */
	c.speed  = S2_TURN_SPEED;
	return submit_new(c);
}

/* ---------------- public API ---------------- */

EXPORT ER moveForward(UW distance_mm)	{ return distance_cmd(distance_mm, +1); }
EXPORT ER moveBackward(UW distance_mm)	{ return distance_cmd(distance_mm, -1); }
EXPORT ER turnLeft(UW angle_deg)	{ return turn_cmd(angle_deg, -1, +1); }
EXPORT ER turnRight(UW angle_deg)	{ return turn_cmd(angle_deg, +1, -1); }

EXPORT ER stop(void)
{
	CMD c = {0};
	ER er;

	c.mode = M_IDLE;
	er = submit_new(c);
	tk_set_flg(flg_motion, S2_FLG_DONE);	/* nothing left to wait for */
	return er;
}

EXPORT ER motion_set_speeds(INT left_mm_s, INT right_mm_s)
{
	CMD c = {0};
	ER er;

	c.mode = M_SPEED;
	c.sp_l = s2_clamp(left_mm_s,  -S2_SPEED_MAX, S2_SPEED_MAX);
	c.sp_r = s2_clamp(right_mm_s, -S2_SPEED_MAX, S2_SPEED_MAX);

	er = tk_loc_mtx(mtx_motion, TMO_FEVR);
	if (er < E_OK) return er;
	/* A line follower calls this at ~50 Hz.  Staying in speed mode only
	   changes the setpoints, so the PID state is kept; entering speed
	   mode from another mode counts as a new command. */
	if (cmd.mode != M_SPEED) {
		c.seq = cmd.seq + 1;
		tk_clr_flg(flg_motion, ~(UINT)S2_FLG_DONE);
	} else {
		c.seq = cmd.seq;
	}
	cmd = c;
	tk_unl_mtx(mtx_motion);
	return E_OK;
}

EXPORT void motion_set_cruise(UW mm_s)
{
	cruise_mm_s = (UW)s2_clamp((INT)mm_s, S2_SPEED_MIN, S2_SPEED_MAX);
}

EXPORT BOOL motion_is_done(void)
{
	UINT ptn = 0;
	tk_wai_flg(flg_motion, S2_FLG_DONE, TWF_ORW, &ptn, TMO_POL);
	return (ptn & S2_FLG_DONE) ? TRUE : FALSE;
}

EXPORT ER motion_wait_done(TMO tmout)
{
	UINT ptn;
	return tk_wai_flg(flg_motion, S2_FLG_DONE, TWF_ORW, &ptn, tmout);
}

EXPORT void motion_get_status(MOTION_STATUS *st)
{
	st->speed_l = pub.speed_l;	st->speed_r = pub.speed_r;
	st->duty_l  = pub.duty_l;	st->duty_r  = pub.duty_r;
	st->ticks_l = pub.ticks_l;	st->ticks_r = pub.ticks_r;
	st->distance_mm = pub.distance_mm;
	st->state   = pub.state;	st->error   = pub.error;
}

static volatile BOOL reset_req;
EXPORT void motion_reset_distance(void)
{
	reset_req = TRUE;	/* done by motion_task at its next tick */
}

/* ------------------------------------------------------------------ */
/*  1 ms cyclic handler: poll the encoders, pace the control task      */
/* ------------------------------------------------------------------ */

EXPORT void motion_cyclic_handler(void *exinf)
{
	static UW n;
	(void)exinf;

	enc_poll();
	n += S2_ENC_POLL_MS;
	if (n >= S2_CTRL_MS) {
		n = 0;
		tk_wup_tsk(tid_motion);
	}
}

/* ------------------------------------------------------------------ */
/*  The control task                                                   */
/* ------------------------------------------------------------------ */

typedef struct {
	UW	raw_prev;	/* last raw edge count */
	INT	dir;		/* sign of the last non-zero duty */
	INT	pos;		/* signed ticks since reset */
	INT	base;		/* pos at the start of the current command */
	INT	hist[S2_SPD_WIN];	/* pos history for the speed window */
	INT	duty;
	S2_PID	pid;
} WHEEL;

static WHEEL wheel[2];

/* Read the encoder, sign it, update position and measured speed (mm/s) */
static INT wheel_update(INT side, INT hist_idx)
{
	WHEEL *w = &wheel[side];
	UW raw = enc_raw(side);
	INT delta = (INT)(raw - w->raw_prev);
	INT oldest;

	w->raw_prev = raw;
	w->pos += (w->dir >= 0) ? delta : -delta;

	oldest = w->hist[hist_idx];		/* position S2_SPD_WIN ticks ago */
	w->hist[hist_idx] = w->pos;
	/* ticks/window -> mm/s: ticks * circ_um / (ticks_per_rev * window_ms) */
	return ((w->pos - oldest) * (INT)S2_CIRC_UM) /
	       (INT)(S2_TICKS_PER_REV * S2_SPD_WIN * S2_CTRL_MS);
}

static void wheel_drive(INT side, INT sp, INT meas)
{
	WHEEL *w = &wheel[side];
	INT duty;

	if (sp == 0) {
		w->pid.integ = 0;
		duty = 0;
	} else {
		duty = feed_forward(sp) + s2_pid_step(&w->pid, sp, meas);
		duty = s2_clamp(duty, -S2_DUTY_MAX, S2_DUTY_MAX);
		/* never push the wrong way for the wanted direction */
		if ((sp > 0 && duty < 0) || (sp < 0 && duty > 0)) duty = 0;
	}
	w->duty = duty;
	if (duty != 0) w->dir = (duty > 0) ? +1 : -1;
	motor_set(side, duty);
}

static void finish(INT err)
{
	motor_set(S2_LEFT, 0);
	motor_set(S2_RIGHT, 0);
	wheel[0].duty = wheel[1].duty = 0;
	wheel[0].pid.integ = wheel[1].pid.integ = 0;
	pub.error = err;
	tk_set_flg(flg_motion, S2_FLG_DONE);
}

EXPORT void motion_task(INT stacd, void *exinf)
{
	CMD	c = {0};
	UW	my_seq = 0;
	INT	mode = M_IDLE;
	INT	hist_idx = 0;
	INT	sp_l, sp_r, v_l, v_r;
	INT	stall_ms = 0, last_sum = 0;

	(void)stacd; (void)exinf;

	while (1) {
		tk_slp_tsk(TMO_FEVR);
		tk_can_wup(TSK_SELF);	/* drop ticks missed while busy */

		/* 1. Take a snapshot of the latest command */
		tk_loc_mtx(mtx_motion, TMO_FEVR);
		c = cmd;
		tk_unl_mtx(mtx_motion);

		/* 2. Measure */
		v_l = wheel_update(S2_LEFT,  hist_idx);
		v_r = wheel_update(S2_RIGHT, hist_idx);
		hist_idx = (hist_idx + 1) % S2_SPD_WIN;

		if (reset_req) {
			reset_req = FALSE;
			wheel[0].base -= wheel[0].pos;  wheel[0].pos = 0;
			wheel[1].base -= wheel[1].pos;  wheel[1].pos = 0;
			{ INT i; for (i = 0; i < S2_SPD_WIN; i++)
				wheel[0].hist[i] = wheel[1].hist[i] = 0; }
		}

		/* 3. New command? */
		if (c.seq != my_seq) {
			my_seq = c.seq;
			mode = c.mode;
			wheel[0].base = wheel[0].pos;
			wheel[1].base = wheel[1].pos;
			wheel[0].pid.integ = wheel[1].pid.integ = 0;
			stall_ms = 0;
			last_sum = wheel[0].pos + wheel[1].pos;
			pub.error = S2_ERR_NONE;
			if (mode == M_IDLE) finish(S2_ERR_NONE);
		}

		/* 4. Decide wheel speed setpoints */
		sp_l = sp_r = 0;
		if (mode == M_SPEED) {
			sp_l = c.sp_l;
			sp_r = c.sp_r;
		} else if (mode == M_DIST || mode == M_TURN) {
			INT ds_l = c.sl * (wheel[0].pos - wheel[0].base);
			INT ds_r = c.sr * (wheel[1].pos - wheel[1].base);
			INT progress = (ds_l + ds_r) / 2;
			INT rem = (INT)c.target - progress;

			if (rem <= 0) {			/* arrived */
				mode = M_IDLE;
				finish(S2_ERR_NONE);
			} else {
				/* ramp the speed down near the end */
				INT v = c.speed;
				INT corr;
				if (rem < (INT)c.ramp)
					v = s2_clamp(c.speed * rem / (INT)c.ramp,
						     S2_SPEED_MIN, c.speed);
				/* keep the two wheels' progress equal */
				corr = s2_clamp(S2_KSYNC * (ds_l - ds_r),
						-S2_SYNC_MAX, S2_SYNC_MAX);
				sp_l = c.sl * s2_clamp(v - corr, 0, S2_SPEED_MAX);
				sp_r = c.sr * s2_clamp(v + corr, 0, S2_SPEED_MAX);

				/* stall watchdog */
				if ((wheel[0].pos + wheel[1].pos) != last_sum) {
					last_sum = wheel[0].pos + wheel[1].pos;
					stall_ms = 0;
				} else if ((stall_ms += S2_CTRL_MS) >= S2_STALL_MS) {
					mode = M_IDLE;
					finish(S2_ERR_STALL);
					sp_l = sp_r = 0;
				}
			}
		}

		/* 5. Drive.  While idle the motors stay at zero. */
		if (mode == M_IDLE) {
			sp_l = sp_r = 0;
			if (wheel[0].duty != 0 || wheel[1].duty != 0) {
				motor_set(S2_LEFT, 0);
				motor_set(S2_RIGHT, 0);
				wheel[0].duty = wheel[1].duty = 0;
			}
		} else {
			wheel_drive(S2_LEFT,  sp_l, v_l);
			wheel_drive(S2_RIGHT, sp_r, v_r);
		}

		/* 6. Publish status */
		pub.speed_l    = v_l;
		pub.speed_r    = v_r;
		pub.duty_l     = wheel[0].duty;
		pub.duty_r     = wheel[1].duty;
		pub.ticks_l    = wheel[0].pos;
		pub.ticks_r    = wheel[1].pos;
		pub.distance_mm = s2_ticks_to_mm((wheel[0].pos + wheel[1].pos) / 2);
		pub.state      = (mode == M_IDLE) ? S2_ST_IDLE :
				 (mode == M_SPEED) ? S2_ST_SPEED :
				 (mode == M_DIST)  ? S2_ST_MOVING : S2_ST_TURNING;
	}
}
