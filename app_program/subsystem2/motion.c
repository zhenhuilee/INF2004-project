/*
 *	motion.c
 *	Subsystem 2 -- Motion control: the motion task and the public API.
 *
 *	Structure
 *	---------
 *	  any task --motion_xxx()--> mbf_motion_cmd --> motion_task (100 Hz)
 *	                                                  |  reads encoders
 *	                                                  |  speed profile
 *	                                                  |  straight/turn sync
 *	                                                  |  PID per wheel
 *	                                                  v
 *	                                                motors
 *	  any task <--motion_get_status()-- g_status (under mtx_motion_status)
 *
 *	The control loop is paced by a cyclic handler that wakes the task
 *	every MOTION_PERIOD_MS.  The elapsed time is measured with the
 *	RP2040 microsecond timer, so a late wake-up does not upset the
 *	speed estimate.
 *
 *	APIs used: tk_cre_mbf tk_snd_mbf tk_rcv_mbf, tk_cre_mtx tk_loc_mtx
 *	           tk_unl_mtx, tk_cre_cyc tk_sta_cyc, tk_slp_tsk tk_wup_tsk,
 *	           tk_dly_tsk tk_get_tim
 */

#include "subsystem2.h"
#include "motion.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "motion_config.h"


#define PI_F			(3.14159265f)
#define MM_PER_TICK		((PI_F * MOTION_WHEEL_DIAMETER_MM * MOTION_DIST_SCALE) \
				 / MOTION_TICKS_PER_REV)
#define US_PER_S		(1000000.0f)
#define GAIN_SCALE		(1000.0f)
#define WHEELS			(2U)
#define WHEEL_L			(0U)
#define WHEEL_R			(1U)

/* The RP2040 microsecond timer only counts once the watchdog "tick"
   generator is set to divide clk_ref down to 1 MHz.  The kernel does not
   do this; the boot ROM leaves it running after a UF2 flash, but after a
   reset or power-on it is not set, and the timer runs at the wrong rate.
   (datasheet 4.7.2 / 4.6.3) */
#define WATCHDOG_TICK_REG	(0x4005802CU)
#define WATCHDOG_TICK_ENABLE	(1U << 9)

/* Accept a measured loop period only within these limits; otherwise use
   the nominal period (protects the speed estimate from a bad clock) */
#define DT_MIN_S		(0.002f)
#define DT_MAX_S		(0.100f)

typedef enum
{
	CMD_STOP = 0,
	CMD_MOVE,		/* arg0 = signed distance mm, arg1 = speed */
	CMD_TURN,		/* arg0 = signed angle deg (+ = left), arg1 = speed */
	CMD_DRIVE,		/* arg0 = speed, arg1 = steer */
	CMD_SET_PID		/* arg0..2 = kp, ki, kd x1000 */
} cmd_type_t;

typedef struct
{
	uint32_t	type;
	uint32_t	seq;
	int32_t		arg[3];
} motion_cmd_t;

typedef struct
{
	float	integ;
	float	prev_meas;
} pid_state_t;

/* ---- kernel objects ----------------------------------------------- */
static ID	g_mbf_cmd;		/* commands in */
static ID	g_mtx_status;		/* protects g_status */
static ID	g_mtx_seq;		/* protects g_next_seq */
static ID	g_cyc_tick;		/* wakes the task */
static ID	g_tid_motion;

/* ---- shared with callers (under mutex) ------------------------------ */
static motion_status_t	g_status;
static uint32_t		g_next_seq;

/* ---- private to the motion task ------------------------------------ */
static motion_mode_t	g_mode;
static uint32_t		g_cur_seq;
static float		g_sign[WHEELS];		/* direction of each wheel in a move/turn */
static float		g_goal_mm;		/* per-wheel path length to cover */
static float		g_cruise;		/* requested speed */
static float		g_v_cmd;		/* ramped speed */
static float		g_dist[WHEELS];		/* mm since the move started */
static float		g_speed[WHEELS];	/* filtered mm/s */
static float		g_target[WHEELS];
static int32_t		g_duty[WHEELS];
static int32_t		g_last_count[WHEELS];
static float		g_odometer;
static uint32_t		g_last_us;
static uint32_t		g_drive_age_ms;
static pid_state_t	g_pid[WHEELS];
static float		g_kp = MOTION_KP;
static float		g_ki = MOTION_KI;
static float		g_kd = MOTION_KD;

/* ===================================================================== */
/*  Small helpers                                                        */
/* ===================================================================== */

static float
abs_f (float x)
{
	return (x < 0.0f) ? -x : x;
}

static float
clamp_f (float x, float lo, float hi)
{
	if (x < lo)
	{
		return lo;
	}
	if (x > hi)
	{
		return hi;
	}
	return x;
}

static float
speed_limit (int32_t speed_mm_s)
{
	return clamp_f(abs_f((float)speed_mm_s), MOTION_MIN_SPEED, MOTION_MAX_SPEED);
}

static int32_t
round_i (float x)
{
	return (x < 0.0f) ? (int32_t)(x - 0.5f) : (int32_t)(x + 0.5f);
}

static void
pid_reset (pid_state_t *p_pid)
{
	p_pid->integ = 0.0f;
	p_pid->prev_meas = 0.0f;
}

/*
 * One wheel's speed controller.  Output = feed-forward (what the motor
 * roughly needs for this speed) + PID on the error.  The D term uses the
 * measurement, not the error, so a step in set-point gives no kick.
 */
static int32_t
pid_step (pid_state_t *p_pid, float target, float meas, float dt_s)
{
	float	err;
	float	ff;
	float	deriv;
	float	out;

	if (target == 0.0f)
	{
		pid_reset(p_pid);
		p_pid->prev_meas = meas;
		return 0;
	}

	err = target - meas;
	p_pid->integ = clamp_f(p_pid->integ + (g_ki * err * dt_s),
			       -MOTION_I_LIMIT, MOTION_I_LIMIT);
	deriv = -(meas - p_pid->prev_meas) / dt_s;
	p_pid->prev_meas = meas;

	ff = (MOTION_KF * target) + ((target > 0.0f) ? MOTION_DEADBAND : -MOTION_DEADBAND);
	out = ff + (g_kp * err) + p_pid->integ + (g_kd * deriv);

	/* Never drive a wheel against the direction it is meant to turn:
	   to slow down, the duty only drops to 0 (the wheel coasts).  Driving
	   it backwards would also make a B-only encoder count the wrong way
	   while the wheel is still rolling forwards (see enc_set_direction). */
	if (target > 0.0f)
	{
		out = clamp_f(out, 0.0f, (float)MOTOR_DUTY_MAX);
	}
	else
	{
		out = clamp_f(out, -(float)MOTOR_DUTY_MAX, 0.0f);
	}
	return round_i(out);
}

/* ===================================================================== */
/*  Motion task                                                          */
/* ===================================================================== */

static void
motion_begin (motion_mode_t mode, float sign_l, float sign_r,
	      float goal_mm, int32_t speed_mm_s)
{
	uint32_t	w;

	g_mode = mode;
	g_sign[WHEEL_L] = sign_l;
	g_sign[WHEEL_R] = sign_r;
	g_goal_mm = goal_mm;
	g_cruise = speed_limit(speed_mm_s);
	g_v_cmd = 0.0f;
	for (w = 0U; w < WHEELS; w++)
	{
		g_dist[w] = 0.0f;
	}
}

static void
motion_finish (void)
{
	uint32_t	w;

	for (w = 0U; w < WHEELS; w++)
	{
		g_target[w] = 0.0f;
		g_duty[w] = 0;
		pid_reset(&g_pid[w]);
		motor_brake((motor_id_t)w);
	}
	g_mode = MOTION_IDLE;
}

static void
motion_handle_cmd (const motion_cmd_t *p_cmd, uint32_t *p_done_seq)
{
	float	arc_mm;

	/* New gains apply at once and do not interrupt the current motion */
	if (p_cmd->type == (uint32_t)CMD_SET_PID)
	{
		g_kp = (float)p_cmd->arg[0] / GAIN_SCALE;
		g_ki = (float)p_cmd->arg[1] / GAIN_SCALE;
		g_kd = (float)p_cmd->arg[2] / GAIN_SCALE;
		return;
	}

	/* Accepting a motion command ends whatever ran before it */
	*p_done_seq = p_cmd->seq - 1U;
	g_cur_seq = p_cmd->seq;

	switch (p_cmd->type)
	{
		case CMD_MOVE:
			if (p_cmd->arg[0] >= 0)
			{
				motion_begin(MOTION_MOVING, 1.0f, 1.0f,
					     (float)p_cmd->arg[0], p_cmd->arg[1]);
			}
			else
			{
				motion_begin(MOTION_MOVING, -1.0f, -1.0f,
					     (float)(-p_cmd->arg[0]), p_cmd->arg[1]);
			}
		break;

		case CMD_TURN:
			/* In-place spin: each wheel travels angle/360 of a circle
			   whose diameter is the track width */
			arc_mm = abs_f((float)p_cmd->arg[0]) * PI_F * MOTION_TRACK_WIDTH_MM
				 * MOTION_TURN_SCALE / 360.0f;
			if (p_cmd->arg[0] >= 0)
			{
				motion_begin(MOTION_TURNING, -1.0f, 1.0f, arc_mm, p_cmd->arg[1]);
			}
			else
			{
				motion_begin(MOTION_TURNING, 1.0f, -1.0f, arc_mm, p_cmd->arg[1]);
			}
		break;

		case CMD_DRIVE:
			g_mode = MOTION_DRIVING;
			g_target[WHEEL_L] = clamp_f((float)(p_cmd->arg[0] - p_cmd->arg[1]),
						    -MOTION_MAX_SPEED, MOTION_MAX_SPEED);
			g_target[WHEEL_R] = clamp_f((float)(p_cmd->arg[0] + p_cmd->arg[1]),
						    -MOTION_MAX_SPEED, MOTION_MAX_SPEED);
			g_drive_age_ms = 0U;
			*p_done_seq = p_cmd->seq;	/* accepted = done */
		break;

		case CMD_STOP:
		default:
			motion_finish();
			*p_done_seq = p_cmd->seq;
		break;
	}
}

/* Speed profile + keep-the-wheels-in-step correction for moves/turns */
static void
motion_profile (float dt_s, uint32_t *p_done_seq)
{
	float	prog_l = g_dist[WHEEL_L] * g_sign[WHEEL_L];	/* + = right way */
	float	prog_r = g_dist[WHEEL_R] * g_sign[WHEEL_R];
	float	remaining = g_goal_mm - ((prog_l + prog_r) * 0.5f);
	float	v_max;
	float	corr;

	if (remaining <= MOTION_STOP_TOL_MM)
	{
		motion_finish();
		*p_done_seq = g_cur_seq;
		return;
	}

	/* Ramp up at MOTION_ACCEL, slow down in proportion to distance left */
	v_max = clamp_f(MOTION_DECEL_GAIN * remaining, MOTION_MIN_SPEED, g_cruise);
	g_v_cmd += MOTION_ACCEL * dt_s;
	if (g_v_cmd > v_max)
	{
		g_v_cmd = v_max;
	}

	/* If the left wheel is ahead, slow it and speed up the right one */
	corr = clamp_f(MOTION_KP_SYNC * (prog_l - prog_r),
		       -MOTION_SYNC_LIMIT, MOTION_SYNC_LIMIT);
	g_target[WHEEL_L] = g_sign[WHEEL_L] * (g_v_cmd - corr);
	g_target[WHEEL_R] = g_sign[WHEEL_R] * (g_v_cmd + corr);
}

static void
motion_publish (uint32_t done_seq)
{
	uint32_t	w;

	if (tk_loc_mtx(g_mtx_status, TMO_FEVR) < E_OK)
	{
		return;
	}
	g_status.mode = g_mode;
	for (w = 0U; w < WHEELS; w++)
	{
		g_status.target_mm_s[w] = round_i(g_target[w]);
		g_status.speed_mm_s[w] = round_i(g_speed[w]);
		g_status.duty_permille[w] = g_duty[w];
		g_status.enc_count[w] = g_last_count[w];
	}
	g_status.odometer_mm = round_i(g_odometer);
	g_status.enc_errors = enc_get_error_count();
	g_status.done_seq = done_seq;
	(void)tk_unl_mtx(g_mtx_status);
}

static void
motion_debug_print (void)
{
#if S2_DEBUG
	tm_printf((UB*)"[S2] mode=%d tgt=%d,%d spd=%d,%d duty=%d,%d cnt=%d,%d odo=%d err=%d\n",
		  (INT)g_mode,
		  round_i(g_target[WHEEL_L]), round_i(g_target[WHEEL_R]),
		  round_i(g_speed[WHEEL_L]), round_i(g_speed[WHEEL_R]),
		  g_duty[WHEEL_L], g_duty[WHEEL_R],
		  g_last_count[WHEEL_L], g_last_count[WHEEL_R],
		  round_i(g_odometer), (INT)enc_get_error_count());
#endif
}

static void
motion_cyclic_handler (void *exinf)
{
	(void)exinf;
	(void)tk_wup_tsk(g_tid_motion);
}

static void
motion_task (INT stacd, void *exinf)
{
	motion_cmd_t	cmd;
	uint32_t	done_seq = 0U;
	uint32_t	now_us;
	uint32_t	loops = 0U;
	uint32_t	w;
	int32_t		count;
	float		dt_s;
	float		d_mm;
	ER		er;

	(void)stacd;
	(void)exinf;

	/* This task owns the hardware, so it sets it up itself.  enc_init()
	   must run here: the encoder interrupt goes to this task's core. */
	out_w(WATCHDOG_TICK_REG, WATCHDOG_TICK_ENABLE | (UW)XOSC_MHz);	/* 1 us timer */
	motor_init();
	er = enc_init();
	CHECK(er, "enc_init");
	for (w = 0U; w < WHEELS; w++)
	{
		g_last_count[w] = enc_get_count((enc_id_t)w);
	}
	motion_finish();
	g_last_us = in_w(TIMER_TIMELR);
	CHECK(tk_sta_cyc(g_cyc_tick), "tk_sta_cyc");

	for (;;)
	{
		(void)tk_slp_tsk(TMO_FEVR);

#if S2_BRINGUP == 1
		/* Bring-up step 2/3: open loop, no PID.  Both wheels forward
		   at a fixed duty for 2 s, then braked for 2 s.  Watch which
		   way each wheel turns and whether its count goes up. */
		loops++;
		if ((loops % 400U) < 200U)
		{
			enc_set_direction(ENC_LEFT, S2_BRINGUP_DUTY);
			enc_set_direction(ENC_RIGHT, S2_BRINGUP_DUTY);
			motor_set_duty(MOTOR_LEFT, S2_BRINGUP_DUTY);
			motor_set_duty(MOTOR_RIGHT, S2_BRINGUP_DUTY);
		}
		else
		{
			motor_brake(MOTOR_LEFT);
			motor_brake(MOTOR_RIGHT);
		}
		if ((loops % MOTION_DEBUG_EVERY) == 0U)
		{
			tm_printf((UB*)"[S2] duty=%d  count L=%d R=%d  err=%d  enc_init=%d\n",
				  ((loops % 400U) < 200U) ? S2_BRINGUP_DUTY : 0,
				  enc_get_count(ENC_LEFT), enc_get_count(ENC_RIGHT),
				  (INT)enc_get_error_count(), (INT)er);
			enc_debug_print();
		}
		continue;
#endif

		/* 1. Take every waiting command (the newest one wins) */
		while (tk_rcv_mbf(g_mbf_cmd, &cmd, TMO_POL) == (INT)sizeof(cmd))
		{
			motion_handle_cmd(&cmd, &done_seq);
		}

		/* 2. Measure: encoder ticks -> distance and speed */
		now_us = in_w(TIMER_TIMELR);
		dt_s = (float)(now_us - g_last_us) / US_PER_S;	/* wraps safely */
		g_last_us = now_us;
		if ((dt_s < DT_MIN_S) || (dt_s > DT_MAX_S))
		{
			dt_s = (float)MOTION_PERIOD_MS / 1000.0f;
		}
		for (w = 0U; w < WHEELS; w++)
		{
			count = enc_get_count((enc_id_t)w);
			d_mm = (float)(count - g_last_count[w]) * MM_PER_TICK;
			g_last_count[w] = count;
			g_dist[w] += d_mm;
			g_odometer += abs_f(d_mm) * 0.5f;
			g_speed[w] += MOTION_SPEED_ALPHA * ((d_mm / dt_s) - g_speed[w]);
		}

		/* 3. Decide the wheel set-points */
		if ((g_mode == MOTION_MOVING) || (g_mode == MOTION_TURNING))
		{
			motion_profile(dt_s, &done_seq);
		}
		else if (g_mode == MOTION_DRIVING)
		{
			g_drive_age_ms += MOTION_PERIOD_MS;
			if (g_drive_age_ms > MOTION_DRIVE_TIMEOUT_MS)
			{
				motion_finish();	/* caller went quiet: stop */
			}
		}
		else
		{
			/* idle: motors stay braked */
		}

		/* 4. Close the speed loops */
		if (g_mode != MOTION_IDLE)
		{
			for (w = 0U; w < WHEELS; w++)
			{
				g_duty[w] = pid_step(&g_pid[w], g_target[w], g_speed[w], dt_s);
				/* Direction from the set-point, not the duty: the wheel
				   turns the way the move wants it to */
				enc_set_direction((enc_id_t)w, round_i(g_target[w]));
				motor_set_duty((motor_id_t)w, g_duty[w]);
			}
		}

		/* 5. Publish for telemetry */
		motion_publish(done_seq);
		loops++;
		if ((loops % MOTION_DEBUG_EVERY) == 0U)
		{
			motion_debug_print();
		}
	}
}

/* ===================================================================== */
/*  Setup (called from subsystem2.c)                                     */
/* ===================================================================== */

int32_t
motion_init (void)
{
	T_CMBF	cmbf = {0};
	T_CMTX	cmtx = {0};
	T_CCYC	ccyc = {0};

	/* Message buffer: each message is copied in, with a small header */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = MOTION_CMD_QUEUE * (sizeof(motion_cmd_t) + sizeof(INT) * 2U);
	cmbf.maxmsz = sizeof(motion_cmd_t);
	g_mbf_cmd = tk_cre_mbf(&cmbf);
	if (g_mbf_cmd < E_OK)
	{
		return g_mbf_cmd;
	}

	cmtx.mtxatr = TA_INHERIT;
	g_mtx_status = tk_cre_mtx(&cmtx);
	if (g_mtx_status < E_OK)
	{
		return g_mtx_status;
	}
	g_mtx_seq = tk_cre_mtx(&cmtx);
	if (g_mtx_seq < E_OK)
	{
		return g_mtx_seq;
	}

	/* Created stopped; the motion task starts it once the hardware is set up */
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)motion_cyclic_handler;
	ccyc.cyctim = MOTION_PERIOD_MS;
	ccyc.cycphs = MOTION_PERIOD_MS;
	g_cyc_tick = tk_cre_cyc(&ccyc);
	return (g_cyc_tick < E_OK) ? g_cyc_tick : E_OK;
}

int32_t
motion_start (void)
{
	/* Core 1: keeps the encoder interrupt away from the USB console
	   and WiFi service tasks */
	g_tid_motion = app_start_task((FP)motion_task, MOTION_PRI, TP_PRC2,
				      "motion_task");
	return (g_tid_motion < E_OK) ? g_tid_motion : E_OK;
}

/* ===================================================================== */
/*  Public API (any task)                                                */
/* ===================================================================== */

static int32_t
motion_send (cmd_type_t type, int32_t a0, int32_t a1, int32_t a2)
{
	motion_cmd_t	cmd;
	ER		er;

	cmd.type = (uint32_t)type;
	cmd.arg[0] = a0;
	cmd.arg[1] = a1;
	cmd.arg[2] = a2;

	/* Numbering and sending under one lock keeps the queue in order */
	er = tk_loc_mtx(g_mtx_seq, TMO_FEVR);
	if (er < E_OK)
	{
		return er;
	}
	g_next_seq++;
	cmd.seq = g_next_seq;
	er = tk_snd_mbf(g_mbf_cmd, &cmd, (INT)sizeof(cmd), MOTION_SEND_TMO_MS);
	(void)tk_unl_mtx(g_mtx_seq);

	return (er < E_OK) ? er : (int32_t)cmd.seq;
}

int32_t
motion_move_forward (int32_t distance_mm, int32_t speed_mm_s)
{
	return motion_send(CMD_MOVE, (distance_mm < 0) ? -distance_mm : distance_mm,
			   speed_mm_s, 0);
}

int32_t
motion_move_backward (int32_t distance_mm, int32_t speed_mm_s)
{
	return motion_send(CMD_MOVE, (distance_mm < 0) ? distance_mm : -distance_mm,
			   speed_mm_s, 0);
}

int32_t
motion_turn_left (int32_t angle_deg, int32_t speed_mm_s)
{
	return motion_send(CMD_TURN, (angle_deg < 0) ? -angle_deg : angle_deg,
			   speed_mm_s, 0);
}

int32_t
motion_turn_right (int32_t angle_deg, int32_t speed_mm_s)
{
	return motion_send(CMD_TURN, (angle_deg < 0) ? angle_deg : -angle_deg,
			   speed_mm_s, 0);
}

int32_t
motion_stop (void)
{
	return motion_send(CMD_STOP, 0, 0, 0);
}

int32_t
motion_drive (int32_t speed_mm_s, int32_t steer_mm_s)
{
	return motion_send(CMD_DRIVE, speed_mm_s, steer_mm_s, 0);
}

int32_t
motion_set_pid (int32_t kp_milli, int32_t ki_milli, int32_t kd_milli)
{
	return motion_send(CMD_SET_PID, kp_milli, ki_milli, kd_milli);
}

void
motion_get_status (motion_status_t *p_status)
{
	if (p_status == NULL)
	{
		return;
	}
	if (tk_loc_mtx(g_mtx_status, TMO_FEVR) == E_OK)
	{
		*p_status = g_status;
		(void)tk_unl_mtx(g_mtx_status);
	}
}

int32_t
motion_wait_done (int32_t seq, int32_t timeout_ms)
{
	motion_status_t	st;
	int32_t		waited_ms = 0;

	if (seq < 0)
	{
		return seq;		/* the command itself failed */
	}

	for (;;)
	{
		motion_get_status(&st);
		if (st.done_seq >= (uint32_t)seq)
		{
			return E_OK;
		}
		if ((timeout_ms >= 0) && (waited_ms >= timeout_ms))
		{
			return E_TMOUT;
		}
		(void)tk_dly_tsk(MOTION_PERIOD_MS);
		waited_ms += (int32_t)MOTION_PERIOD_MS;
	}
}