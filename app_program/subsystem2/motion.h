/*
 *	motion.h
 *	Subsystem 2 -- Motion control.  PUBLIC API for the whole team.
 *
 *	Any task, in any subsystem, on either core, may call these functions.
 *	They only send a command to the motion task (a message buffer); the
 *	motion task is the only code that touches the motors and encoders.
 *
 *	Units: distance in mm, speed in mm/s, angle in degrees.
 *
 *	Example (Buddy 3, barcode "turn left" at a junction):
 *
 *	    int32_t seq = motion_turn_left(90, MOTION_DEFAULT_SPEED);
 *	    (void)motion_wait_done(seq, 3000);    // block up to 3 s
 *
 *	Example (Buddy 3, line following, called every 10-20 ms):
 *
 *	    (void)motion_drive(250, correction_mm_s);   // + = steer left
 */

#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

#define MOTION_DEFAULT_SPEED	(200)	/* mm/s */
#define MOTION_WAIT_FOREVER	(-1)	/* timeout for motion_wait_done() */

typedef enum
{
	MOTION_IDLE = 0,	/* stopped, motors braked */
	MOTION_MOVING,		/* moveForward / moveBackward in progress */
	MOTION_TURNING,		/* turnLeft / turnRight in progress */
	MOTION_DRIVING		/* continuous speed control (line following) */
} motion_mode_t;

/* Snapshot for telemetry (Buddy 1) and debugging.  Index 0 = left
   wheel, 1 = right wheel. */
typedef struct
{
	motion_mode_t	mode;
	int32_t		target_mm_s[2];	/* speed set-point */
	int32_t		speed_mm_s[2];	/* measured speed */
	int32_t		duty_permille[2];
	int32_t		enc_count[2];	/* raw encoder counts since boot */
	int32_t		odometer_mm;	/* total distance travelled */
	uint32_t	enc_errors;	/* missed-edge counter, should stay 0 */
	uint32_t	done_seq;	/* last finished command sequence no. */
} motion_status_t;

/* Motion commands.  Each returns a sequence number (> 0) that can be
   passed to motion_wait_done(), or a negative micro T-Kernel error code
   (e.g. E_TMOUT if the command queue is full).  A new command replaces
   the one in progress. */
int32_t	motion_move_forward(int32_t distance_mm, int32_t speed_mm_s);
int32_t	motion_move_backward(int32_t distance_mm, int32_t speed_mm_s);
int32_t	motion_turn_left(int32_t angle_deg, int32_t speed_mm_s);
int32_t	motion_turn_right(int32_t angle_deg, int32_t speed_mm_s);
int32_t	motion_stop(void);

/* Continuous mode for line following: base speed plus a steering term.
   left wheel = speed - steer, right wheel = speed + steer (+ = left).
   Must be called again at least every MOTION_DRIVE_TIMEOUT_MS, or the
   car stops by itself (protects against a crashed caller). */
int32_t	motion_drive(int32_t speed_mm_s, int32_t steer_mm_s);

/* Speed-PID gains, x1000 (e.g. kp_milli = 1500 means Kp = 1.5).  For
   tuning without rebuilding; defaults are in motion_config.h.  Takes
   effect at once; do not pass its result to motion_wait_done(). */
int32_t	motion_set_pid(int32_t kp_milli, int32_t ki_milli, int32_t kd_milli);

/* Block until command 'seq' has finished (or was replaced by a newer
   command).  timeout_ms = MOTION_WAIT_FOREVER waits forever.
   Returns E_OK, E_TMOUT, or 'seq' itself if it was an error code. */
int32_t	motion_wait_done(int32_t seq, int32_t timeout_ms);

void	motion_get_status(motion_status_t *p_status);

/* Called by subsystem2.c only */
int32_t	motion_init(void);
int32_t	motion_start(void);

#endif /* MOTION_H */
