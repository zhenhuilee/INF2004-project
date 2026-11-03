/*
 *	subsystem4.h
 *	Subsystem 4 -- Monitor (reporting).  Student ID: <student ID>.
 *
 *	Public interface of subsystem 4.  Other subsystems include it as
 *	"../subsystem4/subsystem4.h".
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

#define S4_PRI_MONITOR		18	/* priority band 18-19 (TEAM.md) */
#define S4_PRI_IMU		19
#define S4_ALARM_MS		2000	/* report anyway if nothing for 2 s */
#define S4_LED_TOGGLE_REPORTS	6	/* pause/resume the LED every N reports */

/* IF-03: send one text line (at most S4_SUMMARY_MAX bytes, no '\0'
   needed) to mbf_summary, then set FLG_BATCH in flg_events. */
#define S4_SUMMARY_MAX		48
#define FLG_BATCH		0x0002	/* set by subsystem 3 */
#define FLG_ALARM		0x0001	/* private: set by our alarm handler */

typedef enum
{
	S4_TERRAIN_FLAT = 0,
	S4_TERRAIN_UPHILL,
	S4_TERRAIN_PEAK,
	S4_TERRAIN_DOWNHILL
} S4_TERRAIN_STATE;

#define S4_TERRAIN_BITS		0x03
#define S4_IMU_FAULT_BIT	0x04

typedef UB S4_IMU_STATUS;

IMPORT ID	mbf_summary;
IMPORT ID	flg_events;

/* Private to subsystem 4 */
IMPORT ID	alm_monitor;
IMPORT UW	s4_lines_received;	/* for the tests */
IMPORT void	monitor_task(INT stacd, void *exinf);
IMPORT void	imu_task(INT stacd, void *exinf);
IMPORT void	monitor_alarm_handler(void *exinf);

IMPORT S4_TERRAIN_STATE s4_classify_terrain(INT y_value, INT flat_y);

#endif /* SUBSYSTEM4_H */