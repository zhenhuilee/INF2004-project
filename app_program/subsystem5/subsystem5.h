/*
 *	subsystem5.h
 *	Subsystem 5 -- <name>.  Student ID: <student ID>.
 *
 *	EMPTY TEMPLATE.  Subsystems 1-4 show a complete example; copy their
 *	pattern.  Put here only what OTHER subsystems may use (object IDs,
 *	message formats, priorities); keep everything else LOCAL in the .c.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"

#define S5_PRI_MAIN	20		/* priority band 20-21 (TEAM.md) */

/*Public interface for other subsystems */
IMPORT BOOL subsystem5_is_blocked(void);

/* Approximate obstacle width, in mm */
IMPORT INT subsystem5_get_obstacle_width(void);

#endif /* SUBSYSTEM5_H */
