#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include <stdint.h>
#include <stdbool.h>

#ifndef S5_PRI_MAIN
#define S5_PRI_MAIN 14
#endif

typedef struct {
    bool  valid;            /* True once a sweep measurement is ready */
    float distance_cm;      /* Average distance to obstacle (cm) */
    float span_deg;         /* Angular width seen across sweep (deg) */
    float est_width_cm;     /* Estimated object width (cm) */
    float bypass_offset_cm; /* Estimated width + side clearance margin (cm) */
} S5_OBSTACLE_INFO;

bool subsystem5_is_blocked(void);
bool subsystem5_get_obstacle_info(S5_OBSTACLE_INFO *out_info);
void subsystem5_clear_obstacle_info(void);

EXPORT ER subsystem5_init(void);
EXPORT ER subsystem5_start(void);

#endif /* SUBSYSTEM5_H */