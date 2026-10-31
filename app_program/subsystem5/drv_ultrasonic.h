#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include <stdbool.h>

void ultrasonic_init(void);
float ultrasonic_get_distance(void);

#endif
