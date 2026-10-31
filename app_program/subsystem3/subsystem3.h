#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

#define S3_PRI_CONSUMER     16  
#define S3_BATCH_SIZE       5   

/* IR Sensors and LED indicators */
#define S3_IR_PIN_2         2   /* Input for sensor 1 */
#define S3_IR_PIN_3         3   /* Input for sensor 2 */
#define S3_LED_PIN_1        1   /* Output LED controlled by inputs */

/* IF-01: send SAMPLE_MSG here with tk_snd_mbx */
IMPORT ID   mbx_samples;

/* IF-04: statistics */
typedef struct {
    UW  received;   
    INT min;
    INT max;
    W   sum;
} APP_STATS;

IMPORT ID       mtx_stats;
IMPORT APP_STATS    app_stats;

/* Private to subsystem 3 */
IMPORT void consumer_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM3_H */