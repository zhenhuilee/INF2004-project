#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

#define S3_PRI_CONSUMER     16  
#define S3_BATCH_SIZE       5   

/* IR Sensors and LED pin mapping */
#define S3_IR_PIN_LEFT    6   /* Left tracking sensor */
#define S3_IR_PIN_CENTER  16   /* Center tracking sensor */
#define S3_IR_PIN_RIGHT   4   /* Right barcode sensor */
#define S3_LED_PIN_1      1   /* Status LED */

/* 4 Possible Decoded Instructions (Encoded in 2 bits: 0 to 3) */
#define S3_INST_NONE      0x00 /* 00: no barcode detected */
#define S3_INST_LEFT      0x01 /* 01: go left at crossroads */
#define S3_INST_RIGHT  0x02 /* 10: go right at crossroadse */
#define S3_INST_STRAIGHT      0x03 /* 11: go straight at crossroads */

/* IF-01: send SAMPLE_MSG here with tk_snd_mbx */
IMPORT ID   mbx_samples;

/* IF-04: Memory-optimized statistics using bitfields */
typedef struct {
    UW  received;   
    INT min;
    INT max;
    W   sum;
    UB  barcode_history;    /* Packed bits of recent Right-IR readings */
    UB  last_instruction:2; /* Stores one of the 4 instructions using 2 bits */
} APP_STATS;

IMPORT ID       mtx_stats;
IMPORT APP_STATS    app_stats;

/* Private to subsystem 3 */
IMPORT void consumer_task(INT stacd, void *exinf);

#endif 