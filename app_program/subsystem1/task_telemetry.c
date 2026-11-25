/*
 * task_telemetry.c
 * Buddy 1 -- collects subsystem telemetry and queues it for MQTT.
 */

#include "subsystem1.h"
#include "buddy1_comms.h"
#include "../subsystem4/subsystem4.h"

#define S1_TELEMETRY_PERIOD_MS 500

EXPORT void buddy1_telemetry_task(INT stacd, void *exinf)
{
    UB s4_status[S4_COMPACT_STATUS_SIZE];
    uint8_t payload[3];
    INT highest_height;

    (void)stacd;
    (void)exinf;

    while (1) {
        /*
         * Buddy 4 already provides:
         *   status[0] = terrain/fault/accel/turn/lift
         *   status[1] = current hump height
         */
        s4_encode_compact_status(s4_status);

        highest_height = s4_get_highest_hump_height_mm();

        if (highest_height < 0)
            highest_height = 0;
        else if (highest_height > 255)
            highest_height = 255;

        payload[0] = (uint8_t)s4_status[0];
        payload[1] = (uint8_t)s4_status[1];
        payload[2] = (uint8_t)highest_height;

        (void)buddy1_send_packet(
            BUDDY1_TLM_HUMP,
            payload,
            sizeof(payload)
        );

        tk_dly_tsk(S1_TELEMETRY_PERIOD_MS);
    }
}
