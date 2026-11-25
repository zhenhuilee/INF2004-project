/*
 * task_telemetry.c
 * Buddy 1 -- collects subsystem telemetry and queues it for MQTT.
 */

#include "subsystem1.h"
#include "buddy1_comms.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem2/motion.h"

#define S1_TELEMETRY_PERIOD_MS 500

static void put_i32_le(uint8_t *dst, int32_t value)
{
    uint32_t v = (uint32_t)value;

    dst[0] = (uint8_t)(v);
    dst[1] = (uint8_t)(v >> 8);
    dst[2] = (uint8_t)(v >> 16);
    dst[3] = (uint8_t)(v >> 24);
}
static void send_buddy2_telemetry(void)
{
    motion_status_t st;

    /*
     * MOTOR packet:
     * byte 0      = BUDDY1_TLM_MOTOR (added by buddy1_send_packet)
     * payload[0]  = motion mode
     * payload[1..4]  = left measured speed, mm/s, signed int32 LE
     * payload[5..8]  = right measured speed, mm/s, signed int32 LE
     * payload[9..12] = left encoder count, signed int32 LE
     * payload[13..16]= right encoder count, signed int32 LE
     */
    uint8_t motor_payload[17];

    /*
     * DISTANCE packet:
     * byte 0      = BUDDY1_TLM_DISTANCE
     * payload[0..3] = odometer distance in mm, signed int32 LE
     */
    uint8_t distance_payload[4];

    motion_get_status(&st);

    motor_payload[0] = (uint8_t)st.mode;
    put_i32_le(&motor_payload[1], st.speed_mm_s[0]);
    put_i32_le(&motor_payload[5], st.speed_mm_s[1]);
    put_i32_le(&motor_payload[9], st.enc_count[0]);
    put_i32_le(&motor_payload[13], st.enc_count[1]);

    (void)buddy1_send_packet(
        BUDDY1_TLM_MOTOR,
        motor_payload,
        sizeof(motor_payload)
    );

    put_i32_le(distance_payload, st.odometer_mm);

    (void)buddy1_send_packet(
        BUDDY1_TLM_DISTANCE,
        distance_payload,
        sizeof(distance_payload)
    );
}

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

        send_buddy2_telemetry();

        tk_dly_tsk(S1_TELEMETRY_PERIOD_MS);
    }
}
