#ifndef BUDDY1_COMMS_H
#define BUDDY1_COMMS_H
#define BUDDY1_TELEMETRY_MAX_SIZE 32
#include <stdint.h>


/* Telemetry message types */
typedef enum {
    BUDDY1_TLM_ROBOT_STATE = 0x01,
    BUDDY1_TLM_MOTOR       = 0x02,
    BUDDY1_TLM_LINE        = 0x03,
    BUDDY1_TLM_BARCODE     = 0x04,
    BUDDY1_TLM_HUMP        = 0x05,
    BUDDY1_TLM_OBSTACLE    = 0x06,
    BUDDY1_TLM_DISTANCE    = 0x07
} buddy1_telemetry_type_t;

/* Send compact binary telemetry:
 * byte 0 = telemetry type
 * bytes 1..N = payload
 */
int buddy1_send_packet(buddy1_telemetry_type_t type,
                       const uint8_t *payload,
                       uint8_t payload_len);


typedef enum {
    BUDDY1_CMD_NONE = 0,
    BUDDY1_CMD_START,
    BUDDY1_CMD_STOP,
    BUDDY1_CMD_LEFT,
    BUDDY1_CMD_RIGHT,
    BUDDY1_CMD_FORWARD,
    BUDDY1_CMD_BACKWARD,
    BUDDY1_CMD_INVALID
} buddy1_command_t;

/* Convert received MQTT text into a command */
buddy1_command_t buddy1_parse_command(const char *payload);

/* Put a valid command into the micro T-Kernel message buffer.
 * Returns 0 on success, -1 on failure.
 */
int buddy1_queue_command(buddy1_command_t cmd);

int buddy1_receive_command(buddy1_command_t *cmd);

/* Queue telemetry for Buddy 1 to publish over MQTT.
 * Payload may contain binary data.
 * Maximum payload size is S1_TX_MAX_SIZE (currently 32 bytes).
 * Returns 0 on success, -1 on failure/full buffer.
 */
int buddy1_send_telemetry(const uint8_t *data, uint16_t len);

/* Called by the WiFi/MQTT owner task to drain outgoing telemetry.
 * Non-blocking.
 * Returns message length (>0), or -1 if no message is available.
 */
int buddy1_receive_telemetry(uint8_t *data);


#endif /* BUDDY1_COMMS_H */
