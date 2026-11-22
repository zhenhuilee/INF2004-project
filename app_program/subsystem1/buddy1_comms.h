#ifndef BUDDY1_COMMS_H
#define BUDDY1_COMMS_H

#include <stdint.h>

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

#endif /* BUDDY1_COMMS_H */
/* Receive a queued command.
 * Returns 0 on success, -1 if no command is available.
 */
