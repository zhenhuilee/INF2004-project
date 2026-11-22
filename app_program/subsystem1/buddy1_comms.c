#include "buddy1_comms.h"
#include "subsystem1.h"

/* Compare two null-terminated strings without libc */
static int buddy1_str_equal(const char *a, const char *b)
{
    if (a == NULL || b == NULL)
        return 0;

    while (*a != '\0' && *b != '\0') {
        if (*a != *b)
            return 0;

        a++;
        b++;
    }

    return (*a == '\0' && *b == '\0');
}

/* Convert MQTT text into a command */
buddy1_command_t buddy1_parse_command(const char *payload)
{
    if (payload == NULL)
        return BUDDY1_CMD_INVALID;

    if (buddy1_str_equal(payload, "START"))
        return BUDDY1_CMD_START;

    if (buddy1_str_equal(payload, "STOP"))
        return BUDDY1_CMD_STOP;

    if (buddy1_str_equal(payload, "LEFT"))
        return BUDDY1_CMD_LEFT;

    if (buddy1_str_equal(payload, "RIGHT"))
        return BUDDY1_CMD_RIGHT;

    if (buddy1_str_equal(payload, "FORWARD"))
        return BUDDY1_CMD_FORWARD;

    if (buddy1_str_equal(payload, "BACKWARD"))
        return BUDDY1_CMD_BACKWARD;

    return BUDDY1_CMD_INVALID;
}

/* Queue one parsed command for a future NavigationTask */
int buddy1_queue_command(buddy1_command_t cmd)
{
    UB command_byte;
    ER er;

    /* Reject NONE and INVALID commands */
    if (cmd <= BUDDY1_CMD_NONE || cmd >= BUDDY1_CMD_INVALID) {
        return -1;
    }

    /* Check that the message buffer exists */
    if (mbf_buddy1_cmd <= 0) {
        return -1;
    }

    /* Convert command enum to one byte */
    command_byte = (UB)cmd;

    /* Send command to the message buffer without waiting */
    er = tk_snd_mbf(mbf_buddy1_cmd,
                    &command_byte,
                    sizeof(command_byte),
                    TMO_POL);

    return (er == E_OK) ? 0 : -1;
}

int buddy1_receive_command(buddy1_command_t *cmd)
{
    UB command_byte;
    INT received_len;

    if (cmd == NULL) {
        return -1;
    }

    if (mbf_buddy1_cmd <= 0) {
        return -1;
    }

    received_len = tk_rcv_mbf(
        mbf_buddy1_cmd,
        &command_byte,
        TMO_POL
    );

    if (received_len != 1) {
        return -1;
    }

    if (command_byte <= BUDDY1_CMD_NONE ||
        command_byte >= BUDDY1_CMD_INVALID) {
        return -1;
    }

    *cmd = (buddy1_command_t)command_byte;

    return 0;
}