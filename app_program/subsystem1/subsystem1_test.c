/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make TEST=1
 *
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem1/logs/ and record the result
 *	in testing/subsystem1/TEST_LOG.md.  Test IDs: S1-U01, S1-U02, ...
 */
#include "buddy1_comms.h"
#include "subsystem1.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem1_test(void)
{
	T_RCYC	rcyc;
	T_RTSK	rtsk;
	ER	er;

	TEST_INIT();

	/* S1-U01: the cyclic handler exists and is running */
	er = tk_ref_cyc(cyc_led, &rcyc);
	TEST_CHECK("S1-U01", er == E_OK && (rcyc.cycstat & TCYC_STA),
		   "cyclic handler is running");

	/* S1-U02: between ticks the LED task is asleep, not busy-looping.
	   Sample it 10 times over ~1 s; it should almost always be waiting. */
	{
		INT	i, waiting = 0;
		for (i = 0; i < 10; i++) {
			tk_dly_tsk(97);	/* not a multiple of the tick */
			if (tk_ref_tsk(tid_led, &rtsk) == E_OK &&
			    (rtsk.tskstat & TTS_WAI)) {
				waiting++;
			}
		}
		TEST_CHECK("S1-U02", waiting >= 9, "LED task sleeps between ticks");
	}
	TEST_CHECK("S1-U03",
           buddy1_parse_command("START") == BUDDY1_CMD_START,
           "START command parsed");

	TEST_CHECK("S1-U04",
			buddy1_parse_command("STOP") == BUDDY1_CMD_STOP,
			"STOP command parsed");

	TEST_CHECK("S1-U05",
			buddy1_parse_command("LEFT") == BUDDY1_CMD_LEFT,
			"LEFT command parsed");

	TEST_CHECK("S1-U06",
			buddy1_parse_command("INVALID") == BUDDY1_CMD_INVALID,
			"Invalid command rejected");

	/* S1-U07: Buddy 1 command buffer exists */
	{
		T_RMBF rmbf;
		ER er_buf;

		er_buf = tk_ref_mbf(mbf_buddy1_cmd, &rmbf);

		TEST_CHECK("S1-U07",
				er_buf == E_OK,
				"Buddy 1 command buffer created");
	}

	/* S1-U08: Verify command can be sent and received */
	{
		UB sent_cmd = (UB)BUDDY1_CMD_START;
		UB received_cmd = 0;
		INT received_len;
		int send_result;

		send_result = buddy1_queue_command(BUDDY1_CMD_START);

		received_len = tk_rcv_mbf(
			mbf_buddy1_cmd,
			&received_cmd,
			TMO_POL
		);

		TEST_CHECK("S1-U08",
				send_result == 0 &&
				received_len == 1 &&
				received_cmd == sent_cmd,
				"Buddy 1 command buffer send/receive");
	}
	/* S1-U09: Verify Buddy 1 public receive API */
	{
		buddy1_command_t received_cmd = BUDDY1_CMD_NONE;
		int send_result;
		int receive_result;

		send_result = buddy1_queue_command(BUDDY1_CMD_STOP);

		receive_result = buddy1_receive_command(&received_cmd);

		TEST_CHECK("S1-U09",
				send_result == 0 &&
				receive_result == 0 &&
				received_cmd == BUDDY1_CMD_STOP,
				"Buddy 1 public receive API");
	}

	TEST_SUMMARY("subsystem1");

}

#endif /* APP_TEST */
