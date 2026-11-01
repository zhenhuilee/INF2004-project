/*
 *	drv_encoder.c
 *	Subsystem 2 -- wheel encoder edge counter.
 *
 *	The encoder pins are SAMPLED every S2_ENC_POLL_MS from the cyclic
 *	handler (no GPIO interrupt), counting both edges.  At 1 ms sampling
 *	a 20-slot disc can be tracked up to ~25 rev/s, far above what this
 *	car reaches, and the handler costs two GPIO reads per millisecond.
 *	An edge closer than S2_ENC_DEBOUNCE_MS to the previous one is noise.
 *
 *	The counters are single 32-bit words written by one handler and read
 *	by one task, so a read is atomic on the Cortex-M0+.
 *	The encoder has one channel, so it cannot sense direction: the
 *	control task signs the count from the direction it commanded.
 */

#include "subsystem2.h"

static const UINT enc_pin[2] = { S2_PIN_L_ENC, S2_PIN_R_ENC };

static volatile UW	edges[2];	/* accepted edges since boot */
static UW		now_ms;		/* poll counter (only used here) */
static UW		last_ms[2];	/* time of last accepted edge */
static UINT		last_lvl[2];

EXPORT void enc_init(void)
{
	INT i;

	for (i = 0; i < 2; i++) {
		gpio_set_pin(enc_pin[i], GPIO_MODE_IN);
		last_lvl[i] = gpio_get_val(enc_pin[i]);
		last_ms[i]  = 0;
		edges[i]    = 0;
	}
}

/* Called from the cyclic handler (interrupt context): keep it short */
EXPORT void enc_poll(void)
{
	INT i;
	UINT lvl;

	now_ms += S2_ENC_POLL_MS;
	for (i = 0; i < 2; i++) {
		lvl = gpio_get_val(enc_pin[i]);
		if (lvl != last_lvl[i]) {
			last_lvl[i] = lvl;
			if ((now_ms - last_ms[i]) >= S2_ENC_DEBOUNCE_MS) {
				last_ms[i] = now_ms;
				edges[i]++;
			}
		}
	}
}

EXPORT UW enc_raw(INT side)
{
	return edges[side];
}
