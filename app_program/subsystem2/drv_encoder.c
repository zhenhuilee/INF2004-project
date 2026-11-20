/*
 *	drv_encoder.c
 *	Subsystem 2 -- Motion control.  Quadrature encoder driver.
 *
 *	How it works
 *	------------
 *	Every rising and falling edge on A or B raises IO_IRQ_BANK0.  The
 *	handler reads both pins of each encoder and looks the (old, new)
 *	state pair up in a table that says +1, -1 or 0.  An "impossible"
 *	jump (both pins changed at once) means an edge was missed; it is
 *	counted in g_enc_errors so you can see if the interrupt is too slow.
 *
 *	The counts are 32-bit and only written by the interrupt handler, so
 *	a task can read them at any time without locking (a 32-bit load is
 *	atomic on the Cortex-M0+).
 *
 *	The interrupt is taken by the core that calls enc_init(): call it
 *	from the motion task, which is pinned to one core.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>
#include "drv_encoder.h"

/* RP2040 IO_BANK0 interrupt registers (datasheet 2.19.6.1) */
#define INTNO_IO_BANK0		(13U)
#define IO_INTR(reg)		(IO_BANK0_BASE + 0x0F0U + ((reg) * 4U))
#define IO_PROC0_INTE(reg)	(IO_BANK0_BASE + 0x100U + ((reg) * 4U))
#define IO_PROC0_INTS(reg)	(IO_BANK0_BASE + 0x120U + ((reg) * 4U))
#define IO_PROC1_INTE(reg)	(IO_BANK0_BASE + 0x130U + ((reg) * 4U))
#define IO_PROC1_INTS(reg)	(IO_BANK0_BASE + 0x150U + ((reg) * 4U))
#define IO_INT_REGS		(4U)
#define REG_ALIAS_SET_OFFSET	(0x2000U)	/* atomic bit-set alias */
#define REG_ALIAS_CLR_OFFSET	(0x3000U)	/* atomic bit-clear alias */

#define GPIO_EVT_EDGE_LOW	(0x4U)
#define GPIO_EVT_EDGE_HIGH	(0x8U)
#define GPIO_EVT_BOTH_EDGES	(GPIO_EVT_EDGE_LOW | GPIO_EVT_EDGE_HIGH)

#define IO_REG_OF(pin)		((pin) / 8U)
#define IO_SHIFT_OF(pin)	(((pin) % 8U) * 4U)

#define ENC_IRQ_LEVEL		(2)	/* same level as the kernel's PWM timer */
#define ENC_STEP_INVALID	(2)

typedef struct
{
	uint32_t	pin_a;
	uint32_t	pin_b;
	uint32_t	b_invert;
} enc_hw_t;

static const enc_hw_t g_enc_hw[ENC_COUNT] =
{
	{ ENC_LEFT_PIN_A,  ENC_LEFT_PIN_B,  ENC_LEFT_INVERT  },
	{ ENC_RIGHT_PIN_A, ENC_RIGHT_PIN_B, ENC_RIGHT_INVERT }
};

/* index = (old_state << 2) | new_state, state = (A << 1) | B */
static const int8_t g_quad_table[16] =
{
	 0, -1, +1,  2,
	+1,  0,  2, -1,
	-1,  2,  0, +1,
	 2, +1, -1,  0
};

static volatile int32_t		g_enc_count[ENC_COUNT];
static volatile uint32_t	g_enc_state[ENC_COUNT];
static volatile uint32_t	g_enc_errors;
static volatile uint32_t	g_isr_calls;	/* diagnostics */
static volatile int32_t		g_enc_dir[ENC_COUNT];	/* single-channel mode */
static volatile enc_gpio_hook_t	g_p_gpio_hook;

static uint32_t	g_irq_mask[IO_INT_REGS];	/* our pins' edge bits */
static uint32_t	g_inte_base;			/* PROCn_INTE0 of our core */
static uint32_t	g_ints_base;			/* PROCn_INTS0 of our core */

static uint32_t
enc_read_state (const enc_hw_t *p_hw)
{
	uint32_t	in = in_w(GPIO_IN);

	return (((in >> p_hw->pin_a) & 1U) << 1U) | ((in >> p_hw->pin_b) & 1U);
}

static void
enc_isr (UINT intno)
{
	uint32_t	reg;
	uint32_t	ints;
	uint32_t	idx;
	uint32_t	new_state;
	int32_t		step;

	(void)intno;
	g_isr_calls++;

	/* 1. Acknowledge our edges first, so an edge that arrives while we
	      are decoding raises the interrupt again instead of being lost */
	for (reg = 0U; reg < IO_INT_REGS; reg++)
	{
		ints = in_w(g_ints_base + (reg * 4U));
		if ((ints & g_irq_mask[reg]) != 0U)
		{
			out_w(IO_INTR(reg), ints & g_irq_mask[reg]);
		}
		if (((ints & ~g_irq_mask[reg]) != 0U) && (g_p_gpio_hook != NULL))
		{
			g_p_gpio_hook(reg, ints & ~g_irq_mask[reg]);
		}
	}

	/* 2. Decode both encoders from the current pin levels */
	for (idx = 0U; idx < (uint32_t)ENC_COUNT; idx++)
	{
		new_state = enc_read_state(&g_enc_hw[idx]);
#if ENC_SINGLE_CHANNEL
		/* Only B is used: every change of B is one count, in the
		   direction the motor is being driven (enc_set_direction) */
		if (((new_state ^ g_enc_state[idx]) & 1U) != 0U)
		{
			g_enc_count[idx] += g_enc_dir[idx];
		}
		g_enc_state[idx] = new_state;
		continue;
#endif
		step = g_quad_table[(g_enc_state[idx] << 2U) | new_state];

		if (step == ENC_STEP_INVALID)
		{
			g_enc_errors++;
		}
		else if (step != 0)
		{
			g_enc_count[idx] += (g_enc_hw[idx].b_invert != 0U) ? -step : step;
		}
		else
		{
			/* no change on this encoder */
		}
		g_enc_state[idx] = new_state;
	}
}

static void
enc_pin_setup (uint32_t pin, uint32_t b_irq)
{
	(void)gpio_set_pin(pin, GPIO_MODE_IN);
	/* Pull-up ON, pull-down OFF.  The pad comes out of reset with the
	   pull-down enabled; leaving both on makes a weak "bus keeper" that
	   open-collector encoder outputs cannot drive high. */
	out_w(GPIO(pin) + REG_ALIAS_CLR_OFFSET, GPIO_PDE);
	out_w(GPIO(pin) + REG_ALIAS_SET_OFFSET, GPIO_PUE);
	if (b_irq == 0U)
	{
		return;
	}
	out_w(IO_INTR(IO_REG_OF(pin)), GPIO_EVT_BOTH_EDGES << IO_SHIFT_OF(pin));
	g_irq_mask[IO_REG_OF(pin)] |= GPIO_EVT_BOTH_EDGES << IO_SHIFT_OF(pin);
}

int32_t
enc_init (void)
{
	T_DINT		dint = {0};
	ER		er;
	uint32_t	idx;
	uint32_t	reg;

#if TK_SUPPORT_SMP
	if (tk_get_prc() == 2)		/* tk_get_prc(): 1 = core 0, 2 = core 1 */
	{
		g_inte_base = IO_PROC1_INTE(0U);
		g_ints_base = IO_PROC1_INTS(0U);
	}
	else
#endif
	{
		g_inte_base = IO_PROC0_INTE(0U);
		g_ints_base = IO_PROC0_INTS(0U);
	}

	for (idx = 0U; idx < (uint32_t)ENC_COUNT; idx++)
	{
		enc_pin_setup(g_enc_hw[idx].pin_a, (ENC_SINGLE_CHANNEL == 0) ? 1U : 0U);
		enc_pin_setup(g_enc_hw[idx].pin_b, 1U);
		g_enc_count[idx] = 0;
		g_enc_dir[idx] = 1;
		g_enc_state[idx] = enc_read_state(&g_enc_hw[idx]);
	}
	g_enc_errors = 0U;

	dint.intatr = TA_HLNG;
	dint.inthdr = (FP)enc_isr;
	er = tk_def_int(INTNO_IO_BANK0, &dint);
	if (er < E_OK)
	{
		return (int32_t)er;
	}

	/* Enable our pins' edge interrupts for this core only */
	for (reg = 0U; reg < IO_INT_REGS; reg++)
	{
		if (g_irq_mask[reg] != 0U)
		{
			out_w(g_inte_base + (reg * 4U) + REG_ALIAS_SET_OFFSET,
			      g_irq_mask[reg]);
		}
	}
	EnableInt(INTNO_IO_BANK0, ENC_IRQ_LEVEL);
	return (int32_t)E_OK;
}

int32_t
enc_get_count (enc_id_t id)
{
	return ((uint32_t)id < (uint32_t)ENC_COUNT) ? g_enc_count[id] : 0;
}

uint32_t
enc_get_error_count (void)
{
	return g_enc_errors;
}

void
enc_set_gpio_hook (enc_gpio_hook_t p_hook)
{
	g_p_gpio_hook = p_hook;
}

void
enc_gpio_irq_enable (uint32_t pin, uint32_t events)
{
	if (pin < GPIO_NUM)
	{
		out_w(IO_INTR(IO_REG_OF(pin)), (events & 0xFU) << IO_SHIFT_OF(pin));
		out_w(g_inte_base + (IO_REG_OF(pin) * 4U) + REG_ALIAS_SET_OFFSET,
		      (events & 0xFU) << IO_SHIFT_OF(pin));
	}
}

void
enc_debug_print (void)
{
	uint32_t	in = in_w(GPIO_IN);

	/* Pin levels, interrupt calls, and edges latched but not yet handled.
	   If the pins change but isr stays 0, the interrupt is not arriving;
	   if the pins never change, the encoders give no signal. */
	tm_printf((UB*)"[ENC] L: A=%d B=%d  R: A=%d B=%d  isr=%d  latched=%08x %08x  inte=%08x %08x\n",
		  (INT)((in >> ENC_LEFT_PIN_A) & 1U), (INT)((in >> ENC_LEFT_PIN_B) & 1U),
		  (INT)((in >> ENC_RIGHT_PIN_A) & 1U), (INT)((in >> ENC_RIGHT_PIN_B) & 1U),
		  (INT)g_isr_calls,
		  (UINT)(in_w(IO_INTR(0U)) & g_irq_mask[0]),
		  (UINT)(in_w(IO_INTR(3U)) & g_irq_mask[3]),
		  (UINT)in_w(g_inte_base), (UINT)in_w(g_inte_base + 12U));
}

void
enc_set_direction (enc_id_t id, int32_t duty)
{
	/* Remember the last non-zero drive direction: a braking or coasting
	   wheel keeps turning the way it was driven */
	if ((uint32_t)id < (uint32_t)ENC_COUNT)
	{
		if (duty > 0)
		{
			g_enc_dir[id] = 1;
		}
		else if (duty < 0)
		{
			g_enc_dir[id] = -1;
		}
		else
		{
			/* keep the previous direction */
		}
	}
}
