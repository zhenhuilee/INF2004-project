/*
 *	drv_encoder.h
 *	Subsystem 2 -- Motion control.  Quadrature wheel encoders, decoded
 *	in the GPIO interrupt (IO_IRQ_BANK0) on every edge of phase A and B
 *	("x4" decoding).
 *
 *	IO_IRQ_BANK0 is ONE interrupt shared by all GPIO pins.  This driver
 *	owns it (TEAM.md section 3).  Another subsystem that needs GPIO edge
 *	interrupts (e.g. an ultrasonic echo pin) must not call tk_def_int()
 *	for it: register a hook with enc_set_gpio_hook() instead.
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>

/* Wiring: Motor A encoder = left wheel, Motor B encoder = right wheel */
#define ENC_LEFT_PIN_A		(7U)	/* Grove 7 */
#define ENC_LEFT_PIN_B		(28U)	/* Grove 7 */
#define ENC_RIGHT_PIN_A		(27U)	/* Grove 6 */
#define ENC_RIGHT_PIN_B		(26U)	/* Grove 6 */

/* Set to 1 if that wheel's count goes DOWN when it rolls forward.
   Check it with bring-up step 3 (GUIDE.md). */
#define ENC_LEFT_INVERT		(0U)
#define ENC_RIGHT_INVERT	(1U)

/* 1 = use phase B only (phase A not working).  Each change of B is one
   count, and the direction comes from the motor command, so the
   *_INVERT switches above are not used.  0 = full A+B decoding. */
#ifndef ENC_SINGLE_CHANNEL
#define ENC_SINGLE_CHANNEL	(1)
#endif

typedef enum
{
	ENC_LEFT = 0,
	ENC_RIGHT,
	ENC_COUNT
} enc_id_t;

/* Called from the GPIO interrupt with the pending events for one of the
   four IO_BANK0 interrupt registers (reg 0..3, 8 pins each).  The hook
   must clear its own pins' edge bits in INTR<reg>. */
typedef void (*enc_gpio_hook_t)(uint32_t reg, uint32_t pending);

/* Events for enc_gpio_irq_enable() */
#define ENC_GPIO_EDGE_FALL	(0x4U)
#define ENC_GPIO_EDGE_RISE	(0x8U)

int32_t		enc_init(void);
int32_t		enc_get_count(enc_id_t id);
uint32_t	enc_get_error_count(void);
void		enc_set_gpio_hook(enc_gpio_hook_t p_hook);
void		enc_gpio_irq_enable(uint32_t pin, uint32_t events);
void		enc_debug_print(void);
void		enc_set_direction(enc_id_t id, int32_t duty);

#endif /* DRV_ENCODER_H */
