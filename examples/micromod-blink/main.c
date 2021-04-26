/*
 * Copyright (c) 2019, Emil Renner Berthing
 * Copyright (c) 2019, Thomas Flummer
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 */
#include "lib/mtimer.h"
#include "lib/eclic.h"
#include "lib/rcu.h"
#include "lib/gpio.h"

#include "MicroMod.h"

#define BLINK MTIMER_FREQ/8 /* 1/8 second */

int blinkState = 0;

void MTIMER_IRQHandler(void)
{
	uint64_t next;

	if(blinkState == 0)
	{
		gpio_pin_toggle(LED_RED);
		blinkState = 1;
	}
	else if(blinkState == 1)
	{
		gpio_pin_toggle(LED_GREEN);
		blinkState = 2;
	}
	else
	{
		gpio_pin_toggle(LED_BLUE);
		blinkState = 0;
	}

	next = mtimer_mtimecmp() + BLINK;
	MTIMER->mtimecmp_hi = next >> 32;
	MTIMER->mtimecmp_lo = next;
}

/* if the compiler can't generate functions suitable
 * for interrupt handlers, we can't implement this
 * function directly in C
 */
#ifdef __interrupt
void trap_entry(void)
{
	gpio_pin_clear(LED_RED);
	gpio_pin_clear(LED_GREEN);
	gpio_pin_clear(LED_BLUE);

	while (1)
		/* forever */;
}
#endif

static void mtimer_enable(void)
{
	uint64_t next = mtimer_mtime() + BLINK;

	MTIMER->mtimecmp_hi = next >> 32;
	MTIMER->mtimecmp_lo = next;

	eclic_config(MTIMER_IRQn, ECLIC_ATTR_TRIG_LEVEL, 1);
	eclic_enable(MTIMER_IRQn);
}

int main(void)
{
	/* initialize system clock */
	rcu_sysclk_init();

	/* initialize eclic */
	eclic_init();
	/* enable global interrupts */
	eclic_global_interrupt_enable();

	/* turn on power to GPIOA */
	RCU->APB2EN |= RCU_APB2EN_PAEN | RCU_APB2EN_PCEN;

	gpio_pin_set(LED_RED);
	gpio_pin_config(LED_RED, GPIO_MODE_PP_50MHZ);

	gpio_pin_set(LED_GREEN);
	gpio_pin_config(LED_GREEN, GPIO_MODE_PP_50MHZ);

	gpio_pin_set(LED_BLUE);
	gpio_pin_config(LED_BLUE, GPIO_MODE_PP_50MHZ);

	mtimer_enable();

	while (1)
		wait_for_interrupt();
}
