/*
 * Disc-slot LED diagnostic via MLOAD's existing SWI handler.
 *
 * Earlier version of this file wrote directly to HW_GPIO1OUT and never got
 * the LED to turn on. The reason: the disc-slot LED is in `gpio_owner` by
 * default (per Dolphin source: GPIO::SLOT_LED is one of the bits owned by
 * PPC at boot). On real hardware, Starlet writes to HW_GPIO1OUT are merged
 * such that owner-controlled bits keep their old value — Starlet has to
 * write through the *PPC-side* HW_GPIO1BOUT (offset 0xc0) to drive SLOT_LED.
 *
 * MLOAD already implements this correctly — `mload-module/gpio.c:30` aliases
 * `GPIO_OUT = HW_GPIO1BOUT`. So instead of duplicating that logic (and
 * worrying about Perms_Write at the right moment), we call MLOAD's existing
 * SWI handler via Swi_LedOn / Swi_LedOff from cios-lib/swi_mload.h.
 *
 * SWI dispatch runs in kernel mode (SVC), so it handles its own AHB perms.
 */

#include "led.h"
#include "syscalls.h"
#include "swi_mload.h"

extern void os_thread_yield(void);

void led_init(void)
{
	/* Start with LED ON — first visible confirmation that the thread runs. */
	Swi_LedOn();
}

void led_set(int on)
{
	if (on) Swi_LedOn();
	else    Swi_LedOff();
}

/* Coarse busy-wait delay — ~1ms per outer iter at 243MHz, approximate. */
static void led_delay_ms(s32 ms)
{
	volatile u32 spins;
	s32 i;
	for (i = 0; i < ms; i++) {
		for (spins = 0; spins < 80000; spins++)
			;
		os_thread_yield();
	}
}

void led_blink(int count, s32 period_ms)
{
	int i;
	for (i = 0; i < count; i++) {
		Swi_LedOn();
		led_delay_ms(period_ms);
		Swi_LedOff();
		led_delay_ms(period_ms);
	}
}
