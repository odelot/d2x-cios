#ifndef _LED_H_
#define _LED_H_

#include "types.h"

/* Visible-output diagnostic via the Wii's disc-slot LED (GPIO1 bit 5).
 *
 * Reason this exists: ESP32 is silent — we have no way of knowing if the
 * ra-module poll thread is even reaching the EXI code, let alone returning
 * from it. Blink-counts at key execution points let us see what's running
 * without any external hardware.
 *
 * Implementation: forwards to MLOAD's Swi_LedOn / Swi_LedOff (SWI 0xCC,
 * commands 128/129). See led.c for why we don't poke GPIO registers
 * directly. */

void led_init(void);
void led_set(int on);

/* Blink `count` times with `period_ms` between transitions. Sleeps using the
 * same os_create_timer mechanism as ra_sleep — must be called from a thread
 * context, never during module init. */
void led_blink(int count, s32 period_ms);

#endif
