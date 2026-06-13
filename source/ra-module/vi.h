/*
 * VI (Video Interface) polling helpers for ra-module.
 *
 * Detect vblank by reading Hollywood VI's vertical-position register from
 * Starlet view. No IRQ install, no PPC code patches — just a polled read
 * via the same MLOAD SWI dispatch that exi.c uses.
 *
 * Used by the adaptive hybrid scheduler in main.c: when VI shows vblank,
 * the snapshot fires aligned with the game's frame quiescent window;
 * if VI is dormant (loading screen, mode change, etc.), the scheduler
 * falls through to fixed-cadence ra_sleep.
 *
 * Inspired by nes-ra-adapter's OAMDMA-write detector with time-based
 * fallback (nes-pico-firmware/src/main.c).
 */

#ifndef _RA_MODULE_VI_H_
#define _RA_MODULE_VI_H_

#include "types.h"

/* Read Hollywood TIMER register (free-running, 1 tick = 526.7 ns at the
 * Hollywood bus clock). Used by the adaptive scheduler to measure
 * inter-frame deltas. Wraps every ~37 minutes. */
u32  vi_read_hw_timer(void);

/* Convert HW_TIMER ticks → microseconds. Approximate but tight enough
 * for our 16667 us frame budget. */
u32  vi_ticks_to_us(u32 ticks);

/* True if VPOS just wrapped from end-of-field back to top — i.e. a
 * vertical retrace just happened since the previous call. Wrap detection
 * is robust to ra_sleep granularity vs the narrow ~1ms vblank window:
 * vpos rises monotonically through the active picture (~30 lines/ms) and
 * resets sharply at retrace. Any DROP in vpos between two reads means a
 * field boundary was crossed. We use a 100-line threshold to ignore
 * possible read-glitches; a real wrap drops vpos by 400+ lines.
 *
 * Returns 1 on the first call where a wrap was observed; 0 otherwise.
 * v0.20.0 used an "is currently in vblank window" check, which only
 * fired when the polling loop happened to read DURING the ~1ms retrace
 * — meaning ~50% of vblanks were missed, dropping cadence from the
 * theoretical 60Hz to ~41Hz. The wrap-detection approach catches every
 * field boundary regardless of when we sample. */
s32  vi_vblank_edge(void);

/* Diagnostic: dump current vpos/hpos + last-observed vpos + wrap counters
 * to ra_debug_send. Used by the main loop's periodic diagnostic to verify
 * that the VI register reads are actually changing. If vpos stays constant
 * across calls, the register address is wrong (Starlet MMU not mapping
 * 0x0D002000) and wrap-detection will never fire. */
void vi_dump_state(void);

/* Exploratory: read 4 bytes from each of several candidate VI register
 * addresses, dump the values via ra_debug_send. Done twice with a small
 * delay between to reveal addresses where the contents *change* (likely
 * a live VI register) vs constant (unmapped or wrong offset).
 *
 * Includes a known-working control (HW_TIMER at 0x0D800010) so we can
 * tell "all reads return 0" (Swi_MLoad fault handler) from "this specific
 * address returns 0" (mapped to zero memory). Safe — read-only via the
 * same Swi_MLoad(3, addr, ...) path the rest of the module uses. */
void vi_probe_addrs(void);

#endif
