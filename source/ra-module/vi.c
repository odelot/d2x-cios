/*
 * vi.c — Hollywood HW_TIMER helpers for the ra-module scheduler.
 *
 * v0.27.0: this file once carried an entire ARM-side VI/vblank detection
 * effort (vpos polling, register probing). All of it is gone, and for a
 * hardware reason worth remembering: the VI IRQ routes ONLY to Broadway's
 * Processor Interface — Hollywood's ARM IRQ controller has no VI line and
 * the Starlet MMU does not map the VI registers (reads returned constant
 * zeros; probing speculative addresses data-aborted in SVC and KILLED THE
 * KERNEL, 2026-06-09 — see [[project-vi-probe-crashed-kernel]]. NEVER
 * probe unverified MMIO from Starlet). Vblank detection lives on the PPC
 * side instead: the Gecko C0 hook increments a counter at phys 0x2FF8
 * (see main.c, [[project-ppc-vbi-hook-design]]).
 *
 * What remains here is the HW_TIMER (0x0D800010) — a free-running counter
 * at ~1.898 MHz (1 tick ≈ 526.7 ns), verified working from Starlet via
 * the MLOAD SWI register-read path. The VBI loop uses it for the 2-frame
 * timer fallback and the 5s diagnostics cadence.
 */

#include "vi.h"
#include "swi_mload.h"

#define HW_TIMER  0x0D800010

/* All Hollywood register access goes through MLOAD's SWI dispatch —
 * module threads run in USER mode and direct ldr/str to these ranges is
 * dropped or fatal ([[project-starlet-hw-access-via-swi]]). */
static inline u32 vi_r32(u32 addr) {
	return (u32)Swi_MLoad(3, addr, 0, 0);
}

u32 vi_read_hw_timer(void) {
	return vi_r32(HW_TIMER);
}

/* ticks → microseconds. 1 tick ≈ 526.7ns. Divide FIRST: keeps everything
 * in u32 (mul-first overflows past ~4.3s deltas, and 64-bit division
 * would pull libgcc's uldivmod + ARM unwind tables, which this module's
 * link.ld doesn't carry — link failure). Precision ±527us per call is
 * plenty for the 16.7ms frame budget and the 5s diag window. */
u32 vi_ticks_to_us(u32 ticks) {
	return (ticks / 1000u) * 527u;
}
