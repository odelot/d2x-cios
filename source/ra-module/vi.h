#ifndef _RA_MODULE_VI_H_
#define _RA_MODULE_VI_H_

#include "types.h"

/* Hollywood HW_TIMER (free-running, ~1.898 MHz, 1 tick ≈ 526.7 ns,
 * wraps every ~37 minutes). Used by the VBI loop for the 2-frame timer
 * fallback and the diagnostics cadence.
 *
 * (ARM-side vblank detection — vi_vblank_edge, vi_dump_state,
 * vi_probe_addrs — was removed in v0.27.0: hardware-impossible from
 * Starlet; see the vi.c header for the post-mortem.) */
u32  vi_read_hw_timer(void);

/* Convert HW_TIMER ticks → microseconds. Approximate but tight enough
 * for our 16667 us frame budget. */
u32  vi_ticks_to_us(u32 ticks);

#endif
