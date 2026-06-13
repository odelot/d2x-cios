/*
 * vi.c — Hollywood VI polling for vblank detection from Starlet.
 *
 * Register addresses verified against:
 *   - libogc/libogc/video.c (PPC view: _viReg = 0xCC002000)
 *   - cios-lib/hollywood.h (Starlet ↔ PPC bridge: PPC peripherals at
 *     HW_PPC_REG_BASE = 0x0D000000 from Starlet view)
 *
 * Therefore VI base on Starlet = 0x0D002000.
 *
 * Register of interest:
 *   _viReg[22] = byte offset 0x2C = VI_VPOS
 *     bits 10..0 = current vertical position (half-line counter, 1-based)
 *     Hollywood is big-endian → from BE u32 read at 0x2C:
 *       high u16 = vpos, low u16 = hpos
 *
 * Vblank window: NTSC 480i has 525 full lines / 1050 half-lines per frame.
 * The vertical retrace lives in the first ~16 lines (~32 half-lines) of
 * each field. We use a generous threshold (vpos <= 64 half-lines) to give
 * the scheduler a few ms to react before the active picture starts.
 */

#include "vi.h"
#include "exi.h"          /* not strictly needed, but keeps hardware
                           * register access discipline visible */
#include "syscalls.h"
#include "swi_mload.h"

/* Hollywood PPC peripheral block, Starlet view. */
#define HW_PPC_REG_BASE   0x0D000000

/* VI base from Starlet view. */
#define VI_BASE           (HW_PPC_REG_BASE + 0x002000)

/* VI vertical/horizontal position register. Each u16 lives in the
 * specified half-word; the BE u32 read returns vpos in the upper half. */
#define VI_VPOS_HPOS      (VI_BASE + 0x2C)

/* Hollywood free-running timer, Starlet IO block. 1 tick = 526.7 ns.
 * (HW_REG_BASE = 0x0D800000, HW_TIMER offset 0x010.) */
#define HW_TIMER_ADDR     0x0D800010

/* Drop threshold for wrap detection. During active picture vpos rises
 * monotonically (~30 lines per ms at NTSC 60Hz × 525 lines). At field
 * boundary it resets to 1. So any read where vpos < (last_vpos - 100)
 * conclusively means a field boundary was crossed. 100 is much larger
 * than per-poll vpos delta (~30) so transient read glitches don't
 * falsely trigger; much smaller than the real wrap delta (400+) so a
 * real wrap is always detected. */
#define VI_WRAP_THRESHOLD  100

/* MLOAD SWI 3 = read u32 from MMIO. We can't direct-load these because
 * user-mode ARM threads UNDEF on Hollywood register access; same trick as
 * exi.c. */
static inline u32 vi_r32(u32 addr) {
	return (u32)Swi_MLoad(3, addr, 0, 0);
}

u32 vi_read_hw_timer(void) {
	return vi_r32(HW_TIMER_ADDR);
}

/* Hollywood timer rate: 1 tick = 526.7 ns = 1 us / 1.898.
 * Approximate ticks→us as ticks * 527 / 1000, which fits in u32 for
 * deltas under ~8 million ticks (~4 seconds — far longer than any frame).
 * For typical 16667 us frame budgets the math stays well within precision. */
u32 vi_ticks_to_us(u32 ticks) {
	/* ticks * 527 / 1000 — split to avoid intermediate overflow. */
	u32 us = (ticks / 1000) * 527;
	u32 rem = (ticks % 1000) * 527 / 1000;
	return us + rem;
}

/* Last observed vpos, used by vi_vblank_edge for wrap detection. */
static u32 vi_last_vpos = 0;

/* Diagnostic counters — every vi_vblank_edge call increments vi_calls;
 * wraps increment vi_wraps. vi_dump_state reports both so we can tell
 * "called 60 times, 0 wraps" (read returning garbage) vs "called 60 times,
 * 60 wraps" (algorithm working). */
static u32 vi_calls = 0;
static u32 vi_wraps = 0;

s32 vi_vblank_edge(void) {
	u32 vh = vi_r32(VI_VPOS_HPOS);
	u32 vpos = (vh >> 16) & 0x7FF;
	s32 fire = 0;

	vi_calls++;

	/* Wrap detection: vpos rises monotonically during the active picture,
	 * then resets sharply at the next field's start. A DROP of more than
	 * VI_WRAP_THRESHOLD lines unambiguously indicates a field boundary
	 * was crossed since the previous call. Real wraps drop vpos by ~520
	 * (NTSC) or ~620 (PAL); transient read glitches stay under ~30. */
	if (vi_last_vpos > vpos + VI_WRAP_THRESHOLD) {
		fire = 1;
		vi_wraps++;
	}

	vi_last_vpos = vpos;
	return fire;
}

/* Diagnostic only — sends a hex-encoded snapshot of VPOS/HPOS via
 * ra_debug_send. Useful during initial bring-up to confirm the address is
 * correct and the values track the game's frame. */
extern void ra_debug_send(const u8 *msg, u8 msg_len);
extern u32 ra_dbg_hex_bytes(u8 *dst, const u8 *src, u32 n, u8 sep);
extern u32 ra_dbg_u16_dec(u8 *dst, u16 v);

/* Forward decl to avoid circular include — main.c provides this. */
extern void ra_sleep_us(s32 time_us);

/* Read u32 at addr, format as " a=HHHHHHHH v=HHHHHHHH" and append to m.
 * Returns updated offset. */
static u32 vi_probe_one(u8 *m, u32 o, u32 addr) {
	u32 val = vi_r32(addr);
	u8 ab[4], vb[4];
	ab[0] = (addr >> 24) & 0xFF;
	ab[1] = (addr >> 16) & 0xFF;
	ab[2] = (addr >>  8) & 0xFF;
	ab[3] =  addr        & 0xFF;
	vb[0] = (val  >> 24) & 0xFF;
	vb[1] = (val  >> 16) & 0xFF;
	vb[2] = (val  >>  8) & 0xFF;
	vb[3] =  val         & 0xFF;
	m[o++] = ' '; m[o++] = 'a'; m[o++] = '=';
	o += ra_dbg_hex_bytes(m + o, ab, 4, 0);
	m[o++] = '='; /* sentinel between addr and value */
	o += ra_dbg_hex_bytes(m + o, vb, 4, 0);
	return o;
}

void vi_probe_addrs(void) {
	/* IMPORTANT: do NOT use `static const u32 candidates[] = {...}` here.
	 * .rodata is not loaded by the cIOS module loader on this build, so
	 * any static-const array reads back as zeros — and vi_r32(0) would
	 * read from physical 0 (game ID storage), making every probe useless.
	 * Build the candidates on the stack via plain assignments instead;
	 * the compiler inlines numeric constants directly into the code
	 * stream. See [[project-ra-module-rodata-not-loaded]].
	 *
	 * Candidates ranked by likelihood:
	 *   [0] 0x0D80202C — VI by analogy with EXI mapping (EXI: PPC
	 *       0xCC006800 → Starlet 0x0D806800; so VI: PPC 0xCC002000 →
	 *       Starlet 0x0D802000)
	 *   [1] 0x0D00202C — v0.20.2 attempt (HW_PPC_REG_BASE from
	 *       hollywood.h, returned 0 always)
	 *   [2] 0x0CC0202C — PPC virtual view (probably unmapped)
	 *   [3] 0x0D000000 — top of HW_PPC_REG_BASE, sanity sample
	 *   [4] 0x0D800010 — HW_TIMER, known working control read */
	u32 candidates[5];
	u32 i;
	u8 m[120];
	u32 o;

	candidates[0] = 0x0D80202C;
	candidates[1] = 0x0D00202C;
	candidates[2] = 0x0CC0202C;
	candidates[3] = 0x0D000000;
	candidates[4] = 0x0D800010;

	/* First pass — read all candidates. */
	o = 0;
	m[o++] = 'P'; m[o++] = '1';
	for (i = 0; i < 5; i++) {
		o = vi_probe_one(m, o, candidates[i]);
	}
	ra_debug_send(m, (u8)o);

	/* Wait ~3 ms (more than a vblank, less than a full frame) so any
	 * live VI register's vpos changes between passes. HW_TIMER will
	 * also change measurably across this gap (~5700 ticks). */
	ra_sleep_us(3000);

	o = 0;
	m[o++] = 'P'; m[o++] = '2';
	for (i = 0; i < 5; i++) {
		o = vi_probe_one(m, o, candidates[i]);
	}
	ra_debug_send(m, (u8)o);
}

void vi_dump_state(void) {
	u32 vh = vi_r32(VI_VPOS_HPOS);
	u32 vpos = (vh >> 16) & 0x7FF;
	u32 hpos = vh & 0x7FF;
	u32 raw_hi = (vh >> 16) & 0xFFFF;
	u32 raw_lo = vh & 0xFFFF;
	u8 m[128];
	u32 o = 0;

	/* Format: VI v=N h=M last=L calls=C wraps=W raw=HHHH:LLLL
	 * - v / h: masked 11-bit vpos / hpos (the values wrap-detect uses)
	 * - last: last vpos seen by vi_vblank_edge (should track v closely)
	 * - calls / wraps: cumulative counters since boot
	 * - raw: unmasked u16 halves — reveals if we're reading 0xFFFF
	 *   (unmapped MMIO) or some other constant garbage */
	m[o++] = 'V'; m[o++] = 'I'; m[o++] = ' ';
	m[o++] = 'v'; m[o++] = '=';
	o += ra_dbg_u16_dec(m + o, (u16)vpos);
	m[o++] = ' '; m[o++] = 'h'; m[o++] = '=';
	o += ra_dbg_u16_dec(m + o, (u16)hpos);
	m[o++] = ' '; m[o++] = 'l'; m[o++] = '=';
	o += ra_dbg_u16_dec(m + o, (u16)vi_last_vpos);
	m[o++] = ' '; m[o++] = 'c'; m[o++] = '=';
	o += ra_dbg_u16_dec(m + o, (u16)vi_calls);
	m[o++] = ' '; m[o++] = 'w'; m[o++] = '=';
	o += ra_dbg_u16_dec(m + o, (u16)vi_wraps);
	m[o++] = ' '; m[o++] = 'r'; m[o++] = '=';
	{
		u8 raw_bytes[4];
		raw_bytes[0] = (raw_hi >> 8) & 0xFF;
		raw_bytes[1] = raw_hi & 0xFF;
		raw_bytes[2] = (raw_lo >> 8) & 0xFF;
		raw_bytes[3] = raw_lo & 0xFF;
		o += ra_dbg_hex_bytes(m + o, raw_bytes, 4, 0);
	}
	ra_debug_send(m, (u8)o);
}
