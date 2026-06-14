/*
 * EXI driver for ra-module — modeled after mload-module/gecko.c which is
 * known to drive USB Gecko on slot B (channel 1) from Starlet. We use the
 * SAME register-access pattern (inline-asm ldr/str via Write32/Read32),
 * the SAME approach of overwriting CSR completely (not read-modify-write),
 * and the SAME CR bit layout. Differences: we target channel 0 (slot A,
 * where the ESP32 lives), with our own freq + data length.
 */

#include "exi.h"
#include "types.h"
#include "ios.h"
#include "syscalls.h"
#include "tools.h"       /* Perms_Read / Perms_Write — for AHB-level access */
#include "swi_mload.h"   /* Swi_LedOn / Swi_LedOff for bisection markers */

/* ra_sleep is defined in main.c — gives us a cooperative yield that blocks
 * via os_message_queue_receive + timer. Using os_thread_yield() here led
 * to apparent hangs: with the game running and consuming CPU, yielding
 * gave control to threads that didn't yield back. A real timed block lets
 * the scheduler do the right thing. */
extern void ra_sleep(s32 time_ms);
extern void ra_sleep_us(s32 time_us);

/* ----- Hardware register addresses (Starlet virtual via MMU) -----
 * Match cios-lib/hollywood.h. EXI_REG_BASE = 0xd806800.
 * Channel offsets: 0x000, 0x014, 0x028 (per EXI0/1/2 in hollywood.h). */
#define HW_EXICTRL          0x0D800070
#define EXICTRL_ENABLE_EXI  1

#define EXI_BASE            0x0D806800
#define EXI_CHAN_STRIDE     0x14
#define EXI_CSR(ch)         (EXI_BASE + (ch) * EXI_CHAN_STRIDE + 0x00)
#define EXI_MAR(ch)         (EXI_BASE + (ch) * EXI_CHAN_STRIDE + 0x04)  /* DMA memory address */
#define EXI_LENGTH(ch)      (EXI_BASE + (ch) * EXI_CHAN_STRIDE + 0x08)  /* DMA byte count */
#define EXI_CR(ch)          (EXI_BASE + (ch) * EXI_CHAN_STRIDE + 0x0C)
#define EXI_DATA(ch)        (EXI_BASE + (ch) * EXI_CHAN_STRIDE + 0x10)

/* ----- Spin/yield budget for EXI start-bit completion -----
 * The "5000 yields = ~50ms" estimate was wildly optimistic — under load,
 * os_thread_yield() can cost milliseconds each because other runnable
 * threads execute on the yield. With EXICTRL off (or any case where CR
 * bit 0 stays stuck at 1), 5000 yields ≈ tens of seconds, which looks
 * like a hang. Keep the timeout short so a stuck transaction returns
 * quickly and the poll loop can keep blinking the LED. */
#define SPINS_BEFORE_YIELD  256
#define MAX_YIELDS          200

/* ----- Hollywood register access via MLOAD's SWI dispatch -----
 *
 * Direct inline ldr/str caused two problems:
 *   1. From a thread context (user mode), writes to certain Hollywood regs
 *      were silently dropped OR crashed the kernel (e.g. HW_EXICTRL, EXI_CSR).
 *   2. Perms_Read/Perms_Write (MRC/MCR) UNDEF in user mode.
 *
 * MLOAD's Swi_Handler (swi.c) exposes cmds 3 (read u32 from addr) and 4
 * (write u32 to addr), and runs in SVC/kernel mode via SVC 0xCC. So every
 * register access we route through Swi_MLoad bypasses both issues.
 *
 * Same syntactic interface as before (r32/w32) — just a different
 * implementation under the hood. */
static inline u32 r32(u32 addr) {
	return (u32)Swi_MLoad(3, addr, 0, 0);
}
static inline void w32(u32 addr, u32 val) {
	(void)Swi_MLoad(4, addr, val, 0);
}

/* Mirror mload-module/gecko.c:Gecko_Init() — the only known-working Starlet
 * EXI setup on this stack. Touches all three channels (not just ours):
 *   - Zero every CSR (drops any leftover CS / interrupt state)
 *   - EXI0_CSR = 0x2000 (ROMDIS — detach bootrom from chan 0)
 *   - EXI0/EXI1 CSR = (3<<10) (clear EXIINT+TCINT masks)
 *
 * Gecko's safe path only CHECKS HW_EXICTRL (gecko.c:116), assuming it's
 * already enabled. But in our cIOS the bit appears to be off at the moment
 * ra-module runs — empirically, EXI transactions never complete (CR bit 0
 * never falls), which is the signature of bus master disabled. So we
 * additionally OR EXICTRL_ENABLE_EXI into HW_EXICTRL.
 *
 * IMPORTANT: must be called from thread context, NOT at module init time.
 * The HW_EXICTRL write at module init time has been observed to hang
 * IOS_ReloadIOS (presumably because IOS is still mid-setup of EXI). */
#define EXI_BISECT_MARK()                                       \
	do { Swi_LedOn(); ra_sleep(120);                            \
	     Swi_LedOff(); ra_sleep(120); } while (0)

/* READ-ONLY exi_init — write to HW_EXICTRL crashed the kernel on 2026-05-28
 * (with 20s grace + game past wiimote warning, console still froze when we
 * OR'd EXICTRL_ENABLE_EXI into HW_EXICTRL). Hypothesis: HW_EXICTRL is
 * shared with the Disc Interface (mini's hollywood.h comments hint at this:
 * "something to do with PPCBOOT and legacy DI it seems ?!?") and any write
 * during PPC's DI/game-DOL load corrupts kernel state.
 *
 * We now just READ HW_EXICTRL and encode the current bit state in the LED:
 *   1 mark  → EXICTRL bit 0 was CLEAR (bus master OFF — bad for us)
 *   2 marks → EXICTRL bit 0 was SET   (bus master ON  — good, proceed) */
void exi_init(void) {
	u32 ctrl;
	ctrl = r32(HW_EXICTRL);
	EXI_BISECT_MARK();                 /* 1: read succeeded */
	if (ctrl & EXICTRL_ENABLE_EXI) {
		EXI_BISECT_MARK();             /* 2: bus master already ON */
	}
	/* NO WRITE — would crash kernel. If bit was clear, ra_identify will
	 * time out, but at least the thread keeps running and the LED gives
	 * us the diagnostic signal. */
}

/* CSR bit layout (per Hollywood EXI, verified against libogc/libogc/exi.c).
 * Earlier draft had bit 0/1 wrong; correct layout is:
 *   bit 1  EXI_IRQ    device-asserted INT pin pending (0x0002, RW1C)
 *   bit 3  TC_IRQ     transfer complete (0x0008, RW1C)
 *   bit 4..6  CLK     frequency (0..5)
 *   bit 7..9  CS      chip select (one bit per device 0..2)
 *   bit 11 EXT_IRQ    card insert/remove edge IRQ (0x0800, RW1C)
 *   bit 12 EXT_BIT    card present (0x1000, level)
 *
 * IRQ bits are write-1-to-clear: writing 0 leaves them unchanged. Our
 * build_csr leaves all three IRQ bits zero, so exi_select does NOT clear
 * a pending INT. That's intentional — Phase B relies on the INT bit
 * sitting latched in the CSR until we explicitly clear it. */
#define EXI_CSR_EXI_IRQ  0x0002u
#define EXI_CSR_TC_IRQ   0x0008u
#define EXI_CSR_EXT_IRQ  0x0800u

static u32 build_csr(s32 dev, s32 freq) {
	return (freq & 0x70) | (1u << (dev + 7));
}

/* CR bit layout (per Hollywood EXI):
 *   bit 0  TSTART     start / busy
 *   bit 1  DMA        0 = immediate, 1 = DMA
 *   bit 2..3  RW      0=read, 1=write   (write uses bit 2 = 1)
 *   bit 4..5  TLEN-1  transfer length minus 1 (00=1B, 11=4B)
 */
static u32 build_cr(u32 transfer_len, int write) {
	u32 cr = 1u;                                  /* TSTART */
	cr |= ((transfer_len - 1) & 3) << 4;          /* TLEN */
	if (write) cr |= (1u << 2);                   /* RW */
	return cr;
}

static s32 wait_cr_done(s32 chan) {
	u32 spins = 0, yields = 0;
	while (r32(EXI_CR(chan)) & 1) {
		if (++spins >= SPINS_BEFORE_YIELD) {
			spins = 0;
			if (++yields >= MAX_YIELDS) return -2;
			/* 250us blocking sleep — see extern comment above for why we
			 * don't use os_thread_yield. Granularity matters for the DMA
			 * paths: a 1KB DMA burst at 8MHz takes ~1ms, so the first
			 * sleep usually completes it; 1ms steps overshot by ~0.5ms
			 * per transfer. Immediate (4-byte) transfers finish inside
			 * the spin budget and never reach the sleep. Worst-case
			 * timeout: 200 yields x 250us = ~50ms. */
			ra_sleep_us(250);
		}
	}
	return 0;
}

s32 exi_select(s32 chan, s32 dev, s32 freq) {
	if (chan < 0 || chan > 2) return -1;
	if (dev  < 0 || dev  > 2) return -1;
	w32(EXI_CSR(chan), build_csr(dev, freq));
	return 0;
}

s32 exi_deselect(s32 chan) {
	if (chan < 0 || chan > 2) return -1;
	w32(EXI_CSR(chan), 0);
	return 0;
}

s32 exi_imm_write(s32 chan, const void *data, u32 len) {
	const u8 *buf = (const u8 *)data;
	while (len > 0) {
		u32 n = (len >= 4) ? 4 : len;
		u32 val = 0;
		u32 i;
		for (i = 0; i < n; i++) val |= ((u32)buf[i]) << ((3 - i) * 8);
		/* Atomic DATA+CR write through MLOAD SWI 7. Going through two
		 * separate Swi_MLoad(4,...) calls let IRQs run between the DATA
		 * register write and the CR trigger; under load that window
		 * corrupted bytes 4+ of multi-chunk transfers (matches libogc's
		 * EXI_Imm which wraps the same pair in _CPU_ISR_Disable). */
		Swi_ExiDataCr(chan, val, build_cr(n, 1));
		if (wait_cr_done(chan) < 0) return -2;
		len -= n;
		buf += n;
	}
	return 0;
}

/* (v0.27.0: exi_dma_write / exi_dma_read were removed. Two field lessons
 * survive them, documented here because both WILL bite again if DMA is
 * ever retried: (1) cios-lib's DCFlushRange is a raw `mcr p15` cache op —
 * UNDEF in USER mode, kills the thread; use the os_sync_* syscalls.
 * (2) The legacy EXI DMA engine cannot master MEM2 module memory from
 * Starlet — it clocks zeros and stalls mid-transfer (2026-06-12). The
 * kernel-batched immediate path below is the proven replacement.) */

/* ---------- Kernel-batched immediate transfers ----------
 *
 * Same wire behavior as exi_imm_write/exi_imm_read (4-byte immediate
 * chunks, CS held by the caller), but the chunk loop runs inside MLOAD's
 * SVC handler: ONE syscall per 256-byte slice instead of ~5 per chunk
 * (1 trigger + CR-poll reads). A 790-byte snapshot write drops from
 * ~1000 SVC round-trips (~3-4ms) to 4 (~1.1ms, mostly wire time).
 *
 * 256-byte slices bound the IRQs-masked window inside the kernel to
 * ~260us at 8MHz. No alignment or length constraints.
 *
 * Why not EXI DMA (exi_dma_*): field test 2026-06-12 — the engine
 * clocked zeros and stalled mid-transfer with MAR pointing at MEM2
 * module memory. The legacy EXI DMA engine can't master that path from
 * Starlet. Keep using the immediate interface, just batched. */
#define EXI_BATCH_SLICE 256u

s32 exi_batch_write(s32 chan, const void *data, u32 len) {
	const u8 *p = (const u8 *)data;
	if (chan < 0 || chan > 2) return -1;
	while (len > 0) {
		u32 n = (len >= EXI_BATCH_SLICE) ? EXI_BATCH_SLICE : len;
		if (Swi_ExiBatchWrite(chan, p, n) != 0) return -2;
		p   += n;
		len -= n;
	}
	return 0;
}

s32 exi_batch_read(s32 chan, void *data, u32 len) {
	u8 *p = (u8 *)data;
	if (chan < 0 || chan > 2) return -1;
	while (len > 0) {
		u32 n = (len >= EXI_BATCH_SLICE) ? EXI_BATCH_SLICE : len;
		if (Swi_ExiBatchRead(chan, p, n) != 0) return -2;
		p   += n;
		len -= n;
	}
	return 0;
}

s32 exi_imm_read(s32 chan, void *data, u32 len) {
	u8 *buf = (u8 *)data;
	while (len > 0) {
		u32 n = (len >= 4) ? 4 : len;
		/* Atomic CR-trigger + bounded-poll + DATA-read via MLOAD SWI 8.
		 * Doing the trigger / poll-completion / DATA-sample as three
		 * separate Swi_MLoad calls left two IRQ-enabled windows on each
		 * 4-byte chunk; for the 4108-byte chunk read in
		 * ra_fetch_watchlist_chunk (≈1027 cycles) any disturbance on
		 * the bus made parsing fail. SVC handler keeps IRQs masked
		 * through the full sequence. */
		u32 val = (u32)Swi_ExiCrRead(chan, build_cr(n, 0));
		u32 i;
		for (i = 0; i < n; i++) buf[i] = (val >> ((3 - i) * 8)) & 0xFF;
		len -= n;
		buf += n;
	}
	return 0;
}

s32 exi_transaction(s32 chan, s32 dev, s32 freq,
                    const void *wbuf, u32 wlen,
                    void *rbuf, u32 rlen) {
	/* No Perms_Read/Perms_Write here — same reason as in exi_init: the
	 * MRC/MCR instructions UNDEF in user mode and kill the thread. We
	 * accept that writes may or may not take effect depending on default
	 * DACR. If they don't take effect, wait_cr_done will time out. */
	s32 ret = exi_select(chan, dev, freq);
	if (ret < 0) return ret;
	if (wbuf && wlen) {
		ret = exi_imm_write(chan, wbuf, wlen);
		if (ret < 0) goto out;
	}
	if (rbuf && rlen) {
		ret = exi_imm_read(chan, rbuf, rlen);
	}
out:
	exi_deselect(chan);
	return ret;
}

/* ---------- Phase B device-INT handshake ----------
 *
 * ESP32 drives slot pin 2 (INT) low when it has prepared a response.
 * Hollywood's EXI controller latches EXI_CSR bit 1 (EXI_EXI_IRQ) on the
 * falling edge. The latch is sticky — stays set until we write 1 to it.
 *
 * No need to UnmaskIrq the EXI IRQ in the Starlet IRQ controller; we
 * poll the CSR directly. Avoids fighting libogc on PPC if it ever
 * attaches an EXI IRQ handler post-IOS-reload. */

s32 exi_wait_int(s32 chan, u32 timeout_ms) {
	u32 spins = 0, yields = 0;
	if (chan < 0 || chan > 2) return -1;
	while (!(r32(EXI_CSR(chan)) & EXI_CSR_EXI_IRQ)) {
		if (++spins >= SPINS_BEFORE_YIELD) {
			spins = 0;
			if (++yields >= timeout_ms) return -1;
			ra_sleep(1);
		}
	}
	return 0;
}

void exi_clear_int(s32 chan) {
	u32 csr;
	if (chan < 0 || chan > 2) return;
	csr = r32(EXI_CSR(chan));
	/* Mask out all three RW1C IRQ bits so we don't accidentally clear
	 * a TC_IRQ or EXT_IRQ pending alongside, then OR in the one we DO
	 * want to clear. Mirrors libogc's __exi_clearirqs pattern. */
	csr &= ~(EXI_CSR_EXI_IRQ | EXI_CSR_TC_IRQ | EXI_CSR_EXT_IRQ);
	csr |= EXI_CSR_EXI_IRQ;
	w32(EXI_CSR(chan), csr);
}
