/*
 *  ra-module — Wii RetroAchievements memory server (ARM/Starlet side)
 *  Minimal Hello World implementation for Starlet EXI.
 */

#include <stdio.h>
#include <string.h>

#include "ios.h"
#include "ipc.h"
#include "mem.h"
#include "module.h"
#include "syscalls.h"
#include "types.h"
#include "swi_mload.h"
#include "timer.h"

#include "exi.h"
#include "vi.h"
#include "led.h"
#include "gc_ra_protocol.h"

/* Scheduling: PPC VBI sync (the only mode since v0.27.0 — the HW_TIMER
 * pacer and the fixed-16ms builds were retired after the VBI lock was
 * validated on hw 2026-06-12).
 *
 * True VBLANK alignment: WiiFlow injects a Gecko C0 cheat code that the
 * Ocarina codehandler executes on every vertical retrace (the VBI
 * hooktype patches the game's __VIRetraceHandler). The C0 code
 * increments a u32 frame counter at physical 0x2FF8 via the PPC's
 * UNCACHED MEM1 mirror. We poll that counter (plain MEM1 read with
 * os_sync_before_read — Starlet maps PPC RAM cached and a tight poll
 * pins the line) and fire a SNAPSHOT on change; if the counter never
 * moves (hook failed to install, loading screen, WiiFlow without RA
 * support) a 2-frame timer fallback fires on time instead — the
 * nes-ra-adapter hybrid pattern. See [[project-ppc-vbi-hook-design]].
 * The counter also serves as: the game-boot callback (ra_poll_thread
 * waits for it to move before starting), and the true game-frame clock
 * shipped to the ESP for rcheevos timer accuracy (v0.26.1).
 *
 * Why not ARM-side VI detection: the VI IRQ routes only to Broadway's
 * Processor Interface; Hollywood's ARM IRQ controller has no VI line
 * and the Starlet MMU doesn't map VI registers (v0.20.2 read zeros;
 * v0.20.4 probing crashed the kernel — [[project-vi-probe-crashed-
 * kernel]]). */

/* Physical MEM1 address of the PPC-side VBI frame counter. Must match the
 * C0 cheat code WiiFlow injects (ocarina_load_code in WiiFlow's
 * source/loader/fst.c writes 0xC0002FF8 = uncached mirror of this). The
 * address sits in the last 8 bytes of the Ocarina codelist region
 * (0x22A8/0x28B8..0x3000); WiiFlow caps GCT size so cheat codes can never
 * grow into it, and the booter zeroes it at boot via the codelist memset. */
#define RA_VBI_COUNTER_PHYS  0x00002FF8u

/* On-screen trophy overlay (matches RA_TROPHY_OVERLAY in WiiFlow's
 * source/loader/fst.c). When enabled, an achievement unlock writes a 1/0
 * "draw this frame" flag to physical MEM1 0x2FFC; the extended C0 VBlank
 * hook on the PPC reads it (uncached 0xC0002FFC) and paints a yellow
 * rectangle in the top-right of the active XFB, blinking 3x in lockstep
 * with the disc-slot LED. Sits in the same reserved codelist tail as the
 * frame counter (0x2FF8). Safe to leave on even if WiiFlow ships the
 * counter-only hook — the flag is simply never read. */
#define RA_TROPHY_OVERLAY    1
#define RA_TROPHY_FLAG_PHYS  0x00002FFCu

/* Galaxy-freeze forensics (2026-06-18).
 * RA_HEARTBEAT_LED RESULT: at the galaxy freeze the LED STOPPED blinking = the
 *   main loop is NOT cycling = a TRUE Starlet hang inside a fire (not alive-loop).
 *   So: every op is bounded yet it hangs -> an EXCEPTION (data abort) or a sleep
 *   that never returns. Prime suspect: a read fault on a garbage HIGH MEM2 addr
 *   (the storm queries 0x13A1879B / 0x13F20475 = IOS-reserved region above MLOAD).
 * RA_READ_LED_DIAG (now OFF): LED ON while inside a ra_read_ppc_byte loop. Was the
 *   snapshot-freeze forensic (SOLID ON at the freeze = hung on a memory READ). The
 *   freeze it chased is resolved, so it's retired — kept guarded for revert.
 * RA_INT_TIMEOUT_LED (now OFF): blip the disc-slot LED for ~1s on every new INT-wait
 *   timeout (exi_wait_int hitting its RA_INT_TIMEOUT_MS=50ms cap, counted in
 *   g_to_phaseb / g_to_dbg). Lets us SEE how often the ESP→Wii INT line fails to
 *   assert in time, by eye, without a serial cable. Non-blocking: a wall-clock
 *   deadline stepped in the main loop drives the off edge — never a 1s busy-wait,
 *   which would starve other IOS threads. Served its purpose (2026-06-20: most
 *   blinks were the RA_SPIKE_LOG observer effect, not real losses; timeouts are
 *   eval-spike-driven, see [[project_int_timeout_recovery]]) — retired, guarded. */
#define RA_READ_LED_DIAG    0
#define RA_INT_TIMEOUT_LED  0
#define RA_HEARTBEAT_LED    0

/* RA_SPIKE_LOG (default OFF): event-triggered "SPK ms=.. r=.. w=.. tb=.. td=.."
 * dump whenever a fire's convergence ran > 30ms. Useful, but NOT free: each dump
 * is a ra_debug_send that blocks on a 50ms debug-ACK INT wait, and during heavy
 * scenes (SMG bunnies) the ESP is too busy to ACK in time — so the logging itself
 * manufactures g_to_dbg INT timeouts (the bulk of the 2026-06-20 LED blinks were
 * this observer effect, not real snapshot losses). Turn ON only when actively
 * chasing a spike; leave OFF for normal play. */
#define RA_SPIKE_LOG        0

/* RA_DEBUG_SEND (default OFF): master gate for ra_debug_send, the EXI debug
 * channel that the ESP forwards to serial as "DEBUG=GC: ...". Each send is
 * NOT free — it blocks ra_sleep(10) + up to RA_INT_TIMEOUT_MS waiting for the
 * ESP's debug-ACK INT, during which do_frame does not run. With this OFF the
 * 5s VBI/PHB/OCAP diag burst (and the boot RX dumps) become no-ops, costing
 * zero sleep/EXI on the in-game path. Turn ON only when actively reading the
 * serial diag. */
#define RA_DEBUG_SEND       0

char *moduleName = "RA_MOD";

/* ------------------------------------------------------------------------
 * SD logging via fat-module — feature flag.
 *
 * Set RA_DEBUG_LOG=1 to enable. ra-module opens fat0:/ra_module.log on the
 * SD card and writes diagnostic lines as it executes. fat-module is loaded
 * in slot 247 and SD is already mounted by WiiFlow PPC (libogc) before our
 * IOS reload, so the mount state survives the reload.
 *
 * To retrieve: power off Wii, pull SD card, open /ra_module.log on PC.
 * The file is truncated each ra-module load (FA_CREATE_ALWAYS), so it
 * always contains the latest session.
 *
 * Set to 0 to make every ra_log* a no-op (zero cost when disabled).
 * ------------------------------------------------------------------------ */
/* Disabled — fat-module SD mount kept failing from our Starlet thread
 * context. Switched to pure LED-encoded diagnostic in ra_poll_thread.
 * Set back to 1 once we figure out how to mount fat-module's SDIO
 * correctly. */
#define RA_DEBUG_LOG 0

#if RA_DEBUG_LOG

static s32 ra_log_fd = -1;
/* Last status codes so the LED-blink diagnostic can report them. */
static s32 ra_log_mount_ret = 0x7FFFFFFF;
static s32 ra_log_open_ret  = 0x7FFFFFFF;

static int ra_strlen_local(const char *s)
{
	int n = 0;
	while (s[n]) n++;
	return n;
}

/* Mount SD inside fat-module. WiiFlow's PPC libogc has its own SDHC driver
 * that DOES NOT mount FatFs in fat-module from the Starlet side — that's
 * a separate concept. We have to do it ourselves before file I/O works.
 * Pattern copied from dip-plugin/fat.c:FAT_Mount — same struct layout to
 * keep vector + payload in a single 32-byte-aligned cacheable block. */
typedef struct {
	ioctlv vec[1];
	u32    partition;
} ATTRIBUTE_PACKED ra_fat_mount_buf;

static ra_fat_mount_buf ra_mount_buf __attribute__((aligned(32)));

static s32 ra_fat_mount_sd(void)
{
	s32 fd, ret;

	fd = os_open("fat", 0);
	if (fd < 0) return fd;

	ra_mount_buf.partition  = 0;  /* default partition */
	ra_mount_buf.vec[0].data = &ra_mount_buf.partition;
	ra_mount_buf.vec[0].len  = sizeof(u32);

	/* IOCTL_FAT_MOUNT_SD = 0xF0 (see dip-plugin/fat.c). */
	ret = os_ioctlv(fd, 0xF0, 1, 0, ra_mount_buf.vec);
	os_close(fd);
	return ret;
}

static void ra_log_init(void)
{
	ra_log_mount_ret = ra_fat_mount_sd();
	/* Try opening regardless — if some other module already mounted SD,
	 * the open may succeed even if our mount call errored. */
	ra_log_open_ret  = os_open("fat0:/ra_module.log", 0x0A);
	ra_log_fd        = ra_log_open_ret;
}

static void ra_log(const char *msg)
{
	if (ra_log_fd < 0) return;
	os_write(ra_log_fd, (void *)msg, ra_strlen_local(msg));
}

/* Append "label=0xVAL\n" — used for printing return codes etc. */
static void ra_log_hex(const char *label, u32 val)
{
	if (ra_log_fd < 0) return;
	char buf[80];
	int  i = 0, j;
	int  hi = 0;
	char hex[9];

	while (label[i] && i < 60) { buf[i] = label[i]; i++; }
	buf[i++] = '=';
	buf[i++] = '0';
	buf[i++] = 'x';

	if (val == 0) {
		hex[hi++] = '0';
	} else {
		while (val && hi < 8) {
			int d = val & 0xf;
			hex[hi++] = d < 10 ? ('0' + d) : ('a' + d - 10);
			val >>= 4;
		}
	}
	for (j = hi - 1; j >= 0; j--) buf[i++] = hex[j];
	buf[i++] = '\n';
	os_write(ra_log_fd, buf, i);
}

/* Force the FatFs buffer to disk so writes survive a crash. There's no
 * f_sync ioctl in fat-module, so we close + reopen as a flush proxy.
 * Each open re-creates the file (CREATE_ALWAYS), which would TRUNCATE.
 * To avoid that, we'd need OPEN_ALWAYS + seek, which fat-module doesn't
 * expose. For now, accept that flush only happens via buffer-full / power
 * cycle. Diagnostic only — losing the last <512 bytes is acceptable. */

#else
#define ra_log_init()       ((void)0)
#define ra_log(msg)         ((void)0)
#define ra_log_hex(lbl, v)  ((void)0)
#endif

/* EXI wiring: ESP32 moved to Slot B (chan 1, CS0) — matches mload-module's
 * proven gecko.c pattern. Slot A (chan 0) appears to be reserved or specially
 * handled by the IOS internals (likely memcard probing), preventing our
 * Starlet-side direct register access from driving the bus. */
#define RA_EXI_CHAN  EXI_CHAN_1
#define RA_EXI_DEV   EXI_DEVICE_0
/* v0.33: 8->16MHz. The snapshot Phase-B round-trip is ~10ms/frame (PHB s=10), dominated
 * by the EXI transfer of the ~1940-byte watchlist + response at 8MHz (wait/read are tiny).
 * At 60Hz that eats 10 of the 16.6ms budget, so convergence rounds overrun the VBI edge ->
 * the d2x misses edges -> fewer snapshots -> df/s<59 with debt=0. 16MHz ~halves the
 * transfer slice of every snapshot AND every ADDR_RESPONSE round. The earlier 16MHz attempt
 * "broke the boot" but that was actually the start.s 0x50 priority (now 0x48) + the
 * time-based GET_CHUNK (now INT-handshake/Phase B, clock-independent) — both fixed, so the
 * real EXI-clock A/B can finally run. Bad reads at higher freq degrade GRACEFULLY (magic
 * mismatch -> retry, never corruption). If 16MHz is clean, try EXI_SPEED32MHZ. Revert to
 * EXI_SPEED8MHZ if the memcard wiring can't hold the clock (symptom: bad-magic retries). */
#define RA_EXI_FREQ  EXI_SPEED16MHZ   /* 32MHz HW-tested 2026-06-30: small txns OK but the
                                       * SNAPSHOT transfer corrupts (bad mag=D7 flood) — the
                                       * memcard wiring can't hold 32MHz. 16MHz is the ceiling. */

/* Thread priorities. IOS convention: HIGHER number = HIGHER priority (same as
 * Nintendont's kernel; MLOAD sits at 0x79 = top because it's the SWI dispatcher).
 * The I/O drivers (DI/ES/FFS/USB/EHCI) sit at 0x48; FAT 0x62.
 *
 * FIELD-PROVEN CONSTRAINTS (2026-06-24):
 *   1. IOS forbids os_thread_create() from spawning a CHILD with HIGHER priority
 *      than the calling parent ("can lower, can't raise"). os_thread_create at
 *      0x64 from a 0x48 main was REJECTED -> poll thread never ran.
 *   2. Running the MAIN/IPC thread above 0x48 BREAKS boot: at 0x50 the IPC thread
 *      preempts the I/O drivers, the VBI hook never engages, 0x2FF8 stays frozen
 *      and the module is silent. The ra-module's IPC thread MUST stay at 0x48.
 *
 * So we use the Nintendont idiom: main is BORN high (start.s = RA_POLL_PRIORITY)
 * purely so it can spawn the poll thread at that priority, then main LOWERS
 * ITSELF to RA_MAIN_PRIORITY (0x48) before entering the IPC loop. Net result:
 * poll thread runs at 0x50, IPC thread at 0x48. Keep RA_POLL_PRIORITY < MLOAD 0x79
 * and start.s in sync with RA_POLL_PRIORITY. */
//#define RA_POLL_PRIORITY  0x50
#define RA_MAIN_PRIORITY  0x48

static u8  poll_thread_stack[4096] __attribute__((aligned(32)));
static u32 esp_present = 0;   /* set after a successful IDENTIFY round-trip */

/* ------------------------------------------------------------------------
 * Watchlist storage. Filled in by ra_fetch_watchlist() once after a
 * successful IDENTIFY against an ESP32 in state GAME_LOADED.
 * Addresses come from rcheevos on the ESP32 — typically PPC raw RAM
 * offsets (MEM1 = 0x00000000.. or 0x80000000..). ESP32 sends them
 * big-endian, which matches our (big-endian) Starlet, so no byte swap.
 * ------------------------------------------------------------------------ */
static u32 ra_watchlist[RA_MAX_WATCH_ADDRS] __attribute__((aligned(32)));
static u16 ra_watchlist_count = 0;

/* Receive buffer sized for the maximum possible chunk response:
 *   watchlist: esp hdr (6) + ra_watchlist_chunk_t (6) + 1024 addrs × 4 = 4108
 *   chain:     esp hdr (6) + ra_chain_chunk_t (8) + 512 nodes × 8    = 4110 */
static u8  ra_chunk_rx_buf[6 + 6 + 8 + RA_WATCHLIST_CHUNK_ADDRS * 4]  /* +6 PAD for Phase B */
                          __attribute__((aligned(32)));

/* ------------------------------------------------------------------------
 * Phase C chain descriptor table + per-frame walk state. Fetched once per
 * game via RA_CMD_GET_CHAIN_CHUNK (after the watchlist); node semantics in
 * gc_ra_protocol.h. node_count==0 => Phase C off (pure legacy).
 * ------------------------------------------------------------------------ */
#define RA_CHAIN_PARENT_NONE  0xFFFF

typedef struct __attribute__((packed)) {
	u32 operand;   /* const offset/mask/immediate, or root absolute addr */
	u16 parent;    /* earlier node index, or RA_CHAIN_PARENT_NONE */
	u8  op;        /* bits 0-4 RA_CN_OP_*, bits 5-6 width-1, bit 7 shipped */
	u8  psize;     /* RA_CN_SZ_* transform applied to the parent value */
} ra_chain_node_t;

static ra_chain_node_t ra_chain_nodes[RA_MAX_CHAIN_NODES] __attribute__((aligned(32)));
static u32 ra_chain_values[RA_MAX_CHAIN_NODES];        /* LE-pack raw per node */
static u16 ra_chain_ship_node[RA_MAX_CHAIN_NODES];     /* shipped idx -> node idx */
static u8  ra_chain_node_valid[RA_MAX_CHAIN_NODES / 8];/* bitmap per NODE */
static u16 ra_chain_node_count = 0;
static u16 ra_chain_shipped = 0;
static u16 ra_chain_win_next = 0;                      /* rotating window cursor */

static s32 ra_send(const void *tx, u32 tx_len, void *rx, u32 rx_len)
{
	return exi_transaction(RA_EXI_CHAN, RA_EXI_DEV, RA_EXI_FREQ,
	                       tx, tx_len, rx, rx_len);
}

/* ---------- Debug log channel ----------
 *
 * .rodata is not loaded in this module ([[project-ra-module-rodata-not-loaded]])
 * so we cannot use C string literals as message bodies. Instead we build the
 * payload byte-by-byte on the stack using char literals (which the compiler
 * resolves to immediate u8 constants at compile time) and a hex-nibble helper.
 *
 * Wire: RA_CMD_DEBUG_LOG + u8 msg_len + ASCII text. Fire-and-forget — no read
 * phase, no wait_int. Whatever the ESP queues as the implicit ACK sits in
 * tx_buf until the next real round, which pre-clears EXI_IRQ in ra_send_phase_b
 * anyway. Keeps the debug path from interfering with the SNAPSHOT timing. */

/* Forward declarations. ra_sleep / ra_sleep_us are defined further down
 * but the debug helpers below need to call ra_sleep before that point. */
void ra_sleep(s32 time_ms);
void ra_sleep_us(s32 time_us);

u8 ra_dbg_hex(u8 nibble)
{
	if (nibble < 10) return (u8)('0' + nibble);
	return (u8)('A' + (nibble - 10));
}

u32 ra_dbg_hex_bytes(u8 *dst, const u8 *src, u32 n, u8 sep)
{
	u32 j;
	u32 o = 0;
	for (j = 0; j < n; j++) {
		dst[o++] = ra_dbg_hex(src[j] >> 4);
		dst[o++] = ra_dbg_hex(src[j] & 0xF);
		if (sep && j + 1 < n) dst[o++] = sep;
	}
	return o;
}

/* 2026-06-19 spike forensics. The periodic PHB diag samples g_t_wait_ms every ~5s
 * → it MISSES the rare convergence-frame spikes (it=2 yet ap_us=139ms — impossible
 * for 2 round-trips; smells like ONE exi_wait_int hitting its timeout). Two INT-wait
 * sites: Phase B (the snapshot/addr-response round trip) and the debug-ACK (after
 * every ra_debug_send — and we log DEBUG=ADDR_QUERY per convergence round, so the
 * logging itself could be stalling the ESP's arm). Count timeouts at each site and
 * dump a SPK line ONLY when the convergence loop runs long. Reduced the timeout
 * 100→50 so a stall costs 50ms not 100 — if the spike halves with it, it WAS a
 * timeout. Declared up here (before ra_debug_send) since the debug-ACK uses them. */
#define RA_INT_TIMEOUT_MS  50u
volatile u32 g_to_phaseb = 0;   /* Phase B INT-wait timeouts (lifetime) */
volatile u32 g_to_dbg    = 0;   /* debug-ACK INT-wait timeouts (lifetime) */

u32 ra_dbg_u16_dec(u8 *dst, u16 v)
{
	u8 tmp[6];
	u32 n = 0, o = 0;
	if (v == 0) { dst[0] = '0'; return 1; }
	while (v > 0) { tmp[n++] = (u8)('0' + (v % 10)); v /= 10; }
	while (n > 0) dst[o++] = tmp[--n];
	return o;
}

void ra_debug_send(const u8 *msg, u8 msg_len)
{
#if !RA_DEBUG_SEND
	/* Disabled: no EXI traffic, no ra_sleep, no INT wait. Zero cost on the
	 * in-game path. See RA_DEBUG_SEND above. */
	(void)msg; (void)msg_len;
	return;
#else
	/* Largest packet: 4 hdr + 1 len + 255 msg = 260 bytes. */
	static u8 buf[260] __attribute__((aligned(32)));
	ra_gc_header_t *hdr = (ra_gc_header_t *)buf;
	u32 total;

	hdr->magic       = RA_MAGIC_GC_TO_ESP;
	hdr->command     = RA_CMD_DEBUG_LOG;
	hdr->payload_len = (u16)(1 + msg_len);

	buf[sizeof(ra_gc_header_t)] = msg_len;
	if (msg_len > 0) {
		memcpy(buf + sizeof(ra_gc_header_t) + 1, msg, msg_len);
	}

	total = sizeof(ra_gc_header_t) + 1 + msg_len;

	/* Guard against ESP still processing the previous Phase B read: give the
	 * main loop a chance to call exi_spi_arm. 10ms is generous. */
	ra_sleep(10);

	/* Pre-clear any stale INT latch from earlier rounds (a SNAPSHOT response
	 * assertion that we already consumed in the prior Phase B read, etc).
	 * Mirrors ra_send_phase_b's pre-clear discipline. */
	exi_clear_int(RA_EXI_CHAN);

	if (exi_select(RA_EXI_CHAN, RA_EXI_DEV, RA_EXI_FREQ) < 0) return;
	(void)exi_imm_write(RA_EXI_CHAN, buf, total);
	exi_deselect(RA_EXI_CHAN);

	/* CRITICAL SYNC: wait for the ESP's DEBUG_LOG handler to finish. The ESP
	 * does Serial.print(msg) (~2ms at 250000 baud for a 50-char message),
	 * preps an ACK via prepare_response, calls exi_spi_arm, and finally
	 * asserts INT (because g_response_prepared=true). We block here until
	 * that INT assertion is observed.
	 *
	 * Without this wait, ra-module returns IMMEDIATELY after its write
	 * CS-low and the main loop fires the next Phase B WRITE while the ESP
	 * is still mid-Serial.print — the slave is NOT armed → bytes are
	 * dropped → ra-module's subsequent read returns all 0xFF (idle MISO).
	 * Verified in the v0.19.1 + debug-no-sync log: every debug_send was
	 * followed by an all-FF response on the next round (lines 117/118 of
	 * wii.log timestamp 00:20:15).
	 *
	 * We don't bother reading the ACK; we just need to know the ESP is
	 * armed and ready. Timeout budget shared with Phase B. */
	if (exi_wait_int(RA_EXI_CHAN, RA_INT_TIMEOUT_MS) < 0) g_to_dbg++;
	exi_clear_int(RA_EXI_CHAN);
#endif /* RA_DEBUG_SEND */
}

/* Phase B request/response pair, gated by ESP→Wii INT line (slot pin 2 →
 * GPIO14). Eliminates the arm-after-prepare 1-tx delay that plagued the
 * single-CS-low ra_send: the ESP has time to parse the request, prepare
 * the response into tx_buf, and arm BEFORE the Wii opens the read CS-low.
 *
 * Sequence:
 *   1. WRITE phase  — exi_select + imm_write(req) + exi_deselect
 *   2. WAIT phase   — poll EXI_CSR EXI_IRQ (timeout = 100 ms)
 *   3. CLEAR phase  — RW1C the latch so the next round starts clean
 *   4. READ phase   — exi_select + imm_read(resp) + exi_deselect
 *
 * Returns 0 on success, negative on EXI error or INT timeout. INT timeout
 * means the ESP didn't respond — could indicate firmware crash, wiring
 * issue, or extreme processing latency. Caller can treat it as a missed
 * round and continue (next SNAPSHOT will try again). */
/* v0.28.9 — Phase-B timing breakdown (ms). Sampled on the snapshot path,
 * shipped in the 5s VBI diag, to find where the ~134ms ESP-side cycle goes:
 * wait (INT latency) vs mem1 (reading the watchlist) vs the rest. */
volatile u32 g_t_wait_ms = 0;
volatile u32 g_t_mem1_ms = 0;
volatile u32 g_t_snap_ms = 0;

static s32 ra_send_phase_b(void *tx, u32 tx_len, void *rx, u32 rx_len)
{
	s32 ret;

	/* Pre-clear any stale EXI_IRQ latch. The ESP firmware sets
	 * g_response_prepared=true on every prepare_response, which triggers
	 * an INT assertion after arm — including paths where the Wii doesn't
	 * actually wait for the INT (e.g. the boot IDENTIFY round-trip on
	 * legacy ra_send, or any future Phase A residual call site). Without
	 * this pre-clear, the next ra_send_phase_b's wait_int would observe
	 * the leftover latch immediately and race ahead of the ESP's
	 * SNAPSHOT-response arm.
	 *
	 * RW1C is cheap and idempotent — clearing an already-clear bit is a
	 * no-op. No race with concurrent ESP assertion because the ESP only
	 * asserts INT inside its arm() path, which can only execute after a
	 * transaction completes — and no transaction is in flight at this
	 * point in our call. */
	exi_clear_int(RA_EXI_CHAN);

	/* WRITE phase — one CS-low, write only.
	 *
	 * Kernel-batched immediate transfer: same wire behavior as the old
	 * exi_imm_write loop, but the chunk loop runs inside MLOAD's SVC
	 * handler — 1 syscall per 256-byte slice instead of ~5 per 4-byte
	 * chunk. 790-byte snapshot: ~3-4ms → ~1.1ms. (EXI DMA was tried
	 * first and field-failed: the engine can't master MEM2 module
	 * memory from Starlet — clocked zeros and stalled. See exi.c.) */
	ret = exi_select(RA_EXI_CHAN, RA_EXI_DEV, RA_EXI_FREQ);
	if (ret < 0) return ret;
	if (tx && tx_len) {
		ret = exi_batch_write(RA_EXI_CHAN, tx, tx_len);
		if (ret < 0) { exi_deselect(RA_EXI_CHAN); return ret; }
	}
	exi_deselect(RA_EXI_CHAN);

	/* WAIT phase — ESP processes + arms + asserts INT.
	 * 100 ms timeout: a healthy round trip is sub-ms; 100 ms is "the ESP
	 * is hung, give up". The CSR latch is sticky so we cannot miss the
	 * signal — only a complete absence of INT assertion can time out. */
	{   /* v0.28.9 — time the wait: if this is ~100ms the INT is arriving
	     * late (or timing out), which would explain the ~107ms cycle gap. */
		u32 _wt0 = vi_read_hw_timer();
		ret = exi_wait_int(RA_EXI_CHAN, RA_INT_TIMEOUT_MS);
		g_t_wait_ms = vi_ticks_to_us(vi_read_hw_timer() - _wt0) / 1000u;
	}
	if (ret < 0) { g_to_phaseb++; return ret; }

	/* CLEAR phase — RW1C the EXI_IRQ bit so the next round's wait starts
	 * from a clean slate. Must happen BEFORE the read CS-low so we don't
	 * race a second assertion (shouldn't happen given our serial protocol,
	 * but cheap insurance). */
	exi_clear_int(RA_EXI_CHAN);

	/* No SETTLE phase since v0.23.0. The ESP now asserts INT from the
	 * SPI driver's post_setup callback — i.e. only after the peripheral
	 * registers/DMA descriptors are genuinely loaded — so a latched INT
	 * means the read CS-low can start immediately. (Pre-v0.23.0 the
	 * assert happened right after spi_slave_queue_trans returned, before
	 * the driver task armed the DMA; responses >= 520 bytes clocked out
	 * idle 0xFF and a 1ms ra_sleep here papered over the race. Pairing
	 * rule: this d2x build REQUIRES ESP fw v0.23.0+.) */

	/* READ phase — second CS-low, read only. Kernel-batched for the same
	 * reason as the write phase: the 1024-byte response read was ~256
	 * SVC chunk round-trips (~3-5ms); batched it's 4 syscalls (~1.1ms,
	 * mostly wire time). */
	ret = exi_select(RA_EXI_CHAN, RA_EXI_DEV, RA_EXI_FREQ);
	if (ret < 0) return ret;
	if (rx && rx_len) {
		ret = exi_batch_read(RA_EXI_CHAN, rx, rx_len);
	}
	exi_deselect(RA_EXI_CHAN);
	return ret;
}

static void ra_fill_header(ra_gc_header_t *h, u8 cmd, u16 payload_len)
{
	h->magic       = RA_MAGIC_GC_TO_ESP;
	h->command     = cmd;
	h->payload_len = payload_len;
}

/* Try a single IDENTIFY round-trip. Returns the ra_send result so the
 * caller can show it as a diagnostic LED pattern. Sets esp_present on
 * successful magic+device-id match. */
static s32 ra_identify(void)
{
	ra_gc_header_t hdr;
	u32 device_id_be = 0;
	u8  rx[sizeof(ra_esp_header_t) + sizeof(u32)];
	s32 ret;
	const ra_esp_header_t *r;

	ra_fill_header(&hdr, RA_CMD_IDENTIFY, 0);
	memset(rx, 0, sizeof(rx));

	ret = ra_send(&hdr, sizeof(hdr), rx, sizeof(rx));
	if (ret < 0) {
		svc_write("[RA] EXI transmission failed\n");
		return ret;
	}

	r = (const ra_esp_header_t *)rx;
	if (r->magic != RA_MAGIC_ESP_TO_GC) {
		return 1;  /* transaction completed but magic mismatch */
	}

	memcpy(&device_id_be, rx + sizeof(ra_esp_header_t), sizeof(u32));
	if (device_id_be == RA_DEVICE_ID) {
		esp_present = 1;
	}
	return 0;
}

/* ------------------------------------------------------------------------
 * Polling thread — probes every 1 second
 * ------------------------------------------------------------------------ */

/* Cooperative blocking sleep. Create+destroy queue/timer per call —
 * slightly wasteful vs Timer_Sleep's singleton, but Timer_Sleep's
 * "send 0x555 → recv until 0x555 → recv timer" pattern caused
 * IOS_ReloadIOS to hang when called from inside wait_cr_done. The
 * dedicated per-call queue avoids whatever interaction was breaking.
 *
 * Non-static so exi.c's wait_cr_done can call it for cooperative yield. */
void ra_sleep(s32 time_ms)
{
	static u32 sleep_q_buf[4] __attribute__((aligned(32)));
	s32 q = os_message_queue_create(sleep_q_buf, 4);
	if (q < 0) return;

	s32 t = os_create_timer(time_ms * 1000, 0, q, 0x777);
	if (t >= 0) {
		u32 msg = 0;
		while (1) {
			os_message_queue_receive(q, &msg, 0);
			if (msg == 0x777) break;
		}
		os_destroy_timer(t);
	}
	os_message_queue_destroy(q);
}

/* Sub-millisecond sleep variant. Same pattern as ra_sleep but takes
 * microseconds directly. Used by the HW_TIMER-based pacer in the main
 * loop: after a frame's SNAPSHOT + multi-pass work, we sleep the residual
 * (16667us - elapsed_us) which is often a few hundred microseconds shy
 * of a full millisecond. Calling ra_sleep(0) would no-op (and also
 * doesn't accept sub-ms anyway).
 *
 * os_create_timer's first argument IS microseconds, so this is a direct
 * pass-through. Granularity in practice depends on the IOS scheduler —
 * likely ~10-50us at the low end. For 60Hz pacing that's plenty. */
void ra_sleep_us(s32 time_us)
{
	static u32 sleep_q_buf[4] __attribute__((aligned(32)));
	s32 q, t;

	if (time_us <= 0) return;

	q = os_message_queue_create(sleep_q_buf, 4);
	if (q < 0) return;

	t = os_create_timer(time_us, 0, q, 0x777);
	if (t >= 0) {
		u32 msg = 0;
		while (1) {
			os_message_queue_receive(q, &msg, 0);
			if (msg == 0x777) break;
		}
		os_destroy_timer(t);
	}
	os_message_queue_destroy(q);
}

/* ------------------------------------------------------------------------
 * Watchlist fetch — RA_CMD_GET_WATCHLIST_CHUNK
 *
 * Per gc_ra_protocol.h: GC sends header + chunk_index (2 bytes), ESP32
 * responds with esp_header + chunk_header (chunk_index, addr_count,
 * is_last, reserved) + addr_count × 4 bytes of addresses.
 *
 * We loop chunk_index 0..N until is_last==1 or hard sanity limit.
 * ------------------------------------------------------------------------ */

/* Returns:
 *   1 if this was the last chunk (caller stops iterating)
 *   0 if more chunks remain
 *   negative on error
 * Side effect: appends fetched addresses to ra_watchlist[] / count.
 *
 * INT handshake (Phase B), v0.33 — mirrors the Nintendont kernel fix
 * (ra_module.c ra_fetch_watchlist_chunk). Replaces the legacy time-based
 * two-transaction (T1 + ra_sleep(50) + T2 via plain ra_send), which guessed
 * 50ms for the ESP to prep its tx_buf and desynced when the EXI clock changed.
 * Write request -> wait for the ESP response-ready INT -> read the chunk.
 *
 * CRITICAL — the 6-byte PAD: the ESP prepends 6 bytes of 0xFF
 * (GC_WRITE_PADDING) before the response. With the legacy 2-tx those were
 * consumed by T2's WRITE phase, so the response started at read offset 0. In
 * Phase B the write and read are SEPARATE CS-low ops, so the pad lands at the
 * START of the read -> the real response begins at +PAD. (Confirmed on hw: the
 * GET_CHUNK resp head is "FF FF FF FF FF FF AE 06 ...", magic AE at +6.) */
static s32 ra_fetch_watchlist_chunk(u16 chunk_index)
{
	struct __attribute__((packed)) {
		ra_gc_header_t            hdr;
		ra_watchlist_chunk_req_t  req;
	} tx;
	ra_esp_header_t       resp;
	ra_watchlist_chunk_t  chunk;
	const u32 PAD = sizeof(ra_gc_header_t) + sizeof(ra_watchlist_chunk_req_t); /* 6 */
	u16 i;
	s32 ret;

	tx.hdr.magic       = RA_MAGIC_GC_TO_ESP;
	tx.hdr.command     = RA_CMD_GET_WATCHLIST_CHUNK;
	tx.hdr.payload_len = sizeof(ra_watchlist_chunk_req_t);
	tx.req.chunk_index = chunk_index;

	/* Single INT-gated round: write request -> wait for ESP INT -> read chunk.
	 * The INT guarantees the chunk response is prepared before we read it. */
	memset(ra_chunk_rx_buf, 0, sizeof(ra_chunk_rx_buf));
	ret = ra_send_phase_b(&tx, sizeof(tx), ra_chunk_rx_buf, sizeof(ra_chunk_rx_buf));
	if (ret < 0) return ret;

	memcpy(&resp, ra_chunk_rx_buf + PAD, sizeof(resp));
	if (resp.magic != RA_MAGIC_ESP_TO_GC) return -100;

	memcpy(&chunk, ra_chunk_rx_buf + PAD + sizeof(ra_esp_header_t), sizeof(chunk));
	if (chunk.addr_count > RA_WATCHLIST_CHUNK_ADDRS) return -101;
	if (chunk.chunk_index != chunk_index)            return -102;

	{
		const u8 *src = ra_chunk_rx_buf + PAD
		              + sizeof(ra_esp_header_t)
		              + sizeof(ra_watchlist_chunk_t);
		for (i = 0; i < chunk.addr_count
		            && ra_watchlist_count < RA_MAX_WATCH_ADDRS; i++) {
			u32 a;
			memcpy(&a, src + i * 4, sizeof(a));
			ra_watchlist[ra_watchlist_count++] = a;
		}
	}

	return chunk.is_last ? 1 : 0;
}

/* Fetch all chunks until is_last. Returns 0 on success, negative on error. */
static s32 ra_fetch_watchlist(void)
{
	u16 chunk_idx = 0;
	s32 ret;

	ra_watchlist_count = 0;

	/* Sanity cap: 32 chunks × 1024 addrs = 32k addrs (way over max). */
	while (chunk_idx < 32) {
		ret = ra_fetch_watchlist_chunk(chunk_idx);
		if (ret < 0) return ret;
		if (ret == 1) return 0;  /* is_last */
		chunk_idx++;
	}
	return -200;  /* never saw is_last */
}

/* ------------------------------------------------------------------------
 * Phase C — chain table fetch + per-frame walker.
 *
 * Semantics mirror the ESP resolver EXACTLY (spec in gc_ra_protocol.h):
 * deref node value = LE-pack of raw bytes (b0 = lowest address = bits 0-7);
 * the edge transform (psize) masks/byteswaps the PARENT's value; combines
 * are u32 ALU. All helpers are if/else chains, NOT switch — gcc can lower
 * a dense switch to a .rodata jump table and .rodata IS NOT LOADED in this
 * module ([[project-ra-module-rodata-not-loaded]]).
 * ------------------------------------------------------------------------ */

static u8 ra_read_ppc_byte(u32 addr);   /* defined below (snapshot section) */

static u32 ra_chain_xform(u32 v, u8 psize)
{
	if (psize == RA_CN_SZ_32)    return v;
	if (psize == RA_CN_SZ_32_BE) return ((v & 0xFF000000u) >> 24) |
	                                    ((v & 0x00FF0000u) >>  8) |
	                                    ((v & 0x0000FF00u) <<  8) |
	                                    ((v & 0x000000FFu) << 24);
	if (psize == RA_CN_SZ_8)     return v & 0x000000FFu;
	if (psize == RA_CN_SZ_16)    return v & 0x0000FFFFu;
	if (psize == RA_CN_SZ_24)    return v & 0x00FFFFFFu;
	if (psize == RA_CN_SZ_16_BE) return ((v & 0xFF00u) >> 8) | ((v & 0x00FFu) << 8);
	if (psize == RA_CN_SZ_24_BE) return ((v & 0xFF0000u) >> 16) | (v & 0x00FF00u) |
	                                    ((v & 0x0000FFu) << 16);
	return v;
}

static u32 ra_chain_combine(u32 pv, u32 o, u8 op)
{
	if (op == RA_CN_OP_AND)        return pv & o;
	if (op == RA_CN_OP_ADD)        return pv + o;
	if (op == RA_CN_OP_SUB)        return pv - o;
	if (op == RA_CN_OP_MULT)       return pv * o;
	if (op == RA_CN_OP_DIV)        return o ? pv / o : 0;
	if (op == RA_CN_OP_XOR)        return pv ^ o;
	if (op == RA_CN_OP_MOD)        return o ? pv % o : 0;
	if (op == RA_CN_OP_SUB_PARENT) return o - pv;
	if (op == RA_CN_OP_ADD_ACC)    return pv + o;
	if (op == RA_CN_OP_SUB_ACC)    return pv - o;
	return 0;
}

/* Same MEM1/MEM2 guard as the ESP collect: a wild deref can hang the
 * Starlet thread (SVC data abort — [[project_vi_probe_crashed_kernel]]). */
static int ra_chain_addr_ok(u32 addr, u8 w)
{
	u32 end = addr + (u32)w - 1u;
	if (end < addr) return 0;                                 /* wrap */
	if (end <= 0x017FFFFFu) return 1;                         /* MEM1 */
	if (addr >= 0x10000000u && end <= 0x137FFFFFu) return 1;  /* MEM2 (IOS top excl.) */
	return 0;
}

/* Walk the whole table in slot order (parents always precede children).
 * Fills ra_chain_values[] + ra_chain_node_valid[]. Children of an invalid
 * node are invalid without reading. */
static void ra_chain_walk(void)
{
	u16 i;

	for (i = 0; i < ra_chain_node_count; i++) {
		const ra_chain_node_t *n = &ra_chain_nodes[i];
		u8  op    = n->op & 0x1F;
		u8  valid = 1;
		u32 pv    = 0;
		u32 v     = 0;

		if (n->parent != RA_CHAIN_PARENT_NONE) {
			if (!(ra_chain_node_valid[n->parent >> 3] & (1u << (n->parent & 7))))
				valid = 0;
			else
				pv = ra_chain_xform(ra_chain_values[n->parent], n->psize);
		}

		if (op == RA_CN_OP_NONE) {
			v = n->operand;
		}
		else if (op == RA_CN_OP_DEREF) {
			u32 addr = (n->parent == RA_CHAIN_PARENT_NONE) ? n->operand
			                                               : (pv + n->operand);
			u8 w = (u8)(((n->op >> 5) & 3) + 1);
			if (valid && ra_chain_addr_ok(addr, w)) {
				u8 k;
				for (k = 0; k < w; k++)
					v |= (u32)ra_read_ppc_byte(addr + k) << (8u * k);
			} else {
				valid = 0;
				v = 0;
			}
		}
		else {
			if (valid)
				v = ra_chain_combine(pv, n->operand, op);
		}

		ra_chain_values[i] = v;
		if (valid)
			ra_chain_node_valid[i >> 3] |=  (u8)(1u << (i & 7));
		else
			ra_chain_node_valid[i >> 3] &= (u8)~(1u << (i & 7));
	}
}

/* Fetch one chain-table chunk. Same Phase B + PAD=6 pattern as
 * ra_fetch_watchlist_chunk. *base = nodes accumulated so far. Returns 1 on
 * is_last, 0 for more, negative on error. */
static s32 ra_fetch_chain_chunk(u16 chunk_index, u16 *base)
{
	struct __attribute__((packed)) {
		ra_gc_header_t        hdr;
		ra_chain_chunk_req_t  req;
	} tx;
	ra_esp_header_t   resp;
	ra_chain_chunk_t  chunk;
	const u32 PAD = sizeof(ra_gc_header_t) + sizeof(ra_chain_chunk_req_t); /* 6 */
	u16 i;
	s32 ret;

	tx.hdr.magic       = RA_MAGIC_GC_TO_ESP;
	tx.hdr.command     = RA_CMD_GET_CHAIN_CHUNK;
	tx.hdr.payload_len = sizeof(ra_chain_chunk_req_t);
	tx.req.chunk_index = chunk_index;

	memset(ra_chunk_rx_buf, 0, sizeof(ra_chunk_rx_buf));
	ret = ra_send_phase_b(&tx, sizeof(tx), ra_chunk_rx_buf, sizeof(ra_chunk_rx_buf));
	if (ret < 0) return ret;

	memcpy(&resp, ra_chunk_rx_buf + PAD, sizeof(resp));
	if (resp.magic != RA_MAGIC_ESP_TO_GC) return -100;

	memcpy(&chunk, ra_chunk_rx_buf + PAD + sizeof(ra_esp_header_t), sizeof(chunk));
	if (chunk.node_count > RA_CHAIN_CHUNK_NODES)          return -101;
	if (chunk.chunk_index != chunk_index)                 return -102;
	if (chunk.total_nodes > RA_MAX_CHAIN_NODES)           return -103;
	if ((u32)*base + chunk.node_count > chunk.total_nodes) return -104;

	memcpy(&ra_chain_nodes[*base],
	       ra_chunk_rx_buf + PAD + sizeof(ra_esp_header_t) + sizeof(ra_chain_chunk_t),
	       (u32)chunk.node_count * RA_CHAIN_NODE_SIZE);

	/* Structural validation: parents must reference EARLIER slots (the
	 * walker is a single forward pass) and ops must be known. A violation
	 * poisons the whole table -> caller disables Phase C. */
	for (i = 0; i < chunk.node_count; i++) {
		const ra_chain_node_t *n = &ra_chain_nodes[*base + i];
		u8 op = n->op & 0x1F;
		if (n->parent != RA_CHAIN_PARENT_NONE && n->parent >= (u16)(*base + i))
			return -105;
		if (op != RA_CN_OP_NONE && op != RA_CN_OP_DEREF &&
		    (op < RA_CN_OP_MULT || op > RA_CN_OP_SUB_ACC))
			return -106;
	}

	*base = (u16)(*base + chunk.node_count);
	return chunk.is_last ? 1 : 0;
}

/* Fetch the whole chain table. Any failure => Phase C off (legacy-only);
 * an ESP without a table answers node_count=0/is_last=1 on chunk 0. */
static void ra_fetch_chain_table(void)
{
	u16 base = 0;
	u16 chunk_idx = 0;
	u16 i;
	s32 ret;

	ra_chain_node_count = 0;
	ra_chain_shipped    = 0;
	ra_chain_win_next   = 0;

	while (chunk_idx < (RA_MAX_CHAIN_NODES / RA_CHAIN_CHUNK_NODES) + 1) {
		ret = ra_fetch_chain_chunk(chunk_idx, &base);
		if (ret < 0) {
			u8 m[24];
			u32 o = 0;
			m[o++]='C';m[o++]='H';m[o++]='N';m[o++]=' ';m[o++]='e';m[o++]='r';m[o++]='r';m[o++]='=';
			o += ra_dbg_u16_dec(m + o, (u16)(-ret));
			ra_debug_send(m, (u8)o);
			return;   /* legacy-only */
		}
		if (ret == 1) break;
		chunk_idx++;
	}

	ra_chain_node_count = base;
	for (i = 0; i < ra_chain_node_count; i++) {
		if (ra_chain_nodes[i].op & 0x80)
			ra_chain_ship_node[ra_chain_shipped++] = i;
	}

	{
		u8 m[32];
		u32 o = 0;
		m[o++]='C';m[o++]='H';m[o++]='N';m[o++]=' ';m[o++]='n';m[o++]='=';
		o += ra_dbg_u16_dec(m + o, ra_chain_node_count);
		m[o++]=' ';m[o++]='s';m[o++]='h';m[o++]='=';
		o += ra_dbg_u16_dec(m + o, ra_chain_shipped);
		ra_debug_send(m, (u8)o);
	}
}

/* ------------------------------------------------------------------------
 * SNAPSHOT / ADDR_RESPONSE state machine — reconstruction 2026-06-01.
 *
 * The previous full-featured main.c (with these functions) was overwritten
 * after building ra-module.elf.orig (May 31 10:20). This reconstruction
 * follows the protocol semantics defined in gc_ra_protocol.h, mirrors the
 * GameCube ra_agent.c (swiss-gc) pattern, and applies every fix the bug
 * archaeology documented:
 *
 *   - Twin function divergence: WATCHLIST_APPEND parse MUST be in both
 *     ra_send_snapshot AND ra_send_addr_response. (Same for the new
 *     WATCHLIST_TRUNCATE event added in Phase A.)
 *
 *   - Canonical write padding: every command must pad its write to
 *     SNAPSHOT_REQ_LEN so ESP32's last_snap_req_len pad on its prepared
 *     response stays in sync with where we start reading.
 *
 *   - Pre-burst sleep: a brief sleep before each ADDR_RESPONSE gives the
 *     ESP32 SPI slave time to arm its descriptor; without it, the first
 *     multi-pass write races into "no descriptor queued" and gets dropped.
 *
 *   - .rodata not loaded: string literals would read as zeros at runtime
 *     (linker layout quirk), so we keep all communication payload-driven
 *     (no string compares, no const tables that aren't compile-time
 *     constants the compiler inlines).
 * ------------------------------------------------------------------------ */

/* Max addresses ESP32 can request in one ADDR_QUERY round. Must match
 * the ESP32 side's ADDR_QUERY_MAX (v0.28.7: 128→512 to collapse the
 * multi-pass resolution rounds — fewer EXI round-trips per transition). */
#define RA_ADDR_QUERY_MAX  512

/* Pending ADDR_QUERY state, populated by ra_parse_response when the
 * response carries an ADDR_QUERY event. Consumed by ra_send_addr_response. */
static u32 ra_pending_query_addrs[RA_ADDR_QUERY_MAX] __attribute__((aligned(32)));
static u16 ra_pending_query_count = 0;

/* Shared TX buffer — large enough for the worst-case SNAPSHOT
 * (header + max watchlist values) and reused for ADDR_RESPONSE writes.
 * Goes out via exi_batch_write (kernel-batched immediate chunks — no
 * alignment/length constraints; the 32B alignment + headroom are
 * leftovers from the abandoned EXI-DMA attempt, kept as harmless).
 * Phase C: sized to the full 8192B EXI transaction so the snapshot can
 * carry flat values + the chain window (bitmap + blob); the window
 * budget in ra_send_snapshot is derived from sizeof() this buffer. */
static u8 ra_snap_tx_buf[8192] __attribute__((aligned(32)));

/* RX buffer — sized to hold the ESP32's longest response. Since v0.28.7
 * (RA_ADDR_QUERY_MAX=512) that is an ADDR_QUERY or WATCHLIST_APPEND:
 * 6 (esp_hdr) + 4 (append seq+count) + 512*4 = 2058 bytes. (A 256-entry
 * REMOVE_IDX is only 6+4+256*2 = 1032.) Derived from RA_ADDR_QUERY_MAX so
 * the two stay in lockstep, rounded up to a multiple of 32 for the aligned
 * attr. NOTE: ra_send_phase_b reads this whole buffer every transaction,
 * so its size is added wire-time on EVERY snapshot (~+1ms at 2080B) — the
 * win is fewer rounds per transition, not cheaper steady-state reads. */
#define RA_SNAP_RX_BUF_SZ  (((6 + 4 + RA_ADDR_QUERY_MAX * 4) + 31) & ~31u)
static u8 ra_snap_rx_buf[RA_SNAP_RX_BUF_SZ] __attribute__((aligned(32)));

/* Monotonic per-frame counter so ESP32 can detect dropped frames. */
static u32 ra_frame_counter = 0;

/* Latest PPC VBI counter value (true game-frame clock from the C0 hook).
 * Updated by the VBI loop each fire. Shipped in the SNAPSHOT header
 * (v0.26.1) so rcheevos timers count real game frames, not delivered
 * snapshots. */
static u32 ra_game_frame = 0;

/* Phase D2 (v0.26.0) — verified watchlist sync. ra_wl_seq is the seq of
 * the last mutation we APPLIED; echoed in every SNAPSHOT header so the
 * ESP knows our exact position in its mutation stream. We apply a
 * mutation iff its seq == ra_wl_seq+1 — duplicates (seq <= ours) and
 * gaps (seq > ours+1) are dropped; the ESP redelivers the right one
 * based on the echo. */
static u16 ra_wl_seq = 0;

/* In-game RESYNC request, set when a WATCHLIST_UPDATE event arrives
 * mid-session (the ESP detected an unrecoverable desync). The main loop
 * re-runs the chunk fetch and adopts the notify's seq base. */
static u8  ra_resync_pending = 0;
static u16 ra_resync_seq = 0;

/* Countdown driving the non-blocking achievement-unlock celebration on
 * the disc-slot LED. Set by ra_parse_response when the ESP attaches an
 * RA_EVT_ACHIEVEMENT to a snapshot ACK; stepped once per fire in the
 * main loop — never sleeps, so the snapshot cadence is untouched. */
static u16 ra_led_celebrate = 0;

#if RA_TROPHY_OVERLAY
/* Trophy-flag diagnostic latch (2026-06-22). The flag write to PPC MEM1
 * 0x2FFC never showed the overlay though the LED blinked; these record the
 * last write/read-back so the reliable 5s diag can report "TRO n=.. w=.. rb=.."
 * (a burst of ra_debug_send during the 0.9s celebration was unreliable).
 * n>0 after an unlock proves the celebration code ran (build is fresh). */
static u32 g_tro_writes = 0;
static u32 g_tro_last_w = 0;
static u32 g_tro_last_rb = 0;
#endif

/* INT-timeout diagnostic blink state (RA_INT_TIMEOUT_LED). ra_led_to_seen
 * tracks the last-observed lifetime INT-timeout total (g_to_phaseb +
 * g_to_dbg); when it grows, the disc-slot LED lights and ra_led_to_on_tick
 * records the hw-timer tick, so the main loop can turn it back off ~1s later
 * without ever blocking. */
#if RA_INT_TIMEOUT_LED
static u32 ra_led_to_seen    = 0;
static u32 ra_led_to_on_tick = 0;
static u8  ra_led_to_on      = 0;
#endif

/* Wii Starlet ARM can read PPC RAM directly: MEM1 at physical 0x00xxxxxx
 * and MEM2 at 0x10xxxxxx are mapped into Starlet's address space. The
 * ESP32 puts raw PPC physical offsets in the watchlist (0x008C2760 etc),
 * so a direct dereference suffices.
 *
 * Staleness caveat (proven in the field, v0.22.2): these mappings are
 * CACHED on Starlet. Scattered snapshot reads (~900 addresses/frame)
 * thrash the whole D-cache so each frame's values are effectively fresh
 * — no sync needed here. But a tight poll of a SINGLE address pins its
 * cache line and reads stale data for tens of ms; the VBI counter poll
 * needs os_sync_before_read before every read (see the VBI loop). */
/* Conservative upper bound of the game's MEM2 arena. The IOS-reserved top of
 * MEM2 (modules/heap/IPC, MLOAD lives at 0x13700000) starts well above any
 * game's working set; a wild pointer landing there can fault the Starlet. The
 * galaxy storm queries garbage like 0x13A1879B / 0x13F20475 (both >= this). */
#define RA_MEM2_SAFE_HI  0x13800000u

static u8 ra_read_ppc_byte(u32 addr)
{
	/* Defensive: the ESP supplies addresses resolved from game pointer chains,
	 * which during a galaxy load can be in-flux GARBAGE. Only deref known-mapped
	 * game RAM (MEM1, all RAM; + the game's MEM2 region). A wild address outside
	 * that range must NOT be dereferenced — an unmapped/protected addr would
	 * data-abort the Starlet and hang IOS (the galaxy-freeze suspect). Return 0
	 * so the chain just reads 0 (the do_frame gate handles the miss). */
	if (addr <= 0x017FFFFFu)                              /* MEM1 (24MB, all RAM) */
		return *(volatile u8 *)addr;
	if (addr >= 0x10000000u && addr < RA_MEM2_SAFE_HI)    /* MEM2 game region */
		return *(volatile u8 *)addr;
	return 0;
}

/* Central response parser — called by BOTH ra_send_snapshot and
 * ra_send_addr_response after ra_send_phase_b returns. Mutates global
 * state (ra_pending_query_*, ra_watchlist*) as the event dictates.
 *
 * The response lives at offset 0 of rx_buf. Under Phase B (v0.19.1+)
 * the ESP places its prepared response at tx_buf[0] and the dedicated
 * read CS-low clocks it out from byte 0 — no leading padding needed.
 * (The pre-Phase-B comment here described "last_snap_req_len bytes of
 * 0xFF" prepended for the legacy single-CS-low write+read pattern; that
 * mechanism is gone.) */
static void ra_parse_response(const u8 *buf)
{
	ra_esp_header_t resp;

	memcpy(&resp, buf, sizeof(resp));

	/* DIAG (anomaly-only): if magic check fails, log the leading byte AND
	 * the first 16 bytes of the response. Magic-fail means ra-module either
	 * read default_ack (timing race), all-FF (slave unarmed), or completely
	 * unexpected garbage — exact bytes pinpoint which. Stays silent on the
	 * happy path, so it's safe at 60Hz. */
	if (resp.magic != RA_MAGIC_ESP_TO_GC) {
		u8 m[80];
		u32 o = 0;
		m[o++] = 'b'; m[o++] = 'a'; m[o++] = 'd'; m[o++] = ' ';
		m[o++] = 'm'; m[o++] = 'a'; m[o++] = 'g'; m[o++] = '=';
		o += ra_dbg_hex_bytes(m + o, &resp.magic, 1, 0);
		m[o++] = ' '; m[o++] = 'R'; m[o++] = 'X'; m[o++] = '=';
		o += ra_dbg_hex_bytes(m + o, buf, 16, ' ');
		ra_debug_send(m, (u8)o);
		return;
	}

	if (resp.event_type == RA_EVT_ADDR_QUERY) {
		ra_addr_query_t aq;
		memcpy(&aq, buf + sizeof(resp), sizeof(aq));
		if (aq.addr_count > 0 && aq.addr_count <= RA_ADDR_QUERY_MAX) {
			const u8 *src = buf + sizeof(resp) + sizeof(aq);
			u16 j;
			for (j = 0; j < aq.addr_count; j++) {
				memcpy(&ra_pending_query_addrs[j], src + j * 4, 4);
			}
			ra_pending_query_count = aq.addr_count;
			/* UNIFY (v0.30): append the queried addresses to the permanent
			 * watchlist NOW, in query order. The ESP appends the SAME set
			 * when it receives our ADDR_RESPONSE — no separate
			 * WATCHLIST_APPEND mutation. Both replicas grow identically;
			 * Phase D2's count check verifies the append, and the wl_seq
			 * channel is now REMOVE-only. No dedup here: collect_missing
			 * already deduped the query (a stray dup lands on BOTH sides ->
			 * still in sync). Cap: if full we skip — the ESP hits the same
			 * cap at the same count and skips too. Reclaiming space via
			 * eviction is the next milestone. */
			if ((u32)ra_watchlist_count + aq.addr_count <= RA_MAX_WATCH_ADDRS) {
				for (j = 0; j < aq.addr_count; j++) {
					ra_watchlist[ra_watchlist_count + j] = ra_pending_query_addrs[j];
				}
				ra_watchlist_count += aq.addr_count;
			}
		}
	}
	else if (resp.event_type == RA_EVT_WATCHLIST_APPEND) {
		/* Phase D2 (v0.26.0): apply iff seq == ra_wl_seq+1. A stale
		 * duplicate (seq <= ours) or a gap (seq > ours+1) is silently
		 * dropped — the ESP redelivers the right mutation based on the
		 * wl_seq echo in our next SNAPSHOT. This makes delivery
		 * idempotent BY CONSTRUCTION (the v0.24.6 duplicated-append
		 * corruption class cannot exist). */
		ra_watchlist_append_t wa;
		memcpy(&wa, buf + sizeof(resp), sizeof(wa));
		if (wa.seq != (u16)(ra_wl_seq + 1)) {
			/* drop — out-of-sequence delivery */
		}
		else if (wa.addr_count > 0
		    && (u32)ra_watchlist_count + wa.addr_count <= RA_MAX_WATCH_ADDRS) {
			const u8 *src = buf + sizeof(resp) + sizeof(wa);
			u16 j;
			for (j = 0; j < wa.addr_count; j++) {
				u32 addr;
				memcpy(&addr, src + j * 4, 4);
				ra_watchlist[ra_watchlist_count + j] = addr;
			}
			ra_watchlist_count += wa.addr_count;
			ra_wl_seq = wa.seq;
		} else {
			/* DIAG (anomaly-only): APPEND rejected. Either zero-count
			 * (protocol violation) or would overflow RA_MAX_WATCH_ADDRS
			 * (ESP-side capacity bug — ESP shouldn't APPEND past 1024).
			 * Log the raw wa.addr_count bytes and current state. */
			u8 m[64];
			u32 o = 0;
			m[o++] = 'A'; m[o++] = 'P'; m[o++] = 'P'; m[o++] = 'X';
			m[o++] = ' '; m[o++] = 'n'; m[o++] = '=';
			o += ra_dbg_u16_dec(m + o, wa.addr_count);
			m[o++] = ' '; m[o++] = 'w'; m[o++] = 'c'; m[o++] = '=';
			o += ra_dbg_u16_dec(m + o, ra_watchlist_count);
			m[o++] = ' '; m[o++] = 'r'; m[o++] = 'a'; m[o++] = 'w'; m[o++] = '=';
			o += ra_dbg_hex_bytes(m + o, buf + sizeof(resp), 2, 0);
			ra_debug_send(m, (u8)o);
		}
	}
	/* (v0.27.0: the legacy address-based RA_EVT_WATCHLIST_REMOVE (0x0D)
	 * parse was removed — the ESP only emits REMOVE_IDX since v0.25.0,
	 * and the O(R*N) address search cost ~2 frames at N=6144. The enum
	 * value stays reserved in gc_ra_protocol.h; do not renumber.) */
	else if (resp.event_type == RA_EVT_WATCHLIST_REMOVE_IDX) {
		/* v0.25.0 — removal by array INDEX. Both replicas are identical
		 * before the removal, so the ESP sends the u16 indices (BE,
		 * ascending) of the entries to drop. Single-pass compaction
		 * with a skip cursor: O(N), vs the address-based REMOVE's
		 * O(R*N) linear search (~25-30ms ≈ 2 frames at N=6144 — the
		 * "cleanup doesn't fit in a frame" weak point). Index identity
		 * also kills the duplicate-address ambiguity class. */
		ra_watchlist_remove_idx_t wri;
		memcpy(&wri, buf + sizeof(resp), sizeof(wri));
		if (wri.seq != (u16)(ra_wl_seq + 1)) {
			/* Phase D2: out-of-sequence — drop; ESP redelivers. */
		}
		else if (wri.idx_count > 0 && wri.idx_count <= ra_watchlist_count) {
			const u8 *src = buf + sizeof(resp) + sizeof(wri);
			u16 i, skip = 0, write_idx = 0;
			for (i = 0; i < ra_watchlist_count; i++) {
				if (skip < wri.idx_count) {
					u16 idx;
					memcpy(&idx, src + skip * 2, 2);
					if (idx == i) {
						skip++;
						continue;
					}
				}
				if (write_idx != i)
					ra_watchlist[write_idx] = ra_watchlist[i];
				write_idx++;
			}
			ra_watchlist_count = write_idx;
			ra_wl_seq = wri.seq;
		}
	}
	else if (resp.event_type == RA_EVT_WATCHLIST_UPDATE) {
		/* Phase D2 in-game RESYNC: the ESP detected an unrecoverable
		 * desync and published a fresh full list. Defer the chunk
		 * re-fetch to the main loop (it takes ~300ms of legacy ra_send
		 * round-trips — too long for this parse context). */
		ra_watchlist_notify_t wn;
		memcpy(&wn, buf + sizeof(resp), sizeof(wn));
		ra_resync_seq = wn.seq;
		ra_resync_pending = 1;
	}
	else if (resp.event_type == RA_EVT_ACHIEVEMENT) {
		/* Achievement unlocked! The payload (id + title) is for richer
		 * clients; here the edge is all we need — celebrate on the
		 * disc-slot LED. The main loop steps the pattern one tick per
		 * fire (see the VBI loop), so nothing blocks. */
		ra_led_celebrate = 54;  /* 9 frames ON / 9 OFF x3 ≈ 0.9s at 60Hz */
	}
}

/* Send SNAPSHOT — read all watchlist values from PPC RAM and ship them via
 * Phase B (INT-handshake). Two CS-low pulses: write the request, wait for
 * the ESP's INT assertion (signaling it has armed the response), read the
 * response. Length is natural — no canonical padding, response lands at
 * tx_buf[0] on ESP side. */
static s32 ra_send_snapshot(void)
{
	ra_gc_header_t        *hdr;
	ra_snapshot_header_t  *snap;
	u16 count = ra_watchlist_count;
	u32 tx_len, rx_len;
	s32 ret;
	u16 i;
	u8 *values;

	u16 win_first = 0, win_count = 0;
	u32 blob_len = 0, bm_bytes = 0;

	hdr = (ra_gc_header_t *)ra_snap_tx_buf;
	hdr->magic       = RA_MAGIC_GC_TO_ESP;
	hdr->command     = RA_CMD_SNAPSHOT;

	snap = (ra_snapshot_header_t *)(ra_snap_tx_buf + sizeof(ra_gc_header_t));
	/* v0.26.1: frame_counter carries the PPC VBI counter (TRUE game
	 * frames, including ones we never fired on) so the ESP can call
	 * rc_client_do_frame once per GAME frame via debt catch-up.
	 * rcheevos hit-count timers assume do_frame==60Hz; our delivered
	 * rate is 55-95% of that, which made "3600 hits = 60s" timers run
	 * 60-109 real seconds (false-easy unlock: SMG bunnies 2026-06-12).
	 * Falls back to the fire counter when the VBI hook isn't running
	 * (ra_game_frame stays 0) — ESP treats a frozen value as +1/frame. */
	++ra_frame_counter;
	snap->frame_counter = (ra_game_frame != 0) ? ra_game_frame
	                                           : ra_frame_counter;
	snap->addr_count    = count;
	snap->wl_seq        = ra_wl_seq;  /* Phase D2 sync echo */

	values = ra_snap_tx_buf + sizeof(ra_gc_header_t)
	                        + sizeof(ra_snapshot_header_t);
	{   /* v0.28.9 — time reading the whole watchlist from PPC MEM1.
	     * Phase C: the chain walk reads MEM1 too, so it lives inside the
	     * same timing block (visible as m= in the PHB diag). */
		u32 _m0 = vi_read_hw_timer();
#if RA_READ_LED_DIAG
		if (!ra_led_celebrate) Swi_LedOn();   /* LED ON = inside the snapshot PPC read loop */
#endif
		for (i = 0; i < count; i++) {
			values[i] = ra_read_ppc_byte(ra_watchlist[i]);
		}
		if (ra_chain_node_count > 0)
			ra_chain_walk();
#if RA_READ_LED_DIAG
		if (!ra_led_celebrate) Swi_LedOff();
#endif
		g_t_mem1_ms = vi_ticks_to_us(vi_read_hw_timer() - _m0) / 1000u;
	}

	/* Phase C chain window — greedy fill from the rotating cursor until the
	 * 8192B transaction budget is hit; next frame resumes where we stopped
	 * (wraps to 0 at the end). Bitmap bit i = window slot i valid. */
	if (ra_chain_node_count > 0 && ra_chain_shipped > 0) {
		u32 avail = sizeof(ra_snap_tx_buf) - sizeof(ra_gc_header_t)
		          - sizeof(ra_snapshot_header_t) - count;
		u16 s = (ra_chain_win_next < ra_chain_shipped) ? ra_chain_win_next : 0;

		win_first = s;
		while (s < ra_chain_shipped) {
			const ra_chain_node_t *n = &ra_chain_nodes[ra_chain_ship_node[s]];
			u8  w  = (u8)(((n->op >> 5) & 3) + 1);
			u32 nb = (((u32)win_count + 1u) + 7u) / 8u;
			if (nb + blob_len + w > avail) break;
			blob_len += w;
			win_count++;
			s++;
		}
		bm_bytes = ((u32)win_count + 7u) / 8u;
		ra_chain_win_next = (s >= ra_chain_shipped) ? 0 : s;

		{
			u8 *bm   = values + count;
			u8 *blob = bm + bm_bytes;
			u16 wi;
			for (wi = 0; wi < bm_bytes; wi++)
				bm[wi] = 0;
			for (wi = 0; wi < win_count; wi++) {
				u16 node = ra_chain_ship_node[win_first + wi];
				const ra_chain_node_t *n = &ra_chain_nodes[node];
				u8  w = (u8)(((n->op >> 5) & 3) + 1);
				u32 v = ra_chain_values[node];
				u8  k;
				if (ra_chain_node_valid[node >> 3] & (1u << (node & 7)))
					bm[wi >> 3] |= (u8)(1u << (wi & 7));
				for (k = 0; k < w; k++)
					blob[k] = (u8)(v >> (8u * k));
				blob += w;
			}
		}
	}
	snap->chain_first    = win_first;
	snap->chain_count    = win_count;
	snap->chain_blob_len = (u16)blob_len;

	hdr->payload_len = (u16)(sizeof(ra_snapshot_header_t) + count
	                         + bm_bytes + blob_len);
	tx_len = sizeof(ra_gc_header_t) + sizeof(ra_snapshot_header_t) + count
	       + bm_bytes + blob_len;
	rx_len = sizeof(ra_snap_rx_buf);
	memset(ra_snap_rx_buf, 0, rx_len);

	{   /* v0.28.9 — time the Phase-B round-trip (write + wait + read). Cycle
	     * budget ≈ g_t_mem1_ms + g_t_snap_ms; wait is the slice inside it. */
		u32 _s0 = vi_read_hw_timer();
		ret = ra_send_phase_b(ra_snap_tx_buf, tx_len, ra_snap_rx_buf, rx_len);
		g_t_snap_ms = vi_ticks_to_us(vi_read_hw_timer() - _s0) / 1000u;
	}
	if (ret < 0) return ret;

	ra_parse_response(ra_snap_rx_buf);
	return 0;
}

/* Send ADDR_RESPONSE for the currently pending query — read each addr from
 * PPC RAM, build the response, ship via Phase B. The response from ESP may
 * carry the next ADDR_QUERY round (multi-pass convergence), a
 * WATCHLIST_APPEND, or a WATCHLIST_REMOVE — all handled by
 * ra_parse_response. */
static s32 ra_send_addr_response(void)
{
	ra_gc_header_t      *hdr;
	ra_addr_response_t  *ar;
	u16 count = ra_pending_query_count;
	u32 tx_len, rx_len;
	s32 ret;
	u16 i;
	u8 *values;

	if (count == 0) return 0;

	hdr = (ra_gc_header_t *)ra_snap_tx_buf;
	hdr->magic       = RA_MAGIC_GC_TO_ESP;
	hdr->command     = RA_CMD_ADDR_RESPONSE;
	hdr->payload_len = sizeof(ra_addr_response_t) + count;

	ar = (ra_addr_response_t *)(ra_snap_tx_buf + sizeof(ra_gc_header_t));
	ar->addr_count = count;

	values = ra_snap_tx_buf + sizeof(ra_gc_header_t)
	                        + sizeof(ra_addr_response_t);
#if RA_READ_LED_DIAG
	/* Freeze forensics: disc-slot LED ON = we are INSIDE a PPC read loop. If the
	 * galaxy freeze leaves the LED stuck ON, the Wii hung on ra_read_ppc_byte
	 * (a MEM read stall during the load); stuck OFF = it hung elsewhere (parse /
	 * EXI). 1 bit, but readable by eye at the freeze (no slow-mo). The WDOG-FREEZE
	 * already pinned this to the ADDR_RESPONSE read vs parse window. */
	if (!ra_led_celebrate) Swi_LedOn();
#endif
	for (i = 0; i < count; i++) {
		values[i] = ra_read_ppc_byte(ra_pending_query_addrs[i]);
	}
#if RA_READ_LED_DIAG
	if (!ra_led_celebrate) Swi_LedOff();
#endif

	tx_len = sizeof(ra_gc_header_t) + sizeof(ra_addr_response_t) + count;
	rx_len = sizeof(ra_snap_rx_buf);
	memset(ra_snap_rx_buf, 0, rx_len);

	ret = ra_send_phase_b(ra_snap_tx_buf, tx_len, ra_snap_rx_buf, rx_len);
	if (ret < 0) return ret;

	ra_pending_query_count = 0;  /* consumed */
	ra_parse_response(ra_snap_rx_buf);
	return 0;
}

/* (v0.26.2: ra_led_encode_count removed — the decimal blink encode of the
 * watchlist count burned ~13s of boot; the ESP serial log carries it.) */

static u32 ra_poll_thread(void *arg)
{
	(void)arg;

	/* LED language since v0.26.2 (the boot blink theater was removed):
	 * 1 blip = game detected, 2 blips = watchlist fetched,
	 * 7 fast blinks = fetch error, 3 pulses = achievement unlock. */

	/* WAIT FOR GAME-ALIVE (v0.26.2, unbounded since v0.27.1). The VBI
	 * counter at phys 0x2FF8 IS the boot callback: the booter zeroes it,
	 * and the game's first __VIRetraceHandler run (= the wiimote warning
	 * screen appearing) starts incrementing it via the C0 hook. Plain
	 * MEM1 reads with cache invalidate — no EXI, no SWI, perfectly safe
	 * while the apploader (or WiiFlow itself!) is running. We require 4
	 * observed changes (robust against the booter's zeroing of stale
	 * garbage counting as one).
	 *
	 * NO TIMEOUT, deliberately — the static-counter case means "this IOS
	 * instance is not hosting an RA game" and the module must stay
	 * SILENT FOREVER (zero EXI traffic). That single property is what
	 * makes it safe to install RAMOD into the STANDARD cIOS slots
	 * (248-251), including WiiFlow's own mainIOS: while WiiFlow browses,
	 * the counter never moves → we never touch the bus → no contention
	 * with WiiFlow's probe/LOAD_GAME ([[project-bus-contention-mainIOS]]
	 * solved structurally — no per-game .ini forcing needed). Boots
	 * without the ESP (WiiFlow doesn't inject the hook) and games where
	 * the hook pattern fails also stay silent — acceptable: RA simply
	 * isn't running for those. */
	{
		u32 prev, cur;
		s32 changes = 0;
		os_sync_before_read((void *)RA_VBI_COUNTER_PHYS, 4);
		prev = *(volatile u32 *)RA_VBI_COUNTER_PHYS;
		while (changes < 4) {
			ra_sleep(50);
			os_sync_before_read((void *)RA_VBI_COUNTER_PHYS, 4);
			cur = *(volatile u32 *)RA_VBI_COUNTER_PHYS;
			if (cur != prev) {
				changes++;
				prev = cur;
			}
		}
	}

	/* Single short blip: ra-module alive, game detected. */
	Swi_LedOn();  ra_sleep(100);
	Swi_LedOff();

	/* Initialize EXI subsystem. Just reads HW_EXICTRL to confirm bus master
	 * is on — actual transactions configure CSR per-call via exi_select. */
	exi_init();

	/* One IDENTIFY round-trip to confirm EXI link is healthy. */
	(void)ra_identify();
	ra_sleep(100);

	/* WATCHLIST FETCH. ESP32 is in state GAME_LOADED at this point —
	 * rcheevos has already loaded the achievement set and computed the
	 * addresses to watch. We pull them in chunks of up to 1024. */
	{
		s32 wl_ret = ra_fetch_watchlist();

		if (wl_ret < 0) {
			/* Fetch failed. 7 fast blinks = error (kept — this is the
			 * one signal worth reading off the LED in the field). */
			int k;
			for (k = 0; k < 7; k++) {
				Swi_LedOn();  ra_sleep(120);
				Swi_LedOff(); ra_sleep(120);
			}
		} else {
			/* Success — double blip and straight into the main loop.
			 * (v0.26.2: the old 5-blink + 1s marker + decimal LED count
			 * encode burned ~13 MORE seconds before the first SNAPSHOT;
			 * the count is visible in the ESP serial log anyway.) */
			Swi_LedOn();  ra_sleep(80);
			Swi_LedOff(); ra_sleep(80);
			Swi_LedOn();  ra_sleep(80);
			Swi_LedOff();

			/* Phase C: pull the chain descriptor table (once per game).
			 * Any failure or an empty table just leaves Phase C off —
			 * the legacy ADDR_QUERY path is unaffected. */
			ra_fetch_chain_table();
		}
	}

	/* (v0.27.0: the one-shot OCA boot dump was removed — it ran BEFORE the
	 * apploader finished, so its view was pre-boot residue; the periodic
	 * OCAP line in the 5s diag covers the steady state and stays.) */

	/* Main SNAPSHOT loop — VBI-edge scheduling with built-in 2-frame timer
	 * fallback (the only mode since v0.27.0; the HW_TIMER pacer and the
	 * fixed-16ms fallback builds were retired after the VBI lock was
	 * validated on hw). Multi-pass inner loop drains pending ADDR_QUERY
	 * rounds before computing next-frame timing; cap at 12 rounds matches
	 * ESP32's ADDR_QUERY_MAX_ITERATIONS. */

#define FRAME_US                16667u   /* 60Hz NTSC frame */

	{
		/* PPC VBI counter sync — the nes-ra-adapter hybrid pattern.
		 *
		 * WiiFlow's Ocarina codehandler executes our C0 cheat code at
		 * every vertical retrace; it increments a u32 at physical 0x2FF8
		 * through the PPC's uncached MEM1 mirror. We poll that address
		 * (plain MEM1 read, same path as ra_read_ppc_byte — no SWI) and
		 * fire on change. The counter value itself doubles as a frame
		 * sequence number; a jump > 1 between polls means we missed
		 * frames (e.g. multi-pass ran long).
		 *
		 * Fallback: if the counter hasn't moved for 2 frame-times, fire
		 * on time anyway. Covers: hook not installed (WiiFlow without RA,
		 * pattern-match failure on exotic games), loading screens that
		 * suspend the VI handler, and the window before the game boots.
		 * The fallback engages silently and disengages the moment the
		 * counter moves again. */
		#define VBI_POLL_MS          1u
		#define FALLBACK_FACTOR      2u

		/* The counter MUST be cache-invalidated before every read. Starlet
		 * maps PPC RAM cached; a tight poll of one address keeps its line
		 * pinned in the D-cache and we see the counter frozen for tens of
		 * ms until something happens to evict the line. (v0.22.2 field
		 * data: counter advanced at 59Hz — Δc≈295/5s — yet ~80% of fires
		 * were the 33ms fallback because the poll read stale values. The
		 * snapshot reads get away without syncing only because scanning
		 * ~900 scattered addresses thrashes the whole D-cache each frame.) */
		u32 last_ctr, ctr;
		u32 last_fire_tick;
		u32 fires = 0, vbi_fires = 0;
		u32 last_diag_tick;

		os_sync_before_read((void *)RA_VBI_COUNTER_PHYS, 4);
		last_ctr = *(volatile u32 *)RA_VBI_COUNTER_PHYS;
		last_fire_tick = vi_read_hw_timer();
		last_diag_tick = last_fire_tick;

		while (1) {
			s32 i;
			u32 now_tick, diff_us;
			s32 fire_now = 0;

#if RA_HEARTBEAT_LED
			/* Freeze forensic: toggle the disc-slot LED every 250ms of wall time
			 * AT THE TOP of the loop. Blinking at the freeze => the loop is still
			 * cycling (Wii alive); frozen => a true hang inside a fire. */
			{
				static u32 hb_last = 0;
				static u8  hb_on   = 0;
				u32 hb_now = vi_read_hw_timer();
				if (vi_ticks_to_us(hb_now - hb_last) >= 250000u) {
					hb_last = hb_now;
					hb_on   = !hb_on;
					if (!ra_led_celebrate) {
						if (hb_on) Swi_LedOn(); else Swi_LedOff();
					}
				}
			}
#endif

#if RA_INT_TIMEOUT_LED
			/* INT-timeout blip: solid LED for ~1s on any new exi_wait_int
			 * timeout. Detection reads the lifetime timeout counters (bumped
			 * inside ra_send_phase_b's WAIT phase); the off edge is wall-clock
			 * so this runs at the top of EVERY iteration (incl. the no-fire
			 * ra_sleep path) and never blocks. Back-to-back timeouts refresh
			 * the deadline → sustained ON reads as "INT failing often".
			 * Gated on !ra_led_celebrate so an unlock celebration wins the LED. */
			{
				u32 _to_total = g_to_phaseb + g_to_dbg;
				u32 _to_tick  = vi_read_hw_timer();
				if (_to_total != ra_led_to_seen) {
					ra_led_to_seen    = _to_total;
					ra_led_to_on      = 1;
					ra_led_to_on_tick = _to_tick;
					if (!ra_led_celebrate) Swi_LedOn();
				}
				if (ra_led_to_on &&
				    vi_ticks_to_us(_to_tick - ra_led_to_on_tick) >= 1000000u) {
					ra_led_to_on = 0;
					if (!ra_led_celebrate) Swi_LedOff();
				}
			}
#endif

			os_sync_before_read((void *)RA_VBI_COUNTER_PHYS, 4);
			ctr = *(volatile u32 *)RA_VBI_COUNTER_PHYS;
			now_tick = vi_read_hw_timer();
			diff_us = vi_ticks_to_us(now_tick - last_fire_tick);

			if (ctr != last_ctr) {
				last_ctr = ctr;
				fire_now = 1;
				vbi_fires++;
			} else if (diff_us > (FRAME_US * FALLBACK_FACTOR)) {
				fire_now = 1;
			}
			ra_game_frame = last_ctr;  /* true game-frame clock (v0.26.1) */

			if (!fire_now) {
				ra_sleep(VBI_POLL_MS);
				continue;
			}

			last_fire_tick = now_tick;
			fires++;

			{
#if RA_SPIKE_LOG
				u32 _ap_t0 = vi_read_hw_timer();
				u32 _pb0 = g_to_phaseb, _db0 = g_to_dbg;
				u32 _maxw;
#endif

				(void)ra_send_snapshot();
#if RA_SPIKE_LOG
				_maxw = g_t_wait_ms;
#endif
				for (i = 0; i < 12 && ra_pending_query_count > 0; i++) {
					(void)ra_send_addr_response();
#if RA_SPIKE_LOG
					if (g_t_wait_ms > _maxw) _maxw = g_t_wait_ms;
#endif
				}

#if RA_SPIKE_LOG
				/* Spike forensics: dump ONLY when this fire's convergence ran
				 * long (event-triggered, unlike the periodic PHB that misses
				 * spikes). ms=wall, r=addr-response rounds, w=longest single INT
				 * wait, tb=Phase B INT-wait timeouts, td=debug-ACK timeouts. A
				 * spike with small r but tb/td>0 (and w≈the 50ms cap) = the INT
				 * timeout is the cost, NOT deep convergence. */
				{
					u32 _ap_us = vi_ticks_to_us(vi_read_hw_timer() - _ap_t0);
					if (_ap_us > 30000u) {
						u8 _sm[40]; u8 _so = 0;
						_sm[_so++]='S';_sm[_so++]='P';_sm[_so++]='K';_sm[_so++]=' ';
						_sm[_so++]='m';_sm[_so++]='s';_sm[_so++]='=';
						_so += ra_dbg_u16_dec(_sm+_so,(u16)(_ap_us/1000u));
						_sm[_so++]=' ';_sm[_so++]='r';_sm[_so++]='=';
						_so += ra_dbg_u16_dec(_sm+_so,(u16)i);
						_sm[_so++]=' ';_sm[_so++]='w';_sm[_so++]='=';
						_so += ra_dbg_u16_dec(_sm+_so,(u16)_maxw);
						_sm[_so++]=' ';_sm[_so++]='t';_sm[_so++]='b';_sm[_so++]='=';
						_so += ra_dbg_u16_dec(_sm+_so,(u16)(g_to_phaseb-_pb0));
						_sm[_so++]=' ';_sm[_so++]='t';_sm[_so++]='d';_sm[_so++]='=';
						_so += ra_dbg_u16_dec(_sm+_so,(u16)(g_to_dbg-_db0));
						ra_debug_send(_sm,_so);
					}
				}
#endif
			}

			/* Achievement-unlock celebration on the disc-slot LED —
			 * one pattern step per fire, no sleeps. From 54: toggles
			 * at 45/36/27/18/9/0 → three 9-frame ON pulses, ends OFF.
			 * Swi_LedOn/Off are single SVCs on phase boundaries only. */
			if (ra_led_celebrate) {
				ra_led_celebrate--;
				if ((ra_led_celebrate % 9) == 0) {
					if ((ra_led_celebrate / 9) & 1)
						Swi_LedOn();
					else
						Swi_LedOff();
				}
#if RA_TROPHY_OVERLAY
				/* Mirror the LED phase to the PPC trophy flag EVERY frame
				 * (robustness — a single lost write no longer blanks a whole
				 * 9-frame window). on = current LED phase = ((cel+8)/9)&1.
				 * os_sync_after_write flushes the Starlet D-cache so the PPC's
				 * uncached read at 0xC0002FFC sees it.
				 *
				 * DIAG (2026-06-22): the trophy never appeared on hw though the
				 * LED blinked — Starlet->MEM1 writes are unproven (we only ever
				 * READ PPC RAM). At each LED boundary, read the flag back (with
				 * invalidate) and log "TRO w=.. rb=..": rb==w => the write
				 * reached Starlet DRAM (so it's a cross-CPU coherency issue);
				 * rb!=w => the low-MEM1 write itself isn't sticking. */
				{
					u32 on = ((u32)(ra_led_celebrate + 8) / 9) & 1;
					*(volatile u32 *)RA_TROPHY_FLAG_PHYS = on;
					os_sync_after_write((void *)RA_TROPHY_FLAG_PHYS, 4);
					/* Latch write + immediate read-back (Starlet side) for the
					 * 5s diag — no ra_debug_send here (would stall the burst). */
					os_sync_before_read((void *)RA_TROPHY_FLAG_PHYS, 4);
					g_tro_last_rb = *(volatile u32 *)RA_TROPHY_FLAG_PHYS;
					g_tro_last_w  = on;
					g_tro_writes++;
				}
#endif
			}

			/* Phase D2 in-game RESYNC — re-fetch the full watchlist via
			 * chunks (~300ms of legacy round-trips, one-time repair) and
			 * adopt the ESP's seq base. Frames during the fetch are simply
			 * skipped; the next SNAPSHOT echoes the new seq and the ESP
			 * resumes verified-sync evaluation. */
			if (ra_resync_pending) {
				ra_resync_pending = 0;
				if (ra_fetch_watchlist() == 0) {
					ra_wl_seq = ra_resync_seq;
				}
				/* On fetch failure ra_wl_seq stays stale — the ESP sees
				 * the unchanged echo and re-triggers the resync. */
			}

			/* Diag every ~5s: counter value + fires + how many came from
			 * the VBI edge (vs timer fallback). "VBI c=xxxxxxxx f=N v=M":
			 * v==f means perfect vblank lock; v==0 means hook not
			 * installed (running on fallback). */
			if (vi_ticks_to_us(now_tick - last_diag_tick) > 5000000u) {
				u8 m[48];
				u32 o = 0;
				u8 cb[4];
				last_diag_tick = now_tick;
				cb[0] = (ctr >> 24) & 0xFF; cb[1] = (ctr >> 16) & 0xFF;
				cb[2] = (ctr >>  8) & 0xFF; cb[3] =  ctr        & 0xFF;
				m[o++] = 'V'; m[o++] = 'B'; m[o++] = 'I'; m[o++] = ' ';
				m[o++] = 'c'; m[o++] = '=';
				o += ra_dbg_hex_bytes(m + o, cb, 4, 0);
				m[o++] = ' '; m[o++] = 'f'; m[o++] = '=';
				o += ra_dbg_u16_dec(m + o, (u16)fires);
				m[o++] = ' '; m[o++] = 'v'; m[o++] = '=';
				o += ra_dbg_u16_dec(m + o, (u16)vbi_fires);
				ra_debug_send(m, (u8)o);

				/* v0.28.9 — Phase-B timing (ms): w=wait(INT latency)
				 * m=mem1(read watchlist) s=snap(write+wait+read). The ESP
				 * cycle (cyc_us) ≈ m+s; if w≈s≈100ms the INT is the gap. */
				o = 0;
				m[o++] = 'P'; m[o++] = 'H'; m[o++] = 'B'; m[o++] = ' ';
				m[o++] = 'w'; m[o++] = '=';
				o += ra_dbg_u16_dec(m + o, (u16)g_t_wait_ms);
				m[o++] = ' '; m[o++] = 'm'; m[o++] = '=';
				o += ra_dbg_u16_dec(m + o, (u16)g_t_mem1_ms);
				m[o++] = ' '; m[o++] = 's'; m[o++] = '=';
				o += ra_dbg_u16_dec(m + o, (u16)g_t_snap_ms);
				ra_debug_send(m, (u8)o);

#if RA_TROPHY_OVERLAY
				/* Trophy-flag diag. n = flag writes since boot (n>0 means an
				 * unlock celebration ran → the build is fresh). w/rb = last
				 * value written / Starlet read-back. rb==w => the write reached
				 * Starlet DRAM (cross-CPU issue then); rb!=w => the MEM1 write
				 * itself isn't sticking. */
				o = 0;
				m[o++] = 'T'; m[o++] = 'R'; m[o++] = 'O'; m[o++] = ' ';
				m[o++] = 'n'; m[o++] = '=';
				o += ra_dbg_u16_dec(m + o, (u16)g_tro_writes);
				m[o++] = ' '; m[o++] = 'w'; m[o++] = '=';
				o += ra_dbg_u16_dec(m + o, (u16)g_tro_last_w);
				m[o++] = ' '; m[o++] = 'r'; m[o++] = 'b'; m[o++] = '=';
				o += ra_dbg_u16_dec(m + o, (u16)g_tro_last_rb);
				ra_debug_send(m, (u8)o);
#endif

				/* OCAP: live peeks at the Ocarina chain landmarks. Unlike
				 * the one-shot OCA dump above (which runs BEFORE the booter's
				 * apploader finishes, so it sees pre-boot residue), this
				 * shows the steady state:
				 *   word 0 @0x1800 — Disc_ID start (game ID ASCII)
				 *   word 1 @0x22A8 — codelist, debugger=0 (00D0C0DE = GCT)
				 *   word 2 @0x28B8 — codelist, debugger=1
				 *   word 3 @0x2FE0 — booter breadcrumb 5242hhpp
				 *                    ('RB' | hooktype | hookpatched)
				 *   word 4 @0x2FE4 — GCT size the booter received */
				{
					u8 m2[64];
					u32 o2 = 0, d2, j2;
					u32 oca2[5];
					oca2[0] = 0x1800; oca2[1] = 0x22A8; oca2[2] = 0x28B8;
					oca2[3] = 0x2FE0; oca2[4] = 0x2FE4;
					m2[o2++] = 'O'; m2[o2++] = 'C'; m2[o2++] = 'A'; m2[o2++] = 'P';
					for (d2 = 0; d2 < 5; d2++) {
						m2[o2++] = ' ';
						os_sync_before_read((void *)oca2[d2], 4);
						for (j2 = 0; j2 < 4; j2++) {
							u8 b2 = *(volatile u8 *)(oca2[d2] + j2);
							o2 += ra_dbg_hex_bytes(m2 + o2, &b2, 1, 0);
						}
					}
					ra_debug_send(m2, (u8)o2);
				}
			}
		}
	}
	return 0;
}

/* ------------------------------------------------------------------------
 * IPC scaffolding — thin debug surface
 * ------------------------------------------------------------------------ */

static s32 __RA_Ioctlv(u32 cmd, ioctlv *vector, u32 inlen, u32 iolen)
{
	s32 ret = IPC_ENOENT;
	InvalidateVector(vector, inlen, iolen);

	switch (cmd) {
	case RA_CMD_PING:
		ret = esp_present ? 1 : 0;
		break;
	default:
		break;
	}

	FlushVector(vector, inlen, iolen);
	return ret;
}

static s32 __RA_Initialize(u32 *queuehandle)
{
	/* Heap space for Mem_Init */
	static u32 heapspace[0x100] ATTRIBUTE_ALIGN(32); /* 1 KB */
	void *buffer;
	s32   ret;

	ret = Mem_Init(heapspace, sizeof(heapspace));
	if (ret < 0) return ret;

	ret = Timer_Init();
	if (ret < 0) return ret;

	buffer = Mem_Alloc(0x20);
	if (!buffer) return IPC_ENOMEM;

	ret = os_message_queue_create(buffer, 8);
	if (ret < 0) return ret;

	os_device_register(DEVICE_NAME, ret);
	*queuehandle = ret;
	return 0;
}

int main(void)
{
	/* Initialized: if __RA_Initialize fails before assigning it, the IPC
	 * loop below blocks on queue 0 instead of reading garbage (also
	 * silences the long-standing may-be-used-uninitialized warning). */
	u32 queuehandle = 0;
	s32 ret;
	s32 thread_id;
	s32 priority;

	svc_write("$IOSVersion: RA-HELLO3: " __DATE__ " " __TIME__ " $\n");

	/* PROOF-OF-LIFE: turn the disc-slot LED ON before doing anything else.
	 * If the LED lights up when a game boots through the RA slot, we know:
	 *   - The cIOS reloaded into the slot
	 *   - ra-module's main() executed
	 *   - MLOAD's SWI table is installed and Swi_LedOn() actually fires
	 * If it doesn't light, the module isn't even being entered. */
	Swi_LedOn();

	ret = __RA_Initialize(&queuehandle);
	if (ret < 0) {
		svc_write("[RA] Initialization failed!\n");
		/* Fall through to IPC loop anyway — kernel needs us to BLOCK on a
		 * syscall, not busy-loop. */
	}

	/* NB: exi_init() is now called from inside the poll thread after the
	 * grace period — see ra_poll_thread. Doing it here (module init time)
	 * caused IOS_ReloadIOS to hang on Hollywood register writes. */

	/* Turn LED OFF before creating the thread. This lets us distinguish:
	 *   - LED stays OFF after thread create → thread didn't run
	 *   - LED toggles → thread ran (it'll Swi_LedOn() in its first iter)
	 * Without this, we can't tell if main's Swi_LedOn flash is what's
	 * lighting up the LED, or if the thread is doing it. */
	Swi_LedOff();

	/* Poll-thread priority. main is born at RA_POLL_PRIORITY (start.s) ONLY so
	 * it can spawn the poll thread that high — IOS won't let a child exceed its
	 * parent. We default to RA_POLL_PRIORITY but fall back to inheriting if it
	 * was left 0. */
	priority = os_thread_get_priority(os_get_thread_id());
#if RA_POLL_PRIORITY
	priority = RA_POLL_PRIORITY;
#endif

	/* Create poll thread in paused state (autostart = 0). */
	thread_id = os_thread_create(ra_poll_thread, NULL,
	                             poll_thread_stack + sizeof(poll_thread_stack),
	                             sizeof(poll_thread_stack), priority, 0);

	if (thread_id >= 0) {
		os_thread_continue(thread_id);
	} else {
		/* Create failed — leave LED OFF as our signal. */
	}

	/* Drop the MAIN/IPC thread back to the I/O-driver tier (0x48). Running the
	 * IPC thread above 0x48 preempts DI/ES/FFS/USB during boot and freezes the
	 * VBI hook (field-proven 2026-06-24). Lowering is always permitted; the poll
	 * thread keeps the 0x50 it was created with. Do this BEFORE the IPC loop so
	 * the high-priority window is just the spawn above. */
	os_thread_set_priority(os_get_thread_id(), RA_MAIN_PRIORITY);

	/* IPC Loop: Block forever on the queue waiting for open/close/ioctl requests. */
	while (1) {
		ipcmessage *message = NULL;
		ret = os_message_queue_receive(queuehandle, (void *)&message, 0);
		if (ret) continue;

		switch (message->command) {
		case IOS_OPEN:
			ret = (!strcmp(message->open.device, DEVICE_NAME))
			      ? message->open.resultfd
			      : IPC_ENOENT;
			break;

		case IOS_CLOSE:
			ret = 0;
			break;

		case IOS_IOCTLV:
			ret = __RA_Ioctlv(message->ioctlv.command,
			                  message->ioctlv.vector,
			                  message->ioctlv.num_in,
			                  message->ioctlv.num_io);
			break;

		default:
			ret = IPC_EINVAL;
		}

		os_message_queue_ack(message, ret);
	}

	return 0;
}
