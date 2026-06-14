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
#include "led.h"
#include "gc_ra_protocol.h"

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
#define RA_EXI_FREQ  EXI_SPEED8MHZ

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
 *   ra_esp_header_t (6) + ra_watchlist_chunk_t (6) + 1024 addrs × 4 = 4108 */
static u8  ra_chunk_rx_buf[6 + 6 + RA_WATCHLIST_CHUNK_ADDRS * 4]
                          __attribute__((aligned(32)));

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

static u8 ra_dbg_hex(u8 nibble)
{
	if (nibble < 10) return (u8)('0' + nibble);
	return (u8)('A' + (nibble - 10));
}

static u32 ra_dbg_hex_bytes(u8 *dst, const u8 *src, u32 n, u8 sep)
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

static u32 ra_dbg_u16_dec(u8 *dst, u16 v)
{
	u8 tmp[6];
	u32 n = 0, o = 0;
	if (v == 0) { dst[0] = '0'; return 1; }
	while (v > 0) { tmp[n++] = (u8)('0' + (v % 10)); v /= 10; }
	while (n > 0) dst[o++] = tmp[--n];
	return o;
}

static void ra_debug_send(const u8 *msg, u8 msg_len)
{
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
	 * armed and ready. 100ms timeout is the same budget as Phase B itself. */
	(void)exi_wait_int(RA_EXI_CHAN, 100);
	exi_clear_int(RA_EXI_CHAN);
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
static s32 ra_send_phase_b(const void *tx, u32 tx_len, void *rx, u32 rx_len)
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

	/* WRITE phase — one CS-low, write only. */
	ret = exi_select(RA_EXI_CHAN, RA_EXI_DEV, RA_EXI_FREQ);
	if (ret < 0) return ret;
	if (tx && tx_len) {
		ret = exi_imm_write(RA_EXI_CHAN, tx, tx_len);
		if (ret < 0) { exi_deselect(RA_EXI_CHAN); return ret; }
	}
	exi_deselect(RA_EXI_CHAN);

	/* WAIT phase — ESP processes + arms + asserts INT.
	 * 100 ms timeout: a healthy round trip is sub-ms; 100 ms is "the ESP
	 * is hung, give up". The CSR latch is sticky so we cannot miss the
	 * signal — only a complete absence of INT assertion can time out. */
	ret = exi_wait_int(RA_EXI_CHAN, 100);
	if (ret < 0) return ret;

	/* CLEAR phase — RW1C the EXI_IRQ bit so the next round's wait starts
	 * from a clean slate. Must happen BEFORE the read CS-low so we don't
	 * race a second assertion (shouldn't happen given our serial protocol,
	 * but cheap insurance). */
	exi_clear_int(RA_EXI_CHAN);

	/* SETTLE phase — ESP asserts INT immediately after spi_slave_queue_trans
	 * returns, but ESP-IDF's SPI slave driver may not have fully configured
	 * DMA yet (the driver task wakes from the queue and sets up DMA
	 * descriptors asynchronously). Without this delay, ra-module opens the
	 * read CS-low before DMA is armed → slave outputs idle MISO (all 0xFF)
	 * instead of tx_buf. Empirically reproduces for response_len >= 520
	 * bytes (ADDR_QUERY for 128 addrs); does NOT reproduce for the 24-byte
	 * ADDR_QUERY for 4 addrs in frame 1. Likely because the larger memcpy
	 * inside the SPI driver task takes more time to set up DMA.
	 *
	 * 1ms is generous — even worst-case DMA setup is <100μs. ra_sleep
	 * blocks via the IOS message-queue timer so it's a real sleep, not a
	 * busy spin. Cost: ~1ms per Phase B round = negligible. */
	ra_sleep(1);

	/* READ phase — second CS-low, read only. ESP's tx_buf is now armed
	 * with the freshly prepared response. */
	ret = exi_select(RA_EXI_CHAN, RA_EXI_DEV, RA_EXI_FREQ);
	if (ret < 0) return ret;
	if (rx && rx_len) {
		ret = exi_imm_read(RA_EXI_CHAN, rx, rx_len);
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
 * Two-transaction protocol due to ESP32's response timing: when we send
 * GET_CHUNK in transaction T1, ESP32 sees the request, processes it in its
 * main loop, and prepares the response for T2. T1's TX buffer holds the
 * PREVIOUS response (e.g. an IDENTIFY ack). So we always send the request
 * TWICE — the second send discards its own stale-response read and gets
 * the prepared chunk data. */
static s32 ra_fetch_watchlist_chunk(u16 chunk_index)
{
	struct __attribute__((packed)) {
		ra_gc_header_t            hdr;
		ra_watchlist_chunk_req_t  req;
	} tx;
	ra_esp_header_t       resp;
	ra_watchlist_chunk_t  chunk;
	u8  discard[16];
	u16 i;
	s32 ret;

	tx.hdr.magic       = RA_MAGIC_GC_TO_ESP;
	tx.hdr.command     = RA_CMD_GET_WATCHLIST_CHUNK;
	tx.hdr.payload_len = sizeof(ra_watchlist_chunk_req_t);
	tx.req.chunk_index = chunk_index;

	/* T1 — tell ESP32 to prepare chunk N. Read a small buffer (we don't
	 * care about its content; it's the previous command's response). */
	ret = ra_send(&tx, sizeof(tx), discard, sizeof(discard));
	if (ret < 0) return ret;

	/* Give ESP32 main loop a moment to process and prepare the response.
	 * The 50ms sleep is conservative — ESP32 typically processes in a few
	 * hundred µs once the SPI ISR signals — but extra padding here costs
	 * nothing and avoids races. */
	ra_sleep(50);

	/* T2 — request the SAME chunk again, this time with the full receive
	 * buffer. The TX buffer on ESP32 now holds the chunk response prepared
	 * after T1. */
	memset(ra_chunk_rx_buf, 0, sizeof(ra_chunk_rx_buf));
	ret = ra_send(&tx, sizeof(tx), ra_chunk_rx_buf, sizeof(ra_chunk_rx_buf));
	if (ret < 0) return ret;

	memcpy(&resp, ra_chunk_rx_buf, sizeof(resp));
	if (resp.magic != RA_MAGIC_ESP_TO_GC) return -100;

	memcpy(&chunk, ra_chunk_rx_buf + sizeof(ra_esp_header_t), sizeof(chunk));
	if (chunk.addr_count > RA_WATCHLIST_CHUNK_ADDRS) return -101;
	if (chunk.chunk_index != chunk_index)            return -102;

	{
		const u8 *src = ra_chunk_rx_buf
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
 * the ESP32 side's ADDR_QUERY_MAX (currently 128). */
#define RA_ADDR_QUERY_MAX  128

/* Pending ADDR_QUERY state, populated by ra_parse_response when the
 * response carries an ADDR_QUERY event. Consumed by ra_send_addr_response. */
static u32 ra_pending_query_addrs[RA_ADDR_QUERY_MAX] __attribute__((aligned(32)));
static u16 ra_pending_query_count = 0;

/* Shared TX buffer — large enough for the worst-case SNAPSHOT
 * (header + max watchlist values) and reused for ADDR_RESPONSE writes
 * (which always pad to SNAPSHOT_REQ_LEN). Aligned 32 in case future DMA. */
static u8 ra_snap_tx_buf[16 + RA_MAX_WATCH_ADDRS] __attribute__((aligned(32)));

/* RX buffer — sized to comfortably hold the ESP32's response. ESP32 pads
 * the response with `last_snap_req_len` bytes of 0xFF that are consumed
 * during our TX phase; the actual ra_esp_header_t lands at offset 0 of
 * what we read during the RX phase. The longest legitimate response is
 * a WATCHLIST_APPEND for ADDR_QUERY_MAX addrs: 6 (esp_hdr) + 2 (wa_hdr)
 * + 128*4 = 520 bytes. 1024 gives generous headroom. */
static u8 ra_snap_rx_buf[1024] __attribute__((aligned(32)));

/* Tracks the SNAPSHOT request length so ADDR_RESPONSE pads to the same
 * width. Updated every SNAPSHOT cycle since watchlist_count can grow
 * (WATCHLIST_APPEND) or shrink (WATCHLIST_TRUNCATE) mid-session. */
static u32 ra_canonical_tx_len = 0;

/* Monotonic per-frame counter so ESP32 can detect dropped frames. */
static u32 ra_frame_counter = 0;

/* Wii Starlet ARM can read PPC RAM directly: MEM1 at physical 0x00xxxxxx
 * and MEM2 at 0x10xxxxxx are mapped into Starlet's address space. The
 * ESP32 puts raw PPC physical offsets in the watchlist (0x008C2760 etc),
 * so a direct dereference suffices. If Starlet's D-cache happened to
 * hold stale PPC bytes after a long idle, an explicit invalidate would
 * be needed — but empirically the .elf.orig (which used the same direct
 * read) never showed stale data, so we don't bother. */
static u8 ra_read_ppc_byte(u32 addr)
{
	return *(volatile u8 *)addr;
}

/* Central response parser — called by BOTH ra_send_snapshot and
 * ra_send_addr_response after their ra_send returns. Mutates global
 * state (ra_pending_query_*, ra_watchlist*) as the event dictates.
 *
 * The response is at offset 0 of rx_buf because ESP32 padded its
 * prepared response with `last_snap_req_len` bytes of 0xFF that got
 * clocked through during our TX phase — by the time we begin reading
 * RX bytes, we're at the start of the actual response data. */
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
		}
	}
	else if (resp.event_type == RA_EVT_WATCHLIST_APPEND) {
		/* No dedup here — ESP32 (v0.17.1+) has mismatch_last_sent_count
		 * dup-suppression so duplicates never reach us. Adding a dedup
		 * here caused sync drift with REMOVE: rejected entries left
		 * ra_watchlist_count behind ESP32's view, then REMOVE for the
		 * "tail" couldn't find some addresses, ra-module ended at a
		 * different count than expected, canonical padding desynced,
		 * multi-pass died (observed v0.18 2026-06-01). Trust ESP32. */
		ra_watchlist_append_t wa;
		memcpy(&wa, buf + sizeof(resp), sizeof(wa));
		if (wa.addr_count > 0
		    && (u32)ra_watchlist_count + wa.addr_count <= RA_MAX_WATCH_ADDRS) {
			const u8 *src = buf + sizeof(resp) + sizeof(wa);
			u16 j;
			for (j = 0; j < wa.addr_count; j++) {
				u32 addr;
				memcpy(&addr, src + j * 4, 4);
				ra_watchlist[ra_watchlist_count + j] = addr;
			}
			ra_watchlist_count += wa.addr_count;
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
	else if (resp.event_type == RA_EVT_WATCHLIST_TRUNCATE) {
		/* Deprecated event (Phase A v0.17.0) — kept parsed for backward
		 * compat. Phase A.5 uses RA_EVT_WATCHLIST_REMOVE instead. */
		ra_watchlist_truncate_t wt;
		memcpy(&wt, buf + sizeof(resp), sizeof(wt));
		if (wt.keep_count <= ra_watchlist_count) {
			ra_watchlist_count = wt.keep_count;
		}
	}
	else if (resp.event_type == RA_EVT_WATCHLIST_REMOVE) {
		/* Phase A.5: ESP32 evicted N specific addresses (LRU). For each,
		 * find it in ra_watchlist[] (linear search) and shift the survivors
		 * down. Insertion order of survivors preserved → matches ESP32's
		 * post-defrag layout, so SNAPSHOT positions stay aligned.
		 *
		 * Implementation: build a "to_remove" bitmap (small, 1024 bits),
		 * then single-pass compact. O(N + R*N) where N=watch_count,
		 * R=remove_count. Fine for N≤1024, R≤256. */
		ra_watchlist_remove_t wr;
		memcpy(&wr, buf + sizeof(resp), sizeof(wr));
		if (wr.addr_count > 0
		    && wr.addr_count <= ra_watchlist_count) {
			const u8 *src = buf
			              + sizeof(resp) + sizeof(wr);
			u16 r, i;
			static u8 to_remove[RA_MAX_WATCH_ADDRS];  /* one byte per entry */
			memset(to_remove, 0, ra_watchlist_count);
			for (r = 0; r < wr.addr_count; r++) {
				u32 addr;
				memcpy(&addr, src + r * 4, 4);
				for (i = 0; i < ra_watchlist_count; i++) {
					if (ra_watchlist[i] == addr && !to_remove[i]) {
						to_remove[i] = 1;
						break;
					}
				}
			}
			/* Compact: shift survivors down. */
			u16 write_idx = 0;
			for (i = 0; i < ra_watchlist_count; i++) {
				if (!to_remove[i]) {
					if (write_idx != i) {
						ra_watchlist[write_idx] = ra_watchlist[i];
					}
					write_idx++;
				}
			}
			ra_watchlist_count = write_idx;
		}
	}
}

/* Send SNAPSHOT — read all watchlist values from PPC RAM and ship them.
 *
 * Phase B (INT-handshake) smoke test: this path uses ra_send_phase_b which
 * splits write and read into two CS-low pulses gated by the ESP's INT
 * assertion. The 1-tx arm-after-prepare delay is gone, so the ESP's
 * response can be variable-length (no canonical padding needed) — we just
 * read enough to cover the header, then the declared payload.
 *
 * ra_canonical_tx_len is no longer used for SNAPSHOT but is kept updated
 * so ra_send_addr_response (still on legacy single-CS-low Phase A path)
 * continues to pad correctly. Once the Phase B smoke test confirms the
 * SNAPSHOT path, ra_send_addr_response will be converted next and the
 * canonical-padding bookkeeping deleted entirely. */
static s32 ra_send_snapshot(void)
{
	ra_gc_header_t        *hdr;
	ra_snapshot_header_t  *snap;
	u16 count = ra_watchlist_count;
	u32 tx_len, rx_len;
	s32 ret;
	u16 i;
	u8 *values;

	hdr = (ra_gc_header_t *)ra_snap_tx_buf;
	hdr->magic       = RA_MAGIC_GC_TO_ESP;
	hdr->command     = RA_CMD_SNAPSHOT;
	hdr->payload_len = sizeof(ra_snapshot_header_t) + count;

	snap = (ra_snapshot_header_t *)(ra_snap_tx_buf + sizeof(ra_gc_header_t));
	snap->frame_counter = ++ra_frame_counter;
	snap->addr_count    = count;

	values = ra_snap_tx_buf + sizeof(ra_gc_header_t)
	                        + sizeof(ra_snapshot_header_t);
	for (i = 0; i < count; i++) {
		values[i] = ra_read_ppc_byte(ra_watchlist[i]);
	}

	tx_len = sizeof(ra_gc_header_t) + sizeof(ra_snapshot_header_t) + count;
	ra_canonical_tx_len = tx_len;  /* still consumed by ra_send_addr_response */

	rx_len = sizeof(ra_snap_rx_buf);
	memset(ra_snap_rx_buf, 0, rx_len);

	ret = ra_send_phase_b(ra_snap_tx_buf, tx_len, ra_snap_rx_buf, rx_len);
	if (ret < 0) return ret;

	ra_parse_response(ra_snap_rx_buf);
	return 0;
}

/* Send ADDR_RESPONSE for the currently pending query — read each addr
 * from PPC RAM, build the response, ship. The response from ESP32 may
 * carry the next ADDR_QUERY round (multi-pass convergence) or
 * WATCHLIST_APPEND/REMOVE — handled by ra_parse_response.
 *
 * Phase B: uses ra_send_phase_b (INT-handshake) so there's no more
 * arm-after-prepare 1-tx delay. Canonical padding is no longer required
 * for correctness here either, but we still pad to ra_canonical_tx_len
 * because the ESP firmware's old send_padded_response path may still be
 * length-sensitive in transient code paths during Phase B rollout. Once
 * the ESP firmware drops canonical padding, this memset goes away.
 *
 * Pre-burst sleep is gone — the race it patched (ESP's "no descriptor
 * queued" window between two back-to-back writes) is impossible under
 * Phase B because the WAIT phase blocks until the ESP has explicitly
 * armed and asserted INT. No descriptor-not-ready window exists. */
static s32 ra_send_addr_response(void)
{
	ra_gc_header_t      *hdr;
	ra_addr_response_t  *ar;
	u16 count = ra_pending_query_count;
	u32 actual_len, tx_len, rx_len;
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
	for (i = 0; i < count; i++) {
		values[i] = ra_read_ppc_byte(ra_pending_query_addrs[i]);
	}

	actual_len = sizeof(ra_gc_header_t) + sizeof(ra_addr_response_t) + count;
	tx_len = (actual_len > ra_canonical_tx_len) ? actual_len
	                                            : ra_canonical_tx_len;
	if (tx_len > actual_len) {
		memset(ra_snap_tx_buf + actual_len, 0xFF, tx_len - actual_len);
	}

	rx_len = sizeof(ra_snap_rx_buf);
	memset(ra_snap_rx_buf, 0, rx_len);

	ret = ra_send_phase_b(ra_snap_tx_buf, tx_len, ra_snap_rx_buf, rx_len);
	if (ret < 0) return ret;

	ra_pending_query_count = 0;  /* consumed */
	ra_parse_response(ra_snap_rx_buf);
	return 0;
}

/* Encode a decimal count as digit blinks (hundreds, tens, ones).
 * Each digit: N medium blinks (250ms each), 800ms separator between digits.
 * Digit 0 shows as a brief blip (50ms) so the user sees "something here". */
static void ra_led_encode_count(u32 count)
{
	int digits[3];
	int d, k;

	digits[0] = (count / 100) % 10;
	digits[1] = (count /  10) % 10;
	digits[2] =  count        % 10;

	for (d = 0; d < 3; d++) {
		if (digits[d] == 0) {
			/* Tiny blip to make "zero digit" visible. */
			Swi_LedOn();  ra_sleep(50);
			Swi_LedOff(); ra_sleep(50);
		} else {
			for (k = 0; k < digits[d]; k++) {
				Swi_LedOn();  ra_sleep(250);
				Swi_LedOff(); ra_sleep(250);
			}
		}
		ra_sleep(800);  /* digit separator */
	}
}

static u32 ra_poll_thread(void *arg)
{
	(void)arg;

	/* Instrumented version — VISIBLE blink markers around each step so we
	 * can see exactly where the thread is in its execution by watching the
	 * disc-slot LED. Markers are 4 fast blinks at ~3Hz between phases.
	 * gcc 4.5.1 (devkitARM r32) is C89 — declare loop var at block top. */
	s32 i;

	/* GRACE: 10 slow blinks (~10s). After the 2026-05-29 SWI breakthrough,
	 * writes to Hollywood registers no longer crash the kernel during boot,
	 * so we can shorten this. Keep some delay to let the game settle and
	 * give the user a chance to see the LED come alive. */
	for (i = 0; i < 10; i++) {
		Swi_LedOn();  ra_sleep(500);
		Swi_LedOff(); ra_sleep(500);
	}

	/* Initialize EXI subsystem. Just reads HW_EXICTRL to confirm bus master
	 * is on — actual transactions configure CSR per-call via exi_select. */
	exi_init();

	/* PROGRAM-START MARKER: 2-second solid ON. After this, the loop runs
	 * forever sending IDENTIFY once per second. */
	Swi_LedOn();
	ra_sleep(2000);
	Swi_LedOff();
	ra_sleep(500);

	/* One IDENTIFY round-trip to confirm EXI link is healthy. */
	(void)ra_identify();
	ra_sleep(500);

	/* WATCHLIST FETCH. ESP32 is in state GAME_LOADED at this point —
	 * rcheevos has already loaded the achievement set and computed the
	 * addresses to watch. We pull them in chunks of up to 1024. */
	{
		s32 wl_ret = ra_fetch_watchlist();

		if (wl_ret < 0) {
			/* Fetch failed. 7 fast blinks = error. */
			int k;
			for (k = 0; k < 7; k++) {
				Swi_LedOn();  ra_sleep(120);
				Swi_LedOff(); ra_sleep(120);
			}
			ra_sleep(1000);
		} else if (ra_watchlist_count == 0) {
			/* Got the chunk but it was empty. 2s solid ON = "0 addrs". */
			Swi_LedOn();
			ra_sleep(2000);
			Swi_LedOff();
			ra_sleep(500);
		} else {
			/* Success! 5 fast blinks ("got it") + 1 long marker + count
			 * encoded as decimal digits. */
			int k;
			for (k = 0; k < 5; k++) {
				Swi_LedOn();  ra_sleep(120);
				Swi_LedOff(); ra_sleep(120);
			}
			ra_sleep(800);
			/* 1 long ON (1s) = "here comes the count". */
			Swi_LedOn();  ra_sleep(1000);
			Swi_LedOff(); ra_sleep(500);
			ra_led_encode_count(ra_watchlist_count);
			ra_sleep(1000);
		}
	}

	/* Main SNAPSHOT loop — at ~60Hz target, send a SNAPSHOT, then drain
	 * any ADDR_QUERY rounds before sleeping until the next frame. The
	 * multi-pass inner loop converges within the 16ms budget for typical
	 * cheevo workloads (1-3 rounds observed on Kirby RtDL). Iteration cap
	 * defends against pathological cheevos that never converge. */
	while (1) {
		s32 i;

		(void)ra_send_snapshot();

		/* Multi-pass: keep responding to ADDR_QUERY events until the
		 * ESP32 stops asking (response is ACK / WATCHLIST_APPEND /
		 * WATCHLIST_TRUNCATE / NONE). Cap at 12 rounds — matches ESP32's
		 * ADDR_QUERY_MAX_ITERATIONS so we don't deadlock if convergence
		 * never reaches new_missing=0 due to a bug. */
		for (i = 0; i < 12 && ra_pending_query_count > 0; i++) {
			(void)ra_send_addr_response();
		}

		/* 16ms = ~60Hz target cadence (matches typical PPC game vblank rate).
		 *
		 * Phase B + SETTLE math at steady state (chain resolved, SNAPSHOT
		 * → ACK only):
		 *   ~0.5ms write + ~1ms wait_int + 1ms SETTLE + ~0.5ms read = ~3ms
		 *   inter-trans overhead, plus ra_parse_response ≈ 0.1ms.
		 *   16ms sleep + 3ms work = ~19ms → ~52Hz effective.
		 *
		 * During multi-pass (chain discovery or post-eviction resync), each
		 * extra ADDR_RESP round adds another ~3ms, eating into the sleep
		 * budget. Worst case observed in v0.19.2 boot: 3 multi-pass rounds
		 * = 12ms of EXI work, leaves 4ms of sleep before next SNAPSHOT.
		 * Steady-state recovers in ~1-2 frames once chain is fully cached.
		 *
		 * History:
		 *   - v0.17.2: ra_sleep(33) for ~30Hz (Phase A throttled).
		 *   - v0.19.0-2: ra_sleep(2000) for debug visibility at 0.5Hz.
		 *   - v0.19.3: ra_sleep(16) for ~60Hz (this version).
		 *
		 * If we ever need stricter 60Hz pacing, compute remaining budget
		 * from the elapsed time since SNAPSHOT start and sleep accordingly.
		 * For now a fixed 16ms gives consistent ESP-side breathing room. */
		ra_sleep(16);
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
	u32 queuehandle;
	s32 ret;
	s32 thread_id;
	s32 priority;

	svc_write("$IOSVersion: RA-HELLO3: " __DATE__ " " __TIME__ " $\n");

	/* PROOF-OF-LIFE: turn the disc-slot LED ON before doing anything else.
	 * If the LED lights up when a game boots through slot 247, we know:
	 *   - The cIOS reloaded into slot 247
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

	/* Inherit main thread priority instead of hardcoding 0x79. Same pattern
	 * as fat-module/led.c — that module's LED blink thread runs at the
	 * parent's priority so it always gets scheduled when main is blocked.
	 * Our main runs at 0x48 (from start.s). Hardcoding 0x79 may have been
	 * starving us. */
	priority = os_thread_get_priority(os_get_thread_id());

	/* Create poll thread in paused state (autostart = 0). */
	thread_id = os_thread_create(ra_poll_thread, NULL,
	                             poll_thread_stack + sizeof(poll_thread_stack),
	                             sizeof(poll_thread_stack), priority, 0);

	if (thread_id >= 0) {
		os_thread_continue(thread_id);
	} else {
		/* Create failed — leave LED OFF as our signal. */
	}

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
