/*
 * usbprobe.c — USB transport SPIKE, Wii side (d2x cIOS, RAMOD slot).
 *
 * Built instead of the RA module when RA_BUILD_USB_PROBE is 1 (ra_build.h).
 * Times bulk transfers from inside the cIOS, with a game running, against the
 * ESP32-S3 echo firmware (c:/dev/wii/usb-spike-esp, VID:PID 303A:4052).
 * Plan, GameCube results and the reasoning behind each mode:
 * c:/dev/wii/PLAN-USB-SPIKE.md (§9, §10). Counterpart of the Nintendont
 * kernel probe (Nintendont kernel/ra_usb_probe.c), same wire protocol.
 *
 * Questions this answers for the Wii:
 *   1. Can a cIOS module reach the ESP at all, and through which API?
 *        base 58         -> /dev/usb/ven (USBV5). Single client: if the d2x
 *                           USBS module already serves a USB drive, the open
 *                           fails with -4 (LED code 2).
 *        bases 38/56/57  -> /dev/usb/oh0/303a/4052 (USBV0/V4). The d2x EHCI
 *                           module hands full-speed ports to the OHCI
 *                           companion, so the ESP lives on a stack separate
 *                           from the USB drive.
 *   2. Does the "stream" result from the GameCube hold here (two OUTs in
 *      flight: ~50 us of thread time per 60 Hz frame, ~850 B/ms)?
 *   3. The production shape never measured before: stream OUTs PLUS one IN
 *      that stays posted for the ESP to answer when it has something
 *      ("prod" mode; the ESP answers every 32nd request).
 *
 * Reporting: every line goes to the ESP console as a LOG request ("USBW ..."
 * prefix). Before the link works there is no console, so failures show on
 * the disc-slot LED instead: N blinks, pause, repeat.
 *   2 = /dev/usb/ven taken (d2x USBS serves a USB drive)
 *   3 = device list answered, no ESP in it, no fallback id answered
 *   4 = no usable USB API (or ven version != 0x50001)
 *   5 = link opened but the first exchange failed
 *   6 = GETDEVICECHANGE rejected
 *   7 = GETDEVICECHANGE never answered (30 s)
 * First run (base 58, game on SD) gave 3 — hence the fallback ids and the
 * discovery report in usbp_connect / usbp_thread.
 *
 * gcc 4.5.1 / C89: declarations at the top of each block.
 */

#include <string.h>
#include <stdarg.h>

#include "ios.h"
#include "ipc.h"
#include "mem.h"
#include "module.h"
#include "syscalls.h"
#include "types.h"
#include "swi_mload.h"

#include "vi.h"
#include "exi.h"
#include "gc_ra_protocol.h"
#include "ra_build.h"

#if RA_BUILD_USB_PROBE

/* cios-lib/vsprintf.c (not declared in its header). */
int vsprintf(char *buf, const char *fmt, va_list args);

char *moduleName = "RA_USBP";

/* ---- device + wire protocol (mirror of Nintendont RAUsbProbe.h) --------- */
#define USBP_VID            0x303A
#define USBP_PID            0x4052
#define USBP_EP_OUT         0x01        /* fixed by usb-spike-esp/main/main.c */
#define USBP_EP_IN          0x81

#define REQ_MAGIC           0x55
#define RESP_MAGIC          0x75
#define CMD_ECHO            1
#define CMD_LOG             2
#define FLAG_NO_REPLY       0x01
#define HDR_LEN             12
#define RESP_HDR_LEN        8
#define ST_OK               0
#define PADDED(n)           ((n) + ((((n) & 63u) == 0) ? 1u : 0u))

/* ---- tunables ------------------------------------------------------------- */
#define USBP_GRACE_MS       25000   /* WiiFlow hand-over + game boot */
#define USBP_ITERS          200
#define USBP_SOAK_SECS      60
#define USBP_WINDOW_SECS    5
#define USBP_STAND_EVERY    32
#define USBP_FRAME_US       16667
#define USBP_SAMPLES        320
/* The OHCI crosses only ONE 4 KB page boundary per transfer descriptor and
 * IOS does not split (GameCube round 2): page-aligned buffers, <= 8192 B. */
#define USBP_TX_MAX         8192
#define USBP_RX_MAX         2560

/* ---- IOS USB APIs ---------------------------------------------------------- */
#define USBV5_IOCTL_GETVERSION      0
#define USBV5_IOCTL_GETDEVICECHANGE 1
#define USBV5_IOCTL_ATTACHFINISH    6
#define USBV5_IOCTL_SUSPEND_RESUME  16
#define USBV5_IOCTL_BULKMSG         21
#define USBV0_IOCTL_BLKMSG          1

/* Raw IOS kernel codes (NOT cios-lib's IPC_E*, where EINVAL is -101):
 * -4 = what IOS58 returned for a second /dev/usb/ven open (GameCube round 5),
 * -6 = no such device. */
#define IOS_RAW_EINVAL              (-4)
#define IOS_RAW_ENOENT              (-6)

enum { BK_NONE = 0, BK_VEN, BK_OH0 };
static const char *const bk_name[] = { "none", "ven(USBV5)", "oh0(USBV0)" };

static const char ven_path[] __attribute__((aligned(32))) = "/dev/usb/ven";
static const char oh0_dev_path[] __attribute__((aligned(32))) = "/dev/usb/oh0/303a/4052";

/* One bulk request: everything IOS reads, each piece on its own 32-byte
 * line(s). 224 bytes = 7 lines, so it stays aligned even in an array (the
 * GameCube round-5 trap was an 88-byte struct in an array). */
typedef struct {
	u32    v5[16];      /* USBV5 request block (64 B, read through vec[0]) */
	u32    v0_ep[8];    /* USBV0: endpoint byte */
	u32    v0_len[8];   /* USBV0: u16 length */
	ioctlv vec[4];
	u32    reply[16];   /* async completion record; IOS fills reply[1] */
} __attribute__((aligned(32))) usbp_req_t;

typedef char usbp_req_size_check[(sizeof(usbp_req_t) % 32) == 0 ? 1 : -1];

typedef struct {
	u16 out_len;
	u16 resp_len;
	const char *name;
} usbp_row_t;

static const usbp_row_t row_floor = {   64,    8, "floor"     };
static const usbp_row_t row_gc    = { 2048,    8, "gc"        };
static const usbp_row_t row_wii   = { 5120,    8, "wii"       };
static const usbp_row_t row_wiiw  = { 8191,    8, "wii-worst" };   /* 8192 -> 8193 = 3 pages */

typedef struct {
	u32 n_ok, n_err, n_bad;
	u32 p50, p90, p99, max;     /* s_pair */
	u32 a50, a90;               /* s_a */
	u32 b50, b90;               /* s_b */
} usbp_stats_t;

/* ---- state --------------------------------------------------------------- */
static u8  usbp_buf0[USBP_TX_MAX] __attribute__((aligned(4096)));
static u8  usbp_buf1[USBP_TX_MAX] __attribute__((aligned(4096)));
static u8  usbp_rx[USBP_RX_MAX] __attribute__((aligned(32)));
static u32 usbp_devlist[0x180 / 4] __attribute__((aligned(32)));
static u32 usbp_iobuf[8] __attribute__((aligned(32)));
static u32 usbp_qbuf_in[4] __attribute__((aligned(32)));
static u32 usbp_qbuf_out[4] __attribute__((aligned(32)));
static u32 usbp_qbuf_disc[4] __attribute__((aligned(32)));

static usbp_req_t req_sync_out;
static usbp_req_t req_sync_in;
static usbp_req_t req_stand_in;
static usbp_req_t req_stream0;
static usbp_req_t req_stream1;

static u32 s_pair[USBP_SAMPLES], s_a[USBP_SAMPLES], s_b[USBP_SAMPLES], s_sort[USBP_SAMPLES];
static char usbp_line[320];

/* NO initializers on writable globals: .data reaches the Wii zeroed (see
 * link_probe.ld). Anything that must start non-zero is set in usbp_globals_init. */
static s32 g_bk;            /* BK_NONE */
static s32 g_fd;            /* /dev/usb/ven fd, or the oh0 device fd */
static s32 g_dev;           /* USBV5 device id */
static s32 g_qin, g_qout, g_qdisc;
static u32 g_seq;
static s32 g_last;          /* raw code of the last failure */
static bool g_link;         /* LOG lines can reach the ESP */
static bool g_stand_posted;

/* Stream: two OUT buffers in flight. */
static u8 *s_buf[2];                  /* usbp_buf0/1, set in usbp_globals_init */
static usbp_req_t *const s_req[2] = { &req_stream0, &req_stream1 };   /* .rodata: loaded */
static u32 s_phys[2], s_t[2], s_inflight, s_next;

static u32 tot_ok, tot_err, tot_bad, tot_worst;

static u8 usbp_stack[8192] __attribute__((aligned(32)));

static void usbp_globals_init(void)
{
	g_bk = BK_NONE;
	g_fd = -1;
	g_dev = -1;
	g_qin = g_qout = g_qdisc = -1;
	s_buf[0] = usbp_buf0;
	s_buf[1] = usbp_buf1;
}

/* ---- time ------------------------------------------------------------------ */
static u32 now_ticks(void)
{
	return vi_read_hw_timer();
}

static u32 ticks_us(u32 t)
{
	/* HW_TIMER = 243 MHz / 128 = 1.8984375 MHz. vi_ticks_to_us is +-527 us;
	 * this is exact enough for sub-ms numbers and stays in u32. */
	return (t / 1898u) * 1000u + ((t % 1898u) * 1000u) / 1898u;
}

static void sleep_us(s32 us)
{
	static u32 q_buf[4] __attribute__((aligned(32)));
	s32 q, t;
	u32 msg;

	if (us <= 0)
		return;
	q = os_message_queue_create(q_buf, 4);
	if (q < 0)
		return;
	t = os_create_timer(us, 0, q, 0x777);
	if (t >= 0) {
		msg = 0;
		while (msg != 0x777)
			os_message_queue_receive(q, &msg, 0);
		os_destroy_timer(t);
	}
	os_message_queue_destroy(q);
}

/* exi.c's cooperative waits (normally provided by main.c). */
void ra_sleep(s32 time_ms)
{
	sleep_us(time_ms * 1000);
}

void ra_sleep_us(s32 time_us)
{
	sleep_us(time_us);
}

/* ---- bulk transfers --------------------------------------------------------- */

static u32 be32(const u8 *p)
{
	return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3];
}

/* Fill r for one bulk transfer on the active backend; returns the ioctl. */
static s32 req_fill(usbp_req_t *r, u8 ep, u16 len, void *data, s32 *n_in, s32 *n_io)
{
	u8 *h;
	s32 dir = (ep & 0x80) ? 1 : 0;

	/* u16 fields written byte by byte (big-endian Starlet): a u16 store
	 * through a punned u32 buffer breaks strict aliasing under -Os. */
	memset(r, 0, sizeof(*r));
	if (g_bk == BK_VEN) {
		h = (u8 *)r->v5;
		r->v5[0] = (u32)g_dev;
		r->v5[2] = (u32)data;
		h[12] = (u8)(len >> 8);
		h[13] = (u8)len;
		h[18] = ep;
		r->vec[0].data = r->v5;
		r->vec[0].len  = 64;
		r->vec[1].data = data;
		r->vec[1].len  = len;
		*n_in = 2 - dir;
		*n_io = dir;
		os_sync_after_write(r, sizeof(*r));
		return USBV5_IOCTL_BULKMSG;
	}
	((u8 *)r->v0_ep)[0] = ep;
	((u8 *)r->v0_len)[0] = (u8)(len >> 8);
	((u8 *)r->v0_len)[1] = (u8)len;
	r->vec[0].data = r->v0_ep;
	r->vec[0].len  = 1;
	r->vec[1].data = r->v0_len;
	r->vec[1].len  = 2;
	r->vec[2].data = data;
	r->vec[2].len  = len;
	*n_in = 2;
	*n_io = 1;
	os_sync_after_write(r, sizeof(*r));
	return USBV0_IOCTL_BLKMSG;
}

static s32 bulk_sync(usbp_req_t *r, u8 ep, u16 len, void *data)
{
	s32 n_in, n_io, ioc;

	ioc = req_fill(r, ep, len, data, &n_in, &n_io);
	return os_ioctlv(g_fd, ioc, n_in, n_io, r->vec);
}

static s32 bulk_async(usbp_req_t *r, u8 ep, u16 len, void *data, s32 queue)
{
	s32 n_in, n_io, ioc;

	ioc = req_fill(r, ep, len, data, &n_in, &n_io);
	return os_ioctlv_async(g_fd, ioc, n_in, n_io, r->vec, queue, r->reply);
}

/* Block for the next completion on queue; returns the transfer result.
 * NO cache invalidate on the reply record: the IOS kernel writes it with the
 * CPU (it sits dirty in the shared D-cache), so os_sync_before_read would
 * DISCARD the fresh result and expose the zero from req_fill's memset. Wii
 * round 5: stream "errors" were exactly that — result 0 whenever the line
 * had not been evicted yet, while the ESP had received every byte.
 * Invalidate only what DMA writes (usbp_rx). */
static s32 reap(s32 queue, usbp_req_t **which)
{
	u32 msg = 0;
	u32 *rep;

	os_message_queue_receive(queue, &msg, 0);
	rep = (u32 *)msg;
	if (which) {
		if (rep == req_stream0.reply)      *which = &req_stream0;
		else if (rep == req_stream1.reply) *which = &req_stream1;
		else                               *which = &req_stand_in;
	}
	return (s32)rep[1];
}

/* ---- protocol ---------------------------------------------------------------- */

static u32 build(u8 *tx, u8 cmd, u8 flags, u32 out_len, u32 resp_len, const char *text, u32 seq)
{
	u32 i, phys = PADDED(out_len);

	tx[0] = REQ_MAGIC;
	tx[1] = cmd;
	tx[2] = (u8)(out_len >> 8);
	tx[3] = (u8)out_len;
	tx[4] = (u8)(resp_len >> 8);
	tx[5] = (u8)resp_len;
	tx[6] = flags;
	tx[7] = 0;
	tx[8] = (u8)(seq >> 24);
	tx[9] = (u8)(seq >> 16);
	tx[10] = (u8)(seq >> 8);
	tx[11] = (u8)seq;
	if (cmd == CMD_LOG)
		memcpy(tx + HDR_LEN, text, out_len - HDR_LEN);
	else
		for (i = HDR_LEN; i < out_len; i++)
			tx[i] = (u8)(seq + i);
	if (phys != out_len)
		tx[out_len] = 0;
	return phys;
}

/* 0 = ok, 1 = malformed/corrupt, 2 = ESP flagged the request. */
static s32 check_reply(s32 r, u32 seq, u32 resp_len, u32 phys)
{
	const u8 *rx = usbp_rx;
	u32 i;

	if (r != (s32)PADDED(resp_len) || rx[0] != RESP_MAGIC || be32(rx + 4) != seq) {
		g_last = r;
		return 1;
	}
	if (rx[1] != ST_OK) {
		g_last = rx[1];
		return 2;
	}
	if ((((u32)rx[2] << 8) | rx[3]) != phys) {
		g_last = ((u32)rx[2] << 8) | rx[3];
		return 1;
	}
	for (i = RESP_HDR_LEN; i < resp_len; i++) {
		if (rx[i] != (u8)((seq ^ 0xA5u) + i)) {
			g_last = (s32)i;
			return 1;
		}
	}
	return 0;
}

/* One blocking exchange (no_reply: OUT only). 0 ok, -1 OUT failed, -2 IN
 * failed, >0 check_reply verdict. Must not run with a standing IN or a
 * stream OUT in flight. */
static s32 exchange(u8 cmd, bool no_reply, u32 out_len, u32 resp_len, const char *text,
                    u32 *us_out, u32 *us_in)
{
	u32 seq, phys, t0, t1, t2;
	s32 r;

	seq = ++g_seq;
	phys = build(usbp_buf0, cmd, no_reply ? FLAG_NO_REPLY : 0, out_len, resp_len, text, seq);

	t0 = now_ticks();
	os_sync_after_write(usbp_buf0, (phys + 31u) & ~31u);
	r = bulk_sync(&req_sync_out, USBP_EP_OUT, (u16)phys, usbp_buf0);
	t1 = now_ticks();
	if (r != (s32)phys) {
		g_last = r;
		return -1;
	}
	*us_out = ticks_us(t1 - t0);
	*us_in = 0;
	if (no_reply)
		return 0;
	r = bulk_sync(&req_sync_in, USBP_EP_IN, USBP_RX_MAX, usbp_rx);
	os_sync_before_read(usbp_rx, USBP_RX_MAX);
	t2 = now_ticks();
	if (r < 0) {
		g_last = r;
		return -2;
	}
	r = check_reply(r, seq, resp_len, phys);
	if (r)
		return r;
	*us_in = ticks_us(t2 - t1);
	return 0;
}

static void usbp_log(const char *fmt, ...)
{
	va_list args;
	u32 n, uo, ui;

	if (!g_link)
		return;
	va_start(args, fmt);
	vsprintf(usbp_line, fmt, args);
	va_end(args);
	n = strlen(usbp_line);
	exchange(CMD_LOG, false, HDR_LEN + n, RESP_HDR_LEN, usbp_line, &uo, &ui);
}

/* ---- EXI side channel (works without the USB link) ------------------------
 * Wii rounds 1-2 failed BEFORE the USB link existed, so the discovery report
 * never left the console. If the RetroAchievements memory-card adapter sits
 * in Slot B, its firmware prints every RA_CMD_DEBUG_LOG as "DEBUG=GC: ..."
 * (RA_LOG_LEVEL 2, on the XIAO's own USB-C). Fire-and-forget like
 * Nintendont's ra_debug: one CS-low write, no INT wait. Harmless with no
 * adapter in the slot. */
static bool g_exi_up;
static u32  g_exi_dead;   /* lines whose EXI write timed out (bus master off?) */
static char diag_line[240];

static void exi_dbg(const char *s)
{
	static u8 buf[260] __attribute__((aligned(32)));
	ra_gc_header_t *hdr = (ra_gc_header_t *)buf;
	u32 n = strlen(s);

	if (!g_exi_up) {
		exi_init();   /* from the worker thread, after the grace period */
		g_exi_up = true;
	}
	if (n > 200)
		n = 200;
	hdr->magic       = RA_MAGIC_GC_TO_ESP;
	hdr->command     = RA_CMD_DEBUG_LOG;
	hdr->payload_len = (u16)(1 + n);
	buf[sizeof(ra_gc_header_t)] = (u8)n;
	memcpy(buf + sizeof(ra_gc_header_t) + 1, s, n);

	/* Same wire recipe as main.c's ra_debug_send_always (16 MHz, INT latch
	 * cleared first). Needs WiiFlow's RetroAchievements setting ON: the PPC
	 * side brings up the EXI bus master during its pre-boot handshake; with
	 * it OFF the Starlet's transfers never complete (exi.c, exi_init notes)
	 * and the adapter only logs 4-6 byte junk frames (Wii round 3). */
	sleep_us(10000);   /* let the ESP re-arm after the previous line */
	exi_clear_int(EXI_CHAN_1);
	if (exi_select(EXI_CHAN_1, EXI_DEVICE_0, EXI_SPEED16MHZ) < 0)
		return;
	if (exi_imm_write(EXI_CHAN_1, buf, sizeof(ra_gc_header_t) + 1 + n) < 0)
		g_exi_dead++;
	exi_deselect(EXI_CHAN_1);
	sleep_us(15000);   /* its Serial print */
}

/* Discovery breadcrumb: EXI always, USB too once the link is up. */
static void diag(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vsprintf(diag_line, fmt, args);
	va_end(args);
	exi_dbg(diag_line);
	if (g_link)
		usbp_log("%s", diag_line);
}

/* ---- stats ------------------------------------------------------------------- */

static void sort_into(const u32 *src, u32 n)
{
	u32 i, j, v;
	for (i = 0; i < n; i++) {
		v = src[i];
		for (j = i; j > 0 && s_sort[j - 1] > v; j--)
			s_sort[j] = s_sort[j - 1];
		s_sort[j] = v;
	}
}

static u32 pct(u32 n, u32 p)
{
	return n ? s_sort[((n - 1) * p) / 100u] : 0;
}

static void summarize(u32 n_pair, u32 n_a, u32 n_b, usbp_stats_t *st)
{
	sort_into(s_a, n_a);
	st->a50 = pct(n_a, 50);
	st->a90 = pct(n_a, 90);
	sort_into(s_b, n_b);
	st->b50 = pct(n_b, 50);
	st->b90 = pct(n_b, 90);
	sort_into(s_pair, n_pair);
	st->p50 = pct(n_pair, 50);
	st->p90 = pct(n_pair, 90);
	st->p99 = pct(n_pair, 99);
	st->max = n_pair ? s_sort[n_pair - 1] : 0;
}

static void account(const usbp_stats_t *st)
{
	tot_ok += st->n_ok;
	tot_err += st->n_err;
	tot_bad += st->n_bad;
	if (st->max > tot_worst)
		tot_worst = st->max;
}

/* ---- sync / noreply rows -------------------------------------------------- */

static void run_row(const usbp_row_t *row, bool no_reply, usbp_stats_t *st)
{
	u32 k, n = 0, consec = 0, uo, ui;
	s32 r;

	memset(st, 0, sizeof(*st));
	for (k = 0; k < USBP_ITERS; k++) {
		r = exchange(CMD_ECHO, no_reply, row->out_len, row->resp_len, NULL, &uo, &ui);
		if (r < 0) {
			st->n_err++;
			if (++consec >= 10)
				break;
			continue;
		}
		consec = 0;
		if (r > 0) {
			st->n_bad++;
			continue;
		}
		s_a[n] = uo;
		s_b[n] = ui;
		s_pair[n] = uo + ui;
		n++;
	}
	st->n_ok = n;
	summarize(n, n, n, st);
	account(st);
}

static void report_row(const char *label, u32 pass, const usbp_row_t *row, bool no_reply,
                       const usbp_stats_t *st)
{
	usbp_log("USBW %s#%u %s %s %u/%u ok=%u err=%u bad=%u last=%d "
	         "pair50/90/99/max=%u/%u/%u/%u out50/90=%u/%u in50/90=%u/%u us",
	         label, pass, no_reply ? "noreply" : "sync", row->name, row->out_len,
	         row->resp_len, st->n_ok, st->n_err, st->n_bad, st->n_err + st->n_bad ? g_last : 0,
	         st->p50, st->p90, st->p99, st->max, st->a50, st->a90, st->b50, st->b90);
}

/* ---- stream (two OUTs in flight) and the standing IN ------------------------ */

static s32 stream_reap(u32 *lat_us)
{
	usbp_req_t *which = NULL;
	u32 k;
	s32 r;

	r = reap(g_qout, &which);
	k = (which == &req_stream0) ? 0 : 1;
	*lat_us = ticks_us(now_ticks() - s_t[k]);
	s_inflight &= ~(1u << k);
	if (r != (s32)s_phys[k]) {
		g_last = r;
		return -1;
	}
	return 0;
}

/* Post one request from the next buffer (waits only while that buffer is
 * still on the wire). flags = FLAG_NO_REPLY except when the ESP must answer
 * the standing IN. *block_us = thread time spent on USB for this post. */
static s32 stream_post(u32 out_len, u8 flags, u32 *seq_out, u32 *block_us, u32 *lat_us,
                       bool *reaped)
{
	u32 k = s_next, t0, t1, t2, t3;
	s32 r = 0, p;

	*reaped = false;
	*block_us = 0;
	t0 = now_ticks();
	if (s_inflight & (1u << k)) {
		r = stream_reap(lat_us);
		*reaped = true;
	}
	t1 = now_ticks();
	*seq_out = ++g_seq;
	s_phys[k] = build(s_buf[k], CMD_ECHO, flags, out_len, RESP_HDR_LEN, NULL, *seq_out);
	t2 = now_ticks();
	os_sync_after_write(s_buf[k], (s_phys[k] + 31u) & ~31u);
	s_t[k] = t2;
	p = bulk_async(s_req[k], USBP_EP_OUT, (u16)s_phys[k], s_buf[k], g_qout);
	if (p < 0) {
		g_last = p;
		return -1;
	}
	s_inflight |= 1u << k;
	s_next ^= 1;
	t3 = now_ticks();
	*block_us = ticks_us(t1 - t0) + ticks_us(t3 - t2);
	return r;
}

static u32 stream_drain(void)
{
	u32 lat, errs = 0;
	while (s_inflight)
		if (stream_reap(&lat) < 0)
			errs++;
	return errs;
}

static s32 stand_post(void)
{
	s32 r = bulk_async(&req_stand_in, USBP_EP_IN, USBP_RX_MAX, usbp_rx, g_qin);
	g_stand_posted = (r >= 0);
	return r;
}

/* Collect the standing IN's reply to request seq (sent with the reply flag
 * and already on its way). Returns check_reply's verdict; *block_us = how
 * long the thread waited for it. */
static s32 stand_collect(u32 seq, u32 phys, u32 *block_us)
{
	u32 t0 = now_ticks();
	s32 r = reap(g_qin, NULL);

	g_stand_posted = false;
	os_sync_before_read(usbp_rx, USBP_RX_MAX);
	*block_us = ticks_us(now_ticks() - t0);
	if (r < 0) {
		g_last = r;
		return -2;
	}
	return check_reply(r, seq, RESP_HDR_LEN, phys);
}

/* Close a "prod" run: drain the stream, then let the ESP answer one last
 * request so the standing IN is consumed (sync OUT; the reply lands in the
 * standing IN, not in a new read). */
static s32 stand_close(void)
{
	u32 seq, phys, blk;
	s32 r;

	stream_drain();
	if (!g_stand_posted)
		return 0;
	seq = ++g_seq;
	phys = build(usbp_buf0, CMD_ECHO, 0, 64, RESP_HDR_LEN, NULL, seq);
	os_sync_after_write(usbp_buf0, 96);
	r = bulk_sync(&req_sync_out, USBP_EP_OUT, (u16)phys, usbp_buf0);
	if (r != (s32)phys) {
		g_last = r;
		return -1;
	}
	return stand_collect(seq, phys, &blk);
}

/* Back-to-back stream: throughput + per-post block time + completion. */
static void run_stream(const usbp_row_t *row, usbp_stats_t *st, u32 *thr)
{
	u32 k, n_blk = 0, n_lat = 0, blk, lat, seq, t0, el;
	bool reaped;
	s32 r;

	memset(st, 0, sizeof(*st));
	*thr = 0;
	t0 = now_ticks();
	for (k = 0; k < USBP_ITERS; k++) {
		r = stream_post(row->out_len, FLAG_NO_REPLY, &seq, &blk, &lat, &reaped);
		if (r < 0)
			st->n_err++;
		else if (reaped)
			s_b[n_lat++] = lat;
		s_a[n_blk++] = blk;
	}
	st->n_err += stream_drain();
	el = ticks_us(now_ticks() - t0);
	if (el)   /* 200 x 8191 x 1000 = 1.64e9: fits in u32 */
		*thr = (USBP_ITERS * (u32)row->out_len * 1000u) / el;
	st->n_ok = USBP_ITERS - st->n_err;
	summarize(0, n_blk, n_lat, st);
	account(st);
}

/* Production shape at full speed: stream OUTs with a standing IN; every
 * USBP_STAND_EVERY-th request asks for a reply, collected one post later. */
static void run_prod(const usbp_row_t *row, usbp_stats_t *st, u32 *thr)
{
	u32 k, n_blk = 0, n_col = 0, blk, lat, seq, t0, el, pend_seq = 0, pend_phys = 0;
	bool reaped, pend = false, want;
	s32 r;

	memset(st, 0, sizeof(*st));
	*thr = 0;
	if (stand_post() < 0) {
		st->n_err = 1;
		return;
	}
	t0 = now_ticks();
	for (k = 0; k < USBP_ITERS; k++) {
		want = (k % USBP_STAND_EVERY) == USBP_STAND_EVERY - 1;
		r = stream_post(row->out_len, want ? 0 : FLAG_NO_REPLY, &seq, &blk, &lat, &reaped);
		if (r < 0)
			st->n_err++;
		s_a[n_blk++] = blk;
		if (pend) {
			/* reply to the previous frame's request: should be there already */
			r = stand_collect(pend_seq, pend_phys, &blk);
			if (r < 0) st->n_err++;
			else if (r > 0) st->n_bad++;
			s_b[n_col++] = blk;
			pend = false;
			if (stand_post() < 0)
				st->n_err++;
		}
		if (want) {
			pend = true;
			pend_seq = seq;
			pend_phys = s_phys[s_next ^ 1];
		}
	}
	stream_drain();
	if (pend) {
		r = stand_collect(pend_seq, pend_phys, &blk);
		if (r != 0) st->n_err++;
	}
	el = ticks_us(now_ticks() - t0);
	if (el)
		*thr = (USBP_ITERS * (u32)row->out_len * 1000u) / el;
	if (stand_close() != 0)
		st->n_err++;
	st->n_ok = USBP_ITERS - st->n_err;
	summarize(0, n_blk, n_col, st);
	account(st);
}

/* ---- matrix + soak -------------------------------------------------------- */

static void matrix(const char *label, u32 pass)
{
	static const usbp_row_t *const sync_rows[] = { &row_floor, &row_gc, &row_wii, &row_wiiw };
	static const usbp_row_t *const nr_rows[]   = { &row_floor, &row_gc, &row_wii };
	static const usbp_row_t *const st_rows[]   = { &row_gc, &row_wii, &row_wiiw };
	static const usbp_row_t *const pr_rows[]   = { &row_gc, &row_wii };
	usbp_stats_t st;
	u32 i, thr;

	for (i = 0; i < sizeof(sync_rows) / sizeof(sync_rows[0]); i++) {
		run_row(sync_rows[i], false, &st);
		report_row(label, pass, sync_rows[i], false, &st);
	}
	for (i = 0; i < sizeof(nr_rows) / sizeof(nr_rows[0]); i++) {
		run_row(nr_rows[i], true, &st);
		report_row(label, pass, nr_rows[i], true, &st);
	}
	for (i = 0; i < sizeof(st_rows) / sizeof(st_rows[0]); i++) {
		run_stream(st_rows[i], &st, &thr);
		usbp_log("USBW %s#%u stream %s %u ok=%u err=%u last=%d block50/90=%u/%u us "
		         "complete50/90=%u/%u us thr=%uB/ms",
		         label, pass, st_rows[i]->name, st_rows[i]->out_len, st.n_ok, st.n_err,
		         st.n_err ? g_last : 0, st.a50, st.a90, st.b50, st.b90, thr);
	}
	for (i = 0; i < sizeof(pr_rows) / sizeof(pr_rows[0]); i++) {
		run_prod(pr_rows[i], &st, &thr);
		usbp_log("USBW %s#%u prod %s %u ok=%u err=%u bad=%u last=%d block50/90=%u/%u us "
		         "reply-wait50/90=%u/%u us thr=%uB/ms",
		         label, pass, pr_rows[i]->name, pr_rows[i]->out_len, st.n_ok, st.n_err,
		         st.n_bad, st.n_err + st.n_bad ? g_last : 0, st.a50, st.a90, st.b50, st.b90, thr);
	}
}

enum { SK_SYNC, SK_NOREPLY, SK_STREAM, SK_PROD };
static const struct { u8 kind; const usbp_row_t *row; const char *name; } soak_plan[] = {
	{ SK_SYNC,    &row_wii, "sync"    },
	{ SK_NOREPLY, &row_wii, "noreply" },
	{ SK_STREAM,  &row_wii, "stream"  },
	{ SK_PROD,    &row_wii, "prod"    },
	{ SK_PROD,    &row_gc,  "prod"    },
	{ SK_STREAM,  &row_gc,  "stream"  },
};
#define SOAK_FLAVOURS (sizeof(soak_plan) / sizeof(soak_plan[0]))

/* 60 Hz production pacing. cost = thread time on USB that frame;
 * "late" = the frame's USB work overran the frame. */
static void soak(u32 pass)
{
	const u32 per_win = 60u * USBP_WINDOW_SECS;
	u32 w, k, f, n, n_b, late, err, bad, uo, ui, cost, next, now, seq, lat;
	u32 pend_seq = 0, pend_phys = 0, kind, out_len, resp_len;
	bool reaped, pend, want;
	s32 r;
	usbp_stats_t st;

	next = now_ticks();
	for (w = 1; w <= USBP_SOAK_SECS / USBP_WINDOW_SECS; w++) {
		f = (w - 1) % SOAK_FLAVOURS;
		kind = soak_plan[f].kind;
		out_len = soak_plan[f].row->out_len;
		resp_len = soak_plan[f].row->resp_len;
		n = n_b = late = err = bad = 0;
		pend = false;
		if (kind == SK_PROD && stand_post() < 0)
			err++;

		for (k = 0; k < per_win; k++) {
			next += (USBP_FRAME_US * 1898u) / 1000u;
			ui = 0;
			if (kind == SK_SYNC || kind == SK_NOREPLY) {
				r = exchange(CMD_ECHO, kind == SK_NOREPLY, out_len, resp_len, NULL, &uo, &ui);
				cost = uo + ui;
			} else {
				want = (kind == SK_PROD) && (k % USBP_STAND_EVERY) == USBP_STAND_EVERY - 1;
				r = stream_post(out_len, want ? 0 : FLAG_NO_REPLY, &seq, &cost, &lat, &reaped);
				if (pend) {
					s32 v = stand_collect(pend_seq, pend_phys, &ui);
					cost += ui;
					if (v < 0) err++;
					else if (v > 0) bad++;
					else if (n_b < USBP_SAMPLES) s_b[n_b++] = ui;
					pend = false;
					if (stand_post() < 0)
						err++;
				}
				if (want) {
					pend = true;
					pend_seq = seq;
					pend_phys = s_phys[s_next ^ 1];
				}
			}
			if (r < 0)
				err++;
			else if (r > 0)
				bad++;
			else if (n < USBP_SAMPLES)
				s_pair[n++] = cost;

			now = now_ticks();
			if ((s32)(now - next) >= 0) {
				late++;
				next = now;
			} else
				sleep_us((s32)ticks_us(next - now));
		}
		if (kind == SK_STREAM || kind == SK_PROD)
			err += stream_drain();
		if (pend) {
			r = stand_collect(pend_seq, pend_phys, &ui);
			if (r != 0) err++;
		}
		if (kind == SK_PROD && stand_close() != 0)
			err++;

		summarize(n, 0, n_b, &st);
		st.n_ok = n;
		st.n_err = err;
		st.n_bad = bad;
		account(&st);
		usbp_log("USBW soak#%u %s %u w=%u n=%u err=%u bad=%u late=%u "
		         "cost50/90/99/max=%u/%u/%u/%u us reply-wait50/90=%u/%u | total ok=%u err=%u "
		         "bad=%u worst=%u",
		         pass, soak_plan[f].name, out_len, w, n, err, bad, late,
		         st.p50, st.p90, st.p99, st.max, st.b50, st.b90,
		         tot_ok, tot_err, tot_bad, tot_worst);
	}
}

/* ---- discovery ----------------------------------------------------------- */

/* Everything discovery saw, reported over EXI (always) and USB (once up).
 * Wii rounds 1-2 (base 58, game on SD) ended in LED code 3 with no way to
 * see why; this is the why. */
#define DISC_ROUNDS_MAX   8
#define DISC_IDS_MAX      8
#define DISC_TRIES_MAX    10
static struct {
	s32 ven_open, ver_ret, oh0_open;
	u32 ver;
	s32 rounds;
	s32 ioctl_ret[DISC_ROUNDS_MAX];   /* os_ioctl_async return per round */
	s32 count[DISC_ROUNDS_MAX];       /* reply[1] per round, -1000 = deadline hit */
	s32 attach_ret[DISC_ROUNDS_MAX];
	u32 entry[DISC_IDS_MAX][3];       /* entries seen: id, vid<<16|pid, word 2 */
	s32 n_entries;
	s32 n_tries;
	u32 try_id[DISC_TRIES_MAX];
	s32 try_resume[DISC_TRIES_MAX];
	s32 try_ret[DISC_TRIES_MAX];      /* exchange() verdict */
	s32 try_last[DISC_TRIES_MAX];     /* raw code behind it */
	s32 via;                          /* -1 none, 0 list, 1 id from list, 2 known id */
	s32 fail;                         /* LED code, 0 = link up */
} disc;

static void disc_report(void)
{
	s32 i;

	diag("USBW disc: fail=%d via=%d ven_open=%d ver_ret=%d ver=%08x oh0_open=%d "
	     "rounds=%d entries=%d tries=%d exi_dead=%u",
	     disc.fail, disc.via, disc.ven_open, disc.ver_ret, disc.ver, disc.oh0_open,
	     disc.rounds, disc.n_entries, disc.n_tries, g_exi_dead);
	for (i = 0; i < disc.rounds; i++)
		diag("USBW disc round %d: ioctl=%d count=%d attachfinish=%d",
		     i + 1, disc.ioctl_ret[i], disc.count[i], disc.attach_ret[i]);
	for (i = 0; i < disc.n_entries; i++)
		diag("USBW disc entry %d: id=%08x vid:pid=%04x:%04x w2=%08x",
		     i, disc.entry[i][0], disc.entry[i][1] >> 16, disc.entry[i][1] & 0xFFFF,
		     disc.entry[i][2]);
	for (i = 0; i < disc.n_tries; i++)
		diag("USBW disc try %d: id=%08x resume=%d exchange=%d last=%d",
		     i, disc.try_id[i], disc.try_resume[i], disc.try_ret[i], disc.try_last[i]);
}

/* Failure before the USB link exists: disc-slot LED shows the code (n
 * blinks, pause) and the report repeats over EXI each cycle, so a serial
 * monitor opened late still catches it. */
static void usbp_fail(u32 code)
{
	u32 i, k;

	disc.fail = (s32)code;
	while (1) {
		disc_report();
		for (k = 0; k < 2; k++) {
			for (i = 0; i < code; i++) {
				Swi_LedOn();
				sleep_us(250000);
				Swi_LedOff();
				sleep_us(250000);
			}
			sleep_us(1500000);
		}
	}
}

/* Device id IOS58 gave the ESP on this console in every Nintendont run
 * (usbprobe.log "dev=e71f0021"): the last-resort candidate. */
#define USBP_KNOWN_DEV_ID   ((s32)0xE71F0021)

/* USBV5 discovery, the libogc/d2x-usb2 way: GETDEVICECHANGE (answers with
 * the attached list), ATTACHFINISH, repeat on change. One request in flight
 * at a time, 30 s overall deadline. Entries are 12 bytes: device_id, vid,
 * pid, ... (big-endian). Device ids have the top bit set (e61f0021 on this
 * console) — negative as s32, so "found" is a flag, never a sign test (Wii
 * round 5 waited out the whole deadline with the ESP listed in round 1). */
static bool ven_find(s32 *dev_out)
{
	static u32 reply[16] __attribute__((aligned(32)));
	const u8 *e;
	u32 msg;
	s32 t, r, n, i, rounds;
	bool found = false;

	t = os_create_timer(30000000, 0, g_qdisc, 0x777);
	for (rounds = 0; rounds < DISC_ROUNDS_MAX && !found; rounds++) {
		disc.rounds = rounds + 1;
		r = os_ioctl_async(g_fd, USBV5_IOCTL_GETDEVICECHANGE, NULL, 0, usbp_devlist, 0x180,
		                   g_qdisc, reply);
		disc.ioctl_ret[rounds] = r;
		if (r < 0)
			break;
		msg = 0;
		os_message_queue_receive(g_qdisc, &msg, 0);
		if (msg != (u32)reply) {
			disc.count[rounds] = -1000;
			break;   /* deadline */
		}
		/* reply: kernel-written, cached — read as is (see reap). devlist:
		 * the USB resource manager flushes its output buffer, so the
		 * invalidate is correct there (round 5 parsed it fine). */
		os_sync_before_read(usbp_devlist, 0x180);
		n = (s32)reply[1];
		disc.count[rounds] = n;
		for (i = 0; i < n && i < 32; i++) {
			e = (const u8 *)usbp_devlist + i * 12;
			if (disc.n_entries < DISC_IDS_MAX) {
				disc.entry[disc.n_entries][0] = be32(e);
				disc.entry[disc.n_entries][1] = be32(e + 4);
				disc.entry[disc.n_entries][2] = be32(e + 8);
				disc.n_entries++;
			}
			if (!found && ((e[4] << 8) | e[5]) == USBP_VID && ((e[6] << 8) | e[7]) == USBP_PID) {
				*dev_out = (s32)be32(e);
				found = true;
			}
		}
		disc.attach_ret[rounds] = os_ioctl(g_fd, USBV5_IOCTL_ATTACHFINISH, NULL, 0, NULL, 0);
		diag("USBW disc round %d: ioctl=%d count=%d attachfinish=%d", rounds + 1,
		     r, n, disc.attach_ret[rounds]);
	}
	if (t >= 0)
		os_destroy_timer(t);
	return found;
}

static s32 ven_resume(s32 dev)
{
	memset(usbp_iobuf, 0, 32);
	usbp_iobuf[0] = (u32)dev;
	usbp_iobuf[2] = 1;   /* resume */
	os_sync_after_write(usbp_iobuf, 32);
	return os_ioctl(g_fd, USBV5_IOCTL_SUSPEND_RESUME, usbp_iobuf, 32, NULL, 0);
}

/* Does an echo exchange work with device id dev? (resume + one 64/8 pair) */
static bool ven_try(s32 dev)
{
	u32 uo, ui;
	s32 k = disc.n_tries, rs, r;

	g_dev = dev;
	g_last = 0;
	rs = ven_resume(dev);
	r = exchange(CMD_ECHO, false, 64, 8, NULL, &uo, &ui);
	if (k < DISC_TRIES_MAX) {
		disc.try_id[k] = (u32)dev;
		disc.try_resume[k] = rs;
		disc.try_ret[k] = r;
		disc.try_last[k] = g_last;
		disc.n_tries++;
	}
	diag("USBW disc try id=%08x resume=%d exchange=%d last=%d", (u32)dev, rs, r, g_last);
	return r == 0;
}

/* Open the link on whichever API this base has. Fills g_bk/g_fd/g_dev. */
static void usbp_connect(void)
{
	u32 *ver = usbp_iobuf;
	s32 r, i, found_dev = 0;

	r = os_open(ven_path, 0);
	disc.ven_open = r;
	diag("USBW disc: /dev/usb/ven open=%d", r);
	if (r >= 0) {
		g_fd = r;
		memset(ver, 0, 32);
		os_sync_after_write(ver, 32);
		r = os_ioctl(g_fd, USBV5_IOCTL_GETVERSION, NULL, 0, ver, 32);
		os_sync_before_read(ver, 32);
		disc.ver_ret = r;
		disc.ver = ver[0];
		diag("USBW disc: GETVERSION ret=%d ver=%08x", r, ver[0]);
		if (r != 0 || ver[0] != 0x50001) {
			os_close(g_fd);
			g_fd = -1;
			usbp_fail(4);
		}
		g_bk = BK_VEN;   /* exchange() needs it for the fallbacks below */
		if (ven_find(&found_dev) && ven_try(found_dev)) {
			disc.via = 0;
			return;
		}
		/* Before giving up: every id the list showed (parse mismatch?) and
		 * the id this console's IOS58 always gave the ESP under Nintendont. */
		for (i = 0; i < disc.n_entries; i++) {
			if (ven_try((s32)disc.entry[i][0])) {
				disc.via = 1;
				return;
			}
		}
		if (ven_try(USBP_KNOWN_DEV_ID)) {
			disc.via = 2;
			return;
		}
		if (disc.ioctl_ret[0] < 0)
			usbp_fail(6);   /* GETDEVICECHANGE rejected */
		if (disc.count[0] == -1000)
			usbp_fail(7);   /* GETDEVICECHANGE never answered */
		usbp_fail(3);       /* answered, but no ESP in the list */
	}
	if (r == IOS_RAW_EINVAL)
		usbp_fail(2);   /* ven exists but is taken: d2x USBS serves a USB drive */

	r = os_open(oh0_dev_path, 0);
	disc.oh0_open = r;
	diag("USBW disc: /dev/usb/oh0/303a/4052 open=%d", r);
	if (r < 0)
		usbp_fail(r == IOS_RAW_ENOENT ? 3 : 4);
	g_fd = r;
	g_bk = BK_OH0;
}

static u32 usbp_thread(void *arg)
{
	s32 r;
	u32 ios, pass, uo, ui;
	(void)arg;

	usbp_globals_init();
	disc.via = -1;

	/* Alive: three short blinks right after the cIOS loads. */
	for (pass = 0; pass < 3; pass++) {
		Swi_LedOn();
		sleep_us(120000);
		Swi_LedOff();
		sleep_us(120000);
	}
	sleep_us(USBP_GRACE_MS * 1000);

	os_sync_before_read((void *)0x3140, 32);
	ios = *(vu32 *)0x3140;
	diag("USBW probe up: IOS%u rev %u, grace %u ms over, starting discovery",
	     ios >> 16, ios & 0xFFFF, USBP_GRACE_MS);

	g_qin = os_message_queue_create(usbp_qbuf_in, 4);
	g_qout = os_message_queue_create(usbp_qbuf_out, 4);
	g_qdisc = os_message_queue_create(usbp_qbuf_disc, 4);
	if (g_qin < 0 || g_qout < 0 || g_qdisc < 0)
		usbp_fail(5);

	usbp_connect();

	/* First LOG proves the link both ways; LED code 5 if it doesn't. */
	g_link = true;
	r = exchange(CMD_ECHO, false, 64, 8, NULL, &uo, &ui);
	if (r != 0) {
		g_link = false;
		usbp_fail(5);
	}

	usbp_log("USBW hello: IOS%u rev %u backend=%s fd=%d dev=%08x "
	         "first-exchange out/in=%u/%u us iters=%u",
	         ios >> 16, ios & 0xFFFF, bk_name[g_bk], g_fd, (u32)g_dev, uo, ui, USBP_ITERS);
	disc_report();

	for (pass = 1; ; pass++) {
		matrix("wii", pass);
		soak(pass);
	}
	return 0;
}

/* ---- module entry (same skeleton as main.c) ------------------------------- */

static s32 usbp_ioctlv(u32 cmd, ioctlv *vector, u32 inlen, u32 iolen)
{
	s32 ret = IPC_ENOENT;

	InvalidateVector(vector, inlen, iolen);
	if (cmd == RA_CMD_PING)
		ret = 0;
	FlushVector(vector, inlen, iolen);
	return ret;
}

int main(void)
{
	static u32 heapspace[0x100] __attribute__((aligned(32)));
	u32 queuehandle = 0;
	void *buffer;
	s32 ret, thread_id;

	/* NO svc_write here (or anywhere in the probe). It goes to MLOAD's USB
	 * Gecko driver on EXI Slot B, whose safe-send loop spins in kernel mode
	 * until the "Gecko" says ready — and the RA memory-card adapter in that
	 * slot never does. Wii round 5: the game hung at boot (RA on AND off).
	 * Production's identical call only survives because its string sits in
	 * the zeroed .rodata and is sent as "". */

	ret = Mem_Init(heapspace, sizeof(heapspace));
	if (ret >= 0) {
		buffer = Mem_Alloc(0x20);
		if (buffer) {
			ret = os_message_queue_create(buffer, 8);
			if (ret >= 0) {
				/* Register EXACTLY what production really registers: an
				 * empty name (its DEVICE_NAME literal sits in the zeroed
				 * .rodata). Wii rounds 1-3 did the same and the game
				 * booted; rounds 4-6 registered a real "/dev/ra" (now that
				 * .rodata loads) and the game hung at boot every time.
				 * Nothing needs to reach the probe over IPC. */
				static char no_name[4];   /* .bss: zero */
				os_device_register(no_name, ret);
				queuehandle = ret;
			}
		}
	}

	/* Probe thread at main's priority (start.s 0x48 — the proven tier). */
	thread_id = os_thread_create(usbp_thread, NULL, usbp_stack + sizeof(usbp_stack),
	                             sizeof(usbp_stack), os_thread_get_priority(os_get_thread_id()), 0);
	if (thread_id >= 0)
		os_thread_continue(thread_id);

	while (1) {
		ipcmessage *message = NULL;
		ret = os_message_queue_receive(queuehandle, (void *)&message, 0);
		if (ret)
			continue;
		switch (message->command) {
		case IOS_OPEN:
			ret = (!strcmp(message->open.device, DEVICE_NAME))
			      ? message->open.resultfd : IPC_ENOENT;
			break;
		case IOS_CLOSE:
			ret = 0;
			break;
		case IOS_IOCTLV:
			ret = usbp_ioctlv(message->ioctlv.command, message->ioctlv.vector,
			                  message->ioctlv.num_in, message->ioctlv.num_io);
			break;
		default:
			ret = IPC_EINVAL;
		}
		os_message_queue_ack(message, ret);
	}
	return 0;
}

#endif /* RA_BUILD_USB_PROBE */
