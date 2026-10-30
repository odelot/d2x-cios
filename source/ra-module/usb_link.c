/*
 * usb_link.c — RA console link over USB (RAMOD built with USB=1).
 *
 * The RA protocol is unchanged; only the wire differs from exi.c. Rules that
 * come from the hardware rounds of the USB spike (c:/dev/wii/PLAN-USB-SPIKE.md,
 * usbprobe.c is the measuring twin of this file):
 *   - API per base: /dev/usb/ven (USBV5) on base 58, /dev/usb/oh0/303a/4052
 *     (USBV0) on 38/56/57 — both validated with a game running (§10.2, §10.3).
 *     ven is single-client: with a USB drive served by d2x USBS it is taken
 *     (open = -4) and there is no adapter link on that base.
 *   - The OHCI crosses ONE 4 KB page boundary per transfer and IOS does not
 *     split: every DMA buffer is checked (dma_ok) and the snapshot buffer is
 *     4096-aligned in USB builds.
 *   - No message is a multiple of 64 bytes (a 0x00 byte is appended), so
 *     each one ends on a short packet — the ESP's end-of-message marker.
 *   - Async request blocks live in separate 32-byte-aligned variables, and
 *     the completion record is NOT cache-invalidated (the kernel writes it
 *     through the CPU; invalidating discarded fresh results in Wii round 5).
 *     Only DMA'd reply bytes are invalidated.
 *   - USBV5 device ids have bit 31 set: "found" is a flag, never id >= 0.
 *
 * One exchange (OUT request + IN reply) is on the wire at a time — the ESP
 * answers every request exactly once, in order. ra_usb_post leaves the
 * snapshot exchange in flight across the game frame; any blocking exchange
 * first waits for it (its reply stays in its buffer for ra_usb_reap).
 *
 * gcc 4.5.1 / C89: declarations at the top of each block. Built without the
 * SADDR strip (see Makefile), so string literals and .data are real here.
 */

#include <string.h>

#include "ios.h"
#include "ipc.h"
#include "syscalls.h"
#include "types.h"

#include "ra_build.h"
#include "usb_link.h"

#if RA_LINK_USB

#define USB_VID     0x303A
#define USB_PID     0x4052
#define EP_OUT      0x01        /* fixed by ra_usb_link.cpp (wii-ra-adapter) */
#define EP_IN       0x81

#define USBV5_IOCTL_GETVERSION      0
#define USBV5_IOCTL_GETDEVICECHANGE 1
#define USBV5_IOCTL_ATTACHFINISH    6
#define USBV5_IOCTL_SUSPEND_RESUME  16
#define USBV5_IOCTL_BULKMSG         21
#define USBV0_IOCTL_BLKMSG          1

/* Raw IOS codes (NOT cios-lib's IPC_E*, where EINVAL is -101). */
#define IOS_RAW_EINVAL  (-4)

#define RA_USB_ESHORT   (-304)      /* OUT moved fewer bytes than asked */
#define RA_USB_EIDLE    (-305)      /* reap with nothing posted */

/* Timer tokens share the queue with the completion records (module RAM
 * addresses, 0x136xxxxx) — this range can never be one of those. */
#define TOK_XFER        0x7A000000u
#define TOK_DISC        0x7B000000u

enum { BK_NONE = 0, BK_VEN, BK_OH0 };

static const char ven_path[] __attribute__((aligned(32))) = "/dev/usb/ven";
static const char oh0_path[] __attribute__((aligned(32))) = "/dev/usb/oh0/303a/4052";

/* One bulk request: everything IOS reads, each piece on its own 32-byte
 * line(s); 224 bytes keeps the alignment (usbprobe.c usbp_req_t). */
typedef struct {
	u32    v5[16];      /* USBV5 request block (64 B, vec[0]) */
	u32    v0_ep[8];    /* USBV0: endpoint byte */
	u32    v0_len[8];   /* USBV0: u16 length */
	ioctlv vec[4];
	u32    reply[16];   /* async completion record; IOS fills reply[1] */
} __attribute__((aligned(32))) usb_req_t;

typedef char usb_req_size_check[(sizeof(usb_req_t) % 32) == 0 ? 1 : -1];

static usb_req_t req_out;
static usb_req_t req_in;

/* Bounce buffers for the small exchanges (requests built on the stack,
 * Phase-A replies whose head must be skipped). 288 = the largest debug-log
 * request (260) + pad; 128 holds any short reply (default_ack is 14). */
static u8  small_out[288] __attribute__((aligned(32)));
static u8  small_in[128]  __attribute__((aligned(32)));
static u32 devlist[0x180 / 4] __attribute__((aligned(32)));
static u32 iobuf[8] __attribute__((aligned(32)));
static u32 disc_reply[16] __attribute__((aligned(32)));
static u32 qbuf[8] __attribute__((aligned(32)));
static u32 qbuf_disc[4] __attribute__((aligned(32)));

static s32 g_bk;
static s32 g_fd;
static s32 g_dev;           /* USBV5 device id (bit 31 set: negative) */
static s32 g_q;
static s32 g_qdisc;
static u32 g_tok;
static u8  g_inited;
static u8  g_up;            /* device open: exchanges allowed */
static u8  g_dead;          /* a post failed: the device is gone */
static u8  g_disc_pending;  /* a GETDEVICECHANGE is waiting for a change */

/* The exchange on the wire. */
static struct {
	u8  busy;
	u8  is_post;
	u8  out_done;
	u8  in_done;
	s32 out_res;
	s32 in_res;
	u32 out_len;
	u8 *in_buf;
	u32 in_cap;
} xs;

/* The pipelined (posted) exchange, from ra_usb_post to ra_usb_reap. */
static u8  post_open;
static u8  post_ready;
static s32 post_res;

static u32 be32(const u8 *p)
{
	return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3];
}

/* 32-byte aligned and at most one 4 KB page crossing. */
static int dma_ok(const void *p, u32 len)
{
	u32 a = (u32)p;

	if (a & 31u)
		return 0;
	return ((a & 4095u) + len) <= 8192u;
}

/* ---- bulk transfers ----------------------------------------------------------- */

static s32 req_fill(usb_req_t *r, u8 ep, u16 len, void *data, s32 *n_in, s32 *n_io)
{
	u8 *h;
	s32 dir = (ep & 0x80) ? 1 : 0;

	/* u16 fields byte by byte (big-endian Starlet; no punned u16 stores). */
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

static s32 bulk_async(usb_req_t *r, u8 ep, u16 len, void *data)
{
	s32 n_in, n_io, ioc;

	ioc = req_fill(r, ep, len, data, &n_in, &n_io);
	return os_ioctlv_async(g_fd, ioc, n_in, n_io, r->vec, g_q, r->reply);
}

/* Result of the finished exchange: 0, or why it failed. */
static s32 xs_result(void)
{
	if (xs.out_res != (s32)xs.out_len)
		return xs.out_res < 0 ? xs.out_res : RA_USB_ESHORT;
	if (xs.in_res < 0)
		return xs.in_res;
	return 0;
}

/* Wait until the exchange on the wire has both completions, at most
 * timeout_ms. On timeout it stays on the wire (busy) and the next wait
 * resumes it: the ESP answers every request, so alternation is kept. */
static s32 xs_wait(u32 timeout_ms)
{
	u32 msg, tok = 0;
	s32 t = -1;

	if (!xs.busy)
		return 0;
	if (timeout_ms) {
		tok = TOK_XFER | (++g_tok & 0xFFFFu);
		t = os_create_timer(timeout_ms * 1000, 0, g_q, tok);
	}
	while (!(xs.out_done && xs.in_done)) {
		msg = 0;
		os_message_queue_receive(g_q, &msg, 0);
		if (msg == (u32)req_out.reply) {
			xs.out_done = 1;
			xs.out_res = (s32)req_out.reply[1];
		} else if (msg == (u32)req_in.reply) {
			xs.in_done = 1;
			xs.in_res = (s32)req_in.reply[1];
		} else if (t >= 0 && msg == tok) {
			break;
		}
		/* anything else: an earlier wait's timer that fired late */
	}
	if (t >= 0)
		os_destroy_timer(t);
	if (!(xs.out_done && xs.in_done))
		return RA_USB_ETIMEOUT;

	xs.busy = 0;
	if (xs.in_res > 0)
		os_sync_before_read(xs.in_buf, xs.in_cap);   /* DMA'd reply bytes */
	if (xs.is_post) {
		xs.is_post = 0;
		post_res = xs_result();
		post_ready = 1;
	}
	return 0;
}

/* Put one exchange on the wire: the reply read first (it must be waiting
 * when the ESP answers), then the request. out_len already includes the
 * framing pad. */
static s32 xs_post(const void *out, u32 out_len, void *in, u32 in_cap, u8 is_post)
{
	s32 r;

	if (g_dead)
		return RA_USB_EDEAD;
	if (xs.busy)
		return RA_USB_EBUSY;
	xs.out_done = 0;
	xs.in_done = 0;
	xs.out_res = 0;
	xs.in_res = 0;
	xs.out_len = out_len;
	xs.in_buf = (u8 *)in;
	xs.in_cap = in_cap;
	xs.is_post = is_post;

	os_sync_after_write((void *)out, out_len);
	os_sync_after_write(in, in_cap);   /* no dirty line may land on the reply later */

	r = bulk_async(&req_in, EP_IN, (u16)in_cap, in);
	if (r < 0)
		return r;
	xs.busy = 1;
	r = bulk_async(&req_out, EP_OUT, (u16)out_len, (void *)out);
	if (r < 0) {
		/* The IN stays posted with no request behind it: the link is
		 * unusable until the device goes away and comes back. */
		xs.out_done = 1;
		xs.out_res = r;
		g_dead = 1;
		return RA_USB_EDEAD;
	}
	return 0;
}

/* ---- discovery --------------------------------------------------------------- */

static s32 ven_resume(s32 dev)
{
	memset(iobuf, 0, 32);
	iobuf[0] = (u32)dev;
	iobuf[2] = 1;   /* resume */
	os_sync_after_write(iobuf, 32);
	return os_ioctl(g_fd, USBV5_IOCTL_SUSPEND_RESUME, iobuf, 32, NULL, 0);
}

/* USBV5 discovery, the libogc way: GETDEVICECHANGE answers with the attached
 * list at once the first time and then only on a change; ATTACHFINISH after
 * each answer. A notification still pending when the budget runs out stays
 * pending (g_disc_pending) and the next call picks it up — never re-issued,
 * never abandoned on a closed fd. Entries: 12 bytes, id + vid + pid (BE). */
static s32 ven_find(s32 *dev_out)
{
	const u8 *e;
	u32 msg, tok;
	s32 t, r, n, i, round;

	for (round = 0; round < 2; round++) {
		if (!g_disc_pending) {
			r = os_ioctl_async(g_fd, USBV5_IOCTL_GETDEVICECHANGE, NULL, 0, devlist,
			                   sizeof(devlist), g_qdisc, disc_reply);
			if (r < 0)
				return r;
			g_disc_pending = 1;
		}
		tok = TOK_DISC | (++g_tok & 0xFFFFu);
		t = os_create_timer(1500000, 0, g_qdisc, tok);
		msg = 0;
		while (msg != (u32)disc_reply && msg != tok)
			os_message_queue_receive(g_qdisc, &msg, 0);
		if (t >= 0)
			os_destroy_timer(t);
		if (msg != (u32)disc_reply)
			return RA_USB_ETIMEOUT;   /* nothing attached changed */
		g_disc_pending = 0;

		os_sync_before_read(devlist, sizeof(devlist));
		n = (s32)disc_reply[1];
		for (i = 0; i < n && i < (s32)(sizeof(devlist) / 12); i++) {
			e = (const u8 *)devlist + i * 12;
			if (((e[4] << 8) | e[5]) == USB_VID && ((e[6] << 8) | e[7]) == USB_PID) {
				*dev_out = (s32)be32(e);
				os_ioctl(g_fd, USBV5_IOCTL_ATTACHFINISH, NULL, 0, NULL, 0);
				return 0;
			}
		}
		os_ioctl(g_fd, USBV5_IOCTL_ATTACHFINISH, NULL, 0, NULL, 0);
	}
	return RA_USB_ETIMEOUT;
}

s32 ra_usb_init(void)
{
	s32 r, dev = 0;

	if (g_up)
		return 0;
	if (!g_inited) {
		g_bk = BK_NONE;
		g_fd = -1;
		g_dev = -1;
		g_q = os_message_queue_create(qbuf, 8);
		g_qdisc = os_message_queue_create(qbuf_disc, 4);
		g_inited = 1;
	}
	if (g_q < 0 || g_qdisc < 0)
		return RA_USB_EDEAD;

	if (g_bk == BK_NONE) {
		r = os_open(ven_path, 0);
		if (r >= 0) {
			g_fd = r;
			memset(iobuf, 0, 32);
			os_sync_after_write(iobuf, 32);
			r = os_ioctl(g_fd, USBV5_IOCTL_GETVERSION, NULL, 0, iobuf, 32);
			os_sync_before_read(iobuf, 32);
			if (r != 0 || iobuf[0] != 0x50001) {
				os_close(g_fd);
				g_fd = -1;
				return RA_USB_EDEAD;
			}
			g_bk = BK_VEN;
		} else if (r == IOS_RAW_EINVAL) {
			return r;   /* ven taken: d2x USBS serves a USB drive on this base */
		} else {
			r = os_open(oh0_path, 0);
			if (r < 0)
				return r;   /* no USBV0 either, or the adapter isn't attached yet */
			g_fd = r;
			g_bk = BK_OH0;
			g_up = 1;
			return 0;
		}
	}

	/* ven: find the adapter in the attached list (retried by the caller). */
	r = ven_find(&dev);
	if (r < 0)
		return r;
	g_dev = dev;
	(void)ven_resume(dev);
	g_up = 1;
	return 0;
}

/* ---- exchanges ------------------------------------------------------------------ */

s32 ra_usb_xfer(const void *tx, u32 tx_len, void *rx, u32 rx_len, u32 skip, u32 timeout_ms)
{
	u8 *out, *in;
	u32 n, cap, c;
	s32 r;

	if (!g_up)
		return RA_USB_EDEAD;
	/* Whatever is still on the wire goes first (a posted snapshot keeps its
	 * reply for ra_usb_reap; a timed-out exchange's late reply is dropped). */
	if (xs_wait(timeout_ms) < 0)
		return RA_USB_EBUSY;

	if (tx_len + 1 <= sizeof(small_out)) {
		memcpy(small_out, tx, tx_len);
		out = small_out;
	} else if (dma_ok(tx, tx_len + 1)) {
		out = (u8 *)tx;   /* the snapshot buffer: room for the pad byte */
	} else {
		return RA_USB_EBUF;
	}
	n = tx_len;
	if ((n & 63u) == 0)
		out[n++] = 0;

	if (rx && skip == 0 && rx_len > sizeof(small_in) && dma_ok(rx, rx_len)) {
		in = (u8 *)rx;
		cap = rx_len;
	} else if (skip + rx_len <= sizeof(small_in)) {
		in = small_in;
		cap = sizeof(small_in);
	} else {
		return RA_USB_EBUF;
	}

	r = xs_post(out, n, in, cap, 0);
	if (r < 0)
		return r;
	r = xs_wait(timeout_ms);
	if (r < 0)
		return r;
	r = xs_result();
	if (r < 0)
		return r;
	if (in != (u8 *)rx && rx && rx_len) {
		memset(rx, 0, rx_len);
		if ((u32)xs.in_res > skip) {
			c = (u32)xs.in_res - skip;
			if (c > rx_len)
				c = rx_len;
			memcpy(rx, small_in + skip, c);
		}
	}
	return 0;
}

s32 ra_usb_post(void *tx, u32 tx_len, void *rx, u32 rx_cap)
{
	u8 *out = (u8 *)tx;
	u32 n = tx_len;
	s32 r;

	if (!g_up)
		return RA_USB_EDEAD;
	if (xs.busy || post_open)
		return RA_USB_EBUSY;
	if (tx_len > RA_USB_TX_MAX || !dma_ok(tx, tx_len + 1) || !dma_ok(rx, rx_cap))
		return RA_USB_EBUF;
	if ((n & 63u) == 0)
		out[n++] = 0;
	r = xs_post(out, n, rx, rx_cap, 1);
	if (r < 0)
		return r;
	post_open = 1;
	post_ready = 0;
	return 0;
}

s32 ra_usb_reap(u32 timeout_ms)
{
	s32 r;

	if (!post_open)
		return RA_USB_EIDLE;
	if (!post_ready) {
		r = xs_wait(timeout_ms ? timeout_ms : 1);
		if (r < 0)
			return r;   /* RA_USB_ETIMEOUT: still in flight */
	}
	post_open = 0;
	post_ready = 0;
	return post_res;
}

u32 ra_usb_posted(void)
{
	return post_open;
}

#endif /* RA_LINK_USB */
