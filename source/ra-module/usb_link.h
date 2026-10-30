/*
 * usb_link.h — RA console link over USB (RAMOD built with USB=1).
 *
 * Carries the RA protocol unchanged over bulk transfers to the adapter's USB
 * device (ESP32-S3, VID:PID 303A:4052, firmware built with
 * CONFIG_RA_LINK_USB). One request = one bulk OUT message, one reply = one
 * bulk IN message holding exactly the bytes the EXI read would have clocked
 * out (Phase-A pads included). Plan: c:/dev/wii/PLAN-USB-SPIKE.md §11.
 */

#ifndef _USB_LINK_H_
#define _USB_LINK_H_

#include "types.h"

/* Errors (raw IOS codes from the USB driver pass through unchanged). */
#define RA_USB_ETIMEOUT   (-300)   /* no reply in time (exchange left in flight) */
#define RA_USB_EBUSY      (-301)   /* a previous exchange is still in flight */
#define RA_USB_EDEAD      (-302)   /* link down (no device / post failed) */
#define RA_USB_EBUF       (-303)   /* buffer not DMA-safe and too big to bounce */

/* Find the adapter (ven on base 58, oh0 on 38/56/57) and prove the link with
 * an IDENTIFY. Returns 0, or <0 when no adapter answered this round (the
 * caller retries). */
s32 ra_usb_init(void);

/* Blocking request/reply. Reply bytes [skip, skip+rx_len) are copied to rx —
 * skip = tx_len reproduces the EXI single-CS transaction (the write phase
 * clocks out the first tx_len bytes of the reply), skip = 0 the Phase-B
 * read. Waits for any exchange still in flight first. */
s32 ra_usb_xfer(const void *tx, u32 tx_len, void *rx, u32 rx_len, u32 skip, u32 timeout_ms);

/* Pipelined exchange: post the request + its reply read and return at once
 * (the wire time overlaps the game frame). tx and rx must stay untouched
 * until ra_usb_reap returns 0. tx_len may reach RA_USB_TX_MAX only when tx
 * is 4096-aligned (OHCI: one page crossing per transfer). */
s32 ra_usb_post(void *tx, u32 tx_len, void *rx, u32 rx_cap);

/* Finish the posted exchange: 0 = reply in rx, RA_USB_ETIMEOUT = still in
 * flight (call again later), other <0 = failed. timeout_ms 0 = poll. */
s32 ra_usb_reap(u32 timeout_ms);

/* 1 while a posted exchange has not been reaped. */
u32 ra_usb_posted(void);

/* Largest request: 8192 would need a framing pad byte (8193 = 3 pages). */
#define RA_USB_TX_MAX     8191u

#endif
