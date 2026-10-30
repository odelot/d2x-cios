#ifndef _RA_BUILD_H_
#define _RA_BUILD_H_

/* Build flavour of RAMOD.
 *
 *   0 = the RetroAchievements module (main.c) — production.
 *   1 = USB transport SPIKE (usbprobe.c): a stand-alone measurement module
 *       that replaces RAMOD entirely (no RA, no EXI). It times bulk transfers
 *       to the ESP32-S3 echo firmware (c:/dev/wii/usb-spike-esp) from inside
 *       the cIOS while a game runs, and reports through the ESP's console.
 *       Install it in a SPARE slot only — see c:/dev/wii/PLAN-USB-SPIKE.md §10.
 *
 * Exactly one of main.c / usbprobe.c compiles to code; both are always in
 * the Makefile. Select the spike with `make USBPROBE=1` (the Makefile passes
 * -DRA_BUILD_USB_PROBE=1); the default stays production. Clean the module
 * when switching flavours — make does not track the flag. */
#ifndef RA_BUILD_USB_PROBE
#define RA_BUILD_USB_PROBE  0
#endif

/* Console link of the RA module (main.c):
 *
 *   0 = EXI, memory-card Slot B (exi.c) — production.
 *   1 = USB (usb_link.c), adapter firmware built with CONFIG_RA_LINK_USB.
 *       Same RA protocol; `make USB=1` (also drops the SADDR strip — see the
 *       Makefile). Plan: c:/dev/wii/PLAN-USB-SPIKE.md §11.
 *
 * RA_USB_PIPELINE (USB only): 1 = the per-frame SNAPSHOT is posted and its
 * reply collected at the next fire, so the ~6-10 ms of wire time overlaps
 * the game frame instead of blocking the module (the spike's "prod" shape:
 * ~60 us of thread time per frame). 0 = blocking round trip like EXI —
 * the A/B fallback if the pipelining ever misbehaves. */
#ifndef RA_LINK_USB
#define RA_LINK_USB  0
#endif

#ifndef RA_USB_PIPELINE
#define RA_USB_PIPELINE  1
#endif

#endif
