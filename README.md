# d2x cIOS — RetroAchievements Fork

> **This is a fork of [wiidev/d2x-cios](https://github.com/wiidev/d2x-cios)** that adds a new IOS
> module — the **`ra-module`** — turning the custom IOS into the memory server that makes
> [RetroAchievements](https://retroachievements.org) possible for **Wii games** on real hardware.
>
> It is one of four cooperating projects; see the
> [**wii-ra-adapter**](https://github.com/odelot/wii-ra-adapter) repository for the system overview
> and for the **pre-built binaries of all four projects** (releases are published there).

---

## What this fork adds

A single new module, [`source/ra-module/`](source/ra-module/), plus its build/packaging hooks:

| File | Purpose |
|---|---|
| `source/ra-module/main.c` | The module itself: game-alive detection, watchlist state machine, pointer-chain walker, and the per-frame SNAPSHOT loop. |
| `source/ra-module/exi.c` | Starlet-side EXI driver for the real Hollywood controller (`0x0d806800`): kernel-batched immediate transfers through MLOAD SWIs, plus the INT-line ("Phase B") handshake. |
| `source/ra-module/vi.c` / `led.c` | HW_TIMER helpers and disc-slot LED control. |
| `source/ra-module/gc_ra_protocol.h` | Shared binary protocol (kept in sync with the copy in wii-ra-adapter). |
| `maked2x.sh` / `data/ciosmaps.xml` | `ra-module` is built as **RAMOD** and injected as an extra content into the **base IOS56 and IOS58** maps, so it ships inside the standard cIOS slots (e.g. 249). |

The module runs on **Starlet** (the Wii's ARM security processor) *inside* the cIOS, next to the
running game — which is what lets it read the game's RAM live, with zero game patching.

## How it works

### Safe activation: the game-alive gate

At boot the module does **nothing** — it silently polls a frame counter at physical MEM1 address
`0x2FF8`. That counter only advances when WiiFlow has injected its Gecko C0 VBlank hook and an
RA-enabled game is actually running. Until then the module never touches the EXI bus.

This single property is what makes it safe to install RAMOD into the standard cIOS slots
(248–251), including the mainIOS WiiFlow itself runs under: while WiiFlow browses menus the
counter never moves, so there is no bus contention with WiiFlow's own probe/`LOAD_GAME`
handshake. Boots without the adapter, or games where the hook pattern fails, also stay silent —
RA simply isn't active for those.

### Startup sequence (once the counter moves)

1. **IDENTIFY** — one round-trip to the ESP32 in memory-card **Slot B** (EXI channel 1, device 0)
   to confirm the link. The bus runs at **16 MHz** (32 MHz was hardware-tested and corrupts large
   transfers; 16 is the ceiling for memory-card wiring).
2. **Watchlist fetch** — the ESP32 has already loaded the achievement set (WiiFlow did the
   `LOAD_GAME` handshake before the IOS reload); the module pulls the list of RAM addresses to
   watch in chunks of up to 1024 (up to 6144 total).
3. **Chain table fetch** — a flat, dependency-ordered table describing every eligible
   *pointer chain* used by the achievement set ("Phase C"). An empty table simply leaves the
   feature off.

### The per-frame SNAPSHOT loop

Every frame, synchronized to the game's vertical retrace:

- **VBI-edge scheduling** — the module polls the PPC-side counter at `0x2FF8` (with an explicit
  cache invalidate before every read — Starlet maps PPC RAM cached and a tight poll pins the
  line) and fires on change. If the counter stalls for 2 frame-times (loading screens, missing
  hook) a timer fallback fires on time and disengages the moment the counter moves again. The
  counter value doubles as the true game-frame clock shipped to the ESP32, so rcheevos
  frame-based timers stay accurate.
- **Chain walk** — before sampling, the module *walks the pointer chains itself* using the
  Phase C table: dereferences roots in MEM1/MEM2, applies offsets/masks/arithmetic per node, and
  collects always-fresh leaf values. Per-node previous/prior value histories ("Phase D") let
  chains that reference rcheevos *delta*/*prior* values resolve on-console too — sound because
  the ESP32 queues snapshots so every walked frame is evaluated exactly once, making the
  walker's previous-walk value the evaluator's delta by definition. Verification values ship
  with each snapshot so the ESP32 can detect and self-heal any divergence.
- **SNAPSHOT** — the flat watched bytes + the chain window (rotating so everything fits one 8 KB
  EXI transaction) + validity bitmaps are sent in a single INT-handshake ("Phase B") transaction:
  the ESP32 asserts the memory card INT pin when its response is ready, and Hollywood latches
  that edge in the CSR for the module to poll — no fragile timing assumptions.
- **Convergence rounds** — the ESP32 may answer with `ADDR_QUERY` (bytes it still needs) or
  watchlist mutations; the module drains up to 12 rounds per frame before waiting for the next
  edge.
- **Watchlist mutations** — the ESP32 grows/shrinks the list incrementally at runtime
  (`APPEND` / `REMOVE_IDX` by array index). Every mutation carries a sequence number verified on
  both sides each frame; a behind replica gets exactly the next mutation resent, an impossible
  state triggers a full resync. This keeps the two copies of the watchlist positionally aligned —
  the snapshot is just bytes, its meaning comes from that alignment.

### Unlock celebration

On an achievement event the module blinks the **disc-slot LED** (3 pulses) and raises a trophy
flag at MEM1 `0x2FFC`; WiiFlow's extended VBlank hook reads that flag on the PPC and draws a
32×32 gold trophy badge on screen, blinking in lockstep with the LED.

### Notes for porters: life as a user-mode IOS module

Things this port had to discover the hard way, preserved here so nobody rediscovers them:

- IOS modules run in **user mode** — every Hollywood register access must go through an MLOAD
  SWI or the kernel faults; hot paths use batched SWIs that run whole transfer slices inside one
  SVC. (The Nintendont port of this module runs in kernel mode and needs none of this.)
- The module loader does not load `.rodata` — string literals read as zeros; initialized data
  must live on the stack or be built at runtime.
- Thread priorities are load-bearing: the IPC thread must stay at 0x48 (above it, boot breaks);
  IOS refuses to spawn a child thread with higher priority than its parent, so the main thread
  is born high (in `start.s`), spawns the poll thread, then lowers itself.
- The VI interrupt does not exist on the ARM side (Hollywood's IRQ controller has no VI line) —
  hence the PPC-side C0 hook + counter-polling design.
- The legacy EXI DMA engine cannot master MEM2 module memory from Starlet — immediate transfers,
  batched in the kernel, are the proven replacement.

## Building

Same as upstream: `./maked2x.sh` (or the Docker wrapper `./docker-build.sh`). The RAMOD module is
built and packaged automatically via `data/ciosmaps.xml`. Install the resulting cIOS with the
usual d2x installer flow — pre-built WADs are published in the
[wii-ra-adapter releases](https://github.com/odelot/wii-ra-adapter/releases).

---

# Original d2x cIOS README

<p align="center"><a href="https://github.com/wiidev/d2x-cios/" title="d2x cIOS"><img src="assets/d2x-logo.jpg"></a></p>
<p align="center">
<a href="https://github.com/wiidev/d2x-cios/releases" title="Releases"><img src="https://img.shields.io/github/v/release/wiidev/d2x-cios?logo=github"></a>
<a href="https://github.com/wiidev/d2x-cios/actions" title="Actions"><img src="https://img.shields.io/github/actions/workflow/status/wiidev/d2x-cios/main.yml?branch=master&logo=github"></a>
</p>
<hr>


#### DISCLAIMER

````
THIS APPLICATION COMES WITH NO WARRANTY AT ALL, NEITHER EXPRESSED NOR IMPLIED.
NO ONE BUT YOURSELF IS RESPONSIBLE FOR ANY DAMAGE TO YOUR WII CONSOLE BECAUSE OF A IMPROPER USAGE OF THIS SOFTWARE.
````



#### DESCRIPTION

  This is a custom IOS for the Wii console, i.e. an IOS modified to add some new features
  not available in the official IOS.

  This custom IOS has been made to be used ONLY with homebrew software.

  The d2x cIOS is an enhanced version of the cIOSX rev21 by Waninkoko. 



#### DOCUMENTATION

  For documentation and tutorials visit our [Wiki](https://github.com/wiidev/d2x-cios/wiki).



#### KUDOS

 * *rodries*, for the help with EHCI improvements.
 * *Crediar*, for all I learned studying [Sneek](http://code.google.com/p/sneek) source code.
 * *Oggzee*, for his brilliant fraglist.
 * *WiiPower*, for the great help with ios reload block from usb.
 * *dragbe* and *NutNut*, for their [d2x cIOS Installer](http://code.google.com/p/d2x-cios-installer).
 * *XFlak*, for his wonderful [ModMii](http://gbatemp.net/topic/207126-modmii-for-windows) which supported d2x wads since its birth. Without ModMii d2x cIOS would probably never have existed. Also, XFlak had the original idea to replace the buggy EHCI module of cIOSX rev21 with the  working one from rev19. 
 * *[HackWii](http://www.hackwii.it)* and *[GBAtemp](http://www.gbatemp.net)* communities, for their ideas and support.
 * *Totoro*, for the official d2x logo
 * *ChaN*, for his [FatFs](http://elm-chan.org/fsw/ff/00index_e.html).
 * *Waninkoko*, for his cIOSX rev21.
 * *Team Twiizers* and *devkitPRO devs* for their great work in libogc.
 * *WiiGator*, for his work in the DIP plugin.
 * *kwiirk*, for his EHCI module.
 * *Hermes*, for his EHCI improvements.
 * *neimod*, for his [Custom IOS Module Toolkit](http://wiibrew.org/wiki/Custom_IOS_Module_Toolkit).
 * All the betatesters.
