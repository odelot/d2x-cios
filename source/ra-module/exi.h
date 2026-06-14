#ifndef _EXI_H_
#define _EXI_H_

#include "types.h"

/* EXI channels (Wii Starlet view) */
#define EXI_CHAN_0 0  /* Memory Card Slot A */
#define EXI_CHAN_1 1  /* Memory Card Slot B */
#define EXI_CHAN_2 2  /* RTC / SRAM / boot rom */

/* Device select (CS) — bit positions in CPR register */
#define EXI_DEVICE_0 0  /* CS0 — memory card / first device on bus */
#define EXI_DEVICE_1 1
#define EXI_DEVICE_2 2

/* Frequencies — CPR bits 4..6 */
#define EXI_SPEED1MHZ  0x00000000
#define EXI_SPEED2MHZ  0x00000010
#define EXI_SPEED4MHZ  0x00000020
#define EXI_SPEED8MHZ  0x00000030
#define EXI_SPEED16MHZ 0x00000040
#define EXI_SPEED32MHZ 0x00000050

/* Mirrors mload-module/gecko.c:Gecko_Init() — clears CSR on all three EXI
 * channels and sets ROMDIS + interrupt masks. Call once, from a worker
 * thread (NOT module init: doing register writes at module load time has
 * been observed to hang IOS_ReloadIOS). Does NOT touch HW_EXICTRL — the EXI
 * bus master is already enabled by the time any cIOS module runs. */
void exi_init(void);
s32  exi_select(s32 chan, s32 dev, s32 freq);
s32  exi_deselect(s32 chan);
s32  exi_imm_write(s32 chan, const void *data, u32 len);
s32  exi_imm_read(s32 chan, void *data, u32 len);

/* Kernel-batched immediate transfers — same wire behavior as the
 * exi_imm_* loops but the per-chunk work runs inside MLOAD's SVC
 * handler, one syscall per 256-byte slice. No alignment constraints.
 * PREFER THESE over the plain exi_imm_* loops for anything bigger than
 * a header. (EXI DMA was tried and removed: the engine cannot master
 * MEM2 module memory from Starlet — see exi.c for the field lessons.) */
s32  exi_batch_write(s32 chan, const void *data, u32 len);
s32  exi_batch_read(s32 chan, void *data, u32 len);

/* Convenience: full half-duplex transaction.
 * Writes wlen bytes from wbuf, then reads rlen bytes into rbuf.
 * Returns 0 on success, negative on error. */
s32  exi_transaction(s32 chan, s32 dev, s32 freq,
                     const void *wbuf, u32 wlen,
                     void *rbuf, u32 rlen);


/* --- Phase B: device-INT handshake (slot pin 2 → ESP32 GPIO14) ---
 *
 * After ESP32 receives a request and prepares the response in its tx_buf,
 * it pulls slot pin 2 (INT) low. The Hollywood EXI controller latches
 * EXI_CSR bit 1 (EXI_EXI_IRQ, 0x002, RW1C, edge-triggered + sticky).
 *
 * exi_wait_int: poll EXI_CSR for the latched INT, with timeout in ms.
 *   Returns 0 on success, -1 on timeout. Does NOT clear the bit.
 *
 * exi_clear_int: write-1-to-clear the EXI_EXI_IRQ bit. Call AFTER
 *   exi_wait_int returns success, BEFORE the response read CS-low.
 *
 * See project_phase_b_design memory for protocol details. */
s32  exi_wait_int(s32 chan, u32 timeout_ms);
void exi_clear_int(s32 chan);

#endif
