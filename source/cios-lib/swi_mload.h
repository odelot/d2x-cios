/*   
	Custom IOS Library

	Copyright (C) 2008 neimod.
	Copyright (C) 2009 WiiGator.
	Copyright (C) 2009 Waninkoko.
	Copyright (C) 2010 Hermes.
	Copyright (C) 2011 davebaol.

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _SWI_MLOAD_H_
#define _SWI_MLOAD_H_

#include "ios.h"
#include "tools.h"
#include "types.h"

typedef s32(*TRCheckFunc)(s32 tid, u32 rights);

/* Macros */
#define Swi_SetRegister(A, V)                         Swi_MLoad(  5, (A), (V), 0)
#define Swi_ClearRegister(A, V)                       Swi_MLoad(  6, (A), (V), 0)
/* Atomic EXI DATA+CR write — writes both registers back-to-back inside
 * the SVC handler (IRQs masked), avoiding the multi-CR-write corruption
 * we hit when DATA and CR went through two separate SWI calls with IRQs
 * re-enabled in between. CHAN=0..2, DATA_VAL = up to 4 bytes packed BE,
 * CR_VAL must include TSTART and encode TLEN+RW. */
#define Swi_ExiDataCr(CHAN, DATA_VAL, CR_VAL)         Swi_MLoad(  7, (CHAN), (DATA_VAL), (CR_VAL))
/* Atomic EXI CR-trigger + bounded-poll + DATA-read. Mirror of
 * Swi_ExiDataCr but for the read path: the kernel handler keeps IRQs
 * masked through the entire trigger → completion → sample window so
 * nothing can disturb the EXI bus between the moment the transfer
 * finishes and the DATA register is captured. Returns the 4-byte
 * DATA contents packed big-endian (top byte = first byte read). */
#define Swi_ExiCrRead(CHAN, CR_VAL)                   Swi_MLoad(  8, (CHAN), (CR_VAL), 0)
/* EXI immediate-mode BATCH transfers — the entire buffer loop (pack,
 * DATA+CR trigger, bounded poll per 4-byte chunk) runs inside ONE SVC.
 * From user mode the same loop costs ~5 SVC round-trips per chunk
 * (trigger + CR polls), i.e. ~1000 SVCs for a 790-byte write; batched
 * it's len/256 SVCs. Caller should slice into <=256-byte calls to bound
 * the IRQs-masked window (~260us per call at 8MHz — see mload swi.c).
 * Returns 0 on success, negative if a chunk timed out. */
#define Swi_ExiBatchWrite(CHAN, SRC, LEN)             Swi_MLoad( 10, (CHAN), (u32)(SRC), (LEN))
#define Swi_ExiBatchRead(CHAN, DST, LEN)              Swi_MLoad( 11, (CHAN), (u32)(DST), (LEN))
#define Swi_GetSyscallBase()                          Swi_MLoad( 17,   0,   0, 0)
#define Swi_SetRunningTitle(V)                        Swi_MLoad( 32, (V),   0, 0)
#define Swi_GetRunningTitle()                         Swi_MLoad( 33,   0,   0, 0)
#define Swi_SetEsRequest(V)                           Swi_MLoad( 34, (V),   0, 0)
#define Swi_GetEsRequest()                            Swi_MLoad( 35,   0,   0, 0)
#define Swi_AddThreadRights(T, R)                     Swi_MLoad( 36, (T), (R), 0)
#define Swi_GetThreadRightsCheckFunc()  ((TRCheckFunc)Swi_MLoad( 37,   0,   0, 0))
#define Swi_LedOn()                                   Swi_MLoad(128,   0,   0, 0)
#define Swi_LedOff()                                  Swi_MLoad(129,   0,   0, 0)
#define Swi_LedBlink()                                Swi_MLoad(130,   0,   0, 0)

/* Prototypes */
void Swi_Memcpy(void *dst, void *src, s32 len);
void Swi_uMemcpy(void *dst, void *src, s32 len);
s32  Swi_CallFunc(s32 (*func)(void *in, void *out), void *in, void *out);
u32  Swi_GetIosInfo(iosInfo *buffer);

#endif
