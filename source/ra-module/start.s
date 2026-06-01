/*
	RA module — minimal IOS module entry point.
	Based on dip-plugin/start.s (the cleanest minimal d2x module).
	Copyright (C) 2008 neimod.
	Adapted for RA module.
*/

	.section ".init"
	.arm

	.EQU	ios_thread_arg,		4
	.EQU	ios_thread_priority,	0x48     /* same as dip-plugin / EHCI rev18 */
	.EQU	ios_thread_stacksize,	0x1000   /* 4KB stack */


	.global _start
_start:
	mov	r0, #0		@ int argc
	mov	r1, #0		@ char *argv[]
	ldr	r3, =main
	bx	r3



/*
 * IOS bss
 */
	.section ".ios_bss", "a", %nobits

	.space	ios_thread_stacksize
	.global ios_thread_stack	/* stack decrements from high address.. */
ios_thread_stack:


/*
 * IOS info table
 */
	.section ".ios_info_table", "ax", %progbits

	.global ios_info_table
ios_info_table:
	.long	0x0
	.long	0x28			@ numentries * 0x28
	.long	0x6

	.long	0xB
	.long	ios_thread_arg		@ passed to thread entry func, maybe module id

	.long	0x9
	.long	_start

	.long	0x7D
	.long	ios_thread_priority

	.long	0x7E
	.long	ios_thread_stacksize

	.long	0x7F
	.long	ios_thread_stack
