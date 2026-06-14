OUTPUT_FORMAT("elf32-bigarm")
OUTPUT_ARCH(arm)
ENTRY(_start)

/* Sections area */
MEMORY {
	table		: ORIGIN = 0x0, LENGTH = 0x4000
	/* Sized to actual usage. Code is at 0x136dc000, plenty of headroom */
	exe(rwx)	: ORIGIN = 0x136dc000, LENGTH = 0x4000
	/* ram runs up to 0x13700000, where MLOAD's 512KB static buffer
	 * begins ([[project-ra-module-memory-layout]]) — the entire
	 * 0x136e0000..0x13700000 gap is ours. 0x20000 = 128KB. The old
	 * 0x8000 (32KB) overflowed when RA_MAX_WATCH_ADDRS went to 4096
	 * (v0.24.5: watchlist 16KB + tx buf 4KB + to_remove 4KB + stacks). */
	ram(rw)		: ORIGIN = 0x136e0000, LENGTH = 0x20000
}

__exe_start_phys__		= 0x136dc000;
__ram_start_phys__		= 0x136e0000;

SECTIONS {
	.ios_info_table : {
		KEEP (*(.ios_info_table))
	} > table

	.init : AT (__exe_start_phys__) {
		*(.init)
		. = ALIGN(4);
	} > exe

	.text : { 
		*(.text*) 
		*(.gnu.warning) 
		*(.gnu.linkonce.t.*) 
		*(.init) 
		*(.glue_7) 
		*(.glue_7t)
		. = ALIGN(4);
	} > exe

	.data : AT (__ram_start_phys__) {
		*(.data*)
		*(.data.*)
		*(.gnu.linkonce.d.*)
		. = ALIGN(4);
	} > ram

	.rodata : { 
		*(.rodata)
		*all.rodata*(*)
		*(.roda)
		*(.rodata.*) 
		*(.gnu.linkonce.r.*)
		. = ALIGN(4);
	} > ram

	.bss : {
		*(.dynsbss)
		*(.gnu.linkonce.sb.*)
		*(.bss*)
		*(COMMON)
		KEEP(*(.ios_bss))
		. = ALIGN(4);
	} > ram
}
