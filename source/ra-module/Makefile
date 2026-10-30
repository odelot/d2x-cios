# devkitARM path
DEVKITARM ?= /opt/devkitARM

# Prefix
PREFIX	=	$(DEVKITARM)/bin/arm-eabi-

# Executables
CC	=	$(PREFIX)gcc
LD	=	$(PREFIX)gcc
STRIP	?=	../../stripios

# Strip address
SADDR	?=	0x136e0000

# Flags
ARCH	=	-mcpu=arm926ej-s -mthumb -mthumb-interwork -mbig-endian
CFLAGS	=	$(ARCH) -I. -I../cios-lib -ffreestanding -fno-builtin -ffunction-sections -fomit-frame-pointer -Os -Wall -Wno-builtin-macro-redefined
LDFLAGS	=	$(ARCH) -nostartfiles -nostdlib -Wl,-T,link.ld,-Map,$(TARGET).map -Wl,--gc-sections -Wl,-static

# `strip $(SADDR)` truncates the whole RAM segment (SADDR = its start), so
# .data and .rodata reach the Wii as ZEROS ([[project-ra-module-rodata-not-
# loaded]]). Production keeps it: RA builds its strings byte by byte, and its
# svc_write() calls only stay harmless because their text is zeroed (a real
# string would spin in MLOAD's Gecko send loop on Slot B and hang the boot).
STRIP_ARGS	=	strip $(SADDR)

# USB transport spike: `make USBPROBE=1` (or USBPROBE=1 in the environment)
# builds RAMOD as the stand-alone probe module — see ra_build.h. It is
# stripped like every other d2x module (USBS, FAT, DIPP...): no SADDR cut, so
# the RAM segment carries .data/.rodata and the exe segment stays code-only.
ifeq ($(USBPROBE),1)
CFLAGS	+=	-DRA_BUILD_USB_PROBE=1
STRIP_ARGS	=
endif

# RA module over USB instead of EXI: `make USB=1` (see ra_build.h,
# usb_link.c). Also stripped without the SADDR cut: usb_link.c needs its
# device paths (.rodata), and main.c neutralizes svc_write for this build.
ifeq ($(USB),1)
CFLAGS	+=	-DRA_LINK_USB=1
STRIP_ARGS	=
endif

# Libraries
LIBS	=	../cios-lib/cios-lib.a

# Target
TARGET	=	ra-module

# Objects
OBJS	=	start.o \
		main.o \
		exi.o \
		vi.o \
		led.o \
		usb_link.o \
		usbprobe.o


$(TARGET).elf: $(OBJS) $(LIBS)
	@echo -e " LD\t$@"
	@$(LD) $(LDFLAGS) $(OBJS) $(LIBS) -lgcc -o $@.orig
	@$(STRIP) $@.orig $@ $(STRIP_ARGS)

%.o: %.s
	@echo -e " CC\t$@"
	@$(CC) $(CFLAGS) -D_LANGUAGE_ASSEMBLY -c -x assembler-with-cpp -o $@ $<

%.o: %.c
	@echo -e " CC\t$@"
	@$(CC) $(CFLAGS) -c -o $@ $<

clean:
	@echo -e "Cleaning..."
	@rm -f $(OBJS) $(TARGET).elf $(TARGET).elf.orig $(TARGET).map
