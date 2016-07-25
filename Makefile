##
## Copyright (c) 2016, Intel Corporation
## All rights reserved.
##
## Author: Jérémy Compostella <jeremy.compostella@intel.com>
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions
## are met:
##
##    * Redistributions of source code must retain the above copyright
##      notice, this list of conditions and the following disclaimer.
##    * Redistributions in binary form must reproduce the above copyright
##      notice, this list of conditions and the following disclaimer
##      in the documentation and/or other materials provided with the
##      distribution.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
## "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
## LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
## FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
## COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
## INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
## (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
## SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
## HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
## STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
## ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
## OF THE POSSIBILITY OF SUCH DAMAGE.
##

GNU_EFI_PATH ?= /build/rvigierx/android-master/external/gnu-efi/gnu-efi-3.0/inc

SOURCES := \
	main.c \
	core/protocol.c \
	core/efiwrapper.c \
	core/efivar.c \
	linux/common.c \
	linux/linux.c \
	linux/serial.c \
	linux/mmc.c \
	linux/runtime_services.c \
	linux/boot_services.c \
	linux/conout.c \
	linux/conin.c \
	linux/fileio.c \

all: installer kernelflinger

INSTALLER_LIB := lib/libinstaller.efi.a

$(INSTALLER_LIB):
	cd lib && ar -M < libinstaller.efi.mri && objcopy --redefine-syms  redef_syms libinstaller.efi.a

KERNELFLINGER_LIB := lib/libkernelflinger.efi.a

$(KERNELFLINGER_LIB):
	cd lib && ar -M < libkernelflinger.efi.mri && objcopy --redefine-syms  redef_syms libkernelflinger.efi.a

OBJS := ${SOURCES:.c=.o}

INCLUDES=-I$(GNU_EFI_PATH) -I$(GNU_EFI_PATH)/ia32 -Iinclude/ -Icore/

CFLAGS=-D_GNU_SOURCE -Wall -g -fshort-wchar -DGNU_EFI_USE_MS_ABI -m32 -D_FILE_OFFSET_BITS=64

installer: $(OBJS) $(INSTALLER_LIB)
	$(CC)  $(CFLAGS) $(INCLUDES) -o $@ $(OBJS) $(INSTALLER_LIB)

kernelflinger: $(OBJS) $(KERNELFLINGER_LIB)
	$(CC)  $(CFLAGS) $(INCLUDES) -o $@ $(OBJS) $(KERNELFLINGER_LIB)

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

clean:
	rm -f $(OBJS) installer $(INSTALLER_LIB) $(KERNELFLINGER_LIB)
