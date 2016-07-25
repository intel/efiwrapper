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

include ../../coreboot/payloads/libpayload/.xcompile

ARCH-$(CONFIG_LP_ARCH_X86)     := i386
GNU_EFI_PATH ?= /build/rvigierx/android-master/external/gnu-efi/gnu-efi-3.0/inc
SRC_BASE_PATH := ./base
DRV_STORAGE_PATH := ./drivers/storage
DRV_BUS_USB_PATH := ./drivers/bus/usb

KPI_IMAGE ?= ./tools/scripts/kpImage.py
CC := $(CC_$(ARCH-y))
AS := $(AS_$(ARCH-y))
LIBPAYLOAD_DIR := ../../coreboot/payloads/libpayload/
XCC := CC="$(CC)" $(LIBPAYLOAD_DIR)/bin/lpgcc
XAS := AS="$(AS)" $(LIBPAYLOAD_DIR)/bin/lpas
CFLAGS := 	-fno-builtin -nostdinc -Wall -Os -fshort-wchar -DGNU_EFI_USE_MS_ABI -std=gnu99 \
		-I../include -I../core -I./protocol -I$(GNU_EFI_PATH) -I$(GNU_EFI_PATH)/ia32 -I$(SRC_BASE_PATH) \
		-I$(DRV_STORAGE_PATH) -I$(DRV_BUS_USB_PATH) -I.
TARGET := main

MMC :=  ./drivers/storage/regaccess_mmio.o \
        ./drivers/storage/sdhci-bxt.o \
	./drivers/storage/mmc.o \
	./drivers/storage/blockdev.o \
	./mmc.o \

OBJS := $(TARGET).o \
	$(MMC) \
	../core/protocol.o \
	../core/efiwrapper.o \
	../core/efivar.o \
	./boot_services.o \
	./runtime_services.o \
	./conin.o \
	./conout.o \
	./serial.o \
	./drivers/bus/usb/XdciUtility.o \
	./drivers/bus/usb/XdciDWC.o \
	./usb.o \
	./smbios.o

LIBS := ../lib/libkernelflinger.efi.a

all: $(TARGET).elf
	$(KPI_IMAGE) -t 4 -o $(TARGET).bin $(TARGET).elf

$(TARGET).elf: $(OBJS)
	$(XCC) -o $@ $(OBJS) $(LIBS)

%.o: %.c
	$(XCC) $(CFLAGS) -c -o $@ $<

%.S.o: %.S
	$(XAS) --32 -o $@ $<

clean:
	rm -f $(TARGET).elf *.o base/*.o drivers/storage/*.o drivers/bus/usb/*.o $(TARGET).bin

distclean: clean
