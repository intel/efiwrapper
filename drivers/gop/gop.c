/*
 * Copyright (c) 2018-2020, Intel Corporation
 * All rights reserved.
 *
 * Authors: Jérémy Compostella <jeremy.compostella@intel.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer
 *      in the documentation and/or other materials provided with the
 *      distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <interface.h>
#include <efiwrapper.h>
#include <string.h>
#include <ewlib.h>
#include <pci/pci.h>
#include <arch/io.h>
#include <hwconfig.h>

#include "gop.h"

typedef struct gop {
	EFI_GRAPHICS_OUTPUT_PROTOCOL prot;
	pcidev_t dev;
	uint8_t *fb;
	size_t pipe;
} gop_t;

static EFI_GUID gop_guid = EFI_GRAPHICS_OUTPUT_PROTOCOL_GUID;
static EFI_HANDLE gop_handle;
static gop_t *gop;

static EFI_GRAPHICS_OUTPUT_MODE_INFORMATION info = {
	.Version = 1,
	.PixelFormat = PixelRedGreenBlueReserved8BitPerColor
};

static EFI_GRAPHICS_OUTPUT_PROTOCOL_MODE mode = {
	.MaxMode = 1,
	.Info = &info,
	.SizeOfInfo = sizeof(info)
};

static EFIAPI EFI_STATUS
query_mode(EFI_GRAPHICS_OUTPUT_PROTOCOL *This,
	   UINT32 ModeNumber,
	   UINTN *SizeOfInfo,
	   EFI_GRAPHICS_OUTPUT_MODE_INFORMATION **Info)
{
	if (!This || ModeNumber != 0 || !SizeOfInfo || !Info)
		return EFI_INVALID_PARAMETER;

	*SizeOfInfo = sizeof(info);
	*Info = &info;

	return EFI_SUCCESS;
}

#define PIPE_NUMBER		3
#define PIPE_OFFSET		0x1000
#define PLANE_SIZE		0x70190
#define PLANE_SURF		0x7019C
#define PLANE_CTL		0x70180
#define PLANE_CTL_ORDER_RGBX	(1 << 20)

/*
 * It assumes that the BIOS (ABL) configured the monitors and setup
 * Surface 1 of PIPE A appropriately.  The two following function are
 * reading back register set by the BIOS to determine the surface
 * resolution and surface address.
 *
 * Registers information are available on 01.org in:
 * - intel-gfx-prm-osrc-skl-vol02c-commandreference-registers-part1.pdf
 * - intel-gfx-prm-osrc-skl-vol02c-commandreference-registers-part2.pdf
 */
static void get_resolution(pcidev_t dev, UINT32 *width, UINT32 *height,
			   size_t *pipe)
{
	unsigned long mmio, size;
	size_t i;

	mmio = pci_read_config32(dev, 0x10) & ~0xf;
	for (i = 0; i < PIPE_NUMBER; i++) {
		uint32_t offset = i * PIPE_OFFSET;
		size = read32((void *)mmio + PLANE_SIZE + offset);
		if (!size)
			continue;
		*pipe = i;
		*width = (size & 0xffff) + 1;
		*height = (size >> 16) + 1;
		return;
	}
}

static uint8_t *get_framebuffer(pcidev_t dev, size_t pipe)
{
	unsigned long mmio, gmaddr, surf;

	mmio = pci_read_config32(dev, 0x10) & ~0xf;
	surf = read32((void *)(mmio + PLANE_SURF + pipe * PIPE_OFFSET));
	gmaddr = pci_read_config32(dev, 0x18) & ~0xf;

	return (uint8_t *)(gmaddr + surf);
}

static void set_efi_color_order(pcidev_t dev)
{
	unsigned long mmio, ctl, surf;
	size_t i;

	mmio = pci_read_config32(dev, 0x10) & ~0xf;
	for (i = 0; i < PIPE_NUMBER; i++) {
		uint32_t offset = i * PIPE_OFFSET;
		ctl = read32((void *)(mmio + PLANE_CTL + offset));
		if (ctl & PLANE_CTL_ORDER_RGBX) {
			write32((void *)(mmio + PLANE_CTL + offset),
				ctl & ~PLANE_CTL_ORDER_RGBX);
			surf = read32((void *)(mmio + PLANE_SURF + offset));
			write32((void *)(mmio + PLANE_SURF + offset), surf);
		}
	}
}

static EFIAPI EFI_STATUS
set_mode(EFI_GRAPHICS_OUTPUT_PROTOCOL *This,
	 UINT32 ModeNumber)
{
	if (!This || (gop_t *)This != gop || ModeNumber != 0)
		return EFI_INVALID_PARAMETER;

	gop->fb = get_framebuffer(gop->dev, gop->pipe);
	set_efi_color_order(gop->dev);
	return gop->fb ? EFI_SUCCESS : EFI_DEVICE_ERROR;
}

static void fill_buffer(EFI_GRAPHICS_OUTPUT_BLT_PIXEL *color,
			UINTN destx,
			UINTN desty,
			UINTN width,
			UINTN height)
{
	size_t x, y;
	EFI_GRAPHICS_OUTPUT_BLT_PIXEL *data;

	data = (EFI_GRAPHICS_OUTPUT_BLT_PIXEL *)gop->fb;
	for (y = desty; y < desty + height; y++) {
		UINTN cur_y = y * info.HorizontalResolution;
		for (x = destx; x < destx + width; x++)
			data[cur_y + x] = *color;
	}
}

static void copy_to_buffer(EFI_GRAPHICS_OUTPUT_BLT_PIXEL *buffer,
			   UINTN srcx,
			   UINTN srcy,
			   UINTN destx,
			   UINTN desty,
			   UINTN width,
			   UINTN height)
{
	size_t i, y, sy;
	EFI_GRAPHICS_OUTPUT_BLT_PIXEL *data, *dst, *src;

	data = (EFI_GRAPHICS_OUTPUT_BLT_PIXEL *)gop->fb;
	for (y = desty, sy = srcy; y < desty + height; y++, sy++) {
		dst = &data[y * info.HorizontalResolution + destx];
		src = &buffer[sy * width + srcx];
		for (i = 0; i < width; i++)
			dst[i] = src[i];
	}
}

static EFIAPI EFI_STATUS
blt(EFI_GRAPHICS_OUTPUT_PROTOCOL *This,
    EFI_GRAPHICS_OUTPUT_BLT_PIXEL *BltBuffer,
    EFI_GRAPHICS_OUTPUT_BLT_OPERATION BltOperation,
    UINTN SourceX,
    UINTN SourceY,
    UINTN DestinationX,
    UINTN DestinationY,
    UINTN Width,
    UINTN Height,
    __attribute__((__unused__)) UINTN Delta)
{
	if (!This || (gop_t *)This != gop || !BltBuffer)
		return EFI_INVALID_PARAMETER;

	if (DestinationX + Width > info.HorizontalResolution ||
	    DestinationY + Height > info.VerticalResolution)
		return EFI_INVALID_PARAMETER;

	if (!gop->fb)
		return EFI_NOT_STARTED;

	switch (BltOperation) {
	case EfiBltVideoFill:
		fill_buffer(BltBuffer, DestinationX, DestinationY,
			    Width, Height);
		break;

	case EfiBltBufferToVideo:
		copy_to_buffer(BltBuffer, SourceX, SourceY,
			       DestinationX, DestinationY,
			       Width, Height);
		break;

	default:
		return EFI_UNSUPPORTED;
	}

	return EFI_SUCCESS;
}

static struct supported_device {
	u16 vid;
	u16 did;
} SUPPORTED_DEVICES[] ={
	{ .vid = 0x8086, .did = VGA_PID },
	{ .vid = 0x8086, .did = VGA_PID2 },
};

static EFI_STATUS gop_init(EFI_SYSTEM_TABLE *st)
{
	static gop_t gop_default = {
		.prot = {
			.QueryMode = query_mode,
			.SetMode = set_mode,
			.Blt = blt,
			.Mode = &mode
		}
	};
	pcidev_t pci_dev = 0;
	size_t i;
	EFI_STATUS ret;

	if (!st)
		return EFI_INVALID_PARAMETER;

	if (gop_handle)
		return EFI_ALREADY_STARTED;

	for (i = 0; i < ARRAY_SIZE(SUPPORTED_DEVICES); i++)
		if (pci_find_device(SUPPORTED_DEVICES[i].vid,
				    SUPPORTED_DEVICES[i].did,
				    &pci_dev))
			break;

	if (!pci_dev)
		return EFI_UNSUPPORTED;

	gop_default.dev = pci_dev;
	get_resolution(pci_dev, &info.HorizontalResolution,
		       &info.VerticalResolution,
		       &gop_default.pipe);

	/* If the BIOS has not programmed any surface, silently
	   disable the Graphical Output Protocol. */
	if (!info.HorizontalResolution || !info.VerticalResolution)
		return EFI_SUCCESS;

	ret = interface_init(st, &gop_guid, &gop_handle,
			     &gop_default, sizeof(gop_default),
			     (void **)&gop);
	if (EFI_ERROR(ret))
		return ret;

	set_mode((EFI_GRAPHICS_OUTPUT_PROTOCOL *)gop, 0);
	return EFI_SUCCESS;
}

static EFI_STATUS gop_exit(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;

	if (!st)
		return EFI_INVALID_PARAMETER;

	if (!gop_handle)
		return EFI_SUCCESS;

	ret = interface_free(st, &gop_guid, gop_handle);
	if (EFI_ERROR(ret))
		return ret;

	gop = NULL;
	gop_handle = NULL;

	return EFI_SUCCESS;
}

ewdrv_t gop_drv = {
	.name = "gop",
	.description = "Graphics Output Protocol",
	.init = gop_init,
	.exit = gop_exit
};
