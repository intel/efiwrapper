/*
 * Copyright (c) 2020, Intel Corporation
 * All rights reserved.
 *
 * Author: Jérémy Compostella <jeremy.compostella@intel.com>
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

#include <efi.h>
#include <ewlog.h>
#include <interface.h>
#include <kconfig.h>
#include <libpayload.h>

#include "lppci/lppci.h"
#include "lppci/PciRootBridgeIo.h"

static EFIAPI EFI_STATUS
lppci_poll_mem(__attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL *This,
	       __attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL_WIDTH Width,
	       __attribute__((__unused__)) UINT64 Address,
	       __attribute__((__unused__)) UINT64 Mask,
	       __attribute__((__unused__)) UINT64 Value,
	       __attribute__((__unused__)) UINT64 Delay,
	       __attribute__((__unused__)) UINT64 *Result)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
lppci_poll_io(__attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL *This,
	      __attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL_WIDTH Width,
	      __attribute__((__unused__)) UINT64 Address,
	      __attribute__((__unused__)) UINT64 Mask,
	      __attribute__((__unused__)) UINT64 Value,
	      __attribute__((__unused__)) UINT64 Delay,
	      __attribute__((__unused__)) UINT64 *Result)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
lppci_access_mem_read(__attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL *This,
		      __attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL_WIDTH Width,
		      __attribute__((__unused__)) UINT64 Address,
		      __attribute__((__unused__)) UINTN Count,
		      __attribute__((__unused__)) VOID *Buffer)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
lppci_access_mem_write(__attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL *This,
		       __attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL_WIDTH Width,
		       __attribute__((__unused__)) UINT64 Address,
		       __attribute__((__unused__)) UINTN Count,
		       __attribute__((__unused__)) VOID *Buffer)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
lppci_access_io_read(__attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL *This,
		     __attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL_WIDTH Width,
		     __attribute__((__unused__)) UINT64 Address,
		     __attribute__((__unused__)) UINTN Count,
		     __attribute__((__unused__)) VOID *Buffer)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
lppci_access_io_write(__attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL *This,
		      __attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL_WIDTH Width,
		      __attribute__((__unused__)) UINT64 Address,
		      __attribute__((__unused__)) UINTN Count,
		      __attribute__((__unused__)) VOID *Buffer)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
lppci_access_pci_read(__attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL *This,
		      EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL_WIDTH Width,
		      UINT64 Address,
		      UINTN Count,
		      VOID *Buffer)
{
	EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL_PCI_ADDRESS *addr;
	pcidev_t dev;
	UINTN i;
	uint16_t reg;

	addr = (EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL_PCI_ADDRESS *)&Address;
	dev = PCI_DEV(addr->Bus, addr->Device, addr->Function);
	reg = addr->Register;

	for (i = 0; i < Count; i++) {
		switch (Width) {
		case EfiPciWidthUint8:
			((uint8_t *)Buffer)[i] = pci_read_config8(dev, addr->Register);
			reg++;
			break;

		case EfiPciWidthUint16:
			((uint16_t *)Buffer)[i] = pci_read_config16(dev, addr->Register);
			reg += 2;
			break;

		case EfiPciWidthUint32:
			((uint32_t *)Buffer)[i] = pci_read_config32(dev, addr->Register);
			reg += 4;
			break;

		default:
			ewerr("pci read width %d is not supported", Width);
			return EFI_UNSUPPORTED;
		}
	}

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
lppci_access_pci_write(__attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL *This,
		       __attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL_WIDTH Width,
		       __attribute__((__unused__)) UINT64 Address,
		       __attribute__((__unused__)) UINTN Count,
		       __attribute__((__unused__)) VOID *Buffer)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
lppci_copy_mem(__attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL *This,
	       __attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL_WIDTH Width,
	       __attribute__((__unused__)) UINT64 DestAddress,
	       __attribute__((__unused__)) UINT64 SrcAddress,
	       __attribute__((__unused__)) UINTN Count)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
lppci_map(__attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL *This,
	  __attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL_OPERATION Operation,
	  __attribute__((__unused__)) VOID *HostAddress,
	  __attribute__((__unused__)) UINTN *NumberOfBytes,
	  __attribute__((__unused__)) EFI_PHYSICAL_ADDRESS *DeviceAddress,
	  __attribute__((__unused__)) VOID **Mapping)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
lppci_unmap(__attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL *This,
	    __attribute__((__unused__)) VOID *Mapping)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
lppci_allocate_buffer(__attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL *This,
		      __attribute__((__unused__)) EFI_ALLOCATE_TYPE Type,
		      __attribute__((__unused__)) EFI_MEMORY_TYPE MemoryType,
		      __attribute__((__unused__)) UINTN Pages,
		      __attribute__((__unused__)) VOID **HostAddress,
		      __attribute__((__unused__)) UINT64 Attributes)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
lppci_free_buffer(__attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL *This,
		  __attribute__((__unused__)) UINTN Pages,
		  __attribute__((__unused__)) VOID *HostAddress)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
lppci_flush(__attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL *This)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
lppci_get_attributes(__attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL *This,
		     __attribute__((__unused__)) UINT64 *Supports,
		     __attribute__((__unused__)) UINT64 *Attributes)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
lppci_set_attributes(__attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL *This,
		     __attribute__((__unused__)) UINT64 Attributes,
		     __attribute__((__unused__)) UINT64 *ResourceBase,
		     __attribute__((__unused__)) UINT64 *ResourceLength)
{
	return EFI_UNSUPPORTED;
}


static EFIAPI EFI_STATUS
lppci_configuration(__attribute__((__unused__)) EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL *This,
		    __attribute__((__unused__)) VOID **Resources)
{
	return EFI_UNSUPPORTED;
}

static EFI_GUID guid = EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL_GUID;
static EFI_HANDLE handle;

static EFI_STATUS lppci_init(EFI_SYSTEM_TABLE *st)
{
	static EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL pciroot_default = {
		.PollMem = lppci_poll_mem,
		.PollIo = lppci_poll_io,
		.Mem = {
			.Read = lppci_access_mem_read,
			.Write = lppci_access_mem_write
		},
		.Io = {
			.Read = lppci_access_io_read,
			.Write = lppci_access_io_write
		},
		.Pci = {
			.Read = lppci_access_pci_read,
			.Write = lppci_access_pci_write
		},
		.CopyMem = lppci_copy_mem,
		.Map = lppci_map,
		.Unmap = lppci_unmap,
		.AllocateBuffer = lppci_allocate_buffer,
		.FreeBuffer = lppci_free_buffer,
		.Flush = lppci_flush,
		.GetAttributes = lppci_get_attributes,
		.SetAttributes = lppci_set_attributes,
		.Configuration = lppci_configuration
	};
	EFI_PCI_ROOT_BRIDGE_IO_PROTOCOL *pciroot;

	if (!st)
		return EFI_INVALID_PARAMETER;

	return interface_init(st, &guid, &handle, &pciroot_default,
			      sizeof(pciroot_default), (void **)&pciroot);
}

static EFI_STATUS lppci_exit(EFI_SYSTEM_TABLE *st)
{
	return interface_free(st, &guid, handle);
}

ewdrv_t lppci_drv = {
	.name = "lppci",
	.description = "PCI Root Bridge IO Protocol",
	.init = lppci_init,
	.exit = lppci_exit
};

