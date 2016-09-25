/*
 * Copyright (c) 2016, Intel Corporation
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

#include <kconfig.h>
#include <libpayload-config.h>
#include <libpayload.h>

#include "lpmemmap/lpmemmap.h"

static UINTN efimemmap_nb;
static EFI_MEMORY_DESCRIPTOR *efimemmap;

#define E820_RAM          1
#define E820_RESERVED     2
#define E820_ACPI         3
#define E820_NVS          4
#define E820_UNUSABLE     5

static EFI_STATUS e820_to_efi(unsigned int e820, UINT32 *efi)
{
	switch (e820) {
	case E820_RAM:
		*efi = EfiConventionalMemory;
		return EFI_SUCCESS;

	case E820_RESERVED:
		*efi = EfiReservedMemoryType;
		return EFI_SUCCESS;

	case E820_ACPI:
		*efi = EfiACPIReclaimMemory;
		return EFI_SUCCESS;

	case E820_NVS:
		*efi = EfiACPIMemoryNVS;
		return EFI_SUCCESS;

	case E820_UNUSABLE:
		*efi = EfiUnusableMemory;
		return EFI_SUCCESS;

	default:
		return EFI_NOT_FOUND;
	}
}

static EFI_STATUS lpmemmap_to_efimemmap(struct memrange *ranges, size_t nb)
{
	EFI_STATUS ret;
	size_t i;

	efimemmap = malloc(nb * sizeof(*efimemmap));
	if (!efimemmap)
		return EFI_OUT_OF_RESOURCES;

	for (i = 0; i < nb; i++) {
		efimemmap[i].NumberOfPages = ranges[i].size / EFI_PAGE_SIZE;
		efimemmap[i].PhysicalStart = ranges[i].base;
		ret = e820_to_efi(ranges[i].type, &efimemmap[i].Type);
		if (EFI_ERROR(ret)) {
			free(efimemmap);
			efimemmap = NULL;
			return ret;
		}
	}

	efimemmap_nb = nb;
	return EFI_SUCCESS;
}

static EFI_CALCULATE_CRC32 crc32;

static EFIAPI EFI_STATUS
get_memory_map(UINTN *MemoryMapSize, EFI_MEMORY_DESCRIPTOR *MemoryMap,
	       UINTN *MapKey, UINTN *DescriptorSize, UINT32 *DescriptorVersion)
{
	EFI_STATUS ret;
	UINT32 key;
	UINTN size;

	if (!MemoryMapSize || !MemoryMap || !MapKey ||
	    !DescriptorSize || !DescriptorVersion)
		return EFI_INVALID_PARAMETER;

	if (!efimemmap_nb || !efimemmap)
		return EFI_UNSUPPORTED;

	size = efimemmap_nb * sizeof(*efimemmap);
	if (size > *MemoryMapSize)
		return EFI_BUFFER_TOO_SMALL;

	ret = uefi_call_wrapper(crc32, 3, efimemmap, size, &key);
	if (EFI_ERROR(ret))
		return ret;

	*MemoryMapSize = size;
	memcpy(MemoryMap, efimemmap, size);
	*MapKey = key;
	*DescriptorSize = sizeof(*efimemmap);
	*DescriptorVersion = EFI_MEMORY_DESCRIPTOR_VERSION;

	return EFI_SUCCESS;
}

static EFI_GET_MEMORY_MAP saved_memmap_bs;

static EFI_STATUS lpmemmap_init(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;

	if (!st)
		return EFI_INVALID_PARAMETER;

	if (!lib_sysinfo.n_memranges)
		return EFI_NOT_FOUND;

	ret = lpmemmap_to_efimemmap(lib_sysinfo.memrange,
				    lib_sysinfo.n_memranges);
	if (EFI_ERROR(ret))
		return ret;

	saved_memmap_bs = st->BootServices->GetMemoryMap;
	st->BootServices->GetMemoryMap = get_memory_map;
	crc32 = st->BootServices->CalculateCrc32;

	return EFI_SUCCESS;
}

static EFI_STATUS lpmemmap_exit(EFI_SYSTEM_TABLE *st)
{
	if (!st)
		return EFI_INVALID_PARAMETER;

	if (efimemmap) {
		st->BootServices->GetMemoryMap = saved_memmap_bs;
		free(efimemmap);
		efimemmap_nb = 0;
	}

	return EFI_SUCCESS;
}

ewdrv_t lpmemmap_drv = {
	.name = "lpmemmap",
	.description = "Convert Libpayload sysinfo memory map to EFI memory map",
	.init = lpmemmap_init,
	.exit = lpmemmap_exit
};
