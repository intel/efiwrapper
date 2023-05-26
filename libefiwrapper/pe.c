/*
 * Copyright (c) 2016-2020, Intel Corporation
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

#include <stddef.h>
#include <stdint.h>
#include <ewlog.h>
#include <ewlib.h>

#if defined(HOST)
#include <sys/mman.h>
#endif

#include "lib.h"
#include "pe.h"

/* The following structure definitions come from:
 * https://en.wikipedia.org/wiki/Portable_Executable. */
#pragma pack(1)

#define DOS_MAGIC 0x5A4D	/* "MZ" */

typedef struct {
	UINT16 magic;	       /* Magic number */
	UINT16 cblp;	       /* Bytes on last page of file */
	UINT16 cp;	       /* Pages in file */
	UINT16 crlc;	       /* Relocations */
	UINT16 cparhdr;	       /* Size of header in paragraphs */
	UINT16 minalloc;       /* Minimum extra paragraphs needed */
	UINT16 maxalloc;       /* Maximum extra paragraphs needed */
	UINT16 ss;	       /* Initial (relative) SS value */
	UINT16 sp;	       /* Initial SP value */
	UINT16 csum;	       /* Checksum */
	UINT16 ip;	       /* Initial IP value */
	UINT16 cs;	       /* Initial (relative) CS value */
	UINT16 lfarlc;	       /* File address of relocation table */
	UINT16 ovno;	       /* Overlay number */
	UINT16 res[4];	       /* Reserved words */
	UINT16 oemid;	       /* OEM identifier (for e_oeminfo) */
	UINT16 oeminfo;	       /* OEM information; e_oemid specific */
	UINT16 res2[10];       /* Reserved words */
	UINT32 lfanew;	       /* File address of new exe header */
} dos_header_t;

#define COFF_HDR_SIGNATURE 0x4550 /* "PE" */

typedef struct {
	UINT32 signature;
	UINT16 machine;
	UINT16 number_of_sections;
	UINT32 time_date_stamp;
	UINT32 pointer_to_symbol_table; /* Deprecated */
	UINT32 number_of_symbols;	/* Deprecated */
	UINT16 size_of_optional_header;
	UINT16 characteristics;
} coff_header_t;

#ifdef __LP64__
#define COFF_STD_MAGIC 0x020b
#else
#define COFF_STD_MAGIC 0x010b
#endif

typedef struct {
	UINT16 magic;
	UINT8 major_linker_version;
	UINT8 minor_linker_version;
	UINT32 size_of_code;
	UINT32 size_of_initialized_data;
	UINT32 size_of_uninitialized_data;
	UINT32 address_of_entry_point;
	UINT32 base_of_code;
#ifndef __LP64__
	UINT32 base_of_data;
#endif
} std_coff_t;

typedef struct {
	unsigned long image_base;
	UINT32 section_alignment;
	UINT32 file_alignment;
	UINT16 major_operating_system_version;
	UINT16 minor_operating_system_version;
	UINT16 major_image_version;
	UINT16 minor_image_version;
	UINT16 major_subsystem_version;
	UINT16 minor_subsystem_version;
	UINT32 win32_version_value;
	UINT32 size_of_image;
	UINT32 size_of_headers;
	UINT32 checksum;
	UINT16 subsystem;
	UINT16 dll_characteristics;
	unsigned long size_of_stack_reserve;
	unsigned long size_of_stack_commit;
	unsigned long size_of_heap_reserve;
	unsigned long size_of_heap_commit;
	UINT32 loader_flags;
	UINT32 number_of_rva_and_sizes;
} win_t;

#define BASERELOC_DIRECTORY_ENTRY 5

typedef struct {
	coff_header_t hdr;
	struct {
		std_coff_t std;
		win_t win;
		struct data_directory {
			UINT32 address;
			UINT32 size;
		} data_directory[16];
	} opt;
} pe_coff_t;

typedef struct {
	UINT8 name[8];
	UINT32 virtual_size;
	UINT32 virtual_address;
	UINT32 size_of_raw_data;
	UINT32 pointer_to_raw_data;
	UINT32 pointer_to_relocations;
	UINT32 pointer_to_line_numbers;
	UINT16 number_of_relocations;
	UINT16 number_of_line_numbers;
	UINT32 characteristics;
} section_header_t;

typedef struct {
	UINT32  virtual_address;
	UINT32  size_of_block;
} relocation_t;

typedef enum {
      ABSOLUTE,
      HIGH,
      LOW,
      HIGHLOW,
      HIGHADJ,
      MACHINE_SPECIFIC_5,
      RESERVED,
      MACHINE_SPECIFIC_7,
      MACHINE_SPECIFIC_8,
      MACHINE_SPECIFIC_9,
      DIR64
} fixup_type_t;

typedef struct {
	UINT16 offset: 12;
	UINT8 type: 4;
} fixup_t;

#pragma pack()

typedef struct {
	void *addr;
	size_t size;
	UINT32 start;
	UINT32 end;
} loaded_section_t;

static void get_section_boundaries(section_header_t *section, UINT16 nb,
				   UINT32 *start, UINT32 *end)
{
	UINT16 i;

	*start = (UINT32)-1;
	*end = 0;

	for (i = 0; i < nb; i++) {
		UINT32 size = section[i].virtual_size;
		if (section[i].size_of_raw_data)
			size = section[i].size_of_raw_data;
		*start = min(*start, section[i].virtual_address);
		*end = max(*end, section[i].virtual_address + size);
	}
}

static EFI_STATUS load_sections(section_header_t *section, UINT16 nb,
				void *base, image_t *image,
				size_t alignment, loaded_section_t **data_p)
{
	loaded_section_t *data;
	UINT16 i;
	char *dst, *src;
	size_t size;

	data = malloc(sizeof(*data));
	if (!data) {
		ewerr("Failed to allocate loaded section");
		return EFI_OUT_OF_RESOURCES;
	}

	get_section_boundaries(section, nb, &data->start, &data->end);

	data->size = data->end - data->start;
#if defined(HOST)
	data->addr = mmap(NULL, data->size, PROT_EXEC | PROT_READ | PROT_WRITE,
			  MAP_ANONYMOUS | MAP_SHARED, -1, 0);
#else
	data->addr = memalign(alignment, data->size);
#endif
	if (data->addr == (void *) -1 || data->addr == NULL) {
		ewerr("Failed to allocate sections memory");
		free(data);
		return EFI_OUT_OF_RESOURCES;
	}

	memset(data->addr, 0, data->size);
	image->data = data;

	for (i = 0; i < nb; i++) {
		src = (char *)base + section[i].pointer_to_raw_data;
		dst = (char *)data->addr + section[i].virtual_address -
			data->start;
		size = section[i].virtual_size;

		if (section[i].size_of_raw_data)
			size = section[i].size_of_raw_data;
		memcpy(dst, src, size);

		if (size < section[i].virtual_size)
			memset(dst + size, 0, section[i].virtual_size - size);

		if (!memcmp(".text", section[i].name, 6))
			ewdbg(".text section loaded at %p", dst);
	}

	*data_p = data;
	return EFI_SUCCESS;
}

static void relocate(loaded_section_t *data, unsigned long base,
		     relocation_t *table, UINT32 size)
{
	relocation_t *table_end = (relocation_t *)((UINT8 *)table + size);
	unsigned long diff = (unsigned long)data->addr - base - data->start;

	while (table < table_end) {
		fixup_t *fixup = (fixup_t *)&table[1];
		fixup_t *fixup_end = (fixup_t *)((char *)table + table->size_of_block);
		unsigned char *mem = (unsigned char *)data->addr +
			table->virtual_address - data->start;

		for (; fixup < fixup_end; fixup++) {
			if (*(UINT16 *)fixup == 0)
				break;
			switch (fixup->type) {
			case HIGHLOW:
				*(UINT32 *)&mem[fixup->offset] += diff;
				break;
			case DIR64:
				*(UINT64 *)&mem[fixup->offset] += diff;
				break;
			default:
				ewerr("Unsupported relocation type %d",
				      fixup->type);
			}
		}
		table = (relocation_t *)fixup_end;
	}
}

EFI_STATUS pe_load(void *data, UINTN size, image_t *image)
{
	EFI_STATUS ret;
	dos_header_t *dos_hdr;
	pe_coff_t *pe;
	char *entry;
	loaded_section_t *section;
	struct data_directory *reloc_dir;

	if (!data || !size || !image)
		return EFI_INVALID_PARAMETER;

	if (size < sizeof(*dos_hdr))
		return EFI_INVALID_PARAMETER;

	dos_hdr = (dos_header_t *)data;
	if (dos_hdr->magic != DOS_MAGIC)
	    return EFI_INVALID_PARAMETER;

	if (!dos_hdr->lfanew || dos_hdr->lfanew + sizeof(*pe) > size)
		return EFI_INVALID_PARAMETER;

	pe = (pe_coff_t *)((char *)data + dos_hdr->lfanew);
	if (pe->hdr.signature != COFF_HDR_SIGNATURE)
		return EFI_INVALID_PARAMETER;

	if (pe->opt.std.magic != COFF_STD_MAGIC)
		return EFI_INVALID_PARAMETER;

	ret = load_sections((section_header_t *)(pe + 1),
			    pe->hdr.number_of_sections,
			    data, image, pe->opt.win.section_alignment,
			    &section);
	if (EFI_ERROR(ret))
		return ret;

	reloc_dir = &pe->opt.data_directory[BASERELOC_DIRECTORY_ENTRY];
	relocate((loaded_section_t *)image->data, pe->opt.win.image_base,
		 section->addr + reloc_dir->address - section->start,
		 reloc_dir->size);

	entry = (char *)((loaded_section_t *)image->data)->addr;
	entry += pe->opt.std.address_of_entry_point;
	entry -= ((loaded_section_t *)image->data)->start;
	image->entry = (EFI_IMAGE_ENTRY_POINT)entry;

	return EFI_SUCCESS;
}

EFI_STATUS pe_unload(image_t *image)
{
	loaded_section_t *data;

	if (!image || !image->data)
		return EFI_INVALID_PARAMETER;

	data = image->data;
#if defined(HOST)
	munmap(data->addr, data->size);
#else
	free(data->addr);
#endif
	free(data);
	image->data = NULL;

	return EFI_SUCCESS;
}
