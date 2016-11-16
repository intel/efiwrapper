/*
 * Copyright (c) 2016, Intel Corporation
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

#include <efi.h>
#include <efiapi.h>
#include <libsmbios.h>

#include "conf_table.h"
#include "external.h"
#include "lib.h"
#include "smbios.h"
#include "version.h"

static EFI_GUID smbios_guid = SMBIOS_TABLE_GUID;

#define MAX_SMBIOS_FIELD 32

static struct {
	struct type0 {
		SMBIOS_TYPE0 type0;
		char vendor[MAX_SMBIOS_FIELD];
		char unused0[2];
		char bios_version[MAX_SMBIOS_FIELD];
		char unused1[2];
		char end;
	} __attribute__((__packed__)) type0;
	struct type1 {
		SMBIOS_TYPE1 type1;
		char serial_number[MAX_SMBIOS_FIELD];
		char unused0[2];
		char product_name[MAX_SMBIOS_FIELD];
		char unused1[2];
		char version[MAX_SMBIOS_FIELD];
		char unused2[2];
		char end;
	} __attribute__((__packed__)) type1;
	struct type2 {
		SMBIOS_TYPE2 type2;
		char manufacturer[MAX_SMBIOS_FIELD];
		char unused0[2];
		char product_name[MAX_SMBIOS_FIELD];
		char unused1[2];
		char version[MAX_SMBIOS_FIELD];
		char unused2[2];
		char end;
	} __attribute__((__packed__)) type2;
} __attribute__((__packed__)) smbios_table = {
	{
		.type0 = {
			.Hdr = {
				.Type = 0,
				.Length = sizeof(SMBIOS_TYPE0)
			},
			.Vendor = 1,
			.BiosVersion = 3
		},
		.unused0 = " ",
		.unused1 = " "
	},
	{
		.type1 = {
			.Hdr = {
				.Type = 1,
				.Length = sizeof(SMBIOS_TYPE1)
			},
			.SerialNumber = 1,
			.ProductName = 3,
			.Version = 5
		},
		.unused0 = " ",
		.unused1 = " ",
		.unused2 = " "
	},
	{
		.type2 = {
			.Hdr = {
				.Type = 2,
				.Length = sizeof(SMBIOS_TYPE2)
			},
			.Manufacturer = 1,
			.ProductName = 3,
			.Version = 5
		},
		.unused0 = " ",
		.unused1 = " ",
		.unused2 = " "
	}
};

static SMBIOS_STRUCTURE_TABLE smbios = {
	.AnchorString = "_SM_",
	.EntryPointLength = sizeof(SMBIOS_STRUCTURE_TABLE),
	.MajorVersion = 2,
	.MinorVersion = 2,
	.IntermediateAnchorString = "_DMI_",
	.TableLength = 3,
	.TableAddress = (UINT32)&smbios_table,
};

static UINT8 checksum(UINT8 *buf, size_t size)
{
	UINT8 sum;
	size_t i;

	for (sum = 0, i = 0; i < size; i++)
		sum += buf[i];

	return !sum ? 0 : 0x100 - sum;
}

static EFI_STATUS set_field(char *field, const char *value)
{
	size_t len;

	len = strlen(value);
	if (len == 0)
		return EFI_INVALID_PARAMETER;
	if (len > MAX_SMBIOS_FIELD - 1)
		return EFI_BUFFER_TOO_SMALL;

	memcpy(field, value, len + 1);
	memset(field + len + 1, ' ', MAX_SMBIOS_FIELD - len - 1);

	return EFI_SUCCESS;
}

static const struct {
	char *field;
	const char *value;
} SMBIOS_DEFAULT[] = {
	{ smbios_table.type0.vendor, 		PRODUCT_MANUFACTURER },
	{ smbios_table.type0.bios_version, 	EFIWRAPPER_VERSION },
	{ smbios_table.type1.serial_number,	SMBIOS_UNDEFINED },
	{ smbios_table.type1.product_name,	PRODUCT_NAME },
	{ smbios_table.type1.version,		SMBIOS_UNDEFINED },
	{ smbios_table.type2.manufacturer,	PRODUCT_MANUFACTURER },
	{ smbios_table.type2.product_name,	PRODUCT_NAME },
	{ smbios_table.type2.version,		SMBIOS_UNDEFINED },
};

EFI_STATUS smbios_init(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;
	EFI_CONFIGURATION_TABLE *table;
	size_t i;

	if (!st)
		return EFI_INVALID_PARAMETER;

	ret = conf_table_new(st, &smbios_guid, &table);
	if (EFI_ERROR(ret))
		return ret;

	for (i = 0; i < ARRAY_SIZE(SMBIOS_DEFAULT); i++) {
		ret = set_field(SMBIOS_DEFAULT[i].field, SMBIOS_DEFAULT[i].value);
		if (EFI_ERROR(ret))
			return ret;
	}

	table->VendorTable = &smbios;
	smbios.EntryPointStructureChecksum = 0;
	smbios.EntryPointStructureChecksum = checksum((UINT8 *)&smbios, sizeof(smbios));

	return EFI_SUCCESS;
}

EFI_STATUS smbios_free(EFI_SYSTEM_TABLE *st)
{
	return conf_table_free(st, &smbios_guid);
}

static char *get_table_field(SMBIOS_HEADER *hdr, UINT8 field)
{
	UINT8 i;
	char *str;

	if (field == 0 || hdr >= (SMBIOS_HEADER *)(&smbios_table + 1))
		return NULL;

	str = (char *)hdr + hdr->Length;
	for (i = 1; i < field; i++) {
		while (*str)
			str++;

		if (*(++str))
			continue;

		return field == (UINT8)-1 ? str + 1 : NULL;
	}

	return str;
}

static char *get_field(UINT8 type, UINT8 offset)
{
	SMBIOS_HEADER *hdr;

	hdr = (SMBIOS_HEADER *)&smbios_table;
	while (hdr && hdr->Type != type)
		hdr = (SMBIOS_HEADER *)get_table_field(hdr, -1);

	if (!hdr)
		return NULL;

	return get_table_field(hdr, ((UINT8 *)hdr)[offset]);
}

EFI_STATUS smbios_set(UINT8 type, UINT8 offset, const char *value)
{
	char *field;

	field = get_field(type, offset);
	if (!field)
		return EFI_NOT_FOUND;

	return set_field(field, value);
}
