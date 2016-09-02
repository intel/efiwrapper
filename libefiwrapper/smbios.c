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

#define DEFAULT_SERIAL_NUMBER "0123456789     "
static const char serial_number[] = DEFAULT_SERIAL_NUMBER;
static const char manufacturer[] = PRODUCT_MANUFACTURER;
static const char product[] = PRODUCT_NAME;
static const char version[] = EFIWRAPPER_VERSION;

static struct smbios_table {
	struct type0 {
		SMBIOS_TYPE0 type0;
		char vendor[ARRAY_SIZE(manufacturer)];
		char bios_version[ARRAY_SIZE(EFIWRAPPER_VERSION)];
		char end;
	} __attribute__((__packed__)) type0;
	struct type1 {
		SMBIOS_TYPE1 type1;
		char serial_number[ARRAY_SIZE(serial_number)];
		char product_name[ARRAY_SIZE(product)];
		char end;
	} __attribute__((__packed__)) type1;
	struct type2 {
		SMBIOS_TYPE2 type2;
		char manufacturer[ARRAY_SIZE(manufacturer)];
		char product_name[ARRAY_SIZE(product)];
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
			.BiosVersion = 2
		},
		.vendor = PRODUCT_MANUFACTURER,
		.bios_version = EFIWRAPPER_VERSION
	},
	{
		.type1 = {
			.Hdr = {
				.Type = 1,
				.Length = sizeof(SMBIOS_TYPE1)
			},
			.SerialNumber = 1,
			.ProductName = 2
		},
		.serial_number = DEFAULT_SERIAL_NUMBER,
		.product_name = PRODUCT_NAME
	},
	{
		.type2 = {
			.Hdr = {
				.Type = 2,
				.Length = sizeof(SMBIOS_TYPE2)
			},
			.Manufacturer = 1,
			.ProductName = 2
		},
		.manufacturer = PRODUCT_MANUFACTURER,
		.product_name = PRODUCT_NAME
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

EFI_STATUS smbios_init(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;
	EFI_CONFIGURATION_TABLE *table;

	if (!st)
		return EFI_INVALID_PARAMETER;

	ret = conf_table_new(st, &smbios_guid, &table);
	if (EFI_ERROR(ret))
		return ret;

	table->VendorTable = &smbios;
	smbios.EntryPointStructureChecksum = 0;
	smbios.EntryPointStructureChecksum = checksum((UINT8 *)&smbios, sizeof(smbios));

	return EFI_SUCCESS;
}

EFI_STATUS smbios_free(EFI_SYSTEM_TABLE *st)
{
	return conf_table_free(st, &smbios_guid);
}

EFI_STATUS smbios_set_serial_number(char *serial)
{
	if (!serial)
		return EFI_INVALID_PARAMETER;

	memcpy(smbios_table.type1.serial_number, serial,
	       min(sizeof(smbios_table.type1.serial_number) - 1,
		   strlen(serial)));

	return EFI_SUCCESS;
}
