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

#include <efiwrapper.h>
#include <stdio.h>
#include <drivers/storage/mmc.h>

#include "smbios.h"

static SMBIOS_STRUCTURE_TABLE smbios = {
	.AnchorString = "_SM_",
	.EntryPointStructureChecksum = 0,
	.EntryPointLength = sizeof(SMBIOS_STRUCTURE_TABLE),
	.MajorVersion = 2,
	.MinorVersion = 2,
	.MaxStructureSize = 0,
	.EntryPointRevision = 0,
	.IntermediateAnchorString = "_DMI_",
	.IntermediateChecksum = 0,
	.TableLength = 0,
	.TableAddress = 0,
	.NumberOfSmbiosStructures = 0,
	.SmbiosBcdRevision = 0
};

struct cid {
	uint8_t mid: 8;
	uint8_t reserved: 6;
	uint8_t cbx: 2;
	uint8_t oid: 8;
	char pnm[6];
	uint8_t prv;
	uint32_t psn;
	uint8_t mdt;
	uint8_t crc: 7;
	uint8_t unused: 1;
} __attribute__((packed));

/* The serial number is the concatenation of the CID PNM and PSN
 * fields. */
static EFI_STATUS build_serial(char *serial)
{
	struct cid cid;
	size_t i;
	uint32_t *tmp = (uint32_t *)&cid;
	int ret;

	ret = mmc_cid((uint8_t *)&cid);
	if (ret)
		return EFI_DEVICE_ERROR;

	for (i = 0; i < sizeof(cid) / sizeof(*tmp); i++)
		tmp[i] = __builtin_bswap32(tmp[i]);

	ew_memcpy(&serial[0], cid.pnm, sizeof(cid.pnm));
	snprintf(&serial[6], 9, "%08x", cid.psn);

	return EFI_SUCCESS;
}

static uint8_t checksum(uint8_t *buf, size_t size)
{
	uint8_t sum = 0;
	size_t i;

	for (i = 0; i < size; i++)
		sum += buf[i];

	return !sum ? 0 : 0x100 - sum;
}

#define VENDOR_NAME "Intel Corporation"
static const char _VENDOR_NAME[] = VENDOR_NAME;

#define PRODUCT_NAME "Broxton-P"
static const char _PRODUCT_NAME[] = PRODUCT_NAME;

#define EFI_WRAPPER_VERSION "00.01"
static const char _EFI_WRAPPER_VERSION[] = EFI_WRAPPER_VERSION;

static struct type0 {
	SMBIOS_TYPE0 type0;
	char vendor[ARRAY_SIZE(_VENDOR_NAME)];
	char bios_version[ARRAY_SIZE(_EFI_WRAPPER_VERSION)];
	char end;
} __attribute__((__packed__)) type0 = {
	.type0 = {
		.Hdr = {
			.Type = 0,
			.Length = sizeof(SMBIOS_TYPE0)
		},
		.Vendor = 1,
		.BiosVersion = 2
	},
	.vendor = VENDOR_NAME,
	.bios_version = EFI_WRAPPER_VERSION
};

static struct type1 {
	SMBIOS_TYPE1 type1;
	char serial_number[15];
	char product_name[ARRAY_SIZE(_PRODUCT_NAME)];
	char end;
} __attribute__((__packed__)) type1 = {
	.type1 = {
		.Hdr = {
			.Type = 1,
			.Length = sizeof(SMBIOS_TYPE1)
		},
		.SerialNumber = 1,
		.ProductName = 2
	},
	.product_name = PRODUCT_NAME
};

static struct type2 {
	SMBIOS_TYPE2 type2;
	char manufacturer[ARRAY_SIZE(_VENDOR_NAME)];
	char product_name[ARRAY_SIZE(_PRODUCT_NAME)];
	char end;
} __attribute__((__packed__)) type2 = {
	.type2 = {
		.Hdr = {
			.Type = 2,
			.Length = sizeof(SMBIOS_TYPE2)
		},
		.Manufacturer = 1,
		.ProductName = 2,
		.Version = 0,
		.SerialNumber = 0
	},
	.manufacturer = VENDOR_NAME,
	.product_name = PRODUCT_NAME
};

static struct types {
	char *buf;
	size_t size;
} TYPES[] = {
	{ (char *)&type0, sizeof(type0) },
	{ (char *)&type1, sizeof(type1) },
	{ (char *)&type2, sizeof(type2) }
};

EFI_STATUS smbios_init_table(EFI_CONFIGURATION_TABLE *table)
{
	EFI_STATUS ret;
	char *buf;
	size_t size, i;

	ew_memcpy(&table->VendorGuid, &SMBIOSTableGuid,
		  sizeof(SMBIOSTableGuid));
	table->VendorTable = &smbios;

	ret = build_serial(type1.serial_number);
	if (EFI_ERROR(ret))
		return ret;

	for (i = 0, size = 0; i < ARRAY_SIZE(TYPES); i++)
		size += TYPES[i].size;

	buf = ew_malloc(size);
	if (!buf)
		return EFI_OUT_OF_RESOURCES;

	smbios.TableLength = ARRAY_SIZE(TYPES);
	smbios.TableAddress = (UINT32)buf;

	for (i = 0; i < ARRAY_SIZE(TYPES); i++) {
		ew_memcpy(buf, TYPES[i].buf, TYPES[i].size);
		buf += TYPES[i].size;
	}

	smbios.EntryPointStructureChecksum = 0;
	smbios.EntryPointStructureChecksum = checksum((uint8_t *)&smbios, sizeof(smbios));

	return EFI_SUCCESS;
}
