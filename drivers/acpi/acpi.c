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

#include <conf_table.h>
#include <stdint.h>
#include <string.h>

#include "acpi.h"

static const char RSDP_MAGIC[8] = "RSD PTR ";

struct RSDP_TABLE {
	char signature[8];		/* "RSD PTR " */
	uint8_t checksum;		/* RSDP Checksum (bytes 0-19) */
	char oem_id[6];			/* OEM ID String */
	char revision;			/* ACPI Revision (0=1.0,2=2.0) */
	uint32_t rsdt_address;		/* 32-bit RSDT Pointer */
	uint32_t length;		/* RSDP Length */
	uint64_t xsdt_address;		/* 64-bit XSDT Pointer */
	uint8_t extended_checksum;	/* rsdp Checksum (full) */
	char reserved[3];		/* Reserved */
} __attribute__((packed));

static uint8_t checksum(uint8_t *buf, size_t size)
{
	uint8_t sum;
	size_t i;

	for (sum = 0, i = 0; i < size; i++)
		sum += buf[i];

	return !sum ? 0 : 0x100 - sum;
}

static struct RSDP_TABLE *lookup_for_rdsp(void)
{
	char *p;

	for (p = (char *)0xE0000; p < (char *)0x100000; p += 16)
		if (!memcmp(p, RSDP_MAGIC, sizeof(RSDP_MAGIC)))
			return (struct RSDP_TABLE *)p;

	return NULL;
}

static EFI_STATUS get_rsdp(struct RSDP_TABLE **rsdp, EFI_GUID *guid)
{
	EFI_GUID acpi_guid = ACPI_TABLE_GUID;
	EFI_GUID acpi2_guid = ACPI_20_TABLE_GUID;
	struct RSDP_TABLE *table;

	table = lookup_for_rdsp();
	if (!table)
		return EFI_NOT_FOUND;

	switch (table->revision) {
	case 0:
		if (checksum((uint8_t *)table,
			     offsetof(struct RSDP_TABLE, length)))
			return EFI_COMPROMISED_DATA;
		memcpy(guid, &acpi_guid, sizeof(*guid));
		break;
	case 2:
		if (checksum((uint8_t *)table, sizeof(*table)))
			return EFI_COMPROMISED_DATA;
		memcpy(guid, &acpi2_guid, sizeof(*guid));
		break;
	default:
		return EFI_UNSUPPORTED;
	}

	*rsdp = table;
	return EFI_SUCCESS;
}

static EFI_STATUS acpi_init(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;
	EFI_CONFIGURATION_TABLE *table;
	struct RSDP_TABLE *rsdp;
	EFI_GUID guid;

	if (!st)
		return EFI_INVALID_PARAMETER;

	ret = get_rsdp(&rsdp, &guid);
	if (EFI_ERROR(ret))
		return ret;

	ret = conf_table_new(st, &guid, &table);
	if (EFI_ERROR(ret))
		return ret;

	table->VendorTable = rsdp;

	return EFI_SUCCESS;
}

static EFI_STATUS acpi_exit(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;
	struct RSDP_TABLE *rsdp;
	EFI_GUID guid;

	if (!st)
		return EFI_INVALID_PARAMETER;

	ret = get_rsdp(&rsdp, &guid);
	if (EFI_ERROR(ret))
		return ret;

	return conf_table_free(st, &guid);
}

ewdrv_t acpi_drv = {
	.name = "acpi",
	.description = "Look-up for ACPI tables and provide them via the System Table",
	.init = acpi_init,
	.exit = acpi_exit
};
