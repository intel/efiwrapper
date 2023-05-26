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

#include "ewacpi.h"
#include "external.h"

#define SIG_SIZE (sizeof(((struct acpi_header *)0)->signature))

struct rsdp_table {
	CHAR8 signature[8];		/* "RSD PTR " */
	UINT8 checksum;			/* RSDP Checksum (bytes 0-19) */
	CHAR8 oem_id[6];		/* OEM ID String */
	CHAR8 revision;			/* ACPI Revision (0=1.0,2=2.0) */
	UINT32 rsdt_address;		/* 32-bit RSDT Pointer */
	UINT32 length;			/* RSDP Length */
	UINT64 xsdt_address;		/* 64-bit XSDT Pointer */
	UINT8 extended_checksum;	/* rsdp Checksum (full) */
	CHAR8 reserved[3];		/* Reserved */
} __attribute__((packed));

struct xsdt_table {
	struct acpi_header header;
	UINT64 entry[1];		/* Table Entries */
} __attribute__((packed));

static EFI_STATUS validate_table(struct acpi_header *table)
{
	UINT8 sum, *buf = (UINT8 *)table;
	UINTN i;

	if (table->length < sizeof(*table))
		return EFI_COMPROMISED_DATA;

	for (sum = 0, i = 0; i < table->length; i++)
		sum += buf[i];

	return sum ? EFI_COMPROMISED_DATA : EFI_SUCCESS;
}

EFI_STATUS ewacpi_get_table(EFI_SYSTEM_TABLE *st, const char *name,
			    struct acpi_header **table)
{
	EFI_STATUS ret;
	const EFI_GUID acpi2_guid = ACPI_20_TABLE_GUID;
	struct rsdp_table *rsdp = NULL;
	struct xsdt_table *xsdt;
	struct acpi_header *cur;
	UINTN i, nb;

	if (!st || !name || !table)
		return EFI_INVALID_PARAMETER;

	if (strlen(name) > SIG_SIZE)
		return EFI_INVALID_PARAMETER;

	for (i = 0; i < st->NumberOfTableEntries; i++) {
		if (memcmp(&st->ConfigurationTable[i].VendorGuid,
			   &acpi2_guid, sizeof(acpi2_guid)))
			continue;
		rsdp = st->ConfigurationTable[i].VendorTable;
		break;
	}

	if (!rsdp)
		return EFI_NOT_FOUND;

	if (!rsdp->xsdt_address)
		return EFI_UNSUPPORTED;

	xsdt = (struct xsdt_table *)(unsigned long)rsdp->xsdt_address;
	nb = (xsdt->header.length - sizeof(xsdt->header)) / sizeof(xsdt->entry);
	for (i = 0; i < nb; i++) {
		cur = (struct acpi_header *)(unsigned long)xsdt->entry[i];
		if (memcmp(name, cur->signature, SIG_SIZE))
			continue;
		ret = validate_table(cur);
		if (EFI_ERROR(ret))
			return ret;

		*table = cur;
		return EFI_SUCCESS;
	}

	return EFI_NOT_FOUND;
}
