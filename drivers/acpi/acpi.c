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
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer
 *   in the documentation and/or other materials provided with the
 *   distribution.
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
#include "AcpiTableProtocol.h"
#include "acpi50.h"
#include "interface.h"
#include "ewlog.h"

#define AML_EXT_REGION_OP 0x80

#define MAX_XSDT_HEADER_ENTRIES 60 // MUST <= the value in ABL/SBL

static const char RSDP_MAGIC[8] = "RSD PTR ";

static struct RSDP_TABLE *Rsdp;
static EFI_GUID acpi_protocol_guid = EFI_ACPI_TABLE_PROTOCOL_GUID;
static EFI_HANDLE handle;	// handle of acpi_protocol
static EFI_SYSTEM_TABLE *p_st;	// pointer to system table

static EFI_STATUS acpi_mem_alloc(UINTN Size, VOID **Buffer, UINTN *no_page)
{
	if (!p_st)
		return EFI_NOT_READY;

	*no_page = EFI_SIZE_TO_PAGES(Size);
	return p_st->BootServices->AllocatePages(
	    AllocateAnyPages, EfiACPIReclaimMemory, *no_page,
	    (EFI_PHYSICAL_ADDRESS *)Buffer);
}

static EFI_STATUS acpi_mem_free(VOID **Buffer, UINTN no_page)
{
	EFI_STATUS ret = EFI_SUCCESS;

	if (!p_st)
		return EFI_NOT_READY;

	ret = p_st->BootServices->FreePages(*(EFI_PHYSICAL_ADDRESS *)Buffer,
					     no_page);
	*Buffer = NULL;
	return ret;
}

struct RSDP_TABLE {
	char signature[8];          /* "RSD PTR " */
	uint8_t checksum;           /* RSDP Checksum (bytes 0-19) */
	char oem_id[6];             /* OEM ID String */
	char revision;              /* ACPI Revision (0=1.0,2=2.0) */
	uint32_t rsdt_address;      /* 32-bit RSDT Pointer */
	uint32_t length;            /* RSDP Length */
	uint64_t xsdt_address;      /* 64-bit XSDT Pointer */
	uint8_t extended_checksum;  /* rsdp Checksum (full) */
	char reserved[3];           /* Reserved */
} __attribute__((packed));

static uint8_t checksum(uint8_t *buf, size_t size)
{
	uint8_t sum;
	size_t i;

	for (sum = 0, i = 0; i < size; i++)
		sum += buf[i];

	return !sum ? 0 : 0x100 - sum;
}

static struct RSDP_TABLE *lookup_for_rdsp(char *from)
{
	char *p;

	if (!from)
		from = (char *)0xE0000;

	for (p = from; p < (char *)0x100000; p += 16)
		if (!memcmp(p, RSDP_MAGIC, sizeof(RSDP_MAGIC)))
			return (struct RSDP_TABLE *)p;

	ewerr("ACPI:can't find RSDP\n");

	return NULL;
}

static EFI_STATUS get_rsdp(struct RSDP_TABLE **rsdp, EFI_GUID *guid)
{
	EFI_GUID acpi_guid = ACPI_TABLE_GUID;
	EFI_GUID acpi2_guid = ACPI_20_TABLE_GUID;
	struct RSDP_TABLE *table;

	for (table = NULL;; table += 16) {
		table = lookup_for_rdsp((char *)table);
		if (!table)
			return EFI_NOT_FOUND;
		if (table->revision == 0) {
			if (checksum((uint8_t *)table,
				     offsetof(struct RSDP_TABLE, length)))
				continue;
			memcpy(guid, &acpi_guid, sizeof(*guid)); //NOLINT
			break;
		} else if (table->revision == 2) {
			if (checksum((uint8_t *)table, sizeof(*table)))
				continue;
			memcpy(guid, &acpi2_guid, sizeof(*guid)); //NOLINT
			break;
		}
	}

	*rsdp = table;
	return EFI_SUCCESS;
}

#define SIGNATURE_16(A, B) ((A) | (B << 8))
#define SIGNATURE_32(A, B, C, D)                                               \
	(SIGNATURE_16(A, B) | (SIGNATURE_16(C, D) << 16))

static void UpdateAcpiGnvs(EFI_ACPI_DESCRIPTION_HEADER *newDsdt,
			   EFI_ACPI_DESCRIPTION_HEADER *oldDsdt)
{
	UINT8 *Ptr;
	UINT8 *End;
	UINT32 GnvsBase = 0;
	UINT16 GnvsSize;

	Ptr = (UINT8 *)oldDsdt;
	End = (UINT8 *)oldDsdt + oldDsdt->Length;

	/*
	 * Loop through the ASL looking for values that we must fix up.
	 */
	for (; Ptr < End; Ptr++) {
		if (*(UINT32 *)Ptr != SIGNATURE_32('G', 'N', 'V', 'S'))
			continue;

		if (*(Ptr - 1) != AML_EXT_REGION_OP)
			continue;

		GnvsBase = *(UINT32 *)(Ptr + 6);
		GnvsSize = *(UINT16 *)(Ptr + 11);
		break;
	}

	if (GnvsBase) {
		Ptr = (UINT8 *)newDsdt;
		End = (UINT8 *)newDsdt + newDsdt->Length;
		for (; Ptr < End; Ptr++) {
			if (*(UINT32 *)Ptr !=
			    SIGNATURE_32('G', 'N', 'V', 'S')) {
				continue;
			}
			if (*(Ptr - 1) != AML_EXT_REGION_OP)
				continue;

			*(UINT32 *)(Ptr + 6) = GnvsBase;
			*(UINT16 *)(Ptr + 11) = GnvsSize;
			break;
		}
	}
}

static EFI_ACPI_DESCRIPTION_HEADER *
FindAcpiTableBySignature(EFI_ACPI_DESCRIPTION_HEADER *Xsdt, UINT32 Signature,
			 UINT32 *EntryIndex, UINT64 OemTableId, UINT32 OemRevision)
{
	EFI_ACPI_DESCRIPTION_HEADER *CurrHdr;
	UINT64 *XsdtEntry;
	UINT32 EntryNum;
	UINT32 Index;

	XsdtEntry =
	    (UINT64 *)((UINT8 *)Xsdt + sizeof(EFI_ACPI_DESCRIPTION_HEADER));
	EntryNum = (Xsdt->Length - sizeof(EFI_ACPI_DESCRIPTION_HEADER)) /
		   sizeof(UINT64);

	for (Index = 0; Index < EntryNum; Index++) {
		CurrHdr = (EFI_ACPI_DESCRIPTION_HEADER *)XsdtEntry[Index];

		if ((CurrHdr != NULL) && (CurrHdr->Signature == Signature) &&
		    ((OemTableId == 0) || ((OemTableId == CurrHdr->OemTableId) &&
		    (CurrHdr->OemRevision <= OemRevision)))) {
			if (EntryIndex != NULL)
				*EntryIndex = Index;

			return CurrHdr;
		}
	}

	return NULL;
}

static EFIAPI EFI_STATUS InstallAcpiTable(__attribute__((__unused__))
					  EFI_ACPI_TABLE_PROTOCOL * This,
					  VOID *AcpiTableBuffer,
					  UINTN AcpiTableBufferSize,
					  __attribute__((__unused__))
					  UINTN *TableKey)
{

	EFI_ACPI_DESCRIPTION_HEADER *Xsdt;
	EFI_ACPI_5_0_FIXED_ACPI_DESCRIPTION_TABLE *Facp;
	EFI_ACPI_DESCRIPTION_HEADER *AcpiHdr;
	UINT8 *NewTable;
	UINT64 *XsdtEntry;
	UINT32 EntryIndex;
	UINT32 Size;
	UINT32 EntryNum;
	EFI_STATUS Status;
	UINT64 OemTableId;
	UINT32 OemRevision;

	if (Rsdp == NULL)
		return EFI_NOT_READY;

	if ((AcpiTableBuffer == NULL) ||
	    (AcpiTableBufferSize < sizeof(EFI_ACPI_DESCRIPTION_HEADER))) {
		return EFI_INVALID_PARAMETER;
	}

	Xsdt = (EFI_ACPI_DESCRIPTION_HEADER *)(UINTN)Rsdp->xsdt_address;
	XsdtEntry =
	    (UINT64 *)((UINT8 *)Xsdt + sizeof(EFI_ACPI_DESCRIPTION_HEADER));
	EntryNum = (Xsdt->Length - sizeof(EFI_ACPI_DESCRIPTION_HEADER)) /
		   sizeof(UINT64);

	Status = EFI_SUCCESS;
	Size = 0;

	while (Size < AcpiTableBufferSize) {
		AcpiHdr =
		    (EFI_ACPI_DESCRIPTION_HEADER *)(AcpiTableBuffer + Size);

		if (checksum((UINT8 *)AcpiHdr, AcpiHdr->Length) != 0) {
			Status = EFI_ABORTED;
			break;
		}

		if (Size + AcpiHdr->Length > Size) {
			Size += AcpiHdr->Length;
		} else {
			Status = EFI_ABORTED;
			break;
		}

		Status = acpi_mem_alloc(AcpiHdr->Length, (VOID **)&NewTable,
					TableKey);

		if (Status != EFI_SUCCESS) {
			ewerr("ACPI: can't allocate memory\n");
			break;
		}

		memcpy(NewTable, AcpiHdr, AcpiHdr->Length); //NOLINT

		// Update the ACPI header to pointer to the new copy
		// And then update the table if required
		AcpiHdr = (EFI_ACPI_DESCRIPTION_HEADER *)NewTable;
		if (AcpiHdr->Signature ==
		    EFI_ACPI_5_0_SECONDARY_SYSTEM_DESCRIPTION_TABLE_SIGNATURE) {
			//need to check OEM table ID and revision for SSDT
			OemTableId = AcpiHdr->OemTableId;
			OemRevision = AcpiHdr->OemRevision;
		} else {
			OemTableId = 0;
			OemRevision = 0;
		}
		if (AcpiHdr->Signature ==
		    EFI_ACPI_5_0_DIFFERENTIATED_SYSTEM_DESCRIPTION_TABLE_SIGNATURE) {
			Facp = (EFI_ACPI_5_0_FIXED_ACPI_DESCRIPTION_TABLE *)
			    FindAcpiTableBySignature(
				Xsdt,
				EFI_ACPI_5_0_FIXED_ACPI_DESCRIPTION_TABLE_SIGNATURE,
				&EntryIndex, OemTableId, OemRevision);

			if (Facp != NULL) { // DSDT override
				EFI_ACPI_DESCRIPTION_HEADER *oldDsdt;

				oldDsdt =
				    (EFI_ACPI_DESCRIPTION_HEADER *)Facp->XDsdt;
				Facp->Dsdt = (UINT32)(UINTN)AcpiHdr;
				Facp->XDsdt = (UINT64)(UINTN)AcpiHdr;

				/* update FADT checksum */
				Facp->Header.Checksum = 0;
				Facp->Header.Checksum =
				    checksum(
					(void *)Facp,
					sizeof(
					    EFI_ACPI_5_0_FIXED_ACPI_DESCRIPTION_TABLE));
				UpdateAcpiGnvs(AcpiHdr, oldDsdt);
				ewdbg("DSDT override\n");
			} else {
				Status = EFI_ABORTED; // can't find FACP
				acpi_mem_free((VOID **)&NewTable, *TableKey);
				break;
			}
		} else {
			// Try to find the table to replace
			if (FindAcpiTableBySignature(Xsdt, AcpiHdr->Signature,
						     &EntryIndex, OemTableId, OemRevision) != NULL) {
				XsdtEntry[EntryIndex] = (UINT32)(UINTN)AcpiHdr;
			} else { // new table, to add
				if (EntryNum >= MAX_XSDT_HEADER_ENTRIES) {
					Status = EFI_OUT_OF_RESOURCES;
					acpi_mem_free((VOID **)&NewTable, *TableKey);
					break;
				}
				XsdtEntry[EntryNum] = (UINT32)(UINTN)AcpiHdr;
				EntryNum++;
			}
		}

		AcpiHdr->Checksum = 0;
		AcpiHdr->Checksum = checksum((void *)AcpiHdr, AcpiHdr->Length);
	}

	Xsdt->Length =
	    sizeof(EFI_ACPI_DESCRIPTION_HEADER) + EntryNum * sizeof(UINT64);
	Xsdt->Checksum = 0;
	Xsdt->Checksum = checksum((void *)Xsdt, Xsdt->Length);

	return Status;
}

static EFIAPI EFI_STATUS UninstallAcpiTable(__attribute__((__unused__))
					    EFI_ACPI_TABLE_PROTOCOL * This,
					    __attribute__((__unused__))
					    UINTN TableKey)
{
	return EFI_UNSUPPORTED;
}

static EFI_STATUS acpi_init(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;
	EFI_CONFIGURATION_TABLE *table;
	EFI_GUID guid;

	static

	    struct _EFI_ACPI_TABLE_PROTOCOL acpi_protocol_default = {
		.InstallAcpiTable = InstallAcpiTable,
		.UninstallAcpiTable = UninstallAcpiTable,
	    };

	struct _EFI_ACPI_TABLE_PROTOCOL *acpi_protocol;

	if (!st)
		return EFI_INVALID_PARAMETER;

	p_st = st;

	ret = get_rsdp(&Rsdp, &guid);

	if (EFI_ERROR(ret))
		return ret;

	ret = interface_init(
	    st, &acpi_protocol_guid, &handle, &acpi_protocol_default,
	    sizeof(acpi_protocol_default), (void **)&acpi_protocol);

	if (EFI_ERROR(ret))
		return ret;

	ret = conf_table_new(st, &guid, &table);

	if (EFI_ERROR(ret))
		return ret;

	table->VendorTable = Rsdp;

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

	ret = interface_free(st, &acpi_protocol_guid, handle);

	if (EFI_ERROR(ret))
		return ret;

	return conf_table_free(st, &guid);
}

ewdrv_t acpi_drv = {.name = "acpi",
		    .description = "Look-up for ACPI tables and provide them "
				   "via the System Table,support ACPI protocol",
		    .init = acpi_init,
		    .exit = acpi_exit};
