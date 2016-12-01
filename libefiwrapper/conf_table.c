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

#include "external.h"
#include "lib.h"

EFI_STATUS conf_table_new(EFI_SYSTEM_TABLE *st,
			  EFI_GUID *guid,
			  EFI_CONFIGURATION_TABLE **table)
{
	EFI_CONFIGURATION_TABLE *tables;

	if (!st || !guid || !table)
		return EFI_INVALID_PARAMETER;

	if (!st->ConfigurationTable) {
		if (st->NumberOfTableEntries)
			return EFI_INVALID_PARAMETER;

		tables = malloc(sizeof(EFI_CONFIGURATION_TABLE));
		if (!tables)
			return EFI_OUT_OF_RESOURCES;

		goto success;
	}

	if (!st->NumberOfTableEntries)
		return EFI_INVALID_PARAMETER;

	tables = realloc(st->ConfigurationTable,
			 (st->NumberOfTableEntries + 1) *
			 sizeof(EFI_CONFIGURATION_TABLE));
	if (!tables)
		return EFI_OUT_OF_RESOURCES;


success:
	st->ConfigurationTable = tables;
	*table = &tables[st->NumberOfTableEntries];
	memcpy(&(*table)->VendorGuid, guid, sizeof(*guid));
	st->NumberOfTableEntries++;

	return EFI_SUCCESS;
}

EFI_STATUS conf_table_free(EFI_SYSTEM_TABLE *st, EFI_GUID *guid)
{
	EFI_CONFIGURATION_TABLE *tables = NULL;
	size_t i;

	if (!st || !st->ConfigurationTable || !st->NumberOfTableEntries)
		return EFI_INVALID_PARAMETER;

	if (st->NumberOfTableEntries == 1) {
		if (!guidcmp(&st->ConfigurationTable[0].VendorGuid, guid))
			return EFI_NOT_FOUND;
		free(st->ConfigurationTable);
		goto success;
	}

	for (i = 0; i < st->NumberOfTableEntries; i++) {
		if (!guidcmp(&st->ConfigurationTable[i].VendorGuid, guid))
			continue;

		tables = malloc((st->NumberOfTableEntries - 1) *
				sizeof(EFI_CONFIGURATION_TABLE));
		if (!tables)
			return EFI_OUT_OF_RESOURCES;

		memcpy(tables, st->ConfigurationTable, i *
		       sizeof(EFI_CONFIGURATION_TABLE));
		memcpy(&tables[i], &st->ConfigurationTable[i + 1],
		       (st->NumberOfTableEntries - i) *
		       sizeof(EFI_CONFIGURATION_TABLE));
		break;
	}

	if (i == st->NumberOfTableEntries)
		return EFI_NOT_FOUND;

	free(st->ConfigurationTable);

success:
	st->ConfigurationTable = tables;
	st->NumberOfTableEntries--;
	return EFI_SUCCESS;
}
