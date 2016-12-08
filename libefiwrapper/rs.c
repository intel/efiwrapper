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

#include "ewvar.h"
#include "lib.h"
#include "rs.h"

static EFIAPI EFI_STATUS
rs_get_variable(CHAR16 *VariableName, EFI_GUID *VendorGuid, UINT32 *Attributes,
		UINTN *DataSize, VOID *Data)
{
	ewvar_t *var;

	if (!VariableName || !VendorGuid || !Attributes || !DataSize || !Data)
		return EFI_INVALID_PARAMETER;

	var = ewvar_get(VariableName, VendorGuid, NULL);
	if (!var)
		return EFI_NOT_FOUND;

	if (var->size > *DataSize) {
		*DataSize = var->size;
		return EFI_BUFFER_TOO_SMALL;
	}

	*Attributes = var->attributes;
	*DataSize = var->size;
	memcpy(Data, var->data, var->size);

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
rs_get_next_variable_name(UINTN *VariableNameSize,
			  CHAR16 *VariableName,
			  EFI_GUID *VendorGuid)
{
	ewvar_t *var;
	size_t name_size;

	if (!VariableNameSize || !VariableName || !VendorGuid)
		return EFI_INVALID_PARAMETER;

	if (VariableName[0] == '\0')
		var = ewvar_get_first();
	else {
		var = ewvar_get(VariableName, VendorGuid, NULL);
		var = var ? var->next : NULL;
	}

	if (!var)
		return EFI_NOT_FOUND;

	name_size = (str16len(var->name) + 1) * sizeof(*var->name);
	if (name_size > *VariableNameSize) {
		*VariableNameSize = name_size;
		return EFI_BUFFER_TOO_SMALL;
	}

	memcpy(VariableName, var->name, name_size);
	memcpy(VendorGuid, &var->guid, sizeof(var->guid));

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
rs_set_variable(CHAR16 *VariableName, EFI_GUID *VendorGuid, UINT32 Attributes,
		UINTN DataSize, VOID *Data)
{
	EFI_STATUS ret;
	ewvar_t *var, *prev;

	if (!VariableName || !VendorGuid)
		return EFI_INVALID_PARAMETER;

	if (Attributes & EFI_VARIABLE_AUTHENTICATED_WRITE_ACCESS ||
	    Attributes & EFI_VARIABLE_TIME_BASED_AUTHENTICATED_WRITE_ACCESS)
		return EFI_UNSUPPORTED;

	var = ewvar_get(VariableName, VendorGuid, &prev);

	if (!Data) {
		if (!var)
			return EFI_NOT_FOUND;
		return ewvar_del(var, prev);
	}

	if (var) {
		if (Attributes != var->attributes)
			return EFI_INVALID_PARAMETER;

		ret = ewvar_update(var, DataSize, Data);
		if (EFI_ERROR(ret))
			return ret;

		return EFI_SUCCESS;
	}

	var = ewvar_new(VariableName, VendorGuid, Attributes, DataSize, Data);
	if (!var)
		return EFI_OUT_OF_RESOURCES;

	ewvar_add(var);

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
rs_get_time(__attribute__((__unused__)) EFI_TIME *Time,
	    __attribute__((__unused__)) EFI_TIME_CAPABILITIES *Capabilities)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
rs_set_time(__attribute__((__unused__)) EFI_TIME *Time)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
rs_get_wakeup_time(__attribute__((__unused__)) BOOLEAN *Enabled,
		   __attribute__((__unused__)) BOOLEAN *Pending,
		   __attribute__((__unused__)) EFI_TIME *Time)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
rs_set_wakeup_time(__attribute__((__unused__)) BOOLEAN Enable,
		   __attribute__((__unused__)) EFI_TIME *Time)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
rs_set_virtual_address_map(__attribute__((__unused__)) UINTN MemoryMapSize,
			   __attribute__((__unused__)) UINTN DescriptorSize,
			   __attribute__((__unused__)) UINT32 DescriptorVersion,
			   __attribute__((__unused__)) EFI_MEMORY_DESCRIPTOR *VirtualMap)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
rs_convert_pointer(__attribute__((__unused__)) UINTN DebugDisposition,
		   __attribute__((__unused__)) VOID **Address)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
rs_get_next_high_monotonic_count(__attribute__((__unused__)) UINT32 *HighCount)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
rs_reset_system(__attribute__((__unused__)) EFI_RESET_TYPE ResetType,
		__attribute__((__unused__)) EFI_STATUS ResetStatus,
		__attribute__((__unused__)) UINTN DataSize,
		__attribute__((__unused__)) CHAR16 *ResetData)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
rs_update_capsule(__attribute__((__unused__)) EFI_CAPSULE_HEADER **CapsuleHeaderArray,
		  __attribute__((__unused__)) UINTN CapsuleCount,
		  __attribute__((__unused__)) EFI_PHYSICAL_ADDRESS ScatterGatherList)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
rs_query_capsule_capabilities(__attribute__((__unused__)) EFI_CAPSULE_HEADER **CapsuleHeaderArray,
			      __attribute__((__unused__)) UINTN CapsuleCount,
			      __attribute__((__unused__)) UINT64 *MaximumCapsuleSize,
			      __attribute__((__unused__)) EFI_RESET_TYPE *ResetType)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
rs_query_variable_info(__attribute__((__unused__)) UINT32 Attributes,
		       __attribute__((__unused__)) UINT64 *MaximumVariableStorageSize,
		       __attribute__((__unused__)) UINT64 *RemainingVariableStorageSize,
		       __attribute__((__unused__)) UINT64 *MaximumVariableSize)
{
	return EFI_UNSUPPORTED;
}

static EFI_RUNTIME_SERVICES runtime_services_default = {
	.Hdr = {
		.Signature = EFI_RUNTIME_SERVICES_SIGNATURE,
		.Revision = EFI_RUNTIME_SERVICES_REVISION,
		.HeaderSize = sizeof(EFI_TABLE_HEADER)
	},
	.GetTime = rs_get_time,
	.SetTime = rs_set_time,
	.GetWakeupTime = rs_get_wakeup_time,
	.SetWakeupTime = rs_set_wakeup_time,
	.SetVirtualAddressMap = rs_set_virtual_address_map,
	.ConvertPointer = rs_convert_pointer,
	.GetVariable = rs_get_variable,
	.GetNextVariableName = rs_get_next_variable_name,
	.SetVariable = rs_set_variable,
	.GetNextHighMonotonicCount = rs_get_next_high_monotonic_count,
	.ResetSystem = rs_reset_system,
	.UpdateCapsule = rs_update_capsule,
	.QueryCapsuleCapabilities = rs_query_capsule_capabilities,
	.QueryVariableInfo = rs_query_variable_info
};

EFI_STATUS rs_init(EFI_SYSTEM_TABLE *st)
{
	EFI_RUNTIME_SERVICES *rs;

	if (!st || !st->RuntimeServices)
		return EFI_INVALID_PARAMETER;

	rs = st->RuntimeServices;
	memcpy(rs, &runtime_services_default, sizeof(*rs));

	return crc32((void *)rs, sizeof(*rs), &rs->Hdr.CRC32);
}
