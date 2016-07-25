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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

#include <efiwrapper.h>
#include <platform.h>

#include "runtime_services.h"

#define GUID_LEN 36
#define GUID_FMT "%08x-%04x-%04x-%02x%02x-%02x%02x%02x%02x%02x%02x"

static EFIAPI EFI_STATUS
_get_variable(CHAR16 *VariableName, EFI_GUID *VendorGuid, UINT32 *Attributes,
	      UINTN *DataSize, VOID *Data)
{
	efivar_t *var;

	if (!VariableName || !VendorGuid || !Attributes || !DataSize || !Data)
		return EFI_INVALID_PARAMETER;

	var = efivar_get(VariableName, VendorGuid, NULL);
	if (!var)
		return EFI_NOT_FOUND;

	if (var->size > *DataSize) {
		*DataSize = var->size;
		return EFI_BUFFER_TOO_SMALL;
	}

	*Attributes = var->attributes;
	*DataSize = var->size;
	ew_memcpy(Data, var->data, var->size);

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
_get_next_variable_name(UINTN *VariableNameSize,
			CHAR16 *VariableName,
			EFI_GUID *VendorGuid)
{
	efivar_t *var;
	size_t name_size;

	if (!VariableNameSize || !VariableName || !VendorGuid)
		return EFI_INVALID_PARAMETER;

	if (VariableName[0] == '\0')
		var = efivar_get_first();
	else {
		var = efivar_get(VariableName, VendorGuid, NULL);
		var = var ? var->next : NULL;
	}

	if (!var)
		return EFI_NOT_FOUND;

	name_size = (ew_str16len(var->name) + 1) * sizeof(*var->name);
	if (name_size > *VariableNameSize)
		return EFI_BUFFER_TOO_SMALL;

	ew_memcpy(VariableName, var->name, name_size);
	ew_memcpy(VendorGuid, &var->guid, sizeof(var->guid));

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
_set_variable(CHAR16 *VariableName, EFI_GUID *VendorGuid, UINT32 Attributes,
	      UINTN DataSize, VOID *Data)
{
	EFI_STATUS ret;
	efivar_t *var, *prev;

	if (!VariableName || !VendorGuid)
		return EFI_INVALID_PARAMETER;

	if (Attributes & EFI_VARIABLE_AUTHENTICATED_WRITE_ACCESS ||
	    Attributes & EFI_VARIABLE_TIME_BASED_AUTHENTICATED_WRITE_ACCESS)
		return EFI_UNSUPPORTED;

	var = efivar_get(VariableName, VendorGuid, &prev);

	if (!Data) {
		/* if (var->attributes & EFI_VARIABLE_NON_VOLATILE) { */
		/* 	ret = unlink_var_file(var); */
		/* 	if (EFI_ERROR(ret)) */
		/* 		return ret; */
		/* } */
		return efivar_del(var, prev);
	}


	if (var) {
		if (Attributes != var->attributes)
			return EFI_INVALID_PARAMETER;

		ret = efivar_update(var, DataSize, Data);
		if (EFI_ERROR(ret))
			return ret;

		/* if (var->attributes & EFI_VARIABLE_NON_VOLATILE) */
		/* 	return write_var_file(var); */

		return EFI_SUCCESS;
	}

	var = efivar_new(VariableName, VendorGuid, Attributes, DataSize, Data);
	if (!var)
		return EFI_OUT_OF_RESOURCES;

	/* if (var->attributes & EFI_VARIABLE_NON_VOLATILE) { */
	/* 	ret = write_var_file(var); */
	/* 	if (EFI_ERROR(ret)) { */
	/* 		ew_free_var(var); */
	/* 		return ret; */
	/* 	} */
	/* } */

	efivar_add(var);

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
_get_time(EFI_TIME *Time,
	  EFI_TIME_CAPABILITIES *Capabilities)
{
	/* int retp; */
	/* time_t cur_time; */
	/* struct tm *now; */

	/* if (!Time) */
	/* 	return EFI_INVALID_PARAMETER; */

	/* if (Capabilities) */
	/* 	return EFI_UNSUPPORTED; */

	/* retp = time(&cur_time); */
	/* if (retp == -1) */
	/* 	return EFI_DEVICE_ERROR; */

	/* now = localtime(&cur_time); */
	/* ew_memset(Time, 0, sizeof(*Time)); */

	/* Time->Year = now->tm_year + 1900; */
	/* Time->Month = now->tm_mon + 1; */
	/* Time->Day = now->tm_mday; */
	/* Time->Hour = now->tm_hour; */
	/* Time->Minute = now->tm_min; */
	/* Time->Second = now->tm_sec; */
	/* Time->TimeZone = EFI_UNSPECIFIED_TIMEZONE; */
	/* if (now->tm_isdst) */
	/* 	Time->Daylight = 1 << EFI_TIME_IN_DAYLIGHT; */

	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_set_time(__attribute__((__unused__)) EFI_TIME *Time)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_get_wakeup_time(__attribute__((__unused__)) BOOLEAN *Enabled,
		 __attribute__((__unused__)) BOOLEAN *Pending,
		 __attribute__((__unused__)) EFI_TIME *Time)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_set_wakeup_time(__attribute__((__unused__)) BOOLEAN Enable,
		 __attribute__((__unused__)) EFI_TIME *Time)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_set_virtual_address_map(__attribute__((__unused__)) UINTN MemoryMapSize,
			 __attribute__((__unused__)) UINTN DescriptorSize,
			 __attribute__((__unused__)) UINT32 DescriptorVersion,
			 __attribute__((__unused__)) EFI_MEMORY_DESCRIPTOR *VirtualMap)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_convert_pointer(__attribute__((__unused__)) UINTN DebugDisposition,
		 __attribute__((__unused__)) VOID **Address)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_get_next_high_monotonic_count(__attribute__((__unused__)) UINT32 *HighCount)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_reset_system(__attribute__((__unused__)) EFI_RESET_TYPE ResetType,
	      __attribute__((__unused__)) EFI_STATUS ResetStatus,
	      __attribute__((__unused__)) UINTN DataSize,
	      __attribute__((__unused__)) CHAR16 *ResetData)
{
	/* exit(EXIT_SUCCESS); */
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_update_capsule(__attribute__((__unused__)) EFI_CAPSULE_HEADER **CapsuleHeaderArray,
		__attribute__((__unused__)) UINTN CapsuleCount,
		__attribute__((__unused__)) EFI_PHYSICAL_ADDRESS ScatterGatherList)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_query_capsule_capabilities(__attribute__((__unused__)) EFI_CAPSULE_HEADER **CapsuleHeaderArray,
			    __attribute__((__unused__)) UINTN CapsuleCount,
			    __attribute__((__unused__)) UINT64 *MaximumCapsuleSize,
			    __attribute__((__unused__)) EFI_RESET_TYPE *ResetType)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_query_variable_info(__attribute__((__unused__)) UINT32 Attributes,
		     __attribute__((__unused__)) UINT64 *MaximumVariableStorageSize,
		     __attribute__((__unused__)) UINT64 *RemainingVariableStorageSize,
		     __attribute__((__unused__)) UINT64 *MaximumVariableSize)
{
	return EFI_UNSUPPORTED;
}

static EFI_RUNTIME_SERVICES runtime_services_struct = {
	.Hdr = {
		.Signature = 0,
		.Revision = 0,
		.HeaderSize = 0,
		.CRC32 = 0,
		.Reserved = 0
	},

	.GetTime = _get_time,
	.SetTime = _set_time,
	.GetWakeupTime = _get_wakeup_time,
	.SetWakeupTime = _set_wakeup_time,

	.SetVirtualAddressMap = _set_virtual_address_map,
	.ConvertPointer = _convert_pointer,

	.GetVariable = _get_variable,
	.GetNextVariableName = _get_next_variable_name,
	.SetVariable = _set_variable,

	.GetNextHighMonotonicCount = _get_next_high_monotonic_count,
	.ResetSystem = _reset_system,

	.UpdateCapsule = _update_capsule,

	.QueryCapsuleCapabilities = _query_capsule_capabilities,

	.QueryVariableInfo = _query_variable_info
};

EFI_STATUS payload_runtime_services(EFI_RUNTIME_SERVICES **rts_p)
{
	*rts_p = &runtime_services_struct;
	return EFI_SUCCESS;
}
