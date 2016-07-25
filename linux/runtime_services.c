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
#include <sys/stat.h>
#include <fcntl.h>
#include <dirent.h>
#include <time.h>

#include <efiwrapper.h>
#include <platform.h>

#include "common.h"
#include "runtime_services.h"

#define GUID_LEN 36
#define GUID_FMT "%08x-%04x-%04x-%02x%02x-%02x%02x%02x%02x%02x%02x"

static char *vardir;

static char *var_path(const CHAR16 *name, const EFI_GUID *guid)
{
	char *path = NULL;
	size_t size;
	int count;

	/* path:<VAR-DIRECTORY>/<VARIABLE-NAME>-<VARIABLE-GUID> */
	size = ew_strlen(vardir) + 1 + ew_str16len(name) + 1 + GUID_LEN + 1;

	path = ew_malloc(size);
	if (!path)
		goto err;

	*path = '\0';
	strcat(path, vardir);
	strcat(path, "/");

	count = snprintf(path + ew_strlen(path), GUID_LEN + 1, GUID_FMT,
			 guid->Data1, guid->Data2, guid->Data3,
			 guid->Data4[0], guid->Data4[1], guid->Data4[2],
			 guid->Data4[3], guid->Data4[4], guid->Data4[5],
			 guid->Data4[6], guid->Data4[7]);
	if (count != GUID_LEN)
		goto err;

	strcat(path, "-");
	count = ew_str16_to_str(name, path + ew_strlen(path));
	if (count == -1)
		goto err;

	return path;

err:
	ew_free(path);
	return NULL;

}

/* The EFI variables are stored with the following filename format:
 * <VARIABLE-NAME>-<VARIABLE-GUID>.  The first 4 bytes of the file
 * stores the EFI variable attributes.
 */
static EFI_STATUS load_var(char *path)
{
	efivar_t *var;
	char *filename, *varname;
	struct stat sb;
	EFI_STATUS ret;
	int retp, fd = -1;

	filename = basename(path);
	if (!filename)
		return EFI_INVALID_PARAMETER;

	if (ew_strlen(filename) <= GUID_LEN + 1)
		return EFI_INVALID_PARAMETER;

	retp = stat(path, &sb);
	if (retp == -1)
		return EFI_INVALID_PARAMETER;

	if (!S_ISREG(sb.st_mode))
		return EFI_INVALID_PARAMETER;

	if (sb.st_size <= sizeof(var->attributes))
		return EFI_INVALID_PARAMETER;

	var = ew_calloc(sizeof(*var), 1);
	if (!var)
		return EFI_OUT_OF_RESOURCES;

	ret = ew_str_to_guid(filename, &var->guid);
	if (EFI_ERROR(ret)) {
		error("Failed to convert %s to GUID", filename);
		goto err;
	}

	fd = open(path, O_RDONLY);
	if (fd == -1) {
		ret = EFI_DEVICE_ERROR;
		goto err;
	}

	ret = read_fd(fd, &var->attributes, sizeof(var->attributes));
	if (EFI_ERROR(ret))
		return ret;

	var->size = sb.st_size - sizeof(var->attributes);
	var->data = ew_malloc(var->size);
	if (!var->data) {
		ret = EFI_OUT_OF_RESOURCES;
		goto err;
	}
	ret = read_fd(fd, var->data, var->size);
	if (EFI_ERROR(ret))
		goto err;

	varname = filename + GUID_LEN + 1;
	var->name = ew_str_to_str16_p(varname);
	if (!var->name) {
		ret = EFI_OUT_OF_RESOURCES;
		goto err;
	}

	efivar_add(var);
	close(fd);

	return EFI_SUCCESS;

err:
	if (fd != -1)
		close(fd);

	efivar_free(var);
	return ret;
}

static EFI_STATUS load_vars(char *new_vardir)
{
	DIR *dp;
	struct dirent *ep;
	char *path;
	EFI_STATUS ret;
	int retp;

	if (!new_vardir)
		return -1;

	vardir = new_vardir;

	dp = opendir(vardir);	/* TODO: create directory if it does not exist. */
	if (!dp)
		return -1;

	for (ep = readdir(dp); ep; ep = readdir(dp)) {
		if (!strcmp(ep->d_name, ".") || !strcmp(ep->d_name, ".."))
			continue;

		retp = asprintf(&path, "%s/%s", vardir, ep->d_name);
		if (retp == -1) {
			efivar_free_all();
			ret = EFI_OUT_OF_RESOURCES;
			goto exit;
		}

		ret = load_var(path);
		ew_free(path);
		if (EFI_ERROR(ret))
			continue;
	}

	ret = EFI_SUCCESS;

exit:
	closedir(dp);
	return ret;
}

static EFI_STATUS unlink_var_file(efivar_t *var)
{
	int retp;
	char *path;

	path = var_path(var->name, &var->guid);
	if (!path)
		return EFI_OUT_OF_RESOURCES;

	retp = unlink(path);
	ew_free(path);
	if (retp == -1)
		return EFI_DEVICE_ERROR;

	return EFI_SUCCESS;
}

static EFI_STATUS write_var_file(efivar_t *var)
{
	int retp, fd = -1;
	EFI_STATUS ret = EFI_DEVICE_ERROR;
	char *path = NULL;

	path = var_path(var->name, &var->guid);
	if (!path) {
		ret = EFI_OUT_OF_RESOURCES;
		goto exit;
	}

	fd = open(path, O_WRONLY | O_CREAT, 0644);
	if (fd == -1)
		goto exit;

	retp = ftruncate(fd, 0);
	if (retp == -1)
		goto exit;

	ret = write_fd(fd, &var->attributes, sizeof(var->attributes));
	if (EFI_ERROR(ret))
		goto exit;

	ret = write_fd(fd, var->data, var->size);
	if (EFI_ERROR(ret))
		goto exit;

	ret = EFI_SUCCESS;

exit:
	if (fd != -1)
		close(fd);
	ew_free(path);
	return ret;
}

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
		if (!var)
			return EFI_NOT_FOUND;
		if (var->attributes & EFI_VARIABLE_NON_VOLATILE) {
			ret = unlink_var_file(var);
			if (EFI_ERROR(ret))
				return ret;
		}
		return efivar_del(var, prev);
	}


	if (var) {
		if (Attributes != var->attributes)
			return EFI_INVALID_PARAMETER;

		ret = efivar_update(var, DataSize, Data);
		if (EFI_ERROR(ret))
			return ret;

		if (var->attributes & EFI_VARIABLE_NON_VOLATILE)
			return write_var_file(var);

		return EFI_SUCCESS;
	}

	var = efivar_new(VariableName, VendorGuid, Attributes, DataSize, Data);
	if (!var)
		return EFI_OUT_OF_RESOURCES;

	if (var->attributes & EFI_VARIABLE_NON_VOLATILE) {
		ret = write_var_file(var);
		if (EFI_ERROR(ret)) {
			efivar_free(var);
			return ret;
		}
	}

	efivar_add(var);

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
_get_time(EFI_TIME *Time,
	  EFI_TIME_CAPABILITIES *Capabilities)
{
	int retp;
	time_t cur_time;
	struct tm *now;

	if (!Time)
		return EFI_INVALID_PARAMETER;

	if (Capabilities)
		return EFI_UNSUPPORTED;

	retp = time(&cur_time);
	if (retp == -1)
		return EFI_DEVICE_ERROR;

	now = localtime(&cur_time);
	ew_memset(Time, 0, sizeof(*Time));

	Time->Year = now->tm_year + 1900;
	Time->Month = now->tm_mon + 1;
	Time->Day = now->tm_mday;
	Time->Hour = now->tm_hour;
	Time->Minute = now->tm_min;
	Time->Second = now->tm_sec;
	Time->TimeZone = EFI_UNSPECIFIED_TIMEZONE;
	if (now->tm_isdst)
		Time->Daylight = 1 << EFI_TIME_IN_DAYLIGHT;

	return EFI_SUCCESS;
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
	exit(EXIT_SUCCESS);
	return EFI_SUCCESS;
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

EFI_STATUS linux_runtime_services(EFI_RUNTIME_SERVICES **rts_p)
{
	EFI_STATUS ret;
	char *tmpdir;

	tmpdir = getenv("EFIVAR_DIR");
	if (!tmpdir) {
		tmpdir = getenv("TMPDIR");
		if (!tmpdir)
			tmpdir = "/tmp";
	}

	ret = load_vars(tmpdir);
	if (EFI_ERROR(ret))
		return ret;

	*rts_p = &runtime_services_struct;
	return EFI_SUCCESS;
}

EFI_STATUS linux_free_runtime_services(EFI_RUNTIME_SERVICES *rts_p)
{
	efivar_free_all();
	return EFI_SUCCESS;
}
