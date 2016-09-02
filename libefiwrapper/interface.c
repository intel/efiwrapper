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

#include "interface.h"
#include "external.h"

EFI_STATUS interface_init(EFI_SYSTEM_TABLE *st, EFI_GUID *guid,
			  EFI_HANDLE *handle,
			  VOID *base, size_t base_size,
			  VOID **interface)
{
	EFI_STATUS ret;

	if (!st || !guid || !handle || !base || !interface)
		return EFI_INVALID_PARAMETER;

	*interface = malloc(base_size);
	if (!*interface)
		return EFI_OUT_OF_RESOURCES;

	memcpy(*interface, base, base_size);

	ret = uefi_call_wrapper(st->BootServices->InstallProtocolInterface, 4,
				handle, guid, EFI_NATIVE_INTERFACE, *interface);
	if (EFI_ERROR(ret))
		free(*interface);

	return ret;
}

EFI_STATUS interface_free(EFI_SYSTEM_TABLE *st, EFI_GUID *guid,
			  EFI_HANDLE handle)
{
	EFI_STATUS ret;
	VOID *interface;

	if (!handle)
		return EFI_INVALID_PARAMETER;

	ret = uefi_call_wrapper(st->BootServices->HandleProtocol, 3,
				handle, guid, (VOID **)&interface);
	if (EFI_ERROR(ret))
		return ret;

	ret = uefi_call_wrapper(st->BootServices->UninstallProtocolInterface, 3,
				handle, guid, interface);
	if (EFI_ERROR(ret))
		return ret;

	free(interface);

	return EFI_SUCCESS;
}
