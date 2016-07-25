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

#include <protocol.h>

#include "conin.h"

static EFIAPI EFI_STATUS
_input_reset(__attribute__((__unused__)) struct _SIMPLE_INPUT_INTERFACE *This,
	     __attribute__((__unused__)) BOOLEAN ExtendedVerification)
{
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
_input_read_key(__attribute__((__unused__)) struct _SIMPLE_INPUT_INTERFACE *This,
		__attribute__((__unused__)) EFI_INPUT_KEY *Key)
{
	return EFI_NOT_FOUND;
}

static SIMPLE_INPUT_INTERFACE conin_struct = {
	.Reset = _input_reset,
	.ReadKeyStroke = _input_read_key,
	.WaitForKey = NULL
};

static EFI_HANDLE conin_handle;
static EFI_GUID conin_guid = SIMPLE_TEXT_OUTPUT_PROTOCOL;

EFI_STATUS linux_register_conin(EFI_HANDLE *handle, SIMPLE_INPUT_INTERFACE **interface)
{
	EFI_STATUS ret;

	if (!handle)
		return EFI_INVALID_PARAMETER;

	if (conin_handle)
		return EFI_ALREADY_STARTED;

	ret = ew_install_protocol_interface(&conin_handle, &conin_guid,
					    EFI_NATIVE_INTERFACE, &conin_struct);
	if (!EFI_ERROR(ret)) {
		*handle = conin_handle;
		*interface = &conin_struct;
	}

	return ret;
}

EFI_STATUS linux_unregister_conin(VOID)
{
	EFI_STATUS ret;

	if (!conin_handle)
		return EFI_INVALID_PARAMETER;

	ret = ew_uninstall_protocol_interface(conin_handle, &conin_guid,
					      &conin_struct);
	if (!EFI_ERROR(ret))
		conin_handle = NULL;

	return ret;
}
