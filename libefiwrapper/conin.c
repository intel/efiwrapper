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

#include "conin.h"
#include "interface.h"

static EFIAPI EFI_STATUS
conin_reset(__attribute__((__unused__)) struct _SIMPLE_INPUT_INTERFACE *This,
	    __attribute__((__unused__)) BOOLEAN ExtendedVerification)
{
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
conin_read_key(__attribute__((__unused__)) struct _SIMPLE_INPUT_INTERFACE *This,
	       EFI_INPUT_KEY *Key)
{
	Key->UnicodeChar = getchar();
	return EFI_SUCCESS;
}

static EFI_GUID guid = SIMPLE_TEXT_OUTPUT_PROTOCOL;

EFI_STATUS conin_init(EFI_SYSTEM_TABLE *st)
{
	static SIMPLE_INPUT_INTERFACE conin_default = {
		.Reset = conin_reset,
		.ReadKeyStroke = conin_read_key,
		.WaitForKey = (EFI_HANDLE)conin_init
	};

	if (!st)
		return EFI_INVALID_PARAMETER;

	if (st->ConsoleInHandle)
		return EFI_ALREADY_STARTED;

	return interface_init(st, &guid, &st->ConsoleInHandle,
			      &conin_default, sizeof(conin_default),
			      (void **)&st->ConIn);
}

EFI_STATUS conin_free(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;

	ret = interface_free(st, &guid, st->ConsoleInHandle);
	if (EFI_ERROR(ret))
		return ret;

	st->ConsoleInHandle = NULL;
	st->ConIn = NULL;

	return EFI_SUCCESS;
}
