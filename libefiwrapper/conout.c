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

#include "conout.h"
#include "interface.h"
#include "lib.h"

static EFIAPI EFI_STATUS
conout_reset(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
	     __attribute__((__unused__)) BOOLEAN ExtendedVerification)
{
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
conout_output_string(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
		     CHAR16 *String)
{
	while (*String)
		printf("%c", *String++);

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
conout_test_string(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
		   __attribute__((__unused__)) CHAR16 *String)
{
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
conout_query_mode(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
		  __attribute__((__unused__)) UINTN ModeNumber,
		  __attribute__((__unused__)) UINTN *Columns,
		  __attribute__((__unused__)) UINTN *Rows)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
conout_set_mode(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
		__attribute__((__unused__)) UINTN ModeNumber)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
conout_set_attribute(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
		     __attribute__((__unused__)) UINTN Attribute)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
conout_clear_screen(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
conout_set_cursor_position(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
			   __attribute__((__unused__)) UINTN Column,
			   __attribute__((__unused__)) UINTN Row)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
conout_enable_cursor(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
		     __attribute__((__unused__)) BOOLEAN Enable)
{
	return EFI_UNSUPPORTED;
}

static SIMPLE_TEXT_OUTPUT_MODE mode;

static EFI_GUID conout_guid = SIMPLE_TEXT_OUTPUT_PROTOCOL;

EFI_STATUS conout_init(EFI_SYSTEM_TABLE *st)
{
	static SIMPLE_TEXT_OUTPUT_INTERFACE conout_default = {
		.Reset = conout_reset,
		.OutputString = conout_output_string,
		.TestString = conout_test_string,
		.QueryMode = conout_query_mode,
		.SetMode = conout_set_mode,
		.SetAttribute = conout_set_attribute,
		.ClearScreen = conout_clear_screen,
		.SetCursorPosition = conout_set_cursor_position,
		.EnableCursor = conout_enable_cursor,
		.Mode = &mode
	};

	if (!st)
		return EFI_INVALID_PARAMETER;

	if (st->ConsoleOutHandle)
		return EFI_ALREADY_STARTED;

	return interface_init(st, &conout_guid, &st->ConsoleOutHandle,
			      &conout_default, sizeof(conout_default),
			      (void **)&st->ConOut);
}

EFI_STATUS conout_free(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;

	ret = interface_free(st, &conout_guid, st->ConsoleOutHandle);
	if (EFI_ERROR(ret))
		return ret;

	st->ConsoleOutHandle = NULL;
	st->ConOut = NULL;

	return EFI_SUCCESS;
}
