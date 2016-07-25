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

#include <stdio.h>

#include <protocol.h>

#include "conout.h"

static EFIAPI EFI_STATUS
_text_reset(struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
	    BOOLEAN ExtendedVerification)
{
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
_text_output_string(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
		    CHAR16 *String)
{
	while (*String)
		fprintf(stdout, "%c", *String++);

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
_text_test_string(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
		  __attribute__((__unused__)) CHAR16 *String)
{
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
_text_query_mode(struct _SIMPLE_TEXT_OUTPUT_INTERFACE     *This,
		 UINTN ModeNumber,
		 UINTN *Columns,
		 UINTN *Rows)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_text_set_mode(struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
	       UINTN ModeNumber)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_text_set_attribute(struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
		    UINTN Attribute)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_text_clear_screen(struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_text_set_cursor_position(struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
			  UINTN Column,
			  UINTN Row)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_text_enable_cursor(struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
		    BOOLEAN Enable)
{
	return EFI_UNSUPPORTED;
}

static SIMPLE_TEXT_OUTPUT_MODE mode = {
	.MaxMode = 0,
	.Mode = 0,
	.Attribute = 0,
	.CursorColumn = 0,
	.CursorRow = 0,
	.CursorVisible = FALSE
};

static SIMPLE_TEXT_OUTPUT_INTERFACE conout_struct = {
	.Reset = _text_reset,

	.OutputString = _text_output_string,
	.TestString = _text_test_string,

	.QueryMode = _text_query_mode,
	.SetMode = _text_set_mode,
	.SetAttribute = _text_set_attribute,

	.ClearScreen = _text_clear_screen,
	.SetCursorPosition = _text_set_cursor_position,
	.EnableCursor = _text_enable_cursor,

	.Mode = &mode
};

static EFI_HANDLE conout_handle;
static EFI_GUID conout_guid = SIMPLE_TEXT_OUTPUT_PROTOCOL;

EFI_STATUS linux_register_conout(EFI_HANDLE *handle, SIMPLE_TEXT_OUTPUT_INTERFACE **interface)
{
	EFI_STATUS ret;

	if (!handle)
		return EFI_INVALID_PARAMETER;

	if (conout_handle)
		return EFI_ALREADY_STARTED;

	conout_handle = NULL;
	ret = ew_install_protocol_interface(&conout_handle, &conout_guid,
					    EFI_NATIVE_INTERFACE, &conout_struct);
	if (!EFI_ERROR(ret)) {
		*handle = conout_handle;
		*interface = &conout_struct;
	}

	return ret;
}

EFI_STATUS linux_unregister_conout()
{
	EFI_STATUS ret;

	if (!conout_handle)
		return EFI_INVALID_PARAMETER;

	ret = ew_uninstall_protocol_interface(conout_handle, &conout_guid,
					      &conout_struct);
	if (!EFI_ERROR(ret))
		conout_handle = NULL;

	return ret;
}
