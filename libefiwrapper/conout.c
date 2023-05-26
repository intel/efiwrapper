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

#include "conout.h"
#include "interface.h"
#include "lib.h"

static EFIAPI EFI_STATUS
conout_reset(struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
	     __attribute__((__unused__)) BOOLEAN ExtendedVerification)
{
	This->SetCursorPosition(This, 0, 0);
	This->ClearScreen(This);
	This->EnableCursor(This, FALSE);
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
conout_output_string(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
		     CHAR16 *String)
{
	while (*String)
		putchar(*String++);

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
		  UINTN ModeNumber,
		  UINTN *Columns,
		  UINTN *Rows)
{
	if (ModeNumber)
		return EFI_UNSUPPORTED;

	*Columns = 80;
	*Rows = 25;

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
conout_set_mode(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
		UINTN ModeNumber)
{
	return ModeNumber ? EFI_UNSUPPORTED : EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
conout_set_attribute(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
		     UINTN Attribute)
{
	static const int FOREGROUND_COLOR[] = {
		[EFI_BLACK]		    = 30,
		[EFI_BLUE]		    = 34,
		[EFI_GREEN]		    = 32,
		[EFI_CYAN]		    = 36,
		[EFI_RED]		    = 31,
		[EFI_MAGENTA]		    = 35,
		[EFI_BROWN]		    = 37,	/* Default to white */
		[EFI_LIGHTGRAY]		    = 90,
		[EFI_BRIGHT]		    = 37,	/* Default to white */
		[EFI_LIGHTBLUE]		    = 94,
		[EFI_LIGHTGREEN]	    = 92,
		[EFI_LIGHTCYAN]		    = 96,
		[EFI_LIGHTRED]		    = 91,
		[EFI_LIGHTMAGENTA]	    = 95,
		[EFI_YELLOW]		    = 33,
		[EFI_WHITE]		    = 37
	};

	static const int BACKGROUND_COLOR[]	= {
		[EFI_BACKGROUND_BLACK >> 4]	= 40,
		[EFI_BACKGROUND_BLUE >> 4]	= 44,
		[EFI_BACKGROUND_GREEN >> 4]	= 42,
		[EFI_BACKGROUND_CYAN >> 4]	= 46,
		[EFI_BACKGROUND_RED >> 4]	= 41,
		[EFI_BACKGROUND_MAGENTA >> 4]   = 45,
		[EFI_BACKGROUND_BROWN >> 4]	= 40,	/* Default to black */
		[EFI_BACKGROUND_LIGHTGRAY >> 4] = 47,
	};

	if (Attribute == 0x0) {
		printf("\e[0m");
		return EFI_SUCCESS;
	}

	printf("\e[%d;%dm", FOREGROUND_COLOR[Attribute & 0xF],
	       BACKGROUND_COLOR[(Attribute >> 4) & 0x7]);

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
conout_clear_screen(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This)
{
	printf("\e[2J");
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
conout_set_cursor_position(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
			   UINTN Column,
			   UINTN Row)
{
	printf("\e[%zd;%zdH", Row + 1, Column + 1);
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
conout_enable_cursor(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
		     BOOLEAN Enable)
{
	printf(Enable ? "\e[?25h" : "\e[?25l");
	return EFI_SUCCESS;
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
