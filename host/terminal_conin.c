/*
 * Copyright (c) 2017, Intel Corporation
 * All rights reserved.
 *
 * Author: Léo Sartre <leo.sartre@intel.com>
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

#include <termios.h>
#include <unistd.h>
#include <ewlog.h>

#include "terminal_conin.h"

static EFIAPI VOID
terminal_conin_wait_for_key(__attribute__((__unused__)) EFI_EVENT Event,
			    VOID *Context)
{
	int n;
	char buf[3];
	EFI_SYSTEM_TABLE *st = (EFI_SYSTEM_TABLE *)Context;

	/* reading 3 char at a time to catch ANSI escape sequences
	 * generated when pressing arrow keys */
	n = read(STDIN_FILENO, buf, sizeof(buf) * sizeof(buf[0]));
	if (n > 0)
		uefi_call_wrapper(st->BootServices->SignalEvent, 1,
				  st->ConIn->WaitForKey);
}

static struct termios *old_term;

static EFI_STATUS makeraw_term(VOID)
{
	int ret;
	struct termios raw_term;

	old_term = malloc(sizeof(*old_term));
	if (!old_term)
		return EFI_LOAD_ERROR;

	ret = tcgetattr(0, old_term);
	if (ret)
		return EFI_LOAD_ERROR;

	raw_term = *old_term;
	cfmakeraw(&raw_term);
	raw_term.c_oflag |= ONLCR | OPOST;
	ret = tcsetattr(0, TCSANOW, &raw_term);
	if (ret)
		return EFI_LOAD_ERROR;

	return EFI_SUCCESS;
}

EFI_STATUS terminal_conin_init(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;

	if (!st)
		return EFI_INVALID_PARAMETER;

	if (!st->ConsoleInHandle)
		return EFI_NOT_FOUND;

	ret = uefi_call_wrapper(st->BootServices->CreateEvent, 5,
				EVT_NOTIFY_WAIT,
				TPL_NOTIFY,
				terminal_conin_wait_for_key,
				(VOID *)st,
				&st->ConIn->WaitForKey);
	if (EFI_ERROR(ret)) {
		ewerr("Fail to create WaitForKey event");
		return ret;
	}

	return makeraw_term();
}

EFI_STATUS terminal_conin_exit(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;

	if (old_term) {
		tcsetattr(0, TCSANOW, old_term);
		free(old_term);
	}

	ret = uefi_call_wrapper(st->BootServices->CloseEvent, 1,
				st->ConIn->WaitForKey);
	if (EFI_ERROR(ret)) {
		ewerr("Fail to destroy WaitForKey event");
		return ret;
	}

	return EFI_SUCCESS;
}

ewdrv_t terminal_conin_drv = {
	.name = "terminal_conin",
	.description = "Provide keyboard terminal support",
	.init = terminal_conin_init,
	.exit = terminal_conin_exit
};
