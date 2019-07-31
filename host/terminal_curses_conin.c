/*
 * Author: Léo Sartre <leo.sartre@geebol.fr>
 */

#include <ewlog.h>

#include "terminal_curses_conin.h"
#include "curses_utils.h"

static EFIAPI VOID
wait_for_input_key(__attribute__((__unused__)) EFI_EVENT Event,
				   VOID *Context)
{
	int ret;
	EFI_SYSTEM_TABLE *st = (EFI_SYSTEM_TABLE *)Context;

	ret = nodelay(stdscr, FALSE);
	if (ret == ERR)
		ewerr("Failed to configure blocking input mode, don't trust the trigger");

	ret = getch();
	if (ret == ERR)
		ewerr("Suspicious key read");
	/* Send the signal anyway to unblock the caller */
	uefi_call_wrapper(st->BootServices->SignalEvent, 1,
					  st->ConIn->WaitForKey);
}

static EFIAPI EFI_STATUS
reset_input(__attribute__((__unused__)) struct _SIMPLE_INPUT_INTERFACE *This,
			__attribute__((__unused__)) BOOLEAN ExtendedVerification)
{
	flushinp();
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
read_key(__attribute__((__unused__)) struct _SIMPLE_INPUT_INTERFACE *This,
		 __attribute__((__unused__)) EFI_INPUT_KEY *Key)
{
	int k = getch();
	if (k == ERR)
		return EFI_NOT_READY;

	Key->ScanCode = curses_key_to_efi_scancode(k);
	if (Key->ScanCode == SCAN_NULL)
		Key->UnicodeChar = k & 0xff;
	else
		Key->UnicodeChar = CHAR_NULL;

	return EFI_SUCCESS;
}

static SIMPLE_INPUT_INTERFACE saved_conin;

EFI_STATUS terminal_conin_init(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;
	static SIMPLE_INPUT_INTERFACE new_conin = {
		.Reset = reset_input,
		.ReadKeyStroke = read_key
	};

	if (!st)
		return EFI_INVALID_PARAMETER;

	if (!st->ConsoleInHandle)
		return EFI_NOT_FOUND;

	memcpy(&saved_conin, st->ConIn, sizeof(*st->ConIn));
	memcpy(st->ConIn, &new_conin, sizeof(new_conin));

	ret = uefi_call_wrapper(st->BootServices->CreateEvent, 5,
							EVT_NOTIFY_WAIT,
							TPL_APPLICATION,
							wait_for_input_key,
							(VOID *)st,
							&st->ConIn->WaitForKey);
	if (EFI_ERROR(ret)) {
		ewerr("Fail to create WaitForKey event");
		return ret;
	}

	return EFI_SUCCESS;
}

EFI_STATUS terminal_conin_exit(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;

	ret = uefi_call_wrapper(st->BootServices->CloseEvent, 1,
							st->ConIn->WaitForKey);
	if (EFI_ERROR(ret)) {
		ewerr("Fail to destroy WaitForKey event");
		return ret;
	}

	memcpy(st->ConIn, &saved_conin, sizeof(saved_conin));

	return EFI_SUCCESS;
}
