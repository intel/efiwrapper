/*
 * Author: Léo Sartre <leo.sartre@geebol.fr>
 */

#include <ewlog.h>
#include <ewlib.h> /* For ARRAY_SIZE */

#include <ncurses.h>

#include "terminal_curses_conout.h"
#include "curses_utils.h"

static const SIMPLE_TEXT_OUTPUT_MODE modes[] = {
	{2, 0, EFI_TEXT_ATTR(EFI_WHITE, EFI_BLACK), 80, 25, TRUE}, /* 80x25 white on black text mode */
	{2, 1, EFI_TEXT_ATTR(EFI_WHITE, EFI_BLACK), 80, 50, TRUE}  /* 80x50 white on black text mode */
};
static SIMPLE_TEXT_OUTPUT_MODE current_mode;

typedef union {
	struct { UINTN fg: 4; UINTN bg: 3; };
	UINTN raw;
} color;

static int current_color_pair;

static EFIAPI EFI_STATUS
conout_output_string(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
					 CHAR16 *String)
{
	while (*String) {
		echochar(*String++);
	}
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
conout_test_string(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
				   __attribute__((__unused__)) CHAR16 *String)
{
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
conout_set_attribute(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
					 UINTN Attribute)
{
	int ret;
	bool support_colors;
	color c;

	support_colors = has_colors();
	if (!support_colors) {
		ewdbg("Device does not support colors");
		return EFI_DEVICE_ERROR;
	}

	c.raw = Attribute;
	if (efi_color_to_ncurses_color(c.fg) < 0 || efi_color_to_ncurses_color(c.bg) < 0) {
		ewdbg("Failed to translate efi colors to ncurses colors");
		return EFI_DEVICE_ERROR;
	}

	ret = attroff(COLOR_PAIR(current_color_pair++));
	if (ret == ERR) {
		ewdbg("Failed to disable previous window colors attribute");
		return EFI_DEVICE_ERROR;
	}

	ret = init_pair(current_color_pair, efi_color_to_ncurses_color(c.fg), efi_color_to_ncurses_color(c.bg));
	if (ret == ERR) {
		ewdbg("Failed to initialize ncurses colors pair");
		return EFI_DEVICE_ERROR;
	}

	ret = attron(COLOR_PAIR(current_color_pair));
	if (ret == ERR) {
		ewdbg("Failed to set new window colors attribute");
		return EFI_DEVICE_ERROR;
	}

	current_mode.Attribute = Attribute;

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
conout_set_cursor_position(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
						   UINTN Column,
						   UINTN Row)
{
	int ret;

	ret = move(Row, Column);
	if (ret == ERR) {
		ewdbg("Failed to move cursor to (%ld, %ld)", Row, Column);
		return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
conout_clear_screen(struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This)
{
	int ret;

	ret = clear();
	if (ret == ERR) {
		ewdbg("Failed to clear the screen");
		return EFI_DEVICE_ERROR;
	}

	return conout_set_cursor_position(This, 0, 0);
}

static EFIAPI EFI_STATUS
conout_reset(struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
			 __attribute__((__unused__)) BOOLEAN ExtendedVerification)
{
	int ret;

	ret = resize_term(current_mode.CursorRow, current_mode.CursorColumn);
	if (ret == ERR) {
		ewdbg("Failed to resize terminal to %dx%d", current_mode.CursorRow, current_mode.CursorColumn);
		return EFI_DEVICE_ERROR;
	}

	return conout_clear_screen(This);
}

static EFIAPI EFI_STATUS
conout_query_mode(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
				  UINTN ModeNumber,
				  UINTN *Columns,
				  UINTN *Rows)
{
	if (ModeNumber >= ARRAY_SIZE(modes))
		return EFI_UNSUPPORTED;

	*Rows = modes[ModeNumber].CursorRow;
	*Columns = modes[ModeNumber].CursorColumn;

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
conout_set_mode(struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
				UINTN ModeNumber)
{
	if (ModeNumber >= ARRAY_SIZE(modes))
		return EFI_UNSUPPORTED;

	memcpy(&current_mode, &modes[ModeNumber], sizeof(modes[ModeNumber]));

	return conout_reset(This, FALSE);
}

static EFIAPI EFI_STATUS
conout_enable_cursor(__attribute__((__unused__)) struct _SIMPLE_TEXT_OUTPUT_INTERFACE *This,
					 BOOLEAN Enable)
{
	int ret;
	int visibility = Enable ? 1 : 0;

	ret = curs_set(visibility);
	if (ret == ERR)
		return EFI_DEVICE_ERROR;

	return EFI_SUCCESS;
}

static SIMPLE_TEXT_OUTPUT_INTERFACE saved_conout;

EFI_STATUS terminal_conout_init(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;

	static SIMPLE_TEXT_OUTPUT_INTERFACE new_conout = {
		.Reset = conout_reset,
		.OutputString = conout_output_string,
		.TestString = conout_test_string,
		.QueryMode = conout_query_mode,
		.SetMode = conout_set_mode,
		.SetAttribute = conout_set_attribute,
		.ClearScreen = conout_clear_screen,
		.SetCursorPosition = conout_set_cursor_position,
		.EnableCursor = conout_enable_cursor,
		.Mode = &current_mode
	};

	if (!st)
		return EFI_INVALID_PARAMETER;

	if (!st->ConsoleOutHandle)
		return EFI_NOT_FOUND;

	/* Set the default 80x25 mode */
	ret = conout_set_mode(&new_conout, 0);
	if (EFI_ERROR(ret)) {
		ewerr("Failed to set default text mode (80x25)");
		return ret;
	}

	ret = conout_enable_cursor(&new_conout, current_mode.CursorVisible);
	if (EFI_ERROR(ret)) {
		ewerr("Failed to %s cursor", current_mode.CursorVisible ? "show" : "hide");
		return ret;
	}

	memcpy(&saved_conout, st->ConOut, sizeof(*st->ConOut));
	memcpy(st->ConOut, &new_conout, sizeof(new_conout));

	return EFI_SUCCESS;
}

EFI_STATUS terminal_conout_exit(EFI_SYSTEM_TABLE *st)
{
	memcpy(st->ConOut, &saved_conout, sizeof(saved_conout));

	return EFI_SUCCESS;
}
