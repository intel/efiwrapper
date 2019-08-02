#ifndef _CURSES_UTILS_H
#define _CURSES_UTILS_H

#include <efi.h>
#include <ncurses.h>

static inline UINT16 curses_key_to_efi_scancode(int key)
{
	switch (key) {
	case KEY_UP:
		return SCAN_UP;
	case KEY_DOWN:
		return SCAN_DOWN;
	case KEY_RIGHT:
		return SCAN_RIGHT;
	case KEY_LEFT:
		return SCAN_LEFT;
	case KEY_HOME:
		return SCAN_HOME;
	case KEY_END:
		return SCAN_END;
	case KEY_IL:
		return SCAN_INSERT;
	case KEY_DL:
		return SCAN_DELETE;
	case KEY_PPAGE:
		return SCAN_PAGE_UP;
	case KEY_NPAGE:
		return SCAN_PAGE_DOWN;
	case KEY_F(1):
		return SCAN_F1;
	case KEY_F(2):
		return SCAN_F2;
	case KEY_F(3):
		return SCAN_F3;
	case KEY_F(4):
		return SCAN_F4;
	case KEY_F(5):
		return SCAN_F5;
	case KEY_F(6):
		return SCAN_F6;
	case KEY_F(7):
		return SCAN_F7;
	case KEY_F(8):
		return SCAN_F8;
	case KEY_F(9):
		return SCAN_F9;
	case KEY_F(10):
		return SCAN_F10;
	case KEY_F(11):
		return SCAN_F11;
	case KEY_F(12):
		return SCAN_F12;
	case 27:
		return SCAN_ESC;
	default:
		return SCAN_NULL;
	}
}

static inline short efi_color_to_ncurses_color(UINTN color)
{
    switch (color) {
    case EFI_BLACK:
	return COLOR_BLACK;
    case EFI_BLUE:
	return COLOR_BLUE;
    case EFI_GREEN:
	return COLOR_GREEN;
    case EFI_CYAN:
	return COLOR_CYAN;
    case EFI_RED:
	return COLOR_RED;
    case EFI_MAGENTA:
	return COLOR_MAGENTA;
    case EFI_BROWN:
    case EFI_LIGHTGRAY:
    case EFI_BRIGHT: /*also EFI_DARKGRAY*/
    case EFI_LIGHTBLUE:
    case EFI_LIGHTGREEN:
    case EFI_LIGHTCYAN:
    case EFI_LIGHTRED:
    case EFI_LIGHTMAGENTA:
	return COLOR_YELLOW;
    case EFI_YELLOW:
	return COLOR_YELLOW;
    case EFI_WHITE:
	return COLOR_WHITE;
    default:
	return -1;
    }
};

#endif	/* _CURSES_UTILS_H */
