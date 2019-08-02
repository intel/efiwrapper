/*
 * Author: Léo Sartre <leo.sartre@geebol.fr>
 */

#include <ewlog.h>
#include <interface.h>

#include <ncurses.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "terminal_curses.h"
#include "terminal_curses_conin.h"
#include "terminal_curses_conout.h"

static SCREEN *screen;

EFI_STATUS terminal_curses_init(EFI_SYSTEM_TABLE *st)
{
	int ret;

	screen = newterm(NULL, stdout, stdin);
	if (!screen) {
		ewerr("Failed to initialize ncurses");
		return EFI_LOAD_ERROR;
	}

	ret = nodelay(stdscr, TRUE);
	if (ret == ERR) {
		endwin();
		ewerr("Failed to set terminal in non blocking mode");
		return EFI_LOAD_ERROR;
	}

	ret = set_escdelay(0);
	if (ret == ERR)
		ewdbg("Warning ESC key will not be usable as signgle key");

	ret = keypad(stdscr, TRUE);
	if (ret == ERR)
		ewdbg("Warning keypad keys will not be available");

	ret = noecho();
	if (ret == ERR)
		ewdbg("Warning failed to set noecho mode");

	ret = start_color();
	if (ret == ERR)
		ewdbg("Warning colors will not be available");

	return terminal_conin_init(st) & terminal_conout_init(st);
}

EFI_STATUS terminal_curses_exit(EFI_SYSTEM_TABLE *st)
{
	if (!st)
		return EFI_INVALID_PARAMETER;

	endwin();
	delscreen(screen);

	return terminal_conin_exit(st) & terminal_conout_exit(st);
}

ewdrv_t terminal_curses_drv = {
	.name = "terminal_curses",
	.description = "Terminal input output support based on ncurses",
	.init = terminal_curses_init,
	.exit = terminal_curses_exit
};
