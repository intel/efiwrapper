/*
 * Author: Léo Sartre <leo.sartre@geebol.fr>
 */

#ifndef _TERMINAL_CURSES_CONIN_H_
#define _TERMINAL_CURSES_CONIN_H_

#include <efi.h>

EFI_STATUS terminal_conin_init(EFI_SYSTEM_TABLE *st);
EFI_STATUS terminal_conin_exit(EFI_SYSTEM_TABLE *st);

#endif /* _TERMINAL_CURSES_CONIN_H_ */
