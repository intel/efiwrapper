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

#ifndef _EWVAR_H_
#define _EWVAR_H_

#include <efi.h>
#include <efiapi.h>

typedef struct ewvar {
	struct ewvar *next;
	CHAR16 *name;
	EFI_GUID guid;
	UINT32 attributes;
	void *data;
	UINTN size;
} ewvar_t;

ewvar_t *ewvar_new(CHAR16 *name, EFI_GUID *guid, UINT32 attr,
		     UINTN size, VOID *data);
void ewvar_add(ewvar_t *var);

ewvar_t *ewvar_get(const CHAR16 *name, EFI_GUID *guid, ewvar_t **prev_p);
ewvar_t *ewvar_get_first(void);
EFI_STATUS ewvar_update(ewvar_t *var, UINTN size, VOID *data);

EFI_STATUS ewvar_del(ewvar_t *var, ewvar_t *prev);

void ewvar_free(ewvar_t *var);
void ewvar_free_all(void);

typedef struct ewvar_storage {
	EFI_STATUS (*load)(void);
	EFI_STATUS (*save)(ewvar_t *);
	EFI_STATUS (*delete)(ewvar_t *);
} ewvar_storage_t;

EFI_STATUS ewvar_register_storage(ewvar_storage_t *s);
EFI_STATUS ewvar_unregister_storage(void);

#endif	/* _EWVAR_H_ */
