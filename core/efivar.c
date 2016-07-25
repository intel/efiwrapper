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

#include <efiwrapper.h>

#include "efivar.h"

static efivar_t *EFI_VARS;

efivar_t *efivar_new(CHAR16 *name, EFI_GUID *guid, UINT32 attr,
		     UINTN size, VOID *data)
{
	efivar_t *var;

	var = ew_calloc(sizeof(*var), 1);
	if (!var)
		return NULL;

	var->name = ew_str16dup(name);
	if (!var->name)
		goto err;

	ew_memcpy(&var->guid, guid, sizeof(var->guid));
	var->attributes = attr;
	var->size = size;

	var->data = ew_malloc(size);
	if (!var->data)
		goto err;

	ew_memcpy(var->data, data, size);
	return var;

err:
	efivar_free(var);
	return NULL;
}


void efivar_free(efivar_t *var)
{
	if (var->data)
		ew_free(var->data);
	if (var->name)
		ew_free(var->name);
	ew_free(var);
}

void efivar_free_all(void)
{
	efivar_t *var, *next;

	for (var = EFI_VARS; var; var = next) {
		next = var->next;
		efivar_free(var);
	}
	EFI_VARS = NULL;
}

void efivar_add(efivar_t *var)
{
	var->next = EFI_VARS;
	EFI_VARS = var;
}

efivar_t *efivar_get(const CHAR16 *name, EFI_GUID *guid, efivar_t **prev_p)
{
	efivar_t *var, *prev = NULL;

	for (var = EFI_VARS; var; var = var->next) {
		if (ew_str16len(name) == ew_str16len(var->name) &&
		    !ew_memcmp(var->name, name, ew_str16len(name) * sizeof(*name)) &&
		    !ew_memcmp(&var->guid, guid, sizeof(*guid)))
			break;
		prev = var;
	}

	if (var && prev_p)
		*prev_p = prev;

	return var;
}

efivar_t *efivar_get_first(void)
{
	return EFI_VARS;
}

EFI_STATUS efivar_del(efivar_t *var, efivar_t *prev)
{
	if (!var)
		return EFI_NOT_FOUND;

	if (prev)
		prev->next = var->next;
	else
		EFI_VARS = var->next;
	efivar_free(var);

	return EFI_SUCCESS;
}

EFI_STATUS efivar_update(efivar_t *var, UINTN size, VOID *data)
{
	if (var->attributes & EFI_VARIABLE_APPEND_WRITE) {
		var->data = ew_realloc(var->data, var->size + size,
				       var->size);
		if (!var->data)
			return EFI_OUT_OF_RESOURCES;

		ew_memcpy(var->data + var->size, data, size);
		var->size += size;
	} else {
		var->data = ew_realloc(var->data, size, var->size);
		if (!var->data)
			return EFI_OUT_OF_RESOURCES;

		var->size = size;
		if (!var->data)
			return EFI_OUT_OF_RESOURCES;
		ew_memcpy(var->data, data, size);
	}

	return EFI_SUCCESS;
}
