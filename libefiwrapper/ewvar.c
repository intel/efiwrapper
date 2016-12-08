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

#include "ewvar.h"
#include "lib.h"

static ewvar_t *EFI_VARS;
static ewvar_storage_t *storage;

ewvar_t *ewvar_new(CHAR16 *name, EFI_GUID *guid, UINT32 attr,
		   UINTN size, VOID *data)
{
	EFI_STATUS ret;
	ewvar_t *var;

	var = calloc(1, sizeof(*var));
	if (!var)
		return NULL;

	var->name = str16dup(name);
	if (!var->name)
		goto err;

	memcpy(&var->guid, guid, sizeof(var->guid));
	var->attributes = attr;
	var->size = size;

	var->data = malloc(size);
	if (!var->data)
		goto err;

	memcpy(var->data, data, size);

	if (attr & EFI_VARIABLE_NON_VOLATILE &&
	    storage && storage->save) {
		ret = storage->save(var);
		if (EFI_ERROR(ret))
			goto err;
	}

	return var;

err:
	ewvar_free(var);
	return NULL;
}

void ewvar_free(ewvar_t *var)
{
	if (var->data)
		free(var->data);
	if (var->name)
		free(var->name);
	free(var);
}

void ewvar_free_all(void)
{
	ewvar_t *var, *next;

	for (var = EFI_VARS; var; var = next) {
		next = var->next;
		ewvar_free(var);
	}
	EFI_VARS = NULL;
}

void ewvar_add(ewvar_t *var)
{
	var->next = EFI_VARS;
	EFI_VARS = var;
}

ewvar_t *ewvar_get(const CHAR16 *name, EFI_GUID *guid, ewvar_t **prev_p)
{
	ewvar_t *var, *prev = NULL;

	for (var = EFI_VARS; var; var = var->next) {
		if (!str16cmp(name, var->name) && !guidcmp(&var->guid, guid))
			break;
		prev = var;
	}

	if (var && prev_p)
		*prev_p = prev;

	return var;
}

ewvar_t *ewvar_get_first(void)
{
	return EFI_VARS;
}

EFI_STATUS ewvar_del(ewvar_t *var, ewvar_t *prev)
{
	EFI_STATUS ret;
	if (!var)
		return EFI_NOT_FOUND;

	if (var->attributes & EFI_VARIABLE_NON_VOLATILE &&
	    storage && storage->delete) {
		ret = storage->delete(var);
		if (EFI_ERROR(ret))
			return ret;
	}

	if (prev)
		prev->next = var->next;
	else
		EFI_VARS = var->next;
	ewvar_free(var);

	return EFI_SUCCESS;
}

EFI_STATUS ewvar_update(ewvar_t *var, UINTN size, VOID *data)
{
	if (var->attributes & EFI_VARIABLE_APPEND_WRITE) {
		var->data = realloc(var->data, var->size + size);
		if (!var->data)
			return EFI_OUT_OF_RESOURCES;

		memcpy((char *)var->data + var->size, data, size);
		var->size += size;
	} else {
		var->data = realloc(var->data, size);
		if (!var->data)
			return EFI_OUT_OF_RESOURCES;

		var->size = size;
		if (!var->data)
			return EFI_OUT_OF_RESOURCES;
		memcpy(var->data, data, size);
	}

	if (var->attributes & EFI_VARIABLE_NON_VOLATILE &&
	    storage && storage->save)
		return storage->save(var);

	return EFI_SUCCESS;
}

EFI_STATUS ewvar_register_storage(ewvar_storage_t *s)
{
	if (!s)
		return EFI_INVALID_PARAMETER;

	storage = s;
	if (s->load)
		return s->load();

	return EFI_SUCCESS;
}

EFI_STATUS ewvar_unregister_storage(void)
{
	storage = NULL;

	return EFI_SUCCESS;
}
