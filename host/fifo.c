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

#include <stdlib.h>
#include <pthread.h>
#include <ewlog.h>

#include "fifo.h"

typedef struct cell {
	void *data;
	struct cell *next;
} cell_t;

struct fifo {
	cell_t *first;
	cell_t *last;
	pthread_mutex_t lock;
};

fifo_t fifo_new(void)
{
	int ret;
	fifo_t f;

	f = calloc(sizeof(struct fifo), 1);
	if (!f)
		return NULL;

	ret = pthread_mutex_init(&f->lock, NULL);
	if (ret) {
		free(f);
		return NULL;
	}

	return f;
}

void fifo_free(fifo_t f)
{
	void *data;

	while (fifo_get(f, &data) == EFI_SUCCESS)
		;
	free(f);
}

static EFI_STATUS _fifo_get(fifo_t f, void **data)
{
	cell_t *cell;

	if (!f->first)
		return EFI_NOT_FOUND;

	cell = f->first;
	*data = cell->data;
	f->first = f->first->next;

	free(cell);
	return EFI_SUCCESS;
}

EFI_STATUS fifo_get(fifo_t f, void **data)
{
	EFI_STATUS ret;
	int pret;

	if (!f)
		return EFI_INVALID_PARAMETER;

	pret = pthread_mutex_lock(&f->lock);
	if (pret)
		return EFI_DEVICE_ERROR;

	ret = _fifo_get(f, data);

	pret = pthread_mutex_unlock(&f->lock);
	if (pret)
		return EFI_DEVICE_ERROR;

	return ret;
}

static EFI_STATUS _fifo_put(fifo_t f, void *data)
{
	cell_t *cell;

	cell = calloc(sizeof(*cell), 1);
	if (!cell)
		return EFI_OUT_OF_RESOURCES;
	cell->data = data;

	if (!f->first) {
		f->first = f->last = cell;
		return EFI_SUCCESS;
	}

	f->last->next = cell;
	if (f->first == f->last)
		f->first->next = cell;
	f->last = cell;

	return EFI_SUCCESS;
}

EFI_STATUS fifo_put(fifo_t f, void *data)
{
	EFI_STATUS ret;
	int pret;

	if (!f)
		return EFI_INVALID_PARAMETER;

	pret = pthread_mutex_lock(&f->lock);
	if (pret)
		return EFI_DEVICE_ERROR;

	ret = _fifo_put(f, data);

	pret = pthread_mutex_unlock(&f->lock);
	if (pret)
		return EFI_DEVICE_ERROR;

	return ret;
}
