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

#include <stddef.h>

#include "ewdrv.h"
#include "ewlog.h"

EFI_STATUS ewdrv_init(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret = EFI_SUCCESS;
	size_t i;

	if (!ew_drivers)
		return EFI_UNSUPPORTED;

	for (i = 0; ew_drivers[i]; i++) {
		ret = ew_drivers[i]->init(st);
		if (EFI_ERROR(ret))
			break;
		ewdbg("'%s' driver succesfully initialized",
		      ew_drivers[i]->name);
	}

	if (EFI_ERROR(ret))
		ewerr("Failed to initialize '%s' driver", ew_drivers[i]->name);
	return ret;
}

EFI_STATUS ewdrv_free(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;
	size_t i;

	if (!ew_drivers)
		return EFI_UNSUPPORTED;

	for (i = 0; ew_drivers[i]; i++) {
		if (!ew_drivers[i]->free)
			continue;
		ret = ew_drivers[i]->free(st);
		if (EFI_ERROR(ret))
			return ret;
	}

	return EFI_SUCCESS;
}
