/*
 * Copyright (c) 2016-2020, Intel Corporation
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

#ifndef _IMAGE_H_
#define _IMAGE_H_

#if defined(HOST)
#include <setjmp.h>
#endif
#include <ewdrv.h>

typedef struct image {
	EFI_LOADED_IMAGE prot;
	void *data;
#if defined(HOST)
	jmp_buf jmp;
#endif
	EFI_IMAGE_ENTRY_POINT entry;
	EFI_STATUS exit_status;
} image_t;

extern ewdrv_t image_drv;

EFI_STATUS image_init(EFI_SYSTEM_TABLE *st);
EFI_STATUS image_free(EFI_SYSTEM_TABLE *st);

#endif	/* _IMAGE_H_ */
