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

#include "external.h"
#include "interface.h"
#include "media.h"

media_t *media_new(storage_t *storage)
{
	static UINT32 id;
	media_t *media;

	media = calloc(1, sizeof(*media));
	if (!media)
		return NULL;

	media->m.MediaId = ++id;
	media->m.MediaPresent = TRUE;
	media->m.BlockSize = storage->blk_sz;
	media->m.LastBlock = storage->blk_cnt - 1;

	media->storage = storage;

	return media;
}

/* Randomly generated GUID */
static EFI_GUID media_guid = { 0xd4b39595, 0xe31b, 0x48d5,
			       { 0xac, 0xa3, 0x03, 0x1c, 0x61, 0x42, 0xc7, 0xab } };

EFI_STATUS media_register(EFI_SYSTEM_TABLE *st, media_t *media,
			  EFI_HANDLE *handle)
{
	return uefi_call_wrapper(st->BootServices->InstallProtocolInterface, 4,
				 handle, &media_guid,
				 EFI_NATIVE_INTERFACE, media);
}

EFI_STATUS media_free(EFI_SYSTEM_TABLE *st, EFI_HANDLE handle)
{
	return interface_free(st, &media_guid, handle);
}
