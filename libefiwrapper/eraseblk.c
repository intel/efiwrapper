/*
 * Copyright (c) 2018, Intel Corporation
 * All rights reserved.
 *
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

#include "eraseblk.h"
#include "protocol/EraseBlock.h"
#include "external.h"
#include "interface.h"

#include <efilib.h>


typedef struct eraseblock {
	EFI_ERASE_BLOCK_PROTOCOL interface;
	media_t *media;
} eraseblk_t;

static EFI_STATUS storage_erase_block(EFI_ERASE_BLOCK_PROTOCOL *This,
				 UINT32 MediaId, EFI_LBA LBA, UINTN Size)
{
	EFI_STATUS ret;
	eraseblk_t *eraseblk = (eraseblk_t *)This;
	media_t *media;

	if (!This)
		return EFI_INVALID_PARAMETER;

	if (!eraseblk->media)
		return EFI_INVALID_PARAMETER;

	media = eraseblk->media;
	if (media->m.MediaId != MediaId)
		return EFI_MEDIA_CHANGED;

	if (media->storage->erase)
		ret = media->storage->erase(media->storage, LBA, Size);
	else
		return EFI_UNSUPPORTED;

	return ret;
}

EFI_STATUS
EFIAPI
erase_block (
  IN     EFI_ERASE_BLOCK_PROTOCOL      *This,
  IN     UINT32                        MediaId,
  IN     EFI_LBA                       Lba,
  __attribute__((__unused__)) IN OUT EFI_ERASE_BLOCK_TOKEN         *Token,
  IN     UINTN                         Size
  )
{
	return storage_erase_block(This, MediaId, Lba, Size);
}

static EFI_GUID erase_block_guid = EFI_ERASE_BLOCK_PROTOCOL_GUID;

EFI_STATUS erase_block_init(EFI_SYSTEM_TABLE *st, media_t *media,
			EFI_HANDLE *handle)
{
	EFI_STATUS ret;
	eraseblk_t *eraseblk;

	static eraseblk_t erase_block_default = {
		.interface = {
			.Revision = EFI_ERASE_BLOCK_PROTOCOL_REVISION,
			.EraseLengthGranularity = 1,
			.EraseBlocks = erase_block,
		}
	};

	ret = interface_init(st, &erase_block_guid, handle,
			     &erase_block_default, sizeof(erase_block_default),
			     (void **)&eraseblk);
	if (EFI_ERROR(ret))
		return ret;

	eraseblk->media = media;

	return EFI_SUCCESS;
}

EFI_STATUS erase_block_free(EFI_SYSTEM_TABLE *st, EFI_HANDLE handle)
{
	return interface_free(st, &erase_block_guid, handle);
}
