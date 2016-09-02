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

#include "blockio.h"
#include "external.h"
#include "interface.h"

#include <efilib.h>

static EFIAPI EFI_STATUS
blockio_reset(__attribute__((__unused__)) EFI_BLOCK_IO *This,
	      __attribute__((__unused__)) BOOLEAN ExtendedVerification)
{
	return EFI_SUCCESS;
}

static EFI_STATUS blockio_access(BOOLEAN read, EFI_BLOCK_IO *This,
				 UINT32 MediaId, EFI_LBA LBA,
				 UINTN BufferSize, VOID *Buffer)
{
	media_t *media;
	UINT32 blksz;
	UINTN size;
	EFI_LBA count;

	if (!This)
		return EFI_INVALID_PARAMETER;

	if (!This->Media)
		return EFI_NO_MEDIA;

	media = (media_t *)This->Media;
	if (MediaId != media->m.MediaId)
		return EFI_MEDIA_CHANGED;

	blksz = media->m.BlockSize;
	if (!blksz)
		return EFI_INVALID_PARAMETER;

	if (BufferSize % blksz)
		return EFI_BAD_BUFFER_SIZE;

	size = BufferSize / blksz;
	if (read)
		count = media->storage->read(media->storage, LBA, size, Buffer);
	else
		count = media->storage->write(media->storage, LBA, size, Buffer);

	return count == size ? EFI_SUCCESS : EFI_DEVICE_ERROR;
}

static EFIAPI EFI_STATUS
blockio_write(EFI_BLOCK_IO *This, UINT32 MediaId, EFI_LBA LBA,
	      UINTN BufferSize, VOID *Buffer)
{
	return blockio_access(FALSE, This, MediaId, LBA, BufferSize, Buffer);
}

static EFIAPI EFI_STATUS
blockio_read(EFI_BLOCK_IO *This, UINT32 MediaId, EFI_LBA LBA,
	     UINTN BufferSize, VOID *Buffer)
{
	return blockio_access(TRUE, This, MediaId, LBA, BufferSize, Buffer);
}

static EFIAPI EFI_STATUS
blockio_flush(__attribute__((__unused__)) EFI_BLOCK_IO *This)
{
	return EFI_SUCCESS;
}

static EFI_GUID blockio_guid = BLOCK_IO_PROTOCOL;

EFI_STATUS blockio_init(EFI_SYSTEM_TABLE *st, media_t *media,
			EFI_HANDLE *handle)
{
	static EFI_BLOCK_IO blockio_default = {
		.Revision = EFI_BLOCK_IO_INTERFACE_REVISION3,
		.Reset = blockio_reset,
		.ReadBlocks = blockio_read,
		.WriteBlocks = blockio_write,
		.FlushBlocks = blockio_flush
	};
	EFI_STATUS ret;
	EFI_BLOCK_IO *blockio;

	ret = interface_init(st, &blockio_guid, handle,
			     &blockio_default, sizeof(blockio_default),
			     (void **)&blockio);
	if (EFI_ERROR(ret))
		return ret;

	blockio->Media = &media->m;

	return EFI_SUCCESS;
}

EFI_STATUS blockio_free(EFI_SYSTEM_TABLE *st, EFI_HANDLE handle)
{
	return interface_free(st, &blockio_guid, handle);
}
