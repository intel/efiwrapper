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

#include "diskio.h"
#include "interface.h"
#include "lib.h"
#include "efiprot.h"

typedef struct diskio {
	EFI_DISK_IO interface;
	media_t *media;
} diskio_t;

static EFI_STATUS read_block(media_t *media, EFI_LBA lba,
			     unsigned char **block)
{
	EFI_LBA count;

	*block = malloc(media->m.BlockSize);
	if (!*block)
		return EFI_OUT_OF_RESOURCES;

	count = media->storage->read(media->storage, lba, 1, *block);
	if (count != 1) {
		free(*block);
		return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
diskio_read(_EFI_DISK_IO *This,
	    UINT32 MediaId,
	    UINT64 Offset,
	    UINTN BufferSize,
	    VOID *Buffer)
{
	EFI_STATUS ret;
	diskio_t *diskio = (diskio_t *)This;
	UINT32 count, size;
	UINT32 blksz;
 	unsigned char *buf = Buffer, *block;
	media_t *media;

	if (!This || !Buffer)
		return EFI_INVALID_PARAMETER;

	if (!diskio->media)
		return EFI_INVALID_PARAMETER;
	media = diskio->media;

	if (media->m.MediaId != MediaId)
		return EFI_MEDIA_CHANGED;

	blksz = media->m.BlockSize;
	if (!blksz)
		return EFI_INVALID_PARAMETER;

	if (Offset % blksz) {
		ret = read_block(media, Offset / blksz, &block);
		if (EFI_ERROR(ret))
			return ret;

		size = min(blksz - (Offset % blksz), BufferSize);
		memcpy(buf, block + (Offset % blksz), size);//NOLINT
		free(block);

		buf += size;
		Offset += size;
		BufferSize -= size;
	}

	size = BufferSize / blksz;
	if (size > 0) {
		count = media->storage->read(media->storage, Offset / blksz, size, buf);
		if (count != size)
			return EFI_DEVICE_ERROR;
		size *= blksz;
		BufferSize -= size;
		Offset += size;
		buf += size;
	}
	if (BufferSize) {
		ret = read_block(media, Offset / blksz, &block);
		if (EFI_ERROR(ret))
			return ret;
		memcpy(buf, block, BufferSize);//NOLINT
		free(block);
	}

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
diskio_write(_EFI_DISK_IO *This,
	     UINT32 MediaId,
	     UINT64 Offset,
	     UINTN BufferSize,
	     VOID *Buffer)
{
	EFI_STATUS ret;
	diskio_t *diskio = (diskio_t *)This;
	UINT32 count, size;
	UINT32 blksz;
	unsigned char *buf = Buffer, *block;
	media_t *media;

	if (!This || !Buffer)
		return EFI_INVALID_PARAMETER;

	if (!diskio->media)
		return EFI_INVALID_PARAMETER;
	media = diskio->media;

	if (media->m.MediaId != MediaId)
		return EFI_MEDIA_CHANGED;

	blksz = media->m.BlockSize;
	if (!blksz)
		return EFI_INVALID_PARAMETER;

	if (Offset % blksz) {
		ret = read_block(media, Offset / blksz, &block);
		if (EFI_ERROR(ret))
			return ret;

		size = min(blksz - (Offset % blksz), BufferSize);
		memcpy(block + (Offset % blksz), buf, size);//NOLINT

		count = media->storage->write(media->storage, Offset / blksz, 1, block);
		free(block);
		if (count != 1)
			return EFI_DEVICE_ERROR;

		buf += size;
		Offset += size;
		BufferSize -= size;
	}

	size = BufferSize / blksz;
	if (size > 0) {
		count = media->storage->write(media->storage, Offset / blksz, size, buf);
		if (count != size)
			return EFI_DEVICE_ERROR;

		size *= blksz;
		BufferSize -= size;
		Offset += size;
		buf += size;
	}
	if (BufferSize) {
		ret = read_block(media, Offset / blksz, &block);
		if (EFI_ERROR(ret))
			return ret;

		memcpy(block, buf, BufferSize);//NOLINT
		count = media->storage->write(media->storage, Offset / blksz, 1, block);
		free(block);
		if (count != 1)
			return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}

static EFI_GUID diskio_guid = DISK_IO_PROTOCOL;

EFI_STATUS diskio_init(EFI_SYSTEM_TABLE *st, media_t *media,
		       EFI_HANDLE *handle)
{
	static diskio_t diskio_default = {
		.interface = {
			.Revision = EFI_DISK_IO_INTERFACE_REVISION,
			.ReadDisk = diskio_read,
			.WriteDisk = diskio_write
		}
	};
	EFI_STATUS ret;
	diskio_t *diskio;

	ret = interface_init(st, &diskio_guid, handle,
			     &diskio_default, sizeof(diskio_default),
			     (void **)&diskio);
	if (EFI_ERROR(ret))
		return ret;

	diskio->media = media;

	return EFI_SUCCESS;
}

EFI_STATUS diskio_free(EFI_SYSTEM_TABLE *st, EFI_HANDLE handle)
{
	return interface_free(st, &diskio_guid, handle);
}
