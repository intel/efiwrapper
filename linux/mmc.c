/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 *
 * Authors: Jérémy Compostella <jeremy.compostella@intel.com>
 *          Romain Vigier <romainx.vigier@intel.com>
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

#define _LARGEFILE64_SOURCE
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <errno.h>

#include <protocol.h>
#include <efiwrapper.h>

#include "common.h"
#include "mmc.h"

typedef struct media {
	EFI_BLOCK_IO_MEDIA m;
	char *path;
} media_t;

static EFIAPI EFI_STATUS
blockio_reset(__attribute__((__unused__)) EFI_BLOCK_IO *This,
	      __attribute__((__unused__)) BOOLEAN ExtendedVerification)
{
	return EFI_SUCCESS;
}

static EFI_STATUS open_media(media_t *media, UINT64 offset, int *fd)
{
	UINT64 offset_r;
	bool newfile = false;
	struct stat sb;
	int ret;

	ret = stat(media->path, &sb);
	if (ret == -1 && errno == ENOENT)
		newfile = true;

	*fd = open(media->path, O_RDWR | O_CREAT, 00755);
	if (*fd == -1)
		return EFI_DEVICE_ERROR;

	if (newfile) {
		/* Ensure the disk size is correct. */
		lseek(*fd, media->m.BlockSize * (media->m.LastBlock + 1) - 1,
		      SEEK_SET);
		char c = 0;
		write(*fd, &c, 1);
	}

	offset_r = lseek(*fd, offset, SEEK_SET);
	if (offset_r != offset) {
		close(*fd);
		return EFI_INVALID_PARAMETER;
	}

	return EFI_SUCCESS;
}

static EFI_STATUS access_blockio(EFI_BLOCK_IO *This, UINT32 MediaId, EFI_LBA LBA,
				 UINTN BufferSize, VOID *Buffer, int *fd)
{
	EFI_STATUS ret;
	media_t *media;
	UINT32 block_size;

	if (!This)
		return EFI_INVALID_PARAMETER;

	if (!This->Media)
		return EFI_NO_MEDIA;

	media = (media_t *)This->Media;
	if (MediaId != media->m.MediaId)
		return EFI_MEDIA_CHANGED;

	block_size = media->m.BlockSize;
	if (BufferSize % block_size)
		return EFI_BAD_BUFFER_SIZE;

	ret = open_media(media, LBA * block_size, fd);
	if (EFI_ERROR(ret))
		return ret;

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
blockio_write(EFI_BLOCK_IO *This, UINT32 MediaId, EFI_LBA LBA,
	      UINTN BufferSize, VOID *Buffer)
{
	EFI_STATUS ret;
	int fd;

	ret = access_blockio(This, MediaId, LBA, BufferSize, Buffer, &fd);
	if (EFI_ERROR(ret))
		return ret;

	ret = write_fd(fd, Buffer, BufferSize);
	close(fd);
	return ret;
}

static EFIAPI EFI_STATUS
blockio_read(EFI_BLOCK_IO *This, UINT32 MediaId, EFI_LBA LBA,
	     UINTN BufferSize, VOID *Buffer)
{
	EFI_STATUS ret;
	int fd;

	ret = access_blockio(This, MediaId, LBA, BufferSize, Buffer, &fd);
	if (EFI_ERROR(ret))
		return ret;

	ret = read_fd(fd, Buffer, BufferSize);
	close(fd);
	return ret;
}

static EFIAPI EFI_STATUS
blockio_flush(EFI_BLOCK_IO *This)
{
	return EFI_SUCCESS;
}

static EFI_BLOCK_IO blockio_struct = {
	.Revision = 0x1,
	.Reset = blockio_reset,
	.ReadBlocks = blockio_read,
	.WriteBlocks = blockio_write,
	.FlushBlocks = blockio_flush
};

typedef struct diskio {
	EFI_BLOCK_IO interface;
	EFI_BLOCK_IO_MEDIA *media;
} diskio_t;

static EFIAPI EFI_STATUS
_disk_read(struct _EFI_DISK_IO *This,
	   UINT32 MediaId,
	   UINT64 Offset,
	   UINTN BufferSize,
	   VOID *Buffer)
{
	EFI_STATUS ret;
	int fd;
	diskio_t *diskio = (diskio_t *)This;

	if (!This || !Buffer)
		return EFI_INVALID_PARAMETER;

	if (diskio->media->MediaId != MediaId)
		return EFI_MEDIA_CHANGED;

	ret = open_media((media_t *)diskio->media, Offset, &fd);
	if (EFI_ERROR(ret))
		return ret;

	ret = read(fd, Buffer, BufferSize);
	close(fd);
	return ret;
}

static EFIAPI EFI_STATUS
_disk_write(struct _EFI_DISK_IO *This,
	    UINT32 MediaId,
	    UINT64 Offset,
	    UINTN BufferSize,
	    VOID *Buffer)
{
	EFI_STATUS ret;
	int fd;
	diskio_t *diskio = (diskio_t *)This;

	if (!This || !Buffer)
		return EFI_INVALID_PARAMETER;

	if (diskio->media->MediaId != MediaId)
		return EFI_MEDIA_CHANGED;

	ret = open_media((media_t *)diskio->media, Offset, &fd);
	if (EFI_ERROR(ret))
		return ret;

	ret = write_fd(fd, Buffer, BufferSize);
	close(fd);
	return ret;
}

static EFI_DISK_IO diskio_struct = {
	.Revision = 0x1,
	.ReadDisk = _disk_read,
	.WriteDisk = _disk_write
};

static UINT32 media_id;
static EFI_HANDLE mmc_handle;

static EFI_STATUS mmc_media(media_t **media_p, const char *path, UINTN blk_size, UINTN lba)
{
	media_t *media;

	media = ew_calloc(1, sizeof(*media));
	if (!media)
		return EFI_OUT_OF_RESOURCES;

	media->m.MediaId = ++media_id;
	media->m.RemovableMedia = FALSE;
	media->m.MediaPresent = TRUE;
	media->m.LogicalPartition = FALSE;
	media->m.ReadOnly = FALSE;
	media->m.WriteCaching = 0x0;
	media->m.BlockSize = blk_size;
	media->m.IoAlign = 0x0;
	media->m.LastBlock = lba - 1;
	media->m.LowestAlignedLba = 0x0;
	media->m.LogicalBlocksPerPhysicalBlock = 0x0;
	media->m.OptimalTransferLengthGranularity = 0x0;
	media->path = ew_strdup(path);
	if (!media->path) {
		ew_free(media);
		return EFI_OUT_OF_RESOURCES;
	}

	*media_p = media;

	return EFI_SUCCESS;
}

static EFI_STATUS mmc_blockio(EFI_HANDLE *handle, media_t *media)
{
	EFI_STATUS ret;
	EFI_BLOCK_IO *blockio;

	blockio = ew_calloc(1, sizeof(*blockio));
	if (!blockio)
		return EFI_OUT_OF_RESOURCES;

	ew_memcpy(blockio, &blockio_struct, sizeof(*blockio));
	blockio->Media = &media->m;
	ret = ew_install_protocol_interface(handle, &BlockIoProtocol,
					    EFI_NATIVE_INTERFACE, blockio);
	if (EFI_ERROR(ret))
		ew_free(blockio);

	return ret;
}

static EFI_STATUS mmc_device_path(EFI_HANDLE handle)
{
	EFI_STATUS ret;
	PCI_DEVICE_PATH pci_dp;
	CONTROLLER_DEVICE_PATH ctrl_dp;
	EFI_DEVICE_PATH end_dp;
	EFI_DEVICE_PATH *dp;
	char *tmp;

	dp = ew_calloc(1, sizeof(pci_dp) + sizeof(ctrl_dp) + sizeof(end_dp));
	if (!dp)
		return EFI_OUT_OF_RESOURCES;

	tmp = (char *)dp;

	pci_dp.Header.Type = HARDWARE_DEVICE_PATH;
	pci_dp.Header.SubType = HW_PCI_DP;
	SetDevicePathNodeLength(&pci_dp.Header, sizeof(pci_dp));
	ew_memcpy(tmp, &pci_dp, sizeof(pci_dp));
	tmp += sizeof(pci_dp);

	ctrl_dp.Header.Type = HARDWARE_DEVICE_PATH;
	ctrl_dp.Header.SubType = HW_CONTROLLER_DP;
	ctrl_dp.Controller = 0;
	SetDevicePathNodeLength(&ctrl_dp.Header, sizeof(ctrl_dp));
	ew_memcpy(tmp, &ctrl_dp, sizeof(ctrl_dp));
	tmp += sizeof(ctrl_dp);

	end_dp.Type = END_DEVICE_PATH_TYPE;
	end_dp.SubType = END_ENTIRE_DEVICE_PATH_SUBTYPE;
	SetDevicePathNodeLength(&end_dp, sizeof(end_dp));
	ew_memcpy(tmp, &end_dp, sizeof(end_dp));

	ret = ew_install_protocol_interface(&handle, &DevicePathProtocol,
					    EFI_NATIVE_INTERFACE, dp);
	if (EFI_ERROR(ret))
		ew_free(dp);

	return ret;
}

static EFI_STATUS mmc_diskio(EFI_HANDLE handle, media_t *media)
{
	EFI_STATUS ret;
	diskio_t *diskio;

	diskio = ew_malloc(sizeof(*diskio));
	if (!diskio)
		return EFI_OUT_OF_RESOURCES;

	ew_memcpy(diskio, &diskio_struct, sizeof(*diskio));
	diskio->media = &media->m;
	ret = ew_install_protocol_interface(&handle, &DiskIoProtocol,
					    EFI_NATIVE_INTERFACE, diskio);
	if (EFI_ERROR(ret))
		ew_free(diskio);

	return ret;
}

static EFI_STATUS unregister_blockio(EFI_HANDLE handle, media_t **media)
{
	EFI_STATUS ret;
	EFI_BLOCK_IO *blockio;

	if (!handle)
		return EFI_INVALID_PARAMETER;

	ret = ew_handle_protocol(handle, &BlockIoProtocol, (VOID **)&blockio);
	if (EFI_ERROR(ret))
		return ret;

	*media = (media_t *)blockio->Media;

	ret = ew_uninstall_protocol_interface(handle, &BlockIoProtocol, blockio);
	if (EFI_ERROR(ret))
		return ret;

	ew_free(blockio);
	return EFI_SUCCESS;
}

static EFI_STATUS unregister_device_path(EFI_HANDLE handle)
{
	EFI_STATUS ret;
	EFI_DEVICE_PATH *dp;

	if (!handle)
		return EFI_INVALID_PARAMETER;

	ret = ew_handle_protocol(handle, &DevicePathProtocol, (VOID **)&dp);
	if (EFI_ERROR(ret))
		return ret;

	ret = ew_uninstall_protocol_interface(handle, &DevicePathProtocol, dp);
	if (EFI_ERROR(ret))
		return ret;

	ew_free(dp);
	return EFI_SUCCESS;
}

static EFI_STATUS unregister_diskio(EFI_HANDLE handle)
{
	EFI_STATUS ret;
	EFI_BLOCK_IO *diskio;

	if (!handle)
		return EFI_INVALID_PARAMETER;

	ret = ew_handle_protocol(handle, &DiskIoProtocol, (VOID **)&diskio);
	if (EFI_ERROR(ret))
		return ret;

	ret = ew_uninstall_protocol_interface(handle, &DiskIoProtocol, diskio);
	if (EFI_ERROR(ret))
		return ret;

	ew_free(diskio);
	return EFI_SUCCESS;
}

EFI_STATUS linux_register_mmc(const char *path, UINTN blk_size, UINTN lba)
{
	EFI_STATUS ret;
	media_t *media = NULL;

	if (!path)
		return EFI_INVALID_PARAMETER;

	if (mmc_handle)
		return EFI_ALREADY_STARTED;

	ret = mmc_media(&media, path, blk_size, lba);
	if (EFI_ERROR(ret))
		return ret;

	ret = mmc_blockio(&mmc_handle, media);
	if (EFI_ERROR(ret))
		goto err;

	ret = mmc_device_path(mmc_handle);
	if (EFI_ERROR(ret))
		goto err;

	ret = mmc_diskio(mmc_handle, media);
	if (EFI_ERROR(ret))
		goto err;

	return EFI_SUCCESS;

err:
	if (mmc_handle) {
		unregister_blockio(mmc_handle, &media);
		unregister_device_path(mmc_handle);
		unregister_diskio(mmc_handle);
		mmc_handle = NULL;
	}

	if (media) {
		ew_free(media->path);
		ew_free(media);
	}

	return ret;
}

EFI_STATUS linux_unregister_mcc()
{
	EFI_STATUS ret;
	media_t *media;

	if (!mmc_handle)
		return EFI_SUCCESS;

	ret = unregister_blockio(mmc_handle, &media);
	if (EFI_ERROR(ret))
		return ret;

	if (media) {
		ew_free(media->path);
		ew_free(media);
	}

	ret = unregister_device_path(mmc_handle);
	if (EFI_ERROR(ret))
		return ret;

	ret = unregister_diskio(mmc_handle);
	if (EFI_ERROR(ret))
		return ret;

	mmc_handle = NULL;
	return ret;
}
