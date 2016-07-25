/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 *
 * Authors: Jérémy Compostella <jeremy.compostella@intel.com>
 *          Romain Vigier <romainx.vigier@intel.com>
 *          Gabriel Touzeau <gabrielx.touzeau@intel.com>
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

#include <stdio.h>
#include <protocol.h>
#include <pci.h>
#include <efiwrapper.h>

#include "protocol/SdHostIo.h"
#include "drivers/storage/sdhci.h"
#include "drivers/storage/mmc.h"
#include "drivers/storage/sdhci-bxt.h"
#include "blockdev.h"

static struct blockdev_ops *ops = NULL;

typedef struct media {
	EFI_BLOCK_IO_MEDIA m;
} media_t;

static EFIAPI EFI_STATUS
blockio_reset(__attribute__((__unused__)) EFI_BLOCK_IO *This,
	      __attribute__((__unused__)) BOOLEAN ExtendedVerification)
{
	return EFI_SUCCESS;
}

static EFI_STATUS access_blockio(EFI_BLOCK_IO *This, UINT32 MediaId, EFI_LBA LBA,
				 UINTN BufferSize, VOID *Buffer)
{
	media_t *media;
	UINT32 blksz;

	if (!This)
		return EFI_INVALID_PARAMETER;

	if (!This->Media)
		return EFI_NO_MEDIA;

	media = (media_t *)This->Media;
	if (MediaId != media->m.MediaId)
		return EFI_MEDIA_CHANGED;

	blksz = media->m.BlockSize;
	if (BufferSize % blksz)
		return EFI_BAD_BUFFER_SIZE;

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
blockio_write(EFI_BLOCK_IO *This, UINT32 MediaId, EFI_LBA LBA,
	      UINTN BufferSize, VOID *Buffer)
{
	EFI_STATUS ret;
	media_t *media;
	uint32_t count, size;

	ret = access_blockio(This, MediaId, LBA, BufferSize, Buffer);
	if (EFI_ERROR(ret))
		return ret;

	media = (media_t *)This->Media;
	size = BufferSize / media->m.BlockSize;

	count = ops->write(LBA, size, Buffer);
	if (count != size)
		return EFI_DEVICE_ERROR;

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
blockio_read(EFI_BLOCK_IO *This, UINT32 MediaId, EFI_LBA LBA,
	     UINTN BufferSize, VOID *Buffer)
{
	EFI_STATUS ret;
	media_t *media;
	uint32_t count, size;

	ret = access_blockio(This, MediaId, LBA, BufferSize, Buffer);
	if (EFI_ERROR(ret))
		return ret;

	media = (media_t *)This->Media;
	size = BufferSize / media->m.BlockSize;

	count = ops->read(LBA, size, Buffer);
	if (count != size)
		return EFI_DEVICE_ERROR;

	return EFI_SUCCESS;
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

typedef struct sdio {
	EFI_SD_HOST_IO_PROTOCOL interface;
	EFI_BLOCK_IO_MEDIA *media;
} sdio_t;

static EFI_STATUS _read_block(uint32_t lba, unsigned char **block, UINT32 blksz)
{
	uint32_t count;

	*block = ew_malloc(blksz);
	if (!*block)
		return EFI_OUT_OF_RESOURCES;

	count = ops->read(lba, 1, *block);
	if (count != 1) {
		ew_free(*block);
		return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}

#define min(a, b) (a < b ? a : b)

static EFIAPI EFI_STATUS
_disk_read(struct _EFI_DISK_IO *This,
	   UINT32 MediaId,
	   UINT64 Offset,
	   UINTN BufferSize,
	   VOID *Buffer)
{
	EFI_STATUS ret;
	diskio_t *diskio = (diskio_t *)This;
	uint32_t count, size;
	UINT32 blksz;
 	unsigned char *buf = Buffer, *block;

	if (!This || !Buffer)
		return EFI_INVALID_PARAMETER;

	if (diskio->media->MediaId != MediaId)
		return EFI_MEDIA_CHANGED;

	blksz = diskio->media->BlockSize;

	if (Offset % blksz) {
		ret = _read_block(Offset / blksz, &block, blksz);
		if (EFI_ERROR(ret))
			return ret;

		size = min(blksz - (Offset % blksz), BufferSize);
		ew_memcpy(buf, block + (Offset % blksz), size);
		ew_free(block);

		buf += size;
		Offset += size;
		BufferSize -= size;
		if (!BufferSize)
			return EFI_SUCCESS;
	}

	size = BufferSize / blksz;
	if (size > 0) {
		count = ops->read(Offset / blksz, size, buf);
		if (count != size)
			return EFI_DEVICE_ERROR;
		size *= blksz;
		BufferSize -= size;
		Offset += size;
		buf += size;
	}
	if (BufferSize) {
		ret = _read_block(Offset / blksz, &block, blksz);
		if (EFI_ERROR(ret))
			return ret;
		ew_memcpy(buf, block, BufferSize);
		ew_free(block);
	}

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
_disk_write(struct _EFI_DISK_IO *This,
	    UINT32 MediaId,
	    UINT64 Offset,
	    UINTN BufferSize,
	    VOID *Buffer)
{
	EFI_STATUS ret;
	diskio_t *diskio = (diskio_t *)This;
	uint32_t count, size;
	UINT32 blksz;
	unsigned char *buf = Buffer, *block;

	if (!This || !Buffer)
		return EFI_INVALID_PARAMETER;

	if (diskio->media->MediaId != MediaId)
		return EFI_MEDIA_CHANGED;

	blksz = diskio->media->BlockSize;

	if (Offset % blksz) {
		ret = _read_block(Offset / blksz, &block, blksz);
		if (EFI_ERROR(ret))
			return ret;

		size = min(blksz - (Offset % blksz), BufferSize);
		ew_memcpy(block + (Offset % blksz), buf, size);

		count = ops->write(Offset / blksz, 1, block);
		ew_free(block);
		if (count != 1)
			return EFI_DEVICE_ERROR;

		buf += size;
		Offset += size;
		BufferSize -= size;
	}

	size = BufferSize / blksz;
	if (size > 0) {
		count = ops->write(Offset / blksz, size, buf);
		if (count != size)
			return EFI_DEVICE_ERROR;

		size *= blksz;
		BufferSize -= size;
		Offset += size;
		buf += size;
	}
	if (BufferSize) {
		ret = _read_block(Offset / blksz, &block, blksz);
		if (EFI_ERROR(ret))
			return ret;

		ew_memcpy(block, buf, BufferSize);
		count = ops->write(Offset / blksz, 1, block);
		ew_free(block);
		if (count != 1)
			return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}

static EFI_DISK_IO diskio_struct = {
	.Revision = 0x1,
	.ReadDisk = _disk_read,
	.WriteDisk = _disk_write
};

static EFIAPI EFI_STATUS
sdio_send_command(EFI_SD_HOST_IO_PROTOCOL *This,
		  UINT16 CommandIndex,
		  UINT32 Argument,
		  TRANSFER_TYPE DataType,
		  UINT8 *Buffer,
		  UINT32 BufferSize,
		  __attribute__((__unused__)) RESPONSE_TYPE ResponseType,
		  __attribute__((__unused__)) UINT32 TimeOut,
		  UINT32 *ResponseData)
{
	struct cmd c;
	int mret;
	sdio_t *sdio = (sdio_t *)This;

	ew_memset(&c, 0, sizeof(c));

	c.index = CommandIndex;

	if (Buffer) {
		c.addr = (uintptr_t)Buffer;
		c.nblock = BufferSize / sdio->media->BlockSize;
	}

	if (DataType == InData)
		c.flags |= CMDF_DATA_XFER | CMDF_RD_XFER | CMDF_USE_DMA;

	c.resp_len = 32;
	c.args = Argument;
	c.retry = 5;

	mret = mmc_send_cmd(&c);
	if (mret != 0)
		return EFI_DEVICE_ERROR;

	mret = mmc_wait_cmd_done(&c);
	if (mret != 0)
		return EFI_DEVICE_ERROR;

	if (ResponseData)
		ew_memcpy(ResponseData, &c.resp, sizeof(*ResponseData));

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
sdio_set_clock_frequency(__attribute__((__unused__)) EFI_SD_HOST_IO_PROTOCOL *This,
			 __attribute__((__unused__)) UINT32 MaxFrequency)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
sdio_set_bus_width(__attribute__((__unused__)) EFI_SD_HOST_IO_PROTOCOL *This,
		   __attribute__((__unused__)) UINT32 BusWidth)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
sdio_set_host_voltage(__attribute__((__unused__)) EFI_SD_HOST_IO_PROTOCOL *This,
		      __attribute__((__unused__)) UINT32 Voltage)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
sdio_set_host_speed_mode(__attribute__((__unused__)) EFI_SD_HOST_IO_PROTOCOL *This,
			 __attribute__((__unused__)) UINT32 HighSpeed)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
sdio_set_host_ddr_mode(__attribute__((__unused__)) EFI_SD_HOST_IO_PROTOCOL *This,
		       __attribute__((__unused__)) UINT32 DdrMode)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
sdio_reset_sd_host(__attribute__((__unused__)) EFI_SD_HOST_IO_PROTOCOL *This,
		   __attribute__((__unused__)) RESET_TYPE ResetType)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
sdio_enable_auto_stop_cmd(__attribute__((__unused__)) EFI_SD_HOST_IO_PROTOCOL *This,
			  __attribute__((__unused__)) BOOLEAN Enable)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
sdio_detect_card_and_init_host(__attribute__((__unused__)) EFI_SD_HOST_IO_PROTOCOL *This)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
sdio_set_block_length(__attribute__((__unused__)) EFI_SD_HOST_IO_PROTOCOL *This,
		      __attribute__((__unused__)) UINT32 BlockLength)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
sdio_setup_device(__attribute__((__unused__)) EFI_SD_HOST_IO_PROTOCOL *This)
{
	return EFI_UNSUPPORTED;
}

EFI_SD_HOST_IO_PROTOCOL sdio_struct = {
	.Revision = 0x1,
	.HostCapability = {
		.HighSpeedSupport = 0,
		.V18Support = 0,
		.V30Support = 0,
		.V33Support = 0,
		.SDR50Support = 0,
		.SDR104Support = 0,
		.DDR50Support = 0,
		.Reserved0 = 0,
		.BusWidth4 = 0,
		.BusWidth8 = 0,
		.Reserved1 = 0,
		.SDMASupport = 0,
		.ADMA2Support = 0,
		.DmaMode = 0,
		.ReTuneTimer = 0,
		.ReTuneMode = 0,
		.Reserved2 = 0,
		.BoundarySize = 8
	},
	.SendCommand = sdio_send_command,
	.SetClockFrequency = sdio_set_clock_frequency,
	.SetBusWidth = sdio_set_bus_width,
	.SetHostVoltage = sdio_set_host_voltage,
	.SetHostDdrMode = sdio_set_host_ddr_mode,
	.ResetSdHost = sdio_reset_sd_host,
	.EnableAutoStopCmd = sdio_enable_auto_stop_cmd,
	.DetectCardAndInitHost =sdio_detect_card_and_init_host,
	.SetBlockLength = sdio_set_block_length,
	.SetupDevice = sdio_setup_device,
	.SetHostSpeedMode = sdio_set_host_speed_mode
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

static EFI_STATUS mmc_sdio(EFI_HANDLE handle, media_t *media)
{
	EFI_STATUS ret;
	sdio_t *sdio;
	EFI_GUID guid = EFI_SD_HOST_IO_PROTOCOL_GUID;

	sdio = ew_malloc(sizeof(*sdio));
	if (!sdio)
		return EFI_OUT_OF_RESOURCES;

	ew_memcpy(sdio, &sdio_struct, sizeof(*sdio));
	sdio->media = &media->m;
	ret = ew_install_protocol_interface(&handle, &guid,
					    EFI_NATIVE_INTERFACE, sdio);
	if (EFI_ERROR(ret))
		ew_free(sdio);

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

static EFI_STATUS unregister_sdio(EFI_HANDLE handle)
{
	EFI_STATUS ret;
	EFI_SD_HOST_IO_PROTOCOL *sdio;
	EFI_GUID guid = EFI_SD_HOST_IO_PROTOCOL_GUID;

	if (!handle)
		return EFI_INVALID_PARAMETER;

	ret = ew_handle_protocol(handle, &guid, (VOID **)&sdio);
	if (EFI_ERROR(ret))
		return ret;

	ret = ew_uninstall_protocol_interface(handle, &guid, sdio);
	if (EFI_ERROR(ret))
		return ret;

	ew_free(sdio);
	return EFI_SUCCESS;
}
static struct supported_device {
	u16 vid;
	u16 did;
} SUPPORTED_DEVICES[] ={
	{ .vid = 0x1b36, .did = 0x7 },
	{ .vid = 0x8086, .did = 0x5acc }
};

EFI_STATUS payload_register_mmc(const char *path)
{
	EFI_STATUS ret;
	media_t *media = NULL;

	pcidev_t pci_dev = 0;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(SUPPORTED_DEVICES); i++)
		if (pci_find_device(SUPPORTED_DEVICES[i].vid,
				    SUPPORTED_DEVICES[i].did,
				    &pci_dev))
			break;

	if (!pci_dev)
		return EFI_UNSUPPORTED;

	ret = mmc_init_card (pci_dev, false);
	if (EFI_ERROR(ret))
		return ret;

	ops = get_block_ops();
	if (!ops)
		return EFI_INVALID_PARAMETER;

	if (!path)
		return EFI_INVALID_PARAMETER;

	if (mmc_handle)
		return EFI_ALREADY_STARTED;

	ret = mmc_media(&media, path, ops->block_size, ops->block_count);
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

	ret = mmc_sdio(mmc_handle, media);
	if (EFI_ERROR(ret))
		goto err;

	return EFI_SUCCESS;

err:
	if (mmc_handle) {
		unregister_blockio(mmc_handle, &media);
		unregister_device_path(mmc_handle);
		unregister_diskio(mmc_handle);
		unregister_sdio(mmc_handle);
		mmc_handle = NULL;
	}

	if (media)
		ew_free(media);

	return ret;
}

EFI_STATUS payload_unregister_mmc()
{
	EFI_STATUS ret;
	media_t *media;

	if (!mmc_handle)
		return EFI_SUCCESS;

	ret = unregister_blockio(mmc_handle, &media);
	if (EFI_ERROR(ret))
		return ret;

	if (media)
		ew_free(media);

	ret = unregister_device_path(mmc_handle);
	if (EFI_ERROR(ret))
		return ret;

	ret = unregister_diskio(mmc_handle);
	if (EFI_ERROR(ret))
		return ret;

	ret = unregister_sdio(mmc_handle);
	if (EFI_ERROR(ret))
		return ret;

	mmc_handle = NULL;
	return ret;
}
