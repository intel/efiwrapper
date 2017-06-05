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

#include <kconfig.h>
#include <libpayload-config.h>
#include <libpayload.h>
#include <storage.h>
#include <sdio.h>

#include "sdhci_mmc/mmc.h"
#include "sdhci_mmc/sdhci-internal.h"
#include "sdhci_mmc/sdhci.h"
#include "sdhci_mmc/sdhci_mmc.h"

const EFI_LBA BLOCK_MAX = 0xffff;

static struct supported_device {
	u16 vid;
	u16 did;
} SUPPORTED_DEVICES[] ={
	{ .vid = 0x1b36, .did = 0x7 },
	{ .vid = 0x8086, .did = 0x5acc }
};

static EFI_STATUS _init(storage_t *s)
{
	int ret;
	pcidev_t pci_dev = 0;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(SUPPORTED_DEVICES); i++)
		if (pci_find_device(SUPPORTED_DEVICES[i].vid,
				    SUPPORTED_DEVICES[i].did,
				    &pci_dev))
			break;

	if (!pci_dev)
		return EFI_UNSUPPORTED;

	ret = mmc_init_card (pci_dev);
	if (ret)
		return EFI_DEVICE_ERROR;

	s->blk_cnt = mmc_read_count();
	s->blk_sz = DEFAULT_BLOCK_SIZE;

	return EFI_SUCCESS;
}

static EFI_STATUS transfer_data(bool read, EFI_LBA start, EFI_LBA count, void *buf)
{
	int ret;
	struct cmd send_cmd;
	struct cmd wait_cmd;

	memset(&send_cmd, 0, sizeof(send_cmd));
	send_cmd.args = start;
	send_cmd.nblock = count;
	send_cmd.addr = (uintptr_t)buf;
	send_cmd.resp_len = 32;
	send_cmd.flags = CMDF_DATA_XFER | CMDF_USE_DMA;

	send_cmd.flags |= read ? CMDF_RD_XFER : CMDF_WR_XFER;
	if (read)
		send_cmd.index = count > 1 ?
			CMD_READ_MULTIPLE_BLOCKS : CMD_READ_SINGLE_BLOCK;
	else
		send_cmd.index = count > 1 ?
			CMD_WRITE_MULTIPLE_BLOCKS : CMD_WRITE_SINGLE_BLOCK;

	ret = mmc_send_cmd(&send_cmd);
	if (ret)
		return EFI_DEVICE_ERROR;

	memset(&wait_cmd, 0, sizeof(wait_cmd));
	wait_cmd.flags = CMDF_DATA_XFER;
	ret = mmc_wait_cmd_done(&wait_cmd);

	return ret ? EFI_DEVICE_ERROR : EFI_SUCCESS;
}

static EFI_LBA split_and_transfer_data(storage_t *s, bool read, EFI_LBA start,
				       EFI_LBA count, void *buf)
{
	EFI_LBA cur_count, transfered = 0;
	EFI_STATUS ret;

	do {
		cur_count = count > BLOCK_MAX ? BLOCK_MAX : count;
		ret = transfer_data(read, start, cur_count, buf);
		if (EFI_ERROR(ret))
			break;
		start += cur_count;
		buf += cur_count * s->blk_sz;
		count -= cur_count;
		transfered += cur_count;
	} while (count > 0);

	return transfered;
}

static EFI_LBA _read(storage_t *s, EFI_LBA start, EFI_LBA count, void *buf)
{
	return split_and_transfer_data(s, true, start, count, buf);
}

static EFI_LBA _write(storage_t *s, EFI_LBA start, EFI_LBA count, const void *buf)
{
	return split_and_transfer_data(s, false, start, count, (void *)buf);
}

static storage_t sdhci_mmc_storage = {
	.init = _init,
	.read = _read,
	.write = _write,
	.pci_function = 0,
	.pci_device = 0,
};

static EFI_HANDLE handle;

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
	EFI_STATUS ret;
	struct cmd c;
	int mret;
	storage_t *storage;

	ret = sdio_get_storage(This, &storage);
	if (EFI_ERROR(ret))
		return ret;

	memset(&c, 0, sizeof(c));

	c.index = CommandIndex;

	if (Buffer) {
		c.addr = (uintptr_t)Buffer;
		c.nblock = BufferSize / storage->blk_sz;
	}

	if (DataType == InData)
		c.flags |= CMDF_DATA_XFER | CMDF_RD_XFER | CMDF_USE_DMA;
	else if (DataType == OutData)
		c.flags |= CMDF_DATA_XFER | CMDF_WR_XFER | CMDF_USE_DMA | CMDF_BUSY_CHECK;

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
		memcpy(ResponseData, &c.resp, sizeof(*ResponseData));

	return EFI_SUCCESS;
}

static EFI_STATUS sdhci_mmc_init(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;
	EFI_SD_HOST_IO_PROTOCOL *sdio;
	EFI_GUID sdio_guid = EFI_SD_HOST_IO_PROTOCOL_GUID;

	if (!st)
		return EFI_INVALID_PARAMETER;

	ret = storage_init(st, &sdhci_mmc_storage, &handle);
	if (EFI_ERROR(ret))
		return ret;

	ret = sdio_init(st, handle, &sdhci_mmc_storage);
	if (EFI_ERROR(ret))
		storage_free(st, handle);

	ret = uefi_call_wrapper(st->BootServices->HandleProtocol, 3,
				handle, &sdio_guid, (void **)&sdio);
	if (EFI_ERROR(ret)) {
		sdio_free(st, handle);
		storage_free(st, handle);
	}

	sdio->SendCommand = sdio_send_command;
	return ret;
}

static EFI_STATUS sdhci_mmc_exit(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;

	if (!st)
		return EFI_INVALID_PARAMETER;

	ret = sdio_free(st, handle);
	if (EFI_ERROR(ret))
		return ret;

	return storage_free(st, handle);
}

ewdrv_t sdhci_mmc_drv = {
	.name = "sdhci_mmc",
	.description = "SDHCI PCI eMMC driver",
	.init = sdhci_mmc_init,
	.exit = sdhci_mmc_exit
};
