/** @file

  Copyright (c) 2018, Intel Corporation. All rights reserved.<BR>
  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/


#include <kconfig.h>
#include <interface.h>
#include <libpayload-config.h>
#include <libpayload.h>
#include <ewlib.h>
#include <ewlog.h>
#include <storage.h>

#include "VirtioDeviceCommon.h"
#include "virtual_media.h"
#include "VirtioBlkAccessLib.h"
#include "VirtioRpmbAccessLib.h"
#include "ScsiPassThruExt.h"
#include "VirtioRpmbDevice.h"

#define DEVICE_INDEX_DEFAULT  0

static struct supported_device {
	u16 vid;
	u16 did;
} SUPPORTED_DEVICES[] = {
	{ .vid = 0x1AF4, .did = 0x1001},
	{ .vid = 0x8086, .did = 0x8601},
};

static EFI_STATUS _init(storage_t *s)
{
	EFI_STATUS ret;
	pcidev_t pci_dev[2] = {0};
	size_t i;
	DEVICE_BLOCK_INFO     BlockInfo ={0};

	for (i = 0; i < ARRAY_SIZE(SUPPORTED_DEVICES); i++)
		pci_find_device(SUPPORTED_DEVICES[i].vid,
				SUPPORTED_DEVICES[i].did,
				&pci_dev[i]);

	if (!pci_dev[0])
		return EFI_UNSUPPORTED;

	ret = VirtioMediaInitialize((UINTN)pci_dev[0]);
	if (ret)
		return EFI_DEVICE_ERROR;

	if (pci_dev[1])
		ret = VirtioRpmbInitialize((UINTN)pci_dev[1]);

	ret = VirtioGetMediaInfo(DEVICE_INDEX_DEFAULT, &BlockInfo);
	if (EFI_ERROR(ret)) {
		ewerr ("VirtioGetMediaInfo Error %x\n",(UINT32)ret);
		return ret;
	}

	ewdbg("BlockNum is 0x%x", (UINT32)BlockInfo.BlockNum);
	ewdbg("BlockSize is 0x%x", BlockInfo.BlockSize);
	s->blk_cnt = BlockInfo.BlockNum;
	s->blk_sz = BlockInfo.BlockSize;

	return EFI_SUCCESS;
}

static EFI_LBA _read(storage_t *s, EFI_LBA start, EFI_LBA count, void *buf)
{
	EFI_STATUS ret;

	ret = VirtioReadBlocks (
		DEVICE_INDEX_DEFAULT,
		start,
		s->blk_sz * count,
		buf);
	if (!EFI_ERROR(ret))
		return count;
	else
		return 0;
}

static EFI_LBA _write(storage_t *s, EFI_LBA start, EFI_LBA count, const void *buf)
{
	EFI_STATUS ret;

	ret = VirtioWriteBlocks (
		DEVICE_INDEX_DEFAULT,
		start,
		s->blk_sz * count,
		(void *)buf);
	if (!EFI_ERROR(ret))
		return count;
	else
		return 0;
}

static storage_t storage_virtual_media = {
	.init = _init,
	.read = _read,
	.write = _write,
	.pci_function = 0,
	.pci_device = 0,
};

static EFI_HANDLE handle;
static EFI_HANDLE vrpmb_handle;
static EFI_GUID passthru_guid = EFI_EXT_SCSI_PASS_THRU_PROTOCOL_GUID;

static EFI_STATUS storage_virtual_media_init(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;
	boot_dev_t *boot_dev;
	VRPMB_DEV *Rpmbdev_context = NULL;
	EFI_EXT_SCSI_PASS_THRU_PROTOCOL *PassThru;

	static EFI_EXT_SCSI_PASS_THRU_PROTOCOL vrpmb_default = {
		NULL,
		VrpmbPassThru,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
		};

	if (!st)
		return EFI_INVALID_PARAMETER;

	boot_dev = get_boot_media();
	if (!boot_dev)
		return EFI_INVALID_PARAMETER;

	boot_dev->type = STORAGE_VIRTUAL;

	storage_virtual_media.pci_device = (boot_dev->diskbus >> 8) & 0xff;
	storage_virtual_media.pci_function = boot_dev->diskbus & 0xff;

	ret = storage_init(st, &storage_virtual_media, &handle);

	if (EFI_ERROR(ret))
		return ret;

	Rpmbdev_context = VirtioRPMBGetContext();
	if (Rpmbdev_context)
		ret = interface_init(st, &passthru_guid, &vrpmb_handle,
			      &vrpmb_default, sizeof(vrpmb_default),
			      (void **)&PassThru);
	else
		ewdbg("Virtio RPMB device not found or initialization failed!\n");

	return ret;

}

static EFI_STATUS storage_virtual_media_exit(EFI_SYSTEM_TABLE *st)
{
	if (!st)
		return EFI_INVALID_PARAMETER;

	return 	storage_free(st, handle);
}

ewdrv_t virtual_media_drv = {
	.name = "storage_virtual_media",
	.description = "STORAGE virtual media driver",
	.init = storage_virtual_media_init,
	.exit = storage_virtual_media_exit
};
