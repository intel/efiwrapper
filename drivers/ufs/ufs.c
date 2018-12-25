/******************************************************************************
 *
 * INTEL CONFIDENTIAL
 *
 * Copyright (c) 2017 Intel Corporation All Rights Reserved.
 *
 * The source code contained or described herein and all documents related to
 * the source code (Material) are owned by Intel Corporation or its suppliers
 * or licensors. Title to the Material remains with Intel Corporation or its
 * suppliers and licensors. The Material contains trade secrets and proprietary
 * and confidential information of Intel or its suppliers and licensors. The
 * Material is protected by worldwide copyright and trade secret laws and
 * treaty provisions. No part of the Material may be used, copied, reproduced,
 * modified, published, uploaded, posted, transmitted, distributed, or
 * disclosed in any way without Intel's prior express written permission.
 *
 * No license under any patent, copyright, trade secret or other intellectual
 * property right is granted to or conferred upon you by disclosure or delivery
 * of the Materials, either expressly, by implication, inducement, estoppel or
 * otherwise. Any license under such intellectual property rights must be
 * express and approved by Intel in writing.
 *
 ******************************************************************************/

#include <hwconfig.h>
#include <storage.h>
#include <ewlib.h>

#include "UfsInternal.h"
#include "ufs.h"
#include "ScsiPassThruExt.h"
#include "UfsBlockIoLib.h"

const EFI_LBA UFS_BLOCK_MAX = 0xffff;

static struct supported_device {
	u16 vid;
	u16 did;
} SUPPORTED_DEVICES[] = {
	{ .vid = 0x8086, .did = UFS_PCI_DID },
};

static EFI_STATUS _init(storage_t *s)
{
	EFI_STATUS ret;
	pcidev_t pci_dev = 0;
	size_t i;
	DEVICE_BLOCK_INFO     BlockInfo;

	for (i = 0; i < ARRAY_SIZE(SUPPORTED_DEVICES); i++)
		if (pci_find_device(SUPPORTED_DEVICES[i].vid,
				SUPPORTED_DEVICES[i].did,
				&pci_dev))
			break;

	if (!pci_dev)
		return EFI_UNSUPPORTED;

	ret = InitializeUfs(pci_dev);
	if (ret)
		return EFI_DEVICE_ERROR;

	ret = UfsGetMediaInfo(DEVICE_INDEX_DEFAULT, &BlockInfo);
	if (EFI_ERROR(ret)) {
		DEBUG_UFS((EFI_D_VERBOSE, "MmcGetMediaInfo Error %d\n", ret));
		return ret;
	}

	DEBUG_UFS((EFI_D_VERBOSE, "Index 0 BlockNum is 0x%x\n", BlockInfo.BlockNum));
	DEBUG_UFS((EFI_D_VERBOSE, "BlockSize is 0x%x\n", BlockInfo.BlockSize));
	s->blk_cnt = BlockInfo.BlockNum;
	s->blk_sz = BlockInfo.BlockSize;

	return EFI_SUCCESS;
}

static EFI_LBA _read(storage_t *s, EFI_LBA start, EFI_LBA count, void *buf)
{
	EFI_STATUS ret;

	ret = UfsReadBlocks (
		DEVICE_INDEX_DEFAULT,
		start,
		s->blk_sz * count,
		buf);
	if (!EFI_ERROR(ret))
		return count;
	else
		return 0;
}

 /* Need to implement it in ufs driver*/
static EFI_LBA _write(storage_t *s, EFI_LBA start, EFI_LBA count, const void *buf)
{
	EFI_LBA cur_count, transfered = 0;
	EFI_STATUS ret;

	do {
		cur_count = count > UFS_BLOCK_MAX ? UFS_BLOCK_MAX : count;
		ret = UfsWriteBlocks (
			DEVICE_INDEX_DEFAULT,
			start,
			s->blk_sz * cur_count,
			(void *)buf);
		if (EFI_ERROR(ret))
			break;
		start += cur_count;
		buf += cur_count * s->blk_sz;
		count -= cur_count;
		transfered += cur_count;
	} while (count > 0);

	return transfered;
}

static storage_t storage_ufs_storage = {
	.init = _init,
	.read = _read,
	.write = _write,
	.erase = NULL,
	.pci_function = 0,
	.pci_device = 0,
};

static EFI_HANDLE handle;
//  static EFI_HANDLE handle_scsi_protocol;
static EFI_GUID ufs_ps_protocol_guid = EFI_EXT_SCSI_PASS_THRU_PROTOCOL_GUID;

static EFI_STATUS storage_ufs_init(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;
	UFS_PEIM_HC_PRIVATE_DATA *Private;
	boot_dev_t *boot_dev;
	static EFI_EXT_SCSI_PASS_THRU_MODE mode = {
		0xFFFFFFFF,
		EFI_EXT_SCSI_PASS_THRU_ATTRIBUTES_PHYSICAL | EFI_EXT_SCSI_PASS_THRU_ATTRIBUTES_LOGICAL,
		sizeof(UINTN)
		};
	EFI_EXT_SCSI_PASS_THRU_PROTOCOL ufs_default = {
		NULL,
		UfsPassThruPassThru,
		UfsPassThruGetNextTargetLun,
		UfsPassThruBuildDevicePath,
		UfsPassThruGetTargetLun,
		UfsPassThruResetChannel,
		UfsPassThruResetTargetLun,
		UfsPassThruGetNextTarget
		};
	ufs_default.Mode = &mode;

	if (!st)
		return EFI_INVALID_PARAMETER;

	boot_dev = get_boot_media();
	if (!boot_dev)
		return EFI_INVALID_PARAMETER;
	if (boot_dev->type != STORAGE_UFS)
		return EFI_SUCCESS;

	storage_ufs_storage.pci_device = (boot_dev->diskbus >> 8) & 0xff;
	storage_ufs_storage.pci_function = boot_dev->diskbus & 0xff;

	ret = storage_init(st, &storage_ufs_storage, &handle);
	if (EFI_ERROR(ret))
		return ret;

	Private = UfsGetPrivateData();
	if (Private == NULL)
		return EFI_NOT_FOUND;

	return interface_init(st, &ufs_ps_protocol_guid, &handle,
		&ufs_default, sizeof(ufs_default),
		(void **)&Private->ufs_ps_protocol);
}

static EFI_STATUS storage_ufs_exit(EFI_SYSTEM_TABLE *st)
{
	if (!st)
		return EFI_INVALID_PARAMETER;

	storage_free(st, handle);
	return interface_free(st, &ufs_ps_protocol_guid, handle);
}

ewdrv_t ufs_drv = {
	.name = "storage_ufs",
	.description = "STORAGE PCI UFS driver",
	.init = storage_ufs_init,
	.exit = storage_ufs_exit
};
