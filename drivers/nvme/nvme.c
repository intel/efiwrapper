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
#include <ewdrv.h>
#include <pci.h>

#include "NvmExpress.h"
#include "NvmCtrlLib.h"

#define DEVICE_INDEX_DEFAULT  0
#define MAX_PCI_BUS_NUM 8

static struct supported_device {
	u16 vid;
	u16 did;
} SUPPORTED_DEVICES[] = {
	{ .vid = 0x144d, .did = NVME_PCI_DID },
	{ .vid = 0x8086, .did = 0xF1A6 },
};

static EFI_STATUS _init(storage_t *s)
{
	DEVICE_BLOCK_INFO	  BlockInfo;
	EFI_STATUS ret;
	pcidev_t pci_dev = 0;
	size_t i,j;

	pci_dev = get_diskbus();
	DEBUG_NVME ((EFI_D_INFO, "pci dev = 0x%X\n", pci_dev));
	if (pci_dev == 0){
		for (i = 0; i < ARRAY_SIZE(SUPPORTED_DEVICES); i++)
			for (j = 0; j< MAX_PCI_BUS_NUM; j++)
				if (pci_find_device_by_bus(j,SUPPORTED_DEVICES[i].vid,
							SUPPORTED_DEVICES[i].did,
							&pci_dev))
					break;
	}

	pci_dev = 0x80000000 | pci_dev;

	DEBUG_NVME ((EFI_D_INFO, "pci_dev = 0x%X\n", pci_dev));
	if (!pci_dev)
		return EFI_UNSUPPORTED;

	ret = NvmeInitialize(pci_dev);
	if (ret)
		DEBUG_NVME ((EFI_D_INFO, "NvmeInitialize ret = 0x%X\n", ret));

	if (ret)
		return EFI_DEVICE_ERROR;

	ret = NvmeGetMediaInfo(DEVICE_INDEX_DEFAULT, &BlockInfo);
	if (EFI_ERROR(ret)) {
		DEBUG_NVME ((EFI_D_ERROR, "MmcGetMediaInfo Error %d\n", ret));
		return ret;
	}

	DEBUG_NVME ((EFI_D_INFO, "Index 0 BlockNum is 0x%x\n", BlockInfo.BlockNum));
	DEBUG_NVME ((EFI_D_INFO, "BlockSize is 0x%x\n", BlockInfo.BlockSize));
	s->blk_cnt = BlockInfo.BlockNum;
	s->blk_sz = BlockInfo.BlockSize;

	return EFI_SUCCESS;
}

static EFI_LBA _read(storage_t *s, EFI_LBA start, EFI_LBA count, void *buf)
{
	EFI_STATUS ret;

	ret = NvmeReadBlocks (
		DEVICE_INDEX_DEFAULT,
		start,
		s->blk_sz * count,
		buf);
	if (!EFI_ERROR(ret))
		return count;

	return 0;
}

static EFI_LBA _write(storage_t *s,
	EFI_LBA start,
	EFI_LBA count,
	const void *buf)
{
	EFI_STATUS ret;

	ret = NvmeWriteBlocks (DEVICE_INDEX_DEFAULT, start,  s->blk_sz * count, (void *)buf);
	if (!EFI_ERROR(ret))
		return count;

	DEBUG_NVME ((EFI_D_INFO, "nvme_write	Error =0x%lx\n", ret));
	return 0;
}


static storage_t nvme_storage = {
	.init = _init,
	.read = _read,
	.write = _write,
	.erase = NULL,
	.pci_function = 0,
	.pci_device = 0,
};


static EFI_HANDLE nvme_handle;
static EFI_GUID nvme_pass_thru_guid = EFI_NVM_EXPRESS_PASS_THRU_PROTOCOL_GUID;
static EFI_GUID nvme_security_guid = EFI_STORAGE_SECURITY_COMMAND_PROTOCOL_GUID;

static EFI_STATUS nvme_drv_init(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;
	boot_dev_t *boot_dev;
	EFI_STORAGE_SECURITY_COMMAND_PROTOCOL *StorageSecurity;
	EFI_NVM_EXPRESS_PASS_THRU_PROTOCOL *nvme_passthru;

	boot_dev = get_boot_media();
	if (!boot_dev)
		return EFI_INVALID_PARAMETER;

	if (boot_dev->type != STORAGE_NVME)
		return EFI_SUCCESS;

	nvme_storage.pci_device = (NVME_DISKBUS >> 8) & 0xff;
	nvme_storage.pci_function = NVME_DISKBUS & 0xff;
	ret = storage_init(st, &nvme_storage, &nvme_handle);
	if (EFI_ERROR(ret))
		return ret;

	StorageSecurity = NvmeGetSecurityInterface();
	if (StorageSecurity != NULL) {
		ret = uefi_call_wrapper(st->BootServices->InstallProtocolInterface,
			4,
			&nvme_handle,
			&nvme_security_guid,
			EFI_NATIVE_INTERFACE,
			StorageSecurity
			);
	}

	nvme_passthru = NvmeGetPassthru();
	return uefi_call_wrapper(st->BootServices->InstallProtocolInterface, 4,
			&nvme_handle, &nvme_pass_thru_guid, EFI_NATIVE_INTERFACE, nvme_passthru);
}

static EFI_STATUS nvme_drv_exit(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;
	VOID *nvme_interface;

	if (!st)
		return EFI_INVALID_PARAMETER;

	storage_free(st, nvme_handle);

	ret = uefi_call_wrapper(st->BootServices->HandleProtocol, 3,
			nvme_handle, &nvme_pass_thru_guid, (VOID **)&nvme_interface);
	if (EFI_ERROR(ret))
		return ret;

	ret = uefi_call_wrapper(st->BootServices->UninstallProtocolInterface, 3,
			nvme_handle, &nvme_pass_thru_guid, nvme_interface);

	ret = uefi_call_wrapper(st->BootServices->HandleProtocol, 3,
			nvme_handle, &nvme_security_guid, (VOID **)&nvme_interface);
	if (EFI_ERROR(ret))
		return ret;

	ret = uefi_call_wrapper(st->BootServices->UninstallProtocolInterface, 3,
			nvme_handle, &nvme_security_guid, nvme_interface);

	return ret;
}

ewdrv_t nvme_drv = {
	.name = "nvme",
	.description = "PCI NVME driver",
	.init = nvme_drv_init,
	.exit = nvme_drv_exit
};

