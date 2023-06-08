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
#include "diskio.h"
#include "eraseblk.h"
#include "external.h"
#include "lib.h"
#include "media.h"
#include "interface.h"
#include "ewlog.h"
#include "ewarg.h"

#include <efilib.h>
#include <storage.h>

#ifndef MSG_UFS_DP
#define MSG_UFS_DP	0x19
#endif

#ifndef MSG_NVME_DP
#define MSG_NVME_DP	0x17
#endif

#ifndef MSG_EMMC_DP
#define MSG_EMMC_DP	29
#endif

#ifndef MSG_VIRTUAL_MEDIA_DP
#define MSG_VIRTUAL_MEDIA_DP	0x20
#endif

static EFI_GUID dp_guid = DEVICE_PATH_PROTOCOL;

#define ABL_BDEV "ABL.bdev"
#define ABL_DISKBUS "ABL.diskbus"
#define ABL_BDEVLIST "ABL.bootdevices"

static boot_dev_t boot_dev = {
	.type = STORAGE_NVME,
	.diskbus = 0x000
};

/* Device path  */
struct storage_dp {
	PCI_DEVICE_PATH pci;
	CONTROLLER_DEVICE_PATH ctrl;
	SCSI_DEVICE_PATH msg_device_path;
	EFI_DEVICE_PATH end;
} __attribute__((__packed__));

typedef struct {
	// Boot medium type, Refer OS_BOOT_MEDIUM_TYPE
	UINT8	DevType;
	//Zero-based hardware partition number
	UINT8	HwPart;
	//For PCI device, its value is 0x00BBDDFF
	//For other device, its value is MMIO address.
	UINT32	DevAddr;
} __attribute__((__packed__)) OS_BOOT_DEVICE;

typedef struct {
	UINT8		Revision;
	UINT8		BootDeviceCount;
	OS_BOOT_DEVICE	BootDevice[0];
}__attribute__((__packed__))  OS_BOOT_DEVICE_LIST;

static EFI_STATUS dp_init(EFI_SYSTEM_TABLE *st, media_t *media,
			  EFI_HANDLE *handle)
{
	EFI_STATUS ret;
	struct storage_dp *dp;

	dp = calloc(1, sizeof(struct storage_dp));
	if (!dp)
		return EFI_OUT_OF_RESOURCES;

	dp->pci.Header.Type = HARDWARE_DEVICE_PATH;
	dp->pci.Header.SubType = HW_PCI_DP;
	dp->pci.Function = media->storage->pci_function;
	dp->pci.Device = media->storage->pci_device;
	SetDevicePathNodeLength(&dp->pci.Header, sizeof(dp->pci));

	dp->ctrl.Header.Type = HARDWARE_DEVICE_PATH;
	dp->ctrl.Header.SubType = HW_CONTROLLER_DP;
	SetDevicePathNodeLength(&dp->ctrl.Header, sizeof(dp->ctrl));

	dp->msg_device_path.Header.SubType = get_boot_media_device_path_type();
	dp->msg_device_path.Header.Type = MESSAGING_DEVICE_PATH;
	dp->msg_device_path.Pun = 0;
	dp->msg_device_path.Lun = 0;
	SetDevicePathNodeLength(&dp->msg_device_path.Header, sizeof(dp->msg_device_path));

	dp->end.Type = END_DEVICE_PATH_TYPE;
	dp->end.SubType = END_ENTIRE_DEVICE_PATH_SUBTYPE;
	SetDevicePathNodeLength(&dp->end, sizeof(dp->end));

	ret = uefi_call_wrapper(st->BootServices->InstallProtocolInterface, 4,
				handle, &dp_guid,
				EFI_NATIVE_INTERFACE, dp);
	if (EFI_ERROR(ret))
		free(dp);

	return ret;
}

EFI_STATUS dp_free(EFI_SYSTEM_TABLE *st, EFI_HANDLE handle)
{
	return interface_free(st, &dp_guid, handle);
}

static struct storage_interface {
	const char *name;
	EFI_STATUS (*init)(EFI_SYSTEM_TABLE *, media_t *, EFI_HANDLE *);
	EFI_STATUS (*free)(EFI_SYSTEM_TABLE *, EFI_HANDLE);
} STORAGE_INTERFACES[] = {
	{ "media", media_register, media_free },
	{ "device path", dp_init, dp_free },
	{ "blockio", blockio_init, blockio_free },
	{ "diskio", diskio_init, diskio_free },
	{ "eraseblock", erase_block_init, erase_block_free }
};

EFI_STATUS storage_init(EFI_SYSTEM_TABLE *st, storage_t *storage,
			EFI_HANDLE *handle)
{
	EFI_STATUS ret, tmp_ret;
	int res;
	size_t i, j;
	media_t *media;

	ewdbg("storage_init called");
	if (!st || !storage || !handle)
		return EFI_INVALID_PARAMETER;

	if (storage->init) {
		ret = storage->init(storage);
		if (EFI_ERROR(ret))
			return ret;
	}

	media = media_new(storage);
	if (!media)
		return EFI_OUT_OF_RESOURCES;

	*handle = NULL;
	for (i = 0; i < ARRAY_SIZE(STORAGE_INTERFACES); i++) {
		res = strcmp("eraseblock", STORAGE_INTERFACES[i].name);
		if (!res && (boot_dev.type != STORAGE_VIRTUAL))
			continue;

		ret = STORAGE_INTERFACES[i].init(st, media, handle);
		if (EFI_ERROR(ret)) {
			ewerr("Failed to register %s interface",
			      STORAGE_INTERFACES[i].name);
			goto err;
		}
	}

	return EFI_SUCCESS;

err:
	for (j = 0; j < i; j++) {
		tmp_ret = STORAGE_INTERFACES[i].free(st, handle);
		if (EFI_ERROR(tmp_ret))
			ewerr("Failed to unregister %s interface",
			      STORAGE_INTERFACES[i].name);
	}
	free(media);
	return ret;
}

EFI_STATUS storage_free(EFI_SYSTEM_TABLE *st, EFI_HANDLE handle)
{
	EFI_STATUS ret;
	size_t i;
	int res;

	if (!st || !handle)
		return EFI_INVALID_PARAMETER;

	for (i = 0; i < ARRAY_SIZE(STORAGE_INTERFACES); i++) {
		res = strcmp("eraseblock", STORAGE_INTERFACES[i].name);
		if (!res && (boot_dev.type != STORAGE_VIRTUAL))
			continue;

		ret = STORAGE_INTERFACES[i].free(st, handle);
		if (EFI_ERROR(ret)) {
			ewerr("Failed to unregister %s interface",
			      STORAGE_INTERFACES[i].name);
			return ret;
		}
	}

	return EFI_SUCCESS;
}

static enum storage_type convert_sbl_dev_type(SBL_OS_BOOT_MEDIUM_TYPE type)
{
	switch(type) {
		case OsBootDeviceUfs:
			return STORAGE_UFS;
		case OsBootDeviceSata:
			return STORAGE_SATA;
		case OsBootDeviceNvme:
			return STORAGE_NVME;
		default:
			return STORAGE_EMMC;
	}
}

EFI_STATUS identify_flash_media(boot_dev_t* pdev)
{
	const char *val;
	OS_BOOT_DEVICE_LIST* plist;
	SBL_OS_BOOT_MEDIUM_TYPE type;

	val = ewarg_getval(ABL_BDEVLIST);
	if (!val) {
		ewdbg("No devlist, select default");
		return EFI_SUCCESS;
	}

	plist = (OS_BOOT_DEVICE_LIST*)(UINTN)strtoull(val, NULL, 16);

	if(plist->BootDeviceCount < 1)  {
		ewdbg("devlist count < 1, select default");
		return EFI_SUCCESS;
	}

	type = (SBL_OS_BOOT_MEDIUM_TYPE)(plist->BootDevice[0].DevType);

	if ((type != OsBootDeviceSpi) && (type != OsBootDeviceMemory)) {
		ewdbg("Select 1st boot dev");
		pdev->type = convert_sbl_dev_type(type);
		pdev->diskbus = plist->BootDevice[0].DevAddr;
		return EFI_SUCCESS;
	}

	//1st boot dev is SPI, check 2nd dev instead...

	if(plist->BootDeviceCount < 2)  {
		ewdbg("devlist count < 2, select default");
		return EFI_SUCCESS;
	}

	ewdbg("Select 2nd boot dev");
	type = (SBL_OS_BOOT_MEDIUM_TYPE)(plist->BootDevice[1].DevType);
	pdev->type = convert_sbl_dev_type(type);
	pdev->diskbus = plist->BootDevice[1].DevAddr;

	return EFI_SUCCESS;
}

EFI_STATUS identify_boot_media()
{
	const char *val;
	size_t len;

	val = ewarg_getval("bdev");
	if (!val)
		return EFI_SUCCESS;

	ewdbg("bdev = %s", val);
	len = strlen(val);
	if (!strncmp(val, "MMC", len))
		boot_dev.type = STORAGE_EMMC;
	else if (!strncmp(val, "UFS", len))
		boot_dev.type = STORAGE_UFS;
	else if (!strncmp(val, "NVME", len))
		boot_dev.type = STORAGE_NVME;
	else if (!strncmp(val, "VIRTUAL", len))
		boot_dev.type = STORAGE_VIRTUAL;
	else if ((!strncmp(val, "SPI", len)) || (!strncmp(val, "MEM", len))) //Fastboot case
		identify_flash_media(&boot_dev);

	//if diskbus is already get from boot option list
	if (boot_dev.diskbus != 0)
		return EFI_SUCCESS;

	val = ewarg_getval("diskbus");
	if (!val)
		return EFI_SUCCESS;

	ewdbg("diskbus = %s", val);
	boot_dev.diskbus = (UINT32)strtoull(val, NULL, 16);

	return EFI_SUCCESS;
}

boot_dev_t* get_boot_media()
{
	return &boot_dev;
}

uint32_t get_diskbus()
{
	return boot_dev.diskbus;
}

UINT8 get_boot_media_device_path_type(void)
{
	switch(boot_dev.type)
	{
	case STORAGE_EMMC:
		return MSG_EMMC_DP;

	case STORAGE_UFS:
		return MSG_UFS_DP;

	case STORAGE_NVME:
		return MSG_NVME_DP;

	case STORAGE_VIRTUAL:
		return MSG_VIRTUAL_MEDIA_DP;

	default:
		break;
	}

	return MSG_EMMC_DP;
}
