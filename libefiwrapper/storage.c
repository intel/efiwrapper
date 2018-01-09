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

static EFI_GUID dp_guid = DEVICE_PATH_PROTOCOL;

#define ABL_BDEV "ABL.bdev"
#define ABL_DISKBUS "ABL.diskbus"

static boot_dev_t boot_dev = {
	.type = STORAGE_EMMC,
	.diskbus = 0
};

/* Device path  */
struct storage_dp {
	PCI_DEVICE_PATH pci;
	CONTROLLER_DEVICE_PATH ctrl;
	SCSI_DEVICE_PATH msg_device_path;
	EFI_DEVICE_PATH end;
} __attribute__((__packed__));

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
	{ "diskio", diskio_init, diskio_free }
};

EFI_STATUS storage_init(EFI_SYSTEM_TABLE *st, storage_t *storage,
			EFI_HANDLE *handle)
{
	EFI_STATUS ret, tmp_ret;
	size_t i, j;
	media_t *media;

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

	if (!st || !handle)
		return EFI_INVALID_PARAMETER;

	for (i = 0; i < ARRAY_SIZE(STORAGE_INTERFACES); i++) {
		ret = STORAGE_INTERFACES[i].free(st, handle);
		if (EFI_ERROR(ret)) {
			ewerr("Failed to unregister %s interface",
			      STORAGE_INTERFACES[i].name);
			return ret;
		}
	}

	return EFI_SUCCESS;
}

EFI_STATUS identify_boot_media()
{
	const char *val;
	size_t len;

	val = ewarg_getval(ABL_BDEV);
	if (!val)
		return EFI_SUCCESS;

	len = strlen(val);
	if (!strncmp(val, "MMC", len))
		boot_dev.type = STORAGE_EMMC;
	else if (!strncmp(val, "UFS", len))
		boot_dev.type = STORAGE_UFS;
	else if (!strncmp(val, "NVME", len))
		boot_dev.type = STORAGE_NVME;

	val = ewarg_getval(ABL_DISKBUS);
	if (!val)
		return EFI_SUCCESS;

	boot_dev.diskbus = (UINT32)strtol(val, NULL, 16);

	return EFI_SUCCESS;
}

boot_dev_t* get_boot_media()
{
	return &boot_dev;
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

	default:
		break;
	}

	return MSG_EMMC_DP;
}

