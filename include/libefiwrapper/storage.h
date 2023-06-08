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

#ifndef _STORAGE_H_
#define _STORAGE_H_

#include <efi.h>
#include <efiapi.h>

typedef struct storage {
	EFI_STATUS (*init)(struct storage *s);
	EFI_LBA (*read)(struct storage *s, EFI_LBA start, EFI_LBA count,
			void *buf);
	EFI_LBA (*write)(struct storage *s, EFI_LBA start, EFI_LBA count,
			 const void *buf);
	EFI_STATUS (*erase)(struct storage *s, EFI_LBA start, UINTN Size);
	UINT8 pci_function;
	UINT8 pci_device;
	EFI_LBA blk_cnt;
	UINT32 blk_sz;
	void *priv;
} storage_t;

enum storage_type {
	STORAGE_EMMC,
	STORAGE_UFS,
	STORAGE_SDCARD,
	STORAGE_SATA,
	STORAGE_NVME,
	STORAGE_VIRTUAL,
	STORAGE_ALL
};

typedef enum {
	OsBootDeviceSata,
	OsBootDeviceSd,
	OsBootDeviceEmmc,
	OsBootDeviceUfs,
	OsBootDeviceSpi,
	OsBootDeviceUsb,
	OsBootDeviceNvme,
	OsBootDeviceMemory,
	OsBootDeviceVirtual,
	OsBootDeviceMax
} SBL_OS_BOOT_MEDIUM_TYPE;

typedef struct boot_dev {
	enum storage_type type;
	UINT32 diskbus;
} boot_dev_t;

EFI_STATUS storage_init(EFI_SYSTEM_TABLE *st, storage_t *storage,
			EFI_HANDLE *handle_p);
EFI_STATUS storage_free(EFI_SYSTEM_TABLE *st, EFI_HANDLE handle);

EFI_STATUS identify_boot_media();

boot_dev_t* get_boot_media();
UINT8 get_boot_media_device_path_type(void);
uint32_t get_diskbus();
#endif	/* _STORAGE_H_ */
