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

#include <libpayload-config.h>
#include <libpayload.h>

#include <efiwrapper.h>
#include <protocol.h>

#include "serial.h"
#include "platform.h"
#include "boot_services.h"
#include "runtime_services.h"
#include "conout.h"
#include "conin.h"
#include "mmc.h"
#include "usb.h"
#include "smbios.h"

platform_t platform_struct;
platform_t *platform = &platform_struct;

static EFI_SYSTEM_TABLE systab = {
	.Hdr = {
		.Signature = 0,
		.Revision = 0,
		.HeaderSize = 0,
		.CRC32 = 0,
		.Reserved = 0,
	},

	.FirmwareVendor = L"Intel",
	.FirmwareRevision = 0,

	.NumberOfTableEntries = 0,
	.ConfigurationTable = NULL
};

static EFI_CONFIGURATION_TABLE config_table;

static EFI_LOADED_IMAGE loaded_image = {
	.Revision = 0x0,
	.ParentHandle = 0x0,
};

EFI_STATUS init(EFI_HANDLE *image)
{
	EFI_STATUS ret;

	platform->malloc = malloc;
	platform->free = free;
	platform->debug = printf;
	platform->error = printf;

	/* Register protocols */
	ret = payload_register_serial();
	if (EFI_ERROR(ret)) {
		printf("Failed to install the serial protocol\n");
		return ret;
	}

	ret = payload_register_conout(&systab.ConsoleOutHandle, &systab.ConOut);
	if (EFI_ERROR(ret)) {
		printf("Failed to install the conout protocol\n");
		return ret;
	}

	ret = payload_register_conin(&systab.ConsoleInHandle, &systab.ConIn);
	if (EFI_ERROR(ret)) {
		printf("Failed to install the conin protocol\n");
		return ret;
	}

	ret = payload_register_mmc("./disk");
	if (EFI_ERROR(ret)) {
		fprintf(stderr, "Failed to register the block io protocol\n");
		return ret;
	}

	ret = payload_register_usb();
	if (EFI_ERROR(ret)) {
		printf("Failed to register the usb device mode protocol\n");
		return ret;
	}

	*image = NULL;
	ret = ew_install_protocol_interface(image, &LoadedImageProtocol,
					    EFI_NATIVE_INTERFACE, &loaded_image);
	if (EFI_ERROR(ret)) {
		printf("Failed to register the LoadedImageProtocol\n");
		return ret;
	}

	ret = smbios_init_table(&config_table);
	if (EFI_ERROR(ret)) {
		printf("Failed to init the SMBIOS table\n");
		return EFI_DEVICE_ERROR;
	}
	systab.ConfigurationTable = &config_table;
	systab.NumberOfTableEntries++;

	loaded_image.LoadOptions = ew_str16dup(L"loader -f");
	loaded_image.SystemTable = &systab;

	ret = payload_boot_services(&systab.BootServices);
	if (EFI_ERROR(ret)) {
		printf("Failed to get the boot services structure\n");
		return ret;
	}

	ret = payload_runtime_services(&systab.RuntimeServices);
	if (EFI_ERROR(ret)) {
		printf("Failed to get the runtime services structure\n");
		return ret;
	}

	return EFI_SUCCESS;
}

/* Entry point of the EFI binary */
EFI_STATUS efi_main(EFI_HANDLE image, EFI_SYSTEM_TABLE *_table);

int main(void)
{
	EFI_HANDLE image;
	EFI_STATUS ret;

	ret = init(&image);
	if (EFI_ERROR(ret))
		return 1;


	ret = efi_main(image, &systab);
	if (EFI_ERROR(ret))
		Print(L"efi_main failed: %r\n", ret);

	halt();
	return 0;
}
