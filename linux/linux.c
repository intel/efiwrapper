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

#include <stdlib.h>
#include <stdio.h>
#include <libgen.h>

#include <efiwrapper.h>

#include "boot_services.h"
#include "runtime_services.h"
#include "serial.h"
#include "platform.h"
#include "protocol.h"
#include "conout.h"
#include "conin.h"
#include "fileio.h"
#include "mmc.h"

platform_t platform_struct;
platform_t *platform = &platform_struct;

static EFI_SYSTEM_TABLE linux_systab = {
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

static EFI_LOADED_IMAGE loaded_image = {
	.Revision = 0x0,
	.ParentHandle = 0x0,
};

CHAR16 *load_options(int argc, char **argv)
{
	size_t i;
	char *options = "", *tmp;
	CHAR16 *opt16;

	for (i = 0; i < argc; i++) {
		if (i == 0) {
			asprintf(&tmp, "%s", argv[i]);
		} else {
			asprintf(&tmp, "%s %s", options, argv[i]);
			ew_free(options);
		}
		options = tmp;
	}

	opt16 = ew_str_to_str16_p(options);
	ew_free(options);
	return opt16;
}

EFI_STATUS linux_init(EFI_HANDLE *image, EFI_SYSTEM_TABLE **systab_p,
		      int argc, char **argv)
{
	EFI_STATUS ret;

	if (!image || !systab_p || !argv)
		return EFI_INVALID_PARAMETER;

	/* Platform specific functions */
	platform->malloc = malloc;
	platform->free = free;
	platform->debug = printf;
	platform->error = printf;

	/* Register protocols */
	ret = linux_register_serial();
	if (EFI_ERROR(ret)) {
		fprintf(stderr, "Failed to install the serial protocol\n");
		return ret;
	}

	ret = linux_register_conout(&linux_systab.ConsoleOutHandle, &linux_systab.ConOut);
	if (EFI_ERROR(ret)) {
		fprintf(stderr, "Failed to install the conout protocol\n");
		return ret;
	}

	ret = linux_register_conin(&linux_systab.ConsoleInHandle, &linux_systab.ConIn);
	if (EFI_ERROR(ret)) {
		fprintf(stderr, "Failed to install the conin protocol\n");
		return ret;
	}

	ret = linux_register_mmc("./disk", 512, 25165824); /* 12GB */
	if (EFI_ERROR(ret)) {
		fprintf(stderr, "Failed to register the block io protocol\n");
		return ret;
	}

	*image = NULL;
	ret = ew_install_protocol_interface(image, &LoadedImageProtocol,
					    EFI_NATIVE_INTERFACE, &loaded_image);
	if (EFI_ERROR(ret)) {
		fprintf(stderr, "Failed to register the LoadedImageProtocol\n");
		return ret;
	}

	/* Loaded image */
	loaded_image.LoadOptions = load_options(argc, argv);
	ret = linux_register_fileio(dirname(argv[0]), &loaded_image.DeviceHandle);
	if (EFI_ERROR(ret)) {
		fprintf(stderr, "Failed to register the file IO protocol\n");
		return ret;
	}
	loaded_image.SystemTable = &linux_systab;

	ret = linux_boot_services(&linux_systab.BootServices);
	if (EFI_ERROR(ret)) {
		fprintf(stderr, "Failed to get the boot services structure\n");
		return ret;
	}

	ret = linux_runtime_services(&linux_systab.RuntimeServices);
	if (EFI_ERROR(ret)) {
		fprintf(stderr, "Failed to get the runtime services structure\n");
		return ret;
	}

	*systab_p = &linux_systab;

	return 0;
}

VOID linux_free(EFI_HANDLE image, EFI_SYSTEM_TABLE *systab_p)
{
	EFI_STATUS ret;

	ret = linux_unregister_serial();
	if (EFI_ERROR(ret))
		fprintf(stderr, "Failed to uninstall the serial protocol\n");

	ret = linux_unregister_conout();
	if (EFI_ERROR(ret))
		fprintf(stderr, "Failed to uninstall the conout protocol\n");

	ret = linux_unregister_conin();
	if (EFI_ERROR(ret))
		fprintf(stderr, "Failed to uninstall the conin protocol\n");

	ret = linux_unregister_mcc();
	if (EFI_ERROR(ret))
		fprintf(stderr, "Failed to uninstall the block io protocol\n");

	ret = linux_unregister_fileio(loaded_image.DeviceHandle);
	if (EFI_ERROR(ret))
		fprintf(stderr, "Failed to uninstall the file io protocol\n");

	ret = linux_free_runtime_services(linux_systab.RuntimeServices);
	if (EFI_ERROR(ret)) {
		fprintf(stderr, "Failed to free the runtime services structure");
	}
}
