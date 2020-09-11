/*
 * Copyright (c) 2016-2020, Intel Corporation
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

#include "bs.h"
#include "conin.h"
#include "conout.h"
#include "ewarg.h"
#include "ewlog.h"
#include "ewvar.h"
#include "image.h"
#include "lib.h"
#include "rs.h"
#include "serialio.h"
#include "smbios.h"
#include "storage.h"
#include "version.h"

static EFI_GUID image_guid = LOADED_IMAGE_PROTOCOL;
static EFI_BOOT_SERVICES bs;
static EFI_RUNTIME_SERVICES rs;

static EFI_SYSTEM_TABLE st = {
	.Hdr = {
		.Signature = EFI_SYSTEM_TABLE_SIGNATURE,
		.Revision = EFI_SYSTEM_TABLE_REVISION,
		.HeaderSize = sizeof(EFI_TABLE_HEADER),
		.CRC32 = 0,
		.Reserved = 0,
	},

	.BootServices = &bs,
	.RuntimeServices = &rs,

	.FirmwareVendor = L"Intel",
	.FirmwareRevision = 0,
};

static EFI_LOADED_IMAGE img = {
	.Revision = EFI_IMAGE_INFORMATION_REVISION,
	.ParentHandle = NULL,
	.SystemTable = &st
};

static EFI_GUID EFIWRAPPER_GUID =
	{ 0x59d0d866, 0x5637, 0x47a9,
	  { 0xb7, 0x50, 0x42, 0x60, 0x0a, 0x54, 0x5b, 0x63 }};

static struct {
	UINT32 MajorRevision;
	UINT32 MinorRevision;
} efiwrapper = {
	.MajorRevision = EFIWRAPPER_MAJOR,
	.MinorRevision = EFIWRAPPER_MINOR
};

static struct component {
	const char *name;
	EFI_STATUS (*init)(EFI_SYSTEM_TABLE *st);
	EFI_STATUS (*free)(EFI_SYSTEM_TABLE *st);
} COMPONENTS[] = {
	{ "boot services", bs_init, NULL },
	{ "runtime services", rs_init, NULL },
	{ "console in", conin_init, conin_free },
	{ "console out", conout_init, conout_free },
	{ "serial", serialio_init, serialio_free },
	{ "smbios", smbios_init, smbios_free },
	{ "image", image_init, image_free }
};

EFI_STATUS set_load_options(int argc, char **argv)
{
	size_t i, size = 0;
	char *opt, *p, *cur;

	if (!argc)
		return EFI_SUCCESS;

	for (i = 0; i < (size_t)argc; i++)
		size += strlen(argv[i]) + 1;

	opt = malloc(size);
	if (!opt)
		return EFI_OUT_OF_RESOURCES;

	p = opt;
	for (i = 0; i < (size_t)argc; i++) {
		if (i != 0)
			*p++ = ' ';
		cur = argv[i];
		while (*cur)
			*p++ = *cur++;
	}
	*p = '\0';
	img.LoadOptions = str2str16_p(opt);
	free(opt);
	if (!img.LoadOptions)
		return EFI_OUT_OF_RESOURCES;

	return EFI_SUCCESS;
}

EFI_STATUS efiwrapper_init(int argc, char **argv, EFI_SYSTEM_TABLE **st_p,
			   EFI_HANDLE *img_handle)
{
	EFI_STATUS ret;
	size_t i, j;

	if ((argc && !argv) || !st_p || !img_handle)
		return EFI_INVALID_PARAMETER;

	for (i = 0; i < ARRAY_SIZE(COMPONENTS); i++) {
		ret = COMPONENTS[i].init(&st);
		if (EFI_ERROR(ret)) {
			ewerr("%s failed to initilized", COMPONENTS[i].name);
			goto err_components;
		}
	}

	ret = uefi_call_wrapper(st.BootServices->InstallProtocolInterface, 4,
				img_handle, &image_guid,
				EFI_NATIVE_INTERFACE, &img);
	if (EFI_ERROR(ret))
		goto err_components;

	ret = uefi_call_wrapper(st.BootServices->InstallProtocolInterface, 4,
				img_handle, &EFIWRAPPER_GUID,
				EFI_NATIVE_INTERFACE, &efiwrapper);
	if (EFI_ERROR(ret))
		goto err_img;

	ret = set_load_options(argc, argv);
	if (EFI_ERROR(ret))
		goto err_efiwrapper;

	ret = crc32((void *)&st, sizeof(st), &st.Hdr.CRC32);
	if (EFI_ERROR(ret))
		goto err_load_options;

	ret = ewarg_init(argc, argv);
	if (EFI_ERROR(ret))
		goto err_load_options;

	ret = identify_boot_media();
	if (EFI_ERROR(ret))
		goto err_load_options;

	*st_p = &st;

	return EFI_SUCCESS;

err_load_options:
	free(img.LoadOptions);

err_efiwrapper:
	uefi_call_wrapper(st.BootServices->UninstallProtocolInterface, 3,
			  *img_handle, &EFIWRAPPER_GUID, &img);

err_img:
	uefi_call_wrapper(st.BootServices->UninstallProtocolInterface, 3,
			  *img_handle, &image_guid, &img);

err_components:
	for (j = 0; j < i; j++) {
		if (!COMPONENTS[j].free)
			continue;
		COMPONENTS[j].free(&st);
	}

	return ret;
}

EFI_STATUS efiwrapper_free(EFI_HANDLE img_handle)
{
	EFI_STATUS ret;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(COMPONENTS); i++) {
		if (!COMPONENTS[i].free)
			continue;
		ret = COMPONENTS[i].free(&st);
		if (EFI_ERROR(ret)) {
			ewerr("%s failed to exit", COMPONENTS[i].name);
			return ret;
		}
	}

	ret = uefi_call_wrapper(st.BootServices->UninstallProtocolInterface, 3,
				img_handle, &image_guid, &img);
	if (EFI_ERROR(ret))
		return ret;

	ewvar_free_all();
	free(img.LoadOptions);

	ewarg_free();

	return EFI_SUCCESS;
}
