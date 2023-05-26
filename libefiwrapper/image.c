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

#include <interface.h>
#include <ewlog.h>

#include "image.h"
#include "pe.h"

static EFI_GUID image_guid = LOADED_IMAGE_PROTOCOL;
EFI_SYSTEM_TABLE *saved_st;

static EFIAPI EFI_STATUS
load_image(__attribute__((__unused__)) BOOLEAN BootPolicy,
	   EFI_HANDLE ParentImageHandle,
	   EFI_DEVICE_PATH *FilePath,
	   VOID *SourceBuffer,
	   UINTN SourceSize,
	   EFI_HANDLE *ImageHandle)
{
	EFI_STATUS ret;
	image_t tmp, *image;

	if (FilePath)
		return EFI_UNSUPPORTED;

	if (!SourceBuffer || !SourceSize || !ImageHandle)
		return EFI_INVALID_PARAMETER;

	memset(&tmp, 0, sizeof(tmp));
	tmp.prot.Revision = EFI_IMAGE_INFORMATION_REVISION;
	tmp.prot.ParentHandle = ParentImageHandle;
	tmp.prot.SystemTable = saved_st;

	ret = pe_load(SourceBuffer, SourceSize, &tmp);
	if (EFI_ERROR(ret)) {
		ewerr("Failed to load PE executable");
		return ret;
	}

	return interface_init(saved_st, &image_guid, ImageHandle,
			      (void *)&tmp, sizeof(tmp), (void **)&image);
}

static inline EFI_STATUS get_image(image_t **image, EFI_HANDLE handle)
{
	return uefi_call_wrapper(saved_st->BootServices->HandleProtocol, 3,
				 handle, &image_guid, (void **)image);
}

static EFIAPI EFI_STATUS
start_image(EFI_HANDLE ImageHandle,
	    UINTN *ExitDataSize,
	    CHAR16 **ExitData)
{
	EFI_STATUS ret;
	image_t *image;
#if defined(HOST)
	int setjmpret;
#endif

	if (ExitDataSize || ExitData)
		return EFI_UNSUPPORTED;

	ret = get_image(&image, ImageHandle);
	if (EFI_ERROR(ret))
		return ret;

#if defined(HOST)
	setjmpret = setjmp(image->jmp);
	if (setjmpret != 0)
		return image->exit_status;
#endif

	return image->entry(ImageHandle, saved_st);
}

static EFIAPI EFI_STATUS
efi_exit(EFI_HANDLE ImageHandle,
	 EFI_STATUS ExitStatus,
	 UINTN ExitDataSize,
	 __attribute__((__unused__)) CHAR16 *ExitData)
{
	EFI_STATUS ret;
	image_t *image;

	if (ExitDataSize)
		return EFI_UNSUPPORTED;

	ret = get_image(&image, ImageHandle);
	if (EFI_ERROR(ret))
		return ret;

	image->exit_status = ExitStatus;
#if defined(HOST)
	longjmp(image->jmp, 1);
#endif
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
unload_image(EFI_HANDLE ImageHandle)
{
	EFI_STATUS ret;
	image_t *image;

	ret = get_image(&image, ImageHandle);
	if (EFI_ERROR(ret))
		return ret;

	pe_unload(image);

	return interface_free(saved_st, &image_guid, ImageHandle);
}

static EFI_IMAGE_LOAD saved_load_image;
static EFI_IMAGE_START saved_start_image;
static EFI_EXIT saved_efi_exit;
static EFI_IMAGE_UNLOAD saved_unload_image;

EFI_STATUS image_init(EFI_SYSTEM_TABLE *st)
{
	if (!st)
		return EFI_INVALID_PARAMETER;

	saved_st = st;

	saved_load_image = st->BootServices->LoadImage;
	saved_start_image = st->BootServices->StartImage;
	saved_efi_exit = st->BootServices->Exit;
	saved_unload_image = st->BootServices->UnloadImage;

	st->BootServices->LoadImage = load_image;
	st->BootServices->StartImage = start_image;
	st->BootServices->Exit = efi_exit;
	st->BootServices->UnloadImage = unload_image;

	return EFI_SUCCESS;
}

EFI_STATUS image_free(EFI_SYSTEM_TABLE *st)
{
	if (!st)
		return EFI_INVALID_PARAMETER;

	st->BootServices->LoadImage = saved_load_image;
	st->BootServices->StartImage = saved_start_image;
	st->BootServices->Exit = saved_efi_exit;
	st->BootServices->UnloadImage = saved_unload_image;

	return EFI_SUCCESS;
}
