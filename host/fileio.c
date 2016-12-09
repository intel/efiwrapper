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

#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <ewlib.h>
#include <errno.h>

#include <interface.h>
#include <efiwrapper.h>

#include "fileio.h"

typedef struct file {
	EFI_FILE interface;
	char *path;
	int fd;
} file_t;

static void str16_to_str(CHAR16 *s16, char *s)
{
	for (; *s16; s16++, s++)
		*s = *s16;

	*s = '\0';
}

static EFIAPI EFI_STATUS
file_open(EFI_FILE_HANDLE File,
	  EFI_FILE_HANDLE *NewHandle,
	  CHAR16 *FileName,
	  UINT64 OpenMode,
	  UINT64 Attributes)
{
	EFI_STATUS ret;
	file_t *file = (file_t *)File;
	file_t *new_file;
	int flags = 0;

	if (!File || !NewHandle || !FileName)
		return EFI_INVALID_PARAMETER;

	new_file = malloc(sizeof(*new_file));
	if (!new_file) {
		ret = EFI_OUT_OF_RESOURCES;
		goto err;
	}

	new_file->path = malloc(strlen(file->path) + 1 + str16len(FileName) + 1);
	if (!new_file->path) {
		ret = EFI_OUT_OF_RESOURCES;
		goto err;
	}
	memcpy(&new_file->interface, &file->interface, sizeof(file->interface));

	sprintf(new_file->path, "%s/", file->path);
	str16_to_str(FileName, new_file->path + strlen(new_file->path));

	if ((Attributes & EFI_FILE_MODE_READ) && (Attributes & EFI_FILE_MODE_WRITE))
		flags |= O_RDWR;
	else if (Attributes & EFI_FILE_MODE_READ)
		flags |= O_RDONLY;
	else if (Attributes & EFI_FILE_MODE_WRITE)
		flags |= O_WRONLY;

	if (Attributes & EFI_FILE_MODE_CREATE)
		flags |= O_CREAT;

	new_file->fd = open(new_file->path, flags, 0644);
	if (new_file->fd == -1) {
		ret = EFI_DEVICE_ERROR;
		goto err;
	}

	*NewHandle = (EFI_FILE_HANDLE)new_file;
	return EFI_SUCCESS;

err:
	if (new_file)
		free(new_file->path);
	free(new_file);

	return ret;
}

static EFIAPI EFI_STATUS
file_close(EFI_FILE_HANDLE File)
{
	file_t *file = (file_t *)File;

	if (!file || file->fd == -1)
		return EFI_INVALID_PARAMETER;

	close(file->fd);
	if (file->path)
		free(file->path);
	free(file);

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
file_delete(EFI_FILE_HANDLE File)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
file_read(EFI_FILE_HANDLE File,
	  UINTN *BufferSize,
	  VOID *Buffer)
{
	file_t *file = (file_t *)File;
	ssize_t nb;
	size_t remaining;

	if (!file)
		return EFI_INVALID_PARAMETER;

	for (remaining = *BufferSize; remaining; remaining -= nb) {
		nb = read(file->fd, (char *)Buffer + *BufferSize - remaining,
			  remaining);
		if (nb == 0)
			return EFI_INVALID_PARAMETER;

		if (nb == -1) {
			if (errno == EINTR)
				continue;
			return EFI_DEVICE_ERROR;
		}
	}

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
file_write(EFI_FILE_HANDLE File,
	   UINTN *BufferSize,
	   VOID *Buffer)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
file_set_position(EFI_FILE_HANDLE File,
		  UINT64 Position)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
file_get_position(EFI_FILE_HANDLE File,
		  UINT64 *Position)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
file_get_info(EFI_FILE_HANDLE File,
	      EFI_GUID *InformationType,
	      UINTN *BufferSize,
	      VOID *Buffer)
{
	int retp;
	file_t *file = (file_t *)File;
	struct stat sb;
	EFI_FILE_INFO *info = Buffer;
	EFI_GUID fileinfo_guid = EFI_FILE_INFO_ID;

	if (!File || !InformationType || !BufferSize || !Buffer)
		return EFI_INVALID_PARAMETER;

	if (guidcmp(InformationType, &fileinfo_guid))
		return EFI_UNSUPPORTED;

	if (*BufferSize < sizeof(EFI_FILE_INFO))
		return EFI_INVALID_PARAMETER;

	retp = fstat(file->fd, &sb);
	if (retp == -1)
		return EFI_DEVICE_ERROR;

	info->Size = sizeof(*info);
	info->FileSize = sb.st_size;

	*BufferSize = sizeof(*info);
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
file_set_info(EFI_FILE_HANDLE File,
	      EFI_GUID *InformationType,
	      UINTN BufferSize,
	      VOID *Buffer)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
file_flush(EFI_FILE_HANDLE File)
{
	return EFI_UNSUPPORTED;
}

static EFI_FILE file_handle = {
	.Revision = EFI_FILE_HANDLE_REVISION,
	.Open = file_open,
	.Close = file_close,
	.Delete = file_delete,
	.Read = file_read,
	.Write = file_write,
	.GetPosition = file_get_position,
	.SetPosition = file_set_position,
	.GetInfo = file_get_info,
	.SetInfo = file_set_info,
	.Flush = file_flush
};

typedef struct volume {
	EFI_FILE_IO_INTERFACE interface;
	char *path;
	file_t file;
} volume_t;

static EFIAPI EFI_STATUS
open_volume(EFI_FILE_IO_INTERFACE *This,
	    EFI_FILE_HANDLE *Root)
{
	volume_t *volume = (volume_t *)This;

	if (!This || !Root)
		return EFI_INVALID_PARAMETER;

	*Root = (EFI_FILE_HANDLE)&volume->file;
	return EFI_SUCCESS;
}

static EFI_FILE_IO_INTERFACE file_io_struct = {
	.Revision = EFI_FILE_IO_INTERFACE_REVISION,
	.OpenVolume = open_volume
};

static EFI_GUID fileio_guid = SIMPLE_FILE_SYSTEM_PROTOCOL;
static EFI_HANDLE fileio_handle;

static EFI_STATUS fileio_init(EFI_SYSTEM_TABLE *st)
{
	static volume_t root_default;
	EFI_STATUS ret;
	volume_t *root;
	EFI_GUID image_guid = LOADED_IMAGE_PROTOCOL;
	EFI_HANDLE image_handle;
	EFI_LOADED_IMAGE *img;
	UINTN size;

	if (!st)
		return EFI_INVALID_PARAMETER;

	if (fileio_handle)
		return EFI_ALREADY_STARTED;

	if (!root_default.path) {
		root_default.path = root_default.file.path = "./";
		root_default.file.fd = -1;
		memcpy(&root_default.file.interface, &file_handle,
		       sizeof(file_handle));
		memcpy(&root_default.interface, &file_io_struct,
		       sizeof(file_io_struct));
	}

	size = sizeof(image_handle);
	ret = uefi_call_wrapper(st->BootServices->LocateHandle, 5,
				ByProtocol, &image_guid, NULL,
				&size, &image_handle);
	if (EFI_ERROR(ret))
		return ret;

	ret = uefi_call_wrapper(st->BootServices->HandleProtocol, 3,
				image_handle, &image_guid, (void **)&img);
	if (EFI_ERROR(ret))
		return ret;

	ret = interface_init(st, &fileio_guid, &fileio_handle,
			     &root_default, sizeof(root_default),
			     (void **)&root);
	if (EFI_ERROR(ret))
		return ret;

	img->DeviceHandle = fileio_handle;
	return EFI_SUCCESS;
}

static EFI_STATUS fileio_exit(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;

	if (!st)
		return EFI_INVALID_PARAMETER;

	if (!fileio_handle)
		return EFI_NOT_STARTED;

	ret = interface_free(st, &fileio_guid, fileio_handle);
	if (EFI_ERROR(ret))
		return ret;

	fileio_handle = NULL;
	return EFI_SUCCESS;
}

ewdrv_t fileio_drv = {
	.name = "fileio",
	.description = "File System Protocol support",
	.init = fileio_init,
	.exit = fileio_exit
};
