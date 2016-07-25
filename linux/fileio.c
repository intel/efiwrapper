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

#include <platform.h>
#include <protocol.h>
#include <efiwrapper.h>

#include "common.h"
#include "fileio.h"

typedef struct file {
	EFI_FILE interface;
	char *path;
	int fd;
} file_t;

static EFIAPI EFI_STATUS
_file_open(struct _EFI_FILE_HANDLE *File,
	   struct _EFI_FILE_HANDLE **NewHandle,
	   CHAR16 *FileName,
	   UINT64 OpenMode,
	   UINT64 Attributes)
{
	EFI_STATUS ret;
	file_t *file = (file_t *)File;
	file_t *new_file = NULL;
	int flags = 0;

	if (!File || !NewHandle || !FileName)
		return EFI_INVALID_PARAMETER;

	new_file = ew_malloc(sizeof(*new_file));
	if (!new_file) {
		ret = EFI_OUT_OF_RESOURCES;
		goto err;
	}

	new_file->path = ew_malloc(ew_strlen(file->path) + 1 + ew_str16len(FileName) + 1);
	if (!new_file->path) {
		ret = EFI_OUT_OF_RESOURCES;
		goto err;
	}
	ew_memcpy(&new_file->interface, &file->interface, sizeof(file->interface));

	sprintf(new_file->path, "%s/", file->path);
	ew_str16_to_str(FileName, new_file->path + ew_strlen(new_file->path));

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

	*NewHandle = (struct _EFI_FILE_HANDLE *)new_file;
	return EFI_SUCCESS;
err:
	if (new_file)
		ew_free(new_file->path);
	ew_free(new_file);
	return ret;
}

static EFIAPI EFI_STATUS
_file_close(struct _EFI_FILE_HANDLE *File)
{
	file_t *file = (file_t *)File;

	if (!file || file->fd == -1)
		return EFI_INVALID_PARAMETER;

	close(file->fd);
	if (file->path)
		ew_free(file->path);
	ew_free(file);
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
_file_delete(struct _EFI_FILE_HANDLE *File)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_file_read(struct _EFI_FILE_HANDLE *File,
	   UINTN *BufferSize,
	   VOID *Buffer)
{
	file_t *file = (file_t *)File;

	if (!file)
		return EFI_INVALID_PARAMETER;

	return read_fd(file->fd, Buffer, *BufferSize);
}

static EFIAPI EFI_STATUS
_file_write(struct _EFI_FILE_HANDLE *File,
	    UINTN *BufferSize,
	    VOID *Buffer)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_file_set_position(struct _EFI_FILE_HANDLE *File,
		   UINT64 Position)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_file_get_position(struct _EFI_FILE_HANDLE *File,
		   UINT64 *Position)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_file_get_info(struct _EFI_FILE_HANDLE *File,
	       EFI_GUID *InformationType,
	       UINTN *BufferSize,
	       VOID *Buffer)
{
	int retp;
	file_t *file = (file_t *)File;
	struct stat sb;
	EFI_FILE_INFO *info = Buffer;

	if (!File || !InformationType || !BufferSize || !Buffer)
		return EFI_INVALID_PARAMETER;

	if (ew_guidcmp(InformationType, &GenericFileInfo))
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
_file_set_info(struct _EFI_FILE_HANDLE *File,
	       EFI_GUID *InformationType,
	       UINTN BufferSize,
	       VOID *Buffer)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_file_flush(IN struct _EFI_FILE_HANDLE *File)
{
	return EFI_UNSUPPORTED;
}

static EFI_FILE file_handle = {
	.Revision = 0x1,
	.Open = _file_open,
	.Close = _file_close,
	.Delete = _file_delete,
	.Read = _file_read,
	.Write = _file_write,
	.GetPosition = _file_get_position,
	.SetPosition = _file_set_position,
	.GetInfo = _file_get_info,
	.SetInfo = _file_set_info,
	.Flush = _file_flush
};

typedef struct volume {
	struct _EFI_FILE_IO_INTERFACE interface;
	char *path;
	file_t file;
} volume_t;

static EFIAPI EFI_STATUS
_open_volume(struct _EFI_FILE_IO_INTERFACE *This,
	     struct _EFI_FILE_HANDLE **Root)
{
	volume_t *volume = (volume_t *)This;

	if (!This || !Root)
		return EFI_INVALID_PARAMETER;

	*Root = (struct _EFI_FILE_HANDLE *)&volume->file;

	return EFI_SUCCESS;
}

static EFI_FILE_IO_INTERFACE file_io_struct = {
	.Revision = 0x0,
	.OpenVolume = _open_volume
};

EFI_STATUS linux_register_fileio(char *rootdir, EFI_HANDLE *handle)
{
	static volume_t root;

	if (!rootdir || !handle)
		return EFI_INVALID_PARAMETER;

	*handle = NULL;
	root.path = rootdir;
	root.file.path = rootdir;
	root.file.fd = -1;
	ew_memcpy(&root.file.interface, &file_handle, sizeof(file_handle));
	ew_memcpy(&root.interface, &file_io_struct, sizeof(file_io_struct));

	return ew_install_protocol_interface(handle, &FileSystemProtocol,
					     EFI_NATIVE_INTERFACE, &root);
}

EFI_STATUS linux_unregister_fileio(EFI_HANDLE handle)
{
	EFI_STATUS ret;
	static volume_t *root;

	if (!handle)
		return EFI_INVALID_PARAMETER;

	ret = ew_handle_protocol(handle, &FileSystemProtocol, (VOID **)&root);
	if (EFI_ERROR(ret))
		return ret;

	return ew_uninstall_protocol_interface(handle, &FileSystemProtocol,
					       (VOID *)root);
}
