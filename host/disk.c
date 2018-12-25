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

#define _LARGEFILE64_SOURCE
#include <ewlog.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <storage.h>
#include <sdio.h>

#include "disk.h"

static const char *DISK_PATH = "./disk.img";
static const UINT32 DISK_BLK_SZ = 512;
static const UINT64 DISK_SIZE = (uint64_t)12 * 1024 * 1024 * 1024;

static int fd = -1;

static EFI_STATUS _init(storage_t *s)
{
	char c = 0;
	ssize_t size;
	off64_t off;
	int ret;
	struct stat sb;

	s->blk_sz = DISK_BLK_SZ;

	fd = open(DISK_PATH, O_RDWR, 0644);
	if (fd >= 0) {
		ret = fstat(fd, &sb);
		if (ret == -1) {
			ewerr("Failed to fstat %s disk file, %s",
			      DISK_PATH, strerror(errno));
			return EFI_DEVICE_ERROR;
		}

		if (!S_ISREG(sb.st_mode))
			return EFI_DEVICE_ERROR;

		s->blk_cnt = sb.st_size / DISK_BLK_SZ;
		return EFI_SUCCESS;
	}
	if (errno != ENOENT) {
		ewerr("Failed to open %s disk file, %s",
		      DISK_PATH, strerror(errno));
		return EFI_DEVICE_ERROR;
	}

	s->blk_cnt = DISK_SIZE / DISK_BLK_SZ;

	fd = open(DISK_PATH, O_RDWR | O_CREAT, 0644);
	if (fd == -1) {
		ewerr("Failed to create %s disk file, %s",
		      DISK_PATH, strerror(errno));
		return EFI_DEVICE_ERROR;
	}

	off = lseek64(fd, DISK_SIZE - 1, SEEK_SET);
	if (off != DISK_SIZE - 1) {
		ewerr("Failed to seek till the end of disk, %s",
		      strerror(errno));
		return EFI_DEVICE_ERROR;
	}

	size = write(fd, &c, sizeof(c));
	if (size != sizeof(c)) {
		ewerr("Failed to initialize disk file, %s",
		      strerror(errno));
		return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}

static EFI_LBA read_or_write(storage_t *s, EFI_LBA start, EFI_LBA count,
			     void *buf, bool do_read)
{
	ssize_t ret;
	off64_t off;
	size_t remaining, total;
	char *b;

	if (!s || !buf || start + count > s->blk_cnt)
		return EFI_INVALID_PARAMETER;

	if (fd == -1)
		return EFI_NOT_STARTED;

	off = lseek64(fd, start * s->blk_sz, SEEK_SET);
	if (off != (off64_t)start * s->blk_sz) {
		ewerr("Failed to seek in the disk file, %s",
		      strerror(errno));
		return 0;
	}

	b = buf;
	total = 0;
	for (remaining = count * s->blk_sz; remaining > 0; remaining -= ret) {
		if (do_read)
			ret = read(fd, b, remaining);
		else
			ret = write(fd, b, remaining);

		if (do_read && ret == 0) {
			ewerr("End of file detected");
			return total / s->blk_sz;
		}
		if (ret == -1) {
			ewerr("Failed to %s disk file, %s",
			      do_read ? "read from" : "write to",
			      strerror(errno));
			return total / s->blk_sz;
		}
		total += ret;
		b += ret;
	}

	return count;
}

static EFI_LBA _read(storage_t *s, EFI_LBA start, EFI_LBA count,
		     void *buf)
{
	return read_or_write(s, start, count, buf, true);
}

static EFI_LBA _write(storage_t *s, EFI_LBA start, EFI_LBA count,
		      const void *buf)
{
	return read_or_write(s, start, count, (void *)buf, false);
}

static storage_t disk_storage = {
	.init = _init,
	.read = _read,
	.write = _write,
	.erase = NULL,
	.pci_function = 0,
	.pci_device = 0
};

static EFI_HANDLE handle;

static EFI_STATUS disk_init(EFI_SYSTEM_TABLE *st)
{

	EFI_STATUS ret;

	if (!st)
		return EFI_INVALID_PARAMETER;

	if (handle)
		return EFI_ALREADY_STARTED;

	ret = storage_init(st, &disk_storage, &handle);
	if (EFI_ERROR(ret))
		return ret;

	ret = sdio_init(st, handle, &disk_storage);
	if (EFI_ERROR(ret))
		storage_free(st, handle);

	return ret;
}

static EFI_STATUS disk_exit(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;

	if (!st)
		return EFI_INVALID_PARAMETER;

	if (!handle)
		return EFI_NOT_STARTED;

	if (fd != -1)
		close(fd);

	ret = sdio_free(st, handle);
	if (EFI_ERROR(ret))
		return ret;

	ret = storage_free(st, handle);
	if (EFI_ERROR(ret))
		return ret;

	handle = NULL;
	return EFI_SUCCESS;
}

ewdrv_t disk_drv = {
	.name = "disk",
	.description = "Emulate eMMC storage",
	.init = disk_init,
	.exit = disk_exit
};
