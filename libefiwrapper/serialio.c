/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 *
 * Authors: Jérémy Compostella <jeremy.compostella@intel.com>
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

#include "external.h"
#include "interface.h"
#include "serialio.h"
#include "efiser.h"

static EFIAPI EFI_STATUS
serialio_reset(__attribute__((__unused__)) SERIAL_IO_INTERFACE *This)
{
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
serialio_write(__attribute__((__unused__)) SERIAL_IO_INTERFACE *This,
	       UINTN *BufferSize, VOID *Buffer)
{
	size_t i;

	if (!This || !BufferSize || !Buffer)
		return EFI_INVALID_PARAMETER;

	for (i = 0; i < *BufferSize; i++)
		printf("%c", ((char *)Buffer)[i]);

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
serialio_set_attributes(__attribute__((__unused__)) _SERIAL_IO_INTERFACE *This,
			__attribute__((__unused__)) UINT64 BaudRate,
			__attribute__((__unused__)) UINT32 ReceiveFifoDepth,
			__attribute__((__unused__)) UINT32 Timeout,
			__attribute__((__unused__)) EFI_PARITY_TYPE Parity,
			__attribute__((__unused__)) UINT8 DataBits,
			__attribute__((__unused__)) EFI_STOP_BITS_TYPE StopBits)
{
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
serialio_set_control(__attribute__((__unused__)) _SERIAL_IO_INTERFACE *This,
		     __attribute__((__unused__)) UINT32 Control)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
serialio_get_control(__attribute__((__unused__)) _SERIAL_IO_INTERFACE *This,
		     __attribute__((__unused__)) UINT32 *Control)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
serialio_read(__attribute__((__unused__)) SERIAL_IO_INTERFACE *This,
	      __attribute__((__unused__)) UINTN *BufferSize,
	      __attribute__((__unused__)) VOID *Buffer)
{
	return EFI_UNSUPPORTED;
}

static SERIAL_IO_MODE io_mode;
static EFI_GUID serialio_guid = SERIAL_IO_PROTOCOL;
static EFI_HANDLE handle;

EFI_STATUS serialio_init(EFI_SYSTEM_TABLE *st)
{
	static SERIAL_IO_INTERFACE serialio_default = {
		.Revision = SERIAL_IO_INTERFACE_REVISION,
		.Reset = serialio_reset,
		.SetAttributes = serialio_set_attributes,
		.SetControl = serialio_set_control,
		.GetControl = serialio_get_control,
		.Write = serialio_write,
		.Read = serialio_read,
		.Mode = &io_mode
	};
	SERIAL_IO_INTERFACE *serialio;

	if (!st)
		return EFI_INVALID_PARAMETER;

	if (handle)
		return EFI_ALREADY_STARTED;

	return interface_init(st, &serialio_guid, &handle,
			      &serialio_default, sizeof(serialio_default),
			      (void **)&serialio);
}

EFI_STATUS serialio_free(EFI_SYSTEM_TABLE *st)
{
	if (!handle)
		return EFI_INVALID_PARAMETER;

	return interface_free(st, &serialio_guid, handle);
}
