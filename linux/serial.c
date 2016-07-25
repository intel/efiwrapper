/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 *
 * Authors: Jérémy Compostella <jeremy.compostella@intel.com>
 *          Romain Vigier <romainx.vigier@intel.com>
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

#include <stdio.h>

#include <protocol.h>

#include "serial.h"

static EFIAPI EFI_STATUS
_reset(SERIAL_IO_INTERFACE *This)
{
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
_write(SERIAL_IO_INTERFACE *This,
       UINTN *BufferSize, VOID *Buffer)
{
	size_t i;

	if (*BufferSize == 0 || !Buffer)
		return EFI_INVALID_PARAMETER;

	for (i = 0; i < *BufferSize; i++)
		fprintf(stdout, "%c", ((char *)Buffer)[i]);

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
_set_attributes(__attribute__((__unused__)) struct _SERIAL_IO_INTERFACE *This,
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
_set_control(__attribute__((__unused__)) struct _SERIAL_IO_INTERFACE *This,
	     __attribute__((__unused__)) UINT32 Control)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_get_control(__attribute__((__unused__)) struct _SERIAL_IO_INTERFACE *This,
	     __attribute__((__unused__)) UINT32 *Control)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_read(SERIAL_IO_INTERFACE *This,
      UINTN *BufferSize, VOID *Buffer)
{
	return EFI_UNSUPPORTED;
}

static SERIAL_IO_MODE io_mode;

SERIAL_IO_INTERFACE serial_struct = {
	.Revision = 0x1,
	.Reset = _reset,
	.SetAttributes = _set_attributes,
	.SetControl = _set_control,
	.GetControl = _get_control,
	.Write = _write,
	.Read = _read,
	.Mode = &io_mode
};

static EFI_HANDLE serial_handle;
static EFI_GUID serial_guid = SERIAL_IO_PROTOCOL;

EFI_STATUS linux_register_serial(VOID)
{
	if (serial_handle)
		return EFI_ALREADY_STARTED;

	return ew_install_protocol_interface(&serial_handle, &serial_guid,
					     EFI_NATIVE_INTERFACE, &serial_struct);
}

EFI_STATUS linux_unregister_serial(VOID)
{
	EFI_STATUS ret;

	if (!serial_handle)
		return EFI_INVALID_PARAMETER;

	ret = ew_uninstall_protocol_interface(serial_handle, &serial_guid,
					      &serial_struct);
	if (!EFI_ERROR(ret))
		serial_handle = NULL;

	return ret;
}
