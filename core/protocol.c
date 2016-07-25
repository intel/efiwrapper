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

#include "efiwrapper.h"
#include "protocol.h"

#define MAX_INTERFACE_NUMBER 32

typedef struct interface {
	EFI_HANDLE handle;
	EFI_GUID protocol;
	VOID *interface;
	BOOLEAN installed;
} interface_t;

static interface_t INTERFACES[MAX_INTERFACE_NUMBER];

EFIAPI EFI_STATUS
ew_install_protocol_interface(EFI_HANDLE *Handle,
			      EFI_GUID *Protocol,
			      EFI_INTERFACE_TYPE InterfaceType,
			      VOID *Interface)
{
	interface_t *inte;
	unsigned int i;

	if (!Handle || !Protocol ||
	    InterfaceType != EFI_NATIVE_INTERFACE)
		return EFI_INVALID_PARAMETER;

	if (*Handle) {
		for (i = 0; i < ARRAY_SIZE(INTERFACES); i++)
			if (INTERFACES[i].installed &&
			    !ew_guidcmp(&INTERFACES[i].protocol, Protocol) &&
			    INTERFACES[i].handle == *Handle)
				return EFI_INVALID_PARAMETER;
	}

	for (i = 0; i < ARRAY_SIZE(INTERFACES); i++) {
		inte = &INTERFACES[i];
		if (!inte->installed) {
			if (*Handle)
				inte->handle = *Handle;
			else {
				inte->handle = inte;
				*Handle = inte->handle;
			}
			ew_memcpy(&inte->protocol, Protocol, sizeof(*Protocol));
			inte->interface = Interface;
			inte->installed = TRUE;

			return EFI_SUCCESS;
		}
	}

	return EFI_OUT_OF_RESOURCES;
}

EFIAPI EFI_STATUS
ew_reinstall_protocol_interface(EFI_HANDLE Handle,
				EFI_GUID *Protocol,
				VOID *OldInterface,
				VOID *NewInterface)
{
	unsigned int i;

	if (!Handle || !Protocol)
		return EFI_INVALID_PARAMETER;

	for (i = 0; i < ARRAY_SIZE(INTERFACES); i++)
		if (INTERFACES[i].installed &&
		    INTERFACES[i].handle == Handle &&
		    INTERFACES[i].interface == OldInterface) {
			INTERFACES[i].interface = NewInterface;
			return EFI_SUCCESS;
		}

	return EFI_NOT_FOUND;
}

EFIAPI EFI_STATUS
ew_uninstall_protocol_interface(EFI_HANDLE Handle,
				EFI_GUID *Protocol,
				VOID *Interface)
{
	interface_t *inte;
	unsigned int i;

	if (!Handle || !Protocol)
		return EFI_INVALID_PARAMETER;

	for (i = 0; i < ARRAY_SIZE(INTERFACES); i++) {
		inte = &INTERFACES[i];
		if (inte->installed && inte->handle == Handle &&
		    !ew_guidcmp(&inte->protocol, Protocol) &&
		    inte->interface == Interface) {
			inte->installed = FALSE;
			return EFI_SUCCESS;
		}
	}

	return EFI_NOT_FOUND;
}

EFIAPI EFI_STATUS
ew_handle_protocol(EFI_HANDLE Handle,
		   EFI_GUID *Protocol,
		   VOID **Interface)
{
	interface_t *inte;
	unsigned int i;

	if (!Handle || !Protocol || !Interface)
		return EFI_INVALID_PARAMETER;

	for (i = 0; i < ARRAY_SIZE(INTERFACES); i++) {
		inte = &INTERFACES[i];
		if (inte->installed &&
		    inte->handle == Handle &&
		    !ew_guidcmp(&inte->protocol, Protocol)) {
			*Interface = inte->interface;
			return EFI_SUCCESS;
		}
	}

	return EFI_NOT_FOUND;
}

EFIAPI EFI_STATUS
ew_locate_handle(EFI_LOCATE_SEARCH_TYPE SearchType,
		 EFI_GUID *Protocol,
		 VOID *SearchKey,
		 UINTN *BufferSize,
		 EFI_HANDLE *Buffer)
{
	interface_t *inte;
	unsigned int i, nb = 0;

	if (!Protocol || !BufferSize || !Buffer)
		return EFI_INVALID_PARAMETER;

	if (SearchType != AllHandles &&
	    SearchType != ByRegisterNotify &&
	    SearchType != ByProtocol)
		return EFI_INVALID_PARAMETER;

	if (SearchType == ByRegisterNotify)
		return EFI_UNSUPPORTED;

	for (i = 0; i < ARRAY_SIZE(INTERFACES); i++) {
		inte = &INTERFACES[i];
		if (!inte->installed)
			continue;

		if (SearchType == ByProtocol &&
		    ew_guidcmp(&inte->protocol, Protocol))
			continue;

		if ((nb + 1) * sizeof(*Buffer) > *BufferSize)
			return EFI_BUFFER_TOO_SMALL;

		Buffer[nb++] = inte->handle;
	}

	if (nb == 0)
		return EFI_NOT_FOUND;

	*BufferSize = nb * sizeof(*Buffer);

	return EFI_SUCCESS;
}

static int dpcmp(EFI_DEVICE_PATH *p1, EFI_DEVICE_PATH *p2)
{
	int cmp;

	while (!IsDevicePathEndType(p1) && !IsDevicePathEndType(p2)) {
		if (DevicePathNodeLength(p1) < DevicePathNodeLength(p2))
			return -1;
		if (DevicePathNodeLength(p1) > DevicePathNodeLength(p2))
			return 1;
		cmp = ew_memcmp(p1, p2, DevicePathNodeLength(p1));
		if (cmp)
			return cmp;
		p1 = NextDevicePathNode(p1);
		p2 = NextDevicePathNode(p2);
	}

	return !(IsDevicePathEndType(p1) && IsDevicePathEndType(p2));
}

EFIAPI EFI_STATUS
ew_locate_device_path(EFI_GUID *Protocol,
		      EFI_DEVICE_PATH **DevicePath,
		      EFI_HANDLE *Device)
{
	EFI_STATUS ret;
	UINTN nb_handle, i;
	EFI_HANDLE *handles;
	EFI_DEVICE_PATH *path;

	if (!Protocol || !DevicePath || !*DevicePath)
		return EFI_INVALID_PARAMETER;

	ret = ew_locate_handle_buffer(ByProtocol, Protocol, NULL, &nb_handle, &handles);
	if (EFI_ERROR(ret) || nb_handle == 0)
		return EFI_NOT_FOUND;

	for (i = 0; i < nb_handle; i++) {
		ret = ew_handle_protocol(handles[i], &DevicePathProtocol, (VOID **)&path);
		if (EFI_ERROR(ret))
			continue;

		if (path == *DevicePath || !dpcmp(*DevicePath, path)) {
			*Device = handles[i];
			ew_free(handles);
			return EFI_SUCCESS;
		}
	}
	ew_free(handles);

	return EFI_UNSUPPORTED;
}

EFIAPI EFI_STATUS
ew_open_protocol(EFI_HANDLE Handle,
		 EFI_GUID *Protocol,
		 VOID **Interface,
		 EFI_HANDLE AgentHandle,
		 EFI_HANDLE ControllerHandle,
		 UINT32 Attributes)
{
	if (Handle != AgentHandle || ControllerHandle != NULL)
		return EFI_UNSUPPORTED;

	return ew_handle_protocol(Handle, Protocol, Interface);
}

EFIAPI EFI_STATUS
ew_close_protocol(EFI_HANDLE Handle,
		  EFI_GUID *Protocol,
		  EFI_HANDLE AgentHandle,
		  EFI_HANDLE ControllerHandle)
{
	return EFI_UNSUPPORTED;
}

EFIAPI EFI_STATUS
ew_protocols_per_handle(EFI_HANDLE Handle,
			EFI_GUID ***ProtocolBuffer,
			UINTN *ProtocolBufferCount)
{
	return EFI_UNSUPPORTED;
}

EFIAPI EFI_STATUS
ew_locate_handle_buffer(EFI_LOCATE_SEARCH_TYPE SearchType,
			EFI_GUID *Protocol,
			VOID *SearchKey,
			UINTN *NoHandles,
			EFI_HANDLE **Buffer)
{
	interface_t *inte;
	unsigned int i, nb, cur;
	EFI_HANDLE *buf;

	if (!Protocol || !NoHandles || !Buffer)
		return EFI_INVALID_PARAMETER;

	if (SearchType != AllHandles &&
	    SearchType != ByRegisterNotify &&
	    SearchType != ByProtocol)
		return EFI_INVALID_PARAMETER;

	if (SearchType == ByRegisterNotify)
		return EFI_UNSUPPORTED;

	for (i = 0, nb = 0; i < ARRAY_SIZE(INTERFACES); i++) {
		inte = &INTERFACES[i];
		if (!inte->installed)
			continue;

		if (SearchType == ByProtocol &&
		    !ew_guidcmp(&inte->protocol, Protocol))
			nb++;
	}

	if (nb == 0)
		return EFI_NOT_FOUND;

	buf = ew_malloc(sizeof(EFI_HANDLE) * nb);
	if (!buf)
		return EFI_OUT_OF_RESOURCES;

	for (i = 0, cur = 0; i < ARRAY_SIZE(INTERFACES); i++) {
		inte = &INTERFACES[i];
		if (!inte->installed)
			continue;

		if (SearchType == ByProtocol &&
		    ew_guidcmp(&inte->protocol, Protocol))
			continue;

		buf[cur++] = inte->handle;
	}

	*NoHandles = nb;
	*Buffer = buf;

	return EFI_SUCCESS;
}

EFIAPI EFI_STATUS
ew_locate_protocol(EFI_GUID *Protocol,
		   VOID *Registration,
		   VOID **Interface)
{
	return EFI_UNSUPPORTED;
}
