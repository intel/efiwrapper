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

#include "bs.h"
#include "lib.h"
#include "protocol.h"

static EFIAPI EFI_TPL
bs_raise_TPL(__attribute__((__unused__)) EFI_TPL NewTpl)
{
	return 0;
}

static EFIAPI VOID
bs_restore_TPL(__attribute__((__unused__)) EFI_TPL OldTpl)
{
}

static EFIAPI EFI_STATUS
bs_allocate_pages(__attribute__((__unused__)) EFI_ALLOCATE_TYPE Type,
		  __attribute__((__unused__)) EFI_MEMORY_TYPE MemoryType,
		  __attribute__((__unused__)) UINTN NoPages,
		  __attribute__((__unused__)) EFI_PHYSICAL_ADDRESS *Memory)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_free_pages(__attribute__((__unused__)) EFI_PHYSICAL_ADDRESS Memory,
	      __attribute__((__unused__)) UINTN NoPages)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_get_memory_map(__attribute__((__unused__)) UINTN *MemoryMapSize,
		  __attribute__((__unused__)) EFI_MEMORY_DESCRIPTOR *MemoryMap,
		  __attribute__((__unused__)) UINTN *MapKey,
		  __attribute__((__unused__)) UINTN *DescriptorSize,
		  __attribute__((__unused__)) UINT32 *DescriptorVersion)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_allocate_pool(__attribute__((__unused__)) EFI_MEMORY_TYPE PoolType,
		 UINTN Size, VOID **Buffer)
{
	void *buf;

	buf = malloc(Size);
	if (!buf)
		return EFI_OUT_OF_RESOURCES;

	*Buffer = buf;

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
bs_free_pool(VOID *Buffer)
{
	free(Buffer);
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
bs_create_event(__attribute__((__unused__)) UINT32 Type,
		__attribute__((__unused__)) EFI_TPL NotifyTpl,
		__attribute__((__unused__)) EFI_EVENT_NOTIFY NotifyFunction,
		__attribute__((__unused__)) VOID *NotifyContext,
		__attribute__((__unused__)) EFI_EVENT *Event)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_set_timer(__attribute__((__unused__)) EFI_EVENT Event,
	     __attribute__((__unused__)) EFI_TIMER_DELAY Type,
	     __attribute__((__unused__)) UINT64 TriggerTime)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_wait_for_event(__attribute__((__unused__)) UINTN NumberOfEvents,
		  __attribute__((__unused__)) EFI_EVENT *Event,
		  __attribute__((__unused__)) UINTN *Index)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_signal_event(__attribute__((__unused__)) EFI_EVENT Event)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_close_event(__attribute__((__unused__)) EFI_EVENT Event)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_check_event(__attribute__((__unused__)) EFI_EVENT Event)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_PC_handle_protocol(__attribute__((__unused__)) EFI_HANDLE Handle,
		      __attribute__((__unused__)) EFI_GUID *Protocol,
		      __attribute__((__unused__)) VOID **Interface)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_register_protocol_notify(__attribute__((__unused__)) EFI_GUID *Protocol,
			    __attribute__((__unused__)) EFI_EVENT Event,
			    __attribute__((__unused__)) VOID **Registration)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_install_configuration_table(__attribute__((__unused__)) EFI_GUID *Guid,
			       __attribute__((__unused__)) VOID *Table)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_load_image(__attribute__((__unused__)) BOOLEAN BootPolicy,
	      __attribute__((__unused__)) EFI_HANDLE ParentImageHandle,
	      __attribute__((__unused__)) EFI_DEVICE_PATH *FilePath,
	      __attribute__((__unused__)) VOID *SourceBuffer,
	      __attribute__((__unused__)) UINTN SourceSize,
	      __attribute__((__unused__)) EFI_HANDLE *ImageHandle)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_start_image(__attribute__((__unused__)) EFI_HANDLE ImageHandle,
	       __attribute__((__unused__)) UINTN *ExitDataSize,
	       __attribute__((__unused__)) CHAR16 **ExitData)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_efi_exit(__attribute__((__unused__)) EFI_HANDLE ImageHandle,
	    __attribute__((__unused__)) EFI_STATUS ExitStatus,
	    __attribute__((__unused__)) UINTN ExitDataSize,
	    __attribute__((__unused__)) CHAR16 *ExitData)
{
	return EFI_UNSUPPORTED;
}


static EFIAPI EFI_STATUS
bs_unload_image(__attribute__((__unused__)) EFI_HANDLE ImageHandle)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_exit_boot_services(__attribute__((__unused__)) EFI_HANDLE ImageHandle,
		      __attribute__((__unused__)) UINTN MapKey)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_get_next_monotonic_count(__attribute__((__unused__)) UINT64 *Count)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_stall(__attribute__((__unused__)) UINTN Microseconds)
{
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
bs_set_watchdog_timer(UINTN Timeout,
		      __attribute__((__unused__)) UINT64 WatchdogCode,
		      __attribute__((__unused__)) UINTN DataSize,
		      __attribute__((__unused__)) CHAR16 *WatchdogData)
{
	return Timeout == 0 ? EFI_SUCCESS : EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_connect_controller(__attribute__((__unused__)) EFI_HANDLE ControllerHandle,
		      __attribute__((__unused__)) EFI_HANDLE *DriverImageHandle,
		      __attribute__((__unused__)) EFI_DEVICE_PATH *RemainingDevicePath,
		      __attribute__((__unused__)) BOOLEAN Recursive)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_disconnect_controller(__attribute__((__unused__)) EFI_HANDLE ControllerHandle,
			 __attribute__((__unused__)) EFI_HANDLE DriverImageHandle,
			 __attribute__((__unused__)) EFI_HANDLE ChildHandle)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_open_protocol_information(__attribute__((__unused__)) EFI_HANDLE Handle,
			     __attribute__((__unused__)) EFI_GUID *Protocol,
			     __attribute__((__unused__)) EFI_OPEN_PROTOCOL_INFORMATION_ENTRY **EntryBuffer,
			     __attribute__((__unused__)) UINTN *EntryCount)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_install_multiple_protocol_interfaces(__attribute__((__unused__)) EFI_HANDLE *Handle,
					...)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_uninstall_multiple_protocol_interfaces(__attribute__((__unused__)) EFI_HANDLE Handle,
					  ...)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
bs_calculate_crc32(VOID *Data,
		   UINTN DataSize,
		   UINT32 *Crc32)
{
	return crc32(Data, DataSize, Crc32);
}

static EFIAPI VOID
bs_copy_mem(VOID *Destination,
	    VOID *Source,
	    UINTN Length)
{
	memcpy(Destination, Source, Length);
}

static EFIAPI VOID
bs_set_mem(VOID *Buffer,
	   UINTN Size,
	   UINT8 Value)
{
	memset(Buffer, Value, Size);
}

static EFIAPI EFI_STATUS
bs_create_event_ex(__attribute__((__unused__)) UINT32 Type,
		   __attribute__((__unused__)) EFI_TPL NotifyTpl,
		   __attribute__((__unused__)) EFI_EVENT_NOTIFY NotifyFunction,
		   __attribute__((__unused__)) const VOID *NotifyContext,
		   __attribute__((__unused__)) const EFI_GUID EventGroup,
		   __attribute__((__unused__)) EFI_EVENT *Event)
{
	return EFI_UNSUPPORTED;
}

static EFI_BOOT_SERVICES boot_services_default = {
	.Hdr = {
		.Signature = EFI_BOOT_SERVICES_SIGNATURE,
		.Revision = EFI_BOOT_SERVICES_REVISION,
		.HeaderSize = sizeof(EFI_TABLE_HEADER)
	},

	.RaiseTPL = bs_raise_TPL,
	.RestoreTPL = bs_restore_TPL,
	.AllocatePages = bs_allocate_pages,
	.FreePages = bs_free_pages,
	.GetMemoryMap = bs_get_memory_map,
	.AllocatePool = bs_allocate_pool,
	.FreePool = bs_free_pool,
	.CreateEvent = bs_create_event,
	.SetTimer = bs_set_timer,
	.WaitForEvent = bs_wait_for_event,
	.SignalEvent = bs_signal_event,
	.CloseEvent = bs_close_event,
	.CheckEvent = bs_check_event,
	.PCHandleProtocol = bs_PC_handle_protocol,
	.RegisterProtocolNotify = bs_register_protocol_notify,
	.InstallConfigurationTable = bs_install_configuration_table,
	.LoadImage = bs_load_image,
	.StartImage = bs_start_image,
	.Exit = bs_efi_exit,
	.UnloadImage = bs_unload_image,
	.ExitBootServices = bs_exit_boot_services,
	.GetNextMonotonicCount = bs_get_next_monotonic_count,
	.Stall = bs_stall,
	.SetWatchdogTimer = bs_set_watchdog_timer,
	.ConnectController = bs_connect_controller,
	.DisconnectController = bs_disconnect_controller,
	.OpenProtocolInformation = bs_open_protocol_information,
	.InstallMultipleProtocolInterfaces = bs_install_multiple_protocol_interfaces,
	.UninstallMultipleProtocolInterfaces = bs_uninstall_multiple_protocol_interfaces,
	.CalculateCrc32 = bs_calculate_crc32,
	.CopyMem = bs_copy_mem,
	.SetMem = bs_set_mem,
	.CreateEventEx = bs_create_event_ex
};

EFI_STATUS bs_init(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;
	EFI_BOOT_SERVICES *bs;

	if (!st || !st->BootServices)
		return EFI_INVALID_PARAMETER;

	bs = st->BootServices;
	memcpy(bs, &boot_services_default, sizeof(*bs));

	ret = protocol_init_bs(bs);
	if (EFI_ERROR(ret))
		return ret;

	return crc32((void *)bs, sizeof(*bs), &bs->Hdr.CRC32);
}
