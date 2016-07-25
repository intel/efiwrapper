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

#include <unistd.h>
#include <efiwrapper.h>

#include "protocol.h"
#include "platform.h"

#include "boot_services.h"

static EFIAPI EFI_TPL
_raise_TPL(__attribute__((__unused__)) EFI_TPL NewTpl)
{
	return 0;
}

static EFIAPI VOID
_restore_TPL(__attribute__((__unused__)) EFI_TPL OldTpl)
{
}

static EFIAPI EFI_STATUS
_allocate_pages(__attribute__((__unused__)) EFI_ALLOCATE_TYPE Type,
		__attribute__((__unused__)) EFI_MEMORY_TYPE MemoryType,
		__attribute__((__unused__)) UINTN NoPages,
		__attribute__((__unused__)) EFI_PHYSICAL_ADDRESS *Memory)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_free_pages(__attribute__((__unused__)) EFI_PHYSICAL_ADDRESS Memory,
	    __attribute__((__unused__)) UINTN NoPages)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_get_memory_map(__attribute__((__unused__)) UINTN *MemoryMapSize,
		__attribute__((__unused__)) EFI_MEMORY_DESCRIPTOR *MemoryMap,
		__attribute__((__unused__)) UINTN *MapKey,
		__attribute__((__unused__)) UINTN *DescriptorSize,
		__attribute__((__unused__)) UINT32 *DescriptorVersion)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_allocate_pool(EFI_MEMORY_TYPE PoolType, UINTN Size, VOID **Buffer)
{
	void *buf;

	buf = ew_malloc(Size);
	if (!buf)
		return EFI_OUT_OF_RESOURCES;

	*Buffer = buf;
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
_free_pool(VOID *Buffer)
{
	ew_free(Buffer);
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
_create_event(__attribute__((__unused__)) UINT32 Type,
	      __attribute__((__unused__)) EFI_TPL NotifyTpl,
	      __attribute__((__unused__)) EFI_EVENT_NOTIFY NotifyFunction,
	      __attribute__((__unused__)) VOID *NotifyContext,
	      __attribute__((__unused__)) EFI_EVENT *Event)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_set_timer(__attribute__((__unused__)) EFI_EVENT Event,
	   __attribute__((__unused__)) EFI_TIMER_DELAY Type,
	   __attribute__((__unused__)) UINT64 TriggerTime)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_wait_for_event(__attribute__((__unused__)) UINTN NumberOfEvents,
		__attribute__((__unused__)) EFI_EVENT *Event,
		__attribute__((__unused__)) UINTN *Index)
{
	return EFI_UNSUPPORTED;
}


static EFIAPI EFI_STATUS
_signal_event(__attribute__((__unused__)) EFI_EVENT Event)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_close_event(__attribute__((__unused__)) EFI_EVENT Event)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_check_event(__attribute__((__unused__)) EFI_EVENT Event)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_PC_handle_protocol(__attribute__((__unused__)) EFI_HANDLE Handle,
		    __attribute__((__unused__)) EFI_GUID *Protocol,
		    __attribute__((__unused__)) VOID **Interface)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_register_protocol_notify(__attribute__((__unused__)) EFI_GUID *Protocol,
			  __attribute__((__unused__)) EFI_EVENT Event,
			  __attribute__((__unused__)) VOID **Registration)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_install_configuration_table(__attribute__((__unused__)) EFI_GUID *Guid,
			     __attribute__((__unused__)) VOID *Table)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_load_image(__attribute__((__unused__)) BOOLEAN BootPolicy,
	    __attribute__((__unused__)) EFI_HANDLE ParentImageHandle,
	    __attribute__((__unused__)) EFI_DEVICE_PATH *FilePath,
	    __attribute__((__unused__)) VOID *SourceBuffer,
	    __attribute__((__unused__)) UINTN SourceSize,
	    __attribute__((__unused__)) EFI_HANDLE *ImageHandle)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_start_image(__attribute__((__unused__)) EFI_HANDLE ImageHandle,
	     __attribute__((__unused__)) UINTN *ExitDataSize,
	     __attribute__((__unused__)) CHAR16 **ExitData)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_efi_exit(__attribute__((__unused__)) EFI_HANDLE ImageHandle,
	  __attribute__((__unused__)) EFI_STATUS ExitStatus,
	  __attribute__((__unused__)) UINTN ExitDataSize,
	  __attribute__((__unused__)) CHAR16 *ExitData)
{
	return EFI_UNSUPPORTED;
}


static EFIAPI EFI_STATUS
_unload_image(__attribute__((__unused__)) EFI_HANDLE ImageHandle)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_exit_boot_services(__attribute__((__unused__)) EFI_HANDLE ImageHandle,
		    __attribute__((__unused__)) UINTN MapKey)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_get_next_monotonic_count(__attribute__((__unused__)) UINT64 *Count)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_stall(UINTN Microseconds)
{
	error("stalling for %dus", Microseconds);
	usleep(Microseconds);
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
_set_watchdog_timer(UINTN Timeout,
		    __attribute__((__unused__)) UINT64 WatchdogCode,
		    __attribute__((__unused__)) UINTN DataSize,
		    __attribute__((__unused__)) CHAR16 *WatchdogData)
{
	return Timeout == 0 ? EFI_SUCCESS : EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_connect_controller(__attribute__((__unused__)) EFI_HANDLE ControllerHandle,
		    __attribute__((__unused__)) EFI_HANDLE *DriverImageHandle,
		    __attribute__((__unused__)) EFI_DEVICE_PATH *RemainingDevicePath,
		    __attribute__((__unused__)) BOOLEAN Recursive)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_disconnect_controller(__attribute__((__unused__)) EFI_HANDLE ControllerHandle,
		       __attribute__((__unused__)) EFI_HANDLE DriverImageHandle,
		       __attribute__((__unused__)) EFI_HANDLE ChildHandle)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_open_protocol_information(__attribute__((__unused__)) EFI_HANDLE Handle,
			   __attribute__((__unused__)) EFI_GUID *Protocol,
			   __attribute__((__unused__)) EFI_OPEN_PROTOCOL_INFORMATION_ENTRY **EntryBuffer,
			   __attribute__((__unused__)) UINTN *EntryCount)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_install_multiple_protocol_interfaces(__attribute__((__unused__)) EFI_HANDLE *Handle,
				      ...)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
_uninstall_multiple_protocol_interfaces(__attribute__((__unused__)) EFI_HANDLE Handle,
					...)
{
	return EFI_UNSUPPORTED;
}

static uint32_t crc32_tab[] = {
	0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
	0xe963a535, 0x9e6495a3,	0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
	0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
	0xf3b97148, 0x84be41de,	0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
	0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec,	0x14015c4f, 0x63066cd9,
	0xfa0f3d63, 0x8d080df5,	0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
	0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,	0x35b5a8fa, 0x42b2986c,
	0xdbbbc9d6, 0xacbcf940,	0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
	0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
	0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
	0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,	0x76dc4190, 0x01db7106,
	0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
	0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
	0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
	0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
	0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
	0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
	0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
	0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
	0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
	0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
	0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
	0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
	0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
	0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
	0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
	0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
	0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
	0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
	0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
	0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
	0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
	0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
	0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
	0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
	0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
	0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
	0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
	0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
	0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
	0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
	0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
	0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

static EFIAPI EFI_STATUS
_calculate_crc32(VOID *Data,
		 UINTN DataSize,
		 UINT32 *Crc32)
{
	UINT32 crc = 0;
	const UINT8 *p;

	p = Data;
	crc = crc ^ ~0U;

	while (DataSize--)
		crc = crc32_tab[(crc ^ *p++) & 0xFF] ^ (crc >> 8);

	*Crc32 = crc ^ ~0U;
	return EFI_SUCCESS;
}

static EFIAPI VOID
_copy_mem(VOID *Destination,
	  VOID *Source,
	  UINTN Length)
{
	ew_memcpy(Destination, Source, Length);
}

static EFIAPI VOID
_set_mem(VOID *Buffer,
	 UINTN Size,
	 UINT8 Value)
{
	ew_memset(Buffer, Value, Size);
}

static EFIAPI EFI_STATUS
_create_event_ex(__attribute__((__unused__)) UINT32 Type,
		 __attribute__((__unused__)) EFI_TPL NotifyTpl,
		 __attribute__((__unused__)) EFI_EVENT_NOTIFY NotifyFunction,
		 __attribute__((__unused__)) const VOID *NotifyContext,
		 __attribute__((__unused__)) const EFI_GUID EventGroup,
		 __attribute__((__unused__)) EFI_EVENT *Event)
{
	return EFI_UNSUPPORTED;
}

static EFI_BOOT_SERVICES boot_services_struct = {
	.Hdr = {
		.Signature = 0,
		.Revision = 0,
		.HeaderSize = 0,
		.CRC32 = 0,
		.Reserved = 0
	},

	.RaiseTPL = _raise_TPL,
	.RestoreTPL = _restore_TPL,

	.AllocatePages = _allocate_pages,
	.FreePages = _free_pages,
	.GetMemoryMap = _get_memory_map,
	.AllocatePool = _allocate_pool,
	.FreePool = _free_pool,

	.CreateEvent = _create_event,
	.SetTimer = _set_timer,
	.WaitForEvent = _wait_for_event,
	.SignalEvent = _signal_event,
	.CloseEvent = _close_event,
	.CheckEvent = _check_event,

	.PCHandleProtocol = _PC_handle_protocol,
	.RegisterProtocolNotify = _register_protocol_notify,
	.InstallConfigurationTable = _install_configuration_table,

	.LoadImage = _load_image,
	.StartImage = _start_image,
	.Exit = _efi_exit,
	.UnloadImage = _unload_image,
	.ExitBootServices = _exit_boot_services,

	.GetNextMonotonicCount = _get_next_monotonic_count,
	.Stall = _stall,
	.SetWatchdogTimer = _set_watchdog_timer,

	.ConnectController = _connect_controller,
	.DisconnectController = _disconnect_controller,

	.OpenProtocolInformation = _open_protocol_information,

	.InstallMultipleProtocolInterfaces = _install_multiple_protocol_interfaces,
	.UninstallMultipleProtocolInterfaces = _uninstall_multiple_protocol_interfaces,

	.CalculateCrc32 = _calculate_crc32,

	.CopyMem = _copy_mem,
	.SetMem = _set_mem,
	.CreateEventEx = _create_event_ex
};

EFI_STATUS linux_boot_services(EFI_BOOT_SERVICES **bs_p)
{
	*bs_p = &boot_services_struct;

	(*bs_p)->InstallProtocolInterface = ew_install_protocol_interface;
	(*bs_p)->ReinstallProtocolInterface = ew_reinstall_protocol_interface;
	(*bs_p)->UninstallProtocolInterface = ew_uninstall_protocol_interface;
	(*bs_p)->HandleProtocol = ew_handle_protocol;
	(*bs_p)->LocateHandle = ew_locate_handle;
	(*bs_p)->LocateDevicePath = ew_locate_device_path;
	(*bs_p)->ProtocolsPerHandle = ew_protocols_per_handle;
	(*bs_p)->LocateHandleBuffer = ew_locate_handle_buffer;
	(*bs_p)->LocateProtocol = ew_locate_protocol;
	(*bs_p)->OpenProtocol = ew_open_protocol;
	(*bs_p)->CloseProtocol = ew_close_protocol;

	return EFI_SUCCESS;
}
