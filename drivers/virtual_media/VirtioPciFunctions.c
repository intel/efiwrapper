/** @file

  This driver produces Virtio Device Protocol instances for Virtio PCI devices.

  Copyright (C) 2012, Red Hat, Inc.
  Copyright (c) 2012, Intel Corporation. All rights reserved.<BR>
  Copyright (C) 2013, ARM Ltd.
  Copyright (C) 2017, AMD Inc, All rights reserved.<BR>

  This program and the accompanying materials are licensed and made available
  under the terms and conditions of the BSD License which accompanies this
  distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS, WITHOUT
  WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/
#include <kconfig.h>
#include <libpayload-config.h>
#include <libpayload.h>
#include <ewlib.h>
#include <efilink.h>
#include <efidef.h>

#include "VirtioDeviceCommon.h"
#include "VirtioPciDevice.h"

/**
  Allocates one or more 4KB pages of type EfiBootServicesData.

  Allocates the number of 4KB pages of type EfiBootServicesData and returns a pointer to the
  allocated buffer.  The buffer returned is aligned on a 4KB boundary.  If Pages is 0, then NULL
  is returned.  If there is not enough memory remaining to satisfy the request, then NULL is
  returned.

  @param  Pages                 The number of 4 KB pages to allocate.

  @return A pointer to the allocated buffer or NULL if allocation fails.

**/
static VOID *
EFIAPI
AllocatePages (
	IN UINTN  Pages
	)
{
	return memalign (EFI_PAGE_SIZE, Pages << EFI_PAGE_SHIFT);
}


/**
  Frees one or more 4KB pages that were previously allocated with one of the page allocation
  functions in the Memory Allocation Library.

  Frees the number of 4KB pages specified by Pages from the buffer specified by Buffer.  Buffer
  must have been allocated on a previous call to the page allocation services of the Memory
  Allocation Library.  If it is not possible to free allocated pages, then this function will
  perform no actions.

  If Buffer was not allocated with a page allocation function in the Memory Allocation Library,
  then ASSERT().
  If Pages is zero, then ASSERT().

  @param  Buffer                The pointer to the buffer of pages to free.
  @param  Pages                 The number of 4 KB pages to free.

**/
VOID
EFIAPI
FreePages (
	IN VOID   *Buffer,
	IN __attribute__((unused)) UINTN  Pages
	)
{
	free(Buffer);
}


/**

  Read a word from Region 0 of the device specified by VirtIo Device protocol.

  The function implements the ReadDevice protocol member of
  VIRTIO_DEVICE_PROTOCOL.

  @param[in] This         VirtIo Device protocol.

  @param[in] FieldOffset  Source offset.

  @param[in] FieldSize    Source field size, must be in { 1, 2, 4, 8 }.

  @param[in] BufferSize   Number of bytes available in the target buffer. Must
                          equal FieldSize.

  @param[out] Buffer      Target buffer.


  @return  Status code returned by PciIo->Io.Read().

**/
EFI_STATUS
EFIAPI
VirtioPciDeviceRead (
	IN  VIRTIO_DEVICE_PROTOCOL    *This,
	IN  UINTN                     FieldOffset,
	IN  UINTN                     FieldSize,
	IN  UINTN                     BufferSize,
	OUT VOID                      *Buffer
	)
{
	VIRTIO_PCI_DEVICE         *Dev;

	Dev = VIRTIO_PCI_DEVICE_FROM_VIRTIO_DEVICE (This);

	return VirtioPciIoRead (Dev,
		Dev->DeviceSpecificConfigurationOffset + FieldOffset,
		FieldSize, BufferSize, Buffer);
}

/**

  Write a word into Region 0 of the device specified by VirtIo Device protocol.

  @param[in] This         VirtIo Device protocol.

  @param[in] FieldOffset  Destination offset.

  @param[in] FieldSize    Destination field size, must be in { 1, 2, 4, 8 }.

  @param[in] Value        Little endian value to write, converted to UINT64.
                          The least significant FieldSize bytes will be used.


  @return  Status code returned by PciIo->Io.Write().

**/
EFI_STATUS
EFIAPI
VirtioPciDeviceWrite (
	IN VIRTIO_DEVICE_PROTOCOL *This,
	IN UINTN                  FieldOffset,
	IN UINTN                  FieldSize,
	IN UINT64                 Value
	)
{
  VIRTIO_PCI_DEVICE         *Dev;

  Dev = VIRTIO_PCI_DEVICE_FROM_VIRTIO_DEVICE (This);

  return VirtioPciIoWrite (Dev,
	Dev->DeviceSpecificConfigurationOffset + FieldOffset, FieldSize, Value);
}

EFI_STATUS
EFIAPI
VirtioPciGetDeviceFeatures (
	IN VIRTIO_DEVICE_PROTOCOL *This,
	OUT UINT64                *DeviceFeatures
	)
{
	VIRTIO_PCI_DEVICE         *Dev;
	EFI_STATUS                Status;
	UINT32                    Features32;

	if (DeviceFeatures == NULL) {
		return EFI_INVALID_PARAMETER;
	}

	Dev = VIRTIO_PCI_DEVICE_FROM_VIRTIO_DEVICE (This);

	Status = VirtioPciIoRead (Dev, VIRTIO_PCI_OFFSET_DEVICE_FEATURES,
		sizeof (UINT32), sizeof (UINT32), &Features32);
	if (!EFI_ERROR (Status)) {
		*DeviceFeatures = Features32;
	}
	return Status;
}

EFI_STATUS
EFIAPI
VirtioPciGetQueueSize (
	IN  VIRTIO_DEVICE_PROTOCOL  *This,
	OUT UINT16                  *QueueNumMax
	)
{
	VIRTIO_PCI_DEVICE         *Dev;

	if (QueueNumMax == NULL) {
		return EFI_INVALID_PARAMETER;
	}

	Dev = VIRTIO_PCI_DEVICE_FROM_VIRTIO_DEVICE (This);

	return VirtioPciIoRead (Dev, VIRTIO_PCI_OFFSET_QUEUE_SIZE, sizeof (UINT16),
		sizeof (UINT16), QueueNumMax);
}

EFI_STATUS
EFIAPI
VirtioPciGetDeviceStatus (
	IN  VIRTIO_DEVICE_PROTOCOL  *This,
	OUT UINT8                   *DeviceStatus
	)
{
	VIRTIO_PCI_DEVICE         *Dev;

	if (DeviceStatus == NULL) {
		return EFI_INVALID_PARAMETER;
	}

	Dev = VIRTIO_PCI_DEVICE_FROM_VIRTIO_DEVICE (This);

	return VirtioPciIoRead (Dev, VIRTIO_PCI_OFFSET_QUEUE_DEVICE_STATUS,
		sizeof (UINT8), sizeof (UINT8), DeviceStatus);
}

EFI_STATUS
EFIAPI
VirtioPciSetGuestFeatures (
	IN VIRTIO_DEVICE_PROTOCOL  *This,
	IN UINT64                   Features
	)
{
	VIRTIO_PCI_DEVICE *Dev;

	Dev = VIRTIO_PCI_DEVICE_FROM_VIRTIO_DEVICE (This);

	if (Features > MAX_UINT32) {
		return EFI_UNSUPPORTED;
	}
	return VirtioPciIoWrite (Dev, VIRTIO_PCI_OFFSET_GUEST_FEATURES,
		sizeof (UINT32), Features);
}

EFI_STATUS
EFIAPI
VirtioPciSetQueueAddress (
	IN VIRTIO_DEVICE_PROTOCOL  *This,
	IN VRING                   *Ring,
	IN  __attribute__((unused)) UINT64                  RingBaseShift
	)
{
	VIRTIO_PCI_DEVICE *Dev;

	ASSERT (RingBaseShift == 0);

	Dev = VIRTIO_PCI_DEVICE_FROM_VIRTIO_DEVICE (This);

	return VirtioPciIoWrite (Dev, VIRTIO_PCI_OFFSET_QUEUE_ADDRESS, sizeof (UINT32),
		(UINT32)((UINTN)Ring->Base >> EFI_PAGE_SHIFT));
}

EFI_STATUS
EFIAPI
VirtioPciSetQueueSel (
	IN  VIRTIO_DEVICE_PROTOCOL    *This,
	IN  UINT16                    Sel
	)
{
	VIRTIO_PCI_DEVICE *Dev;

	Dev = VIRTIO_PCI_DEVICE_FROM_VIRTIO_DEVICE (This);

	return VirtioPciIoWrite (Dev, VIRTIO_PCI_OFFSET_QUEUE_SELECT, sizeof (UINT16),
		Sel);
}

EFI_STATUS
EFIAPI
VirtioPciSetQueueAlignment (
	IN  __attribute__((unused)) VIRTIO_DEVICE_PROTOCOL *This,
	IN  __attribute__((unused)) UINT32                  Alignment
	)
{
	return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
VirtioPciSetPageSize (
	IN  __attribute__((unused)) VIRTIO_DEVICE_PROTOCOL *This,
	IN  UINT32                  PageSize
	)
{
	return (PageSize == EFI_PAGE_SIZE) ? EFI_SUCCESS : EFI_UNSUPPORTED;
}

EFI_STATUS
EFIAPI
VirtioPciSetQueueNotify (
	IN  VIRTIO_DEVICE_PROTOCOL *This,
	IN  UINT16                 Index
	)
{
	VIRTIO_PCI_DEVICE *Dev;

	Dev = VIRTIO_PCI_DEVICE_FROM_VIRTIO_DEVICE (This);

	return VirtioPciIoWrite (Dev, VIRTIO_PCI_OFFSET_QUEUE_NOTIFY, sizeof (UINT16),
		Index);
}

EFI_STATUS
EFIAPI
VirtioPciSetQueueSize (
	IN  __attribute__((unused)) VIRTIO_DEVICE_PROTOCOL *This,
	IN  __attribute__((unused)) UINT16                 Size
	)
{
	//
	// This function is only applicable in Virtio-MMIO.
	// (The QueueSize field is read-only in Virtio proper (PCI))
	//
	return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
VirtioPciSetDeviceStatus (
	IN  VIRTIO_DEVICE_PROTOCOL *This,
	IN  UINT8                  DeviceStatus
	)
{
	VIRTIO_PCI_DEVICE *Dev;

	Dev = VIRTIO_PCI_DEVICE_FROM_VIRTIO_DEVICE (This);

	return VirtioPciIoWrite (Dev, VIRTIO_PCI_OFFSET_QUEUE_DEVICE_STATUS,
		sizeof (UINT8), DeviceStatus);
}

EFI_STATUS
EFIAPI
VirtioPciAllocateSharedPages (
	IN  __attribute__((unused)) VIRTIO_DEVICE_PROTOCOL  *This,
	IN  UINTN                   NumPages,
	OUT VOID                    **HostAddress
	)
{
	VOID        *Buffer;

	Buffer = AllocatePages (NumPages);
	if (Buffer == NULL) {
		return EFI_OUT_OF_RESOURCES;
	}

	*HostAddress = Buffer;
	return EFI_SUCCESS;
}

VOID
EFIAPI
VirtioPciFreeSharedPages (
	IN  __attribute__((unused)) VIRTIO_DEVICE_PROTOCOL  *This,
	IN  UINTN                   NumPages,
	IN  VOID                    *HostAddress
	)
{
	FreePages (HostAddress, NumPages);
}

EFI_STATUS
EFIAPI
VirtioPciMapSharedBuffer (
	IN      __attribute__((unused)) VIRTIO_DEVICE_PROTOCOL  *This,
	IN      __attribute__((unused)) VIRTIO_MAP_OPERATION    Operation,
	IN      VOID                    *HostAddress,
	IN OUT  __attribute__((unused)) UINTN                   *NumberOfBytes,
	OUT     EFI_PHYSICAL_ADDRESS    *DeviceAddress,
	OUT     VOID                    **Mapping
	)
{
	*DeviceAddress = (EFI_PHYSICAL_ADDRESS) (UINTN) HostAddress;
	*Mapping = NULL;

	return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
VirtioPciUnmapSharedBuffer (
	IN __attribute__((unused)) VIRTIO_DEVICE_PROTOCOL    *This,
	IN __attribute__((unused)) VOID                      *Mapping
	)
{
	return EFI_SUCCESS;
}
