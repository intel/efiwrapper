/** @file

  Internal definitions for the VirtIo PCI Device driver

  Copyright (C) 2013, ARM Ltd
  Copyright (c) 2017, AMD Inc, All rights reserved.<BR>

  This program and the accompanying materials are licensed and made available
  under the terms and conditions of the BSD License which accompanies this
  distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS, WITHOUT
  WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef _VIRTIO_PCI_DEVICE_LIB_H_
#define _VIRTIO_PCI_DEVICE_LIB_H_

#include "VirtioDevice.h"
#include "Virtio.h"

#define VIRTIO_PCI_DEVICE_SIGNATURE   SIGNATURE_32 ('V', 'P', 'C', 'I')

typedef struct {
	UINT32                             Signature;
	VIRTIO_DEVICE_PROTOCOL VirtioDevice;
	UINT64                             OriginalPciAttributes;
	UINT32                             DeviceSpecificConfigurationOffset;
} VIRTIO_PCI_DEVICE;

#define VIRTIO_PCI_DEVICE_FROM_VIRTIO_DEVICE(Device) \
	CR (Device, VIRTIO_PCI_DEVICE, VirtioDevice, VIRTIO_PCI_DEVICE_SIGNATURE)

EFI_STATUS
EFIAPI
VirtioPciInit (
	IN OUT VIRTIO_PCI_DEVICE *Device,
	IN UINTN PciBase
	);
      
VOID
EFIAPI
VirtioPciUninit (
	IN OUT VIRTIO_PCI_DEVICE *Device
	);

#endif // _VIRTIO_PCI_DEVICE_DXE_H_
