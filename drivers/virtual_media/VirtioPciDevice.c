/** @file

  This driver produces Virtio Device Protocol instances for Virtio PCI devices.

  Copyright (C) 2012, Red Hat, Inc.
  Copyright (c) 2012 - 2016, Intel Corporation. All rights reserved.<BR>
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
#include <efilib.h>
#include <pci.h>

#include "VirtioDeviceCommon.h"
#include "VirtioPciDevice.h"

static VIRTIO_DEVICE_PROTOCOL mDeviceProtocolTemplate = {
	0,				// Revision
	0,				// SubSystemDeviceId
	VirtioPciGetDeviceFeatures,	// GetDeviceFeatures
	VirtioPciSetGuestFeatures,	// SetGuestFeatures
	VirtioPciSetQueueAddress,	// SetQueueAddress
	VirtioPciSetQueueSel,		// SetQueueSel
	VirtioPciSetQueueNotify,	// SetQueueNotify
	VirtioPciSetQueueAlignment,	// SetQueueAlignment
	VirtioPciSetPageSize,		// SetPageSize
	VirtioPciGetQueueSize,		// GetQueueNumMax
	VirtioPciSetQueueSize,		// SetQueueNum
	VirtioPciGetDeviceStatus,	// GetDeviceStatus
	VirtioPciSetDeviceStatus,	// SetDeviceStatus
	VirtioPciDeviceWrite,		// WriteDevice
	VirtioPciDeviceRead,		// ReadDevice
	VirtioPciAllocateSharedPages,	// AllocateSharedPages
	VirtioPciFreeSharedPages,	// FreeSharedPages
	VirtioPciMapSharedBuffer,	// MapSharedBuffer
	VirtioPciUnmapSharedBuffer,	// UnmapSharedBuffer
};

/**

  Read a word from Region 0 of the device specified by PciIo.

  Region 0 must be an iomem region. This is an internal function for the PCI
  implementation of the protocol.

  @param[in] Dev          Virtio PCI device.

  @param[in] FieldOffset  Source offset.

  @param[in] FieldSize    Source field size, must be in { 1, 2, 4, 8 }.

  @param[in] BufferSize   Number of bytes available in the target buffer. Must
                          equal FieldSize.

  @param[out] Buffer      Target buffer.


  @return  Status code returned by PciIo->Io.Read().

**/
EFI_STATUS
EFIAPI
VirtioPciIoRead (
	IN  VIRTIO_PCI_DEVICE         *Dev,
	IN  UINTN                     FieldOffset,
	IN  UINTN                     FieldSize,
	IN  __attribute__((unused)) UINTN                     BufferSize,
	OUT VOID                      *Buffer
	)
{
	ASSERT (FieldSize == BufferSize);

	switch (FieldSize) {
	case 1:
		//*((u8*)Buffer) = inb(mPciBase + FieldOffset);
		__asm__ __volatile__("inb %w1, %b0" : "=a"(*((UINT8*)Buffer)) : "Nd"((UINT16)(Dev->mPciBase + FieldOffset)));
		//DEBUG ((DEBUG_INFO, "VirtioPciIoRead value8 = 0x%x\n", *((UINT8*)Buffer)));
		break;

	case 2:
		//*((u16*)Buffer) = inw(mPciBase + FieldOffset);
		__asm__ __volatile__("inw %w1, %w0" : "=a"(*((UINT16*)Buffer)) : "Nd"((UINT16)(Dev->mPciBase + FieldOffset)));
		//DEBUG ((DEBUG_INFO, "VirtioPciIoRead value16 = 0x%x\n", *((UINT16*)Buffer)));
		break;

	case 8:
	//
	// The 64bit PCI I/O is broken down into two 32bit reads to prevent
	// any alignment or width issues.
	// The UEFI spec says under EFI_PCI_IO_PROTOCOL.Io.Write():
	//
	// The I/O operations are carried out exactly as requested. The caller
	// is responsible for any alignment and I/O width issues which the
	// bus, device, platform, or type of I/O might require. For example on
	// some platforms, width requests of EfiPciIoWidthUint64 do not work.
	//
		//*((UINT32*)Buffer)= inl(mPciBase + FieldOffset);
		__asm__ __volatile__("inl %w1, %0" : "=a"(*((UINT32*)Buffer)) : "Nd"((UINT16)(Dev->mPciBase + FieldOffset)));
		//*((UINT32*)Buffer + 1)= inl(mPciBase + FieldOffset + sizeof (UINT32));
		__asm__ __volatile__("inl %w1, %0" : "=a"(*((UINT32*)Buffer + 1)) : "Nd"((UINT16)(Dev->mPciBase + FieldOffset + sizeof (UINT32))));
		//DEBUG ((DEBUG_INFO, "VirtioPciIoRead value64_1 = 0x%x\n", *((UINT32*)Buffer)));
		//DEBUG ((DEBUG_INFO, "VirtioPciIoRead value64_2 = 0x%x\n", *((UINT32*)Buffer + 1)));
		break;
	//
	// fall through
	//
	case 4:
		//*((UINT32*)Buffer)= inl(mPciBase + FieldOffset);
		__asm__ __volatile__("inl %w1, %0" : "=a"(*((UINT32*)Buffer)) : "Nd"((UINT16)(Dev->mPciBase + FieldOffset)));
		//DEBUG ((DEBUG_INFO, "VirtioPciIoRead value32 = 0x%x\n", *((UINT32*)Buffer)));
		break;

	default:
		ASSERT (FALSE);
		return EFI_INVALID_PARAMETER;
	}

	return EFI_SUCCESS;
}

/**

  Write a word into Region 0 of the device specified by PciIo.

  Region 0 must be an iomem region. This is an internal function for the PCI
  implementation of the protocol.

  @param[in] Dev          Virtio PCI device.

  @param[in] FieldOffset  Destination offset.

  @param[in] FieldSize    Destination field size, must be in { 1, 2, 4, 8 }.

  @param[in] Value        Little endian value to write, converted to UINT64.
                          The least significant FieldSize bytes will be used.


  @return  Status code returned by PciIo->Io.Write().

**/
EFI_STATUS
EFIAPI
VirtioPciIoWrite (
	IN VIRTIO_PCI_DEVICE          *Dev,
	IN UINTN                      FieldOffset,
	IN UINTN                      FieldSize,
	IN UINT64                     Value
	)
{
	//DEBUG ((DEBUG_INFO, "VirtioPciIoWrite(Base 0x%x, Offset 0x%x, Size 0x%x, Value 0x%x)\n", 
	//(UINT32)mPciBase, (UINT32)FieldOffset, (UINT32)FieldSize, (UINT32)Value));

	switch (FieldSize) {
	case 1:
		//outb (mPciBase + FieldOffset, (UINT8)Value);
		 __asm__ __volatile__("outb %b0, %w1" : : "a"((UINT8)Value), "Nd"((UINT16)(Dev->mPciBase + FieldOffset)));
		break;

	case 2:
		//outw (mPciBase + FieldOffset, (UINT16)Value);
		__asm__ __volatile__("outw %w0, %w1" : : "a"((UINT16)Value), "Nd"((UINT16)(Dev->mPciBase + FieldOffset)));
		break;

	case 8:
		//
		// The 64bit PCI I/O is broken down into two 32bit writes to prevent
		// any alignment or width issues.
		// The UEFI spec says under EFI_PCI_IO_PROTOCOL.Io.Write():
		//
		// The I/O operations are carried out exactly as requested. The caller
		// is responsible for any alignment and I/O width issues which the
		// bus, device, platform, or type of I/O might require. For example on
		// some platforms, width requests of EfiPciIoWidthUint64 do not work
		//
		//outl (mPciBase + FieldOffset, (UINT32)Value);
		__asm__ __volatile__("outl %0, %w1" : : "a"((UINT32)Value), "Nd"((UINT16)(Dev->mPciBase + FieldOffset)));
		//outl (mPciBase + FieldOffset + sizeof (UINT32), (UINT32)(Value >> 32));
		__asm__ __volatile__("outl %0, %w1" : : "a"((UINT32)(Value >> 32)), "Nd"((UINT16)(Dev->mPciBase + FieldOffset + sizeof (UINT32))));
		break;

	//
	// fall through
	//
	case 4:
		//outl (mPciBase + FieldOffset, (UINT32)Value);
		__asm__ __volatile__("outl %0, %w1" : : "a"((UINT32)Value), "Nd"((UINT16)(Dev->mPciBase + FieldOffset)));
		break;

	default:
		ASSERT (FALSE);
		return EFI_INVALID_PARAMETER;
	}

	return EFI_SUCCESS;
}


static UINT8 PciCheckCap(UINTN base, UINT8 id, UINT8 offset)
{
	int i;
	UINT16 status = pci_read_config8(base, 0x06); //PCI_STATUS

	if (!(status & 0x10)) //PCI_STATUS_CAP_LIST
		return 0;

	if (offset == 0) {
		/* find first */
		offset = pci_read_config8(base, 0x34); //PCI_CAPABILITY_LIST
	} else {
		/* find next */
		offset = pci_read_config8(base, offset + 1); //PCI_CAP_LIST_NEXT
	}
	for (i = 0; offset && i <= 0xff; i++) {
		if (pci_read_config8(base, offset) == id)
		    return offset;
		offset = pci_read_config8(base, offset + 1); //PCI_CAP_LIST_NEXT
	}

	return 0;
}


static VOID PciMaskwConfig(UINTN base, UINT32 reg, UINT16 off, UINT16 on)
{
	UINT16 val = pci_read_config16(base, reg);
	val = (val & ~off) | on;
	pci_write_config16(base, reg, val);
}


// Verify an IO bar and return it to the caller
static UINT16 PciEnIo(UINTN base, UINT32 reg)
{
	UINT32 value = pci_read_config32(base, reg);
	if (!(value & 0x1)) {  //PCI_BASE_ADDRESS_SPACE_IO
		return 0;
	}
	value &= (~0x03UL);  //PCI_BASE_ADDRESS_IO_MASK	(~0x03UL)
	if (value == 0 || value > 0xffff) {
		return 0;
	}
	PciMaskwConfig(base, 0x04, 0, 0x1); //PCI_COMMAND, PCI_COMMAND_IO
	return value;
}


/**

  Initialize the VirtIo PCI Device

  @param[in, out] Dev      The driver instance to configure. The caller is
                           responsible for Device->PciIo's validity (ie. working IO
                           access to the underlying virtio-pci device).

  @retval EFI_SUCCESS      Setup complete.

  @retval EFI_UNSUPPORTED  The underlying IO device doesn't support the
                           provided address offset and read size.

  @return                  Error codes from PciIo->Pci.Read().

**/
EFI_STATUS
EFIAPI
VirtioPciInit (
	IN OUT  VIRTIO_PCI_DEVICE *Device,  IN UINTN PciBase
	)
{
	PCI_TYPE00            Pci;
	UINT32                   i;
	UINT8                    cap;

	ASSERT (Device != NULL);

	for (i =0; i<sizeof(Pci); i++) {
		*((UINT8*)&Pci + i) = pci_read_config8(PciBase, i);
	}

	//DEBUG ((DEBUG_INFO, "Pci configure space\n"));
	//DEBUG ((DEBUG_INFO, "VendorId = 0x%x\n", Pci.Hdr.VendorId));
	//DEBUG ((DEBUG_INFO, "DeviceId = 0x%x\n", Pci.Hdr.DeviceId));
	//DEBUG ((DEBUG_INFO, "Status = 0x%x\n", Pci.Hdr.Status));
	//DEBUG ((DEBUG_INFO, "Bar[0] = 0x%x\n", Pci.Device.Bar[0]));
	//DEBUG ((DEBUG_INFO, "Bar[1] = 0x%x\n", Pci.Device.Bar[1]));
	//DEBUG ((DEBUG_INFO, "caplist = 0x%x\n", *((UINT8*)Pci.Device.Reserved)));

	cap = PciCheckCap(PciBase, 0x09, 0); //PCI_CAP_ID_VNDR
	if(cap !=0) {
		DEBUG ((DEBUG_INFO, "PCI_CAP_ID_VNDR = 0x%x\n", cap));
		return EFI_UNSUPPORTED;
	}

	Device->mPciBase =PciEnIo(PciBase, 0x10); //PCI_BASE_ADDRESS_0
	DEBUG ((DEBUG_INFO, "ioaddr = 0x%x\n", (UINT32)Device->mPciBase));
	if (!Device->mPciBase)
		return EFI_UNSUPPORTED;

	CopyMem (&Device->VirtioDevice, &mDeviceProtocolTemplate,
	sizeof (VIRTIO_DEVICE_PROTOCOL));

	//
	// Initialize the protocol interface attributes
	//
	Device->VirtioDevice.Revision = VIRTIO_SPEC_REVISION (0, 9, 5);
	//Device->VirtioDevice.SubSystemDeviceId = Pci.Device.SubsystemID;

	//
	// Note: We don't support the MSI-X capability.  If we did,
	//       the offset would become 24 after enabling MSI-X.
	//
	Device->DeviceSpecificConfigurationOffset =
		VIRTIO_DEVICE_SPECIFIC_CONFIGURATION_OFFSET_PCI;
	Device->Signature = VIRTIO_PCI_DEVICE_SIGNATURE;

	return EFI_SUCCESS;
}

/**

  Uninitialize the internals of a virtio-pci device that has been successfully
  set up with VirtioPciInit().

  @param[in, out]  Dev  The device to clean up.

**/
VOID
EFIAPI
VirtioPciUninit (
	IN OUT __attribute__((unused)) VIRTIO_PCI_DEVICE *Device
	)
{
  // Note: This function mirrors VirtioPciInit() that does not allocate any
  //       resources - there's nothing to free here.
}

