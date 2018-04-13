/** @file
  This file provides some helper functions which are specific for RPMB device.

  Copyright (c) 2015 - 2018, Intel Corporation. All rights reserved.<BR>
  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <kconfig.h>
#include <libpayload-config.h>
#include <libpayload.h>
#include <ewlib.h>
#include <ewlog.h>

#include "VirtioDeviceCommon.h"
#include "VirtioPciDeviceLib.h"
#include "VirtioRpmbDevice.h"
#include "ScsiPassThruExt.h"
#include "VirtioRpmbDevice.h"
#include "VirtioRpmbAccessLib.h"

static VRPMB_DEV *gRpmbdev = NULL;

EFI_STATUS
EFIAPI
VrpmbPassThru(
	IN __attribute__((__unused__)) EFI_EXT_SCSI_PASS_THRU_PROTOCOL   *This,
	IN __attribute__((__unused__)) UINT8                  *Target,
	IN __attribute__((__unused__)) UINT64                 Lun,
	IN OUT EFI_EXT_SCSI_PASS_THRU_SCSI_REQUEST_PACKET     *Packet,
	IN __attribute__((__unused__)) EFI_EVENT              Event OPTIONAL
	)
{
	EFI_STATUS	Status;
	UINT32	BufferSize;
	VOID	*Buffer;
	VRPMB_DEV	*Dev;

	BufferSize = Packet->OutTransferLength;
	Buffer = Packet->OutDataBuffer;

	Dev = VirtioRPMBGetContext();
	if (!Dev)
		return EFI_INVALID_PARAMETER;

	Status = VirtioRpmbSentData (Dev, Buffer, BufferSize);

	return Status;
}

EFI_STATUS
EFIAPI
VirtioRpmbInitialize (
	IN  UINTN               VirtioRpmbPciBase
	)
{
	VIRTIO_PCI_DEVICE *VirtPci;
	VRPMB_DEV   *Dev;
	EFI_STATUS Status;

	VirtPci = (VIRTIO_PCI_DEVICE *) malloc (sizeof *VirtPci);
	if (VirtPci == NULL) {
		return EFI_OUT_OF_RESOURCES;
	}

	Status = VirtioPciInit (VirtPci, VirtioRpmbPciBase);

	if (EFI_ERROR (Status)) {
		ewerr("VirtioPciInit fail Error %x\n",(UINT32)Status);
		free (VirtPci);
		return Status;
	}

	Dev = (VRPMB_DEV *) malloc (sizeof *Dev);
	if (Dev == NULL) {
		free (VirtPci);
		return EFI_OUT_OF_RESOURCES;
	}

	Dev->VirtIo = &VirtPci->VirtioDevice;

	Status = VirtioRpmbInit (Dev);
	if (EFI_ERROR (Status)) {
		ewerr("VirtioRpmbInit fail Error %x\n",(UINT32)Status);
		free (Dev);
		free (VirtPci);
		return Status;
	}

	gRpmbdev = Dev;

	return EFI_SUCCESS;
}

VRPMB_DEV *VirtioRPMBGetContext()
{
	return gRpmbdev;
}

