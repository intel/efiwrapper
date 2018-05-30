/** @file

  Copyright (c) 2018, Intel Corporation. All rights reserved.<BR>
  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

/*
 * VirtioRpmbAccessLib.h
 */
#ifndef _VIRTIORPMB_ACCESS_LIB_H_
#define _VIRTIORPMB_ACCESS_LIB_H_

#include "ScsiPassThruExt.h"
#include "VirtioRpmbDevice.h"

EFI_STATUS
EFIAPI
VrpmbPassThru (
	IN __attribute__((__unused__)) EFI_EXT_SCSI_PASS_THRU_PROTOCOL      *This,
	IN __attribute__((__unused__)) UINT8                  *Target,
	IN __attribute__((__unused__)) UINT64                 Lun,
	IN OUT EFI_EXT_SCSI_PASS_THRU_SCSI_REQUEST_PACKET     *Packet,
	IN __attribute__((__unused__)) EFI_EVENT              Event OPTIONAL
	);

EFI_STATUS
EFIAPI
VirtioRpmbInitialize (
	IN  UINTN               VirtioRpmbPciBase
	);

VRPMB_DEV *VirtioRPMBGetContext();
#endif
