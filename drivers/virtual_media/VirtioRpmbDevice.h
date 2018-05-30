/** @file

  Internal definitions for the virtio-rpmb driver.

  Copyright (C) 2012, Red Hat, Inc.
  Copyright (c) 2012 - 2018, Intel Corporation. All rights reserved.<BR>

  This program and the accompanying materials are licensed and made available
  under the terms and conditions of the BSD License which accompanies this
  distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS, WITHOUT
  WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef _VIRTIO_RPMB_DEVICE_H_
#define _VIRTIO_RPMB_DEVICE_H_

#include <efilink.h>

#include "Virtio.h"
#include "VirtioDevice.h"
#include "ScsiPassThruExt.h"

#define VRPMB_SIG		SIGNATURE_32 ('R', 'P', 'M', 'B')

typedef struct {
	UINT32    Signature;
	VIRTIO_DEVICE_PROTOCOL    *VirtIo;
	EFI_EVENT    ExitBoot;
	VRING    Ring;
	EFI_EXT_SCSI_PASS_THRU_PROTOCOL    *PassThru;
	VOID*    RingMap;
} VRPMB_DEV;

#define VIRTIO_RPMB_FROM_RPMB_IO(RpmbIoPointer) \
	CR (RpmbIoPointer, VRPMB_DEV, PassThru, VRPMB_SIG)

/**

  Set up all and virtio-rpmb aspects of this driver for the specified
  device.

  @param[in out] Dev  The driver instance to configure. The caller is
                      responsible for Dev->VirtIo's validity (ie. working IO
                      access to the underlying virtio-rpmb device).

  @retval EFI_SUCCESS      Setup complete.

  @retval EFI_UNSUPPORTED  The driver is unable to work with the virtio ring.

  @return                  Error codes from VirtioRingInit() or
                           VIRTIO_CFG_READ() / VIRTIO_CFG_WRITE or
                           VirtioRingMap().

**/

EFI_STATUS
EFIAPI
VirtioRpmbInit (
	IN OUT VRPMB_DEV          *Dev
	);

/**

  Uninitialize the internals of a virtio-rpmb device that has been successfully
  set up with VirtioRpmbInit().

  @param[in out]  Dev  The device to clean up.

**/
VOID
EFIAPI
VirtioRpmbUninit (
	IN OUT VRPMB_DEV          *Dev
	);

EFI_STATUS
EFIAPI
VirtioRpmbSentData (
	IN VRPMB_DEV              *Dev,
	IN VOID                   *Buffer,
	IN UINT32                 BufferSize
	);

#endif // _VIRTIO_RPMB_DEVICE_H_
