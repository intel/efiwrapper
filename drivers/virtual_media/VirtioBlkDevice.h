/** @file

  Internal definitions for the virtio-blk driver, which produces Block I/O
  Protocol instances for virtio-blk devices.

  Copyright (C) 2012, Red Hat, Inc.

  This program and the accompanying materials are licensed and made available
  under the terms and conditions of the BSD License which accompanies this
  distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS, WITHOUT
  WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef _VIRTIO_BLK_DEVICE_H_
#define _VIRTIO_BLK_DEVICE_H_

#include <efilink.h>

#include "Virtio.h"
#include "VirtioDevice.h"

#ifndef EFI_BLOCK_IO_PROTOCOL
#define EFI_BLOCK_IO_PROTOCOL EFI_BLOCK_IO
#endif

#ifndef EFI_PEI_LBA
#define EFI_PEI_LBA EFI_LBA
#endif

#define VBLK_SIG		SIGNATURE_32 ('V', 'B', 'L', 'K')

typedef struct {
	//
	// Parts of this structure are initialized / torn down in various functions
	// at various call depths. The table to the right should make it easier to
	// track them.
	//
	//                     field                    init function       init dpth
	//                     ---------------------    ------------------  ---------
	UINT32			Signature;	// DriverBindingStart  0
	VIRTIO_DEVICE_PROTOCOL	*VirtIo;		// DriverBindingStart  0
	EFI_EVENT		ExitBoot;	// DriverBindingStart  0
	VRING			Ring;		// VirtioRingInit      2
	EFI_BLOCK_IO_PROTOCOL	BlockIo;		// VirtioBlkInit       1
	EFI_BLOCK_IO_MEDIA	BlockIoMedia;	// VirtioBlkInit       1
	VOID			*RingMap;	// VirtioRingMap       2
} VBLK_DEV;

#define VIRTIO_BLK_FROM_BLOCK_IO(BlockIoPointer) \
	CR (BlockIoPointer, VBLK_DEV, BlockIo, VBLK_SIG)

/**

  Set up all BlockIo and virtio-blk aspects of this driver for the specified
  device.

  @param[in out] Dev  The driver instance to configure. The caller is
                      responsible for Dev->VirtIo's validity (ie. working IO
                      access to the underlying virtio-blk device).

  @retval EFI_SUCCESS      Setup complete.

  @retval EFI_UNSUPPORTED  The driver is unable to work with the virtio ring or
                           virtio-blk attributes the host provides.

  @return                  Error codes from VirtioRingInit() or
                           VIRTIO_CFG_READ() / VIRTIO_CFG_WRITE or
                           VirtioRingMap().

**/

EFI_STATUS
EFIAPI
VirtioBlkInit (
	IN OUT VBLK_DEV *Dev
	);

/**

  Uninitialize the internals of a virtio-blk device that has been successfully
  set up with VirtioBlkInit().

  @param[in out]  Dev  The device to clean up.

**/
VOID
EFIAPI
VirtioBlkUninit (
	IN OUT VBLK_DEV *Dev
	);

//
// UEFI Spec 2.3.1 + Errata C, 12.8 EFI Block I/O Protocol
// Driver Writer's Guide for UEFI 2.3.1 v1.01,
//   24.2 Block I/O Protocol Implementations
//
EFI_STATUS
EFIAPI
VirtioBlkReset (
	IN EFI_BLOCK_IO_PROTOCOL *This,
	IN BOOLEAN               ExtendedVerification
	);


/**

  ReadBlocks() operation for virtio-blk.

  See
  - UEFI Spec 2.3.1 + Errata C, 12.8 EFI Block I/O Protocol, 12.8 EFI Block I/O
    Protocol, EFI_BLOCK_IO_PROTOCOL.ReadBlocks().
  - Driver Writer's Guide for UEFI 2.3.1 v1.01, 24.2.2. ReadBlocks() and
    ReadBlocksEx() Implementation.

  Parameter checks and conformant return values are implemented in
  VerifyReadWriteRequest() and SynchronousRequest().

  A zero BufferSize doesn't seem to be prohibited, so do nothing in that case,
  successfully.

**/

EFI_STATUS
EFIAPI
VirtioBlkReadBlocks (
	IN  EFI_BLOCK_IO_PROTOCOL *This,
	IN  UINT32                MediaId,
	IN  EFI_PEI_LBA           Lba,
	IN  UINTN                 BufferSize,
	OUT VOID                  *Buffer
	);


/**

  WriteBlocks() operation for virtio-blk.

  See
  - UEFI Spec 2.3.1 + Errata C, 12.8 EFI Block I/O Protocol, 12.8 EFI Block I/O
    Protocol, EFI_BLOCK_IO_PROTOCOL.WriteBlocks().
  - Driver Writer's Guide for UEFI 2.3.1 v1.01, 24.2.3 WriteBlocks() and
    WriteBlockEx() Implementation.

  Parameter checks and conformant return values are implemented in
  VerifyReadWriteRequest() and SynchronousRequest().

  A zero BufferSize doesn't seem to be prohibited, so do nothing in that case,
  successfully.

**/

EFI_STATUS
EFIAPI
VirtioBlkWriteBlocks (
	IN EFI_BLOCK_IO_PROTOCOL *This,
	IN UINT32                MediaId,
	IN EFI_LBA               Lba,
	IN UINTN                 BufferSize,
	IN VOID                  *Buffer
	);


/**

  FlushBlocks() operation for virtio-blk.

  See
  - UEFI Spec 2.3.1 + Errata C, 12.8 EFI Block I/O Protocol, 12.8 EFI Block I/O
    Protocol, EFI_BLOCK_IO_PROTOCOL.FlushBlocks().
  - Driver Writer's Guide for UEFI 2.3.1 v1.01, 24.2.4 FlushBlocks() and
    FlushBlocksEx() Implementation.

  If the underlying virtio-blk device doesn't support flushing (ie.
  write-caching), then this function should not be called by higher layers,
  according to EFI_BLOCK_IO_MEDIA characteristics set in VirtioBlkInit().
  Should they do nonetheless, we do nothing, successfully.

**/

EFI_STATUS
EFIAPI
VirtioBlkFlushBlocks (
	IN EFI_BLOCK_IO_PROTOCOL *This
	);

#endif // _VIRTIO_BLK_DXE_H_
