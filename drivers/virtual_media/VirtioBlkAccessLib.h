/** @file

  Copyright (c) 2017, Intel Corporation. All rights reserved.<BR>
  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

/*
 * VirtioBlkAccessLib.h
 */
#ifndef _VIRTIOBLK_ACCESS_LIB_H_
#define _VIRTIOBLK_ACCESS_LIB_H_

/**
  Gets a block device's media information.

  This function will provide the caller with the specified block device's media
  information. If the media changes, calling this function will update the media
  information accordingly.

  @param[in]  DeviceIndex    Specifies the block device to which the function wants
                             to talk.
  @param[out] DevBlockInfo   The Block Io information of the specified block partition.

  @retval EFI_SUCCESS        The Block Io information about the specified block device
                             was obtained successfully.
  @retval EFI_DEVICE_ERROR   Cannot get the media information due to a hardware
                             error.

**/
EFI_STATUS
EFIAPI
VirtioGetMediaInfo (
	IN  UINTN                          DeviceIndex,
	OUT DEVICE_BLOCK_INFO              *DevBlockInfo
	);

/**
  This function reads data from VirtioBlk to Memory.

  @param[in]  DeviceIndex   Specifies the block device to which the function wants
                            to talk.
  @param[in]  StartLBA      The starting logical block address (LBA) to read from
                            on the device
  @param[in]  BufferSize    The size of the Buffer in bytes. This number must be
                            a multiple of the intrinsic block size of the device.
  @param[out] Buffer        A pointer to the destination buffer for the data.
                            The caller is responsible for the ownership of the
                            buffer.

  @retval EFI_SUCCESS             The data was read correctly from the device.
  @retval EFI_DEVICE_ERROR        The device reported an error while attempting
                                  to perform the read operation.
  @retval EFI_INVALID_PARAMETER   The read request contains LBAs that are not
                                  valid, or the buffer is not properly aligned.
  @retval EFI_NO_MEDIA            There is no media in the device.
  @retval EFI_BAD_BUFFER_SIZE     The BufferSize parameter is not a multiple of
                                  the intrinsic block size of the device.

**/
EFI_STATUS
EFIAPI
VirtioReadBlocks (
	IN  UINTN                         DeviceIndex,
	IN  EFI_LBA                       StartLBA,
	IN  UINTN                         BufferSize,
	OUT VOID                          *Buffer
	);

/**
  This function writes data from Memory to VirtioBlk

  @param[in]  DeviceIndex   Specifies the block device to which the function wants
                            to talk.
  @param[in]  StartLBA      Target VirtioMedia block number(LBA) where data will be written
  @param[in]  DataSize      Total data size to be written in bytes unit
  @param[in] DataAddress   Data address in Memory to be copied to VirtioBlk

  @retval EFI_SUCCESS       The operation is done correctly.
  @retval Others            The operation fails.

**/
EFI_STATUS
EFIAPI
VirtioWriteBlocks (
	IN  UINTN                         DeviceIndex,
	IN  EFI_LBA                       StartLBA,
	IN  UINTN                         DataSize,
	IN  VOID                          *DataAddress
	);


/**
  This function initializes VirtioBlk device

  @param[in]  VirtioBlkPciBase   VirtioMedia Host Controller's PCI ConfigSpace Base address
  @param[in]  VirtioBlkInitMode    For the performance optimization,
                             VirtioBlk initialization is separated to early init and the rest of init.

                             DevInitAll        : Execute generic VirtioMedia device initialization
                             DevInitOnlyPhase1 : Execute only early phase initialization
                             DevInitOnlyPhase2 : Skip early phase initialization,
                                                 and then initialize the rest of initialization


  @retval EFI_SUCCESS           The request is executed successfully.
  @retval EFI_OUT_OF_RESOURCES  The request could not be executed due to a lack of resources.
  @retval Others                The request could not be executed successfully.

**/
EFI_STATUS
EFIAPI
VirtioMediaInitialize (
	IN  UINTN               VirtioBlkPciBase
	);
#endif
