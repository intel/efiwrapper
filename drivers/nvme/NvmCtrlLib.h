/** @file

@copyright
  Copyright (c) 2008 - 2015 Intel Corporation. All rights reserved.
  This software and associated documentation (if any) is furnished
  under a license and may only be used or copied in accordance
  with the terms of the license. Except as permitted by such
  license, no part of this software or documentation may be
  reproduced, stored in a retrieval system, or transmitted in any
  form or by any means without the express written consent of
  Intel Corporation.
  This file contains an 'Intel Pre-EFI Module' and is licensed
  for Intel CPUs and Chipsets under the terms of your license
  agreement with Intel or your vendor. This file may be
  modified by the user, subject to additional terms of the
  license agreement.
**/

#ifndef __NVM_CTRL_LIB_H__
#define __NVM_CTRL_LIB_H__

typedef UINT64  EFI_PEI_LBA;

typedef struct {
	UINT64   BlockNum;
	UINT32   BlockSize;
} DEVICE_BLOCK_INFO;

typedef enum {
	DevInitAll,
	DevInitOnlyPhase1,
	DevInitOnlyPhase2
} DEVICE_INIT_PHASE;


#pragma pack(1)
typedef struct {
	UINT8       Bus;
	UINT8       Device;
	UINT8       Func;
} PciAddrNvm;

typedef struct {
	PciAddrNvm        DeviceAddress;
} NvmCtrlPlatformInfo;
#pragma pack()


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
NvmeGetMediaInfo (
	IN  UINTN             DeviceIndex,
	OUT DEVICE_BLOCK_INFO *DevBlockInfo
);

/**
  This function reads data from Nvme device to Memory.

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
NvmeReadBlocks (
	IN  UINTN         DeviceIndex,
	IN  EFI_PEI_LBA   StartLBA,
	IN  UINTN         BufferSize,
	OUT VOID          *Buffer
);

/**
  This function writes data from Memory to Nvme device

  @param[in]  DeviceIndex   Specifies the block device to which the function wants
                            to talk.
  @param[in]  StartBlock    Target EMMC block number(LBA) where data will be written
  @param[in]  DataSize      Total data size to be written in bytes unit
  @param[out] DataAddress   Data address in Memory to be copied to EMMC

  @retval EFI_SUCCESS       The operation is done correctly.
  @retval Others            The operation fails.

**/
EFI_STATUS
EFIAPI
NvmeWriteBlocks (
	IN  UINTN    DeviceIndex,
	IN  EFI_LBA  StartLBA,
	IN  UINTN    DataSize,
	IN  VOID     *DataAddress
);


/**
  This function initializes Nvme device
  @param[in]  NvmeHcPciBase MMC Host Controller's PCI ConfigSpace Base address
  @param[in]  NvmeInitMode  For the performance optimization, device initialization can be
                            separated to early init and the
                            rest of init.

                            DevInitAll        : Execute generic device initialization
                            DevInitOnlyPhase1 : Execute only early phase initialization
                            DevInitOnlyPhase2 : Skip early phase initialization, and then
                                                initialize the rest of initialization


  @retval EFI_SUCCESS           The request is executed successfully.
  @retval EFI_OUT_OF_RESOURCES  The request could not be executed due to a lack of resources.
  @retval Others                The request could not be executed successfully.

**/
EFI_STATUS EFIAPI NvmeInitialize (IN UINTN NvmeHcPciBase);

EFI_NVM_EXPRESS_PASS_THRU_PROTOCOL *NvmeGetPassthru(void);

EFI_STORAGE_SECURITY_COMMAND_PROTOCOL *NvmeGetSecurityInterface(void);

#endif
