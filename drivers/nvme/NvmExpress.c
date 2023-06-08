/** @file
  NvmExpressDxe driver is used to manage non-volatile memory subsystem which follows
  NVM Express specification.

  Copyright (c) 2013 - 2016, Intel Corporation. All rights reserved.<BR>
  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php.

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <efi.h>
#include <efilib.h>
#include <pci/pci.h>
#include <arch/io.h>

#include "NvmExpress.h"
#include "NvmCtrlLib.h"

#define PCI_BASE_ADDRESSREG_OFFSET                  0x10

NVME_CONTROLLER_PRIVATE_DATA        *mNvmeCtrlPrivate;
NVME_DEVICE_PRIVATE_DATA            *mMultiNvmeDrive[10]; //maxium 10
NvmCtrlPlatformInfo NvmCtrlInfo = { {1,0,0} };

EFI_NVM_EXPRESS_PASS_THRU_PROTOCOL *NvmeGetPassthru(void)
{
  return (void *)&mNvmeCtrlPrivate->Passthru;
}

EFI_STORAGE_SECURITY_COMMAND_PROTOCOL *NvmeGetSecurityInterface(void)
{
  NVME_DEVICE_PRIVATE_DATA *device = mMultiNvmeDrive[0];
  EFI_STORAGE_SECURITY_COMMAND_PROTOCOL *security;

  if (device == NULL)
    return NULL;

  security = &device->StorageSecurity;
  if (security->SendData == NULL || security->ReceiveData == NULL)
    return NULL;

  return security;
}


//
// Template for NVM Express Pass Thru Mode data structure.
//
EFI_NVM_EXPRESS_PASS_THRU_MODE gEfiNvmExpressPassThruMode = {
  EFI_NVM_EXPRESS_PASS_THRU_ATTRIBUTES_PHYSICAL   |
  EFI_NVM_EXPRESS_PASS_THRU_ATTRIBUTES_LOGICAL    |
  EFI_NVM_EXPRESS_PASS_THRU_ATTRIBUTES_NONBLOCKIO |
  EFI_NVM_EXPRESS_PASS_THRU_ATTRIBUTES_CMD_SET_NVM,
  sizeof (UINTN),
  0x10100
};

/**
  Check if the specified Nvm Express device namespace is active, and create child handles
  for them with BlockIo and DiskInfo protocol instances.

  @param[in] Private         The pointer to the NVME_CONTROLLER_PRIVATE_DATA data structure.
  @param[in] NamespaceId     The NVM Express namespace ID  for which a device path node is to be
                             allocated and built. Caller must set the NamespaceId to zero if the
                             device path node will contain a valid UUID.

  @retval EFI_SUCCESS        All the namespaces in the device are successfully enumerated.
  @return Others             Some error occurs when enumerating the namespaces.

**/
EFI_STATUS
EnumerateNvmeDevNamespace (
  IN NVME_CONTROLLER_PRIVATE_DATA       *Private,
  UINT32                                NamespaceId
  )
{
  NVME_ADMIN_NAMESPACE_DATA             *NamespaceData;
  NVME_DEVICE_PRIVATE_DATA              *Device;
  EFI_STATUS                            Status;
  UINT32                                Lbads;
  UINT32                                Flbas;
  UINT32                                LbaFmtIdx;
  UINT8                                 Sn[21];
  UINT8                                 Mn[41];
  Device            = NULL;

  //
  // Allocate a buffer for Identify Namespace data
  //
  NamespaceData = MallocZero(sizeof (NVME_ADMIN_NAMESPACE_DATA));
  if (NamespaceData == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  //
  // Identify Namespace
  //
  Status = NvmeIdentifyNamespace (
             Private,
             NamespaceId,
             (VOID *)NamespaceData
             );
  if (EFI_ERROR(Status)) {
    goto Exit;
  }

  //
  // Validate Namespace
  //
  if (NamespaceData->Ncap == 0) {
    Status = EFI_DEVICE_ERROR;
  } else {
    //
    // allocate device private data for each discovered namespace
    //
    Device = MallocZero(sizeof(NVME_DEVICE_PRIVATE_DATA));
    if (Device == NULL) {
      Status = EFI_OUT_OF_RESOURCES;
      goto Exit;
    }

    //
    // Initialize SSD namespace instance data
    //
    Device->Signature           = NVME_DEVICE_PRIVATE_DATA_SIGNATURE;
    Device->NamespaceId         = NamespaceId;
    Device->NamespaceUuid       = NamespaceData->Eui64;
    Device->Controller          = Private;

    //
    // Build BlockIo media structure
    //
    Device->Media.MediaId        = 0;
    Device->Media.RemovableMedia = FALSE;
    Device->Media.MediaPresent   = TRUE;
    Device->Media.LogicalPartition = FALSE;
    Device->Media.ReadOnly       = FALSE;
    Device->Media.WriteCaching   = FALSE;
    Device->Media.IoAlign        = Private->PassThruMode.IoAlign;

    Flbas     = NamespaceData->Flbas;
    LbaFmtIdx = Flbas & 0xF;
    Lbads     = NamespaceData->LbaFormat[LbaFmtIdx].Lbads;
    Device->Media.BlockSize = (UINT32)1 << Lbads;

    Device->Media.LastBlock                     = NamespaceData->Nsze - 1;
    Device->Media.LogicalBlocksPerPhysicalBlock = 1;
    Device->Media.LowestAlignedLba              = 1;

    //
    // Create BlockIo Protocol instance
    //
    Device->BlockIo.Revision     = EFI_BLOCK_IO_INTERFACE_REVISION2;
    Device->BlockIo.Media        = &Device->Media;
    Device->BlockIo.Reset        = (EFI_BLOCK_RESET)NvmeBlockIoReset;
    Device->BlockIo.ReadBlocks   = (EFI_BLOCK_READ)NvmeBlockIoReadBlocks;
    Device->BlockIo.WriteBlocks  = (EFI_BLOCK_WRITE)NvmeBlockIoWriteBlocks;
    Device->BlockIo.FlushBlocks  = (EFI_BLOCK_FLUSH)NvmeBlockIoFlushBlocks;

    InitializeListHead (&Device->AsyncQueue);

    CopyMem (&Device->NamespaceData, NamespaceData, sizeof (NVME_ADMIN_NAMESPACE_DATA));
    mMultiNvmeDrive[NamespaceId - 1] = Device; //NamespaceId is 1 based

    //
    // Create StorageSecurityProtocol Instance
    //
    Device->StorageSecurity.ReceiveData = NvmeStorageSecurityReceiveData;
    Device->StorageSecurity.SendData    = NvmeStorageSecuritySendData;
    DEBUG_NVME((EFI_D_INFO, "## SECURITY_SEND_RECEIVE %d ####\n", (Private->ControllerData->Oacs & SECURITY_SEND_RECEIVE_SUPPORTED)));
    if ((Private->ControllerData->Oacs & SECURITY_SEND_RECEIVE_SUPPORTED) == 0) {
      Device->StorageSecurity.ReceiveData = NULL;
      Device->StorageSecurity.SendData = NULL;
    }

    //
    // Dump NvmExpress Identify Namespace Data
    //
    DEBUG_NVME ((EFI_D_INFO, " == NVME IDENTIFY NAMESPACE [%d] DATA ==\n", NamespaceId));
    DEBUG_NVME ((EFI_D_INFO, "    NSZE        : 0x%x\n", NamespaceData->Nsze));
    DEBUG_NVME ((EFI_D_INFO, "    NCAP        : 0x%x\n", NamespaceData->Ncap));
    DEBUG_NVME ((EFI_D_INFO, "    NUSE        : 0x%x\n", NamespaceData->Nuse));
    DEBUG_NVME ((EFI_D_INFO, "    LBAF0.LBADS : 0x%x\n", (NamespaceData->LbaFormat[0].Lbads)));

    //
    // Build controller name for Component Name (2) protocol.
    //
    CopyMem (Sn, Private->ControllerData->Sn, sizeof (Private->ControllerData->Sn));
    Sn[20] = 0;
    CopyMem (Mn, Private->ControllerData->Mn, sizeof (Private->ControllerData->Mn));
    Mn[40] = 0;
	snprintf(Device->ModelName, sizeof (Device->ModelName), (const char *)"%a-%a-%x", Sn, Mn, NamespaceData->Eui64);
  }

Exit:
  if (NamespaceData != NULL) {
    FreeZero (NamespaceData);
  }

  if (EFI_ERROR(Status) && (Device != NULL)) {
    FreeZero (Device);
  }
  return Status;
}

/**
  Discover all Nvm Express device namespaces, and create child handles for them with BlockIo
  and DiskInfo protocol instances.

  @param[in] Private         The pointer to the NVME_CONTROLLER_PRIVATE_DATA data structure.

  @retval EFI_SUCCESS        All the namespaces in the device are successfully enumerated.
  @return Others             Some error occurs when enumerating the namespaces.

**/
EFI_STATUS
DiscoverAllNamespaces (
  IN NVME_CONTROLLER_PRIVATE_DATA       *Private
  )
{
  EFI_STATUS                            Status;
  UINT32                                NamespaceId;
  EFI_NVM_EXPRESS_PASS_THRU_PROTOCOL    *Passthru;

  NamespaceId   = 0xFFFFFFFF;
  Passthru      = &Private->Passthru;

  while (TRUE) {
    Status = Passthru->GetNextNamespace (
                         Passthru,
                         (UINT32 *)&NamespaceId
                         );

    if (EFI_ERROR (Status)) {
      break;
    }

    Status = EnumerateNvmeDevNamespace (
               Private,
               NamespaceId
               );

    if (EFI_ERROR(Status)) {
      continue;
    }
  }

  return EFI_SUCCESS;
}

VOID
EFIAPI
MemoryFence (
  VOID
  )
{
  // This is a little bit of overkill and it is more about the compiler that it is
  // actually processor synchronization. This is like the _ReadWriteBarrier
  // Microsoft specific intrinsic
  __asm__ __volatile__ ("":::"memory");
}

#define PCI_BASE_ADDRESSREG_OFFSET     0x10
#define PCI_COMMAND_OFFSET             0x04
#define PCI_DEVICE_ID_OFFSET           0x02

typedef enum {
  FilterWidth8,
  FilterWidth16,
  FilterWidth32,
  FilterWidth64
} FILTER_IO_WIDTH;

BOOLEAN
EFIAPI
FilterBeforeMmIoRead (
  VOID
  )
{
  return TRUE;
}

VOID
EFIAPI
FilterAfterMmIoRead (
  VOID
  )
{
  return;
}

BOOLEAN
EFIAPI
FilterBeforeMmIoWrite (
  VOID
  )
{
  return TRUE;
}

VOID
EFIAPI
FilterAfterMmIoWrite (
  VOID
  )
{
  return;
}

UINT16
EFIAPI
MmioWrite16 (
  IN      UINTN                     Address,
  IN      UINT16                    Value
  )
{
  BOOLEAN                           Flag;

  ASSERT ((Address & 1) == 0);

  Flag = FilterBeforeMmIoWrite ();
  if (Flag) {
    MemoryFence ();
    *(volatile UINT16*)Address = Value;
    MemoryFence ();
  }
  FilterAfterMmIoWrite ();

  return Value;
}

UINT16
EFIAPI
MmioRead16 (
  IN      UINTN                     Address
  )
{
  UINT16         Value=0;
  BOOLEAN        Flag;

  ASSERT ((Address & 1) == 0);
  Flag = FilterBeforeMmIoRead ();
  if (Flag) {
    MemoryFence ();
    Value = *(volatile UINT16*)Address;
    MemoryFence ();
  }
  FilterAfterMmIoRead ();

  return Value;
}

#define EFI_PCI_COMMAND_IO_SPACE                        0x0001
#define EFI_PCI_COMMAND_MEMORY_SPACE                    0x0002
#define EFI_PCI_COMMAND_BUS_MASTER                      0x0004
#define EFI_PCI_COMMAND_SPECIAL_CYCLE                   0x0008
#define EFI_PCI_COMMAND_MEMORY_WRITE_AND_INVALIDATE     0x0010
#define EFI_PCI_COMMAND_VGA_PALETTE_SNOOP               0x0020
#define EFI_PCI_COMMAND_PARITY_ERROR_RESPOND            0x0040
#define EFI_PCI_COMMAND_STEPPING_CONTROL                0x0080
#define EFI_PCI_COMMAND_SERR                            0x0100
#define EFI_PCI_COMMAND_FAST_BACK_TO_BACK               0x0200

UINT16
 EFIAPI
 MmioOr16 (
   IN      UINTN                     Address,
   IN      UINT16                    OrData
   )
 {
   return MmioWrite16 (Address, (UINT16) (MmioRead16 (Address) | OrData));
 }

/**
  Starts a device controller or a bus controller.

  The Start() function is designed to be invoked from the EFI boot service ConnectController().
  As a result, much of the error checking on the parameters to Start() has been moved into this
  common boot service. It is legal to call Start() from other locations,
  but the following calling restrictions must be followed or the system behavior will not be deterministic.
  1. ControllerHandle must be a valid EFI_HANDLE.
  2. If RemainingDevicePath is not NULL, then it must be a pointer to a naturally aligned
     EFI_DEVICE_PATH_PROTOCOL.
  3. Prior to calling Start(), the Supported() function for the driver specified by This must
     have been called with the same calling parameters, and Supported() must have returned EFI_SUCCESS.

  @param[in]  VOID

  @retval EFI_SUCCESS              The device was started.
  @retval EFI_DEVICE_ERROR         The device could not be started due to a device error.Currently not implemented.
  @retval EFI_OUT_OF_RESOURCES     The request could not be completed due to a lack of resources.
  @retval Others                   The driver failded to start the device.

**/
EFI_STATUS
EFIAPI
NvmeInitialize (
  IN  UINTN               NvmeHcPciBase
  )
{
  void *aligned_buf;
  EFI_STATUS                          Status;
  NVME_CONTROLLER_PRIVATE_DATA        *Private;

  DEBUG_NVME ((EFI_D_INFO, "NvmeInitialize:\n"));

  pci_write_config32(NvmeHcPciBase,PCI_COMMAND_OFFSET, \
    (UINT32)(EFI_PCI_COMMAND_IO_SPACE | EFI_PCI_COMMAND_MEMORY_SPACE | EFI_PCI_COMMAND_BUS_MASTER));

  Private          = NULL;

  //
  // Check EFI_ALREADY_STARTED to reuse the original NVME_CONTROLLER_PRIVATE_DATA.
  //
  Private = MallocZero (sizeof (NVME_CONTROLLER_PRIVATE_DATA));
  if (Private == NULL) {
    DEBUG_NVME ((EFI_D_ERROR, "NvmeInitialize: allocating pool for Nvme Private Data failed!\n"));
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }

  //DEBUG_NVME ((EFI_D_INFO, "NvmeControllerInit: NvmeHCBase = 0x%X\n", NvmeHcPciBase));

  //
  // 6 x 4kB aligned buffers will be carved out of this buffer.
  // 1st 4kB boundary is the start of the admin submission queue.
  // 2nd 4kB boundary is the start of the admin completion queue.
  // 3rd 4kB boundary is the start of I/O submission queue #1.
  // 4th 4kB boundary is the start of I/O completion queue #1.
  // 5th 4kB boundary is the start of I/O submission queue #2.
  // 6th 4kB boundary is the start of I/O completion queue #2.
  //
  // Allocate 6 pages of memory, then map it for bus master read and write.
  //
  aligned_buf = nvme_alloc_pages(6);

  Private->Buffer                    = (UINT8 *)aligned_buf;
  Private->Signature                 = NVME_CONTROLLER_PRIVATE_DATA_SIGNATURE;
  Private->NvmeHCBase                = read32((void *)(NvmeHcPciBase + PCI_BASE_ADDRESSREG_OFFSET)) & 0xFFFFF000;
  Private->Passthru.Mode             = &Private->PassThruMode;
  Private->Passthru.PassThru         = (EFI_NVM_EXPRESS_PASS_THRU_PASSTHRU)NvmExpressPassThru;
  Private->Passthru.GetNextNamespace = NvmExpressGetNextNamespace;
  Private->Passthru.GetNamespace = NvmExpressGetNamespace;
  CopyMem (&Private->PassThruMode, &gEfiNvmExpressPassThruMode, sizeof (EFI_NVM_EXPRESS_PASS_THRU_MODE));
  InitializeListHead (&Private->AsyncPassThruQueue);
  InitializeListHead (&Private->UnsubmittedSubtasks);

  uint32_t addr;
  addr = pci_read_config32(NvmeHcPciBase, PCI_BASE_ADDRESS_0);
  Private->NvmeHCBase = (addr & ~0xf);
  DEBUG_NVME ((EFI_D_INFO, "NvmeControllerInit: NvmeHCBase = 0x%X\n", addr));

  Status = NvmeControllerInit (Private);
  if (EFI_ERROR(Status)) {
    goto Exit;
  }
  mNvmeCtrlPrivate = Private;

  Status = DiscoverAllNamespaces (
             Private
             );

  DEBUG_NVME ((EFI_D_INFO, "NvmeInitialize: end successfully\n"));
  return EFI_SUCCESS;

Exit:
  if (EFI_ERROR (Status)) {
      if ((Private != NULL) && (Private->ControllerData != NULL)) {
         FreeZero (Private->ControllerData);
      }
  }

  DEBUG_NVME ((EFI_D_INFO, "NvmeInitialize: end with 0x%X\n", Status));

  return Status;
}


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
  IN  UINTN                          DeviceIndex,
  OUT DEVICE_BLOCK_INFO              *DevBlockInfo
  )
{
	EFI_BLOCK_IO_MEDIA	*Media;

	DevBlockInfo->BlockNum = 0;
	DevBlockInfo->BlockSize = 0;
	if (mMultiNvmeDrive[DeviceIndex] == NULL)
		return EFI_DEVICE_ERROR;

	Media = &mMultiNvmeDrive[DeviceIndex]->Media;

	DevBlockInfo->BlockNum = Media->LastBlock+1;
	DevBlockInfo->BlockSize = Media->BlockSize;
	return EFI_SUCCESS;
}

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
  IN  UINTN                         DeviceIndex __attribute__((unused)),
  IN  EFI_PEI_LBA                   StartLBA,
  IN  UINTN                         BufferSize,
  OUT VOID                          *Buffer
  )
{
  EFI_STATUS Status;

  Status = NvmeBlockIoReadBlocks(&mMultiNvmeDrive[0]->BlockIo, 0, StartLBA, BufferSize, Buffer);

  return Status;
}

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
  IN  UINTN                         DeviceIndex __attribute__((unused)),
  IN  EFI_LBA                       StartLBA,
  IN  UINTN                         DataSize,
  IN  VOID                          *DataAddress
  )
{
  EFI_STATUS Status;

  Status = NvmeBlockIoWriteBlocks(&mMultiNvmeDrive[0]->BlockIo, 0, StartLBA, DataSize, DataAddress);

  return Status;
}

