/** @file

  This driver produces Block I/O Protocol instances for virtio-blk devices.

  The implementation is basic:

  - No attach/detach (ie. removable media).

  - Although the non-blocking interfaces of EFI_BLOCK_IO2_PROTOCOL could be a
    good match for multiple in-flight virtio-blk requests, we stick to
    synchronous requests and EFI_BLOCK_IO_PROTOCOL for now.

  Copyright (C) 2012, Red Hat, Inc.
  Copyright (c) 2012 - 2016, Intel Corporation. All rights reserved.<BR>
  Copyright (c) 2017, AMD Inc, All rights reserved.<BR>

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

#include "VirtioBlkCommon.h"
#include "VirtioBlk.h"
#include "VirtioLib.h"
#include "VirtioBlkDevice.h"

#ifndef SIZE_1GB
#define SIZE_1GB 0x40000000
#endif

#define EFI_BLOCK_IO_PROTOCOL_REVISION3 0x00020031

/**

  Convenience macros to read and write region 0 IO space elements of the
  virtio-blk device, for configuration purposes.

  The following macros make it possible to specify only the "core parameters"
  for such accesses and to derive the rest. By the time VIRTIO_CFG_WRITE()
  returns, the transaction will have been completed.

  @param[in] Dev       Pointer to the VBLK_DEV structure whose VirtIo space
                       we're accessing. Dev->VirtIo must be valid.

  @param[in] Field     A field name from VBLK_HDR, identifying the virtio-blk
                       configuration item to access.

  @param[in] Value     (VIRTIO_CFG_WRITE() only.) The value to write to the
                       selected configuration item.

  @param[out] Pointer  (VIRTIO_CFG_READ() only.) The object to receive the
                       value read from the configuration item. Its type must be
                       one of UINT8, UINT16, UINT32, UINT64.


  @return  Status code returned by Virtio->WriteDevice() /
           Virtio->ReadDevice().

**/

#define VIRTIO_CFG_WRITE(Dev, Field, Value)  ((Dev)->VirtIo->WriteDevice ( \
				(Dev)->VirtIo,             \
				OFFSET_OF_VBLK (Field),    \
				SIZE_OF_VBLK (Field),      \
				(Value)                    \
				))

#define VIRTIO_CFG_READ(Dev, Field, Pointer) ((Dev)->VirtIo->ReadDevice (  \
				(Dev)->VirtIo,             \
				OFFSET_OF_VBLK (Field),    \
				SIZE_OF_VBLK (Field),      \
				sizeof *(Pointer),         \
				(Pointer)                  \
				))

/**
  Divides a 64-bit unsigned integer by a 32-bit unsigned integer and generates
  a 32-bit unsigned remainder.

  This function divides the 64-bit unsigned value Dividend by the 32-bit
  unsigned value Divisor and generates a 32-bit remainder. This function
  returns the 32-bit unsigned remainder.

  If Divisor is 0, then ASSERT().

  @param  Dividend  A 64-bit unsigned value.
  @param  Divisor   A 32-bit unsigned value.

  @return Dividend % Divisor.

**/
static UINT32
EFIAPI
ModU64x32 (
	IN      UINT64                    Dividend,
	IN      UINT32                    Divisor
	)
{
	ASSERT (Divisor != 0);
	UINT64 temp1 = DivU64x32 (Dividend, Divisor, NULL);
	UINT64 temp2 = MultU64x32(temp1, Divisor);
	return (UINT32)(Dividend - temp2);
}

//
// UEFI Spec 2.3.1 + Errata C, 12.8 EFI Block I/O Protocol
// Driver Writer's Guide for UEFI 2.3.1 v1.01,
//   24.2 Block I/O Protocol Implementations
//
EFI_STATUS
EFIAPI
VirtioBlkReset (
	IN __attribute__((unused)) EFI_BLOCK_IO_PROTOCOL *This,
	IN __attribute__((unused)) BOOLEAN               ExtendedVerification
	)
{
	//
	// If we managed to initialize and install the driver, then the device is
	// working correctly.
	//
	return EFI_SUCCESS;
}

/**

  Verify correctness of the read/write (not flush) request submitted to the
  EFI_BLOCK_IO_PROTOCOL instance.

  This function provides most verification steps described in:

    UEFI Spec 2.3.1 + Errata C, 12.8 EFI Block I/O Protocol, 12.8 EFI Block I/O
    Protocol,
    - EFI_BLOCK_IO_PROTOCOL.ReadBlocks()
    - EFI_BLOCK_IO_PROTOCOL.WriteBlocks()

    Driver Writer's Guide for UEFI 2.3.1 v1.01,
    - 24.2.2. ReadBlocks() and ReadBlocksEx() Implementation
    - 24.2.3 WriteBlocks() and WriteBlockEx() Implementation

  Request sizes are limited to 1 GB (checked). This is not a practical
  limitation, just conformance to virtio-0.9.5, 2.3.2 Descriptor Table: "no
  descriptor chain may be more than 2^32 bytes long in total".

  Some Media characteristics are hardcoded in VirtioBlkInit() below (like
  non-removable media, no restriction on buffer alignment etc); we rely on
  those here without explicit mention.

  @param[in] Media               The EFI_BLOCK_IO_MEDIA characteristics for
                                 this driver instance, extracted from the
                                 underlying virtio-blk device at initialization
                                 time. We validate the request against this set
                                 of attributes.


  @param[in] Lba                 Logical Block Address: number of logical
                                 blocks to skip from the beginning of the
                                 device.

  @param[in] PositiveBufferSize  Size of buffer to transfer, in bytes. The
                                 caller is responsible to ensure this parameter
                                 is positive.

  @param[in] RequestIsWrite      TRUE iff data transfer goes from guest to
                                 device.


  @@return                       Validation result to be forwarded outwards by
                                 ReadBlocks() and WriteBlocks, as required by
                                 the specs above.

**/
STATIC
EFI_STATUS
EFIAPI
VerifyReadWriteRequest (
	IN  EFI_BLOCK_IO_MEDIA *Media,
	IN  EFI_LBA            Lba,
	IN  UINTN              PositiveBufferSize,
	IN  BOOLEAN            RequestIsWrite
	)
{
	UINTN BlockCount;

	ASSERT (PositiveBufferSize > 0);

	if (PositiveBufferSize > SIZE_1GB ||
	PositiveBufferSize % Media->BlockSize > 0) {
		return EFI_BAD_BUFFER_SIZE;
	}
	BlockCount = PositiveBufferSize / Media->BlockSize;

	//
	// Avoid unsigned wraparound on either side in the second comparison.
	//
	if (Lba > Media->LastBlock || BlockCount - 1 > Media->LastBlock - Lba) {
		return EFI_INVALID_PARAMETER;
	}

	if (RequestIsWrite && Media->ReadOnly) {
		return EFI_WRITE_PROTECTED;
	}

	return EFI_SUCCESS;
}




/**

  Format a read / write / flush request as three consecutive virtio
  descriptors, push them to the host, and poll for the response.

  This is the main workhorse function. Two use cases are supported, read/write
  and flush. The function may only be called after the request parameters have
  been verified by
  - specific checks in ReadBlocks() / WriteBlocks() / FlushBlocks(), and
  - VerifyReadWriteRequest() (for read/write only).

  Parameters handled commonly:

    @param[in] Dev             The virtio-blk device the request is targeted
                               at.

  Flush request:

    @param[in] Lba             Must be zero.

    @param[in] BufferSize      Must be zero.

    @param[in out] Buffer      Ignored by the function.

    @param[in] RequestIsWrite  Must be TRUE.

  Read/Write request:

    @param[in] Lba             Logical Block Address: number of logical blocks
                               to skip from the beginning of the device.

    @param[in] BufferSize      Size of buffer to transfer, in bytes. The caller
                               is responsible to ensure this parameter is
                               positive.

    @param[in out] Buffer      The guest side area to read data from the device
                               into, or write data to the device from.

    @param[in] RequestIsWrite  TRUE iff data transfer goes from guest to
                               device.

  Return values are common to both use cases, and are appropriate to be
  forwarded by the EFI_BLOCK_IO_PROTOCOL functions (ReadBlocks(),
  WriteBlocks(), FlushBlocks()).


  @retval EFI_SUCCESS          Transfer complete.

  @retval EFI_DEVICE_ERROR     Failed to notify host side via VirtIo write, or
                               unable to parse host response, or host response
                               is not VIRTIO_BLK_S_OK or failed to map Buffer
                               for a bus master operation.

**/

STATIC
EFI_STATUS
EFIAPI
SynchronousRequest (
	IN              VBLK_DEV *Dev,
	IN              EFI_LBA  Lba,
	IN              UINTN    BufferSize,
	IN OUT volatile VOID     *Buffer,
	IN              BOOLEAN  RequestIsWrite
	)
{
	UINT32                  BlockSize;
	volatile VIRTIO_BLK_REQ Request;
	volatile UINT8          *HostStatus;
	VOID                    *HostStatusBuffer;
	VOID                    *DataBuffer;
	DESC_INDICES            Indices;
	VOID                    *RequestMapping;
	VOID                    *StatusMapping;
	VOID                    *BufferMapping;
	EFI_PHYSICAL_ADDRESS    BufferDeviceAddress;
	EFI_PHYSICAL_ADDRESS    HostStatusDeviceAddress;
	EFI_PHYSICAL_ADDRESS    RequestDeviceAddress;
	EFI_STATUS              Status;
	EFI_STATUS              UnmapStatus;

	BlockSize = Dev->BlockIoMedia.BlockSize;

	//
	// ensured by VirtioBlkInit()
	//
	ASSERT (BlockSize > 0);
	ASSERT (BlockSize % 512 == 0);

	//
	// ensured by contract above, plus VerifyReadWriteRequest()
	//
	ASSERT (BufferSize % BlockSize == 0);

	//
	// Prepare virtio-blk request header, setting zero size for flush.
	// IO Priority is homogeneously 0.
	//
	Request.Type   = RequestIsWrite ?
		(BufferSize == 0 ? VIRTIO_BLK_T_FLUSH : VIRTIO_BLK_T_OUT) :
		VIRTIO_BLK_T_IN;
	Request.IoPrio = 0;
	Request.Sector = MultU64x32(Lba, BlockSize / 512);

	//DataBuffer
	Status = Dev->VirtIo->AllocateSharedPages (
		Dev->VirtIo,
		EFI_SIZE_TO_PAGES (BufferSize),
		&DataBuffer
		);
	if (EFI_ERROR (Status)) {
		return EFI_DEVICE_ERROR;
	}

	//
	// Copy write data from orignial buffer
	//
	if (RequestIsWrite)
		CopyMem(DataBuffer, (VOID*)Buffer, BufferSize);

	//
	// Host status is bi-directional (we preset with a value and expect the
	// device to update it). Allocate a host status buffer which can be mapped
	// to access equally by both processor and the device.
	//
	Status = Dev->VirtIo->AllocateSharedPages (
		Dev->VirtIo,
		EFI_SIZE_TO_PAGES (sizeof *HostStatus),
		&HostStatusBuffer
		);
	if (EFI_ERROR (Status)) {
		return EFI_DEVICE_ERROR;
	}

	HostStatus = HostStatusBuffer;

	//
	// Map virtio-blk request header (must be done after request header is
	// populated)
	//
	Status = VirtioMapAllBytesInSharedBuffer (
		Dev->VirtIo,
		VirtioOperationBusMasterRead,
		(VOID *) &Request,
		sizeof Request,
		&RequestDeviceAddress,
		&RequestMapping
	);
	if (EFI_ERROR (Status)) {
		Status = EFI_DEVICE_ERROR;
		goto FreeHostStatusBuffer;
	}

	//
	// Map data buffer
	//
	if (BufferSize > 0) {
	Status = VirtioMapAllBytesInSharedBuffer (
		Dev->VirtIo,
		(RequestIsWrite ?
		VirtioOperationBusMasterRead :
		VirtioOperationBusMasterWrite),
		(VOID *) DataBuffer,
		BufferSize,
		&BufferDeviceAddress,
		&BufferMapping
		);
	if (EFI_ERROR (Status)) {
		Status = EFI_DEVICE_ERROR;
		goto UnmapRequestBuffer;
	}
	}

	//
	// preset a host status for ourselves that we do not accept as success
	//
	*HostStatus = VIRTIO_BLK_S_IOERR;

	//
	// Map the Status Buffer with VirtioOperationBusMasterCommonBuffer so that
	// both processor and device can access it.
	//
	Status = VirtioMapAllBytesInSharedBuffer (
		Dev->VirtIo,
		VirtioOperationBusMasterCommonBuffer,
		HostStatusBuffer,
		sizeof *HostStatus,
		&HostStatusDeviceAddress,
		&StatusMapping
		);
	if (EFI_ERROR (Status)) {
		Status = EFI_DEVICE_ERROR;
		goto UnmapDataBuffer;
	}

	VirtioPrepare (&Dev->Ring, &Indices);

	//
	// ensured by VirtioBlkInit() -- this predicate, in combination with the
	// lock-step progress, ensures we don't have to track free descriptors.
	//
	ASSERT (Dev->Ring.QueueSize >= 3);

	//
	// virtio-blk header in first desc
	//
	VirtioAppendDesc (
		&Dev->Ring,
		RequestDeviceAddress,
		sizeof Request,
		VRING_DESC_F_NEXT,
		&Indices
		);

	//
	// data buffer for read/write in second desc
	//
	if (BufferSize > 0) {
		//
		// From virtio-0.9.5, 2.3.2 Descriptor Table:
		// "no descriptor chain may be more than 2^32 bytes long in total".
		//
		// The predicate is ensured by the call contract above (for flush), or
		// VerifyReadWriteRequest() (for read/write). It also implies that
		// converting BufferSize to UINT32 will not truncate it.
		//
		ASSERT (BufferSize <= SIZE_1GB);

		//
		// VRING_DESC_F_WRITE is interpreted from the host's point of view.
		//
		VirtioAppendDesc (
			&Dev->Ring,
			BufferDeviceAddress,
			(UINT32) BufferSize,
			VRING_DESC_F_NEXT | (RequestIsWrite ? 0 : VRING_DESC_F_WRITE),
			&Indices
			);
	}

	//
	// host status in last (second or third) desc
	//
	VirtioAppendDesc (
		&Dev->Ring,
		HostStatusDeviceAddress,
		sizeof *HostStatus,
		VRING_DESC_F_WRITE,
		&Indices
		);

	//
	// virtio-blk's only virtqueue is #0, called "requestq" (see Appendix D).
	//
	if (VirtioFlush (Dev->VirtIo, 0, &Dev->Ring, &Indices,
		NULL) == EFI_SUCCESS &&
		*HostStatus == VIRTIO_BLK_S_OK) {
		Status = EFI_SUCCESS;
		//DEBUG ((DEBUG_INFO, "%s:VirtioFlush Success ^_^\n",
		//__FUNCTION__));
	} else {
		Status = EFI_DEVICE_ERROR;
		DEBUG ((DEBUG_INFO, "%s:VirtioFlush fail !!!!!!!!!!!!\n",
		__FUNCTION__));
	}

	//
	// Copy read data to orignial buffer
	//
	if (!RequestIsWrite)
		CopyMem((VOID*)Buffer, DataBuffer, BufferSize);

	Dev->VirtIo->UnmapSharedBuffer (Dev->VirtIo, StatusMapping);

	UnmapDataBuffer:
	if (BufferSize > 0) {
		UnmapStatus = Dev->VirtIo->UnmapSharedBuffer (Dev->VirtIo, BufferMapping);
	if (EFI_ERROR (UnmapStatus) && !RequestIsWrite && !EFI_ERROR (Status)) {
		//
		// Data from the bus master may not reach the caller; fail the request.
		//
		Status = EFI_DEVICE_ERROR;
	}
	}

	UnmapRequestBuffer:
		Dev->VirtIo->UnmapSharedBuffer (Dev->VirtIo, RequestMapping);

	FreeHostStatusBuffer:
		Dev->VirtIo->FreeSharedPages (
			Dev->VirtIo,
			EFI_SIZE_TO_PAGES (sizeof *HostStatus),
			HostStatusBuffer
			);

	if (BufferSize > 0) {
		Dev->VirtIo->FreeSharedPages (
			Dev->VirtIo,
			EFI_SIZE_TO_PAGES (BufferSize),
			DataBuffer
			);
	}

	return Status;
}


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
	IN  __attribute__((unused)) UINT32                MediaId,
	IN  EFI_LBA               Lba,
	IN  UINTN                 BufferSize,
	OUT VOID                  *Buffer
	)
{
	VBLK_DEV   *Dev;
	EFI_STATUS Status;

	if (BufferSize == 0) {
		return EFI_SUCCESS;
	}

	Dev = VIRTIO_BLK_FROM_BLOCK_IO (This);
	Status = VerifyReadWriteRequest (
		&Dev->BlockIoMedia,
		Lba,
		BufferSize,
		FALSE               // RequestIsWrite
		);
	if (EFI_ERROR (Status)) {
		return Status;
	}

	return SynchronousRequest (
		Dev,
		Lba,
		BufferSize,
		Buffer,
		FALSE       // RequestIsWrite
		);
}

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
	IN __attribute__((unused)) UINT32                MediaId,
	IN EFI_LBA               Lba,
	IN UINTN                 BufferSize,
	IN VOID                  *Buffer
	)
{
	VBLK_DEV   *Dev;
	EFI_STATUS Status;

	if (BufferSize == 0) {
		return EFI_SUCCESS;
	}

	Dev = VIRTIO_BLK_FROM_BLOCK_IO (This);
	Status = VerifyReadWriteRequest (
		&Dev->BlockIoMedia,
		Lba,
		BufferSize,
		TRUE                // RequestIsWrite
	);
	if (EFI_ERROR (Status)) {
		return Status;
	}

	return SynchronousRequest (
		Dev,
		Lba,
		BufferSize,
		Buffer,
		TRUE        // RequestIsWrite
	);
}


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
	)
{
	VBLK_DEV *Dev;

	Dev = VIRTIO_BLK_FROM_BLOCK_IO (This);
	return Dev->BlockIoMedia.WriteCaching ?
		SynchronousRequest (
			Dev,
			0,    // Lba
			0,    // BufferSize
			NULL, // Buffer
			TRUE  // RequestIsWrite
			) :
		EFI_SUCCESS;
}

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
	)
{
	UINT8      NextDevStat;
	EFI_STATUS Status;

	UINT64     Features;
	UINT64     NumSectors;
	UINT32     BlockSize;
	UINT8      PhysicalBlockExp;
	UINT8      AlignmentOffset;
	UINT32     OptIoSize;
	UINT16     QueueSize;
	UINT64     RingBaseShift;

	PhysicalBlockExp = 0;
	AlignmentOffset = 0;
	OptIoSize = 0;

	//
	// Execute virtio-0.9.5, 2.2.1 Device Initialization Sequence.
	//
	NextDevStat = 0;             // step 1 -- reset device
	Status = Dev->VirtIo->SetDeviceStatus (Dev->VirtIo, NextDevStat);
	if (EFI_ERROR (Status)) {
		goto Failed;
	}

	NextDevStat |= VSTAT_ACK;    // step 2 -- acknowledge device presence
	Status = Dev->VirtIo->SetDeviceStatus (Dev->VirtIo, NextDevStat);
	if (EFI_ERROR (Status)) {
		goto Failed;
	}

	NextDevStat |= VSTAT_DRIVER; // step 3 -- we know how to drive it
	Status = Dev->VirtIo->SetDeviceStatus (Dev->VirtIo, NextDevStat);
	if (EFI_ERROR (Status)) {
		goto Failed;
	}

	//
	// Set Page Size - MMIO VirtIo Specific
	//
	Status = Dev->VirtIo->SetPageSize (Dev->VirtIo, EFI_PAGE_SIZE);
	if (EFI_ERROR (Status)) {
		goto Failed;
	}

	//
	// step 4a -- retrieve and validate features
	//
	Status = Dev->VirtIo->GetDeviceFeatures (Dev->VirtIo, &Features);
	if (EFI_ERROR (Status)) {
		goto Failed;
	}

	Status = VIRTIO_CFG_READ (Dev, Capacity, &NumSectors);
	if (EFI_ERROR (Status)) {
		goto Failed;
	}
	if (NumSectors == 0) {
		Status = EFI_UNSUPPORTED;
		goto Failed;
	}

	if (Features & VIRTIO_BLK_F_BLK_SIZE) {
		Status = VIRTIO_CFG_READ (Dev, BlkSize, &BlockSize);
		if (EFI_ERROR (Status)) {
			goto Failed;
		}
		if (BlockSize == 0 || BlockSize % 512 != 0 ||
		ModU64x32 (NumSectors, BlockSize / 512) != 0) {
			//
			// We can only handle a logical block consisting of whole sectors,
			// and only a disk composed of whole logical blocks.
			//
			Status = EFI_UNSUPPORTED;
			goto Failed;
		}
	}
	else {
		BlockSize = 512;
	}

	if (Features & VIRTIO_BLK_F_TOPOLOGY) {
		Status = VIRTIO_CFG_READ (Dev, Topology.PhysicalBlockExp,
			&PhysicalBlockExp);
		if (EFI_ERROR (Status)) {
			goto Failed;
		}
		if (PhysicalBlockExp >= 32) {
			Status = EFI_UNSUPPORTED;
			goto Failed;
		}

		Status = VIRTIO_CFG_READ (Dev, Topology.AlignmentOffset, &AlignmentOffset);
		if (EFI_ERROR (Status)) {
			goto Failed;
		}

		Status = VIRTIO_CFG_READ (Dev, Topology.OptIoSize, &OptIoSize);
		if (EFI_ERROR (Status)) {
			goto Failed;
		}
	}

	Features &= VIRTIO_BLK_F_BLK_SIZE | VIRTIO_BLK_F_TOPOLOGY | VIRTIO_BLK_F_RO |
		VIRTIO_BLK_F_FLUSH | VIRTIO_F_VERSION_1 |
		VIRTIO_F_IOMMU_PLATFORM;

	//
	// In virtio-1.0, feature negotiation is expected to complete before queue
	// discovery, and the device can also reject the selected set of features.
	//
	if (Dev->VirtIo->Revision >= VIRTIO_SPEC_REVISION (1, 0, 0)) {
		Status = Virtio10WriteFeatures (Dev->VirtIo, Features, &NextDevStat);
	if (EFI_ERROR (Status)) {
		goto Failed;
	}
	}

	//
	// step 4b -- allocate virtqueue
	//
	Status = Dev->VirtIo->SetQueueSel (Dev->VirtIo, 0);
	if (EFI_ERROR (Status)) {
		goto Failed;
	}
	Status = Dev->VirtIo->GetQueueNumMax (Dev->VirtIo, &QueueSize);
	if (EFI_ERROR (Status)) {
		goto Failed;
	}
	if (QueueSize < 3) { // SynchronousRequest() uses at most three descriptors
		Status = EFI_UNSUPPORTED;
		goto Failed;
	}

	Status = VirtioRingInit (Dev->VirtIo, QueueSize, &Dev->Ring);
	if (EFI_ERROR (Status)) {
		goto Failed;
	}

	//
	// If anything fails from here on, we must release the ring resources
	//
	Status = VirtioRingMap (
		Dev->VirtIo,
		&Dev->Ring,
		&RingBaseShift,
		&Dev->RingMap
		);
	if (EFI_ERROR (Status)) {
		goto ReleaseQueue;
	}

	//
	// Additional steps for MMIO: align the queue appropriately, and set the
	// size. If anything fails from here on, we must unmap the ring resources.
	//
	Status = Dev->VirtIo->SetQueueNum (Dev->VirtIo, QueueSize);
	if (EFI_ERROR (Status)) {
		goto UnmapQueue;
	}

	Status = Dev->VirtIo->SetQueueAlign (Dev->VirtIo, EFI_PAGE_SIZE);
	if (EFI_ERROR (Status)) {
		goto UnmapQueue;
	}

	//
	// step 4c -- Report GPFN (guest-physical frame number) of queue.
	//
	Status = Dev->VirtIo->SetQueueAddress (
		Dev->VirtIo,
		&Dev->Ring,
		RingBaseShift
		);
	if (EFI_ERROR (Status)) {
		goto UnmapQueue;
	}


	//
	// step 5 -- Report understood features.
	//
	if (Dev->VirtIo->Revision < VIRTIO_SPEC_REVISION (1, 0, 0)) {
		Features &= ~(UINT64)(VIRTIO_F_VERSION_1 | VIRTIO_F_IOMMU_PLATFORM);
		Status = Dev->VirtIo->SetGuestFeatures (Dev->VirtIo, Features);
		if (EFI_ERROR (Status)) {
			goto UnmapQueue;
		}
	}

	//
	// step 6 -- initialization complete
	//
	NextDevStat |= VSTAT_DRIVER_OK;
	Status = Dev->VirtIo->SetDeviceStatus (Dev->VirtIo, NextDevStat);
	if (EFI_ERROR (Status)) {
		goto UnmapQueue;
	}

	//
	// Populate the exported interface's attributes; see UEFI spec v2.4, 12.9 EFI
	// Block I/O Protocol.
	//
	Dev->BlockIo.Revision			= 0;
	Dev->BlockIo.Media			= &Dev->BlockIoMedia;
	Dev->BlockIo.Reset			= &VirtioBlkReset;
	Dev->BlockIo.ReadBlocks			= &VirtioBlkReadBlocks;
	Dev->BlockIo.WriteBlocks		= &VirtioBlkWriteBlocks;
	Dev->BlockIo.FlushBlocks		= &VirtioBlkFlushBlocks;
	Dev->BlockIoMedia.MediaId		= 0;
	Dev->BlockIoMedia.RemovableMedia	= FALSE;
	Dev->BlockIoMedia.MediaPresent		= TRUE;
	Dev->BlockIoMedia.LogicalPartition	= FALSE;
	Dev->BlockIoMedia.ReadOnly		= (BOOLEAN) ((Features & VIRTIO_BLK_F_RO) != 0);
	Dev->BlockIoMedia.WriteCaching		= (BOOLEAN) ((Features & VIRTIO_BLK_F_FLUSH) != 0);
	Dev->BlockIoMedia.BlockSize		= BlockSize;
	Dev->BlockIoMedia.IoAlign		= 0;
	Dev->BlockIoMedia.LastBlock		= DivU64x32 (NumSectors,
						BlockSize / 512, NULL) - 1;

	//DEBUG ((DEBUG_INFO, "%s: LbaSize=0x%x[B] NumBlocks=0x%x[Lba]\n",
	//	__FUNCTION__, Dev->BlockIoMedia.BlockSize,
	//	(UINT32)Dev->BlockIoMedia.LastBlock + 1));

	Dev->Signature = VBLK_SIG;

	if (Features & VIRTIO_BLK_F_TOPOLOGY) {
		Dev->BlockIo.Revision = EFI_BLOCK_IO_PROTOCOL_REVISION3;

		Dev->BlockIoMedia.LowestAlignedLba = AlignmentOffset;
		Dev->BlockIoMedia.LogicalBlocksPerPhysicalBlock = 1u << PhysicalBlockExp;
		Dev->BlockIoMedia.OptimalTransferLengthGranularity = OptIoSize;

		//DEBUG ((DEBUG_INFO, "%s: FirstAligned=0x%x[Lba] PhysBlkSize=0x%x[Lba]\n",
		//	__FUNCTION__, (UINT32)Dev->BlockIoMedia.LowestAlignedLba,
		//	Dev->BlockIoMedia.LogicalBlocksPerPhysicalBlock));
		//DEBUG ((DEBUG_INFO, "%s: OptimalTransferLengthGranularity=0x%x[Lba]\n",
		//	__FUNCTION__, Dev->BlockIoMedia.OptimalTransferLengthGranularity));
	}
	return EFI_SUCCESS;

	UnmapQueue:
		Dev->VirtIo->UnmapSharedBuffer (Dev->VirtIo, Dev->RingMap);

	ReleaseQueue:
		VirtioRingUninit (Dev->VirtIo, &Dev->Ring);

	Failed:
	//
	// Notify the host about our failure to setup: virtio-0.9.5, 2.2.2.1 Device
	// Status. VirtIo access failure here should not mask the original error.
		//
		NextDevStat |= VSTAT_FAILED;
		Dev->VirtIo->SetDeviceStatus (Dev->VirtIo, NextDevStat);

		return Status; // reached only via Failed above
}


/**

  Uninitialize the internals of a virtio-blk device that has been successfully
  set up with VirtioBlkInit().

  @param[in out]  Dev  The device to clean up.

**/
VOID
EFIAPI
VirtioBlkUninit (
	IN OUT VBLK_DEV *Dev
	)
{
	//
	// Reset the virtual device -- see virtio-0.9.5, 2.2.2.1 Device Status. When
	// VIRTIO_CFG_WRITE() returns, the host will have learned to stay away from
	// the old comms area.
	//
	Dev->VirtIo->SetDeviceStatus (Dev->VirtIo, 0);

	Dev->VirtIo->UnmapSharedBuffer (Dev->VirtIo, Dev->RingMap);
	VirtioRingUninit (Dev->VirtIo, &Dev->Ring);

	SetMem (&Dev->BlockIo,      sizeof Dev->BlockIo,      0x00);
	SetMem (&Dev->BlockIoMedia, sizeof Dev->BlockIoMedia, 0x00);
}
