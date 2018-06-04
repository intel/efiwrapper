/* @file
 *
 * This driver produces Protocol instances for virtio-rpmb devices.
 *
 * Copyright (C) 2012, Red Hat, Inc.
 * Copyright (c) 2012 - 2018, Intel Corporation. All rights reserved.<BR>
 * Copyright (c) 2017, AMD Inc, All rights reserved.<BR>
 *
 * This program and the accompanying materials are licensed and made available
 * under the terms and conditions of the BSD License which accompanies this
 * distribution. The full text of the license may be found at
 * http://opensource.org/licenses/bsd-license.php
 *
 * THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS, WITHOUT
 * WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
 *
 */

#include <kconfig.h>
#include <libpayload-config.h>
#include <libpayload.h>
#include <ewlib.h>
#include <efilib.h>

#include "VirtioDeviceCommon.h"
#include "VirtioRpmb.h"
#include "VirtioLib.h"
#include "VirtioRpmbDevice.h"
#include "VirtioRpmbAccessLib.h"

#define QUEUE_INDEX 0
#define SEQ_CMD_MAX	3	/*support up to 3 cmds*/

static RPMB_DATA_FRAME *VrpmbGetFrameAddress(VOID *virtio_buffer, UINT32 index)
{
	VIRTIO_RPMB_IOCTL_SEQ_DATA *seq_data = NULL;
	VIRTIO_RPMB_CMD *cmds, *cmd;
	RPMB_DATA_FRAME *frames;
	UINT32 number_cmds, offset = 0;
	UINT32 i;

	if (!virtio_buffer || index > MAX_COMMAND_RPMB)
		return NULL;

	seq_data = (VIRTIO_RPMB_IOCTL_SEQ_DATA *)virtio_buffer;
	number_cmds = seq_data->n_cmds;
	if (number_cmds > MAX_COMMAND_RPMB)
		return NULL;

	cmds = (VIRTIO_RPMB_CMD *)&seq_data->cmds[0];
	if (!cmds)
		return NULL;

	frames = (RPMB_DATA_FRAME *)&seq_data->cmds[number_cmds + 1];
	if (!frames)
		return NULL;

	for (i = 0; i < index; i++) {
		cmd = &cmds[i];
		if (!cmd)
			return NULL;
		offset += cmd->n_rpmb_frame;
	}

	return (RPMB_DATA_FRAME *)&frames[offset];
}

STATIC
EFI_STATUS
EFIAPI
VirtioRpmbAddVirtqueue(
	IN              VRPMB_DEV *Dev,
	IN              UINT32 BufferSize,
	IN OUT          VOID *Buffer,
	BOOLEAN	        RequestIsWrite
	)
{
	EFI_STATUS              Status = EFI_SUCCESS;
	DESC_INDICES            Indices;
	VOID                    *IoctlCmdMapping;
	VOID                    *SeqCommandDataMapping;
	VOID                    *RpmbFrameMapping[SEQ_CMD_MAX];
	EFI_PHYSICAL_ADDRESS    IoctlCmdDeviceAddress;
	EFI_PHYSICAL_ADDRESS    RpmbFrameDeviceAddress[SEQ_CMD_MAX];
	EFI_PHYSICAL_ADDRESS    SeqCommandDataDeviceAddress;
	BOOLEAN                 DataBufferIsMapped;
	VIRTIO_RPMB_IOCTL_SEQ_DATA *SeqCmdData = NULL;
	UINT64                  CmdNumber;
	VIRTIO_RPMB_IOC         IoctlCommand;
	VIRTIO_RPMB_CMD         *cmds;
	UINTN                   i;

	if (Dev == NULL || Buffer == NULL)
		return EFI_INVALID_PARAMETER;

	SeqCmdData = (VIRTIO_RPMB_IOCTL_SEQ_DATA *)malloc(BufferSize);
	if (!SeqCmdData)
		return EFI_OUT_OF_RESOURCES;

	memcpy(SeqCmdData, Buffer, BufferSize);

	CmdNumber = SeqCmdData->n_cmds;
	memset(&IoctlCommand, 0, sizeof(IoctlCommand));

	IoctlCommand.IoctlCmd = RPMB_IOC_SEQ_CMD;
	IoctlCommand.Result = 0;
	IoctlCommand.Target = 0;

	cmds = &SeqCmdData->cmds[0];
	for (i = 0; i < CmdNumber; i++) {
		cmds[i].rpmb_flag   = SeqCmdData->cmds[i].rpmb_flag;
		cmds[i].n_rpmb_frame = SeqCmdData->cmds[i].n_rpmb_frame;
		cmds[i].addr_rpmb_frame = VrpmbGetFrameAddress(Buffer, i);
		memcpy(cmds[i].addr_rpmb_frame, SeqCmdData->cmds[i].addr_rpmb_frame,
			sizeof(RPMB_DATA_FRAME) * (SeqCmdData->cmds[i].n_rpmb_frame ? : 1));
	}

	//
	// Map ioctl cmd
	//
	Status = VirtioMapAllBytesInSharedBuffer(
			Dev->VirtIo,
			VirtioOperationBusMasterRead,
			(VOID *) &IoctlCommand,
			sizeof(IoctlCommand),
			&IoctlCmdDeviceAddress,
			&IoctlCmdMapping
			);
	if (EFI_ERROR(Status)) {
		Status = EFI_DEVICE_ERROR;
		goto UnmapIoctlCmdBuffer;
	}

	//
	// Map seq cmds Data
	//
	Status = VirtioMapAllBytesInSharedBuffer(
			Dev->VirtIo,
			VirtioOperationBusMasterRead,
			(VOID *) SeqCmdData,
			sizeof(VIRTIO_RPMB_IOCTL_SEQ_DATA),
			&SeqCommandDataDeviceAddress,
			&SeqCommandDataMapping
			);
	if (EFI_ERROR(Status)) {
		Status = EFI_DEVICE_ERROR;
		goto UnmapSeqCmdBuffer;
	}

	//
	// Map rpmb frame
	//
	for (i = 0; i < CmdNumber; i++) {
		Status = VirtioMapAllBytesInSharedBuffer(
			Dev->VirtIo,
			(RequestIsWrite ?
			VirtioOperationBusMasterRead :
			VirtioOperationBusMasterWrite),
			(VOID *) cmds[i].addr_rpmb_frame,
			sizeof(RPMB_DATA_FRAME) * (SeqCmdData->cmds[i].n_rpmb_frame ? : 1),
			&RpmbFrameDeviceAddress[i],
			&RpmbFrameMapping[i]
			);
		if (EFI_ERROR(Status)) {
			Status = EFI_DEVICE_ERROR;
			goto UnmapFrameBuffer;
		}
	}

	VirtioPrepare(&Dev->Ring, &Indices);

	//
	// ensured by VirtioRpmbInit() -- this predicate, in combination with the
	// lock-step progress, ensures we don't have to track free descriptors.
	//
	ASSERT(Dev->Ring.QueueSize >= 5);

	//
	// virtio-rpmb ioctl cmd
	//
	VirtioAppendDesc(
		&Dev->Ring,
		IoctlCmdDeviceAddress,
		sizeof(IoctlCommand),
		VRING_DESC_F_NEXT,
		&Indices
		);

	//
	// virtio-rpmb seq cmd
	//
	VirtioAppendDesc(
		&Dev->Ring,
		SeqCommandDataDeviceAddress,
		sizeof(VIRTIO_RPMB_IOCTL_SEQ_DATA),
		VRING_DESC_F_NEXT,
		&Indices
		);

	//
	// enqueue data
	//
	if (BufferSize > 0) {
		//
		// VRING_DESC_F_WRITE is interpreted from the host's point of view.
		//
		for (i = 0; i < CmdNumber; i++) {
			if (i < (CmdNumber - 1))
				VirtioAppendDesc(
				&Dev->Ring,
				RpmbFrameDeviceAddress[i],
				sizeof(RPMB_DATA_FRAME),
				VRING_DESC_F_NEXT,
				&Indices
				);
			else
				VirtioAppendDesc(
				&Dev->Ring,
				RpmbFrameDeviceAddress[i],
				sizeof(RPMB_DATA_FRAME),
				VRING_DESC_F_WRITE,
				&Indices
				);
		}
	}

	DataBufferIsMapped = TRUE;

	if (VirtioFlush(Dev->VirtIo, QUEUE_INDEX, &Dev->Ring,
			&Indices, NULL) != EFI_SUCCESS)
		Status = EFI_DEVICE_ERROR;

	if (IoctlCommand.Result != 0)
		Status = EFI_DEVICE_ERROR;
	else
		Status = EFI_SUCCESS;

UnmapFrameBuffer:
	if (DataBufferIsMapped) {
		for (i = 0; i < CmdNumber; i++)
			Dev->VirtIo->UnmapSharedBuffer(Dev->VirtIo, RpmbFrameMapping[i]);
	}

UnmapSeqCmdBuffer:
	Dev->VirtIo->UnmapSharedBuffer(Dev->VirtIo, SeqCommandDataMapping);

UnmapIoctlCmdBuffer:
	Dev->VirtIo->UnmapSharedBuffer(Dev->VirtIo, IoctlCmdMapping);

	free(SeqCmdData);
	return Status;
}

EFI_STATUS
EFIAPI
VirtioRpmbSentData(
	IN VRPMB_DEV              *Dev,
	IN VOID                   *Buffer,
	IN UINT32                 BufferSize
	)
{
	UINT32 MaxSize;
	EFI_STATUS Status = EFI_SUCCESS;

	if (BufferSize == 0)
		return EFI_SUCCESS;

	MaxSize = 0x400 << EFI_PAGE_SHIFT;
	if (BufferSize > MaxSize)
		return EFI_INVALID_PARAMETER;

	if (!Dev)
		return EFI_INVALID_PARAMETER;

	Status = VirtioRpmbAddVirtqueue(
			Dev,
			BufferSize,
			Buffer,
			TRUE		 // RequestIsWrite
			);

	return Status;
}

/*
 *
 * Set up all virtio-rpmb aspects of this driver for the specified
 * device.
 *
 * @param[in out] Dev  The driver instance to configure. The caller is
 *		responsible for Dev->VirtIo's validity (ie. working IO
 *		access to the underlying virtio-rpmb device).
 *
 * @retval EFI_SUCCESS      Setup complete.
 *
 * @retval EFI_UNSUPPORTED  The driver is unable to work with the virtio ring.
 *
 * @return                  Error codes from VirtioRingInit() or
 *                          VIRTIO_CFG_READ() / VIRTIO_CFG_WRITE or
 *                          VirtioRingMap().
 */
EFI_STATUS
EFIAPI
VirtioRpmbInit(
	IN OUT           VRPMB_DEV *Dev
	)
{
	UINT8      NextDevStat;
	EFI_STATUS Status;

	UINT64     Features;
	UINT16     QueueSize;
	UINT64     RingBaseShift;

	//
	// Execute virtio-0.9.5, 2.2.1 Device Initialization Sequence.
	//
	NextDevStat = 0;             // step 1 -- reset device
	Status = Dev->VirtIo->SetDeviceStatus(Dev->VirtIo, NextDevStat);
	if (EFI_ERROR(Status))
		goto Failed;

	NextDevStat |= VSTAT_ACK;    // step 2 -- acknowledge device presence
	Status = Dev->VirtIo->SetDeviceStatus(Dev->VirtIo, NextDevStat);
	if (EFI_ERROR(Status))
		goto Failed;

	NextDevStat |= VSTAT_DRIVER; // step 3 -- we know how to drive it
	Status = Dev->VirtIo->SetDeviceStatus(Dev->VirtIo, NextDevStat);
	if (EFI_ERROR(Status))
		goto Failed;

	//
	// Set Page Size - MMIO VirtIo Specific
	//
	Status = Dev->VirtIo->SetPageSize(Dev->VirtIo, EFI_PAGE_SIZE);
	if (EFI_ERROR(Status))
		goto Failed;

	//
	// step 4a -- retrieve and validate features
	//
	Status = Dev->VirtIo->GetDeviceFeatures(Dev->VirtIo, &Features);
	if (EFI_ERROR(Status))
		goto Failed;

	//
	// In virtio-1.0, feature negotiation is expected to complete before queue
	// discovery, and the device can also reject the selected set of features.
	//
	if (Dev->VirtIo->Revision >= VIRTIO_SPEC_REVISION(1, 0, 0)) {
		Status = Virtio10WriteFeatures(Dev->VirtIo, Features, &NextDevStat);
	if (EFI_ERROR(Status))
		goto Failed;
	}

	//
	// step 4b -- allocate virtqueue, just use #0
	//
	Status = Dev->VirtIo->SetQueueSel(Dev->VirtIo, QUEUE_INDEX);
	if (EFI_ERROR(Status))
		goto Failed;

	Status = Dev->VirtIo->GetQueueNumMax(Dev->VirtIo, &QueueSize);
	if (EFI_ERROR(Status))
		goto Failed;

	//
	// VirtioRpmbAddVirtqueue() uses one descriptor.
	//
	if (QueueSize < 1) {
		Status = EFI_UNSUPPORTED;
		goto Failed;
	}

	Status = VirtioRingInit(Dev->VirtIo, QueueSize, &Dev->Ring);
	if (EFI_ERROR(Status))
		goto Failed;

	//
	// If anything fails from here on, we must release the ring resources.
	//
	Status = VirtioRingMap(
		Dev->VirtIo,
		&Dev->Ring,
		&RingBaseShift,
		&Dev->RingMap
		);
	if (EFI_ERROR(Status))
		goto ReleaseQueue;

	//
	// Additional steps for MMIO: align the queue appropriately, and set the
	// size. If anything fails from here on, we must unmap the ring resources.
	//
	Status = Dev->VirtIo->SetQueueNum(Dev->VirtIo, QueueSize);
	if (EFI_ERROR(Status))
		goto UnmapQueue;

	Status = Dev->VirtIo->SetQueueAlign(Dev->VirtIo, EFI_PAGE_SIZE);
	if (EFI_ERROR(Status))
		goto UnmapQueue;

	//
	// step 4c -- Report GPFN (guest-physical frame number) of queue.
	//
	Status = Dev->VirtIo->SetQueueAddress(
		Dev->VirtIo,
		&Dev->Ring,
		RingBaseShift
		);
	if (EFI_ERROR(Status))
		goto UnmapQueue;

	//
	// step 5 -- Report understood features.
	//
	if (Dev->VirtIo->Revision < VIRTIO_SPEC_REVISION(1, 0, 0)) {
		Features &= ~(UINT64)(VIRTIO_F_VERSION_1 | VIRTIO_F_IOMMU_PLATFORM);
		Status = Dev->VirtIo->SetGuestFeatures(Dev->VirtIo, Features);
		if (EFI_ERROR(Status))
			goto UnmapQueue;
	}

	//
	// step 6 -- initialization complete
	//
	NextDevStat |= VSTAT_DRIVER_OK;
	Status = Dev->VirtIo->SetDeviceStatus(Dev->VirtIo, NextDevStat);
	if (EFI_ERROR(Status))
		goto UnmapQueue;

	Dev->Signature = VRPMB_SIG;

	return EFI_SUCCESS;

UnmapQueue:
	Dev->VirtIo->UnmapSharedBuffer(Dev->VirtIo, Dev->RingMap);

ReleaseQueue:
	VirtioRingUninit(Dev->VirtIo, &Dev->Ring);

Failed:
	//
	// Notify the host about our failure to setup: virtio-0.9.5, 2.2.2.1 Device
	// Status. VirtIo access failure here should not mask the original error.
	//
	NextDevStat |= VSTAT_FAILED;
	Dev->VirtIo->SetDeviceStatus(Dev->VirtIo, NextDevStat);

	// reached only via Failed above
	return Status;
}

/*
 *
 * Uninitialize the internals of a virtio-rpmb device that has been successfully
 * set up with VirtioRpmbInit().
 *
 * @param[in out]  Dev  The device to clean up.
 *
 */
VOID
EFIAPI
VirtioRpmbUninit(
	IN OUT           VRPMB_DEV *Dev
	)
{
	//
	// Reset the virtual device -- see virtio-0.9.5, 2.2.2.1 Device Status. When
	// VIRTIO_CFG_WRITE() returns, the host will have learned to stay away from
	// the old comms area.
	//
	Dev->VirtIo->SetDeviceStatus(Dev->VirtIo, 0);

	Dev->VirtIo->UnmapSharedBuffer(Dev->VirtIo, Dev->RingMap);
	VirtioRingUninit(Dev->VirtIo, &Dev->Ring);

	SetMem(&Dev->PassThru, sizeof(Dev->PassThru), 0x00);
}
