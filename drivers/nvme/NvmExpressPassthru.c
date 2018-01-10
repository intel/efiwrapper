/** @file
  NvmExpressDxe driver is used to manage non-volatile memory subsystem which follows
  NVM Express specification.

  (C) Copyright 2014 Hewlett-Packard Development Company, L.P.<BR>
  Copyright (c) 2013 - 2016, Intel Corporation. All rights reserved.<BR>
  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php.

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <kconfig.h>
#include <libpayload-config.h>
#include <stdlib.h>
#include <efi.h>
#include <efidebug.h>

#include "NvmExpress.h"

#ifndef MSG_NVME_NAMESPACE_DP
#define MSG_NVME_NAMESPACE_DP     0x17
#endif

UINTN NanoSecondDelay (UINTN NanoSeconds)
{
	ndelay(NanoSeconds);
	return 0;
}

/**
  Allocates one or more 4KB pages of type EfiBootServicesData.

  Allocates the number of 4KB pages of type EfiBootServicesData and returns a pointer to the
  allocated buffer.  The buffer returned is aligned on a 4KB boundary.  If Pages is 0, then NULL
  is returned.  If there is not enough memory remaining to satisfy the request, then NULL is
  returned.

  @param  Pages                 The number of 4 KB pages to allocate.

  @return A pointer to the allocated buffer or NULL if allocation fails.

**/
VOID *EFIAPI nvme_alloc_pages (IN UINTN Pages)
{
	void *ptr;
	ptr = memalign (EFI_PAGE_SIZE, Pages << EFI_PAGE_SHIFT);
	if (ptr != NULL)
		ZeroMem (ptr, Pages << EFI_PAGE_SHIFT);

	return ptr;
}


/**
  Frees one or more 4KB pages that were previously allocated with one of the page allocation
  functions in the Memory Allocation Library.

  Frees the number of 4KB pages specified by Pages from the buffer specified by Buffer.  Buffer
  must have been allocated on a previous call to the page allocation services of the Memory
  Allocation Library.  If it is not possible to free allocated pages, then this function will
  perform no actions.

  If Buffer was not allocated with a page allocation function in the Memory Allocation Library,
  then ASSERT().
  If Pages is zero, then ASSERT().

  @param  Buffer                The pointer to the buffer of pages to free.
  @param  Pages                 The number of 4 KB pages to free.

**/
VOID EFIAPI nvme_free_pages (IN VOID   *Buffer)
{
	free(Buffer);
}


/**
  Dump the execution status from a given completion queue entry.

  @param[in]     Cq               A pointer to the NVME_CQ item.

**/
VOID
NvmeDumpStatus (
  IN NVME_CQ             *Cq
  )
{
	DEBUG_NVME ((EFI_D_VERBOSE, "Dump NVMe Completion Entry Status from [0x%x]:\n", Cq));
	DEBUG_NVME ((EFI_D_VERBOSE, "  SQ Identifier : [0x%x], Phase Tag : [%d], Cmd Identifier : [0x%x]\n", Cq->Sqid, Cq->Pt, Cq->Cid));
	DEBUG_NVME ((EFI_D_VERBOSE, "  NVMe Cmd Execution Result - "));

	switch (Cq->Sct) {
	case 0x0:
		switch (Cq->Sc) {
		case 0x0:
			DEBUG_NVME ((EFI_D_VERBOSE, "Successful Completion\n"));
			break;
		case 0x1:
			DEBUG_NVME ((EFI_D_VERBOSE, "Invalid Command Opcode\n"));
			break;
		case 0x2:
			DEBUG_NVME ((EFI_D_VERBOSE, "Invalid Field in Command\n"));
			break;
		case 0x3:
			DEBUG_NVME ((EFI_D_VERBOSE, "Command ID Conflict\n"));
			break;
		case 0x4:
			DEBUG_NVME ((EFI_D_VERBOSE, "Data Transfer Error\n"));
			break;
		case 0x5:
			DEBUG_NVME ((EFI_D_VERBOSE, "Commands Aborted due to Power Loss Notification\n"));
			break;
		case 0x6:
			DEBUG_NVME ((EFI_D_VERBOSE, "Internal Device Error\n"));
			break;
		case 0x7:
			DEBUG_NVME ((EFI_D_VERBOSE, "Command Abort Requested\n"));
			break;
		case 0x8:
			DEBUG_NVME ((EFI_D_VERBOSE, "Command Aborted due to SQ Deletion\n"));
			break;
		case 0x9:
			DEBUG_NVME ((EFI_D_VERBOSE, "Command Aborted due to Failed Fused Command\n"));
			break;
		case 0xA:
			DEBUG_NVME ((EFI_D_VERBOSE, "Command Aborted due to Missing Fused Command\n"));
			break;
		case 0xB:
			DEBUG_NVME ((EFI_D_VERBOSE, "Invalid Namespace or Format\n"));
			break;
		case 0xC:
			DEBUG_NVME ((EFI_D_VERBOSE, "Command Sequence Error\n"));
			break;
		case 0xD:
			DEBUG_NVME ((EFI_D_VERBOSE, "Invalid SGL Last Segment Descriptor\n"));
			break;
		case 0xE:
			DEBUG_NVME ((EFI_D_VERBOSE, "Invalid Number of SGL Descriptors\n"));
			break;
		case 0xF:
			DEBUG_NVME ((EFI_D_VERBOSE, "Data SGL Length Invalid\n"));
			break;
		case 0x10:
			DEBUG_NVME ((EFI_D_VERBOSE, "Metadata SGL Length Invalid\n"));
			break;
		case 0x11:
			DEBUG_NVME ((EFI_D_VERBOSE, "SGL Descriptor Type Invalid\n"));
			break;
		case 0x80:
			DEBUG_NVME ((EFI_D_VERBOSE, "LBA Out of Range\n"));
			break;
		case 0x81:
			DEBUG_NVME ((EFI_D_VERBOSE, "Capacity Exceeded\n"));
			break;
		case 0x82:
			DEBUG_NVME ((EFI_D_VERBOSE, "Namespace Not Ready\n"));
			break;
		case 0x83:
			DEBUG_NVME ((EFI_D_VERBOSE, "Reservation Conflict\n"));
			break;
		}
		break;

	case 0x1:
		switch (Cq->Sc) {
		case 0x0:
			DEBUG_NVME ((EFI_D_VERBOSE, "Completion Queue Invalid\n"));
			break;
		case 0x1:
			DEBUG_NVME ((EFI_D_VERBOSE, "Invalid Queue Identifier\n"));
			break;
		case 0x2:
			DEBUG_NVME ((EFI_D_VERBOSE, "Maximum Queue Size Exceeded\n"));
			break;
		case 0x3:
			DEBUG_NVME ((EFI_D_VERBOSE, "Abort Command Limit Exceeded\n"));
			break;
		case 0x5:
			DEBUG_NVME ((EFI_D_VERBOSE, "Asynchronous Event Request Limit Exceeded\n"));
			break;
		case 0x6:
			DEBUG_NVME ((EFI_D_VERBOSE, "Invalid Firmware Slot\n"));
			break;
		case 0x7:
			DEBUG_NVME ((EFI_D_VERBOSE, "Invalid Firmware Image\n"));
			break;
		case 0x8:
			DEBUG_NVME ((EFI_D_VERBOSE, "Invalid Interrupt Vector\n"));
			break;
		case 0x9:
			DEBUG_NVME ((EFI_D_VERBOSE, "Invalid Log Page\n"));
			break;
		case 0xA:
			DEBUG_NVME ((EFI_D_VERBOSE, "Invalid Format\n"));
			break;
		case 0xB:
			DEBUG_NVME ((EFI_D_VERBOSE, "Firmware Application Requires Conventional Reset\n"));
			break;
		case 0xC:
			DEBUG_NVME ((EFI_D_VERBOSE, "Invalid Queue Deletion\n"));
			break;
		case 0xD:
			DEBUG_NVME ((EFI_D_VERBOSE, "Feature Identifier Not Saveable\n"));
			break;
		case 0xE:
			DEBUG_NVME ((EFI_D_VERBOSE, "Feature Not Changeable\n"));
			break;
		case 0xF:
			DEBUG_NVME ((EFI_D_VERBOSE, "Feature Not Namespace Specific\n"));
			break;
		case 0x10:
			DEBUG_NVME ((EFI_D_VERBOSE, "Firmware Application Requires NVM Subsystem Reset\n"));
			break;
		case 0x80:
			DEBUG_NVME ((EFI_D_VERBOSE, "Conflicting Attributes\n"));
			break;
		case 0x81:
			DEBUG_NVME ((EFI_D_VERBOSE, "Invalid Protection Information\n"));
			break;
		case 0x82:
			DEBUG_NVME ((EFI_D_VERBOSE, "Attempted Write to Read Only Range\n"));
			break;
		}
		break;

	case 0x2:
		switch (Cq->Sc) {
		case 0x80:
			DEBUG_NVME ((EFI_D_VERBOSE, "Write Fault\n"));
			break;
		case 0x81:
			DEBUG_NVME ((EFI_D_VERBOSE, "Unrecovered Read Error\n"));
			break;
		case 0x82:
			DEBUG_NVME ((EFI_D_VERBOSE, "End-to-end Guard Check Error\n"));
			break;
		case 0x83:
			DEBUG_NVME ((EFI_D_VERBOSE, "End-to-end Application Tag Check Error\n"));
			break;
		case 0x84:
			DEBUG_NVME ((EFI_D_VERBOSE, "End-to-end Reference Tag Check Error\n"));
			break;
		case 0x85:
			DEBUG_NVME ((EFI_D_VERBOSE, "Compare Failure\n"));
			break;
		case 0x86:
			DEBUG_NVME ((EFI_D_VERBOSE, "Access Denied\n"));
			break;
		}
		break;

	default:
		break;
	}
}

/**
  Create PRP lists for data transfer which is larger than 2 memory pages.
  Note here we calcuate the number of required PRP lists and allocate them at one time.

  @param[in]     PhysicalAddr        The physical base address of data buffer.
  @param[in]     Pages               The number of pages to be transfered.
  @param[out]    PrpListHost         The host base address of PRP lists.
  @param[in,out] PrpListNo           The number of PRP List.

  @retval The pointer to the first PRP List of the PRP lists.

**/
VOID*
NvmeCreatePrpList (
	IN     EFI_PHYSICAL_ADDRESS         PhysicalAddr,
	IN     UINTN                        Pages,
	OUT    VOID                         **PrpListHost,
	IN OUT UINTN                        *PrpListNo
)
{
	UINTN                       PrpEntryNo;
	UINT64                      PrpListBase;
	UINTN                       PrpListIndex;
	UINTN                       PrpEntryIndex;
	UINT64                      Remainder;
	EFI_PHYSICAL_ADDRESS        PrpListPhyAddr;
	VOID                        *PrpList;

	//
	// The number of Prp Entry in a memory page.
	//
	PrpEntryNo = EFI_PAGE_SIZE / sizeof (UINT64);

	//
	// Calculate total PrpList number.
	//
	Remainder = Pages % (PrpEntryNo - 1);
	*PrpListNo = Pages / (PrpEntryNo - 1);
	if (*PrpListNo == 0)
		*PrpListNo = 1;
	else if ((Remainder != 0) && (Remainder != 1))
		*PrpListNo += 1;
	else if (Remainder == 1)
		Remainder = PrpEntryNo;
	else if (Remainder == 0)
		Remainder = PrpEntryNo - 1;

	PrpList = nvme_alloc_pages(*PrpListNo);
	if (PrpList == NULL) {
		DEBUG_NVME ((EFI_D_ERROR, "NvmeCreatePrpList: create PrpList failure!\n"));
		goto EXIT;
	}

	PrpListPhyAddr = (UINT64)(UINTN)PrpList;
	*PrpListHost = PrpList;

	//
	// Fill all PRP lists except of last one.
	//
	for (PrpListIndex = 0; PrpListIndex < *PrpListNo - 1; ++PrpListIndex) {
		PrpListBase = *(UINT64 *)PrpListHost + PrpListIndex * EFI_PAGE_SIZE;

		for (PrpEntryIndex = 0; PrpEntryIndex < PrpEntryNo; ++PrpEntryIndex) {
			if (PrpEntryIndex != PrpEntryNo - 1) {
				//
				// Fill all PRP entries except of last one.
				//
				*((UINT64 *)(UINTN)PrpListBase + PrpEntryIndex) = PhysicalAddr;
				PhysicalAddr += EFI_PAGE_SIZE;
			} else {
				//
				// Fill last PRP entries with next PRP List pointer.
				//
				*((UINT64 *)(UINTN)PrpListBase + PrpEntryIndex) = PrpListPhyAddr + (PrpListIndex + 1) * EFI_PAGE_SIZE;
			}
		}
	}

	//
	// Fill last PRP list.
	//
	PrpListBase = *(UINT64 *)PrpListHost + PrpListIndex * EFI_PAGE_SIZE;
	for (PrpEntryIndex = 0; PrpEntryIndex < Remainder; ++PrpEntryIndex) {
		*((UINT64 *)(UINTN)PrpListBase + PrpEntryIndex) = PhysicalAddr;
		PhysicalAddr += EFI_PAGE_SIZE;
	}

	nvme_free_pages(PrpList);
	return (VOID *)(UINTN)PrpListPhyAddr;

EXIT:
	nvme_free_pages(PrpList);
	return NULL;
}

/**
  Sends an NVM Express Command Packet to an NVM Express controller or namespace. This function supports
  both blocking I/O and non-blocking I/O. The blocking I/O functionality is required, and the non-blocking
  I/O functionality is optional.


  @param[in]     This                A pointer to the EFI_NVM_EXPRESS_PASS_THRU_PROTOCOL instance.
  @param[in]     NamespaceId         A 32 bit namespace ID as defined in the NVMe specification to which the NVM Express Command
                                     Packet will be sent.  A value of 0 denotes the NVM Express controller, a value of all 0xFF's
                                     (all bytes are 0xFF) in the namespace ID specifies that the command packet should be sent to
                                     all valid namespaces.
  @param[in,out] Packet              A pointer to the NVM Express Command Packet.
  @param[in]     Event               If non-blocking I/O is not supported then Event is ignored, and blocking I/O is performed.
                                     If Event is NULL, then blocking I/O is performed. If Event is not NULL and non-blocking I/O
                                     is supported, then non-blocking I/O is performed, and Event will be signaled when the NVM
                                     Express Command Packet completes.

  @retval EFI_SUCCESS                The NVM Express Command Packet was sent by the host. TransferLength bytes were transferred
                                     to, or from DataBuffer.
  @retval EFI_BAD_BUFFER_SIZE        The NVM Express Command Packet was not executed. The number of bytes that could be transferred
                                     is returned in TransferLength.
  @retval EFI_NOT_READY              The NVM Express Command Packet could not be sent because the controller is not ready. The caller
                                     may retry again later.
  @retval EFI_DEVICE_ERROR           A device error occurred while attempting to send the NVM Express Command Packet.
  @retval EFI_INVALID_PARAMETER      NamespaceId or the contents of EFI_NVM_EXPRESS_PASS_THRU_COMMAND_PACKET are invalid. The NVM
                                     Express Command Packet was not sent, so no additional status information is available.
  @retval EFI_UNSUPPORTED            The command described by the NVM Express Command Packet is not supported by the NVM Express
                                     controller. The NVM Express Command Packet was not sent so no additional status information
                                     is available.
  @retval EFI_TIMEOUT                A timeout occurred while waiting for the NVM Express Command Packet to execute.

**/
EFI_STATUS
EFIAPI
NvmExpressPassThru (
	IN     EFI_NVM_EXPRESS_PASS_THRU_PROTOCOL          *This,
	IN     UINT32                                      NamespaceId,
	IN OUT EFI_NVM_EXPRESS_PASS_THRU_COMMAND_PACKET    *Packet,
	IN     ASYNC_IO_CALL_BACK                          *Event
)
{
	NVME_CONTROLLER_PRIVATE_DATA   *Private;
	EFI_STATUS                     Status;
	NVME_SQ                        *Sq;
	NVME_CQ                        *Cq;
	UINT16                         QueueId;
	UINT32                         Bytes;
	UINT16                         Offset;
	EFI_PHYSICAL_ADDRESS           PhyAddr;
	UINT64                         *Prp;
	VOID                           *PrpListHost;
	UINTN                          PrpListNo;
	UINT32                         Attributes;
	UINT32                         IoAlign;
	UINT32                         MaxTransLen;
	UINT32                         Data;
	NVME_PASS_THRU_ASYNC_REQ       *AsyncRequest;
	UINT64                         TimeCount;

	//
	// check the data fields in Packet parameter.
	//
	if ((This == NULL) || (Packet == NULL))
		return EFI_INVALID_PARAMETER;

	if ((Packet->NvmeCmd == NULL) || (Packet->NvmeCompletion == NULL))
		return EFI_INVALID_PARAMETER;

	if (Packet->QueueType != NVME_ADMIN_QUEUE && Packet->QueueType != NVME_IO_QUEUE)
		return EFI_INVALID_PARAMETER;

	//
	// 'Attributes' with neither EFI_NVM_EXPRESS_PASS_THRU_ATTRIBUTES_LOGICAL nor
	// EFI_NVM_EXPRESS_PASS_THRU_ATTRIBUTES_PHYSICAL set is an illegal
	// configuration.
	//
	Attributes  = This->Mode->Attributes;
	if ((Attributes & (EFI_NVM_EXPRESS_PASS_THRU_ATTRIBUTES_PHYSICAL |
		EFI_NVM_EXPRESS_PASS_THRU_ATTRIBUTES_LOGICAL)) == 0)
		return EFI_INVALID_PARAMETER;

	//
	// Buffer alignment check for TransferBuffer & MetadataBuffer.
	//
	IoAlign     = This->Mode->IoAlign;
	if (IoAlign > 0 && (((UINTN) Packet->TransferBuffer & (IoAlign - 1)) != 0))
		return EFI_INVALID_PARAMETER;

	if (IoAlign > 0 && (((UINTN) Packet->MetadataBuffer & (IoAlign - 1)) != 0))
		return EFI_INVALID_PARAMETER;

	Private     = NVME_CONTROLLER_PRIVATE_DATA_FROM_PASS_THRU (This);

	//
	// Check NamespaceId is valid or not.
	//
	if ((NamespaceId > Private->ControllerData->Nn) &&
		(NamespaceId != (UINT32) -1))
		return EFI_INVALID_PARAMETER;


	//
	// Check whether TransferLength exceeds the maximum data transfer size.
	//
	if (Private->ControllerData->Mdts != 0) {
		MaxTransLen = (1 << (Private->ControllerData->Mdts)) *
			(1 << (Private->Cap.Mpsmin + 12));

		if (Packet->TransferLength > MaxTransLen) {
			Packet->TransferLength = MaxTransLen;
			return EFI_BAD_BUFFER_SIZE;
		}
	}

	PrpListHost = NULL;
	PrpListNo   = 0;
	Prp         = NULL;
	Status      = EFI_SUCCESS;

	if (Packet->QueueType == NVME_ADMIN_QUEUE)
		QueueId = 0;
	else {
		if (Event == NULL || *Event == NULL)
			QueueId = 1;
		else {
			QueueId = 2;

			//
			// Submission queue full check.
			//
			if ((Private->SqTdbl[QueueId].Sqt + 1) % (NVME_ASYNC_CSQ_SIZE + 1) ==
				Private->AsyncSqHead)
				return EFI_NOT_READY;
		}
	}

	Sq  = Private->SqBuffer[QueueId] + Private->SqTdbl[QueueId].Sqt;
	Cq  = Private->CqBuffer[QueueId] + Private->CqHdbl[QueueId].Cqh;

	if (Packet->NvmeCmd->Nsid != NamespaceId)
		return EFI_INVALID_PARAMETER;

	ZeroMem (Sq, sizeof (NVME_SQ));
	Sq->Opc  = (UINT8)Packet->NvmeCmd->Cdw0.Opcode;
	Sq->Fuse = (UINT8)Packet->NvmeCmd->Cdw0.FusedOperation;
	Sq->Cid  = Private->Cid[QueueId]++;
	Sq->Nsid = Packet->NvmeCmd->Nsid;

	//
	// Currently we only support PRP for data transfer, SGL is NOT supported.
	//
	if (Sq->Psdt != 0) {
		DEBUG_NVME ((EFI_D_ERROR, "NvmExpressPassThru: doesn't support SGL mechanism\n"));
		return EFI_UNSUPPORTED;
	}

	Sq->Prp[0] = (UINT64)(UINTN)Packet->TransferBuffer;
	//
	// If the NVMe cmd has data in or out, then mapping the user buffer to the PCI controller specific addresses.
	// Note here we don't handle data buffer for CreateIOSubmitionQueue and CreateIOCompletionQueue cmds because
	// these two cmds are special which requires their data buffer must support simultaneous access by both the
	// processor and a PCI Bus Master. It's caller's responsbility to ensure this.
	//
	if (((Sq->Opc & (BIT0 | BIT1)) != 0) && (Sq->Opc != NVME_ADMIN_CRIOCQ_CMD) && (Sq->Opc != NVME_ADMIN_CRIOSQ_CMD)) {
		if ((Packet->TransferLength == 0) || (Packet->TransferBuffer == NULL))
			return EFI_INVALID_PARAMETER;

		PhyAddr = (UINT64)(UINTN)Packet->TransferBuffer;

		Sq->Prp[0] = PhyAddr;
		Sq->Prp[1] = 0;

		if ((Packet->MetadataLength != 0) && (Packet->MetadataBuffer != NULL)) {
			PhyAddr = (UINT64)(UINTN)Packet->TransferBuffer;
			Sq->Mptr = PhyAddr;
		}
	}

	//
	// If the buffer size spans more than two memory pages (page size as defined in CC.Mps),
	// then build a PRP list in the second PRP submission queue entry.
	//
	Offset = ((UINT16)Sq->Prp[0]) & (EFI_PAGE_SIZE - 1);
	Bytes  = Packet->TransferLength;

	if ((Offset + Bytes) > (EFI_PAGE_SIZE * 2)) {
		//
		// Create PrpList for remaining data buffer.
		//
		PhyAddr = (Sq->Prp[0] + EFI_PAGE_SIZE) & ~(EFI_PAGE_SIZE - 1);
		Prp = NvmeCreatePrpList (PhyAddr, EFI_SIZE_TO_PAGES(Offset + Bytes) - 1, &PrpListHost, &PrpListNo);
		if (Prp == NULL)
			goto EXIT;

		Sq->Prp[1] = (UINT64)(UINTN)Prp;
	} else if ((Offset + Bytes) > EFI_PAGE_SIZE)
		Sq->Prp[1] = (Sq->Prp[0] + EFI_PAGE_SIZE) & ~(EFI_PAGE_SIZE - 1);

	if (Packet->NvmeCmd->Flags & CDW2_VALID)
		Sq->Rsvd2 = (UINT64)Packet->NvmeCmd->Cdw2;

	if (Packet->NvmeCmd->Flags & CDW3_VALID)
		Sq->Rsvd2 |= LShiftU64 ((UINT64)Packet->NvmeCmd->Cdw3, 32);

	if (Packet->NvmeCmd->Flags & CDW10_VALID)
		Sq->Payload.Raw.Cdw10 = Packet->NvmeCmd->Cdw10;

	if (Packet->NvmeCmd->Flags & CDW11_VALID)
		Sq->Payload.Raw.Cdw11 = Packet->NvmeCmd->Cdw11;

	if (Packet->NvmeCmd->Flags & CDW12_VALID)
		Sq->Payload.Raw.Cdw12 = Packet->NvmeCmd->Cdw12;

	if (Packet->NvmeCmd->Flags & CDW13_VALID)
		Sq->Payload.Raw.Cdw13 = Packet->NvmeCmd->Cdw13;

	if (Packet->NvmeCmd->Flags & CDW14_VALID)
		Sq->Payload.Raw.Cdw14 = Packet->NvmeCmd->Cdw14;

	if (Packet->NvmeCmd->Flags & CDW15_VALID)
		Sq->Payload.Raw.Cdw15 = Packet->NvmeCmd->Cdw15;

	//
	// Ring the submission queue doorbell.
	//
	if ((Event != NULL && *Event != NULL) && (QueueId != 0)) {
		Private->SqTdbl[QueueId].Sqt =
			(Private->SqTdbl[QueueId].Sqt + 1) % (NVME_ASYNC_CSQ_SIZE + 1);
	} else
		Private->SqTdbl[QueueId].Sqt ^= 1;

	Data = *((UINT32 *)&Private->SqTdbl[QueueId]);

	Status = NvmHcRwMmio(Private->NvmeHCBase, NVME_SQTDBL_OFFSET(QueueId, Private->Cap.Dstrd), FALSE, sizeof (Data), &Data);
	if (EFI_ERROR (Status))
		return Status;

	//
	// For non-blocking requests, return directly if the command is placed
	// in the submission queue.
	//
	if ((Event != NULL && *Event != NULL) && (QueueId != 0)) {
		AsyncRequest = MallocZero (sizeof (NVME_PASS_THRU_ASYNC_REQ));
		if (AsyncRequest == NULL) {
			Status = EFI_DEVICE_ERROR;
			goto EXIT;
		}

		AsyncRequest->Signature     = NVME_PASS_THRU_ASYNC_REQ_SIG;
		AsyncRequest->Packet        = Packet;
		AsyncRequest->CommandId     = Sq->Cid;
		AsyncRequest->CallerEvent   = *Event;
		AsyncRequest->PrpListNo     = PrpListNo;
		AsyncRequest->PrpListHost   = PrpListHost;

		InsertTailList (&Private->AsyncPassThruQueue, &AsyncRequest->Link);
		return EFI_SUCCESS;
	}


	// Wait for completion queue to get filled in. 100ns unit by EFI spec
	//
	TimeCount = (Packet->CommandTimeout) >> 7; //times = 128ns unit
	TimeCount *= 128;
	Status = EFI_TIMEOUT;
	while (TimeCount--) {
		if (Cq->Pt != Private->Pt[QueueId]) {
			Status = EFI_SUCCESS;
			break;
		}
		NanoSecondDelay(100);
	}

	//
	// Check the NVMe cmd execution result
	//
	if (Status != EFI_TIMEOUT) {
		if ((Cq->Sct == 0) && (Cq->Sc == 0))
			Status = EFI_SUCCESS;
		else {
			Status = EFI_DEVICE_ERROR;
			//
			// Copy the Respose Queue entry for this command to the callers response buffer
			//
			CopyMem(Packet->NvmeCompletion, Cq, sizeof(EFI_NVM_EXPRESS_COMPLETION));

			//
			// Dump every completion entry status for debugging.
			//
			DEBUG_CODE_BEGIN();
			NvmeDumpStatus(Cq);
			DEBUG_CODE_END();
		}
	}

	Private->CqHdbl[QueueId].Cqh ^= 1;
	if (Private->CqHdbl[QueueId].Cqh == 0)
		Private->Pt[QueueId] ^= 1;

	Data = *((UINT32 *)&Private->CqHdbl[QueueId]);

	Status = NvmHcRwMmio(Private->NvmeHCBase, NVME_CQHDBL_OFFSET(QueueId, Private->Cap.Dstrd), FALSE, sizeof (Data), &Data);

	//
	// For now, the code does not support the non-blocking feature for admin queue.
	// If Event is not NULL for admin queue, signal the caller's event here.
	//
	if (Event != NULL && *Event != NULL) {
		NVME_BLKIO2_SUBTASK                  *Subtask;
		ASSERT (QueueId == 0);
		Subtask = NVME_BLKIO2_SUBTASK_FROM_EVENT(Event);
		(*Event)(Subtask);
	}

EXIT:
	return Status;
}

/**
  Used to retrieve the next namespace ID for this NVM Express controller.

  The EFI_NVM_EXPRESS_PASS_THRU_PROTOCOL.GetNextNamespace() function retrieves the next valid
  namespace ID on this NVM Express controller.

  If on input the value pointed to by NamespaceId is 0xFFFFFFFF, then the first valid namespace
  ID defined on the NVM Express controller is returned in the location pointed to by NamespaceId
  and a status of EFI_SUCCESS is returned.

  If on input the value pointed to by NamespaceId is an invalid namespace ID other than 0xFFFFFFFF,
  then EFI_INVALID_PARAMETER is returned.

  If on input the value pointed to by NamespaceId is a valid namespace ID, then the next valid
  namespace ID on the NVM Express controller is returned in the location pointed to by NamespaceId,
  and EFI_SUCCESS is returned.

  If the value pointed to by NamespaceId is the namespace ID of the last namespace on the NVM
  Express controller, then EFI_NOT_FOUND is returned.

  @param[in]     This           A pointer to the EFI_NVM_EXPRESS_PASS_THRU_PROTOCOL instance.
  @param[in,out] NamespaceId    On input, a pointer to a legal NamespaceId for an NVM Express
                                namespace present on the NVM Express controller. On output, a
                                pointer to the next NamespaceId of an NVM Express namespace on
                                an NVM Express controller. An input value of 0xFFFFFFFF retrieves
                                the first NamespaceId for an NVM Express namespace present on an
                                NVM Express controller.

  @retval EFI_SUCCESS           The Namespace ID of the next Namespace was returned.
  @retval EFI_NOT_FOUND         There are no more namespaces defined on this controller.
  @retval EFI_INVALID_PARAMETER NamespaceId is an invalid value other than 0xFFFFFFFF.

**/
EFI_STATUS
EFIAPI
NvmExpressGetNextNamespace (
	IN     EFI_NVM_EXPRESS_PASS_THRU_PROTOCOL          *This,
	IN OUT UINT32                                      *NamespaceId
)
{
	NVME_CONTROLLER_PRIVATE_DATA     *Private;
	NVME_ADMIN_NAMESPACE_DATA        *NamespaceData;
	UINT32                           NextNamespaceId;
	EFI_STATUS                       Status;

	if ((This == NULL) || (NamespaceId == NULL))
		return EFI_INVALID_PARAMETER;

	NamespaceData = NULL;
	Status        = EFI_NOT_FOUND;

	Private = NVME_CONTROLLER_PRIVATE_DATA_FROM_PASS_THRU (This);
	//
	// If the NamespaceId input value is 0xFFFFFFFF, then get the first valid namespace ID
	//
	if (*NamespaceId == 0xFFFFFFFF) {
		//
		// Start with the first namespace ID
		//
		NextNamespaceId = 1;
		//
		// Allocate buffer for Identify Namespace data.
		//
		NamespaceData = (NVME_ADMIN_NAMESPACE_DATA *)MallocZero (sizeof (NVME_ADMIN_NAMESPACE_DATA));

		if (NamespaceData == NULL)
			return EFI_NOT_FOUND;

		Status = NvmeIdentifyNamespace (Private, NextNamespaceId, NamespaceData);
		if (EFI_ERROR(Status))
			goto Done;

		*NamespaceId = NextNamespaceId;
	} else {
		if (*NamespaceId > Private->ControllerData->Nn)
			return EFI_INVALID_PARAMETER;

		NextNamespaceId = *NamespaceId + 1;
		if (NextNamespaceId > Private->ControllerData->Nn)
			return EFI_NOT_FOUND;

		//
		// Allocate buffer for Identify Namespace data.
		//
		NamespaceData = (NVME_ADMIN_NAMESPACE_DATA *)MallocZero (sizeof (NVME_ADMIN_NAMESPACE_DATA));
		if (NamespaceData == NULL)
			return EFI_NOT_FOUND;

		Status = NvmeIdentifyNamespace (Private, NextNamespaceId, NamespaceData);
		if (EFI_ERROR(Status))
			goto Done;

		*NamespaceId = NextNamespaceId;
	}

Done:
	if (NamespaceData != NULL)
		FreeZero (NamespaceData);

	return Status;
}


/**
  Used to translate a device path node to a namespace ID.

  The EFI_NVM_EXPRESS_PASS_THRU_PROTOCOL.GetNamespace() function determines the namespace ID associated with the
  namespace described by DevicePath.

  If DevicePath is a device path node type that the NVM Express Pass Thru driver supports, then the NVM Express
  Pass Thru driver will attempt to translate the contents DevicePath into a namespace ID.

  If this translation is successful, then that namespace ID is returned in NamespaceId, and EFI_SUCCESS is returned

  @param[in]  This                A pointer to the EFI_NVM_EXPRESS_PASS_THRU_PROTOCOL instance.
  @param[in]  DevicePath          A pointer to the device path node that describes an NVM Express namespace on
                                  the NVM Express controller.
  @param[out] NamespaceId         The NVM Express namespace ID contained in the device path node.

  @retval EFI_SUCCESS             DevicePath was successfully translated to NamespaceId.
  @retval EFI_INVALID_PARAMETER   If DevicePath or NamespaceId are NULL, then EFI_INVALID_PARAMETER is returned.
  @retval EFI_UNSUPPORTED         If DevicePath is not a device path node type that the NVM Express Pass Thru driver
                                  supports, then EFI_UNSUPPORTED is returned.
  @retval EFI_NOT_FOUND           If DevicePath is a device path node type that the NVM Express Pass Thru driver
                                  supports, but there is not a valid translation from DevicePath to a namespace ID,
                                  then EFI_NOT_FOUND is returned.
**/

EFI_STATUS
EFIAPI
NvmExpressGetNamespace (
	IN     EFI_NVM_EXPRESS_PASS_THRU_PROTOCOL *This,
	IN     EFI_DEVICE_PATH_PROTOCOL           *DevicePath,
	OUT UINT32                                *NamespaceId
)
{
	if ((This == NULL) || (DevicePath == NULL) || (NamespaceId == NULL))
		return EFI_INVALID_PARAMETER;

	if (DevicePath->Type != MESSAGING_DEVICE_PATH)
		return EFI_UNSUPPORTED;

	if (DevicePath->SubType == MSG_NVME_NAMESPACE_DP) {
		*NamespaceId = 0;

		return EFI_SUCCESS;
	} else
		return EFI_UNSUPPORTED;
}

