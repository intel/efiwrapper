/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 *
 * Authors: Jérémy Compostella <jeremy.compostella@intel.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer
 *      in the documentation and/or other materials provided with the
 *      distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdbool.h>
#include <kconfig.h>
#include <libpayload.h>
#include <efi.h>
#include <efiapi.h>
#include <efilib.h>
#include <stdio.h>
#include <stdlib.h>
#include <pci/pci.h>
#include <ewlog.h>

#include <hwconfig.h>

#include "dw3/UsbDeviceModeProtocol.h"
#include "dw3/XdciCommon.h"
#include "dw3/XdciDWC.h"
#include "dw3/XdciUtility.h"
#include "dw3/dw3.h"

USB_DEVICE_DRIVER_OBJ mDrvObj;
USB_DEVICE_IO_REQ  mCtrlIoReq = {
	//
	// IO information containing the buffer and data size
	//
	{
	        NULL,
	        0,
	},
	//
	// Note: This object is used for Control Ep transfers only
	// therefore the endpoint info must always be NULL
	//
	{
	        NULL,
	        NULL,
	}
};

/* parent handle: us */
static void *device_core_ptr = NULL;

/* initialized by dwc_xdci_core_init */
static void *core_handle = NULL;

/* controller config for device role */
/* register offset */
#define R_XHCI_MEM_DUAL_ROLE_CFG0 0x80D8
/* programming value for device */
#define CFG0_DEVICE_ROLE_CONFIG   0x1310800

/* USB XDCI, XHCI PCI IDs */
#define INTEL_VID 0x8086

static USB_DEV_CONFIG_PARAMS config_params = {	.ControllerId = USB_ID_DWC_XDCI,
						.BaseAddress  = 0,
						.Flags = 0,
						.Speed = USB_SPEED_SUPER,
						.Role = USB_ROLE_DEVICE};

BOOLEAN mXdciRun = FALSE;

EFI_STATUS
EFIAPI
UsbdResetEvtHndlr (
	__attribute__((__unused__)) IN USB_DEVICE_CALLBACK_PARAM  *Param
	)
{
	EFI_STATUS    Status = EFI_DEVICE_ERROR;

	DEBUG ((DEBUG_INFO, "UsbdResetEvtHndlr\n"));

	/* reset device address to 0 */
	Status = dwc_xdci_core_set_address (core_handle, 0x0);
	if (EFI_ERROR (Status)) {
	        DEBUG ((DEBUG_INFO, "UsbdResetHdlr() - Failed to set address in XDCI\n"));
	}

	return Status;
}

EFI_STATUS
EFIAPI
UsbdConnDoneEvtHndlr (
	__attribute__((__unused__)) IN USB_DEVICE_CALLBACK_PARAM *Param
	)
{
	EFI_STATUS    Status = EFI_DEVICE_ERROR;

	DEBUG ((DEBUG_INFO, "UsbdConnDoneEvtHndlr\n"));

	/* reset device address to 0 */
	Status = dwc_xdci_core_set_address (core_handle, 0x0);
	if (EFI_ERROR (Status)) {
	        DEBUG ((DEBUG_INFO, "UsbdConnDoneHdlr() - Failed to set address in XDCI\n"));
	}

	/* set the device state to attached/connected */
	mDrvObj.State = UsbDevStateAttached;

	return Status;
}

VOID
UsbdSetEpInfo (
	IN USB_EP_INFO		 *EpDest,
	IN USB_DEVICE_ENDPOINT_INFO    *EpSrc
	)
{
	EFI_USB_ENDPOINT_DESCRIPTOR	      *EpDesc = NULL;
	EFI_USB_ENDPOINT_COMPANION_DESCRIPTOR    *EpCompDesc = NULL;

	/* start by clearing all data in the destination */
	memset (EpDest, 0, sizeof(USB_EP_INFO));
	EpDesc = EpSrc->EndpointDesc;
	EpCompDesc = EpSrc->EndpointCompDesc;

	if (EpDesc != NULL) {
	        EpDest->ep_num = EpDesc->EndpointAddress & 0x0F; /* Bits 0-3 are ep num */
	        EpDest->ep_dir = ((EpDesc->EndpointAddress & USB_ENDPOINT_DIR_IN) > 0) ? UsbEpDirIn : UsbEpDirOut;
	        EpDest->ep_type = EpDesc->Attributes & USB_ENDPOINT_TYPE_MASK;
	        EpDest->max_pkt_size = EpDesc->MaxPacketSize;
	        EpDest->interval = EpDesc->Interval;
	}
	if (EpCompDesc != NULL) {
	        EpDest->max_streams = EpCompDesc->Attributes & USB_EP_BULK_BM_ATTR_MASK;
	        EpDest->burst_size = EpCompDesc->MaxBurst;
	        EpDest->mult = EpCompDesc->BytesPerInterval;
	}

	return;
}

VOID
EFIAPI
UsbdXferDoneHndlr (
	__attribute__((__unused__)) IN VOID		    *XdciHndl,
	IN USB_XFER_REQUEST	*XferReq
	)
{
	EFI_USB_DEVICE_XFER_INFO  XferInfo;

	DEBUG ((DEBUG_INFO, "UsbdXferDoneHndlr\n"));

	XferInfo.EndpointNum = (UINT8)XferReq->ep_info.ep_num;
	XferInfo.EndpointDir = XferReq->ep_info.ep_dir;
	XferInfo.EndpointType = XferReq->ep_info.ep_type;
	XferInfo.Buffer = XferReq->xfer_buffer;
	XferInfo.Length = XferReq->actual_xfer_len;

	//
	// If this is a non-control transfer complete, notify the class driver
	//
	if (XferInfo.EndpointNum > 0) {
	        if (mDrvObj.UsbdDevObj->DataCallback != NULL) {
	                mDrvObj.UsbdDevObj->DataCallback (&XferInfo);
	        }
	}

	return;
}

EFI_STATUS
UsbdEpTxData (
	IN VOID	       *XdciHndl,
	IN USB_DEVICE_IO_REQ  *IoReq
	)
{
	EFI_STATUS	Status = EFI_DEVICE_ERROR;
	USB_XFER_REQUEST  TxReq;

	/* set endpoint data */
	UsbdSetEpInfo (&(TxReq.ep_info), &(IoReq->EndpointInfo));

	/* if this is a control endpoint, set the number and direction */
	if (IoReq->EndpointInfo.EndpointDesc == NULL) {
	        TxReq.ep_info.ep_num = 0;
	        TxReq.ep_info.ep_dir = UsbEpDirIn;
	}

	/* setup the trasfer request */
	TxReq.xfer_buffer = IoReq->IoInfo.Buffer;
	TxReq.xfer_len = IoReq->IoInfo.Length;
	TxReq.xfer_done = UsbdXferDoneHndlr;

	DEBUG ((DEBUG_INFO,  "TX REQUEST: epNum: 0x%x, epDir: 0x%x, epType: 0x%x, MaxPktSize: 0x%x\n",\
	        TxReq.ep_info.ep_num, TxReq.ep_info.ep_dir, TxReq.ep_info.ep_type, TxReq.ep_info.max_pkt_size));

	Status = dwc_xdci_ep_tx_data (XdciHndl, &TxReq);

	return Status;
}

EFI_STATUS
UsbdGetConfigDesc (
	IN VOID      *Buffer,
	IN UINT8     DescIndex,
	IN UINT32    ReqLen,
	IN UINT32    *DataLen
	)
{
	EFI_STATUS	     Status = EFI_DEVICE_ERROR;
	UINT8		  NumConfigs = 0;
	UINT32		 ConfigLen = 0;
	USB_DEVICE_CONFIG_OBJ  *ConfigObj = NULL;
	VOID		   *Descriptor = 0;
	UINT32		 Length = 0;

	DEBUG ((DEBUG_INFO, "UsbdGetConfigDesc()\n"));

	/*
	 * For a CONFIGURATION request we send back all descriptors branching out
	 * from this descriptor including the INTERFACE and ENDPOINT descriptors
	 */

	/* Verify the requested configuration exists - check valid index */
	NumConfigs = mDrvObj.UsbdDevObj->DeviceDesc->NumConfigurations;

	if (DescIndex < NumConfigs) {
	        /* get the configuration object using the index offset */
	        ConfigObj = (mDrvObj.UsbdDevObj->ConfigObjs + DescIndex);
	        /* get the complete configuration buffer block including Interface and Endpoint data */
	        Descriptor = ConfigObj->ConfigAll;
	        /* The config descriptor TotalLength has the full value for all desc buffers */
	        ConfigLen = ConfigObj->ConfigDesc->TotalLength;
	        /* copy the data to the output buffer */
	        Length = MIN (ReqLen, ConfigLen);
	        memcpy (Buffer, Descriptor, Length);
	        *DataLen = Length;
	        Status = EFI_SUCCESS;
	} else {
	        DEBUG ((DEBUG_INFO, "UsbdGetConfigDesc() - Invalid Config index: %i\n", DescIndex));
	}

	if (Status == EFI_SUCCESS) {
	        if (ConfigObj != NULL) {
	                PrintConfigDescriptor (ConfigObj->ConfigDesc);
	        }
	}

	return Status;
}

EFI_STATUS
UsbdGetStringDesc (
	VOID      *Buffer,
	UINT8     DescIndex,
	UINT16    LangId,
	UINT32    ReqLen,
	UINT32    *DataLen
	)
{
	EFI_STATUS	     Status = EFI_DEVICE_ERROR;
	UINT32		 Length = 0;
	USB_STRING_DESCRIPTOR  *StringDesc;
	UINT8		  Index = 0;
	UINT8		  StrLangEntries = 0;
	BOOLEAN		StrLangFound = FALSE;

	DEBUG ((DEBUG_INFO, "UsbdGetStringDesc: Index: 0x%x, LangId: 0x%x, ReqLen: 0x%x\n", DescIndex, LangId, ReqLen));

	/* index zero of the string table contains the supported language codes */
	if (DescIndex == 0) {
	        StringDesc = (mDrvObj.UsbdDevObj->StringTable);
	        Length = MIN (ReqLen, StringDesc->Length);
	        memcpy (Buffer, StringDesc, Length);
	        *DataLen = Length;
	        Status = EFI_SUCCESS;
	} else {

	        /*
	         * Verify the requested language ID is supported. String descriptor Zero
	         * (First entry in the string table) is expected to contain the language list.
	         * The requested language ID is specified in the Index member of the request.
	         */
	        StringDesc = mDrvObj.UsbdDevObj->StringTable; /* get language string descriptor */
	        StrLangEntries = ((StringDesc->Length - 2) >> 1);
	        DEBUG ((DEBUG_INFO, "StrLangEntries=%x\n", StrLangEntries));

	        DEBUG ((DEBUG_INFO, "Looking LangID: \n"));

	        for (Index = 0; Index < StrLangEntries; Index++) {
	                DEBUG ((DEBUG_INFO, "LangID [%x]= %x\n", Index, StringDesc->LangID [Index]));

	                if (StringDesc->LangID [Index] == LangId) {
	                        DEBUG ((DEBUG_INFO, "Found it\n"));
	                        StrLangFound = TRUE;
	                }
	        }



	        /* If we found a matching language, attempt to get the string index requested */
	        if (StrLangFound == TRUE) {
	                DEBUG ((DEBUG_INFO, "UsbdGetStringDesc: StrLangFound=Found, DescIndex=%x, StrTblEntries=%x\n", DescIndex, mDrvObj.UsbdDevObj->StrTblEntries));

	                if (DescIndex < mDrvObj.UsbdDevObj->StrTblEntries) {
	                        /* get the string descriptor for the requested index */
	                        StringDesc = (mDrvObj.UsbdDevObj->StringTable + DescIndex);

	                        Length = MIN (ReqLen, StringDesc->Length);
	                        DEBUG ((DEBUG_INFO, "ReqLen=%x, StringLength=%x, Length=%x\n", ReqLen, StringDesc->Length, Length));

	                        memcpy (Buffer, StringDesc, Length);
	                        *DataLen = Length;
	                        Status = EFI_SUCCESS;
	                } else {
	                        DEBUG ((DEBUG_INFO, "UsbdGetStringDesc: Invalid String index in USB_REQ_GET_DESCRIPTOR request\n"));
	                }
	        } else {
	                DEBUG ((DEBUG_INFO, "UsbdGetStringDesc: Unsupported String Language ID for USB_REQ_GET_DESCRIPTOR request\n"));
	        }
	}

	if (Status == EFI_SUCCESS) {
	        PrintStringDescriptor (StringDesc);
	}
	return Status;
}

EFI_STATUS
UsbdGetConfig (
	VOID      *Buffer,
	UINT32    ReqLen,
	UINT32    *DataLen
	)
{
	EFI_STATUS    Status = EFI_DEVICE_ERROR;

	DEBUG ((DEBUG_INFO, "UsbdGetConfig()\n"));

	if (ReqLen >= 1) { /* length of data expected must be 1 */
	        if (mDrvObj.ActiveConfigObj != NULL) { /* assure we have a config active */
	                *DataLen = 1; /* one byte for ConfigurationValue */
	                *(UINT8*)Buffer = mDrvObj.ActiveConfigObj->ConfigDesc->ConfigurationValue;

	                Status = EFI_SUCCESS;
	        } else {
	                DEBUG ((DEBUG_INFO, "UsbdGetConfig() - No active configuration available\n"));
	        }
	} else {
	        DEBUG ((DEBUG_INFO, "UsbdGetConfig() - Invalid data length\n"));
	}

	return Status;
}

EFI_STATUS
UsbdInitEp (
	IN VOID		      *XdciHndl,
	IN USB_DEVICE_ENDPOINT_INFO  *DevEpInfo
	)
{
	EFI_STATUS   Status = EFI_DEVICE_ERROR;
	USB_EP_INFO  EpInfo;

	UsbdSetEpInfo (&EpInfo, DevEpInfo);
	Status = dwc_xdci_init_ep (XdciHndl, &EpInfo);

	return Status;
}

EFI_STATUS
UsbdSetAddress (
	UINT8    Address
	)
{
	EFI_STATUS  Status = EFI_DEVICE_ERROR;

	DEBUG ((DEBUG_INFO, "UsbdSetAddress: setting address: 0x%x\n", Address));

	if (Address <= 0x7F) { /* address must not be > 127 */
	        mDrvObj.Address = Address;

	        /* Configure Address in the XDCI */
	        Status = dwc_xdci_core_set_address (core_handle, mDrvObj.Address);
	        if (!EFI_ERROR (Status)) {
	                mDrvObj.State = UsbDevStateAddress;
	        } else {
	                DEBUG ((DEBUG_INFO, "UsbdSetAddress: Failed to set address in XDCI\n"));
	        }
	} else {
	        DEBUG ((DEBUG_INFO, "UsbdSetAddress: Invalid address: 0x%x\n", Address));
	}

	return Status;
}

EFI_STATUS
UsbdGetStatus (
	VOID      *Buffer,
	UINT8     ReqType,
	UINT32    ReqLen,
	UINT32    *DataLen
	)
{
	EFI_STATUS  Status = EFI_DEVICE_ERROR;

	DEBUG ((DEBUG_INFO, "UsbdGetStatus()\n"));

	if (ReqLen >= 2) { /* length of data must be at least 2 bytes */
	        switch (ReqType & USB_TARGET_MASK) {
	        case USB_TARGET_DEVICE:
	                *DataLen = 2; /* two byte for status */
	                *(UINT16*)Buffer = USB_STATUS_SELFPOWERED;
	                Status = EFI_SUCCESS;
	                break;

	        case USB_TARGET_INTERFACE:
	                /* No implementation needed at this time */
	                break;

	        case USB_TARGET_ENDPOINT:
	                /* No implementation needed at this time */
	                /* Should specify if endpoint is halted. Implement as necessary. */
	                break;

	        case USB_TARGET_OTHER:
	                /* No implementation needed at this time */
	                break;

	        default:
	                break;
	        }
	} else {
	        DEBUG ((DEBUG_INFO, "UsbdGetStatus() - Invalid data length\n"));
	}

	return Status;
}

EFI_STATUS
UsbdSetConfig (
	UINT8  CfgValue
	)
{
	EFI_STATUS		 Status = EFI_DEVICE_ERROR;
	UINT8		      numConfigs = 0;
	USB_DEVICE_CONFIG_OBJ      *pConfigObj = NULL;
	USB_DEVICE_INTERFACE_OBJ   *pIfObj = NULL;
	USB_DEVICE_ENDPOINT_OBJ    *pEpObj = NULL;
	UINT8		      cfgItr = 0;
	UINT8		      ifItr = 0;
	UINT8		      epItr = 0;
	USB_DEVICE_ENDPOINT_INFO   epInfo;
	USB_EP_INFO		UsbEpInfo;

	DEBUG ((DEBUG_INFO, "UsbdSetConfig()\n"));

	/* Verify the requested configuration exists - check valid index */
	numConfigs = mDrvObj.UsbdDevObj->DeviceDesc->NumConfigurations;

	if (CfgValue != 0) {
	        /* Search for a matching configuration */
	        for (cfgItr = 0; cfgItr < numConfigs; cfgItr++) {
	                pConfigObj = (mDrvObj.UsbdDevObj->ConfigObjs + cfgItr);
	                if (pConfigObj->ConfigDesc->ConfigurationValue == CfgValue) {

	                        /* Set the active configuration object */
	                        mDrvObj.ActiveConfigObj = pConfigObj;

	                        /* Find all interface objects for this configuration */
	                        for (ifItr = 0; ifItr < pConfigObj->ConfigDesc->NumInterfaces; ifItr++) {
	                                pIfObj = (pConfigObj->InterfaceObjs + ifItr);

	                                /* Configure the Endpoints in the XDCI */
	                                for (epItr = 0; epItr < pIfObj->InterfaceDesc->NumEndpoints; epItr++) {
	                                        pEpObj = (pIfObj->EndpointObjs + epItr);

	                                        epInfo.EndpointDesc = pEpObj->EndpointDesc;
	                                        epInfo.EndpointCompDesc = pEpObj->EndpointCompDesc;

	                                        if (UsbdInitEp (core_handle, &epInfo) == EFI_SUCCESS) {
	                                                UsbdSetEpInfo(&UsbEpInfo, &epInfo);
	                                                if (dwc_xdci_ep_enable (core_handle, &UsbEpInfo) == EFI_SUCCESS) {
	                                                        Status = EFI_SUCCESS;
	                                                } else {
	                                                        DEBUG ((DEBUG_INFO, "UsbdSetConfig() - Failed to enable endpoint\n"));
	                                                }
	                                        } else {
	                                                DEBUG ((DEBUG_INFO, "UsbdSetConfig() - Failed to initialize endpoint\n"));
	                                        }
	                                }
	                        }

	                        /* Let the class driver know it is configured */
	                        if (Status == EFI_SUCCESS) {
	                                if (mDrvObj.UsbdDevObj->ConfigCallback != NULL) {
	                                        mDrvObj.UsbdDevObj->ConfigCallback (CfgValue);
	                                }
	                        }

	                        mDrvObj.State = UsbDevStateConfigured; /* we are now configured */

	                        break; /* break from config search loop */
	                }
	        }
	}

	if (EFI_ERROR (Status)) {
	        DEBUG ((DEBUG_INFO, "UsbdSetConfig() - Invalid requested configuration value: %i\n", CfgValue));
	}

	return Status;
}


EFI_STATUS
UsbdSetupHdlr (
	IN EFI_USB_DEVICE_REQUEST    *CtrlRequest
	)
{
	EFI_STATUS	      Status = EFI_DEVICE_ERROR;
	UINT8		   DescIndex = 0;
	USB_DEVICE_DESCRIPTOR   *DevDesc = 0;

	/* Initialize the IO object */
	mCtrlIoReq.IoInfo.Length = 0;

	DEBUG ((DEBUG_INFO, "UsbdSetupHdlr start\n"));
	PrintDeviceRequest (CtrlRequest);

	/* Handle Standard Device Requests */
	if ((CtrlRequest->RequestType & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD) {
	        switch (CtrlRequest->Request) {
	        case USB_REQ_GET_DESCRIPTOR:
	                DEBUG ((DEBUG_INFO, "UsbdSetupHdlr: Host requests get descriptor\n"));
	                if (CtrlRequest->RequestType == USB_RT_TX_DIR_D_TO_H) {
	                        DescIndex = (CtrlRequest->Value & 0xff); /* low byte is the index requested */
	                        switch (CtrlRequest->Value >> 8) { /* high byte contains request type */
	                        case USB_DESC_TYPE_DEVICE:
	                                DEBUG ((DEBUG_INFO, "Descriptor tyep: Device\n"));
	                                DevDesc = mDrvObj.UsbdDevObj->DeviceDesc;
	                                /* copy the data to the output buffer */
	                                mCtrlIoReq.IoInfo.Length = MIN (CtrlRequest->Length, DevDesc->Length);
	                                memcpy (mCtrlIoReq.IoInfo.Buffer, DevDesc, mCtrlIoReq.IoInfo.Length);
	                                PrintDeviceDescriptor (DevDesc);
	                                break;

	                        case USB_DESC_TYPE_CONFIG:
	                                DEBUG ((DEBUG_INFO, "Descriptor tyep: Configuration\n"));
	                                Status = UsbdGetConfigDesc (
	                                        mCtrlIoReq.IoInfo.Buffer,
	                                        DescIndex,
	                                        CtrlRequest->Length,
	                                        &(mCtrlIoReq.IoInfo.Length)
	                                        );
	                                break;

	                        case USB_DESC_TYPE_STRING:
	                                DEBUG ((DEBUG_INFO, "Descriptor tyep: String\n"));
	                                Status = UsbdGetStringDesc (
	                                        mCtrlIoReq.IoInfo.Buffer,
	                                        DescIndex,
	                                        CtrlRequest->Index,
	                                        CtrlRequest->Length,
	                                        &(mCtrlIoReq.IoInfo.Length)
	                                        );
	                                break;

#ifdef SUPPORT_SUPER_SPEED
	                        case USB_DESC_TYPE_BOS:
	                                DEBUG ((DEBUG_INFO, "Descriptor tyep: BOS\n"));
	                                Status = UsbdGetBOSDesc (
	                                        mCtrlIoReq.IoInfo.Buffer,
	                                        CtrlRequest->Length,
	                                        &(mCtrlIoReq.IoInfo.Length)
	                                        );
	                                break;

	                        case USB_DESC_TYPE_SS_ENDPOINT_COMPANION:
	                                DEBUG ((DEBUG_INFO, "Descriptor tyep: Endpoint Companion\n"));
	                                break;
#endif

	                        default:
	                                DEBUG ((DEBUG_INFO, "Descriptor tyep: Unsupported, USB_REQ_GET_DESCRIPTOR request: 0x%x\n", (CtrlRequest->Value >> 8)));
	                                break;
	                        }
	                } else {
	                        DEBUG ((DEBUG_INFO, "UsbdSetupHdlr() - Invalid direction for USB_REQ_GET_DESCRIPTOR request\n"));
	                }
	                break;

	        case USB_REQ_GET_CONFIG:
	                DEBUG ((DEBUG_INFO, "USB_REQ_GET_CONFIG\n"));
	                if (CtrlRequest->RequestType == USB_RT_TX_DIR_D_TO_H) {
	                        Status = UsbdGetConfig (mCtrlIoReq.IoInfo.Buffer, CtrlRequest->Length, &(mCtrlIoReq.IoInfo.Length));
	                } else {
	                        DEBUG ((DEBUG_INFO, "UsbdSetupHdlr: Invalid direction for USB_REQ_GET_CONFIG request\n"));
	                }
	                break;

	        case USB_REQ_SET_CONFIG:
	                DEBUG ((DEBUG_INFO, "USB_REQ_SET_CONFIG\n"));
	                if (CtrlRequest->RequestType == USB_RT_TX_DIR_H_TO_D) {
	                        Status = UsbdSetConfig ((UINT8)CtrlRequest->Value);
	                } else {
	                        DEBUG ((DEBUG_INFO, "UsbdSetupHdlr: Invalid direction for USB_REQ_SET_CONFIG request\n"));
	                }
	                break;

	        case USB_REQ_SET_ADDRESS:
	                DEBUG ((DEBUG_INFO, "USB_REQ_SET_ADDRESS\n"));
	                if (CtrlRequest->RequestType == USB_RT_TX_DIR_H_TO_D) {
	                        Status = UsbdSetAddress ((UINT8)CtrlRequest->Value);
	                } else {
	                        DEBUG ((DEBUG_INFO, "UsbdSetupHdlr: Invalid direction for USB_REQ_SET_ADDRESS request\n"));
	                }
	                break;

	        case USB_REQ_GET_STATUS:
	                DEBUG ((DEBUG_INFO, "USB_REQ_GET_STATUS\n"));
	                if (CtrlRequest->RequestType & USB_RT_TX_DIR_D_TO_H) {
	                        Status = UsbdGetStatus (mCtrlIoReq.IoInfo.Buffer, CtrlRequest->RequestType, CtrlRequest->Length, &(mCtrlIoReq.IoInfo.Length));
	                } else {
	                        DEBUG ((DEBUG_INFO, "UsbdSetupHdlr: Invalid direction for USB_REQ_GET_STATUS request\n"));
	                }
	                break;
#ifdef SUPPORT_SUPER_SPEED
	        case USB_REQ_CLEAR_FEATURE:
	        case USB_REQ_SET_FEATURE:
	        case USB_REQ_SET_DESCRIPTOR:
	        case USB_REQ_GET_INTERFACE:
	        case USB_REQ_SET_INTERFACE:
	        case USB_REQ_SYNCH_FRAME:
#endif
	        default:
	                DEBUG ((DEBUG_INFO, "UsbdSetupHdlr: Unsupported Standard Request: 0x%x\n", CtrlRequest->Request));
	                break;
	        }
	} else { /* This is not a Standard request, it specifies Class/Vendor handling */
	        /* Forward request to class driver */
	        DEBUG ((DEBUG_INFO, "UsbdSetupHdlr: Class/Vendor Request\n"));
	        if (mDrvObj.UsbdDevObj->SetupCallback != NULL) {
	                mDrvObj.UsbdDevObj->SetupCallback (CtrlRequest, &(mCtrlIoReq.IoInfo));
	        }
	}

	DEBUG ((DEBUG_INFO, "dataLen=%x\n", mCtrlIoReq.IoInfo.Length));

	/* Transfer data according to request if necessary */
	if (mCtrlIoReq.IoInfo.Length> 0) {
	        Status = UsbdEpTxData (core_handle, &mCtrlIoReq);
	        if (EFI_ERROR (Status)) {
	                DEBUG ((DEBUG_INFO, "UsbdSetupHdlr: Failed to TX data\n"));
	        }
	} else {
	        /* If we are not responding with data, send control status */
	        Status = dwc_xdci_ep0_send_status_pkt (core_handle);
	        if (EFI_ERROR (Status)) {
	                DEBUG ((DEBUG_INFO, "UsbdSetupHdlr: Failed to Tx Ep0 Status\n"));
	        }
	}

	return Status;
}

EFI_STATUS
EFIAPI
UsbdSetupEvtHndlr (
	IN USB_DEVICE_CALLBACK_PARAM *Param
	)
{
	EFI_STATUS	      Status = EFI_SUCCESS;
	EFI_USB_DEVICE_REQUEST  Req;

	DEBUG ((DEBUG_INFO, "UsbdSetupEvtHndlr\n"));

	/* Fill out request object from the incomming buffer */
	memcpy (&Req, Param->buffer, sizeof(EFI_USB_DEVICE_REQUEST));

	Status = UsbdSetupHdlr (&Req);
	if (EFI_ERROR (Status)) {
	        DEBUG ((DEBUG_INFO, "UsbdSetupEvtHndlr: EFI_DEVICE_ERROR\n"));
	}

	return Status;
}

EFI_STATUS
EFIAPI
UsbdNrdyEvtHndlr (
	__attribute__((__unused__)) IN USB_DEVICE_CALLBACK_PARAM *Param
	)
{
	DEBUG ((DEBUG_INFO, "UsbdNrdyEvtHndlr\n"));
	return EFI_SUCCESS;
}

EFI_STATUS
UsbdRegisterCallbacks (
	IN VOID  *XdciHndl
	)
{
	if (dwc_xdci_core_register_callback (XdciHndl, USB_DEVICE_RESET_EVENT, UsbdResetEvtHndlr) != EFI_SUCCESS) {
	        goto UdciRegCallbackError;
	}

	if (dwc_xdci_core_register_callback (XdciHndl, USB_DEVICE_CONNECTION_DONE, UsbdConnDoneEvtHndlr) != EFI_SUCCESS) {
	        goto UdciRegCallbackError;
	}

	if (dwc_xdci_core_register_callback (XdciHndl, USB_DEVICE_SETUP_PKT_RECEIVED, UsbdSetupEvtHndlr) != EFI_SUCCESS) {
	        goto UdciRegCallbackError;
	}

	if (dwc_xdci_core_register_callback (XdciHndl, USB_DEVICE_XFER_NRDY, UsbdNrdyEvtHndlr) != EFI_SUCCESS) {
	        goto UdciRegCallbackError;
	}

	return EFI_SUCCESS;

UdciRegCallbackError:
	return EFI_DEVICE_ERROR;
}

EFI_STATUS
UsbdEpRxData (
	IN VOID	       *XdciHndl,
	IN USB_DEVICE_IO_REQ  *IoReq
	)
{
	EFI_STATUS	Status = EFI_DEVICE_ERROR;
	USB_XFER_REQUEST  RxReq;
	UINT32	    ReqPacket;

	DEBUG ((DEBUG_INFO,  "RX REQUEST in: IoReq->IoInfo.Length: 0x%x\n", IoReq->IoInfo.Length));
	DEBUG ((DEBUG_INFO,  "RX REQUEST in: MaxPacketSize: 0x%x\n", IoReq->EndpointInfo.EndpointDesc->MaxPacketSize));

	if (IoReq->EndpointInfo.EndpointDesc->MaxPacketSize == 0) {
	        return EFI_DEVICE_ERROR;
	}

	/* set endpoint data */
	UsbdSetEpInfo (&(RxReq.ep_info), &(IoReq->EndpointInfo));

	/* setup the trasfer request */
	RxReq.xfer_buffer = IoReq->IoInfo.Buffer;

	//
	// Transfer length should be multiple of USB packet size.
	//
	ReqPacket = IoReq->IoInfo.Length / IoReq->EndpointInfo.EndpointDesc->MaxPacketSize;
	ReqPacket = ((IoReq->IoInfo.Length % IoReq->EndpointInfo.EndpointDesc->MaxPacketSize) == 0)? ReqPacket : ReqPacket + 1;
	RxReq.xfer_len = ReqPacket * IoReq->EndpointInfo.EndpointDesc->MaxPacketSize;

	RxReq.xfer_done = UsbdXferDoneHndlr;

	DEBUG ((DEBUG_INFO,  "RX REQUEST: epNum: 0x%x, epDir: 0x%x, epType: 0x%x\n",\
	        RxReq.ep_info.ep_num, RxReq.ep_info.ep_dir, RxReq.ep_info.ep_type));
	DEBUG ((DEBUG_INFO,  "RX REQUEST send: xfer_len: 0x%x\n", RxReq.xfer_len));

	Status = dwc_xdci_ep_rx_data(XdciHndl, &RxReq);

	return Status;
}

#define  PCI_BASE_ADDRESS_MEM_TYPE_MASK	0x06
#define  PCI_BASE_ADDRESS_MEM_TYPE_64	0x04	/* 64 bit address */
static uint64_t pci_read_bar64(pcidev_t device)
{
	uint64_t addr64;
	uint32_t addr;

	addr = pci_read_config32(device, PCI_BASE_ADDRESS_0);
	if ((addr & PCI_BASE_ADDRESS_MEM_TYPE_MASK) != PCI_BASE_ADDRESS_MEM_TYPE_64)
		return addr & ~0xf;

	addr64 = pci_read_config32(device, PCI_BASE_ADDRESS_0 + 4);
	addr64 <<= 32;
	addr64 |= addr & ~0xf;

	return addr64;
}

static EFIAPI EFI_STATUS
_usb_init_xdci(EFI_USB_DEVICE_MODE_PROTOCOL *This)
{
	EFI_STATUS status;
	uint32_t pci_command;
	UINTN addr_hci;
	pcidev_t pci_dev;

#if defined(FB_SET_USB_DEVICE_MODE)
        /*need to set device mode in fastboot,p2sb pci is hide, access base address directly*/
	uint32_t value;

	value = *(uint32_t *)(P2SB_BASE_ADDR|USB_DAP_COMM_CTRL_REG_OFFSET);
	*(uint32_t *)(P2SB_BASE_ADDR|USB_DAP_COMM_CTRL_REG_OFFSET) = value | 0x01000000;
	value = *(uint32_t *)(P2SB_BASE_ADDR|USB_DAP_USB2_CTRL0_REG_OFFSET);
	*(uint32_t *)(P2SB_BASE_ADDR|USB_DAP_USB2_CTRL0_REG_OFFSET) = value | 0x00000160;
#endif
	/* actually not used, could stay NULL */
	device_core_ptr = (void *) This;

	/* get xDCI base address */
	pci_find_device(INTEL_VID, XDCI_PID, &pci_dev);

	ewdbg("PCI xDCI [%x:%x] %d.%d.%d", INTEL_VID, XDCI_PID,
	      PCI_BUS(pci_dev), PCI_SLOT(pci_dev), PCI_FUNC(pci_dev));

	config_params.BaseAddress = (UINTN)pci_read_bar64(pci_dev);
	ewdbg("xDCI BaseAddress =0x%" PRIx64 "\n", (uint64_t)config_params.BaseAddress);

	/* configure xDCI as a system bus master */
	pci_command = pci_read_config32(pci_dev, PCI_COMMAND);
	pci_command |= PCI_COMMAND_MASTER;
	pci_write_config32(pci_dev, PCI_COMMAND, pci_command);

	/* get xHCI base address */
	pci_find_device(INTEL_VID, XHCI_PID, &pci_dev);

	ewdbg("PCI xHCI [%x:%x] %d.%d.%d", INTEL_VID, XHCI_PID,
	      PCI_BUS(pci_dev), PCI_SLOT(pci_dev), PCI_FUNC(pci_dev));

	addr_hci = (UINTN)pci_read_bar64(pci_dev);
	ewdbg("xHCI BaseAddress =0x%" PRIx64 "\n", (uint64_t)addr_hci);

	/* enable xHCI bus master and I/O access */
	pci_command = pci_read_config32(pci_dev, PCI_COMMAND);
	pci_command |= PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY;
	pci_write_config32(pci_dev, PCI_COMMAND, pci_command);

	/* configure device role */
	usb_reg_write(addr_hci, R_XHCI_MEM_DUAL_ROLE_CFG0, CFG0_DEVICE_ROLE_CONFIG);

	/* disable xHCI bus master and I/O access */
	pci_command &= ~(PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY);
	pci_write_config32(pci_dev, PCI_COMMAND, pci_command);

	/* init device driver */
	status = dwc_xdci_core_init(&config_params, device_core_ptr, &core_handle);
	if (status)
	        return EFI_DEVICE_ERROR;

	status = UsbdRegisterCallbacks(core_handle);
	if (status)
	        return EFI_DEVICE_ERROR;

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
_usb_connect(__attribute__((__unused__)) EFI_USB_DEVICE_MODE_PROTOCOL *This)
{
	EFI_STATUS status;

	status = dwc_xdci_core_connect(core_handle);

	if (status)
	        return EFI_DEVICE_ERROR;

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
_usb_disconnect(__attribute__((__unused__)) EFI_USB_DEVICE_MODE_PROTOCOL *This)
{
	EFI_STATUS status = EFI_DEVICE_ERROR;

	if (dwc_xdci_core_disconnect(core_handle) == EFI_SUCCESS) {
	        mDrvObj.State = UsbDevStateInit;
	        status = EFI_SUCCESS;
	}

	return status;
}

static EFIAPI EFI_STATUS
_usb_ep_tx_data(__attribute__((__unused__)) EFI_USB_DEVICE_MODE_PROTOCOL *This,
		USB_DEVICE_IO_REQ *IoRequest)
{
	EFI_STATUS  Status;

	Status = UsbdEpTxData (core_handle, IoRequest);
	return Status;
}

static EFIAPI EFI_STATUS
_usb_ep_rx_data(__attribute__((__unused__)) EFI_USB_DEVICE_MODE_PROTOCOL *This,
		USB_DEVICE_IO_REQ *IoRequest)
{
	EFI_STATUS  Status;

	Status = UsbdEpRxData (core_handle, IoRequest);
	return Status;
}

static EFIAPI EFI_STATUS
_usb_bind(__attribute__((__unused__)) EFI_USB_DEVICE_MODE_PROTOCOL *This,
	  USB_DEVICE_OBJ *UsbdDevObj)
{
	EFI_STATUS  Status = EFI_SUCCESS;

	ewdbg("USB bind:");
	ewdbg("IdVendor  : 0x%x", UsbdDevObj->DeviceDesc->IdVendor);
	ewdbg("IdProduct : 0x%x", UsbdDevObj->DeviceDesc->IdProduct);

	/* allocate Tx buffer */
	mCtrlIoReq.IoInfo.Buffer = calloc (1, USB_EPO_MAX_PKT_SIZE_ALL);
	if (mCtrlIoReq.IoInfo.Buffer != NULL) {
	        mDrvObj.UsbdDevObj = UsbdDevObj;
	        mDrvObj.ActiveConfigObj = NULL;
	        mDrvObj.Address = 0;
	        mDrvObj.State = UsbDevStateInit;
	} else {
	        DEBUG ((DEBUG_INFO, "UsbDeviceBind() - Failed to allocate IO Buffer\n"));
	        Status = EFI_DEVICE_ERROR;
	}

	return Status;
}

static EFIAPI EFI_STATUS
_usb_unbind(__attribute__((__unused__)) EFI_USB_DEVICE_MODE_PROTOCOL *This)
{
	mDrvObj.UsbdDevObj = NULL;
	mDrvObj.ActiveConfigObj = NULL;
	mDrvObj.Address = 0;
	mDrvObj.State = UsbDevStateOff;

	/* release allocated buffer data */
	if (mCtrlIoReq.IoInfo.Buffer) {
	        free (mCtrlIoReq.IoInfo.Buffer);
	}

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
_usb_stop(__attribute__((__unused__)) EFI_USB_DEVICE_MODE_PROTOCOL *This)
{
	mXdciRun = FALSE;
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
_usb_run(__attribute__((__unused__)) EFI_USB_DEVICE_MODE_PROTOCOL *This,
	 UINT32 TimeoutMs)
{
	EFI_STATUS Status = EFI_DEVICE_ERROR;

	/* TODO: check initialized */

	mXdciRun = TRUE;
	Status = EFI_SUCCESS;

	while (TRUE) {
	        if (dwc_xdci_core_isr_routine(core_handle) != EFI_SUCCESS) {
	                DEBUG ((DEBUG_INFO, "UsbDeviceRun() - Failed to execute event ISR\n"));
	        }

	        /* check for timeout */
	        if (TimeoutMs == 0) {
	                return EFI_TIMEOUT;
	        }
	        udelay(50);
	        TimeoutMs--;
	}

	return Status;
}

static EFI_USB_DEVICE_MODE_PROTOCOL usb_struct = {
	.InitXdci = _usb_init_xdci,
	.Connect = _usb_connect,
	.DisConnect = _usb_disconnect,
	.EpTxData = _usb_ep_tx_data,
	.EpRxData = _usb_ep_rx_data,
	.Bind = _usb_bind,
	.UnBind = _usb_unbind,
	.Run = _usb_run,
	.Stop = _usb_stop
};

static EFI_HANDLE dw3_handle;
static EFI_GUID usb_guid = EFI_USB_DEVICE_MODE_PROTOCOL_GUID;

static EFI_STATUS dw3_init(EFI_SYSTEM_TABLE *st)
{
	if (!st)
		return EFI_INVALID_PARAMETER;

	if (dw3_handle)
	        return EFI_ALREADY_STARTED;

	return uefi_call_wrapper(st->BootServices->InstallProtocolInterface, 4,
				 &dw3_handle, &usb_guid,
				 EFI_NATIVE_INTERFACE, &usb_struct);
}

static EFI_STATUS dw3_exit(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;

	if (!st)
		return EFI_INVALID_PARAMETER;

	if (!dw3_handle)
	        return EFI_INVALID_PARAMETER;


	ret = uefi_call_wrapper(st->BootServices->UninstallProtocolInterface, 3,
				dw3_handle, &usb_guid, &usb_struct);
	if (!EFI_ERROR(ret))
	        dw3_handle = NULL;

	return ret;
}

ewdrv_t dw3_drv = {
	.name = "dw3",
	.description = "XDCI driver",
	.init = dw3_init,
	.exit = dw3_exit
};
