/*++

  Copyright (c)  1999 - 2014 Intel Corporation. All rights reserved
  This software and associated documentation (if any) is furnished
  under a license and may only be used or copied in accordance
  with the terms of the license. Except as permitted by such
  license, no part of this software or documentation may be
  reproduced, stored in a retrieval system, or transmitted in any
  form or by any means without the express written consent of
  Intel Corporation.

  --*/

#include <efi.h>
#include <efilib.h>
#include <stdio.h>
#include <kconfig.h>
#include <libpayload.h>
#include <stdbool.h>

#include "dw3/UsbDeviceModeProtocol.h"
#include "dw3/XdciDWC.h"
#include "dw3/XdciDevice.h"

UINT32
usb_reg_read (
	IN UINT32    base,
	IN UINT32    offset
	)
{
	volatile UINT32 *addr = (volatile UINT32 *)(UINTN)(base + offset);
	return *addr;
}

VOID
usb_reg_write (
	IN UINT32    base,
	IN UINT32    offset,
	IN UINT32    val
	)
{
	volatile UINT32 *addr = (volatile UINT32 *)(UINTN)(base + offset);
	*addr = val;
}


/**
   Internal utility function:
   This function is used to obtain physical endpoint number
   xDCI needs physical endpoint number for EP registers
   We also use it to index into our EP array
   Note: Certain data structures/commands use logical EP numbers
   as opposed to physical endpoint numbers so one should be
   careful when interpreting EP numbers
   @ep_num: Logical endpoint number
   @ep_dir: Direction for the endpoint

**/
STATIC
UINT32
dwc_xdci_get_physical_ep_num (
	IN UINT32        EndpointNum,
	IN USB_EP_DIR    EndpointDir
	)
{
	return EndpointDir? ((EndpointNum << 1) | EndpointDir) : (EndpointNum << 1);
}


/**
   Internal utility function:
   This function is used to obtain the MPS for control transfers
   based on the speed. If this is called before bus reset completes
   then it returns MPS based on desired speed. If it is after bus
   reset then MPS returned is based on actual negotiated speed
   @core_handle: xDCI controller handle address
   @mps: address of 32-bit variable to return the MPS

**/
STATIC
EFI_STATUS
dwc_xdci_core_get_ctrl_mps (
	IN XDCI_CORE_HANDLE    *core_handle,
	IN UINT32              *mps
	)
{
	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_core_get_ctrl_mps: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	if (mps == NULL) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_core_get_ctrl_mps: INVALID parameter\n"));
	        return EFI_INVALID_PARAMETER;
	}

	switch (core_handle->actual_speed) {
	case USB_SPEED_HIGH:
	        *mps = DWC_XDCI_HS_CTRL_EP_MPS;
	        break;
	case USB_SPEED_FULL:
	        *mps = DWC_XDCI_FS_CTRL_EP_MPS;
	        break;
	case USB_SPEED_LOW:
	        *mps = DWC_XDCI_LS_CTRL_EP_MPS;
	        break;
	case USB_SPEED_SUPER:
	        *mps = DWC_XDCI_SS_CTRL_EP_MPS;
	        break;
	default:
	        *mps = 0;
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_core_get_ctrl_mps: UNKNOWN speed\n"));
	        break;
	}

	return EFI_SUCCESS;
}


/**
   Internal utility function:
   This function is used to initialize the parameters required
   for executing endpoint command
   @core_handle: xDCI controller handle address
   @ep_info: EP info address
   @config_action: Configuration action specific to EP command
   @ep_cmd: xDCI EP command for which parameters are initialized
   @ep_cmd_params: address of struct to return EP params

**/
STATIC
EFI_STATUS
dwc_xdci_core_init_ep_cmd_params (
	IN XDCI_CORE_HANDLE                *core_handle,
	IN USB_EP_INFO                     *ep_info,
	IN UINT32                          config_action,
	IN DWC_XDCI_ENDPOINT_CMD           ep_cmd,
	IN DWC_XDCI_ENDPOINT_CMD_PARAMS    *ep_cmd_params
	)
{
	EFI_STATUS  status = EFI_SUCCESS;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_core_init_ep_cmd_params: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	/* Reset params */
	ep_cmd_params->param0 = ep_cmd_params->param1 = ep_cmd_params->param2 = 0;

	switch (ep_cmd) {
	case EPCMD_SET_EP_CONFIG:
	        /* Issue DEPCFG command for EP */
	        /* Issue a DEPCFG (Command 1) command for endpoint */

	        if (ep_info->max_streams) {
	                ep_cmd_params->param1 = DWC_XDCI_PARAM1_SET_EP_CFG_STRM_CAP_MASK;
	        }

	        if (ep_info->interval) {
	                ep_cmd_params->param1 |= ((ep_info->interval-1) << DWC_XDCI_PARAM1_SET_EP_CFG_BINTM1_BIT_POS);
	        }

	        /* Set EP num */
	        ep_cmd_params->param1 |= (ep_info->ep_num << DWC_XDCI_PARAM1_SET_EP_CFG_EP_NUM_BIT_POS);

	        /* Set EP direction */
	        ep_cmd_params->param1 |= (ep_info->ep_dir << DWC_XDCI_PARAM1_SET_EP_CFG_EP_DIR_BIT_POS);

	        /* Set EP-specific Event enable for not ready and
	         * complete events
	         */
	        ep_cmd_params->param1 &= ~DWC_XDCI_PARAM1_SET_EP_CFG_EVT_EN_MASK;

	        /* Setup the events we want enabled for this EP */
	        ep_cmd_params->param1 |= (DWC_XDCI_PARAM1_SET_EP_CFG_EVT_XFER_NRDY_MASK |
	                                  DWC_XDCI_PARAM1_SET_EP_CFG_EVT_XFER_IN_PRG_MASK |
	                                  DWC_XDCI_PARAM1_SET_EP_CFG_EVT_XFER_CMPLT_MASK);

	        /* We only have one interrupt line for this core.
	         * Set interrupt number to 0
	         */
	        ep_cmd_params->param1 &= ~DWC_XDCI_PARAM1_SET_EP_CFG_INTR_NUM_MASK;

	        /* Set FIFOnum = 0 for control EP0 */
	        ep_cmd_params->param0 &= ~DWC_XDCI_PARAM0_SET_EP_CFG_FIFO_NUM_MASK;

	        /* Program FIFOnum for non-EP0 EPs */
	        if (ep_info->ep_num && ep_info->ep_dir) {
	                ep_cmd_params->param0 |= (ep_info->ep_num << DWC_XDCI_PARAM0_SET_EP_CFG_FIFO_NUM_BIT_POS);
	        }

	        /* Program max packet size */
	        ep_cmd_params->param0 &= ~DWC_XDCI_PARAM0_SET_EP_CFG_MPS_MASK;
	        ep_cmd_params->param0 |= (ep_info->max_pkt_size << DWC_XDCI_PARAM0_SET_EP_CFG_MPS_BIT_POS);

	        /* Set Burst size. 0 means burst size of 1 */
	        ep_cmd_params->param0 &= ~DWC_XDCI_PARAM0_SET_EP_CFG_BRST_SIZE_MASK;
	        ep_cmd_params->param0 |= (ep_info->burst_size << DWC_XDCI_PARAM0_SET_EP_CFG_BRST_SIZE_BIT_POS);

	        /* Set EP type */
	        ep_cmd_params->param0 &= ~DWC_XDCI_PARAM0_SET_EP_CFG_EP_TYPE_MASK;
	        ep_cmd_params->param0 |= (ep_info->ep_type << DWC_XDCI_PARAM0_SET_EP_CFG_EP_TYPE_BIT_POS);

	        /* Set config action */
	        ep_cmd_params->param0 &= ~DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_MASK;
	        ep_cmd_params->param0 |= (config_action << DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_BIT_POS);
	        break;

	case EPCMD_SET_EP_XFER_RES_CONFIG:
	        /* Set param0 to 1. Same for all EPs when resource
	         *  configuration is done
	         */
	        ep_cmd_params->param0 = 1;
	        break;

	case EPCMD_END_XFER:
	        /* Nothing to set. Already reset params for all cmds */
	        break;

	case EPCMD_START_NEW_CONFIG:
	        /* Nothing to set. Already reset params for all cmds */
	        break;

	default:
	        status = EFI_INVALID_PARAMETER;
	        DEBUG ((DEBUG_INFO, "\ndwc_xdci_core_init_ep_cmd_params: INVALID Parameter"));
	        break;
	}

	return status;
}


/**
   Internal utility function:
   This function is used to issue the xDCI endpoint command
   @core_handle: xDCI controller handle address
   @ep_num: Physical EP num
   @ep_cmd: xDCI EP command
   @ep_cmd_params: EP command parameters address

**/
STATIC
EFI_STATUS
dwc_xdci_core_issue_ep_cmd (
	IN XDCI_CORE_HANDLE                *core_handle,
	IN UINT32                          ep_num,
	IN UINT32                          ep_cmd,
	IN DWC_XDCI_ENDPOINT_CMD_PARAMS    *ep_cmd_params
	)
{
	UINT32 base_addr;
	UINT32 max_delay_iter = 5000;//DWC_XDCI_MAX_DELAY_ITERATIONS;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_core_issue_ep_cmd: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	base_addr = core_handle->base_address;

	/* Set EP command parameter values */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_EPCMD_PARAM2_REG(ep_num),
	        ep_cmd_params->param2
	        );

	usb_reg_write (
	        base_addr,
	        DWC_XDCI_EPCMD_PARAM1_REG(ep_num),
	        ep_cmd_params->param1
	        );

	usb_reg_write (
	        base_addr,
	        DWC_XDCI_EPCMD_PARAM0_REG(ep_num),
	        ep_cmd_params->param0
	        );

	/* Set the command code and activate it */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_EPCMD_REG(ep_num),
	        ep_cmd | DWC_XDCI_EPCMD_CMD_ACTIVE_MASK
	        );

	/* Wait until command completes */
	do {
	        if (!(usb_reg_read (base_addr, DWC_XDCI_EPCMD_REG(ep_num)) & DWC_XDCI_EPCMD_CMD_ACTIVE_MASK))
	                break;
	        else
	                udelay(DWC_XDCI_MAX_DELAY_ITERATIONS);
	} while (--max_delay_iter);

	if (!max_delay_iter) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_issue_ep_cmd. ERROR: Failed to issue Command\n"));
	        return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}


/**
   Internal utility function:
   This function is used to flush all FIFOs
   @core_handle: xDCI controller handle address

**/
STATIC
EFI_STATUS
dwc_xdci_core_flush_all_fifos (
	IN XDCI_CORE_HANDLE    *core_handle
	)
{
	UINT32 base_addr;
	UINT32 max_delay_iter = DWC_XDCI_MAX_DELAY_ITERATIONS;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_core_flush_all_fifos: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	base_addr = core_handle->base_address;

	/* Write the command to flush all FIFOs */
	usb_reg_write(
	        base_addr,
	        DWC_XDCI_DGCMD_REG,
	        (usb_reg_read (base_addr, DWC_XDCI_DGCMD_REG) | DWC_XDCI_DGCMD_CMD_ALL_FIFO_FLUSH | DWC_XDCI_DGCMD_CMD_ACTIVE_MASK)
	        );

	/* Wait until command completes */
	do {
	        if (!(usb_reg_read (base_addr, DWC_XDCI_DGCMD_REG) & DWC_XDCI_DGCMD_CMD_ACTIVE_MASK))
	                break;
	        else
	                udelay(DWC_XDCI_MAX_DELAY_ITERATIONS);
	} while (--max_delay_iter);

	if (!max_delay_iter) {
	        DEBUG ((DEBUG_INFO, "Failed to issue Command\n"));
	        return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}


/**
   Internal utility function:
   This function is used to flush Tx FIFO specific to an endpoint
   @core_handle: xDCI controller handle address
   @ep_num: Physical EP num

**/
STATIC
EFI_STATUS
dwc_xdci_core_flush_ep_tx_fifo (
	IN XDCI_CORE_HANDLE    *core_handle,
	__attribute__((__unused__)) IN UINT32              ep_num
	)
{
	UINT32 base_addr;
	UINT32 max_delay_iter = DWC_XDCI_MAX_DELAY_ITERATIONS;
	//UINT32 fifo_num;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_core_flush_ep_tx_fifo: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	base_addr = core_handle->base_address;

	/* Translate to FIFOnum
	 * NOTE: Assuming this is a Tx EP
	 */
	//fifo_num = (ep_num >> 1);

	/* TODO: Currently we are only using TxFIFO 0. Later map these
	 * Write the FIFO num/dir param for the generic command.
	 */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DGCMD_PARAM_REG,
	        ((usb_reg_read (base_addr, DWC_XDCI_DGCMD_PARAM_REG) & ~DWC_XDCI_DGCMD_PARAM_TX_FIFO_NUM_MASK) | DWC_XDCI_DGCMD_PARAM_TX_FIFO_DIR_MASK)
	        );

	/* Write the command to flush all FIFOs */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DGCMD_REG,
	        (usb_reg_read(base_addr, DWC_XDCI_DGCMD_REG) | DWC_XDCI_DGCMD_CMD_SEL_FIFO_FLUSH | DWC_XDCI_DGCMD_CMD_ACTIVE_MASK)
	        );


	/* Wait until command completes */
	do {
	        if (!(usb_reg_read(base_addr, DWC_XDCI_DGCMD_REG) & DWC_XDCI_DGCMD_CMD_ACTIVE_MASK))
	                break;
	        else
	                udelay(DWC_XDCI_MAX_DELAY_ITERATIONS);
	} while (--max_delay_iter);

	if (!max_delay_iter) {
	        DEBUG ((DEBUG_INFO, "Failed to issue Command\n"));
	        return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}



STATIC
EFI_STATUS
dwc_xdci_core_prepare_one_trb (
	IN DWC_XDCI_TRB            *trb,
	IN DWC_XDCI_TRB_CONTROL    trb_ctrl,
	IN UINT32                  LastBit,
	IN UINT32                  ChainBit,
	IN UINT8                   *buffer_ptr,
	IN UINT32                  size
	)
{
	DEBUG ((DEBUG_INFO, "trb is 0x%x, buffer_ptr is 0x%x, size is 0x%x\n", (unsigned) trb, (unsigned) buffer_ptr, size));

	trb->buff_ptr_low = (UINT32)(UINTN)buffer_ptr;
	trb->buff_ptr_high = 0;
	trb->len_xfer_params = size;
	trb->trb_ctrl = trb_ctrl << DWC_XDCI_TRB_CTRL_TYPE_BIT_POS;

	if (ChainBit)
	        trb->trb_ctrl |= ChainBit << DWC_XDCI_TRB_CTRL_CHAIN_BUFF_BIT_POS;

	if (LastBit)
	        trb->trb_ctrl |= LastBit << DWC_XDCI_TRB_CTRL_LST_TRB_BIT_POS;

	trb->trb_ctrl |= DWC_XDCI_TRB_CTRL_IOSP_MISOCH_MASK| DWC_XDCI_TRB_CTRL_HWO_MASK;

	DEBUG ((DEBUG_INFO, "(dwc_xdci_core_prepare_one_trb) trb->buff_ptr_low = 0x%x, trb->len_xfer_params is 0x%x, trb->trb_ctrl is 0x%x\n",
	        trb->buff_ptr_low, trb->len_xfer_params, trb->trb_ctrl));
	return EFI_SUCCESS;
}


/**
   Internal utility function:
   This function is used to initialize transfer request block
   @core_handle: xDCI controller handle address
   @trb: Address of TRB to initialize
   @trb_ctrl: TRB control value
   @buff_ptr: Transfer buffer address
   @size: Size of the transfer

**/
STATIC
EFI_STATUS
dwc_xdci_core_init_trb (
	IN XDCI_CORE_HANDLE        *core_handle,
	IN DWC_XDCI_TRB            *trb,
	IN DWC_XDCI_TRB_CONTROL    trb_ctrl,
	IN UINT8                   *buffer_ptr,
	IN UINT32                  size
	)
{
#define ONE_TRB_SIZE      (DWC_XDCI_TRB_BUFF_SIZE_MASK & 0x00F00000)
	UINT8                   *TrbBuffer;
	UINT32                  TrbCtrlLast;
	UINT32                  TrbCtrlChain;
	UINT32                  TrbIndex;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_core_init_trb: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	if (trb == NULL) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_core_init_trb: INVALID handle\n"));
	        return EFI_INVALID_PARAMETER;
	}

	/* Init TRB fields
	 * NOTE: Assuming we are only using 32-bit addresses
	 * TODO: update for 64-bit addresses
	 */
	if (size <= DWC_XDCI_TRB_BUFF_SIZE_MASK) {
	        //
	        // Can transfer in one TRB
	        //
	        TrbCtrlChain = 0;
	        TrbCtrlLast = 1;
	        dwc_xdci_core_prepare_one_trb (trb, trb_ctrl, TrbCtrlLast, TrbCtrlChain, buffer_ptr, size);
	        return EFI_SUCCESS;
	}

	//
	// Can't transfer in one TRB.
	// Seperate it in every ONE_TRB_SIZE of TRB
	//
	TrbBuffer = buffer_ptr;
	TrbIndex = 0;
	while (size > ONE_TRB_SIZE) {
	        TrbCtrlChain = 1;
	        TrbCtrlLast = 0;
	        dwc_xdci_core_prepare_one_trb (trb, trb_ctrl, TrbCtrlLast, TrbCtrlChain, TrbBuffer, ONE_TRB_SIZE);
	        TrbBuffer += ONE_TRB_SIZE;
	        size -= ONE_TRB_SIZE;
	        trb++;
	        TrbIndex++;
	        if (TrbIndex >= DWC_XDCI_TRB_NUM)
	                return EFI_OUT_OF_RESOURCES;
	}
	TrbCtrlChain = 0;
	TrbCtrlLast = 1;
	dwc_xdci_core_prepare_one_trb (trb, trb_ctrl, TrbCtrlLast, TrbCtrlChain, TrbBuffer, size);

	return EFI_SUCCESS;
}


/**
   Internal function:
   This function is used to start a SETUP phase on control endpoint
   @core_handle: xDCI controller handle address

**/
STATIC
EFI_STATUS
dwc_xdci_core_start_ep0_setup_xfer (
	IN XDCI_CORE_HANDLE    *core_handle
	)
{
	DWC_XDCI_ENDPOINT_CMD_PARAMS    ep_cmd_params;
	EFI_STATUS                      status = EFI_DEVICE_ERROR;
	DWC_XDCI_TRB                    *trb;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_core_start_ep0_setup_xfer: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	if (core_handle->ep_handles[0].state == USB_EP_STATE_SETUP) {
	        DEBUG ((DEBUG_INFO, "EP0 was already in SETUP phase\n"));
	        return EFI_SUCCESS;
	}

	core_handle->ep_handles[0].state = USB_EP_STATE_SETUP;
	trb = core_handle->trbs;
	DEBUG ((DEBUG_INFO, "(dwc_xdci_core_start_ep0_setup_xfer)\n"));

	status = dwc_xdci_core_init_trb (
	        core_handle,
	        trb,
	        TRBCTL_SETUP,
	        core_handle->aligned_setup_buffer,
	        8
	        );

	if (status)
	        return status;

	//
	// Issue a DEPSTRTXFER for EP0
	// Reset params
	//
	ep_cmd_params.param0 = ep_cmd_params.param1 = ep_cmd_params.param2 = 0;

	//
	// Init the lower re-bits for TRB address
	//
	ep_cmd_params.param1 = (UINT32)(UINTN)trb;

	//
	// Issue the command to start transfer on physical
	// endpoint 0
	//
	status = dwc_xdci_core_issue_ep_cmd (
	        core_handle,
	        0,
	        EPCMD_START_XFER,
	        &ep_cmd_params
	        );

	/* Save new resource index for this transfer */
	core_handle->ep_handles[0].currentXferRscIdx = ((usb_reg_read (
	                                                         core_handle->base_address,
	                                                         DWC_XDCI_EPCMD_REG(0)) & DWC_XDCI_EPCMD_RES_IDX_MASK) >> DWC_XDCI_EPCMD_RES_IDX_BIT_POS
	        );

	return status;
}


/**
   Internal function:
   This function is used to process the state change event
   @core_handle: xDCI controller handle address
   @event: device event dword

**/
STATIC
EFI_STATUS
dwc_xdci_process_device_state_change_event (
	IN XDCI_CORE_HANDLE    *core_handle,
	IN UINT32              event
	)
{
	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_process_device_state_change_event: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	core_handle->hird_val = (event & DWC_XDCI_EVENT_BUFF_DEV_HIRD_MASK) >> DWC_XDCI_EVENT_BUFF_DEV_HIRD_BIT_POS;

	core_handle->link_state = ((event & DWC_XDCI_EVENT_BUFF_DEV_LINK_STATE_MASK) >> DWC_XDCI_EVENT_BUFF_DEV_LINK_STATE_BIT_POS);

	if (core_handle->event_callbacks.dev_link_state_callback) {
	        core_handle->event_callbacks.cb_event_params.parent_handle = core_handle->parent_handle;
	        core_handle->event_callbacks.cb_event_params.link_state = core_handle->link_state;
	        core_handle->event_callbacks.cb_event_params.hird = core_handle->hird_val;
	        core_handle->event_callbacks.cb_event_params.ss_event = (event & DWC_XDCI_EVENT_BUFF_DEV_SS_EVENT_MASK) ? 1 : 0;
	        core_handle->event_callbacks.dev_link_state_callback (&core_handle->event_callbacks.cb_event_params);
	}

	return EFI_SUCCESS;
}


/**
   Internal function:
   This function is used to issue a command to end transfer
   @core_handle: xDCI controller handle address
   @ep_num: Physical EP num for which transfer is to be ended

**/
STATIC
EFI_STATUS
dwc_xdci_end_xfer (
	IN XDCI_CORE_HANDLE    *core_handle,
	IN UINT32              ep_num
	)
{
	EFI_STATUS                      status;
	DWC_XDCI_ENDPOINT_CMD_PARAMS    ep_cmd_params;
	UINT32                          cmd_params;
	DWC_XDCI_TRB                    *TrbPtr;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_end_xfer: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	core_handle->ep_handles[ep_num].CheckFlag = FALSE;

	/* Issue a DEPENDXFER for EP */
	/* Reset params */
	ep_cmd_params.param0 = ep_cmd_params.param1 = ep_cmd_params.param2 = 0;

	cmd_params = ((core_handle->ep_handles[ep_num].currentXferRscIdx << DWC_XDCI_EPCMD_RES_IDX_BIT_POS) | DWC_XDCI_EPCMD_FORCE_RM_MASK);

	if (core_handle->ep_handles[ep_num].currentXferRscIdx == 0) {
	        return EFI_SUCCESS;
	}
	/* Issue the command */
	status = dwc_xdci_core_issue_ep_cmd(
	        core_handle,
	        ep_num,
	        cmd_params | DWC_XDCI_EPCMD_END_XFER,
	        &ep_cmd_params
	        );

	if (!status) {
	        core_handle->ep_handles[ep_num].currentXferRscIdx = 0;
	        TrbPtr = core_handle->trbs + (ep_num * DWC_XDCI_TRB_NUM);
	        memset (TrbPtr, 0, DWC_XDCI_TRB_NUM * sizeof (DWC_XDCI_TRB));
	}

	return status;
}


/**
   Internal function:
   This function is used to process bus reset detection event
   @core_handle: xDCI controller handle address

**/
STATIC
EFI_STATUS
dwc_xdci_process_device_reset_det (
	IN XDCI_CORE_HANDLE    *core_handle
	)
{
	EFI_STATUS  status = EFI_SUCCESS;

	if (core_handle == NULL) {
	        return EFI_DEVICE_ERROR;
	}

	/* Flush all FIFOs */
	status = dwc_xdci_core_flush_all_fifos(core_handle);
	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_process_device_reset_det: Failed to flush FIFOs\n"));
	}

	/* NOTE: Not treating flush FIFOs status to be fatal */

	/* Start SETUP phase on EP0 */
	status = dwc_xdci_core_start_ep0_setup_xfer(core_handle);

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_process_device_reset_det: Failed to start SETUP phase for EP0\n"));
	        return status;
	}

	/* Notify upper layer if a callback is registerd for
	 *  this event
	 */
	if (core_handle->event_callbacks.dev_bus_reset_callback) {
	        core_handle->event_callbacks.cb_event_params.parent_handle = core_handle->parent_handle;
	        status = core_handle->event_callbacks.dev_bus_reset_callback (&core_handle->event_callbacks.cb_event_params);
	}

	return status;
}


/**
   Internal function:
   This function is used to process connection done (means reset
   complete) event
   @core_handle: xDCI controller handle address

**/
STATIC
EFI_STATUS
dwc_xdci_process_device_reset_done (
	IN XDCI_CORE_HANDLE    *core_handle
	)
{
	DWC_XDCI_ENDPOINT_CMD_PARAMS    ep_cmd_params;
	UINT32                          base_addr;
	EFI_STATUS                      status = EFI_SUCCESS;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_process_device_reset_done: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	base_addr = core_handle->base_address;
	core_handle->actual_speed = (usb_reg_read (base_addr, DWC_XDCI_DSTS_REG) & DWC_XDCI_DSTS_CONN_SPEED_MASK);
	DEBUG ((DEBUG_INFO, "dwc_xdci_process_device_reset_done core_handle->actual_speed is %x\n", core_handle->actual_speed));

	/* Program MPS based on the negotiated speed */
	dwc_xdci_core_get_ctrl_mps (core_handle, &core_handle->ep_handles[0].ep_info.max_pkt_size);
	dwc_xdci_core_get_ctrl_mps (core_handle, &core_handle->ep_handles[1].ep_info.max_pkt_size);

	/* Init DEPCFG cmd params for EP0 */
	status = dwc_xdci_core_init_ep_cmd_params (
	        core_handle,
	        &core_handle->ep_handles[0].ep_info,
	        DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_MDFY_STATE,
	        EPCMD_SET_EP_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        return status;
	}

	/* Issue the command */
	status = dwc_xdci_core_issue_ep_cmd (
	        core_handle,
	        0,
	        EPCMD_SET_EP_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        return status;
	}

	/* Init DEPCFG cmd params for EP1 */
	status = dwc_xdci_core_init_ep_cmd_params (
	        core_handle,
	        &core_handle->ep_handles[1].ep_info,
	        DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_MDFY_STATE,
	        EPCMD_SET_EP_CONFIG,
	        &ep_cmd_params
	        );

	/* Issue the command */
	status = dwc_xdci_core_issue_ep_cmd (
	        core_handle,
	        1,
	        EPCMD_SET_EP_CONFIG,
	        &ep_cmd_params
	        );

	/* Put the other PHY into suspend */
	if (core_handle->actual_speed == USB_SPEED_SUPER) {
	        /* Put HS PHY to suspend */
	        usb_reg_write (
	                base_addr,
	                DWC_XDCI_GUSB2PHYCFG_REG (0),
	                (usb_reg_read (base_addr, DWC_XDCI_GUSB2PHYCFG_REG(0)) | DWC_XDCI_GUSB2PHYCFG_SUSPEND_PHY_MASK)
	                );

	        /* Clear SS PHY's suspend mask */
	        usb_reg_write (
	                base_addr,
	                DWC_XDCI_GUSB3PIPECTL_REG (0),
	                (usb_reg_read (base_addr, DWC_XDCI_GUSB3PIPECTL_REG(0)) & ~DWC_XDCI_GUSB3PIPECTL_SUSPEND_PHY_MASK)
	                );

	} else {
	        /* Put SS PHY to suspend */
	        usb_reg_write (
	                base_addr,
	                DWC_XDCI_GUSB3PIPECTL_REG(0),
	                (usb_reg_read(base_addr, DWC_XDCI_GUSB3PIPECTL_REG(0)) | DWC_XDCI_GUSB3PIPECTL_SUSPEND_PHY_MASK)
	                );

	        /* Clear HS PHY's suspend mask */
	        usb_reg_write (
	                base_addr,
	                DWC_XDCI_GUSB2PHYCFG_REG(0),
	                (usb_reg_read(base_addr, DWC_XDCI_GUSB2PHYCFG_REG(0)) & ~DWC_XDCI_GUSB2PHYCFG_SUSPEND_PHY_MASK)
	                );
	}

	/* Notify upper layer if callback is registered */
	if (core_handle->event_callbacks.dev_reset_done_callback) {
	        core_handle->event_callbacks.cb_event_params.parent_handle = core_handle->parent_handle;
	        core_handle->event_callbacks.cb_event_params.speed = core_handle->actual_speed;
	        core_handle->event_callbacks.dev_reset_done_callback (&core_handle->event_callbacks.cb_event_params);
	}

	return status;
}


/**
   Internal function:
   This function is used to process device event
   @core_handle: xDCI controller handle address
   @int_line_event_buffer: event buffer pointing to device event
   @processed_event_size: address of variable to save the size of
   the event that was processed

**/
STATIC
EFI_STATUS
dwc_xdci_process_device_event (
	IN XDCI_CORE_HANDLE         *core_handle,
	IN DWC_XDCI_EVENT_BUFFER    *int_line_event_buffer,
	IN UINT32                   *processed_event_size
	)
{
	UINT32 event;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_process_device_event: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	/* Extract device event */
	event = (int_line_event_buffer->event & DWC_XDCI_EVENT_BUFF_DEV_EVT_MASK);
	event >>= DWC_XDCI_EVENT_BUFF_DEV_EVT_BIT_POS;

	/* Assume default event size. Change it in switch case if
	 *  different
	 */
	*processed_event_size = DWC_XDCI_DEV_EVENT_DEFAULT_SIZE_IN_BYTES;

	switch (event) {
	case DWC_XDCI_EVENT_BUFF_DEV_DISCONN_EVENT:
	        DEBUG ((DEBUG_INFO, "Device DWC_XDCI_EVENT_BUFF_DEV_DISCONN_EVENT\n"));
	        break;

	case DWC_XDCI_EVENT_BUFF_DEV_USB_RESET_EVENT:
	        DEBUG ((DEBUG_INFO, "Device DWC_XDCI_EVENT_BUFF_DEV_USB_RESET_EVENT\n"));
	        dwc_xdci_process_device_reset_det (core_handle);
	        break;

	case DWC_XDCI_EVENT_BUFF_DEV_CONN_DONE_EVENT:
	        DEBUG ((DEBUG_INFO, "Device DWC_XDCI_EVENT_BUFF_DEV_CONN_DONE_EVENT\n"));
	        dwc_xdci_process_device_reset_done (core_handle);
	        break;

	case DWC_XDCI_EVENT_BUFF_DEV_STATE_CHANGE_EVENT:
	        DEBUG ((DEBUG_INFO, "Device DWC_XDCI_EVENT_BUFF_DEV_STATE_CHANGE_EVENT\n"));
	        dwc_xdci_process_device_state_change_event (core_handle, int_line_event_buffer->event);
	        break;

	case DWC_XDCI_EVENT_BUFF_DEV_WKUP_EVENT:
	        DEBUG ((DEBUG_INFO, "Device DWC_XDCI_EVENT_BUFF_DEV_WKUP_EVENT\n"));
	        break;

	case DWC_XDCI_EVENT_BUFF_DEV_HBRNTN_REQ_EVENT:
	        DEBUG ((DEBUG_INFO, "Device DWC_XDCI_EVENT_BUFF_DEV_HBRNTN_REQ_EVENT\n"));
	        break;

	case DWC_XDCI_EVENT_BUFF_DEV_SOF_EVENT:
	        DEBUG ((DEBUG_INFO, "Device DWC_XDCI_EVENT_BUFF_DEV_SOF_EVENT\n"));
	        break;

	case DWC_XDCI_EVENT_BUFF_DEV_ERRATIC_ERR_EVENT:
	        DEBUG ((DEBUG_INFO, "Device DWC_XDCI_EVENT_BUFF_DEV_ERRATIC_ERR_EVENT\n"));
	        break;

	case DWC_XDCI_EVENT_BUFF_DEV_CMD_CMPLT_EVENT:
	        DEBUG ((DEBUG_INFO, "Device DWC_XDCI_EVENT_BUFF_DEV_CMD_CMPLT_EVENT\n"));
	        break;

	case DWC_XDCI_EVENT_BUFF_DEV_BUFF_OVFL_EVENT:
	        DEBUG ((DEBUG_INFO, "Device DWC_XDCI_EVENT_BUFF_DEV_BUFF_OVFL_EVENT\n"));
	        break;

	case DWC_XDCI_EVENT_BUFF_DEV_TST_LMP_RX_EVENT:
	        DEBUG ((DEBUG_INFO, "Device DWC_XDCI_EVENT_BUFF_DEV_TST_LMP_RX_EVENT\n"));
	        *processed_event_size = DWC_XDCI_DEV_EVENT_TST_LMP_SIZE_IN_BYTES;
	        break;

	default:
	        DEBUG ((DEBUG_INFO, "dwc_xdci_process_device_event: UNHANDLED device event: %x\n", event));
	        break;
	}

	return EFI_SUCCESS;
}


/**
   Internal function:
   This function is used to process EP not ready for
   non-control endpoints
   @core_handle: xDCI controller handle address
   @ep_num: Physical endpoint number

**/
STATIC
EFI_STATUS
dwc_xdci_process_ep_xfer_not_ready (
	__attribute__((__unused__)) IN XDCI_CORE_HANDLE    *core_handle,
	__attribute__((__unused__)) IN UINT32              ep_num
	)
{
	/* TODO: Not doing on-demand transfers
	 * Revisit if required for later use
	 */
	return EFI_SUCCESS;
}


/**
   Internal function:
   This function is used to process EP not ready for
   control endpoints
   @core_handle: xDCI controller handle address
   @ep_num: Physical endpoint number
   @data_stage: EP not ready when data stage token was received
   @status_stage: EP not ready when status stage token was received

**/
STATIC
EFI_STATUS
dwc_xdci_process_ep0_xfer_not_ready (
	IN XDCI_CORE_HANDLE    *core_handle,
	IN UINT32              ep_num,
	IN UINT32              ep_event_status
	)
{
	USB_EP_STATE        ep_state = USB_EP_STATE_SETUP;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_process_ep0_xfer_not_ready: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}
	/* Is it data stage or status stage */
	if (ep_event_status & DWC_XDCI_EVENT_BUFF_EP_CTRL_DATA_REQ_MASK) {
	        ep_state = USB_EP_STATE_DATA;
	} else if (ep_event_status & DWC_XDCI_EVENT_BUFF_EP_CTRL_STATUS_REQ_MASK) {
	        ep_state = USB_EP_STATE_STATUS;
	}

	if ((ep_num == 0) && (ep_state == USB_EP_STATE_STATUS)) {
	        if (ep_event_status & DWC_XDCI_EVENT_BUFF_EP_XFER_ACTIVE_MASK) {
	                DEBUG ((DEBUG_INFO, "XFER_ACTIVE\n"));
	        } else {
	                DEBUG ((DEBUG_INFO, "XFER_NOT_ACTIVE\n"));
	        }
	        dwc_xdci_ep0_receive_status_pkt (core_handle);
	}

	/* Notify upper layer if a callback is registered for
	 * this event
	 */
	if (core_handle->event_callbacks.dev_xfer_nrdy_callback) {
	        core_handle->event_callbacks.cb_event_params.parent_handle = core_handle->parent_handle;
	        core_handle->event_callbacks.cb_event_params.ep_state = ep_state;
	        core_handle->event_callbacks.dev_xfer_nrdy_callback (&core_handle->event_callbacks.cb_event_params);
	}

	return EFI_SUCCESS;
}


/**
   Internal function:
   This function is used to process transfer phone done for EP0
   @core_handle: xDCI controller handle address
   @ep_num: Physical endpoint number (0 for OUT and 1 for IN)

**/
STATIC
EFI_STATUS
dwc_xdci_process_ep0_xfer_phase_done (
	IN XDCI_CORE_HANDLE    *core_handle,
	IN UINT32              ep_num
	)
{
	DWC_XDCI_ENDPOINT    *ep_handle;
	DWC_XDCI_TRB         *trb;
	EFI_STATUS           status = EFI_SUCCESS;
	UINT32               trb_sts;
	UINT32               trb_ctrl;
	UINT32               trb_bufsize;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_process_ep0_xfer_phase_done: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	ep_handle = &core_handle->ep_handles[ep_num];
	trb = core_handle->trbs + (ep_num * DWC_XDCI_TRB_NUM);
	DEBUG ((DEBUG_INFO, "(dwc_xdci_process_ep0_xfer_phase_done)ep_num is %d\n", ep_num));

	if (trb->trb_ctrl & DWC_XDCI_TRB_CTRL_HWO_MASK) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_process_ep0_xfer_phase_done. HW owns TRB: %x!!!\n", (UINT32)(UINTN)trb));
	}

	ep_handle->currentXferRscIdx = 0;
	ep_handle->state = USB_EP_STATE_ENABLED;
	trb_ctrl = (trb->trb_ctrl & DWC_XDCI_TRB_CTRL_TYPE_MASK) >> DWC_XDCI_TRB_CTRL_TYPE_BIT_POS;
	trb_sts = (trb->len_xfer_params & DWC_XDCI_TRB_STATUS_MASK) >> DWC_XDCI_TRB_STATUS_BIT_POS;
	trb_bufsize = trb->len_xfer_params & DWC_XDCI_TRB_BUFF_SIZE_MASK;

	switch (trb_ctrl) {
	case DWC_XDCI_TRB_CTRL_TYPE_SETUP:
	        DEBUG ((DEBUG_INFO, "SETUP\n"));
	        DEBUG ((DEBUG_INFO, "param buff: 0x%x\n", (unsigned) core_handle->aligned_setup_buffer));

	        if (core_handle->event_callbacks.dev_setup_pkt_received_callback) {
	                core_handle->event_callbacks.cb_event_params.parent_handle = core_handle->parent_handle;
	                core_handle->event_callbacks.cb_event_params.buffer = core_handle->aligned_setup_buffer;
	                status = core_handle->event_callbacks.dev_setup_pkt_received_callback (&core_handle->event_callbacks.cb_event_params);
	        }

	        if (!(core_handle->aligned_setup_buffer[0] & USB_SETUP_DATA_PHASE_DIRECTION_MASK)) {
	                /* Keep a buffer ready for setup phase */
	                dwc_xdci_core_start_ep0_setup_xfer (core_handle);
	        }

	        break;

	case DWC_XDCI_TRB_CTRL_TYPE_STATUS2:
	        DEBUG ((DEBUG_INFO, "STATUS2\n"));
	        break;

	case DWC_XDCI_TRB_CTRL_TYPE_STATUS3:
	        DEBUG ((DEBUG_INFO, "STATUS3\n"));
	        /* Notify upper layer of control transfer completion
	         * if a callback function was registerd
	         */
	        if (core_handle->event_callbacks.dev_xfer_done_callback) {
	                core_handle->event_callbacks.cb_event_params.parent_handle = core_handle->parent_handle;
	                core_handle->event_callbacks.cb_event_params.ep_num = (ep_num >> 1);
	                core_handle->event_callbacks.cb_event_params.ep_dir = (ep_num & 1);
	                core_handle->event_callbacks.cb_event_params.buffer = (UINT8 *)(UINTN)(trb->buff_ptr_low);
	                core_handle->event_callbacks.dev_xfer_done_callback (&core_handle->event_callbacks.cb_event_params);
	        }

	        /* Status phase done. Queue next SETUP packet */
	        status = dwc_xdci_core_start_ep0_setup_xfer(core_handle);

	        if (status) {
	                DEBUG ((DEBUG_INFO, "dwc_xdci_process_ep0_xfer_phase_done: FAILED to queue SETUP\n"));
	        }
	        break;

	case DWC_XDCI_TRB_CTRL_TYPE_DATA:
	        DEBUG ((DEBUG_INFO, "DATA\n"));
	        if (trb_sts == DWC_XDCI_TRB_STATUS_SETUP_PENDING || trb_bufsize != 0) {
	                DEBUG ((DEBUG_INFO, "ERROR: Control transfert aborted by host: Setup pending\n"));
	                dwc_xdci_core_start_ep0_setup_xfer (core_handle);
	        }

	        if (core_handle->event_callbacks.dev_xfer_done_callback) {
	                core_handle->event_callbacks.cb_event_params.parent_handle = core_handle->parent_handle;
	                core_handle->event_callbacks.cb_event_params.ep_num = (ep_num >> 1);
	                core_handle->event_callbacks.cb_event_params.ep_dir = (ep_num & 1);
	                core_handle->event_callbacks.cb_event_params.buffer = (UINT8 *)(UINTN)(trb->buff_ptr_low);
	                core_handle->event_callbacks.dev_xfer_done_callback (&core_handle->event_callbacks.cb_event_params);
	        }
	        break;

	default:
	        DEBUG ((DEBUG_INFO, "dwc_xdci_process_ep0_xfer_phase_done: UNHANDLED STATE in TRB\n"));
	        break;
	}

	return status;
}


/**
   Internal function:
   This function is used to process transfer done for
   non-control endpoints
   @core_handle: xDCI controller handle address
   @ep_num: Physical endpoint number

**/
STATIC
EFI_STATUS
dwc_xdci_process_ep_xfer_done (
	IN XDCI_CORE_HANDLE    *core_handle,
	IN UINT32              ep_num
	)
{
	DWC_XDCI_ENDPOINT    *ep_handle;
	DWC_XDCI_TRB         *trb;
	USB_XFER_REQUEST     *xfer_req;
	UINT32               remaining_len;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_process_ep_xfer_done: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	ep_handle = &core_handle->ep_handles[ep_num];
	ep_handle->currentXferRscIdx = 0;
	trb = ep_handle->trb;
	xfer_req = &ep_handle->xfer_handle;

	//
	// if transfer done, set CheckFlag to FALSE for allow next transfer request.
	//
	ep_handle->CheckFlag = FALSE;

	if ((trb == NULL) || (xfer_req == NULL)) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_process_ep_xfer_done: INVALID parameter\n"));
	        return EFI_INVALID_PARAMETER;
	}

	/* Compute the actual transfer length */
	xfer_req->actual_xfer_len = xfer_req->xfer_len;
	remaining_len = (trb->len_xfer_params & DWC_XDCI_TRB_BUFF_SIZE_MASK);

	if (remaining_len > xfer_req->xfer_len) {
	        /* Buffer overrun? This should never happen */
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_process_ep_xfer_done: Possible buffer overrun\n"));
	} else {
	        xfer_req->actual_xfer_len -= remaining_len;
	}

	/* Notify upper layer of request-specific transfer completion
	 * if there is a callback specifically for this request
	 */
	if (xfer_req->xfer_done) {
	        xfer_req->xfer_done(core_handle->parent_handle, xfer_req);
	}

	/* Notify upper layer if a callback was registered */
	if (core_handle->event_callbacks.dev_xfer_done_callback) {
	        core_handle->event_callbacks.cb_event_params.parent_handle = core_handle->parent_handle;
	        core_handle->event_callbacks.cb_event_params.ep_num = (ep_num >> 1);
	        core_handle->event_callbacks.cb_event_params.ep_dir = (ep_num & 1);
	        core_handle->event_callbacks.cb_event_params.ep_type = ep_handle->ep_info.ep_type;
	        core_handle->event_callbacks.cb_event_params.buffer = (UINT8 *)(UINTN)(ep_handle->trb->buff_ptr_low);
	        core_handle->event_callbacks.dev_xfer_done_callback (&core_handle->event_callbacks.cb_event_params);
	}

	return EFI_SUCCESS;
}


/**
   Internal function:
   This function is used to process endpoint events
   @core_handle: xDCI controller handle address
   @int_line_event_buffer:  address of buffer containing event
   to process
   @processed_event_size: address to save the size of event
   processed

**/
STATIC
EFI_STATUS
dwc_xdci_process_ep_event (
	IN XDCI_CORE_HANDLE         *core_handle,
	IN DWC_XDCI_EVENT_BUFFER    *int_line_event_buffer,
	IN UINT32                   *processed_event_size
	)
{
	UINT32          ep_num;
	UINT32          ep_event;
	UINT32          ep_event_status;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_process_ep_event: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	ep_event = int_line_event_buffer->event;
	DEBUG ((DEBUG_INFO, "dwc_xdci_process_ep_event event: 0x%x\n", int_line_event_buffer->event));
	DEBUG ((DEBUG_INFO, "dwc_xdci_process_ep_event lmp1: 0x%x\n", int_line_event_buffer->dev_tst_lmp1));
	DEBUG ((DEBUG_INFO, "dwc_xdci_process_ep_event lmp2: 0x%x\n", int_line_event_buffer->dev_tst_lmp2));
	DEBUG ((DEBUG_INFO, "dwc_xdci_process_ep_event reserved: 0x%x\n", int_line_event_buffer->reserved));

	*processed_event_size = DWC_XDCI_DEV_EVENT_DEFAULT_SIZE_IN_BYTES;

	/* Get EP num */
	ep_num = ((ep_event & DWC_XDCI_EVENT_BUFF_EP_NUM_MASK) >> DWC_XDCI_EVENT_BUFF_EP_NUM_BIT_POS);
	ep_event_status = (ep_event & DWC_XDCI_EVENT_BUFF_EP_EVENT_STATUS_MASK);

	/* Interpret event and handle transfer completion here */
	ep_event = ((ep_event & DWC_XDCI_EVENT_BUFF_EP_EVENT_MASK) >> DWC_XDCI_EVENT_BUFF_EP_EVENT_BIT_POS);

	switch (ep_event) {
	case DWC_XDCI_EVENT_BUFF_EP_XFER_CMPLT:
	        DEBUG ((DEBUG_INFO, "XFER_CMPLT ep %d\n", ep_num));
	        if (ep_num > 1) {
	                dwc_xdci_process_ep_xfer_done (core_handle, ep_num);
	        } else {
	                dwc_xdci_process_ep0_xfer_phase_done (core_handle, ep_num);
	        }
	        break;

	case DWC_XDCI_EVENT_BUFF_EP_XFER_IN_PROGRESS:
	        DEBUG ((DEBUG_INFO, "IN_PROGRESS\n"));
	        break;

	case DWC_XDCI_EVENT_BUFF_EP_XFER_NOT_READY:
	        DEBUG ((DEBUG_INFO, "NOT_READY ep %d\n", ep_num));
	        if (ep_num > 1) {
	                /* Endpoint transfer is not ready */
	                dwc_xdci_process_ep_xfer_not_ready (core_handle, ep_num);
	        } else {
	                dwc_xdci_process_ep0_xfer_not_ready (core_handle, ep_num, ep_event_status);
	        }
	        break;

	default:
	        DEBUG ((DEBUG_INFO, "dwc_xdci_process_ep_event: UNKNOWN EP event 0x%x\n", ep_event));
	        break;
	}

	return EFI_SUCCESS;
}


/**
   Internal function:
   This function is used to process events on single interrupt line
   @core_handle: xDCI controller handle address
   @event_count:  event bytes to process
   @processed_event_count: address to save the size
   (in bytes) of event processed
   processed

**/
STATIC
EFI_STATUS
dwc_xdci_process_interrupt_line_events (
	IN XDCI_CORE_HANDLE    *core_handle,
	IN UINT32              event_count,
	IN UINT32              *processed_event_count
	)
{
	UINT32    processed_event_size = 0;
	UINT32    current_event_addr;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_process_interrupt_line_events: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	if (core_handle->current_event_buffer == NULL) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_process_interrupt_line_events: INVALID event buffer\n"));
	        return EFI_INVALID_PARAMETER;
	}

	current_event_addr = (UINT32)(UINTN)(core_handle->current_event_buffer);

	/* Process event_count/event_size number of events
	 * in this run
	 */
	while (event_count) {
	        if (core_handle->current_event_buffer->event & DWC_XDCI_EVENT_DEV_MASK) {
	                dwc_xdci_process_device_event (
	                        core_handle,
	                        core_handle->current_event_buffer,
	                        &processed_event_size
	                        );
	        } else {
	                dwc_xdci_process_ep_event (
	                        core_handle,
	                        core_handle->current_event_buffer,
	                        &processed_event_size);
	        }

	        event_count -= processed_event_size;
	        *processed_event_count += processed_event_size;
	        if ((current_event_addr + processed_event_size) >=
	            ((UINT32)(UINTN)(core_handle->aligned_event_buffers) + (sizeof(DWC_XDCI_EVENT_BUFFER) * DWC_XDCI_MAX_EVENTS_PER_BUFFER))
	                ) {
	                current_event_addr = (UINT32)(UINTN)(core_handle->aligned_event_buffers);
	                DEBUG ((DEBUG_INFO, "dwc_xdci_process_interrupt_line_events: Event buffer bound reached\n"));
	        } else {
	                current_event_addr += processed_event_size;
	        }

	        core_handle->current_event_buffer = (DWC_XDCI_EVENT_BUFFER *)(UINTN)current_event_addr;
	}

	return EFI_SUCCESS;
}


/* DWC XDCI APIs */

/**
   Interface:

   This function is used to initialize the xDCI core
   @config_params: Parameters from app to configure the core
   @device_core_ptr:  HW-independent APIs handle for device core
   @core_handle: xDCI controller handle retured

**/
EFI_STATUS
EFIAPI
dwc_xdci_core_init (
	IN USB_DEV_CONFIG_PARAMS    *ConfigParams,
	IN VOID                     *device_core_ptr,
	IN VOID                     **core_handle
	)
{
	EFI_STATUS                      status = EFI_DEVICE_ERROR;
	UINT32                          base_addr;
	XDCI_CORE_HANDLE                *local_core_handle;
	DWC_XDCI_ENDPOINT_CMD_PARAMS    ep_cmd_params;
	UINT32                          max_delay_iter = DWC_XDCI_MAX_DELAY_ITERATIONS;
	UINT8                           i;

	if (core_handle == NULL) {
	        return EFI_INVALID_PARAMETER;
	}

	local_core_handle = (XDCI_CORE_HANDLE *)calloc (1, sizeof(XDCI_CORE_HANDLE));

	if (local_core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to allocate handle for xDCI\n"));
	        return EFI_OUT_OF_RESOURCES;
	}

	memset (local_core_handle, 0, sizeof(XDCI_CORE_HANDLE));

	local_core_handle->parent_handle = device_core_ptr;

	*core_handle = (VOID *)local_core_handle;

	local_core_handle->id = ConfigParams->ControllerId;
	local_core_handle->base_address = base_addr = ConfigParams->BaseAddress;
	local_core_handle->flags = ConfigParams->Flags;
	local_core_handle->desired_speed = local_core_handle->actual_speed = ConfigParams->Speed;
	local_core_handle->role = ConfigParams->Role;

	DEBUG ((DEBUG_INFO, "Resetting the USB core\n"));
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DCTL_REG,
	        usb_reg_read (base_addr, DWC_XDCI_DCTL_REG) | DWC_XDCI_DCTL_CSFTRST_MASK
	        );

	/* Wait until core soft reset completes */
	do {
	        if (!(usb_reg_read (base_addr, DWC_XDCI_DCTL_REG) & DWC_XDCI_DCTL_CSFTRST_MASK)) {
	                break;
	        } else {
	                udelay(DWC_XDCI_MAX_DELAY_ITERATIONS);
	        }
	} while (--max_delay_iter);

	if (!max_delay_iter) {
	        DEBUG ((DEBUG_INFO, "Failed to reset device controller\n"));
	        return EFI_DEVICE_ERROR;
	}

	DEBUG ((DEBUG_INFO, "USB core has been reset\n"));

	/* All FIFOs are flushed at this point */

	/* Ensure we have EP0 Rx/Tx handles initialized */
	local_core_handle->ep_handles[0].ep_info.ep_num = 0;
	local_core_handle->ep_handles[0].ep_info.ep_dir = UsbEpDirOut;
	local_core_handle->ep_handles[0].ep_info.ep_type = USB_ENDPOINT_CONTROL;
	local_core_handle->ep_handles[0].ep_info.max_pkt_size = DWC_XDCI_SS_CTRL_EP_MPS;
	/* 0 means burst size of 1 */
	local_core_handle->ep_handles[0].ep_info.burst_size = 0;

	local_core_handle->ep_handles[1].ep_info.ep_num = 0;
	local_core_handle->ep_handles[1].ep_info.ep_dir = UsbEpDirIn;
	local_core_handle->ep_handles[1].ep_info.ep_type = USB_ENDPOINT_CONTROL;
	local_core_handle->ep_handles[1].ep_info.max_pkt_size = DWC_XDCI_SS_CTRL_EP_MPS;
	/* 0 means burst size of 1 */
	local_core_handle->ep_handles[1].ep_info.burst_size = 0;

	local_core_handle->dev_state = UsbDevStateDefault;

	/* Clear KeepConnect bit so we can allow disconnect and
	 * re-connect. Stay in RX_DETECT state
	 */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DCTL_REG,
	        usb_reg_read (base_addr, DWC_XDCI_DCTL_REG) &
	        (~DWC_XDCI_DCTL_KEEP_CONNECT_MASK) &
	        ((~DWC_XDCI_DCTL_STATE_CHANGE_REQ_MASK) | (DWC_XDCI_DCTL_STATE_CHANGE_REQ_RX_DETECT << DWC_XDCI_DCTL_STATE_CHANGE_REQ_BIT_POS))
	        );

	DEBUG ((DEBUG_INFO, "Device controller Synopsys ID: %x\n", usb_reg_read (base_addr, DWC_XDCI_GSNPSID_REG)));
	DEBUG ((DEBUG_INFO, "Default value of xDCI GSBUSCFG0 and GSBUSCFG1: %x, %x\n",
	        usb_reg_read (base_addr, DWC_XDCI_GSBUSCFG0_REG),
	        usb_reg_read (base_addr, DWC_XDCI_GSBUSCFG1_REG)));

	DEBUG ((DEBUG_INFO, "Default value of xDCI GTXTHRCFG and GRXTHRCFG: %x, %x\n",
	        usb_reg_read (base_addr, DWC_XDCI_GTXTHRCFG_REG),
	        usb_reg_read (base_addr, DWC_XDCI_GRXTHRCFG_REG)));

	/* Clear ULPI auto-resume bit */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_GUSB2PHYCFG_REG (0),
	        (usb_reg_read (base_addr, DWC_XDCI_GUSB2PHYCFG_REG (0)) & ~DWC_XDCI_GUSB2PHYCFG_ULPI_AUTO_RESUME_MASK)
	        );

	DEBUG ((DEBUG_INFO, "Default value of xDCI GUSB2PHYCFG and GUSB3PIPECTL: %x, %x\n",
	        usb_reg_read (base_addr, DWC_XDCI_GUSB2PHYCFG_REG (0)),
	        usb_reg_read (base_addr, DWC_XDCI_GUSB3PIPECTL_REG (0))));

	/* Only one RxFIFO */
	DEBUG ((DEBUG_INFO, "Default value of DWC_XDCI_GRXFIFOSIZ: %x\n",
	        usb_reg_read (base_addr, DWC_XDCI_GRXFIFOSIZ_REG (0))));

	for (i = 0; i < DWC_XDCI_MAX_ENDPOINTS; i++) {
	        DEBUG ((DEBUG_INFO, "Default value of xDCI DWC_XDCI_GTXFIFOSIZ %d: %x\n",
	                i, usb_reg_read (base_addr, DWC_XDCI_GTXFIFOSIZ_REG (i))));
	}

	/* TODO: Need to check if TxFIFO should start where RxFIFO ends
	 * or default is correct i.e. TxFIFO starts at 0 just like RxFIFO
	 */

	/* Allocate and Initialize Event Buffers */
	local_core_handle->max_dev_int_lines = ((usb_reg_read (base_addr, DWC_XDCI_GHWPARAMS1_REG) &
	                                         DWC_XDCI_GHWPARAMS1_NUM_INT_MASK) >>
	                                        DWC_XDCI_GHWPARAMS1_NUM_INT_BIT_POS);

	DEBUG ((DEBUG_INFO, "Max dev int lines: %d\n", local_core_handle->max_dev_int_lines));

	/* One event buffer per interrupt line.
	 *  Need to align it to size of event buffer
	 *  Buffer needs to be big enough. Otherwise the core
	 *  won't operate
	 */
	local_core_handle->aligned_event_buffers = (DWC_XDCI_EVENT_BUFFER *)
	        ((UINT32)(UINTN)(local_core_handle->event_buffers) +
	         ((sizeof (DWC_XDCI_EVENT_BUFFER) * DWC_XDCI_MAX_EVENTS_PER_BUFFER) -
	          (((UINT32)(UINTN)(local_core_handle->event_buffers)) %
	           (sizeof (DWC_XDCI_EVENT_BUFFER) * DWC_XDCI_MAX_EVENTS_PER_BUFFER))));

	for (i = 0; i < local_core_handle->max_dev_int_lines; i++) {
	        usb_reg_write (
	                base_addr,
	                DWC_XDCI_GEVNTADR_REG (i),
	                (UINT32)(UINTN)(local_core_handle->aligned_event_buffers + i * sizeof(DWC_XDCI_EVENT_BUFFER) * DWC_XDCI_MAX_EVENTS_PER_BUFFER)
	                );

	        //
	        // Clear High 32bit address register, GEVNTADR register is 64-bit register
	        // default is 0xffffffffffffffff
	        //
	        usb_reg_write (base_addr, DWC_XDCI_GEVNTADR_REG (i) + 4, 0x00000000);

	        local_core_handle->current_event_buffer = local_core_handle->aligned_event_buffers;
	        /* Write size and clear the mask */
	        usb_reg_write (
	                base_addr,
	                DWC_XDCI_EVNTSIZ_REG (i),
	                sizeof (DWC_XDCI_EVENT_BUFFER) * DWC_XDCI_MAX_EVENTS_PER_BUFFER
	                );

	        /* Write 0 to the event count register as the last step
	         *  for event configuration
	         */
	        usb_reg_write (base_addr, DWC_XDCI_EVNTCOUNT_REG (i), 0);

	        DEBUG ((DEBUG_INFO, "Value of xDCI Event buffer %d: %x%x, Size: %x, Count: %x\n",
	                i,
	                usb_reg_read (base_addr, DWC_XDCI_GEVNTADR_REG (i) + 4),
	                usb_reg_read (base_addr, DWC_XDCI_GEVNTADR_REG (i)),
	                usb_reg_read (base_addr, DWC_XDCI_EVNTSIZ_REG (i)),
	                usb_reg_read (base_addr, DWC_XDCI_EVNTCOUNT_REG (i))));
	}

//  /* Program Global Control Register to disable scaledown,
//   * disable clock gating
//   */
//  usb_reg_write (
//    base_addr,
//    DWC_XDCI_GCTL_REG,
//    ((usb_reg_read (base_addr, DWC_XDCI_GCTL_REG) & ~DWC_XDCI_GCTL_SCALE_DOWN_MODE_MASK) | DWC_XDCI_GCTL_DISABLE_CLK_GATING_MASK)
//    );
	/* Program Global Control Register to disable scaledown,
	 * disable clock gating
	 */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_GCTL_REG,
	        ((usb_reg_read(base_addr, DWC_XDCI_GCTL_REG) &
//HSLE_DEBUG              ~DWC_XDCI_GCTL_SCALE_DOWN_MODE_MASK) |
//HSLE_DEBUG                    DWC_XDCI_GCTL_DISABLE_CLK_GATING_MASK));
	          ~(DWC_XDCI_GCTL_SCALE_DOWN_MODE_MASK + DWC_XDCI_GCTL_RAMCLKSEL_MASK + DWC_XDCI_GCTL_DISABLE_SCRAMB_MASK)) |
	         DWC_XDCI_GCTL_DISABLE_CLK_GATING_MASK |
	         (DWC_XDCI_GCTL_PRT_CAP_DEVICE << DWC_XDCI_GCTL_PRT_CAP_DIR_BIT_POS)));

//    //
//    //HSLE_DEBUG
//    //
//    usb_reg_write(base_addr, DWC_XDCI_GCTL_REG,
//            ((usb_reg_read(base_addr, DWC_XDCI_GCTL_REG) &
//              ~DWC_XDCI_GCTL_SCALE_DOWN_MODE_MASK) |
//                    DWC_XDCI_GCTL_DISABLE_CLK_GATING_MASK));

	DEBUG ((DEBUG_INFO, "Setup value of xDCI DWC_XDCI_GCTL_REG: 0x%x\n", usb_reg_read (base_addr, DWC_XDCI_GCTL_REG)));


	/* TODO: Program desired speed and set LPM capable
	 * We will do this when Superspeed works. For now,
	 * force into High-speed mode to aVOID anyone trying this
	 * on Super speed port
	 */
#ifdef SUPPORT_SUPER_SPEED
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DCFG_REG,
	        (usb_reg_read (base_addr, DWC_XDCI_DCFG_REG) & ~DWC_XDCI_DCFG_DESIRED_DEV_SPEED_MASK) | local_core_handle->desired_speed
	        );
#else
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DCFG_REG,
	        (usb_reg_read (base_addr, DWC_XDCI_DCFG_REG) & ~DWC_XDCI_DCFG_DESIRED_DEV_SPEED_MASK) | DWC_XDCI_DCFG_DESIRED_HS_SPEED
	        );
#endif

	DEBUG ((DEBUG_INFO, "Setup value of xDCI DWC_XDCI_DCFG_REG: 0x%x\n", usb_reg_read (base_addr, DWC_XDCI_DCFG_REG)));
	DEBUG ((DEBUG_INFO, "Setup value of xDCI DWC_XDCI_DSTS_REG: 0x%x\n", usb_reg_read (base_addr, DWC_XDCI_DSTS_REG)));

	/* Enable Device Interrupt Events */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DEVTEN_REG,
	        DWC_XDCI_DEVTEN_DEVICE_INTS
	        );

	/* Program the desired role */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_GCTL_REG,
	        (usb_reg_read (base_addr, DWC_XDCI_GCTL_REG) & ~DWC_XDCI_GCTL_PRT_CAP_DIR_MASK) | (local_core_handle->role << DWC_XDCI_GCTL_PRT_CAP_DIR_BIT_POS)
	        );

	/* Clear USB2 suspend for start new config command */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_GUSB2PHYCFG_REG (0),
	        (usb_reg_read (base_addr, DWC_XDCI_GUSB2PHYCFG_REG(0)) & ~DWC_XDCI_GUSB2PHYCFG_SUSPEND_PHY_MASK)
	        );

	/* Clear USB3 suspend for start new config command */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_GUSB3PIPECTL_REG (0),
	        (usb_reg_read (base_addr, DWC_XDCI_GUSB3PIPECTL_REG(0)) & ~DWC_XDCI_GUSB3PIPECTL_SUSPEND_PHY_MASK)
	        );

	/* Issue DEPSTARTCFG command for EP0 */
	status = dwc_xdci_core_init_ep_cmd_params (
	        local_core_handle,
	        &local_core_handle->ep_handles[0].ep_info,
	        DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_NONE,
	        EPCMD_START_NEW_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to init params for START_NEW_CONFIG EP command on xDCI\n"));
	        return status;
	}

	/* Issue the command */
	status = dwc_xdci_core_issue_ep_cmd (
	        local_core_handle,
	        0,
	        EPCMD_START_NEW_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to issue START_NEW_CONFIG EP command on xDCI\n"));
	        return status;
	}

	/* Issue DEPCFG command for EP0 */
	status = dwc_xdci_core_init_ep_cmd_params (
	        local_core_handle,
	        &local_core_handle->ep_handles[0].ep_info,
	        DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_INIT_STATE,
	        EPCMD_SET_EP_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to init params for SET_EP_CONFIG command on xDCI for EP0\n"));
	        return status;
	}

	/* Issue the command */
	status = dwc_xdci_core_issue_ep_cmd (
	        local_core_handle,
	        0,
	        EPCMD_SET_EP_CONFIG,
	        &ep_cmd_params);

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to issue SET_EP_CONFIG command on xDCI for EP0\n"));
	        return status;
	}

	/* Issue DEPCFG command for EP1 */
	status = dwc_xdci_core_init_ep_cmd_params (
	        local_core_handle,
	        &local_core_handle->ep_handles[1].ep_info,
	        DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_INIT_STATE,
	        EPCMD_SET_EP_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to init params for SET_EP_CONFIG command on xDCI for EP1\n"));
	        return status;
	}

	/* Issue the command */
	status = dwc_xdci_core_issue_ep_cmd (
	        local_core_handle,
	        1,
	        EPCMD_SET_EP_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to issue SET_EP_CONFIG command on xDCI for EP1\n"));
	        return status;
	}

	/* Issue DEPXFERCFG command for EP0 */
	status = dwc_xdci_core_init_ep_cmd_params (
	        local_core_handle,
	        &local_core_handle->ep_handles[0].ep_info,
	        DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_NONE,
	        EPCMD_SET_EP_XFER_RES_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to init params for EPCMD_SET_EP_XFER_RES_CONFIG command on xDCI for EP0\n"));
	        return status;
	}

	/* Issue the command */
	status = dwc_xdci_core_issue_ep_cmd (
	        local_core_handle,
	        0,
	        EPCMD_SET_EP_XFER_RES_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to issue EPCMD_SET_EP_XFER_RES_CONFIG command on xDCI for EP0\n"));
	        return status;
	}

	/* Issue DEPXFERCFG command for EP1 */
	status = dwc_xdci_core_init_ep_cmd_params (
	        local_core_handle,
	        &local_core_handle->ep_handles[1].ep_info,
	        DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_NONE,
	        EPCMD_SET_EP_XFER_RES_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to init params for EPCMD_SET_EP_XFER_RES_CONFIG command on xDCI for EP1\n"));
	        return status;
	}

	/* Issue the command */
	status = dwc_xdci_core_issue_ep_cmd (
	        local_core_handle,
	        1,
	        EPCMD_SET_EP_XFER_RES_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to issue EPCMD_SET_EP_XFER_RES_CONFIG command on xDCI for EP1\n"));
	        return status;
	}

	/* Prepare a buffer for SETUP packet */
	local_core_handle->trbs = (DWC_XDCI_TRB *)(UINTN)((UINT32)(UINTN)
	                                                  local_core_handle->unaligned_trbs +
	                                                  (DWC_XDCI_TRB_BYTE_ALIGNMENT -
	                                                   ((UINT32)(UINTN)local_core_handle->unaligned_trbs %
	                                                    DWC_XDCI_TRB_BYTE_ALIGNMENT)));

	DEBUG ((DEBUG_INFO, "(dwc_xdci_core_init)@@@@@@@@@ unaligned_trbs address is 0x%x\n", (unsigned) local_core_handle->unaligned_trbs));
	DEBUG ((DEBUG_INFO, "(dwc_xdci_core_init)@@@@@@@@@ TRB address is 0x%x\n", (unsigned) local_core_handle->trbs));

	/* Allocate Setup buffer that is 8-byte aligned */
	local_core_handle->aligned_setup_buffer = local_core_handle->default_setup_buffer +
	        (DWC_XDCI_SETUP_BUFF_SIZE -
	         ((UINT32)(UINTN)(local_core_handle->default_setup_buffer) % DWC_XDCI_SETUP_BUFF_SIZE));

	/* Aligned buffer for status phase */
	local_core_handle->aligned_misc_buffer = local_core_handle->misc_buffer +
	        (DWC_XDCI_SETUP_BUFF_SIZE -
	         ((UINT32)(UINTN)(local_core_handle->aligned_misc_buffer) % DWC_XDCI_SETUP_BUFF_SIZE));

	/* We will queue SETUP request when we see bus reset */

	/* Enable Physical Endpoints 0 */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_EP_DALEPENA_REG,
	        usb_reg_read (base_addr, DWC_XDCI_EP_DALEPENA_REG) | (1 << 0)
	        );

	/* Enable Physical Endpoints 1 */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_EP_DALEPENA_REG,
	        usb_reg_read (base_addr, DWC_XDCI_EP_DALEPENA_REG) | (1 << 1)
	        );

	DEBUG ((DEBUG_INFO, "Default value of xDCI DWC_XDCI_DEVTEN_REG: 0x%x\n", usb_reg_read (base_addr, DWC_XDCI_DEVTEN_REG)));
	return status;
}


/**
   Interface:
   This function is used to de-initialize the xDCI core
   @core_handle: xDCI controller handle
   @flags: Special flags for de-initializing the core in
   particular way

**/
EFI_STATUS
EFIAPI
dwc_xdci_core_deinit (
	IN VOID      *core_handle,
	__attribute__((__unused__)) IN UINT32    flags
	)
{
	/* TODO: Need to implement this */
	free (core_handle);
	return EFI_SUCCESS;
}


/**
   Interface:
   This function is used to register event callback function
   @core_handle: xDCI controller handle
   @event: Event for which callback is to be registered
   @callback_fn: Callback function to invoke after event occurs

**/
EFI_STATUS
EFIAPI
dwc_xdci_core_register_callback (
	IN VOID                      *core_handle,
	IN USB_DEVICE_EVENT_ID       event,
	IN USB_DEVICE_CALLBACK_FUNC  CallbackFunc
	)
{
	XDCI_CORE_HANDLE  *local_core_handle = (XDCI_CORE_HANDLE *)core_handle;

	if (local_core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_register_callback: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	DEBUG ((DEBUG_INFO, "dwc_xdci_core_register_callback: event is %d\n", event));
	switch (event) {
	case USB_DEVICE_DISCONNECT_EVENT:
	        local_core_handle->event_callbacks.dev_disconnect_callback = CallbackFunc;
	        break;

	case USB_DEVICE_RESET_EVENT:
	        local_core_handle->event_callbacks.dev_bus_reset_callback = CallbackFunc;
	        break;

	case USB_DEVICE_CONNECTION_DONE:
	        local_core_handle->event_callbacks.dev_reset_done_callback = CallbackFunc;
	        break;

	case USB_DEVICE_STATE_CHANGE_EVENT:
	        local_core_handle->event_callbacks.dev_link_state_callback = CallbackFunc;
	        break;

	case USB_DEVICE_WAKEUP_EVENT:
	        local_core_handle->event_callbacks.dev_wakeup_callback = CallbackFunc;
	        break;

	case USB_DEVICE_HIBERNATION_REQ_EVENT:
	        local_core_handle->event_callbacks.dev_hibernation_callback = CallbackFunc;
	        break;

	case USB_DEVICE_SOF_EVENT:
	        local_core_handle->event_callbacks.dev_sof_callback = CallbackFunc;
	        break;

	case USB_DEVICE_ERRATIC_ERR_EVENT:
	        local_core_handle->event_callbacks.dev_erratic_err_callback = CallbackFunc;
	        break;

	case USB_DEVICE_CMD_CMPLT_EVENT:
	        local_core_handle->event_callbacks.dev_cmd_cmplt_callback = CallbackFunc;
	        break;

	case USB_DEVICE_BUFF_OVERFLOW_EVENT:
	        local_core_handle->event_callbacks.dev_buff_ovflw_callback = CallbackFunc;
	        break;

	case USB_DEVICE_TEST_LMP_RX_EVENT:
	        local_core_handle->event_callbacks.dev_test_lmp_rx_callback = CallbackFunc;
	        break;

	case USB_DEVICE_SETUP_PKT_RECEIVED:
	        local_core_handle->event_callbacks.dev_setup_pkt_received_callback = CallbackFunc;
	        break;

	case USB_DEVICE_XFER_NRDY:
	        local_core_handle->event_callbacks.dev_xfer_nrdy_callback = CallbackFunc;
	        break;

	case USB_DEVICE_XFER_DONE:
	        local_core_handle->event_callbacks.dev_xfer_done_callback = CallbackFunc;
	        break;

	default:
	        break;
	}

	return EFI_SUCCESS;
}


/**
   Interface:
   This function is used to unregister event callback function
   @core_handle: xDCI controller handle
   @event: Event for which callback function is to be unregistered

**/
EFI_STATUS
EFIAPI
dwc_xdci_core_unregister_callback (
	IN VOID                   *core_handle,
	IN USB_DEVICE_EVENT_ID    event
	)
{
	XDCI_CORE_HANDLE  *local_core_handle = (XDCI_CORE_HANDLE *)core_handle;

	if (local_core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_unregister_callback: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	switch (event) {
	case USB_DEVICE_DISCONNECT_EVENT:
	        local_core_handle->event_callbacks.dev_disconnect_callback = NULL;
	        break;

	case USB_DEVICE_RESET_EVENT:
	        local_core_handle->event_callbacks.dev_bus_reset_callback = NULL;
	        break;

	case USB_DEVICE_CONNECTION_DONE:
	        local_core_handle->event_callbacks.dev_reset_done_callback = NULL;
	        break;

	case USB_DEVICE_STATE_CHANGE_EVENT:
	        local_core_handle->event_callbacks.dev_link_state_callback = NULL;
	        break;

	case USB_DEVICE_WAKEUP_EVENT:
	        local_core_handle->event_callbacks.dev_wakeup_callback = NULL;
	        break;

	case USB_DEVICE_HIBERNATION_REQ_EVENT:
	        local_core_handle->event_callbacks.dev_hibernation_callback = NULL;
	        break;

	case USB_DEVICE_SOF_EVENT:
	        local_core_handle->event_callbacks.dev_sof_callback = NULL;
	        break;

	case USB_DEVICE_ERRATIC_ERR_EVENT:
	        local_core_handle->event_callbacks.dev_erratic_err_callback = NULL;
	        break;

	case USB_DEVICE_CMD_CMPLT_EVENT:
	        local_core_handle->event_callbacks.dev_cmd_cmplt_callback = NULL;
	        break;

	case USB_DEVICE_BUFF_OVERFLOW_EVENT:
	        local_core_handle->event_callbacks.dev_buff_ovflw_callback = NULL;
	        break;

	case USB_DEVICE_TEST_LMP_RX_EVENT:
	        local_core_handle->event_callbacks.dev_test_lmp_rx_callback = NULL;
	        break;

	case USB_DEVICE_SETUP_PKT_RECEIVED:
	        local_core_handle->event_callbacks.dev_setup_pkt_received_callback = NULL;
	        break;

	case USB_DEVICE_XFER_NRDY:
	        local_core_handle->event_callbacks.dev_xfer_nrdy_callback = NULL;
	        break;

	case USB_DEVICE_XFER_DONE:
	        local_core_handle->event_callbacks.dev_xfer_done_callback = NULL;
	        break;

	default:
	        break;
	}

	return EFI_SUCCESS;
}

/*
  inline void
  clflush(volatile void *p)
  {
  asm volatile ("clflush (%0)" :: "r"(p));
  }
*/

/**
   Interface:
   This function is used as an interrupt service routine
   @core_handle: xDCI controller handle

**/
EFI_STATUS
EFIAPI
dwc_xdci_core_isr_routine (
	IN VOID     *core_handle
	)
{
	XDCI_CORE_HANDLE    *local_core_handle = (XDCI_CORE_HANDLE *)core_handle;
	UINT32              base_addr;
	UINT32              event_count;
	UINT32              processed_event_count;
	UINT32              i;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_isr_routine: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	if (local_core_handle->interrup_processing == TRUE) {
	        DEBUG ((DEBUG_INFO, "interrup_processing.........\n"));
	        return EFI_SUCCESS;
	}

	base_addr = local_core_handle->base_address;

	/* Event buffer corresponding to each interrupt line needs
	 * to be processed
	 */
	local_core_handle->interrup_processing = TRUE;
	for (i = 0; i < local_core_handle->max_dev_int_lines; i++) {
	        /* Get the number of events HW has written for this
	         *  interrupt line
	         */
	        event_count = usb_reg_read (base_addr, DWC_XDCI_EVNTCOUNT_REG (i));
	        event_count &= DWC_XDCI_EVNTCOUNT_MASK;
	        processed_event_count = 0;

	        /* Process interrupt line buffer only if count is non-zero */
	        if (event_count) {

	                /* Process events in this buffer */
	                dwc_xdci_process_interrupt_line_events (local_core_handle, event_count, &processed_event_count);

	                /* Write back the processed number of events so HW decrements it from current
	                 * event count
	                 */
	                usb_reg_write (base_addr, DWC_XDCI_EVNTCOUNT_REG (i), processed_event_count);
	        }
	}
	local_core_handle->interrup_processing = FALSE;
	return EFI_SUCCESS;
}


/**
   Interface:
   This function is used as an interrupt service routine and it processes only one event at a time.
   @core_handle: xDCI controller handle

**/
EFI_STATUS
EFIAPI
dwc_xdci_core_isr_routine_timer_based (
	IN VOID     *core_handle
	)
{
	XDCI_CORE_HANDLE    *local_core_handle = (XDCI_CORE_HANDLE *)core_handle;
	UINT32              base_addr;
	UINT32              event_count;
	UINT32              processed_event_count;
	UINT32              current_event_addr;
	UINT32              processed_event_size = 0;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_isr_routine_timer_based: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	if (local_core_handle->current_event_buffer == NULL) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_core_isr_routine_timer_based: INVALID event buffer\n"));
	        return EFI_INVALID_PARAMETER;
	}

	base_addr = local_core_handle->base_address;

	event_count = usb_reg_read (base_addr, DWC_XDCI_EVNTCOUNT_REG (0)) & DWC_XDCI_EVNTCOUNT_MASK;

	if (local_core_handle->interrup_processing == TRUE) {
	        DEBUG ((DEBUG_INFO, "interrup_processing.........\n"));
	        return EFI_SUCCESS;
	}

	local_core_handle->interrup_processing = TRUE;

	processed_event_count = 0;
	current_event_addr = (UINT32)(UINTN)(local_core_handle->current_event_buffer);

	if (local_core_handle->current_event_buffer->event & DWC_XDCI_EVENT_DEV_MASK) {
	        dwc_xdci_process_device_event (
	                local_core_handle,
	                local_core_handle->current_event_buffer,
	                &processed_event_size
	                );
	} else {
	        dwc_xdci_process_ep_event (
	                local_core_handle,
	                local_core_handle->current_event_buffer,
	                &processed_event_size);
	}

	event_count -= processed_event_size;
	processed_event_count += processed_event_size;
	if ((current_event_addr + processed_event_size) >=
	    ((UINT32)(UINTN)(local_core_handle->aligned_event_buffers) + (sizeof(DWC_XDCI_EVENT_BUFFER) * DWC_XDCI_MAX_EVENTS_PER_BUFFER))
	        ) {
	        current_event_addr = (UINT32)(UINTN)(local_core_handle->aligned_event_buffers);
	        DEBUG ((DEBUG_INFO, "dwc_xdci_process_interrupt_line_events: Event buffer bound reached\n"));
	} else {
	        current_event_addr += processed_event_size;
	}

	local_core_handle->current_event_buffer = (DWC_XDCI_EVENT_BUFFER *)(UINTN)current_event_addr;
	usb_reg_write (base_addr, DWC_XDCI_EVNTCOUNT_REG (0), processed_event_count);
	local_core_handle->interrup_processing = FALSE;

	return EFI_SUCCESS;
}


/**
   Interface:
   This function is used to enable xDCI to connect to the host
   @core_handle: xDCI controller handle

**/
EFI_STATUS
EFIAPI
dwc_xdci_core_connect (
	IN VOID     *core_handle
	)
{
	XDCI_CORE_HANDLE    *local_core_handle = (XDCI_CORE_HANDLE *)core_handle;
	UINT32              max_delay_iter = DWC_XDCI_MAX_DELAY_ITERATIONS;
	UINT32              base_addr;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_connect: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	base_addr = local_core_handle->base_address;

	/* Clear KeepConnect bit so we can allow disconnect and re-connect
	 * Also issue No action on state change to aVOID any link change
	 */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DCTL_REG,
	        (usb_reg_read(base_addr, DWC_XDCI_DCTL_REG) & ~DWC_XDCI_DCTL_KEEP_CONNECT_MASK) & ~DWC_XDCI_DCTL_STATE_CHANGE_REQ_MASK
	        );

	/* Set Run bit to connect to the host */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DCTL_REG,
	        usb_reg_read (base_addr, DWC_XDCI_DCTL_REG) | DWC_XDCI_DCTL_RUN_STOP_MASK
	        );

	/* Wait until core starts running */
	do {
	        if (!(usb_reg_read (base_addr, DWC_XDCI_DSTS_REG) & DWC_XDCI_DSTS_DEV_CTRL_HALTED_MASK)) {
	                break;
	        } else {
	                udelay(DWC_XDCI_MAX_DELAY_ITERATIONS);
	        }
	} while (--max_delay_iter);

	if (!max_delay_iter) {
	        DEBUG ((DEBUG_INFO, "Failed to run the device controller\n"));
	        return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}


/**
   Interface:
   This function is used to disconnect xDCI from the host
   @core_handle: xDCI controller handle

**/
EFI_STATUS
EFIAPI
dwc_xdci_core_disconnect (
	IN VOID    *core_handle
	)
{
	XDCI_CORE_HANDLE  *local_core_handle = (XDCI_CORE_HANDLE *)core_handle;
	UINT32            max_delay_iter = DWC_XDCI_MAX_DELAY_ITERATIONS;
	UINT32            base_addr;
	UINT32            event_count;
	UINT32            dsts;
	UINT32            i;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_disconnect: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	base_addr = local_core_handle->base_address;

	event_count = usb_reg_read (base_addr, DWC_XDCI_EVNTCOUNT_REG (0));
	event_count &= DWC_XDCI_EVNTCOUNT_MASK;

	DEBUG ((DEBUG_INFO, "dwc_xdci_core_disconnect: event_count=%d\n", event_count));
	while (event_count) {
	        dwc_xdci_core_isr_routine(local_core_handle);
	        event_count = usb_reg_read (base_addr, DWC_XDCI_EVNTCOUNT_REG (0));
	        event_count &= DWC_XDCI_EVNTCOUNT_MASK;
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_disconnect: event_count=%d\n", event_count));
	}

	/* Issue DEPENDXFER for active transfers */
	for (i = 0; i < DWC_XDCI_MAX_ENDPOINTS; i++){
	        if (local_core_handle->ep_handles[i].currentXferRscIdx){
	                dwc_xdci_end_xfer(local_core_handle, i);
	        }
	}
	/* Clear Run bit to disconnect from host */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DCTL_REG,
	        usb_reg_read(base_addr, DWC_XDCI_DCTL_REG) & ~DWC_XDCI_DCTL_RUN_STOP_MASK);

	/* Wait until core is halted */
	do {
//    if ((usb_reg_read (base_addr, DWC_XDCI_DSTS_REG) & DWC_XDCI_DSTS_LINK_STATE_MASK) == DWC_XDCI_DSTS_LINK_STATE_DISCONNECT)
	        dsts = usb_reg_read (base_addr, DWC_XDCI_DSTS_REG);
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_disconnect: waiting halt: DSTS=0x%x\n", dsts));
	        if ((dsts & DWC_XDCI_DSTS_DEV_CTRL_HALTED_MASK) != 0){
	                break;
	        } else {
	                udelay(DWC_XDCI_MAX_DELAY_ITERATIONS);
	        }
	} while (--max_delay_iter);

	if (!max_delay_iter) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_disconnect: Failed to halt the device controller\n"));
	        return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}


/**
   Interface:
   This function is used to obtain current USB bus speed
   @core_handle: xDCI controller handle
   @speed: Address of variable to save the speed

**/
EFI_STATUS
EFIAPI
dwc_xdci_core_get_speed (
	IN VOID         *core_handle,
	IN USB_SPEED    *speed
	)
{
	XDCI_CORE_HANDLE *local_core_handle = (XDCI_CORE_HANDLE *)core_handle;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_get_speed: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	if (speed == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_get_speed: INVALID parameter\n"));
	        return EFI_INVALID_PARAMETER;
	}

	*speed = usb_reg_read (local_core_handle->base_address, DWC_XDCI_DSTS_REG) & DWC_XDCI_DSTS_CONN_SPEED_MASK;

	return EFI_SUCCESS;
}


/**
   Interface:
   This function is used to obtain current USB bus speed
   @core_handle: xDCI controller handle
   @address: USB address to set (assigned by USB host)

**/
EFI_STATUS
EFIAPI
dwc_xdci_core_set_address (
	IN VOID      *core_handle,
	IN UINT32    address
	)
{
	XDCI_CORE_HANDLE  *local_core_handle = (XDCI_CORE_HANDLE *)core_handle;
	UINT32            base_addr;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_set_address: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	base_addr = local_core_handle->base_address;

	DEBUG ((DEBUG_INFO, "dwc_xdci_core_set_address is 0x%x \n", address));
	/* Program USB device address */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DCFG_REG,
	        (usb_reg_read(base_addr, DWC_XDCI_DCFG_REG) & ~DWC_XDCI_DCFG_DEV_ADDRESS_MASK) | (address << DWC_XDCI_DCFG_DEV_ADDRESS_BIT_POS)
	        );

	local_core_handle->dev_state = UsbDevStateAddress;
	DEBUG ((DEBUG_INFO, "Setup value of xDCI DWC_XDCI_GCTL_REG: 0x%x\n", usb_reg_read (base_addr, DWC_XDCI_GCTL_REG)));
	DEBUG ((DEBUG_INFO, "Setup value of xDCI DWC_XDCI_DEVTEN_REG: 0x%x\n", usb_reg_read (base_addr, DWC_XDCI_DEVTEN_REG)));
	DEBUG ((DEBUG_INFO, "Setup value of xDCI DWC_XDCI_DCFG_REG: 0x%x\n", usb_reg_read (base_addr, DWC_XDCI_DCFG_REG)));
	DEBUG ((DEBUG_INFO, "Setup value of xDCI DWC_XDCI_DSTS_REG: 0x%x\n", usb_reg_read (base_addr, DWC_XDCI_DSTS_REG)));

	return EFI_SUCCESS;
}


/**
   Interface:
   This function is used to set configuration
   @core_handle: xDCI controller handle
   @config_num: config num to set (assigned by USB host)

**/
EFI_STATUS
EFIAPI
dwc_xdci_core_set_config (
	IN VOID      *core_handle,
	__attribute__((__unused__)) IN UINT32    config_num
	)
{
	XDCI_CORE_HANDLE              *local_core_handle = (XDCI_CORE_HANDLE *)core_handle;
	DWC_XDCI_ENDPOINT_CMD_PARAMS  ep_cmd_params;
	EFI_STATUS                    status;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_set_config: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	/* TODO: Disable all non-CTRL endpoints here if
	 * stack is not doing it
	 */
	/* TODO: Cancel all non-CTRL transfers here on all EPs
	 *  if stack is not doing it
	 */
	/* TODO: Change EP1 TXFIFO allocation if necessary */

	/* Re-initialize transfer resource allocation */

	/* Issue DEPSTARTCFG command on EP0 (new config for
	 * non-control EPs)
	 */
	status = dwc_xdci_core_init_ep_cmd_params (
	        local_core_handle,
	        &local_core_handle->ep_handles[0].ep_info,
	        DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_NONE,
	        EPCMD_START_NEW_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_set_config: Failed to init params for EPCMD_START_NEW_CONFIG command\n"));
	        return status;
	}

	/* Issue the command */
	status = dwc_xdci_core_issue_ep_cmd (
	        local_core_handle,
	        0,
	        (EPCMD_START_NEW_CONFIG | (2 << DWC_XDCI_EPCMD_RES_IDX_BIT_POS)),
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_set_config: Failed to issue EPCMD_START_NEW_CONFIG command\n"));
	        return status;
	}

	return status;
}


/**
   Interface:
   This function is used to set link state
   @core_handle: xDCI controller handle
   @state: Desired link state

**/
EFI_STATUS
EFIAPI
dwc_xdci_set_link_state (
	IN VOID                        *core_handle,
	IN USB_DEVICE_SS_LINK_STATE    state
	)
{
	XDCI_CORE_HANDLE  *local_core_handle = (XDCI_CORE_HANDLE *)core_handle;
	UINT32            base_addr;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_set_link_state: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	base_addr = local_core_handle->base_address;

	/* Clear old mask */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DCTL_REG,
	        usb_reg_read (base_addr, DWC_XDCI_DCTL_REG) & ~DWC_XDCI_DCTL_STATE_CHANGE_REQ_MASK
	        );

	/* Request new state */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DCTL_REG,
	        usb_reg_read (base_addr, DWC_XDCI_DCTL_REG) | (state << DWC_XDCI_DCTL_STATE_CHANGE_REQ_BIT_POS)
	        );

	return EFI_SUCCESS;
}


/**
   Interface:
   This function is used to initialize endpoint
   @core_handle: xDCI controller handle
   @ep_info: Address of structure describing properties of EP
   to be initialized

**/
EFI_STATUS
EFIAPI
dwc_xdci_init_ep (
	IN VOID           *core_handle,
	IN USB_EP_INFO    *ep_info
	)
{
	XDCI_CORE_HANDLE              *local_core_handle = (XDCI_CORE_HANDLE *)core_handle;
	DWC_XDCI_ENDPOINT_CMD_PARAMS  ep_cmd_params;
	EFI_STATUS                    status;
	UINT32                        ep_num;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_init_ep: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	/* Convert to physical endpoint */
	ep_num = dwc_xdci_get_physical_ep_num (ep_info->ep_num, ep_info->ep_dir);

	/* Save EP properties */
	memcpy (&(local_core_handle->ep_handles[ep_num].ep_info), ep_info, sizeof (USB_EP_INFO));

	// Init CheckFlag
	local_core_handle->ep_handles[ep_num].CheckFlag = FALSE;

	/* Init DEPCFG cmd params for EP */
	status = dwc_xdci_core_init_ep_cmd_params (
	        core_handle,
	        &local_core_handle->ep_handles[ep_num].ep_info,
	        DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_INIT_STATE,
	        EPCMD_SET_EP_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_init_ep: Failed to init params for  EPCMD_SET_EP_CONFIG command\n"));
	        return status;
	}

	/* Issue the command */
	status = dwc_xdci_core_issue_ep_cmd (
	        core_handle,
	        ep_num,
	        EPCMD_SET_EP_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_init_ep: Failed to issue  EPCMD_SET_EP_CONFIG command\n"));
	        return status;
	}

	/* Issue a DEPXFERCFG command for endpoint */
	status = dwc_xdci_core_init_ep_cmd_params (
	        local_core_handle,
	        &local_core_handle->ep_handles[ep_num].ep_info,
	        DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_NONE,
	        EPCMD_SET_EP_XFER_RES_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_init_ep: Failed to init params for  EPCMD_SET_EP_XFER_RES_CONFIG command\n"));
	        return status;
	}

	/* Issue the command */
	status = dwc_xdci_core_issue_ep_cmd (
	        local_core_handle,
	        ep_num,
	        EPCMD_SET_EP_XFER_RES_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_init_ep: Failed to issue EPCMD_SET_EP_XFER_RES_CONFIG command\n"));
	}

	return status;
}


/**
   Interface:
   This function is used to enable non-Ep0 endpoint
   @core_handle: xDCI controller handle
   @ep_info: Address of structure describing properties of EP
   to be enabled

**/
EFI_STATUS
EFIAPI
dwc_xdci_ep_enable (
	IN VOID           *core_handle,
	IN USB_EP_INFO    *ep_info
	)
{
	XDCI_CORE_HANDLE  *local_core_handle = (XDCI_CORE_HANDLE *)core_handle;
	UINT32            ep_num;
	UINT32            base_addr;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep_enable: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	base_addr = local_core_handle->base_address;

	/* Convert to physical endpoint */
	ep_num = dwc_xdci_get_physical_ep_num (ep_info->ep_num, ep_info->ep_dir);

	/* Enable Physical Endpoint ep_num */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_EP_DALEPENA_REG,
	        usb_reg_read (base_addr, DWC_XDCI_EP_DALEPENA_REG) | (1 << ep_num)
	        );

	return EFI_SUCCESS;
}


/**
   Interface:
   This function is used to disable non-Ep0 endpoint
   @core_handle: xDCI controller handle
   @ep_info: Address of structure describing properties of EP
   to be enabled

**/
EFI_STATUS
EFIAPI
dwc_xdci_ep_disable (
	IN VOID           *core_handle,
	IN USB_EP_INFO    *ep_info
	)
{
	XDCI_CORE_HANDLE  *local_core_handle = (XDCI_CORE_HANDLE *)core_handle;
	UINT32            ep_num;
	UINT32            base_addr;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep_disable: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	base_addr = local_core_handle->base_address;

	/* Convert to physical endpoint */
	ep_num = dwc_xdci_get_physical_ep_num (ep_info->ep_num, ep_info->ep_dir);

	/* Disable Physical Endpoint ep_num */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_EP_DALEPENA_REG,
	        usb_reg_read (base_addr, DWC_XDCI_EP_DALEPENA_REG) & ~(1 << ep_num)
	        );

	return EFI_SUCCESS;
}


/**
   Interface:
   This function is used to STALL and endpoint
   @core_handle: xDCI controller handle
   @ep_info: Address of structure describing properties of EP
   to be enabled

**/
EFI_STATUS
EFIAPI
dwc_xdci_ep_stall (
	IN VOID           *core_handle,
	IN USB_EP_INFO    *ep_info
	)
{
	XDCI_CORE_HANDLE              *local_core_handle = (XDCI_CORE_HANDLE *)core_handle;
	DWC_XDCI_ENDPOINT_CMD_PARAMS  ep_cmd_params;
	EFI_STATUS                    status;
	UINT32                        ep_num;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep_stall: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	/* Convert to physical endpoint */
	ep_num = dwc_xdci_get_physical_ep_num (ep_info->ep_num, ep_info->ep_dir);

	/* Set Ep State Info */
	if (local_core_handle->ep_handles[ep_num].state != USB_EP_STATE_STALLED) {
	        local_core_handle->ep_handles[ep_num].Orgstate = local_core_handle->ep_handles[ep_num].state;
	        local_core_handle->ep_handles[ep_num].state = USB_EP_STATE_STALLED;
	}
	/* Issue a DWC_XDCI_EPCMD_SET_STALL for EP */
	/* Reset params */
	ep_cmd_params.param0 = ep_cmd_params.param1 = ep_cmd_params.param2 = 0;

	/* Issue the command */
	status = dwc_xdci_core_issue_ep_cmd (
	        local_core_handle,
	        ep_num,
	        DWC_XDCI_EPCMD_SET_STALL,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep_stall: Failed to issue EP stall command\n"));
	}

	return status;
}


/**
   Interface:
   This function is used to clear endpoint STALL
   @core_handle: xDCI controller handle
   @ep_info: Address of structure describing properties of EP
   to be enabled

**/
EFI_STATUS
EFIAPI
dwc_xdci_ep_clear_stall (
	IN VOID           *core_handle,
	IN USB_EP_INFO    *ep_info
	)
{
	XDCI_CORE_HANDLE              *local_core_handle = (XDCI_CORE_HANDLE *)core_handle;
	DWC_XDCI_ENDPOINT_CMD_PARAMS  ep_cmd_params;
	EFI_STATUS                    status;
	UINT32                        ep_num;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep_clear_stall: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	/* Convert to physical endpoint */
	ep_num = dwc_xdci_get_physical_ep_num (ep_info->ep_num, ep_info->ep_dir);

	/* Set Ep State Info */
	local_core_handle->ep_handles[ep_num].state = local_core_handle->ep_handles[ep_num].Orgstate;

	/* Issue a DWC_XDCI_EPCMD_CLEAR_STALL for EP */
	/* Reset params */
	ep_cmd_params.param0 = ep_cmd_params.param1 = ep_cmd_params.param2 = 0;

	/* Issue the command */
	status = dwc_xdci_core_issue_ep_cmd (
	        local_core_handle,
	        ep_num,
	        DWC_XDCI_EPCMD_CLEAR_STALL,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep_stall: Failed to issue EP clea stall command\n"));
	}

	return status;
}


/**
   Interface:
   This function is used to set endpoint in NOT READY state
   @core_handle: xDCI controller handle
   @ep_info: Address of structure describing properties of EP
   to be enabled

**/
EFI_STATUS
EFIAPI
dwc_xdci_ep_set_nrdy (
	IN VOID           *core_handle,
	IN USB_EP_INFO    *ep_info
	)
{
	XDCI_CORE_HANDLE  *local_core_handle = (XDCI_CORE_HANDLE *)core_handle;
	UINT32            ep_num;
	UINT32            base_addr;
	UINT32            max_delay_iter = DWC_XDCI_MAX_DELAY_ITERATIONS;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep_set_nrdy: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	base_addr = local_core_handle->base_address;

	/* Convert to physical endpoint */
	ep_num = dwc_xdci_get_physical_ep_num (ep_info->ep_num, ep_info->ep_dir);

	/* Program the EP number in command's param reg */
	usb_reg_write (base_addr, DWC_XDCI_DGCMD_PARAM_REG, ep_num);

	/* Issue EP not ready generic device command */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DGCMD_REG,
	        (usb_reg_read (base_addr, DWC_XDCI_DGCMD_REG) | DWC_XDCI_DGCMD_CMD_SET_EP_NRDY)
	        );

	/* Activate the command */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DGCMD_REG,
	        (usb_reg_read (base_addr, DWC_XDCI_DGCMD_REG) | DWC_XDCI_DGCMD_CMD_ACTIVE_MASK)
	        );

	/* Wait until command completes */
	do {
	        if (!(usb_reg_read (base_addr, DWC_XDCI_DGCMD_REG) & DWC_XDCI_DGCMD_CMD_ACTIVE_MASK))
	                break;
	        else
	                udelay(DWC_XDCI_MAX_DELAY_ITERATIONS);
	} while (--max_delay_iter);

	if (!max_delay_iter) {
	        DEBUG ((DEBUG_INFO, "Failed to issue Command\n"));
	        return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}


/**
   Interface:
   This function is used to queue receive SETUP packet request
   @core_handle: xDCI controller handle
   @buffer: Address of buffer to receive SETUP packet

**/
EFI_STATUS
EFIAPI
dwc_xdci_ep0_receive_setup_pkt (
	IN VOID     *core_handle,
	IN UINT8    *buffer
	)
{
	XDCI_CORE_HANDLE                *local_core_handle = (XDCI_CORE_HANDLE *)core_handle;
	DWC_XDCI_ENDPOINT_CMD_PARAMS    ep_cmd_params;
	EFI_STATUS                      Status = EFI_DEVICE_ERROR;
	DWC_XDCI_TRB                    *trb;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep0_receive_setup_pkt: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	local_core_handle->ep_handles[0].ep_info.ep_num = 0;
	local_core_handle->ep_handles[0].ep_info.ep_dir = 0;
	local_core_handle->ep_handles[0].state = USB_EP_STATE_SETUP;
	trb = local_core_handle->trbs;
	DEBUG ((DEBUG_INFO, "(dwc_xdci_ep0_receive_setup_pkt)\n"));

	Status = dwc_xdci_core_init_trb (
	        local_core_handle,
	        trb,
	        TRBCTL_SETUP,
	        buffer,
	        8
	        );

	if (Status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep0_receive_setup_pkt: Init TRB Failed \n"));
	        return Status;
	}

	/* Issue a DEPSTRTXFER for EP0 */
	/* Reset params */
	ep_cmd_params.param0 = ep_cmd_params.param1 = ep_cmd_params.param2 = 0;

	/* Init the lower re-bits for TRB address */
	ep_cmd_params.param1 = (UINT32)(UINTN)trb;

	/* Issue the command */
	Status = dwc_xdci_core_issue_ep_cmd (
	        local_core_handle,
	        0,
	        EPCMD_START_XFER,
	        &ep_cmd_params
	        );

	if (Status) {
	        DEBUG ((DEBUG_INFO, "\ndwc_xdci_ep0_receive_setup_pkt: Failed to issue Start Transfer command"));
	}

	/* Save new resource index for this transfer */
	local_core_handle->ep_handles[0].currentXferRscIdx = ((usb_reg_read(local_core_handle->base_address, DWC_XDCI_EPCMD_REG(0)) &
	                                                       DWC_XDCI_EPCMD_RES_IDX_MASK) >> DWC_XDCI_EPCMD_RES_IDX_BIT_POS
	        );

	return Status;
}


/**
   Interface:
   This function is used to queue receive status packet on EP0
   @core_handle: xDCI controller handle

**/
EFI_STATUS
EFIAPI
dwc_xdci_ep0_receive_status_pkt (
	IN VOID    *core_handle
	)
{
	XDCI_CORE_HANDLE                *local_core_handle = (XDCI_CORE_HANDLE *)core_handle;
	DWC_XDCI_TRB                    *trb;
	DWC_XDCI_TRB_CONTROL            trb_ctrl;
	DWC_XDCI_ENDPOINT_CMD_PARAMS    ep_cmd_params;
	EFI_STATUS                      Status;
	UINT32                          base_addr;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep0_receive_status_pkt: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	base_addr = local_core_handle->base_address;

	/* We are receiving on EP0 so physical EP is 0 */
	trb = local_core_handle->trbs;
	DEBUG ((DEBUG_INFO, "(dwc_xdci_ep0_receive_status_pkt)\n"));
	if (trb->trb_ctrl & DWC_XDCI_TRB_CTRL_HWO_MASK) {
	        DEBUG ((DEBUG_INFO, "status_pkt still not transferred.\n"));
	        return EFI_SUCCESS;
	}

	local_core_handle->ep_handles[0].ep_info.ep_num = 0;
	local_core_handle->ep_handles[0].ep_info.ep_dir = 0;

	/* OUT data phase for 3-phased control transfer */
	trb_ctrl = TRBCTL_3_PHASE;

	/* Init TRB for the transfer */
	Status = dwc_xdci_core_init_trb (
	        local_core_handle,
	        trb,
	        trb_ctrl,
	        local_core_handle->aligned_setup_buffer,
	        0
	        );

	if (!Status) {
	        /* Issue a DEPSTRTXFER for EP0 */
	        /* Reset params */
	        ep_cmd_params.param0 = ep_cmd_params.param1 = ep_cmd_params.param2 = 0;

	        /* Init the lower bits for TRB address */
	        ep_cmd_params.param1 = (UINT32)(UINTN)trb;

	        /* Issue the command */
	        Status = dwc_xdci_core_issue_ep_cmd (
	                local_core_handle,
	                0,
	                EPCMD_START_XFER,
	                &ep_cmd_params
	                );

	        if (Status) {
	                DEBUG ((DEBUG_INFO, "dwc_xdci_ep0_receive_status_pkt: Failed to issue Start Transfer command for EP0\n"));
	        }

	        /* Save new resource index for this transfer */
	        local_core_handle->ep_handles[0].currentXferRscIdx = ((usb_reg_read(base_addr, DWC_XDCI_EPCMD_REG(0)) & DWC_XDCI_EPCMD_RES_IDX_MASK) >> DWC_XDCI_EPCMD_RES_IDX_BIT_POS);

	        /* TODO: We are not using the EP state for control transfers
	         * right now simply because we're only supporting IN
	         * data phase. For the current use case, we don't
	         * need OUT data phase. We can add that later and we will
	         * add some of the state and SETUP packet awareness code
	         */
	        local_core_handle->ep_handles[0].state = USB_EP_STATE_STATUS;
	}

	return Status;
}


/**
   Interface:
   This function is used to send status packet on EP0
   @core_handle: xDCI controller handle

**/
EFI_STATUS
EFIAPI
dwc_xdci_ep0_send_status_pkt (
	IN VOID    *core_handle
	)
{
	XDCI_CORE_HANDLE                *local_core_handle = (XDCI_CORE_HANDLE *)core_handle;
	DWC_XDCI_TRB                    *trb;
	DWC_XDCI_ENDPOINT_CMD_PARAMS    ep_cmd_params;
	EFI_STATUS                      Status;
	UINT32                          base_addr;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep0_send_status_pkt: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	base_addr = local_core_handle->base_address;

	/* We are sending on EP0 so physical EP is 1 */
	trb = (local_core_handle->trbs + (1 * DWC_XDCI_TRB_NUM));
	DEBUG ((DEBUG_INFO, "(dwc_xdci_ep0_send_status_pkt)\n"));

	local_core_handle->ep_handles[0].state = USB_EP_STATE_STATUS;
	Status = dwc_xdci_core_init_trb (
	        local_core_handle,
	        trb,
	        TRBCTL_2_PHASE,
	        local_core_handle->aligned_misc_buffer,
	        0
	        );

	if (Status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep0_send_status_pkt: TRB failed during status phase\n"));
	        return Status;
	}

	/* Issue a DEPSTRTXFER for EP1 */
	/* Reset params */
	ep_cmd_params.param0 = ep_cmd_params.param1 = ep_cmd_params.param2 = 0;

	/* Init the lower re-bits for TRB address */
	ep_cmd_params.param1 = (UINT32)(UINTN)trb;

	/* Issue the command */
	Status = dwc_xdci_core_issue_ep_cmd (
	        local_core_handle,
	        1,
	        EPCMD_START_XFER,
	        &ep_cmd_params
	        );

	if (Status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep0_send_status_pkt: Failed to issue Start Transfer on EP0\n"));
	}

	/* Save new resource index for this transfer */
	local_core_handle->ep_handles[1].currentXferRscIdx = ((usb_reg_read(base_addr, DWC_XDCI_EPCMD_REG(1)) & DWC_XDCI_EPCMD_RES_IDX_MASK) >> DWC_XDCI_EPCMD_RES_IDX_BIT_POS);
	local_core_handle->ep_handles[0].state = USB_EP_STATE_STATUS;

	return Status;
}


/**
   Interface:
   This function is used to send data on non-EP0 endpoint
   @core_handle: xDCI controller handle
   @ep_info: Address of structure describing properties of EP
   @buffer: buffer containing data to transmit
   @size: Size of transfer (in bytes)

**/
EFI_STATUS
EFIAPI
dwc_xdci_ep_tx_data (
	IN VOID                *core_handle,
	IN USB_XFER_REQUEST    *xfer_req
	)
{
	XDCI_CORE_HANDLE              *local_core_handle = (XDCI_CORE_HANDLE *)core_handle;
	DWC_XDCI_ENDPOINT_CMD_PARAMS  ep_cmd_params;
	DWC_XDCI_TRB                  *trb;
	DWC_XDCI_TRB_CONTROL          trb_ctrl;
	EFI_STATUS                    Status;
	UINT32                        ep_num;
	UINT32                        base_addr;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep_tx_data: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	if (xfer_req == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep_tx_data: INVALID transfer request\n"));
	        return EFI_INVALID_PARAMETER;
	}

	base_addr = local_core_handle->base_address;

	/* Convert to physical endpoint */
	ep_num = dwc_xdci_get_physical_ep_num (
	        xfer_req->ep_info.ep_num,
	        xfer_req->ep_info.ep_dir
	        );

	trb = (local_core_handle->trbs + (ep_num * DWC_XDCI_TRB_NUM));
	DEBUG ((DEBUG_INFO, "(dwc_xdci_ep_tx_data)ep_num is %d\n", ep_num));


	if (ep_num > 1)
	        trb_ctrl = TRBCTL_NORMAL;
	else
	        trb_ctrl = TRBCTL_CTRL_DATA_PHASE;

	if (trb->trb_ctrl & DWC_XDCI_TRB_CTRL_HWO_MASK) {
	        Status = dwc_xdci_end_xfer (local_core_handle, ep_num);
	        if (Status) {
	                DEBUG ((DEBUG_INFO, "dwc_xdci_ep_tx_data: Failed to end previous transfer\n"));
	        }

	        Status = dwc_xdci_core_flush_ep_tx_fifo (local_core_handle, ep_num);
	        if (Status) {
	                DEBUG ((DEBUG_INFO, "dwc_xdci_ep_tx_data: Failed to end previous transfer\n"));
	        }
	}

	/* Data phase */
//  local_core_handle->ep_handles[ep_num].xfer_handle = *xfer_req;
	memcpy (&(local_core_handle->ep_handles[ep_num].xfer_handle), xfer_req, sizeof (USB_XFER_REQUEST));
	local_core_handle->ep_handles[ep_num].state = USB_EP_STATE_DATA;

	local_core_handle->ep_handles[ep_num].trb = trb;

	Status = dwc_xdci_core_init_trb (
	        local_core_handle,
	        trb,
	        trb_ctrl,
	        xfer_req->xfer_buffer,
	        xfer_req->xfer_len
	        );

	if (Status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep_tx_data: TRB failed\n"));
	        return Status;
	}

	/* Issue a DEPSTRTXFER for EP */
	/* Reset params */
	ep_cmd_params.param0 = ep_cmd_params.param1 = ep_cmd_params.param2 = 0;

	/* Init the lower re-bits for TRB address */
	ep_cmd_params.param1 = (UINT32)(UINTN)trb;

	/* Issue the command */
	Status = dwc_xdci_core_issue_ep_cmd (
	        local_core_handle,
	        ep_num,
	        EPCMD_START_XFER,
	        &ep_cmd_params
	        );

	/* Save new resource index for this transfer */
	local_core_handle->ep_handles[ep_num].currentXferRscIdx = ((usb_reg_read (base_addr, DWC_XDCI_EPCMD_REG(ep_num)) & DWC_XDCI_EPCMD_RES_IDX_MASK) >> DWC_XDCI_EPCMD_RES_IDX_BIT_POS);

	return Status;
}


/**
   Interface:
   This function is used to receive data on non-EP0 endpoint
   @core_handle: xDCI controller handle
   @ep_info: Address of structure describing properties of EP
   @buffer: buffer containing data to transmit
   @size: Size of transfer (in bytes)

**/
EFI_STATUS
EFIAPI
dwc_xdci_ep_rx_data (
	IN VOID                *core_handle,
	IN USB_XFER_REQUEST    *xfer_req
	)
{
	XDCI_CORE_HANDLE              *local_core_handle = (XDCI_CORE_HANDLE *)core_handle;
	DWC_XDCI_ENDPOINT_CMD_PARAMS  ep_cmd_params;
	DWC_XDCI_TRB                  *trb;
	DWC_XDCI_TRB_CONTROL          trb_ctrl;
	EFI_STATUS                    Status;
	UINT32                        ep_num;
	UINT32                        base_addr;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep_rx_data: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	if (xfer_req == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep_rx_data: INVALID transfer request\n"));
	        return EFI_INVALID_PARAMETER;
	}

	base_addr = local_core_handle->base_address;

	/* Convert to physical endpoint */
	ep_num = dwc_xdci_get_physical_ep_num (xfer_req->ep_info.ep_num, xfer_req->ep_info.ep_dir);

	trb = (local_core_handle->trbs + (ep_num * DWC_XDCI_TRB_NUM));
	DEBUG ((DEBUG_INFO, "(dwc_xdci_ep_rx_data)ep_num is %d\n", ep_num));

	if (ep_num > 1)
	        trb_ctrl = TRBCTL_NORMAL;
	else
	        trb_ctrl = TRBCTL_CTRL_DATA_PHASE;

	//
	// If CheckFlag didn't set to FALSE, means the previous transfer request didn't complete,
	// need to wait the previous request done.
	//
	if (local_core_handle->ep_handles[ep_num].CheckFlag == TRUE) {
	        return EFI_NOT_READY;
	}

	local_core_handle->ep_handles[ep_num].CheckFlag = TRUE;

	/* Data phase */
	memcpy (&(local_core_handle->ep_handles[ep_num].xfer_handle), xfer_req, sizeof (USB_XFER_REQUEST));

	local_core_handle->ep_handles[ep_num].state = USB_EP_STATE_DATA;

	local_core_handle->ep_handles[ep_num].trb = trb;

	DEBUG ((DEBUG_INFO, "(dwc_xdci_ep_rx_data)xfer_req->xfer_len is 0x%x\n", xfer_req->xfer_len));

	Status = dwc_xdci_core_init_trb (
	        local_core_handle,
	        trb,
	        trb_ctrl,
	        xfer_req->xfer_buffer,
	        xfer_req->xfer_len
	        );

	if (Status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep_rx_data: TRB failed\n"));
	        return Status;
	}

	/* Issue a DEPSTRTXFER for EP */
	/* Reset params */
	ep_cmd_params.param0 = ep_cmd_params.param1 = ep_cmd_params.param2 = 0;

	/* Init the lower re-bits for TRB address */
	ep_cmd_params.param1 = (UINT32)(UINTN)trb;

	/* Issue the command */
	Status = dwc_xdci_core_issue_ep_cmd (
	        local_core_handle,
	        ep_num,
	        EPCMD_START_XFER,
	        &ep_cmd_params
	        );

	if (Status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep_rx_data: Failed to start transfer\n"));
	}

	/* Save new resource index for this transfer */
	local_core_handle->ep_handles[ep_num].currentXferRscIdx = ((usb_reg_read(base_addr, DWC_XDCI_EPCMD_REG(ep_num)) & DWC_XDCI_EPCMD_RES_IDX_MASK) >> DWC_XDCI_EPCMD_RES_IDX_BIT_POS);

	return Status;
}



STATIC
EFI_STATUS
dwc_xdci_core_flush_ep_fifo (
	IN XDCI_CORE_HANDLE    *core_handle,
	IN UINT32              ep_num
	)
{
	UINT32 base_addr;
	UINT32 max_delay_iter = DWC_XDCI_MAX_DELAY_ITERATIONS;
	UINT32 fifo_num;
	UINT32 Param;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "ERROR: dwc_xdci_core_flush_ep_tx_fifo: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	base_addr = core_handle->base_address;

	/* Translate to FIFOnum
	 * NOTE: Assuming this is a Tx EP
	 */
	fifo_num = (ep_num >> 1);

	/* TODO: Currently we are only using TxFIFO 0. Later map these
	 * Write the FIFO num/dir param for the generic command.
	 */

	Param = usb_reg_read (base_addr, DWC_XDCI_DGCMD_PARAM_REG);
	Param &= ~(DWC_XDCI_DGCMD_PARAM_TX_FIFO_NUM_MASK | DWC_XDCI_DGCMD_PARAM_TX_FIFO_DIR_MASK);

	if ((ep_num & 0x01) != 0) {
	        Param |= (fifo_num | DWC_XDCI_DGCMD_PARAM_TX_FIFO_DIR_MASK);
	} else {
	        Param |= fifo_num;
	}

	DEBUG ((DEBUG_INFO, "USB FU Flash: CMD 0x%08x :: Param 0x%08x\n",
	        (usb_reg_read(base_addr, DWC_XDCI_DGCMD_REG) | DWC_XDCI_DGCMD_CMD_SEL_FIFO_FLUSH | DWC_XDCI_DGCMD_CMD_ACTIVE_MASK),
	        Param));

	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DGCMD_PARAM_REG,
	        Param
	        );

	/* Write the command to flush all FIFOs */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DGCMD_REG,
	        (usb_reg_read(base_addr, DWC_XDCI_DGCMD_REG) | DWC_XDCI_DGCMD_CMD_SEL_FIFO_FLUSH | DWC_XDCI_DGCMD_CMD_ACTIVE_MASK)
	        );


	/* Wait until command completes */
	do {
	        if (!(usb_reg_read(base_addr, DWC_XDCI_DGCMD_REG) & DWC_XDCI_DGCMD_CMD_ACTIVE_MASK))
	                break;
	        else
	                udelay(DWC_XDCI_MAX_DELAY_ITERATIONS);
	} while (--max_delay_iter);

	if (!max_delay_iter) {
	        DEBUG ((DEBUG_INFO, "Failed to issue Command\n"));
	        return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}

/**
   Interface:
   This function is used to cancel a transfer on non-EP0 endpoint
   @core_handle: xDCI controller handle
   @ep_info: Address of structure describing properties of EP

**/
EFI_STATUS
EFIAPI
dwc_xdci_ep_cancel_transfer (
	IN VOID           *core_handle,
	IN USB_EP_INFO    *ep_info
	)
{
	EFI_STATUS  Status = EFI_DEVICE_ERROR;
	UINT32      ep_num;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep_cancel_transfer: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	/* Get physical EP num */
	ep_num = dwc_xdci_get_physical_ep_num (ep_info->ep_num, ep_info->ep_dir);
	Status = dwc_xdci_end_xfer(core_handle, ep_num);
	dwc_xdci_core_flush_ep_fifo(core_handle, ep_num);

	return Status;
}


EFI_STATUS
usb_process_device_reset_det (
	IN XDCI_CORE_HANDLE    *core_handle
	)
{
	return dwc_xdci_process_device_reset_det (core_handle);
}

EFI_STATUS
usb_process_device_reset_done (
	IN XDCI_CORE_HANDLE    *core_handle
	)
{
	return dwc_xdci_process_device_reset_done (core_handle);
}

UINT32
usb_get_physical_ep_num (
	IN UINT32        EndpointNum,
	IN USB_EP_DIR    EndpointDir
	)
{
	return dwc_xdci_get_physical_ep_num(
	        EndpointNum,
	        EndpointDir
	        );
}


EFI_STATUS
EFIAPI
usb_xdci_core_reinit (
//   IN USB_DEV_CONFIG_PARAMS    *ConfigParams,
//   IN VOID                     *device_core_ptr,
	IN VOID                     *core_handle
	)
{
	EFI_STATUS                      status = EFI_DEVICE_ERROR;
	UINT32                          base_addr;
	XDCI_CORE_HANDLE                *local_core_handle;
	DWC_XDCI_ENDPOINT_CMD_PARAMS    ep_cmd_params;
	UINT32                          max_delay_iter = DWC_XDCI_MAX_DELAY_ITERATIONS;
	UINT8                           i;

//   local_core_handle = (XDCI_CORE_HANDLE *)AllocateZeroPool (sizeof(XDCI_CORE_HANDLE));

	local_core_handle = core_handle;

	if (core_handle == NULL) {
	        return EFI_INVALID_PARAMETER;
	}

	if (local_core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to allocate handle for xDCI\n"));
	        return EFI_OUT_OF_RESOURCES;
	}

//   ZeroMem (local_core_handle, sizeof(XDCI_CORE_HANDLE));
//
//   local_core_handle->parent_handle = device_core_ptr;

//   *core_handle = (VOID *)local_core_handle;
//
//   local_core_handle->id = ConfigParams->ControllerId;
//   local_core_handle->base_address = base_addr = ConfigParams->BaseAddress;
//   local_core_handle->flags = ConfigParams->Flags;
//   local_core_handle->desired_speed = local_core_handle->actual_speed = ConfigParams->Speed;
//   local_core_handle->role = ConfigParams->Role;
	base_addr = local_core_handle->base_address;

	DEBUG ((DEBUG_INFO, "Resetting the USB core\n"));
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DCTL_REG,
	        usb_reg_read (base_addr, DWC_XDCI_DCTL_REG) | DWC_XDCI_DCTL_CSFTRST_MASK
	        );

	/* Wait until core soft reset completes */
	do {
	        if (!(usb_reg_read (base_addr, DWC_XDCI_DCTL_REG) & DWC_XDCI_DCTL_CSFTRST_MASK)) {
	                break;
	        } else {
	                udelay(DWC_XDCI_MAX_DELAY_ITERATIONS);
	        }
	} while (--max_delay_iter);

	if (!max_delay_iter) {
	        DEBUG ((DEBUG_INFO, "Failed to reset device controller\n"));
	        return EFI_DEVICE_ERROR;
	}

	DEBUG ((DEBUG_INFO, "USB core has been reset\n"));

	/* All FIFOs are flushed at this point */

//   /* Ensure we have EP0 Rx/Tx handles initialized */
//   local_core_handle->ep_handles[0].ep_info.ep_num = 0;
//   local_core_handle->ep_handles[0].ep_info.ep_dir = UsbEpDirOut;
//   local_core_handle->ep_handles[0].ep_info.ep_type = USB_ENDPOINT_CONTROL;
//   local_core_handle->ep_handles[0].ep_info.max_pkt_size = DWC_XDCI_SS_CTRL_EP_MPS;
//   /* 0 means burst size of 1 */
//   local_core_handle->ep_handles[0].ep_info.burst_size = 0;
//
//   local_core_handle->ep_handles[1].ep_info.ep_num = 0;
//   local_core_handle->ep_handles[1].ep_info.ep_dir = UsbEpDirIn;
//   local_core_handle->ep_handles[1].ep_info.ep_type = USB_ENDPOINT_CONTROL;
//   local_core_handle->ep_handles[1].ep_info.max_pkt_size = DWC_XDCI_SS_CTRL_EP_MPS;
//   /* 0 means burst size of 1 */
//   local_core_handle->ep_handles[1].ep_info.burst_size = 0;

	local_core_handle->dev_state = UsbDevStateDefault;

	/* Clear KeepConnect bit so we can allow disconnect and
	 * re-connect. Stay in RX_DETECT state
	 */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DCTL_REG,
	        (usb_reg_read (base_addr, DWC_XDCI_DCTL_REG) &
	         (~DWC_XDCI_DCTL_KEEP_CONNECT_MASK) &
	         (~DWC_XDCI_DCTL_STATE_CHANGE_REQ_MASK)) |
	        (DWC_XDCI_DCTL_STATE_CHANGE_REQ_RX_DETECT << DWC_XDCI_DCTL_STATE_CHANGE_REQ_BIT_POS)
	        );

	DEBUG ((DEBUG_INFO, "Device controller Synopsys ID: %x\n", usb_reg_read (base_addr, DWC_XDCI_GSNPSID_REG)));
	DEBUG ((DEBUG_INFO, "Default value of xDCI GSBUSCFG0 and GSBUSCFG1: %x, %x\n",
	        usb_reg_read (base_addr, DWC_XDCI_GSBUSCFG0_REG),
	        usb_reg_read (base_addr, DWC_XDCI_GSBUSCFG1_REG)));

	DEBUG ((DEBUG_INFO, "Default value of xDCI GTXTHRCFG and GRXTHRCFG: %x, %x\n",
	        usb_reg_read (base_addr, DWC_XDCI_GTXTHRCFG_REG),
	        usb_reg_read (base_addr, DWC_XDCI_GRXTHRCFG_REG)));

	/* Clear ULPI auto-resume bit */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_GUSB2PHYCFG_REG (0),
	        (usb_reg_read (base_addr, DWC_XDCI_GUSB2PHYCFG_REG (0)) & ~DWC_XDCI_GUSB2PHYCFG_ULPI_AUTO_RESUME_MASK)
	        );

	DEBUG ((DEBUG_INFO, "Default value of xDCI GUSB2PHYCFG and GUSB3PIPECTL: %x, %x\n",
	        usb_reg_read (base_addr, DWC_XDCI_GUSB2PHYCFG_REG (0)),
	        usb_reg_read (base_addr, DWC_XDCI_GUSB3PIPECTL_REG (0))));

	/* Only one RxFIFO */
	DEBUG ((DEBUG_INFO, "Default value of DWC_XDCI_GRXFIFOSIZ: %x\n",
	        usb_reg_read (base_addr, DWC_XDCI_GRXFIFOSIZ_REG (0))));

	for (i = 0; i < DWC_XDCI_MAX_ENDPOINTS; i++) {
	        DEBUG ((DEBUG_INFO, "Default value of xDCI DWC_XDCI_GTXFIFOSIZ %d: %x\n",
	                i, usb_reg_read (base_addr, DWC_XDCI_GTXFIFOSIZ_REG (i))));
	}

	/* TODO: Need to check if TxFIFO should start where RxFIFO ends
	 * or default is correct i.e. TxFIFO starts at 0 just like RxFIFO
	 */

	/* Allocate and Initialize Event Buffers */
	local_core_handle->max_dev_int_lines = ((usb_reg_read (base_addr, DWC_XDCI_GHWPARAMS1_REG) &
	                                         DWC_XDCI_GHWPARAMS1_NUM_INT_MASK) >>
	                                        DWC_XDCI_GHWPARAMS1_NUM_INT_BIT_POS);

	DEBUG ((DEBUG_INFO, "Max dev int lines: %d\n", local_core_handle->max_dev_int_lines));

	/* One event buffer per interrupt line.
	 *  Need to align it to size of event buffer
	 *  Buffer needs to be big enough. Otherwise the core
	 *  won't operate
	 */
	local_core_handle->aligned_event_buffers = (DWC_XDCI_EVENT_BUFFER *)
	        ((UINT32)(UINTN)(local_core_handle->event_buffers) +
	         ((sizeof (DWC_XDCI_EVENT_BUFFER) * DWC_XDCI_MAX_EVENTS_PER_BUFFER) -
	          (((UINT32)(UINTN)(local_core_handle->event_buffers)) %
	           (sizeof (DWC_XDCI_EVENT_BUFFER) * DWC_XDCI_MAX_EVENTS_PER_BUFFER))));

	for (i = 0; i < local_core_handle->max_dev_int_lines; i++) {
	        usb_reg_write (
	                base_addr,
	                DWC_XDCI_GEVNTADR_REG (i),
	                (UINT32)(UINTN)(local_core_handle->aligned_event_buffers + i * sizeof(DWC_XDCI_EVENT_BUFFER) * DWC_XDCI_MAX_EVENTS_PER_BUFFER)
	                );

	        //
	        // Clear High 32bit address register, GEVNTADR register is 64-bit register
	        // default is 0xffffffffffffffff
	        //
	        usb_reg_write (base_addr, DWC_XDCI_GEVNTADR_REG (i) + 4, 0x00000000);

	        local_core_handle->current_event_buffer = local_core_handle->aligned_event_buffers;
	        /* Write size and clear the mask */
	        usb_reg_write (
	                base_addr,
	                DWC_XDCI_EVNTSIZ_REG (i),
	                sizeof (DWC_XDCI_EVENT_BUFFER) * DWC_XDCI_MAX_EVENTS_PER_BUFFER
	                );

	        /* Write 0 to the event count register as the last step
	         *  for event configuration
	         */
	        usb_reg_write (base_addr, DWC_XDCI_EVNTCOUNT_REG (i), 0);

	        DEBUG ((DEBUG_INFO, "Value of xDCI Event buffer %d: %x, Size: %x, Count: %x\n",
	                i,
	                usb_reg_read (base_addr, DWC_XDCI_GEVNTADR_REG (i)),
	                usb_reg_read (base_addr, DWC_XDCI_EVNTSIZ_REG (i)),
	                usb_reg_read (base_addr, DWC_XDCI_EVNTCOUNT_REG (i))));
	}

//  /* Program Global Control Register to disable scaledown,
//   * disable clock gating
//   */
//  usb_reg_write (
//    base_addr,
//    DWC_XDCI_GCTL_REG,
//    ((usb_reg_read (base_addr, DWC_XDCI_GCTL_REG) & ~DWC_XDCI_GCTL_SCALE_DOWN_MODE_MASK) | DWC_XDCI_GCTL_DISABLE_CLK_GATING_MASK)
//    );
	/* Program Global Control Register to disable scaledown,
	 * disable clock gating
	 */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_GCTL_REG,
	        ((usb_reg_read(base_addr, DWC_XDCI_GCTL_REG) &
//HSLE_DEBUG              ~DWC_XDCI_GCTL_SCALE_DOWN_MODE_MASK) |
//HSLE_DEBUG                    DWC_XDCI_GCTL_DISABLE_CLK_GATING_MASK));
	          ~(DWC_XDCI_GCTL_SCALE_DOWN_MODE_MASK + DWC_XDCI_GCTL_RAMCLKSEL_MASK + DWC_XDCI_GCTL_DISABLE_SCRAMB_MASK)) |
	         DWC_XDCI_GCTL_DISABLE_CLK_GATING_MASK |
	         (DWC_XDCI_GCTL_PRT_CAP_DEVICE << DWC_XDCI_GCTL_PRT_CAP_DIR_BIT_POS)));

//    //
//    //HSLE_DEBUG
//    //
//    usb_reg_write(base_addr, DWC_XDCI_GCTL_REG,
//            ((usb_reg_read(base_addr, DWC_XDCI_GCTL_REG) &
//              ~DWC_XDCI_GCTL_SCALE_DOWN_MODE_MASK) |
//                    DWC_XDCI_GCTL_DISABLE_CLK_GATING_MASK));

	DEBUG ((DEBUG_INFO, "Setup value of xDCI DWC_XDCI_GCTL_REG: 0x%x\n", usb_reg_read (base_addr, DWC_XDCI_GCTL_REG)));


	/* TODO: Program desired speed and set LPM capable
	 * We will do this when Superspeed works. For now,
	 * force into High-speed mode to aVOID anyone trying this
	 * on Super speed port
	 */
#ifdef SUPPORT_SUPER_SPEED
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DCFG_REG,
	        (usb_reg_read (base_addr, DWC_XDCI_DCFG_REG) & ~DWC_XDCI_DCFG_DESIRED_DEV_SPEED_MASK) | local_core_handle->desired_speed
	        );
#else
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DCFG_REG,
	        (usb_reg_read (base_addr, DWC_XDCI_DCFG_REG) & ~DWC_XDCI_DCFG_DESIRED_DEV_SPEED_MASK) | DWC_XDCI_DCFG_DESIRED_HS_SPEED
	        );
#endif

	DEBUG ((DEBUG_INFO, "Setup value of xDCI DWC_XDCI_DCFG_REG: 0x%x\n", usb_reg_read (base_addr, DWC_XDCI_DCFG_REG)));
	DEBUG ((DEBUG_INFO, "Setup value of xDCI DWC_XDCI_DSTS_REG: 0x%x\n", usb_reg_read (base_addr, DWC_XDCI_DSTS_REG)));

	/* Enable Device Interrupt Events */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_DEVTEN_REG,
	        DWC_XDCI_DEVTEN_DEVICE_INTS
	        );

	/* Program the desired role */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_GCTL_REG,
	        (usb_reg_read (base_addr, DWC_XDCI_GCTL_REG) & ~DWC_XDCI_GCTL_PRT_CAP_DIR_MASK) | (local_core_handle->role << DWC_XDCI_GCTL_PRT_CAP_DIR_BIT_POS)
	        );

	/* Clear USB2 suspend for start new config command */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_GUSB2PHYCFG_REG (0),
	        (usb_reg_read (base_addr, DWC_XDCI_GUSB2PHYCFG_REG(0)) & ~DWC_XDCI_GUSB2PHYCFG_SUSPEND_PHY_MASK)
	        );

	/* Clear USB3 suspend for start new config command */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_GUSB3PIPECTL_REG (0),
	        (usb_reg_read (base_addr, DWC_XDCI_GUSB3PIPECTL_REG(0)) & ~DWC_XDCI_GUSB3PIPECTL_SUSPEND_PHY_MASK)
	        );

	/* Issue DEPSTARTCFG command for EP0 */
	status = dwc_xdci_core_init_ep_cmd_params (
	        local_core_handle,
	        &local_core_handle->ep_handles[0].ep_info,
	        DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_NONE,
	        EPCMD_START_NEW_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to init params for START_NEW_CONFIG EP command on xDCI\n"));
	        return status;
	}

	/* Issue the command */
	status = dwc_xdci_core_issue_ep_cmd (
	        local_core_handle,
	        0,
	        EPCMD_START_NEW_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to issue START_NEW_CONFIG EP command on xDCI\n"));
	        return status;
	}

	/* Issue DEPCFG command for EP0 */
	status = dwc_xdci_core_init_ep_cmd_params (
	        local_core_handle,
	        &local_core_handle->ep_handles[0].ep_info,
	        DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_INIT_STATE,
	        EPCMD_SET_EP_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to init params for SET_EP_CONFIG command on xDCI for EP0\n"));
	        return status;
	}

	/* Issue the command */
	status = dwc_xdci_core_issue_ep_cmd (
	        local_core_handle,
	        0,
	        EPCMD_SET_EP_CONFIG,
	        &ep_cmd_params);

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to issue SET_EP_CONFIG command on xDCI for EP0\n"));
	        return status;
	}

	/* Issue DEPCFG command for EP1 */
	status = dwc_xdci_core_init_ep_cmd_params (
	        local_core_handle,
	        &local_core_handle->ep_handles[1].ep_info,
	        DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_INIT_STATE,
	        EPCMD_SET_EP_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to init params for SET_EP_CONFIG command on xDCI for EP1\n"));
	        return status;
	}

	/* Issue the command */
	status = dwc_xdci_core_issue_ep_cmd (
	        local_core_handle,
	        1,
	        EPCMD_SET_EP_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to issue SET_EP_CONFIG command on xDCI for EP1\n"));
	        return status;
	}

	/* Issue DEPXFERCFG command for EP0 */
	status = dwc_xdci_core_init_ep_cmd_params (
	        local_core_handle,
	        &local_core_handle->ep_handles[0].ep_info,
	        DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_NONE,
	        EPCMD_SET_EP_XFER_RES_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to init params for EPCMD_SET_EP_XFER_RES_CONFIG command on xDCI for EP0\n"));
	        return status;
	}

	/* Issue the command */
	status = dwc_xdci_core_issue_ep_cmd (
	        local_core_handle,
	        0,
	        EPCMD_SET_EP_XFER_RES_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to issue EPCMD_SET_EP_XFER_RES_CONFIG command on xDCI for EP0\n"));
	        return status;
	}

	/* Issue DEPXFERCFG command for EP1 */
	status = dwc_xdci_core_init_ep_cmd_params (
	        local_core_handle,
	        &local_core_handle->ep_handles[1].ep_info,
	        DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_NONE,
	        EPCMD_SET_EP_XFER_RES_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to init params for EPCMD_SET_EP_XFER_RES_CONFIG command on xDCI for EP1\n"));
	        return status;
	}

	/* Issue the command */
	status = dwc_xdci_core_issue_ep_cmd (
	        local_core_handle,
	        1,
	        EPCMD_SET_EP_XFER_RES_CONFIG,
	        &ep_cmd_params
	        );

	if (status) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_core_init: Failed to issue EPCMD_SET_EP_XFER_RES_CONFIG command on xDCI for EP1\n"));
	        return status;
	}

	/* Prepare a buffer for SETUP packet */
	local_core_handle->trbs = (DWC_XDCI_TRB *)(UINTN)((UINT32)(UINTN)
	                                                  local_core_handle->unaligned_trbs +
	                                                  (DWC_XDCI_TRB_BYTE_ALIGNMENT -
	                                                   ((UINT32)(UINTN)local_core_handle->unaligned_trbs %
	                                                    DWC_XDCI_TRB_BYTE_ALIGNMENT)));

	DEBUG ((DEBUG_INFO, "(dwc_xdci_core_init)@@@@@@@@@ unaligned_trbs address is 0x%x\n", (unsigned) local_core_handle->unaligned_trbs));
	DEBUG ((DEBUG_INFO, "(dwc_xdci_core_init)@@@@@@@@@ TRB address is 0x%x\n", (unsigned) local_core_handle->trbs));

	/* Allocate Setup buffer that is 8-byte aligned */
	local_core_handle->aligned_setup_buffer = local_core_handle->default_setup_buffer +
	        (DWC_XDCI_SETUP_BUFF_SIZE -
	         ((UINT32)(UINTN)(local_core_handle->default_setup_buffer) % DWC_XDCI_SETUP_BUFF_SIZE));

	/* Aligned buffer for status phase */
	local_core_handle->aligned_misc_buffer = local_core_handle->misc_buffer +
	        (DWC_XDCI_SETUP_BUFF_SIZE -
	         ((UINT32)(UINTN)(local_core_handle->aligned_misc_buffer) % DWC_XDCI_SETUP_BUFF_SIZE));

	/* We will queue SETUP request when we see bus reset */

	/* Enable Physical Endpoints 0 */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_EP_DALEPENA_REG,
	        usb_reg_read (base_addr, DWC_XDCI_EP_DALEPENA_REG) | (1 << 0)
	        );

	/* Enable Physical Endpoints 1 */
	usb_reg_write (
	        base_addr,
	        DWC_XDCI_EP_DALEPENA_REG,
	        usb_reg_read (base_addr, DWC_XDCI_EP_DALEPENA_REG) | (1 << 1)
	        );

	DEBUG ((DEBUG_INFO, "Default value of xDCI DWC_XDCI_DEVTEN_REG: 0x%x\n", usb_reg_read (base_addr, DWC_XDCI_DEVTEN_REG)));
	return status;


}


EFI_STATUS
usb_xdci_core_flush_ep_fifo (
	IN VOID           *core_handle,
	IN USB_EP_INFO    *ep_info
	)
{
	EFI_STATUS  Status = EFI_DEVICE_ERROR;
	UINT32      ep_num;

	if (core_handle == NULL) {
	        DEBUG ((DEBUG_INFO, "dwc_xdci_ep_cancel_transfer: INVALID handle\n"));
	        return EFI_DEVICE_ERROR;
	}

	/* Get physical EP num */
	ep_num = dwc_xdci_get_physical_ep_num (ep_info->ep_num, ep_info->ep_dir);
	dwc_xdci_core_flush_ep_fifo(core_handle, ep_num);

	return Status;
}
