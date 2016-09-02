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

#ifndef _XDCI_DWC_H_
#define _XDCI_DWC_H_

#include "XdciCommon.h"
#include "XdciDevice.h"

#define DWC_XDCI_MAX_ENDPOINTS                             (16)
#define DWC_XDCI_SS_CTRL_EP_MPS                            (512)
#define DWC_XDCI_HS_CTRL_EP_MPS                            (64)
#define DWC_XDCI_FS_CTRL_EP_MPS                            (64)
#define DWC_XDCI_LS_CTRL_EP_MPS                            (8)
#define DWC_XDCI_SS_CTRL_BUF_SIZE                          (512)
#define DWC_XDCI_SETUP_BUFF_SIZE                           (8)
#define DWC_XDCI_MAX_EVENTS_PER_BUFFER                     (16)
#define DWC_XDCI_TRB_BYTE_ALIGNMENT                        (16)
#define DWC_XDCI_DEFAULT_TX_FIFO_SIZE                      (1024)
#define DWC_XDCI_TRB_NUM                                   (32)
#define DWC_XDCI_MASK                                      (DWC_XDCI_TRB_NUM - 1)

/* TODO: Platform-specific define. Later move it to platform-specific
 *  header file
 *   */
#define DWC_XDCI_MAX_DELAY_ITERATIONS                      (1000)

/* Top-level register offsets from base address */
#define DWC_XDCI_GLOBAL_REG_OFFSET                         (0xC100)
#define DWC_XDCI_DEVICE_REG_OFFSET                         (0xC700)
#define DWC_XDCI_OTG_BC_REG_OFFSET                         (0xCC00)

#define DWC_XDCI_GSBUSCFG0_REG                             (0xC100)
#define DWC_XDCI_GSBUSCFG1_REG                             (0xC104)
#define DWC_XDCI_GTXTHRCFG_REG                             (0xC108)
#define DWC_XDCI_GRXTHRCFG_REG                             (0xC10C)

/* Global Control Register and bit definitions */
#define DWC_XDCI_GCTL_REG                                  (0xC110)
#define DWC_XDCI_GCTL_PWRDNSCALE_MASK                      (0xFFF80000)
#define DWC_XDCI_GCTL_PWRDNSCALE_VAL                       (0x13880000)
#define DWC_XDCI_GCTL_U2RSTECN_MASK                        (0x00010000)
#define DWC_XDCI_GCTL_PRT_CAP_DIR_MASK                     (0x00003000)
#define DWC_XDCI_GCTL_PRT_CAP_DIR_BIT_POS                  (12)
#define DWC_XDCI_GCTL_PRT_CAP_HOST                         (1)
#define DWC_XDCI_GCTL_PRT_CAP_DEVICE                       (2)
#define DWC_XDCI_GCTL_PRT_CAP_OTG                          (3)
#define DWC_XDCI_GCTL_RAMCLKSEL_MASK                       (0x000000C0)
#define DWC_XDCI_GCTL_SCALE_DOWN_MODE_MASK                 (0x00000030)
#define DWC_XDCI_GCTL_DISABLE_CLK_GATING_MASK              (0x00000001)
#define DWC_XDCI_GCTL_DISABLE_SCRAMB_MASK                  (0x00000008)

#define DWC_XDCI_GSTS_REG                                  (0xC118)
#define DWC_XDCI_GSNPSID_REG                               (0xC120)
#define DWC_XDCI_GGPIO_REG                                 (0xC124)
#define DWC_XDCI_GUID_REG                                  (0xC128)
#define DWC_XDCI_GUCTL_REG                                 (0xC12C)
#define DWC_XDCI_GBUSERRADDR                               (0xC130)

/* Global Hardware Parameters Registers */
#define DWC_XDCI_GHWPARAMS0_REG                            (0xC140)
#define DWC_XDCI_GHWPARAMS1_REG                            (0xC144)
#define DWC_XDCI_GHWPARAMS1_NUM_INT_MASK                   (0x1F8000)
#define DWC_XDCI_GHWPARAMS1_NUM_INT_BIT_POS                (15)

#define DWC_XDCI_GHWPARAMS2_REG                            (0xC148)
#define DWC_XDCI_GHWPARAMS3_REG                            (0xC14C)
#define DWC_XDCI_GHWPARAMS4_REG                            (0xC150)
#define DWC_XDCI_GHWPARAMS4_CACHE_TRBS_PER_XFER_MASK       (0x0000003F)
#define DWC_XDCI_GHWPARAMS5_REG                            (0xC154)
#define DWC_XDCI_GHWPARAMS6_REG                            (0xC158)
#define DWC_XDCI_GHWPARAMS7_REG                            (0xC15C)
#define DWC_XDCI_GHWPARAMS8_REG                            (0xC600)

#define DWC_XDCI_GDBGFIFOSPACE_REG                         (0xC160)

#define DWC_XDCI_GUSB2PHYCFG_REG(n)                        (0xC200 + (n << 2))
#define DWC_XDCI_GUSB2PHYCFG_ULPI_AUTO_RESUME_MASK         (0x00008000)
#define DWC_XDCI_GUSB2PHYCFG_SUSPEND_PHY_MASK              (0x00000040)

#define DWC_XDCI_GUSB3PIPECTL_REG(n)                       (0xC2C0 + (n << 2))
#define DWC_XDCI_GUSB3PIPECTL_SUSPEND_PHY_MASK             (0x00020000)

#define DWC_XDCI_GTXFIFOSIZ_REG(n)                         (0xC300 + (n << 2))
#define DWC_XDCI_GTXFIFOSIZ_START_ADDRESS_MASK             (0xFFFF0000)
#define DWC_XDCI_GTXFIFOSIZ_START_ADDRESS_BIT_POS          (16)
#define DWC_XDCI_GRXFIFOSIZ_REG(n)                         (0xC380 + (n << 2))

/* Global Event Buffer Registers */
#define DWC_XDCI_GEVNTADR_REG(n)                           (0xC400 + (n << 4))
#define DWC_XDCI_EVNTSIZ_REG(n)                            (0xC408 + (n << 4))
#define DWC_XDCI_EVNTSIZ_MASK                              (0x0000FFFF)
#define DWC_XDCI_EVNT_INTR_MASK                            (0x80000000)
#define DWC_XDCI_EVNTCOUNT_REG(n)                          (0xC40C + (n << 4))
#define DWC_XDCI_EVNTCOUNT_MASK                            (0x0000FFFF)

/* Device Configuration Register and Bit Definitions */
#define DWC_XDCI_DCFG_REG                                  (0xC700)
#define DWC_XDCI_DCFG_LPM_CAPABLE_MASK                     (0x00400000)
#define DWC_XDCI_DCFG_DEV_ADDRESS_MASK                     (0x000003F8)
#define DWC_XDCI_DCFG_DEV_ADDRESS_BIT_POS                  (3)
#define DWC_XDCI_DCFG_DESIRED_DEV_SPEED_MASK               (0x00000007)
#define DWC_XDCI_DCFG_DESIRED_SS_SPEED                     (0x00000004)
#define DWC_XDCI_DCFG_DESIRED_FS_SPEED                     (0x00000001)
#define DWC_XDCI_DCFG_DESIRED_HS_SPEED                     (0x00000000)

/* Device Control Register */
#define DWC_XDCI_DCTL_REG                                  (0xC704)
#define DWC_XDCI_DCTL_RUN_STOP_MASK                        (0x80000000)
#define DWC_XDCI_DCTL_RUN_STOP_BIT_POS                     (31)
#define DWC_XDCI_DCTL_CSFTRST_MASK                         (0x40000000)
#define DWC_XDCI_DCTL_CSFTRST_BIT_POS                      (30)
#define DWC_XDCI_DCTL_KEEP_CONNECT_MASK                    (0x00080000)
#define DWC_XDCI_DCTL_KEEP_CONNECT_BIT_POS                 (19)
#define DWC_XDCI_DCTL_CSFTRST_BIT_POS                      (30)
#define DWC_XDCI_DCTL_STATE_CHANGE_REQ_MASK                (0x000001E0)
#define DWC_XDCI_DCTL_STATE_CHANGE_REQ_BIT_POS             (5)
#define DWC_XDCI_DCTL_STATE_CHANGE_REQ_NO_ACTION           (1)
#define DWC_XDCI_DCTL_STATE_CHANGE_REQ_SS_DISABLED         (4)
#define DWC_XDCI_DCTL_STATE_CHANGE_REQ_RX_DETECT           (5)
#define DWC_XDCI_DCTL_STATE_CHANGE_REQ_SS_INACTIVE         (6)
#define DWC_XDCI_DCTL_STATE_CHANGE_REQ_RECOVERY            (8)
#define DWC_XDCI_DCTL_STATE_CHANGE_REQ_COMPLIANCE          (10)
#define DWC_XDCI_DCTL_STATE_CHANGE_REQ_REMOTE_WAKEUP       (8)

/* Device Event Enable Register */
#define DWC_XDCI_DEVTEN_REG                                (0xC708)
#define DWC_XDCI_DEVTEN_DISCONN_DET_EN_MASK                (0x00000001)
#define DWC_XDCI_DEVTEN_RESET_DET_EN_MASK                  (0x00000002)
#define DWC_XDCI_DEVTEN_CONN_DONE_DET_EN_MASK              (0x00000004)
#define DWC_XDCI_DEVTEN_LINK_STATE_CHANGE_DET_EN_MASK      (0x00000008)
#define DWC_XDCI_DEVTEN_RESUME_WAKEUP_DET_EN_MASK          (0x00000010)
#define DWC_XDCI_DEVTEN_HIBERNATION_REQ_EN_MASK            (0x00000020)
#define DWC_XDCI_DEVTEN_U3L2L1_DET_EN_MASK                 (0x00000040)
#define DWC_XDCI_DEVTEN_SOF_DET_EN_MASK                    (0x00000080)
#define DWC_XDCI_DEVTEN_ERRATIC_ERR_DET_EN_MASK            (0x00000200)
#define DWC_XDCI_DEVTEN_VNDR_DEV_TST_RX_DET_EN_MASK        (0x00001000)

#define DWC_XDCI_DEVTEN_DEVICE_INTS                        (DWC_XDCI_DEVTEN_DISCONN_DET_EN_MASK | \
                                                            DWC_XDCI_DEVTEN_RESET_DET_EN_MASK | DWC_XDCI_DEVTEN_CONN_DONE_DET_EN_MASK | \
                                                            DWC_XDCI_DEVTEN_LINK_STATE_CHANGE_DET_EN_MASK | DWC_XDCI_DEVTEN_RESUME_WAKEUP_DET_EN_MASK | \
                                                            DWC_XDCI_DEVTEN_HIBERNATION_REQ_EN_MASK | DWC_XDCI_DEVTEN_U3L2L1_DET_EN_MASK | \
                                                            DWC_XDCI_DEVTEN_ERRATIC_ERR_DET_EN_MASK)

#define DWC_XDCI_EVENT_BUFF_BULK_STREAM_ID_MASK            (0xFFFF0000)
#define DWC_XDCI_EVENT_BUFF_ISOCH_UFRAME_NUM_MASK          (0xFFFF0000)
#define DWC_XDCI_EVENT_BUFF_EP_CMD_TYPE_MASK               (0x0F000000)
#define DWC_XDCI_EVENT_BUFF_EP_XFER_RES_INDEX_MASK         (0x007F0000)
#define DWC_XDCI_EVENT_BUFF_EP_XFER_ACTIVE_MASK            (0x00008000)
#define DWC_XDCI_EVENT_BUFF_EP_CTRL_DATA_REQ_MASK          (0x00001000)
#define DWC_XDCI_EVENT_BUFF_EP_CTRL_STATUS_REQ_MASK        (0x00002000)
#define DWC_XDCI_EVENT_BUFF_EP_LST_MASK                    (0x00008000)
#define DWC_XDCI_EVENT_BUFF_EP_MISSED_ISOCH_MASK           (0x00008000)
#define DWC_XDCI_EVENT_BUFF_EP_IOC_MASK                    (0x00004000)
#define DWC_XDCI_EVENT_BUFF_EP_LAST_PKT_MASK               (0x00002000)
#define DWC_XDCI_EVENT_BUFF_EP_STREAM_NOT_FND_MASK         (0x00002000)
#define DWC_XDCI_EVENT_BUFF_EP_STREAM_FND_MASK             (0x00001000)
#define DWC_XDCI_EVENT_BUFF_EP_ERR_NO_RES_MASK             (0x00001000)
#define DWC_XDCI_EVENT_BUFF_EP_INVALID_RES_MASK            (0x00001000)

#define DWC_XDCI_EVENT_BUFF_EP_EVENT_MASK                  (0x000003C0)
#define DWC_XDCI_EVENT_BUFF_EP_EVENT_BIT_POS               (6)
#define DWC_XDCI_EVENT_BUFF_EP_XFER_CMPLT                  (1)
#define DWC_XDCI_EVENT_BUFF_EP_XFER_IN_PROGRESS            (2)
#define DWC_XDCI_EVENT_BUFF_EP_XFER_NOT_READY              (3)
#define DWC_XDCI_EVENT_BUFF_EP_STREAM_EVENT                (6)
#define DWC_XDCI_EVENT_BUFF_EP_CMD_CMPLT                   (7)

#define DWC_XDCI_EVENT_BUFF_EP_NUM_MASK                    (0x0000003E)
#define DWC_XDCI_EVENT_BUFF_EP_NUM_BIT_POS                 (1)

#define DWC_XDCI_EVENT_BUFF_EP_EVENT_STATUS_MASK           (0x0000F000)


#define DWC_XDCI_EVENT_BUFF_DEV_HIRD_MASK                  (0x01E00000)
#define DWC_XDCI_EVENT_BUFF_DEV_HIRD_BIT_POS               (21)
#define DWC_XDCI_EVENT_BUFF_DEV_SS_EVENT_MASK              (0x00100000)
#define DWC_XDCI_EVENT_BUFF_DEV_LINK_STATE_MASK            (0x000F0000)
#define DWC_XDCI_EVENT_BUFF_DEV_LINK_STATE_BIT_POS         (16)

#define DWC_XDCI_EVENT_BUFF_DEV_EVT_MASK                   (0x00000F00)
#define DWC_XDCI_EVENT_BUFF_DEV_EVT_BIT_POS                (8)
#define DWC_XDCI_EVENT_BUFF_DEV_TST_LMP_RX_EVENT           (12)
#define DWC_XDCI_EVENT_BUFF_DEV_BUFF_OVFL_EVENT            (11)
#define DWC_XDCI_EVENT_BUFF_DEV_CMD_CMPLT_EVENT            (10)
#define DWC_XDCI_EVENT_BUFF_DEV_ERRATIC_ERR_EVENT          (9)
#define DWC_XDCI_EVENT_BUFF_DEV_SOF_EVENT                  (7)
#define DWC_XDCI_EVENT_BUFF_DEV_HBRNTN_REQ_EVENT           (5)
#define DWC_XDCI_EVENT_BUFF_DEV_WKUP_EVENT                 (4)
#define DWC_XDCI_EVENT_BUFF_DEV_STATE_CHANGE_EVENT         (3)
#define DWC_XDCI_EVENT_BUFF_DEV_CONN_DONE_EVENT            (2)
#define DWC_XDCI_EVENT_BUFF_DEV_USB_RESET_EVENT            (1)
#define DWC_XDCI_EVENT_BUFF_DEV_DISCONN_EVENT              (0)

#define DWC_XDCI_EVENT_DEV_MASK                            (0x00000001)

/* Device Status Register and Bit Definitions */
#define DWC_XDCI_DSTS_REG                                  (0xC70C)
#define DWC_XDCI_DSTS_DEV_CTRL_HALTED_MASK                 (0x00400000)
#define DWC_XDCI_DSTS_DEV_CTRL_HALTED_BIT_POS              (22)
#define DWC_XDCI_DSTS_CORE_IDLE                            (1 << 23)
#define DWC_XDCI_DSTS_CONN_SPEED_MASK                      (0x00000007)
#define DWC_XDCI_DSTS_LINK_STATE_MASK                      (0x003C0000)
#define DWC_XDCI_DSTS_LINK_STATE_DISCONNECT                (0x00100000)

/* Device Generic Command Parameter Register */
#define DWC_XDCI_DGCMD_PARAM_REG                           (0xC710)
#define DWC_XDCI_DGCMD_PARAM_TX_FIFO_NUM_MASK              (0x0000001F)
#define DWC_XDCI_DGCMD_PARAM_TX_FIFO_DIR_MASK              (0x00000020)
#define DWC_XDCI_DGCMD_PARAM_TX_FIFO_DIR_BIT_POS           (5)

/* Device Generic Command Register */
#define DWC_XDCI_DGCMD_REG                                 (0xC714)
#define DWC_XDCI_DGCMD_CMD_STATUS_MASK                     (0x00008000)
#define DWC_XDCI_DGCMD_CMD_ACTIVE_MASK                     (0x00000400)
#define DWC_XDCI_DGCMD_CMD_IOC_MASK                        (0x00000100)
#define DWC_XDCI_DGCMD_CMD_TYPE_MASK                       (0x000000FF)
#define DWC_XDCI_DGCMD_CMD_SET_PERIODIC_PARAMS             (0x2)
#define DWC_XDCI_DGCMD_CMD_SET_SCRATCH_PAD_BUFF_ARR_LO     (0x4)
#define DWC_XDCI_DGCMD_CMD_SET_SCRATCH_PAD_BUFF_ARR_HI     (0x5)
#define DWC_XDCI_DGCMD_CMD_XMIT_DEVICE_NOTIFICATION        (0x7)
#define DWC_XDCI_DGCMD_CMD_SEL_FIFO_FLUSH                  (0x9)
#define DWC_XDCI_DGCMD_CMD_ALL_FIFO_FLUSH                  (0xA)
#define DWC_XDCI_DGCMD_CMD_SET_EP_NRDY                     (0xC)
#define DWC_XDCI_DGCMD_CMD_RUN_SOC_BUS_LPBK                (0x10)

/* Device Active USB EP Enable Register */
#define DWC_XDCI_EP_DALEPENA_REG                           (0xC720)

/* Device Physical EP CMD Param 2 Register. Value is 32-bit */
#define DWC_XDCI_EPCMD_PARAM2_REG(n)                       (0xC800 + (n << 4))

/* Device Physical EP CMD Param 1 Register. Value is 32-bit */
#define DWC_XDCI_EPCMD_PARAM1_REG(n)                       (0xC804 + (n << 4))

/* Device Physical EP CMD Param 0 Register. Value is 32-bit */
#define DWC_XDCI_EPCMD_PARAM0_REG(n)                       (0xC808 + (n << 4))

/* Device Physical EP Command Registers and Bit Definitions */
#define DWC_XDCI_EPCMD_REG(n)                              (0xC80C + (n << 4))
#define DWC_XDCI_EPCMD_RES_IDX_MASK                        (0x007F0000)
#define DWC_XDCI_EPCMD_RES_IDX_BIT_POS                     (16)
#define DWC_XDCI_EPCMD_CMDTYPE_MASK                        (0x0000000F)
#define DWC_XDCI_EPCMD_SET_EP_CONFIG                       (0x1)
#define DWC_XDCI_EPCMD_SET_EP_XFER_RES_CONFIG              (0x2)
#define DWC_XDCI_EPCMD_GET_EP_STATE                        (0x3)
#define DWC_XDCI_EPCMD_SET_STALL                           (0x4)
#define DWC_XDCI_EPCMD_CLEAR_STALL                         (0x5)
#define DWC_XDCI_EPCMD_START_XFER                          (0x6)
#define DWC_XDCI_EPCMD_UPDATE_XFER                         (0x7)
#define DWC_XDCI_EPCMD_END_XFER                            (0x8)
#define DWC_XDCI_EPCMD_START_NEW_CONFIG                    (0x9)

#define DWC_XDCI_EPCMD_CMD_IOC_MASK                        (0x00000100)
#define DWC_XDCI_EPCMD_CMD_ACTIVE_MASK                     (0x00000400)
#define DWC_XDCI_EPCMD_HIGH_PRIO_MASK                      (0x00000800)
#define DWC_XDCI_EPCMD_FORCE_RM_MASK                       (0x00000800)

/* Command status and parameter values same as event status and parameters values */
#define DWC_XDCI_EPCMD_CMD_STATUS_MASK                     (0x0000F000)

/* Command Params bit masks */
#define DWC_XDCI_PARAM1_SET_EP_CFG_FIFO_BASED_MASK         (0x80000000)
#define DWC_XDCI_PARAM1_SET_EP_CFG_BULK_BASED_MASK         (0x40000000)
#define DWC_XDCI_PARAM1_SET_EP_CFG_EP_NUM_MASK             (0x3C000000)
#define DWC_XDCI_PARAM1_SET_EP_CFG_EP_DIR_MASK             (0x02000000)
#define DWC_XDCI_PARAM1_SET_EP_CFG_STRM_CAP_MASK           (0x01000000)
#define DWC_XDCI_PARAM1_SET_EP_CFG_BINTM1_MASK             (0x00FF0000)
#define DWC_XDCI_PARAM1_SET_EP_CFG_BINTM1_BIT_POS          (16)
#define DWC_XDCI_PARAM1_SET_EP_CFG_EBC_MASK                (0x00008000)
#define DWC_XDCI_PARAM1_SET_EP_CFG_EVT_EN_MASK             (0x00003F00)
#define DWC_XDCI_PARAM1_SET_EP_CFG_EVT_EN_BIT_POS          (8)
#define DWC_XDCI_PARAM1_SET_EP_CFG_EVT_STRM_MASK           (0x00002000)
#define DWC_XDCI_PARAM1_SET_EP_CFG_EVT_XFER_NRDY_MASK      (0x00000400)
#define DWC_XDCI_PARAM1_SET_EP_CFG_EVT_XFER_IN_PRG_MASK    (0x00000200)
#define DWC_XDCI_PARAM1_SET_EP_CFG_EVT_XFER_CMPLT_MASK     (0x00000100)
#define DWC_XDCI_PARAM1_SET_EP_CFG_INTR_NUM_MASK           (0x0000001F)

/* CMD 1 param 0 */
#define DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_MASK               (0xC0000000)
#define DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_BIT_POS            (30)
#define DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_INIT_STATE         (0)
#define DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_RESTORE_ST         (1)
#define DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_MDFY_STATE         (2)
#define DWC_XDCI_PARAM0_SET_EP_CFG_ACTN_NONE               (3)
#define DWC_XDCI_PARAM0_SET_EP_CFG_BRST_SIZE_MASK          (0x03C00000)
#define DWC_XDCI_PARAM0_SET_EP_CFG_BRST_SIZE_BIT_POS       (22)
#define DWC_XDCI_PARAM0_SET_EP_CFG_FIFO_NUM_MASK           (0x003E0000)
#define DWC_XDCI_PARAM0_SET_EP_CFG_FIFO_NUM_BIT_POS        (17)
#define DWC_XDCI_PARAM0_SET_EP_CFG_MPS_MASK                (0x00003FF8)
#define DWC_XDCI_PARAM0_SET_EP_CFG_MPS_BIT_POS             (3)
#define DWC_XDCI_PARAM0_SET_EP_CFG_EP_TYPE_MASK            (0x00000006)
#define DWC_XDCI_PARAM0_SET_EP_CFG_EP_TYPE_BIT_POS         (1)
#define DWC_XDCI_PARAM0_EP_TYPE_CTRL                       (0)
#define DWC_XDCI_PARAM0_EP_TYPE_ISOCH                      (1)
#define DWC_XDCI_PARAM0_EP_TYPE_BULK                       (2)
#define DWC_XDCI_PARAM0_EP_TYPE_INTR                       (3)

/* CMD 1 param 1 */
#define DWC_XDCI_PARAM1_SET_EP_CFG_BULK_BASED_MASK         (0x40000000)
#define DWC_XDCI_PARAM1_SET_EP_CFG_EP_NUM_MASK             (0x3C000000)
#define DWC_XDCI_PARAM1_SET_EP_CFG_EP_NUM_BIT_POS          (26)
#define DWC_XDCI_PARAM1_SET_EP_CFG_EP_DIR_MASK             (0x02000000)
#define DWC_XDCI_PARAM1_SET_EP_CFG_EP_DIR_BIT_POS          (25)
#define DWC_XDCI_PARAM1_SET_EP_CFG_STRM_CAP_MASK           (0x01000000)
#define DWC_XDCI_PARAM1_SET_EP_CFG_BINTM1_MASK             (0x00FF0000)
#define DWC_XDCI_PARAM1_SET_EP_CFG_BINTM1_BIT_POS          (16)
#define DWC_XDCI_PARAM1_SET_EP_CFG_EBC_MASK                (0x00008000)
#define DWC_XDCI_PARAM1_SET_EP_CFG_EVT_EN_MASK             (0x00003F00)
#define DWC_XDCI_PARAM1_SET_EP_CFG_EVT_EN_BIT_POS          (8)
#define DWC_XDCI_PARAM1_SET_EP_CFG_EVT_STRM_MASK           (0x00002000)
#define DWC_XDCI_PARAM1_SET_EP_CFG_EVT_XFER_NRDY_MASK      (0x00000400)
#define DWC_XDCI_PARAM1_SET_EP_CFG_EVT_XFER_IN_PRG_MASK    (0x00000200)
#define DWC_XDCI_PARAM1_SET_EP_CFG_EVT_XFER_CMPLT_MASK     (0x00000100)
#define DWC_XDCI_PARAM1_SET_EP_CFG_INTR_NUM_MASK           (0x0000001F)

/* CMD 2 param 0 */
#define DWC_XDCI_PARAM0_SET_EP_XFER_RES_NUM_MASK           (0x0000FFFF)

/* CMD 3 param 2 */
#define DWC_XDCI_PARAM2_GET_EP_STATE_MASK                  (0xFFFFFFFF)

/* CMD 6 param 1 */
#define DWC_XDCI_PARAM1_STRT_XFER_TD_ADDR_LO_MASK          (0xFFFFFFFF)

/* CMD 6 param 0 */
#define DWC_XDCI_PARAM0_STRT_XFER_TD_ADDR_HI_MASK          (0xFFFFFFFF)

/* Transfer Request Block Fields' Bit Definitions */
#define DWC_XDCI_TRB_BUFF_SIZE_MASK                        (0x00FFFFFF)
#define DWC_XDCI_TRB_PCM1_MASK                             (0x03000000)
#define DWC_XDCI_TRB_PCM1_BIT_POS                          (24)
#define DWC_XDCI_TRB_STATUS_MASK                           (0xF0000000)
#define DWC_XDCI_TRB_STATUS_BIT_POS                        (28)
#define DWC_XDCI_TRB_STATUS_OK                             (0)
#define DWC_XDCI_TRB_STATUS_MISSED_ISOCH                   (1)
#define DWC_XDCI_TRB_STATUS_SETUP_PENDING                  (2)

#define DWC_XDCI_TRB_CTRL_HWO_MASK                         (0x00000001)
#define DWC_XDCI_TRB_CTRL_LST_TRB_MASK                     (0x00000002)
#define DWC_XDCI_TRB_CTRL_LST_TRB_BIT_POS                  (1)
#define DWC_XDCI_TRB_CTRL_CHAIN_BUFF_MASK                  (0x00000004)
#define DWC_XDCI_TRB_CTRL_CHAIN_BUFF_BIT_POS               (2)
#define DWC_XDCI_TRB_CTRL_CSP_MASK                         (0x00000008)
#define DWC_XDCI_TRB_CTRL_CSP_BIT_POS                      (3)
#define DWC_XDCI_TRB_CTRL_TYPE_MASK                        (0x000003F0)
#define DWC_XDCI_TRB_CTRL_TYPE_BIT_POS                     (4)
#define DWC_XDCI_TRB_CTRL_TYPE_NORMAL                      (1)
#define DWC_XDCI_TRB_CTRL_TYPE_SETUP                       (2)
#define DWC_XDCI_TRB_CTRL_TYPE_STATUS2                     (3)
#define DWC_XDCI_TRB_CTRL_TYPE_STATUS3                     (4)
#define DWC_XDCI_TRB_CTRL_TYPE_DATA                        (5)
#define DWC_XDCI_TRB_CTRL_TYPE_ISOCH_FIRST                 (6)
#define DWC_XDCI_TRB_CTRL_TYPE_ISOCH                       (7)
#define DWC_XDCI_TRB_CTRL_TYPE_LINK_TRB                    (8)
#define DWC_XDCI_TRB_CTRL_IOSP_MISOCH_MASK                 (0x00000400)
#define DWC_XDCI_TRB_CTRL_IOSP_MISOCH_BIT_POS              (10)
#define DWC_XDCI_TRB_CTRL_IOC_MASK                         (0x00000800)
#define DWC_XDCI_TRB_CTRL_IOC_BIT_POS                      (11)
#define DWC_XDCI_TRB_CTRL_STRM_ID_SOF_NUM_MASK             (0x3FFFC000)
#define DWC_XDCI_TRB_CTRL_STRM_ID_SOF_BIT_POS              (14)

#define DWC_XDCI_DEV_EVENT_DEFAULT_SIZE_IN_BYTES           (4)
#define DWC_XDCI_DEV_EVENT_TST_LMP_SIZE_IN_BYTES           (12)

typedef enum {
	EPCMD_SET_EP_CONFIG = 1,
	EPCMD_SET_EP_XFER_RES_CONFIG,
	EPCMD_GET_EP_STATE,
	EPCMD_SET_STALL,
	EPCMD_CLEAR_STALL,
	EPCMD_START_XFER,
	EPCMD_UPDATE_XFER,
	EPCMD_END_XFER,
	EPCMD_START_NEW_CONFIG = 9
} DWC_XDCI_ENDPOINT_CMD;

typedef enum {
	ON = 0,
	SLEEP = 2,
	SUSPEND,
	DISCONNECTED,
	EARLY_SUSPEND,
	RESET = 14,
	RESUME = 15
} DWC_XDCI_HS_LINK_STATE;

typedef enum {
	TRBCTL_NORMAL = 1,
	TRBCTL_SETUP,
	TRBCTL_2_PHASE,
	TRBCTL_3_PHASE,
	TRBCTL_CTRL_DATA_PHASE,
	TRBCTL_ISOCH_FIRST,
	TRBCTL_ISOCH,
	TRBCTL_LINK
} DWC_XDCI_TRB_CONTROL;

/* TODO: Make sure none of the mechanisms and data structures
 *  are leaving any holes
 */

/* DWC XDCI Endpoint Commands Parameters struct */
typedef struct {
	UINT32 param2;
	UINT32 param1;
	UINT32 param0;
} DWC_XDCI_ENDPOINT_CMD_PARAMS;

/* Event Buffer Struct */
typedef struct {
	UINT32 event;
	UINT32 dev_tst_lmp1;
	UINT32 dev_tst_lmp2;
	UINT32 reserved;
} DWC_XDCI_EVENT_BUFFER;

/* Transfer Request Block */
typedef struct {
	UINT32 buff_ptr_low;
	UINT32 buff_ptr_high;
	UINT32 len_xfer_params;
	UINT32 trb_ctrl;
} DWC_XDCI_TRB;

typedef struct  {
	USB_EP_INFO       ep_info;
	DWC_XDCI_TRB      *trb;
	USB_XFER_REQUEST  xfer_handle;
	UINT32            currentXferRscIdx;
	VOID              *core_handle;
	USB_EP_STATE      state;
	USB_EP_STATE      Orgstate;
	BOOLEAN           CheckFlag;
} DWC_XDCI_ENDPOINT;

typedef struct {
	/* cb_event_params must be copied over by upper layer if
	 * it defers event processing
	 */
	USB_DEVICE_CALLBACK_PARAM cb_event_params;

	/* Callback function list */
	USB_DEVICE_CALLBACK_FUNC  dev_disconnect_callback;
	USB_DEVICE_CALLBACK_FUNC  dev_bus_reset_callback;
	USB_DEVICE_CALLBACK_FUNC  dev_reset_done_callback;
	USB_DEVICE_CALLBACK_FUNC  dev_link_state_callback;
	USB_DEVICE_CALLBACK_FUNC  dev_wakeup_callback;
	USB_DEVICE_CALLBACK_FUNC  dev_hibernation_callback;
	USB_DEVICE_CALLBACK_FUNC  dev_sof_callback;
	USB_DEVICE_CALLBACK_FUNC  dev_erratic_err_callback;
	USB_DEVICE_CALLBACK_FUNC  dev_cmd_cmplt_callback;
	USB_DEVICE_CALLBACK_FUNC  dev_buff_ovflw_callback;
	USB_DEVICE_CALLBACK_FUNC  dev_test_lmp_rx_callback;
	USB_DEVICE_CALLBACK_FUNC  dev_setup_pkt_received_callback;
	USB_DEVICE_CALLBACK_FUNC  dev_xfer_nrdy_callback;
	USB_DEVICE_CALLBACK_FUNC  dev_xfer_done_callback;
} USB_DEV_CALLBACK_LIST;

typedef struct {
	VOID                     *parent_handle;                                      // Pointer to the parent this driver is associated
	USB_CONTROLLER_ID        id;                                                  // ID of the controllers supported in our DCD
	USB_SPEED                desired_speed;                                       // Desired SS, HS, FS or LS speeds for the core
	USB_ROLE                 role;                                                // Desired role i.e. host, device or OTG
	USB_SPEED                actual_speed;                                        // Actual speed
	USB_DEVICE_STATE         dev_state;                                           // Device state
	UINT32                   base_address;                                        // Register base address
	UINT32                   flags;                                               // Init flags
	UINT32                   max_dev_int_lines;                                   // One event buffer per interrupt line
	DWC_XDCI_EVENT_BUFFER    event_buffers [DWC_XDCI_MAX_EVENTS_PER_BUFFER * 2];  // Event buffer pool
	DWC_XDCI_EVENT_BUFFER    *aligned_event_buffers;                              // Aligned event buffer pool
	DWC_XDCI_EVENT_BUFFER    *current_event_buffer;                               // Current event buffer address
	DWC_XDCI_TRB             unaligned_trbs [(DWC_XDCI_MAX_ENDPOINTS + 1) * DWC_XDCI_TRB_NUM];    // TRBs.
	DWC_XDCI_TRB             *trbs;                                               // 16-bytes aligned TRBs.
	DWC_XDCI_ENDPOINT        ep_handles [DWC_XDCI_MAX_ENDPOINTS];                 // EPs
	UINT8                    default_setup_buffer [DWC_XDCI_SETUP_BUFF_SIZE * 2]; // Unaligned setup buffer
	UINT8                    *aligned_setup_buffer;                               // Aligned setup buffer. Aligned to 8-byte boundary
	UINT8                    misc_buffer [528];                                   // Unaligned misc buffer
	UINT8                    *aligned_misc_buffer;                                // Aligned misc buffer
	UINT32                   link_state;                                          // Link state
	UINT32                   hird_val;                                            // HIRD value
	USB_DEV_CALLBACK_LIST    event_callbacks;
	volatile BOOLEAN         interrup_processing;
} XDCI_CORE_HANDLE;

/* DWC XDCI API prototypes */
EFI_STATUS
EFIAPI
dwc_xdci_core_init (
	IN USB_DEV_CONFIG_PARAMS    *config_params,
	IN VOID                     *parent_handle,
	IN VOID                     **core_handle
	);

EFI_STATUS
EFIAPI
dwc_xdci_core_deinit (
	IN VOID      *core_handle,
	IN UINT32    flags
	);

EFI_STATUS
EFIAPI
dwc_xdci_core_register_callback (
	IN VOID                      *core_handle,
	IN USB_DEVICE_EVENT_ID       event,
	IN USB_DEVICE_CALLBACK_FUNC  CallbackFunc
	);

EFI_STATUS
EFIAPI
dwc_xdci_core_unregister_callback (
	IN VOID                   *core_handle,
	IN USB_DEVICE_EVENT_ID    event
	);

EFI_STATUS
EFIAPI
dwc_xdci_core_isr_routine (
	IN VOID *core_handle
	);

EFI_STATUS
EFIAPI
dwc_xdci_core_isr_routine_timer_based (
	IN VOID *core_handle
	);

EFI_STATUS
EFIAPI
dwc_xdci_core_connect (
	IN VOID    *core_handle
	);

EFI_STATUS
EFIAPI
dwc_xdci_core_disconnect (
	IN VOID *core_handle
	);

EFI_STATUS
EFIAPI
dwc_xdci_core_get_speed (
	IN VOID         *core_handle,
	IN USB_SPEED    *speed
	);

EFI_STATUS
EFIAPI
dwc_xdci_core_set_address (
	IN VOID      *core_handle,
	IN UINT32    address
	);

EFI_STATUS
EFIAPI
dwc_xdci_core_set_config (
	IN VOID      *core_handle,
	IN UINT32    config_num
	);

EFI_STATUS
EFIAPI
dwc_xdci_set_link_state (
	IN VOID                        *core_handle,
	IN USB_DEVICE_SS_LINK_STATE    state
	);

EFI_STATUS
EFIAPI
dwc_xdci_init_ep (
	IN VOID           *core_handle,
	IN USB_EP_INFO    *ep_info
	);

EFI_STATUS
EFIAPI
dwc_xdci_ep_enable (
	IN VOID           *core_handle,
	IN USB_EP_INFO    *ep_info
	);

EFI_STATUS
EFIAPI
dwc_xdci_ep_disable (
	IN VOID           *core_handle,
	IN USB_EP_INFO    *ep_info
	);

EFI_STATUS
EFIAPI
dwc_xdci_ep_stall (
	IN VOID           *core_handle,
	IN USB_EP_INFO    *ep_info
	);

EFI_STATUS
EFIAPI
dwc_xdci_ep_clear_stall (
	IN VOID           *core_handle,
	IN USB_EP_INFO    *ep_info
	);

EFI_STATUS
EFIAPI
dwc_xdci_ep_set_nrdy (
	IN VOID           *core_handle,
	IN USB_EP_INFO    *ep_info
	);

EFI_STATUS
EFIAPI
dwc_xdci_ep0_receive_setup_pkt (
	IN VOID     *core_handle,
	IN UINT8    *buffer
	);

EFI_STATUS
EFIAPI
dwc_xdci_ep0_receive_status_pkt (
	IN VOID    *core_handle
	);

EFI_STATUS
EFIAPI
dwc_xdci_ep0_send_status_pkt (
	IN VOID    *core_handle
	);

EFI_STATUS
EFIAPI
dwc_xdci_ep_tx_data (
	IN VOID                *core_handle,
	IN USB_XFER_REQUEST    *xfer_req
	);

EFI_STATUS
EFIAPI
dwc_xdci_ep_rx_data(
	IN VOID                *core_handle,
	IN USB_XFER_REQUEST    *xfer_req
	);

EFI_STATUS
EFIAPI
dwc_xdci_ep_cancel_transfer (
	IN VOID           *core_handle,
	IN USB_EP_INFO    *ep_info
	);

EFI_STATUS
usb_process_device_reset_det (
	IN XDCI_CORE_HANDLE    *core_handle
	);

EFI_STATUS
usb_process_device_reset_done (
	IN XDCI_CORE_HANDLE    *core_handle
	);

UINT32
usb_get_physical_ep_num (
	IN UINT32        EndpointNum,
	IN USB_EP_DIR    EndpointDir
	);

UINT32
usb_reg_read (
	IN UINT32    base,
	IN UINT32    offset
	);

VOID
usb_reg_write (
	IN UINT32    base,
	IN UINT32    offset,
	IN UINT32    val
	);

EFI_STATUS
usb_xdci_core_flush_ep_fifo (
	IN VOID           *core_handle,
	IN USB_EP_INFO    *ep_info
	);
#endif
