/******************************************************************************
 *
 * INTEL CONFIDENTIAL
 *
 * Copyright (c) 1999-2013 Intel Corporation All Rights Reserved.
 *
 * The source code contained or described herein and all documents related to
 * the source code (Material) are owned by Intel Corporation or its suppliers
 * or licensors. Title to the Material remains with Intel Corporation or its
 * suppliers and licensors. The Material contains trade secrets and proprietary
 * and confidential information of Intel or its suppliers and licensors. The
 * Material is protected by worldwide copyright and trade secret laws and
 * treaty provisions. No part of the Material may be used, copied, reproduced,
 * modified, published, uploaded, posted, transmitted, distributed, or
 * disclosed in any way without Intel's prior express written permission.
 *
 * No license under any patent, copyright, trade secret or other intellectual
 * property right is granted to or conferred upon you by disclosure or delivery
 * of the Materials, either expressly, by implication, inducement, estoppel or
 * otherwise. Any license under such intellectual property rights must be
 * express and approved by Intel in writing.
 *
 ******************************************************************************/

#ifndef _BXT_SDHCI_H
#define _BXT_SDHCI_H_

#include "sdhci.h"

/*
 * Controller registers
 */
#define SDHCI_DMA_ADDR		0x00
#define SDHCI_BLOCK_SIZE	0x04

/* Block size constants */
#define DMA_4K_BOUNDRY		0x0000
#define DMA_8K_BOUNDRY		0x1000
#define DMA_16K_BOUNDRY		0x2000
#define DMA_32K_BOUNDRY		0x3000
#define DMA_64K_BOUNDRY		0x4000
#define DMA_128K_BOUNDRY	0x5000
#define DMA_256K_BOUNDRY	0x6000
#define DMA_512K_BOUNDRY	0x7000

#define SDHCI_BLOCK_CNT	0x06
#define SDHCI_ARGUMENT	0x08

#define SDHCI_TRANSFER_MODE	0x0C
/* Transfer mode register constants */
#define TM_USE_DMA              0x0001
#define TM_BLOCK_CNT_ENABLE     0x0002
#define TM_AUTO_CMD12_ENABLE    0x0004
#define TM_AUTO_CMD23_ENABLE    0x0008
#define TM_READ                 0x0010
#define TM_WRITE                0x0000
#define TM_MULTI_BLOCK          0x0020

#define SDHCI_CMD_REG			0x0E
/* SDHC Command register definitions */
#define SDHCI_CMD_NO_RESP		0x0000 /* No response                                      */
#define SDHCI_CMD_RL136			0x0001 /* 136 byte response                                */
#define SDHCI_CMD_RL48			0x0002 /* 48 byte response                                 */
#define SDHCI_CMD_RL48_CB		0x0003 /* 48 byte response with busy check after response  */
#define SDHCI_CMD_CRC_CHECK_ENABLE	0x0008
#define SDHCI_CMD_INDEX_CHECK_ENABLE	0x0010
#define SDHCI_CMD_DATA_PRESENT		0x0020
#define SDHCI_CMD_TYPE_NORMAL		0x0000
#define SDHCI_CMD_TYPE_SUSPEND		0x0040
#define SDHCI_CMD_TYPE_RESUME		0x0080
#define SDHCI_CMD_TYPE_ABORT		0x00C0
#define SDHCI_CMD_INDEX_SHIFT		8

#define SDHCI_RESPONSE	0x10
#define SDHCI_BUFFER	0x20

#define SDHCI_PRESENT_STATE	0x24
#define SDHCI_CMD_INHIBIT	0x01
#define SDHCI_DATA_INHIBIT	0x02

#define SDHCI_HOST_CTRL	0x28

#define SDHCI_LED_ON            0x01
#define SDHCI_WIDTH_1BIT        0x00
#define SDHCI_WIDTH_4BITS       0x02
#define SDHCI_WIDTH_8BITS       0x20
#define SDHCI_HS_ENABLE         0x04
#define SDHCI_DMA_SDMA          0x00
#define SDHCI_DMA_ADMA32        0x10
#define SDHCI_DMA_ADMA64        0x18

#define SDHCI_POWER_CONTROL	0x29
#define SDHCI_POWER_ON		0x01
#define SDHCI_POWER_18V		0x0A
#define SDHCI_POWER_30V		0x0C
#define SDHCI_POWER_33V		0x0E

#define SDHCI_BLOCK_GAP_CTRL	0x2A
#define BOOT_EN                 0x80
#define BOOT_ACK_RCV            0x20

#define SDHCI_WAKE_UP_CONTROL	0x2B

#define  SDHCI_CLOCK_CONTROL		0x2C
#define  SDHCI_DIVIDER_SHIFT		8
#define  SDHCI_DIVIDER_HI_SHIFT		6
#define  SDHCI_DIV_MASK			0xFF
#define  SDHCI_DIV_MASK_LEN		8
#define  SDHCI_DIV_HI_MASK		0x300
#define  SDHCI_CLOCK_CARD_ENABLE	(1 << 2)
#define  SDHCI_CLOCK_STABLE		(1 << 1)
#define  SDHCI_CLOCK_ENABLE		(1 << 0)

#define SDHCI_TIMEOUT_CONTROL	0x2E

#define  SDHCI_SOFTWARE_RESET	0x2F
#define  SDHCI_RESET_ALL	0x01
#define  SDHCI_RESET_CMD	0x02
#define  SDHCI_RESET_DATA	0x04

#define SDHCI_INT_STATUS	0x30
#define SDHCI_ERR_INT_STATUS    0x32
#define SDHCI_INT_ENABLE	0x34
#define SDHCI_ERR_INT_ENABLE	0x36
#define SDHCI_SIGNAL_ENABLE	0x38

/* Normal interrupt status constants */
#define SDHCI_INT_CMD_COMPLETE          0x0001
#define SDHCI_INT_XFER_COMPLETE         0x0002
#define SDHCI_INT_BLOCK_GAP_EVENT       0x0004
#define SDHCI_INT_DMA_INT               0x0008
#define SDHCI_INT_BUFFER_WRITE_RDY      0x0010
#define SDHCI_INT_BUFFER_READ_RDY       0x0020
#define SDHCI_INT_CARD_INSERTION        0x0040
#define SDHCI_INT_CARD_REMOVAL          0x0080
#define SDHCI_INT_CARD_INT              0x0100
#define SDHCI_INT_ERR_INT               0x8000
#define SDHCI_INT_BOOT_ACK_RCV          0x2000
#define SDHCI_INT_BOOT_TERM             0x4000

#define SDHCI_INT_ALL  (SDHCI_INT_CMD_COMPLETE | SDHCI_INT_XFER_COMPLETE | SDHCI_INT_DMA_INT | \
			SDHCI_INT_BUFFER_READ_RDY  | SDHCI_INT_BUFFER_WRITE_RDY   | \
			SDHCI_INT_BOOT_ACK_RCV | SDHCI_INT_BOOT_TERM | SDHCI_INT_ERR_INT ) \

#define SDHCI_HOST_CTRL2	0x3e
/* Host Ctrl2 */
#define SDHCI_SIGNALING_EN	0x08
#define SDHCI_EXECUTE_TUNING	0x40
#define SDHCI_CLOCK_TUNED	0x80
#define SDHCI_UHS_MODE_SELECT	0x07
#define SDHCI_UHS_SDR12		0x00
#define SDHCI_UHS_SDR25		0x01
#define SDHCI_UHS_SDR50		0x02
#define SDHCI_UHS_HS200		0x03
#define SDHCI_UHS_DDR50		0x04
#define SDHCI_UHS_HS400		0x05


#define  SDHCI_CAPABILITIES	        0x40
#define  SDHCI_TIMEOUT_CLK_MASK		0x0000003F
#define  SDHCI_TIMEOUT_CLK_SHIFT	0
#define  SDHCI_TIMEOUT_CLK_UNIT		0x80
#define  SDHCI_CLOCK_BASE_MASK		0x3F00
#define  SDHCI_CLOCK_V3_BASE_MASK	0xFF00
#define  SDHCI_CLOCK_BASE_SHIFT		8
#define  SDHCI_MAX_BLOCK_MASK		0x30000
#define  SDHCI_MAX_BLOCK_SHIFT		16
#define  SDHCI_CAPS_8BIT		0x40000

#define  SDHCI_CAPABILITIES2    0x44
#define  SDHCI_CAPS2_SDR50      0x01
#define  SDHCI_CAPS2_HS200      0x02
#define  SDHCI_CAPS2_DDR50      0x04
#define  SDHCI_SUPPORT_HS400    0x80000000 /* Non-standard -- Linux Kernel */

#define  SDHCI_CAPS_VS33	0x01000000
#define  SDHCI_CAPS_VS30	0x02000000
#define  SDHCI_CAPS_VS18	0x04000000

#define SDHCI_MAX_CURRENT	0x48

/* 4C-4F reserved for more max current */

#define SDHCI_SET_ACMD12_ERROR	0x50
#define SDHCI_SET_INT_ERROR	0x52

#define SDHCI_ADMA_ERROR	0x54

/* 55-57 reserved */

#define SDHCI_ADMA_ADDRESS	0x58

/* 60-FB reserved */

#define SDHCI_SLOT_INT_STATUS	0xFC

#define SDHCI_BOOT_TIMEOUT_CTRL 0x70

#define SDHCI_HOST_VERSION	0xFE

#define SDHCI_GET_VERSION(x) (x->version & SDHCI_SPEC_VER_MASK)

/* These flags map directly to hardware.  DO NOT CHANGE */
#define CMDF_DATA_XFER		0x00000020
#define CMDF_DIRECT_MASK	0x00000038
#define CMDF_USE_DMA		0x00000100
#define CMDF_NO_RESPONSE	0x00000200
#define CMDF_BUSY_CHECK		0x00000800
#define CMDF_RD_XFER		0x00001000
#define CMDF_WR_XFER		0x00002000
#define CMDF_BOOT_EN		0x00004000

#define CMDF_CRC_CHECK		SDHCI_CMD_CRC_CHECK_ENABLE
#define CMDF_CMD_INDEX_CHECK	SDHCI_CMD_CMD_INDEX_CHECK_ENABLE

/*
 * End of controller registers.
 */
#define SDHCI_MAX_DIV_SPEC_200	256
#define SDHCI_MAX_DIV_SPEC_300	2046

#define DEFAULT_BLOCK_SIZE	512

//
// HS400 Tuning Definitions
//
#define RX_STROBE_DLL1_TAP_MAX_RANGE		79
#define RX_STROBE_DLL1_TAP_MAX_RANGE_HS400	39
#define RX_STROBE_DLL1_TAP_MIN_RANGE		0

#define TX_DATA_DLL_TAP_MAX_RANGE       79
#define TX_DATA_DLL_TAP_MIN_RANGE       0


#define R_SCC_MEM_SW_LTR_VALUE                              0x804 // Software LTR Register
#define R_SCC_MEM_CAP_BYPASS_REG1                           0x814 // Capabilities Bypass Register
#define R_SCC_MEM_IDLE_CTRL                                 0x81C // DevIdle Control per SCC slice
#define B_SCC_MEM_CAP_BYPASS_REG1_HS400                     BIT29
#define R_SCC_MEM_TX_CMD_DLL_CNTL                           0x820 // Tx CMD Path Ctrl
#define R_SCC_MEM_TX_DATA_DLL_CNTL1                         0x824 // Tx Data Path Ctrl 1
#define R_SCC_MEM_TX_DATA_DLL_CNTL2                         0x828 // Tx Data Path Ctrl 2
#define R_SCC_MEM_RX_CMD_DATA_DLL_CNTL1                     0x82C // Rx CMD&Data Path Ctrl 1
#define R_SCC_MEM_RX_STROBE_DLL_CNTL                        0x830 // Rx Strobe Ctrl Path
#define R_SCC_MEM_RX_CMD_DATA_DLL_CNTL2                     0x834 // Rx CMD&Data Path Ctrl 2
#define N_SCC_MEM_RX_CMD_DATA_DLL_CNTL2_CLKSRC_RX           16
#define V_SCC_MEM_RX_CMD_DATA_DLL_CNTL2_CLKSRC_RX_CLK_AUTO  0x2
#define R_SCC_MEM_CUR_XFSM                                  0x858 // Internal Clock Unit XFSM

#endif
