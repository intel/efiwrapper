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

#ifndef _MMC_H_
#define _MMC_H_

#include <pci.h>
#include <stdbool.h>
#include <stdint.h>

/*
** Jedec commands
*/
#define CMD_RESET			0
#define CMD_GET_OP			1
#define CMD_ALL_SEND_CID		2
#define CMD_SEND_RCA			3
#define CMD_SWITCH			6
#define CMD_SELECT_CARD			7
#define CMD_GET_EXT_CSD			8
#define CMD_GET_STATE			13
#define CMD_SET_BLOCK_LENGTH		16
#define CMD_READ_SINGLE_BLOCK		17
#define CMD_READ_MULTIPLE_BLOCKS	18
#define CMD_SET_BLOCK_COUNT		23
#define CMD_WRITE_SINGLE_BLOCK		24
#define CMD_WRITE_MULTIPLE_BLOCKS	25

// SD Card
#define CMD_SEND_IF_COND	8
#define CMD_SEND_OP_COND	41
#define CMD_APP			55

/*
 * SET_BUS_WIDTH parameters
 */
#define SD_SET_BUS_WIDTH4	2
#define SD_SET_BUS_WIDTH1	0


#define MMC_SWITCH_MODE_WRITE_BYTE	0x03

/**
 * General structure reflecting the proprieties of a eMMC card
 * @Host: Underlaying host controller
 * @bus_width: Current bus width
 * @uhs_timing: operating mode (DDR50, SDR50, HS200)
 * @freq: Clock frequency
 * @rca: Relative card address
 * @ext_csd: Container for EXT_CSD register
 * @cid: Container for CID register
 * @init: Card was initialized or not
 */
struct mmc
{
	struct sdhci *host;
	uint8_t  bus_width;
	uint8_t  uhs_timing;
	uint8_t  card_type;
	uint32_t freq;
	uint32_t rca;
	uint8_t  ext_csd[512];
	uint8_t  cid[16];
	unsigned init;
};

/**
 * Command structure
 * @index: actuall comand beeing sent
 * @resp_len: length of the response
 * @addr: dma buffer for data transfers
 * @nblock: number of blocks to transfer
 * @flags: mask for determining the transfer type
 * @resp: buffer holding the response
 * @retry: how many times should we resend the command if cmd fails
 */
struct cmd
{
	unsigned  index;
	unsigned  resp_len;
	uintptr_t addr;
	unsigned  nblock;
	uint32_t  flags;
	uint32_t  args;
	uint32_t  resp[32];
	uint8_t   retry;

};

/*
** OCR Register constants
*/
#define OCR_BUSY	0x80000000
#define OCR_CCS		0x40000000
#define OCR_VDD_18	0x00000080
#define OCR_VDD_27_28	0x00008000
#define OCR_VDD_28_29	0x00010000
#define OCR_VDD_29_30	0x00020000
#define OCR_VDD_30_31	0x00040000
#define OCR_VDD_31_32	0x00080000
#define OCR_VDD_32_33	0x00100000
#define OCR_VDD_33_34	0x00200000
#define OCR_VDD_34_35	0x00400000
#define OCR_VDD_35_36	0x00800000

#define CARD_TYPE_SD10  1
#define CARD_TYPE_SD20  2
#define CARD_TYPE_MMC   3

#define RCA_MMC	1

/*
**  EXT_CSD byte offsets used for SWITCH command
*/
#define EXT_CSD_PARTITION_CONFIG	179
#define EXT_CSD_BOOT_BUS_CONDITIONS	177

#define EXT_CSD_BUS_WIDTH	183
#define MMC_BUS_WIDTH_4		0x01
#define MMC_BUS_WIDTH_8		0x02
#define MMC_BUS_WIDTH_4_DDR	0x05
#define MMC_BUS_WIDTH_8_DDR	0x06


#define EXT_CSD_HS_TIMING	185
#define EXT_CSD_HS_ENABLE	0x01
#define EXT_CSD_HS200_ENABLE	0x02
#define EXT_CSD_HS400_ENABLE	0x03


#define EXT_CSD_DEVICE_TYPE	196
#define CARD_TYPE_HS		0x03
#define CARD_TYPE_DDR50		0x0c
#define CARD_TYPE_HS200		0x30
#define CARD_TYPE_HS400		0xc0

#define EXT_CSD_SEC_COUNT	212

extern int mmc_init_card(pcidev_t dev);
extern int mmc_send_cmd(struct cmd *c);
extern int mmc_wait_cmd_done(struct cmd *c);
extern int mmc_switch(struct mmc *m, uint8_t index, uint8_t value);
extern int mmc_cid(uint8_t cid[16]);

extern int mmc_enable_hs200(struct mmc *m);
extern int mmc_enable_hs400(struct mmc *m);
extern int mmc_enable_ddr50(struct mmc *m);
extern int mmc_enable_sdr50(struct mmc *m);
extern int mmc_enable_sdr25(struct mmc *m);

extern void mmc_dll_tune(void);
extern uint64_t mmc_read_count(void);
extern int mmc_update_ext_csd(void);
#endif
