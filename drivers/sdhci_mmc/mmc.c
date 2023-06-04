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

#include <stdbool.h>
#include <kconfig.h>
#include <libpayload.h>
#include <pci.h>
#include <ewlog.h>

#include <hwconfig.h>

#include "sdhci_mmc/mmc.h"
#include "sdhci_mmc/sdhci.h"
#include "sdhci_mmc/sdhci-internal.h"


#if defined (PLATFORM_KABYLAKE) || defined (PLATFORM_ICELAKE)
#define NO_HS400
#endif

/*
** Global instance of the eMMC card
*/
struct mmc card;

static int __mmc_send_cmd(struct cmd *c)
{
	struct mmc *m = &card;

	m->host->send_cmd(m, c);
#if MMC_DEBUG
	ewdbg("=== DEBUG CMD %d args %x ==== ", c->index, c->args);
	ewdbg("SDHCI_INT_ENABLE %x", sdhci_read16(m->host, SDHCI_INT_ENABLE));
	ewdbg("SDHCI_TRANSFER_MODE %x", sdhci_read16(m->host, SDHCI_TRANSFER_MODE));
	ewdbg("SDHCI_CMD_REG %x", sdhci_read16(m->host, SDHCI_CMD_REG));
	ewdbg("SDHCI_BLOCK_SIZE %x", sdhci_read16(m->host, SDHCI_BLOCK_SIZE));
	ewdbg("SDHCI_DMA_ADDR %x", sdhci_read32(m->host, SDHCI_DMA_ADDR));
	ewdbg("SDHCI_HOST_CTRL %x", sdhci_read8(m->host, SDHCI_HOST_CTRL));
	ewdbg("SDHCI_POWER_CONTROL %x", sdhci_read8(m->host, SDHCI_POWER_CONTROL));
	ewdbg("SDHCI_CLOCK_CONTROL %x", sdhci_read16(m->host, SDHCI_CLOCK_CONTROL));
	ewdbg("SDHCI_ARGUMENT %x", sdhci_read32(m->host, SDHCI_ARGUMENT));
	ewdbg("SDHCI_BLOCK_CNT %x", sdhci_read16(m->host, SDHCI_BLOCK_CNT));
	ewdbg("SDHCI_INT_STATUS %x", sdhci_read16(m->host, SDHCI_INT_STATUS));
#endif

	return m->host->wait_cmd_done(m, c);
}

int mmc_send_cmd(struct cmd *c)
{
	struct mmc *m = &card;

	m->host->send_cmd(m, c);
#if MMC_DEBUG
	ewdbg("==== DEBUG CMD %d args %x ===== ", c->index, c->args);
	ewdbg("SDHCI_INT_ENABLE %x", sdhci_read16(m->host, SDHCI_INT_ENABLE));
	ewdbg("SDHCI_TRANSFER_MODE %x", sdhci_read16(m->host, SDHCI_TRANSFER_MODE));
	ewdbg("SDHCI_CMD_REG %x", sdhci_read16(m->host, SDHCI_CMD_REG));
	ewdbg("SDHCI_BLOCK_SIZE %x", sdhci_read16(m->host, SDHCI_BLOCK_SIZE));
	ewdbg("SDHCI_DMA_ADDR %x", sdhci_read32(m->host, SDHCI_DMA_ADDR));
	ewdbg("SDHCI_HOST_CTRL %x", sdhci_read8(m->host, SDHCI_HOST_CTRL));
	ewdbg("SDHCI_POWER_CONTROL %x", sdhci_read8(m->host, SDHCI_POWER_CONTROL));
	ewdbg("SDHCI_CLOCK_CONTROL %x", sdhci_read16(m->host, SDHCI_CLOCK_CONTROL));
	ewdbg("SDHCI_ARGUMENT %x", sdhci_read32(m->host, SDHCI_ARGUMENT));
	ewdbg("SDHCI_BLOCK_CNT %x", sdhci_read16(m->host, SDHCI_BLOCK_CNT));
#endif
	return 0;
}

uint64_t mmc_read_count(void) {

	struct mmc *m = &card;

	return  (m->ext_csd[EXT_CSD_SEC_COUNT + 0] << 0 |
		 m->ext_csd[EXT_CSD_SEC_COUNT + 1] << 8 |
		 m->ext_csd[EXT_CSD_SEC_COUNT + 2] << 16 |
		 m->ext_csd[EXT_CSD_SEC_COUNT + 3] << 24);
}

/* ------------------------------------------------------------------------ */
/*
** Print salient properties read from the eMMC CID and EXT_CSD registers
** (protocol version, device capacity, boot partition settings etc.)
*/
#if DEBUG_MESSAGES
static void
emmc_show_hwinfo(struct mmc *m)
{
	// Device density, EXT_CSD.SEC_COUNT [215:212]
	uint64_t sec_count;
	char *uhs_timing[6] = {"SDR12", "SDR25", "SDR50", "HS200", "DDR50", "HS400"};
	sec_count = mmc_read_count() >> 11;

	// eMMC device revision, derived from EXT_CSD_REF[192]
	unsigned ext_csd_rev = m->ext_csd[192];
	const char *rev = (ext_csd_rev == 5) ? "4.41" :
		(ext_csd_rev == 6) ? "4.51" :
		(ext_csd_rev == 7) ? "5.1"  : "?";

	// Device firmware revision, CID.PRV
	unsigned cid_prv = m->cid[6-1];

	// Manufacuring date, CID.MDT
	unsigned cid_mdt = m->cid[1-1];

	ewdbg("MMC driver: %dMB, boot %x/%x  [%s %s FW %x.%x %x %d/%d]",
	      (unsigned int)sec_count,
	      m->ext_csd[179],	// EXT_CSD.PARTITION_CONFIG
	      m->ext_csd[177],	// EXT_CSD.BOOT_BUS_CONDITION
	      rev, uhs_timing[m->uhs_timing],
	      cid_prv >> 4, cid_prv & 0xf,
	      m->cid[2-1],    // CID.PSN, product serial number
	      cid_mdt >> 4, (cid_mdt & 0xf) + 13);
}
#endif

int mmc_wait_cmd_done(struct cmd *c)
{
	struct mmc *m = &card;

	return m->host->wait_cmd_done(m, c);
}

static void mmc_reset()
{
	struct cmd c;
	c.index    = CMD_RESET;
	c.args     = 0x0;
	c.flags    = CMDF_NO_RESPONSE;
	c.resp_len = 0;

	__mmc_send_cmd(&c);
}

/* ------------------------------------------------------------------------ */
/*
** eMMC: cmd API
*/
static int mmc_send_cmd1()
{
	struct cmd c;
	unsigned busy = 1;
	uint64_t start = timer_us(0);

	c.index    = CMD_GET_OP;
	c.flags    = 0;
	c.resp_len = 32;
	c.args     = OCR_VDD_18 | OCR_CCS;

	while (busy)
	{
		if (timer_us(start) > 2000 * 1000)
			return 1;

		if (__mmc_send_cmd(&c) != 0)
			return 1;

		busy = !(c.resp[0] & OCR_BUSY);
	}

	return 0;
}

static int mmc_get_cid(struct mmc *m)
{
	struct cmd c;
	int err;

	c.index    = CMD_ALL_SEND_CID;
	c.resp_len = 128;
	c.args     = 0;
	c.flags    = 0;

	err = __mmc_send_cmd(&c);
	if (err)
	{
		ewerr("Error %s() failed ", __func__);
		return err;
	}

	memcpy(m->cid, c.resp, sizeof(m->cid));//NOLINT
	return 0;
}

static void mmc_card_select(struct mmc *m)
{
	struct cmd c;

	c.index    = CMD_SELECT_CARD;
	c.args     = m->rca << 16;
	c.resp_len = 32;
	c.flags    = 0;

	__mmc_send_cmd(&c);
}

static void mmc_set_rca(struct mmc *m)
{
	struct cmd c;

	m->rca     = RCA_MMC;

	c.index    = CMD_SEND_RCA;
	c.args     = m->rca << 16;
	c.resp_len = 32;
	c.flags    = 0;

	__mmc_send_cmd(&c);
}

static int mmc_read_ext_csd(struct mmc *m)
{
	struct cmd c;
	c.index    = CMD_GET_EXT_CSD;
	c.addr     = (uintptr_t) m->ext_csd;
	c.flags    = CMDF_DATA_XFER | CMDF_RD_XFER | CMDF_USE_DMA;
	c.resp_len = 32;
	c.nblock   = 1;
	c.args     = 0;

	return __mmc_send_cmd(&c);
}

int mmc_update_ext_csd()
{
	struct mmc *m = &card;
	return mmc_read_ext_csd(m);
}

int
mmc_switch(struct mmc *m, uint8_t index, uint8_t value)
{
	struct cmd c;
	uint8_t state;
	uint64_t start = timer_us(0);

	c.args     = (MMC_SWITCH_MODE_WRITE_BYTE << 24)
		| (index << 16)
		| (value <<  8);
	c.resp_len = 32;
	c.index    = CMD_SWITCH;
	c.flags    = 0;
	c.retry    = 5;

	if (__mmc_send_cmd(&c) != 0)
		return 1;

	mdelay(1);
	/*
	** After Switch command the card can be still in
	** Programming state. Wait for it to become ready.
	*/
	c.resp_len = 32;
	c.index    = CMD_GET_STATE;
	c.flags    = 0;
	c.args     = m->rca << 16;
	c.retry    = 5;

	do
	{
		if (timer_us(start) > 100 * 1000)
			return 1;

		if (__mmc_send_cmd(&c) != 0)
			return 1;

		if (c.resp [0] & 0x80) /* Switch Error */
			return 1;

		state = (c.resp [0] >> 9) & 0xf;

	} while (state == 7); /* 7 = Programming State */

	return 0;
}

int mmc_cid(uint8_t cid[16])
{
	struct mmc *m = &card;
	memcpy(cid, m->cid, sizeof(m->cid));//NOLINT
	return 0;
}

static int mmc_card_hs200(struct mmc *m)
{
	return (((m->host->caps2 & SDHCI_CAPS2_HS200) == SDHCI_CAPS2_HS200)
		&& (m->ext_csd[EXT_CSD_DEVICE_TYPE] & CARD_TYPE_HS200));
}

#ifndef NO_HS400
static int mmc_card_hs400(struct mmc *m)
{
	return ((m->host->caps2 & SDHCI_SUPPORT_HS400)
		&& (m->ext_csd[EXT_CSD_DEVICE_TYPE] & CARD_TYPE_HS400));
}
#endif

int mmc_enable_hs200(struct mmc *m)
{
	unsigned err = 0;
	err = mmc_switch(m, EXT_CSD_BUS_WIDTH, MMC_BUS_WIDTH_8);
	if (err)
	{
		ewerr("%s() BUS_WIDTH 8", __func__);
		return err;
	}

	err = mmc_switch(m, EXT_CSD_HS_TIMING, EXT_CSD_HS200_ENABLE);
	if (err)
	{
		ewerr("%s() HS_TIMING HS200", __func__);
		return err;
	}

	m->freq	   = 200000;
	m->bus_width      = 8;
	m->uhs_timing     = SDHCI_UHS_HS200;

	m->host->set_mode(m);

	return 0;
}

#ifndef NO_HS400
/*
** Select HS400 mode - see JEDEC84-B51 standard
** Mode selection assumes HS400 is already enabled
*/
static int mmc_hs200_to_hs400(struct mmc *m)
{
	unsigned err = 0;

	/*
	** 7. Set the HS_TIMING [185] to 0x1 and clk <= 52 Mhz
	*/
	m->freq       = 50000;
	m->uhs_timing = SDHCI_UHS_SDR50;

	m->host->set_mode(m);

	err = mmc_switch(m, EXT_CSD_HS_TIMING, EXT_CSD_HS_ENABLE);
	if (err)
	{
		return err;
	}

	/*
	** 8. Set BUS_WIDTH [183] to 0x06 to select the dual data rate x8 bus mode
	*/
	err = mmc_switch(m, EXT_CSD_BUS_WIDTH, MMC_BUS_WIDTH_8_DDR);
	if (err)
	{
		return err;
	}

	/*
	** 9. Set HS_TIMING [185] to 0x3 to select HS400
	*/
	err = mmc_switch(m, EXT_CSD_HS_TIMING, EXT_CSD_HS400_ENABLE);
	if (err)
	{
		return err;
	}

	m->freq	   = 200000;
	m->bus_width      = 8;
	m->uhs_timing     = SDHCI_UHS_HS400;

	m->host->set_mode(m);

	return 0;
}


/*
** Enable HS400: Restore HS200 if fails
*/
int mmc_enable_hs400(struct mmc *m)
{
	int err = 0;

	err = mmc_hs200_to_hs400(m);

	return err;
}
#endif

/*
** Main function for initializing SD/eMMC card.
*/
int mmc_init_card(pcidev_t dev)
{
	struct mmc *m = &card;
	int err;

	if (m->init)
		return 0;

	/*
	** SD card init may fail due to some residuous
	** eMMC initialization (e.g from tuning).
	** Make sure we start with a clean structure.
	*/
	memset((void *)m, 0, sizeof(struct mmc));//NOLINT

	struct sdhci *host = sdhci_find_controller(dev);
	if (! host)
	{
		ewerr("Error SDHCI host controller not found");
		return 1;
	}

	m->host = host;

	host->init_controller(host);

	mmc_reset();

	/*
	** There are two ways of differentiating between an SD or eMMC card.
	** CMD1 is illegal (timeout response) for SD card thus card is
	** eMMC. Same applies for eMMC and CMD8.
	** For fastboot sake assume card is eMMC.
	*/
	err = mmc_send_cmd1();

	if (err) {
		ewerr("Error MMC host controller not found");
		return 1;
	}
	m->card_type = CARD_TYPE_MMC;

	mmc_get_cid(m);

	mmc_set_rca(m);

	mmc_card_select(m);

	// MMC
	err = mmc_read_ext_csd(m);
	if (err) {
		ewerr(" MMC read ext csd failure");
		return err;
	}

	if (mmc_card_hs200(m)) {
		err = mmc_enable_hs200(m);
		if (err) {
			ewerr("MMC host hs200 enabling failure");
			return err;
		}

		if(host->execute_tuning != NULL)
			host->execute_tuning(m);

#ifndef NO_HS400
		if (mmc_card_hs400(m)) {
			err = mmc_enable_hs400(m);
			if (err) {
				ewerr("MMC host hs400 enabling failure");
				return err;
			}
			ewdbg("MMC host hs400 enabled");
		} else
#endif
			ewdbg("MMC host hs200 enabled");
	}
	else {
		ewerr("MMC host hs200 not supported");
		return 1;
	}

#if DEBUG_MESSAGES
	emmc_show_hwinfo(m);
#endif
	m->init = 1;

	return 0;
}
