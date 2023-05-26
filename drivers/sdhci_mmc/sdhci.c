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
#include <pci/pci.h>
#include <ewlog.h>

#include "sdhci_mmc/sdhci-internal.h"
#include "sdhci_mmc/mmc.h"

#define ms(x)			  	((x) * 1000)
#define RPMB_PARTITION		  	3
#define EXT_CSD_PARTITION_CONFIG	179

#define MAX_TUNING_LOOP			40

static void sdhci_reset(struct sdhci *host, uint8_t mask);

static void sdhci_start_tuning(struct sdhci *host)
{
	uint16_t ctrl;

	ctrl = sdhci_read16(host, SDHCI_HOST_CTRL2);
	ctrl |= SDHCI_EXECUTE_TUNING;
	ctrl |= SDHCI_SIGNALING_EN;
	sdhci_write16(host, SDHCI_HOST_CTRL2, ctrl);

	sdhci_write32(host, SDHCI_INT_ENABLE, SDHCI_INT_DATA_AVAIL);
	sdhci_write32(host, SDHCI_SIGNAL_ENABLE, SDHCI_INT_DATA_AVAIL);
}

static void sdhci_reset_tuning(struct sdhci *host)
{
	uint16_t ctrl;

	ctrl = sdhci_read16(host, SDHCI_HOST_CTRL2);
	ctrl &= ~SDHCI_CTRL_TUNED_CLK;
	ctrl &= ~SDHCI_EXECUTE_TUNING;
	sdhci_write16(host, SDHCI_HOST_CTRL2, ctrl);
}

static void sdhci_abort_tuning(struct sdhci *host)
{
	struct cmd tuning_cmd = {0};
	struct cmd wait_cmd;

	sdhci_write32(host, SDHCI_INT_ENABLE, SDHCI_INT_DATA_AVAIL);
	sdhci_write32(host, SDHCI_SIGNAL_ENABLE, SDHCI_INT_DATA_AVAIL);

	memset(&tuning_cmd, 0, sizeof(tuning_cmd));
	tuning_cmd.index = CMD_MMC_STOP_TRANSMISSION;
	tuning_cmd.flags = MMC_RESPONSE_SPI_R1B | MMC_RESPONSE_R1B | MMC_COMMAND_AC;
	tuning_cmd.flags |= CMDF_DATA_XFER | CMDF_USE_DMA;
	tuning_cmd.flags |= CMDF_WR_XFER;

	mmc_send_cmd(&tuning_cmd);

	memset(&wait_cmd, 0, sizeof(wait_cmd));
	wait_cmd.flags = CMDF_DATA_XFER;
	mmc_wait_cmd_done(&wait_cmd);
}

static EFI_STATUS sdhci_send_tuning(struct sdhci *host, uint32_t opcode)
{
	struct cmd tuning_cmd = {0};
	uint32_t buf[256]={0};
	uint64_t start;
	int ret;

	tuning_cmd.index = opcode;
	tuning_cmd.args = 0;
	tuning_cmd.flags = MMC_RESPONSE_R1 | MMC_COMMAND_ADTC;
	tuning_cmd.flags |= CMDF_RD_XFER;
	tuning_cmd.flags |= SDHCI_CMD_DATA_PRESENT;
	tuning_cmd.flags |= TM_USE_DMA;
	tuning_cmd.flags |= TM_BLOCK_CNT_ENABLE;
	tuning_cmd.resp_len = 32;
	tuning_cmd.addr = (uintptr_t)buf;
	tuning_cmd.retry = 0;

	sdhci_write16(host, SDHCI_BLOCK_SIZE, SDHCI_MAKE_BLKSZ(7, 128));
	sdhci_write16(host, SDHCI_TRANSFER_MODE, SDHCI_TRNS_READ );

	ret = mmc_send_cmd(&tuning_cmd);
	if (ret)
		return EFI_DEVICE_ERROR;

	sdhci_write16(host, SDHCI_TRANSFER_MODE, SDHCI_TRNS_READ );

	start = timer_us(0);
	do
	{
		unsigned int intmask;
		intmask = sdhci_read16(host, SDHCI_INT_STATUS);
		if ((intmask & SDHCI_INT_DATA_AVAIL))
		{
			sdhci_write16(host, SDHCI_INT_STATUS, intmask & SDHCI_INT_DATA_AVAIL);
			return EFI_SUCCESS;
		}
	} while (timer_us(start) < 10 * 1000);

	return EFI_NOT_READY;
}

static EFI_STATUS sdhci_execute_tuning(struct mmc *m)
{
	struct sdhci *host = m->host;
	uint32_t ier1;
	uint32_t ier2;
	uint16_t ctrl = 0;
	int counter;
	int ret = 0;
	EFI_STATUS err = 0;

	ier1 = sdhci_read32(host, SDHCI_INT_ENABLE);
	ier2 = sdhci_read32(host, SDHCI_SIGNAL_ENABLE);

	sdhci_start_tuning(host);

	/*
	 * Issue TUNING_CMD repeatedly till Execute Tuning is set to 0 or the number
	 * of loops reaches 40 times.
	 */
	for (counter = 0; counter <= MAX_TUNING_LOOP; counter ++)
	{
		ret = sdhci_send_tuning(host, CMD_MMC_SEND_TUNING_BLOCK_HS200);
		if (ret != EFI_SUCCESS)
		{
			sdhci_reset_tuning(host);
			sdhci_reset(host, SDHCI_RESET_CMD);
			sdhci_reset(host, SDHCI_RESET_DATA);
			sdhci_abort_tuning(host);
			break;
		}

		ctrl = sdhci_read16(host, SDHCI_HOST_CTRL2);
		if ((ctrl & SDHCI_EXECUTE_TUNING) == 0)
			break;
	}

	if (!(ctrl & SDHCI_CLOCK_TUNED))
	{
		err = EFI_DEVICE_ERROR;
		printf("tuning failed, ctrl=0x%X\n", ctrl);
	}

	sdhci_write32(host, SDHCI_INT_ENABLE, ier1);
	sdhci_write32(host, SDHCI_SIGNAL_ENABLE, ier2);

	return err;
}


static void sdhci_reset(struct sdhci *host, uint8_t mask)
{
        uint64_t start = timer_us(0);

	sdhci_write8(host, SDHCI_SOFTWARE_RESET, mask);

	while (sdhci_read8(host, SDHCI_SOFTWARE_RESET))
	{
                if (timer_us(start) > ms(100))
		{
			ewerr("ERROR: Failed to reset controller");
			return ;
		}
	}
	return;
}

static void sdhci_set_voltage(struct sdhci *host, uint8_t power)
{
	uint8_t pwr = 0;

	/* Select voltage based on capability register */
	if (!power)
	{
		if (host->caps1 & SDHCI_CAPS_VS18)
			power = 18;
		else if (host->caps1 & SDHCI_CAPS_VS30)
			power = 30;
		else if (host->caps1 & SDHCI_CAPS_VS33)
			power = 33;
	}

	sdhci_write8(host, SDHCI_POWER_CONTROL, 0);

	switch (power)
	{
	case 33:
		pwr = SDHCI_POWER_ON | SDHCI_POWER_33V;
		break;
	case 30:
		pwr = SDHCI_POWER_ON | SDHCI_POWER_30V;
		break;
	case 18:
		pwr = SDHCI_POWER_ON | SDHCI_POWER_18V;
		break;
	default:
		break;
	}

	sdhci_write8(host, SDHCI_POWER_CONTROL, pwr);
	return;
}

static int sdhci_set_clock(struct sdhci *host, uint32_t freq)
{
	uint16_t div, clk;
	uint64_t start = timer_us(0);

	/* Use 400 Khz as an initialization frequency */
	if (freq == (uint32_t)-1)
		freq = 400;

	sdhci_write16(host, SDHCI_CLOCK_CONTROL, 0);

	if (host->f_max <= freq)
		div = 1;
	else
	{
		for (div = 2; div < SDHCI_MAX_DIV_SPEC_300; div += 2)
		{
			if ((host->f_max / div) <= freq)
				break;
		}
	}

	div >>= 1;

	clk = SDHCI_CLOCK_ENABLE | (div << SDHCI_DIVIDER_SHIFT);

	sdhci_write16(host, SDHCI_CLOCK_CONTROL, clk);
	while (!(sdhci_read16(host, SDHCI_CLOCK_CONTROL) & SDHCI_CLOCK_STABLE))
	{
		if (timer_us(start) > 100 * 1000)
			return 1;
	}

	clk |= SDHCI_CLOCK_CARD_ENABLE;
	sdhci_write16(host, SDHCI_CLOCK_CONTROL, clk);

	return 0;
}

static int init_controller(struct sdhci *host)
{
	/* Reset Controller */
	sdhci_reset(host, SDHCI_RESET_ALL);

	/* Set Minimum voltage */
	sdhci_set_voltage(host, 0);

	/* Set Clock to base frequency */
	if (sdhci_set_clock(host, -1) != 0)
	{
		ewerr("Error: Failed to set clock base frequency ");
		return 1;
	}

	sdhci_write8(host, SDHCI_TIMEOUT_CONTROL, 0xe);

	/*
	** Clear IRQ status, error status and
	** enable interrupts
	*/
	sdhci_write16(host, SDHCI_INT_STATUS, 0xffff);
	sdhci_write16(host, SDHCI_ERR_INT_STATUS, 0xffff);
	sdhci_write16(host, SDHCI_INT_ENABLE, SDHCI_INT_ALL);
	sdhci_write16(host, SDHCI_ERR_INT_ENABLE, 0xffff);

	return 0;
}

static int
sdhci_set_mode(struct mmc *m)
{
	struct sdhci *host = m->host;
	int err;

	err = sdhci_set_clock(host, m->freq);
	if (err)
	{
		ewerr("Failed to set clock to %d ", m->freq);
		return err;
	}

	uint16_t ctrl2 = sdhci_read16(host, SDHCI_HOST_CTRL2);
	ctrl2 &= ~SDHCI_UHS_MODE_SELECT;
	ctrl2 |= m->uhs_timing;
	sdhci_write16(host, SDHCI_HOST_CTRL2, ctrl2);

	uint8_t ctrl = sdhci_read8(host, SDHCI_HOST_CTRL);
	if (m->bus_width == 8)
	{
		ctrl &= ~SDHCI_WIDTH_4BITS;
		ctrl |=  SDHCI_WIDTH_8BITS;
	} else if (m->bus_width == 4)
	{
		ctrl &= ~SDHCI_WIDTH_8BITS;
		ctrl |=  SDHCI_WIDTH_4BITS;
	}

	if (m->freq > 25000)
		ctrl |= SDHCI_HS_ENABLE;

	sdhci_write8(host, SDHCI_HOST_CTRL, ctrl);

	return 0;
}

static uint16_t sdhci_make_cmd(struct cmd *c)
{
	int ret = c->flags & CMDF_DIRECT_MASK;

	switch (c->resp_len)
	{
	case 0:
		ret |= SDHCI_CMD_NO_RESP;
		break;
	case 128:
		ret |= SDHCI_CMD_RL136;
		break;
	case 32:
		if (c->flags & CMDF_BUSY_CHECK)
			ret |= SDHCI_CMD_RL48_CB;
		else
			ret |= SDHCI_CMD_RL48;
		break;
	default:
		break;
	}

	if (c->flags & CMDF_DATA_XFER)
		ret |= SDHCI_CMD_DATA_PRESENT;

	ret |= (c->index << SDHCI_CMD_INDEX_SHIFT);
	return ret;

}

static void
sdhci_wait_for_state_unset(struct sdhci *host, uint16_t bits, uint32_t timeout_us)
{
	uint16_t value;

	do {
		value = sdhci_read16(host, SDHCI_PRESENT_STATE);
		if (!(value & bits))
			break;
		udelay(1);
	} while (timeout_us-- > 0);

	if (!timeout_us)
		ewerr("%s() timeout for 0x%x", __func__, bits);
}

static void
sdhci_send_cmd(struct mmc *m, struct cmd *c)
{
	struct sdhci *host = m->host;
	uint16_t tmode = 0;

	sdhci_wait_for_state_unset(host, SDHCI_CMD_INHIBIT, 100000);

	if (c->index != CMD_MMC_SEND_TUNING_BLOCK_HS200)
		sdhci_wait_for_state_unset(host, SDHCI_DATA_INHIBIT, 180000);

	/* clear irq_status/err_sts register */
	sdhci_write32(host, SDHCI_INT_STATUS, 0xffffffff);

	if (c->flags & (CMDF_DATA_XFER | CMDF_BOOT_EN))
	{
		tmode  = (c->flags & CMDF_RD_XFER) ? TM_READ : TM_WRITE;

		if (c->flags & CMDF_USE_DMA)
			tmode |= TM_USE_DMA;

		if (c->nblock > 1)
			tmode |= (TM_MULTI_BLOCK | TM_BLOCK_CNT_ENABLE);

		sdhci_write16(host, SDHCI_BLOCK_CNT, c->nblock);
		sdhci_write32(host, SDHCI_DMA_ADDR, c->addr);

		if (c->index == 21)
			sdhci_write16(host, SDHCI_BLOCK_SIZE, 128 | DMA_128K_BOUNDRY);
		else
			sdhci_write16(host, SDHCI_BLOCK_SIZE, DEFAULT_BLOCK_SIZE | DMA_512K_BOUNDRY);

		sdhci_write16(host, SDHCI_TRANSFER_MODE, tmode);

		if (c->flags & CMDF_BOOT_EN)
		{
			sdhci_write32(host, SDHCI_BOOT_TIMEOUT_CTRL, 0xffff);
			sdhci_write8(host, SDHCI_BLOCK_GAP_CTRL, BOOT_EN | BOOT_ACK_RCV);
		}
	}
	sdhci_write32(host, SDHCI_ARGUMENT, c->args);
	sdhci_write16(host, SDHCI_CMD_REG, sdhci_make_cmd(c));

	return;
}

/* ======================================================================== */
/*
** Wait for interrupt, analize the response depending on the transfer type
*/
static int
sdhci_wait_cmd_done(struct mmc *m, struct cmd *c)
{
	struct sdhci *host = m->host;
	uint16_t nis;
        uint64_t start = timer_us(0);
	size_t i;

	/* Something went wrong if we do not get an interrupt in the first 100 ms */
	while (! (nis = sdhci_read16(host, SDHCI_INT_STATUS)))
	{
                if (timer_us(start) > 100 * 1000)
		{
			ewerr("CMD%d timeout err %x", c->index, sdhci_read16(host, SDHCI_ERR_INT_STATUS));
			goto exit;
		}
	}

	/* Handle commands that do not involve data transfer */
	if (!(c->flags & CMDF_DATA_XFER))
	{
		/* handle error case first */
		if (nis & SDHCI_INT_ERR_INT)
		{
			ewerr("%s() CMD%d Error Int asserted %x",
			      __func__, c->index, sdhci_read16(host, SDHCI_ERR_INT_STATUS));
			goto exit;
		}

		if (nis & SDHCI_INT_CMD_COMPLETE)
		{
			switch (c->resp_len)
			{
			case 128:
				for (i = 0; i < 4; i++)
				{
					c->resp[i] = sdhci_read32(host, SDHCI_RESPONSE + (3-i)*4)<<8;
					if (i != 3)
						c->resp[i] |= sdhci_read8(host, SDHCI_RESPONSE + (3-i)*4-1);
				}
				break;
			case 32:
				c->resp[0] = sdhci_read32(host, SDHCI_RESPONSE);
				break;
			default:
				break;
			}

			sdhci_write16(host, SDHCI_INT_STATUS, SDHCI_INT_CMD_COMPLETE);
		}
	}

	/* Handle Boot Partition ACK message */
	else if (c->flags & CMDF_BOOT_EN)
	{
		if (! (nis & SDHCI_INT_BOOT_ACK_RCV))
			goto exit;

		sdhci_write32(host, SDHCI_BOOT_TIMEOUT_CTRL, 0xffffffff);
		sdhci_write16(host, SDHCI_INT_STATUS, SDHCI_INT_BOOT_ACK_RCV);

		return 0;
	}

	/* Handle R/W transfers */
	else if (c->flags & CMDF_DATA_XFER)
	{
		/* Give slow SD cards a change to finish the transfer (4s) */
                start = timer_us(0);

		while (! (nis & SDHCI_INT_XFER_COMPLETE))
		{
			nis = sdhci_read16(host, SDHCI_INT_STATUS);

			if (timer_us(start) > ms(6000))
			{
				ewerr("CMD timeout");
				goto exit;
			}
			if (sdhci_read16(host, SDHCI_ERR_INT_STATUS))
			{
				ewerr("err_sts %x auto_cmd12 %x", sdhci_read16(host, SDHCI_ERR_INT_STATUS),
				      sdhci_read8(host, 0x3c));
				goto exit;
			}

			if (nis & SDHCI_INT_DMA_INT)
			{
				sdhci_write32(host, SDHCI_DMA_ADDR, sdhci_read32(host, SDHCI_DMA_ADDR));
				sdhci_write16(host, SDHCI_INT_STATUS, SDHCI_INT_DMA_INT);
			}

			if (nis & SDHCI_INT_BUFFER_READ_RDY)
			{
				sdhci_write16(host, SDHCI_INT_STATUS, SDHCI_INT_BUFFER_READ_RDY);
				return 0;
			}
		}

		sdhci_write16(host, SDHCI_INT_STATUS, SDHCI_INT_DMA_INT);
		sdhci_write16(host, SDHCI_INT_STATUS, SDHCI_INT_XFER_COMPLETE);
	}

	return 0;

exit:
	sdhci_reset(host, SDHCI_RESET_DATA | SDHCI_RESET_CMD);
	sdhci_write16(host, SDHCI_INT_STATUS, 0xffff);
	sdhci_write16(host, SDHCI_ERR_INT_STATUS, 0xffff);
	return 1;
}

static int sdhci_wait_boot_done(struct mmc *m, uintptr_t *dma_addr)
{
        unsigned res;
        uint16_t nis = sdhci_read16(m->host, SDHCI_INT_STATUS);
        unsigned sdma_addr = 0;

        res = nis & SDHCI_INT_BOOT_TERM;

        if (nis & SDHCI_INT_DMA_INT)
        {
                sdma_addr = sdhci_read32(m->host, SDHCI_DMA_ADDR);
                sdhci_write32(m->host, SDHCI_DMA_ADDR, sdma_addr);
                sdhci_write16(m->host, SDHCI_INT_STATUS, SDHCI_INT_DMA_INT);
        }

        *dma_addr = sdma_addr;

        return res;
}

static void sdhci_boot_stop(struct mmc *m)
{
	uint8_t gap_ctrl;

	gap_ctrl = sdhci_read8(m->host, SDHCI_BLOCK_GAP_CTRL);
	sdhci_write8(m->host, SDHCI_BLOCK_GAP_CTRL, gap_ctrl & ~BOOT_EN);
        sdhci_write16(m->host, SDHCI_INT_STATUS, 0xffff);
        sdhci_write16(m->host, SDHCI_ERR_INT_STATUS, 0xffff);

        sdhci_reset(m->host, SDHCI_RESET_DATA | SDHCI_RESET_CMD);
}

static struct sdhci host;

struct sdhci *sdhci_find_controller(pcidev_t dev)
{
        uint32_t addr;

	host.init_controller = init_controller;
	host.send_cmd        = sdhci_send_cmd;
	host.wait_cmd_done   = sdhci_wait_cmd_done;
	host.wait_boot_done  = sdhci_wait_boot_done;
	host.boot_stop       = sdhci_boot_stop;
	host.set_mode        = sdhci_set_mode;
	host.execute_tuning  = sdhci_execute_tuning;

	addr = pci_read_config32(dev, PCI_BASE_ADDRESS_0);
        host.ioaddr = (addr & ~0xf);

	/*
	** Discover controller capabilities
	*/
	host.voltage = 0;
	host.caps1 = sdhci_read32(&host, SDHCI_CAPABILITIES);
	host.caps2 = sdhci_read32(&host, SDHCI_CAPABILITIES + 4);

	host.f_max = (host.caps1 & SDHCI_CLOCK_V3_BASE_MASK)
		>> SDHCI_CLOCK_BASE_SHIFT;

	host.f_max *= 1000;

	if (host.f_max == 0) {
		ewerr("%s: Hardware doesn't specify base clock frequency",
		       __func__);
	}

	sdhci_reset(&host, SDHCI_RESET_ALL);

	return &host;
}
