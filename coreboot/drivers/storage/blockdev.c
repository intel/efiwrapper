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

#include <libpayload.h>

#include "blockdev.h"
#include "mmc.h"
#include "sdhci-bxt.h"

static struct blockdev_ops bdev_ops;

static int mmc_transfer_data(unsigned blockno, int read, uint32_t nblock, void *addr)
{
	struct cmd c;

	c.args     = blockno;
	c.nblock   = nblock;
	c.addr     = (uintptr_t) addr;
	c.resp_len = 32;
	c.flags    = CMDF_DATA_XFER | CMDF_USE_DMA;

	if (read == 0)
	{
		c.flags |= CMDF_WR_XFER;
		c.index = nblock > 1 ? CMD_WRITE_MULTIPLE_BLOCKS : CMD_WRITE_SINGLE_BLOCK;
	}
	else
	{
		c.flags |= CMDF_RD_XFER;
		c.index = nblock > 1 ? CMD_READ_MULTIPLE_BLOCKS : CMD_READ_SINGLE_BLOCK;
	}

	mmc_send_cmd(&c);

	return 0;
}

static int mmc_transfer_finish(void)
{
	struct cmd c;

	c.flags = CMDF_DATA_XFER;

	return mmc_wait_cmd_done(&c);
}

/* Generic Block Dev Read/Write APIs */
static lba_t blockio_rw(int reading, lba_t Blockno, uint32_t BufferSize, const void* Buffer)
{
	lba_t start = Blockno;
	lba_t num;
	uint32_t size = BufferSize;
	uint8_t *ptr = (uint8_t *)Buffer;
	int ret;

	do {
		if (size > bdev_ops.block_max)
			num = bdev_ops.block_max;
		else
			num = size;
		mmc_transfer_data(start, reading, num, ptr);
		if ((ret = mmc_transfer_finish()) != 0)
			return ret;
		start += num;
		ptr += num * bdev_ops.block_size;
		size -= num;
	} while (size > 0);

	return BufferSize;
}

lba_t blockio_read(lba_t Blockno, uint32_t BufferSize, const void* Buffer)
{
	return blockio_rw(1, Blockno, BufferSize, Buffer);
}

lba_t blockio_write(lba_t Blockno, uint32_t BufferSize, const void* Buffer)
{
	return blockio_rw(0, Blockno, BufferSize, Buffer);
}

blockdev_ops* get_block_ops(void)
{
	bdev_ops.read = &blockio_read;
	bdev_ops.write = &blockio_write;
	bdev_ops.block_count = mmc_read_count();
	bdev_ops.block_max = 65535;
	bdev_ops.block_size = DEFAULT_BLOCK_SIZE;

	return &bdev_ops;
}
