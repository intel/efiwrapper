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

#ifndef _BLOCKDEV_H_
#define _BLOCKDEV_H_

typedef uint32_t lba_t;

typedef struct blockdev_ops
{
	lba_t (*read)(lba_t start, lba_t count,
			const void *buffer);
	lba_t (*write)(lba_t start, lba_t count,
			const void *buffer);
	lba_t block_count;
	lba_t block_max;
	uint32_t block_size;
} blockdev_ops;

extern blockdev_ops* get_block_ops(void);
#endif
