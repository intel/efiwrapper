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

#include "regaccess.h"


// ========================================================================
/*
**  MMIO access functions:
*/

unsigned __attribute__((stdcall, regparm(1)))
_DO_mmio8_SET_FIELDS(uintptr_t mmio_addr, unsigned mask, unsigned val)
{
	volatile uint8_t *addr = (void *)mmio_addr;
	unsigned res  = *addr;

	*addr = res = (res & ~mask) | (val & mask);
	__RW_BARRIER();

	return res;
}

unsigned __attribute__((stdcall, regparm(1)))
_DO_mmio16_SET_FIELDS(uintptr_t mmio_addr, unsigned mask, unsigned val)
{
	volatile uint16_t *addr = (void *)mmio_addr;
	unsigned res  = *addr;

	*addr = res = (res & ~mask) | (val & mask);
	__RW_BARRIER();

	return res;
}

unsigned __attribute__((stdcall, regparm(1)))
_DO_mmio32_SET_FIELDS(uintptr_t mmio_addr, unsigned mask, unsigned val)
{
	volatile uint32_t *addr = (void *)mmio_addr;
	unsigned res  = *addr;

	*addr = res = (res & ~mask) | (val & mask);
	__RW_BARRIER();

	return res;
}
