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

#ifndef _REGACCESS_H
#define _REGACCESS_H

#include <stdint.h>

/* ======================================================================== */

//
//  Tell the compiler to not move memory instructions accross this barrier.
//
#define __RW_BARRIER()  asm volatile("" ::: "memory")

//
//  Tell the CPU to not move loads and stores accross this barrier.
//  [Note that this is hardly ever necessary ...]
//
#define __MEM_BARRIER() asm volatile("mfence" ::: "memory")

/* ======================================================================== */

#define _INLINE inline __attribute__((gnu_inline, always_inline, unused, artificial))

// ========================================================================
//
//  Memory-mapped I/O -- addressed by(base + offset)
//

static unsigned _INLINE mmio8(uintptr_t base, unsigned offs);
static unsigned _INLINE mmio16(uintptr_t base, unsigned offs);
static unsigned _INLINE mmio32(uintptr_t base, unsigned offs);

static void _INLINE mmio8_SET(uintptr_t base, unsigned offs, unsigned val);
static void _INLINE mmio16_SET(uintptr_t base, unsigned offs, unsigned val);
static void _INLINE mmio32_SET(uintptr_t base, unsigned offs, unsigned val);

static void _INLINE mmio8_SET_BITS(uintptr_t base, unsigned offs, unsigned bits);
static void _INLINE mmio16_SET_BITS(uintptr_t base, unsigned offs, unsigned bits);
static void _INLINE mmio32_SET_BITS(uintptr_t base, unsigned offs, unsigned bits);

static void _INLINE mmio8_CLR_BITS(uintptr_t base, unsigned offs, unsigned bits);
static void _INLINE mmio16_CLR_BITS(uintptr_t base, unsigned offs, unsigned bits);
static void _INLINE mmio32_CLR_BITS(uintptr_t base, unsigned offs, unsigned bits);

#define EC_BASE 0xE0000000
#define BDF_(b, d, f)((pci_device_t)(EC_BASE | ((b) << 20) | ((d) << 15) | ((f) << 12)))


#include "regaccess_impl.h"

#endif  /* _REGACCESS_H */
