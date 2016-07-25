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
# error "Don't include <regaccess_impl.h> directly; use <regaccess.h> instead!"
#endif

// ========================================================================
// Implementation / optimization knobs:

#define _INLINE_MMMIO_FIELD_OPS           0
#define _INLINE_IOPORT_FIELD_OPS          1
#define _ALLOW_NON_CONSTANT_MSGBUS_PORTS  1

/* ======================================================================== */
/*
**  Memory-mapped I/O registers:
**
**  TODO:
**  - BIS does not generate "OR  [base+off], bits" -- it's using EAX as a temp.
**  - BIC likewise ...
*/

#define __DEFINE_MMIO_FUNC_GET_(size)                                   \
                                                                        \
        unsigned _INLINE                                                \
        mmio ## size(uintptr_t base, unsigned off)                     \
        {                                                               \
                return *(volatile uint ## size ##_t *)(base + off);   \
        }

__DEFINE_MMIO_FUNC_GET_(32)
__DEFINE_MMIO_FUNC_GET_(16)
__DEFINE_MMIO_FUNC_GET_(8)


#define __DEFINE_MMIO_FUNC_SET_(size)                                   \
                                                                        \
        void _INLINE                                                    \
        mmio ## size ## _SET(uintptr_t base, unsigned off, unsigned val) \
        {                                                               \
                *(volatile uint ## size ##_t *)(base + off) = val;    \
                __RW_BARRIER();                                         \
        }

__DEFINE_MMIO_FUNC_SET_(32)
__DEFINE_MMIO_FUNC_SET_(16)
__DEFINE_MMIO_FUNC_SET_(8)


#define __DEFINE_MMIO_FUNC_BIS_(size)                                   \
                                                                        \
        void _INLINE                                                    \
        mmio ## size ## _SET_BITS(uintptr_t base, unsigned off, unsigned bits) \
        {                                                               \
                *(volatile uint ## size ##_t *)(base + off) |= bits;  \
                __RW_BARRIER();                                         \
        }

__DEFINE_MMIO_FUNC_BIS_(32)
__DEFINE_MMIO_FUNC_BIS_(16)
__DEFINE_MMIO_FUNC_BIS_(8)


#define __DEFINE_MMIO_FUNC_BIC_(size)                                   \
                                                                        \
        void _INLINE                                                    \
        mmio ## size ## _CLR_BITS(uintptr_t base, unsigned off, unsigned bits) \
        {                                                               \
                *(volatile uint ## size ##_t *)(base + off) &= ~bits; \
                __RW_BARRIER();                                         \
        }

__DEFINE_MMIO_FUNC_BIC_(32)
__DEFINE_MMIO_FUNC_BIC_(16)
__DEFINE_MMIO_FUNC_BIC_(8)


#if _INLINE_MMMIO_FIELD_OPS
#define __DEFINE_MMIO_FUNC_SETF_(size)                                  \
                                                                        \
        unsigned _INLINE                                                \
        mmio ## size ## _SET_FIELDS(uintptr_t base, unsigned off, unsigned mask, unsigned val) \
        {                                                               \
                volatile uint ## size ##_t *addr = (void *)(base + off); \
                unsigned res = *addr;                                   \
                *addr = res = (res & ~mask) | (val & mask);             \
                __RW_BARRIER();                                         \
                return res;                                             \
        }
#else
#define __DEFINE_MMIO_FUNC_SETF_(size)                                  \
                                                                        \
        extern unsigned __attribute__((stdcall, regparm(1)))            \
        _DO_mmio ## size ## _SET_FIELDS(uintptr_t addr, unsigned mask, unsigned val); \
                                                                        \
        unsigned _INLINE                                                \
        mmio ## size ## _SET_FIELDS(uintptr_t base, unsigned off, unsigned mask, unsigned val) \
        {                                                               \
                unsigned res = _DO_mmio ## size ## _SET_FIELDS(base + off, mask, val); \
                __RW_BARRIER();                                         \
                return res;                                             \
        }
#endif
