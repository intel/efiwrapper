/*
 * Copyright (c) 2017, Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer
 *      in the documentation and/or other materials provided with the
 *      distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef __HECI_IMPL__
#define __HECI_IMPL__

#include <stdint.h>
#include <arch/io.h>

/* Rom Base Address in Bridge, defined in PCI-to-PCI Bridge Architecure Specification */
#define PCI_COMMAND_OFFSET            0x04

/* PCI devices, addressed by (bus, device, function + register offset) */
#define EC_BASE            0xE0000000
#define PCI_BDF(b, d, f)   ((void *)(EC_BASE | ((b) << 20) | ((d) << 15) | ((f) << 12)))

#define HECI_PCI_DEV       PCI_BDF(HECI_BUS, HECI_DEVICE_NUMBER, HECI_FUNCTION_NUMBER)


/* HECI registers */
/* H_CB_WW - Host Circular Buffer (CB) Write Window register */
#define H_CB_WW                         0x00
/* H_CSR - Host Control Status register */
#define H_CSR                           0x04
/* ME_CB_RW - ME Circular Buffer Read Window register (read only) */
#define SEC_CB_RW                       0x08
/* ME_CSR_HA - ME Control Status Host Access register (read only) */
#define SEC_CSR_HA                      0x0C

/* PCI access functions for SEC access */
#define HECI_MBAR0                      0x10
#define HECI_MBAR1                      0x14

#define HECI_BUS                        0
#define HECI_DEVICE_NUMBER              15
#define HECI_FUNCTION_NUMBER            0x00

#define HECI_HOST_ADDR                  0

#define HECI_SEC_FW_STS0                0x40


#define EFI_PCI_COMMAND_MEMORY_SPACE    (1<<1)
#define EFI_PCI_COMMAND_BUS_MASTER      (1<<2)
#define EFI_PCI_COMMAND_SERR            (1<<8)


#define HECI_RESET_TIMEOUT     8000  /* ms */
#define HECI_EVENT_TIMEOUT     3000  /* ms */


/* S_CSR - SEC Control Status */
typedef union {
	uint32_t data;
	struct {
		uint32_t  SEC_IE_HRA : 1;   /* 0 - SEC Interrupt Enable */
		uint32_t  SEC_IS_HRA : 1;   /* 1 - SEC Interrupt Status */
		uint32_t  SEC_IG_HRA : 1;   /* 2 - SEC Interrupt Generate */
		uint32_t  SEC_RDY_HRA : 1;  /* 3 - SEC Ready  */
		uint32_t  SEC_RST_HRA : 1;  /* 4 - SEC Reset  */
		uint32_t  reserved : 3;     /* 7:5 */
		uint32_t  SEC_CBRP_HRA : 8; /* 15:8 - SEC CB Read Pointer */
		uint32_t  SEC_CBWP_HRA : 8; /* 23:16 - SEC CB Write Pointer */
		uint32_t  SEC_CBD_HRA : 8;  /* 31:24 - SEC Circular Buffer Depth */
	} bit;
} DEV_CTRL_REG;

/* H_CSR - Host Control Status */
typedef union {
	uint32_t  data;
	struct {
		uint32_t  H_IE : 1;     /* 0 - Host Interrupt Enable SEC */
		uint32_t  H_IS : 1;     /* 1 - Host Interrupt Status SEC */
		uint32_t  H_IG : 1;     /* 2 - Host Interrupt Generate */
		uint32_t  H_RDY : 1;    /* 3 - Host Ready */
		uint32_t  H_RST : 1;    /* 4 - Host Reset */
		uint32_t  reserved : 3; /* 7:5 */
		uint32_t  H_CBRP : 8;   /* 15:8 - Host CB Read Pointer */
		uint32_t  H_CBWP : 8;   /* 23:16 - Host CB Write Pointer */
		uint32_t  H_CBD : 8;    /* 31:24 - Host Circular Buffer Depth */
	} bit;
} HOST_CTRL_REG;

typedef union {
	uint32_t  data;
	struct {
		/* logical address of the TXE client of the message */
		uint32_t  sec_address : 8;
		/* logical address of the Host client of the message. */
		uint32_t  host_address : 8;
		/* message length in bytes, not including header */
		uint32_t  length : 9;
		uint32_t  reserved : 6;
		/* indicate the last message of HECI transfer */
		uint32_t  message_complete : 1;
	} bit;
} HECI_MSG_HDR;

#endif	/* __HECI_IMPL__ */

