/*
 * Copyright (c) 2018, Intel Corporation
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

#ifndef __HW_TIGERLAKE__
#define __HW_TIGERLAKE__

/* PCI device id of OTG */
#define XDCI_PID         0x9D30
#define XHCI_PID         0x9D2F

/* PCI device id of EMMC controller */
#define EMMC_DEVICEID    0xA0C4

/* UFS */
#define UFS_PCI_DID    0xA0FA

/* NVME */
#define NVME_PCI_DID    0x0953
#define NVME_DISKBUS    0x1C04

/* serial port base address */
#ifndef EFIWRAPPER_USE_EC_UART
#define SERIAL_PCI_DID        0xA0c7
#define HW_SERIAL_TYPE        CB_SERIAL_TYPE_MEMORY_MAPPED
#define HW_SERIAL_REG_WIDTH   4

#else /* EFIWRAPPER_USE_EC_UART */

#define SERIAL_BASEADDR       0x3f8
#define HW_SERIAL_TYPE        CB_SERIAL_TYPE_IO_MAPPED
#define HW_SERIAL_REG_WIDTH   1

#endif /* EFIWRAPPER_USE_EC_UART */

/* TCO base address, to be determined */
#define TCOBASE    (0xffffffff)

#endif /* __HW_TIGERLAKE__ */

