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

#ifndef __HW_PCI_UART_H__
#define __HW_PCI_UART_H__

#include <stdint.h>
#include <arch/io.h>

#define PCI_COMMAND_OFFSET              0x04
#define EFI_PCI_COMMAND_MEMORY_SPACE    (1<<1)

#define DEFAULT_PCI_BUS_NUMBER_PCH                0
#define PCI_DEVICE_NUMBER_PCH_SERIAL_IO_UART2     25
#define R_PCH_SERIAL_IO_BAR0_LOW                  0x10
#define B_PCH_SERIAL_IO_BAR0_LOW_BAR              0xFFFFF000

#define PCI_LIB_ADDRESS(Bus,Device,Function,Register)   \
   (((Register) & 0xfff) | (((Function) & 0x07) << 12) | (((Device) & 0x1f) << 15) | (((Bus) & 0xff) << 20))


static inline uint32_t GetPciUartBase (uint32_t PciBaseAddr, uint32_t UartNo)
{
    uint32_t  PciUartBase = 0;
    uint32_t  PciUartMmBase;
    uint16_t  Cmd16;

    PciUartMmBase = PCI_LIB_ADDRESS (
                        DEFAULT_PCI_BUS_NUMBER_PCH,
                        PCI_DEVICE_NUMBER_PCH_SERIAL_IO_UART2,
                        UartNo,
                        0);

    PciUartMmBase += PciBaseAddr;
    Cmd16 = read16((void *)(PciUartMmBase + PCI_COMMAND_OFFSET));
    if (Cmd16 != 0xFFFF) {
        if (read8((void *)(PciUartMmBase + PCI_COMMAND_OFFSET)) & EFI_PCI_COMMAND_MEMORY_SPACE)
            PciUartBase = read32((void *)(PciUartMmBase + R_PCH_SERIAL_IO_BAR0_LOW)) & B_PCH_SERIAL_IO_BAR0_LOW_BAR;
    }

    return PciUartBase;
}

#endif /* __HW_PCI_UART_H__ */

