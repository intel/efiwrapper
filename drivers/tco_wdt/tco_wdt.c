/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 *
 * Author: Baofeng, Tian <baofeng.tian@intel.com>
 * Author: Xinanx, Luo <xinanx.luo@intel.com>
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

#include <hwconfig.h>
#include <interface.h>
#include <kconfig.h>
#include <libpayload-config.h>
#include <libpayload.h>

#include "tco_wdt/tco_protocol.h"
#include "tco_wdt/tco_wdt.h"

#ifndef BIT
#define BIT(x) (1 << (x))
#endif

/* TCO Registers */
#define TCO_RLD		(TCOBASE + 0x00) /* TCO Timer Reload/Curr. Value */
#define TCO1_STS		(TCOBASE + 0x04) /* TCO1 Status Register	*/
#define TCO2_STS		(TCOBASE + 0x06) /* TCO2 Status Register	*/
#define TCO1_CNT	(TCOBASE + 0x08) /* TCO1 Control Register	*/
#define TCO2_CNT	(TCOBASE + 0x0a) /* TCO1 Control Register	*/
#define TCOv2_TMR	(TCOBASE + 0x12) /* TCOv2 Timer Initial Value*/

#define PMC_GCR_PMC_CFG_REG	(0xfe043008) /*NO_REBOOT bit config reg*/

/* TCO Registers' bits */
#define TCO_HALT_BIT		BIT(11)
#define TCO_TIMEOUT1_BIT	BIT(3)
#define TCO_TIMEOUT2_BIT	BIT(2)
#define NO_REBOOT_BIT		BIT(4)

#define TCO_MIN_TIMEOUT 4

static EFI_GUID tco_wdt_guid = EFI_TCO_RESET_PROTOCOL_GUID;
static EFI_HANDLE handle;

static EFIAPI EFI_STATUS tco_wdt_enable (UINT32 *timeout)
{
	UINT16 val = 0;
	UINT32 tmp = 0;

	if (NULL == timeout)
		return EFI_INVALID_PARAMETER;
	if (*timeout < TCO_MIN_TIMEOUT)
		*timeout = TCO_MIN_TIMEOUT;

	/* Halt TCO */
	val = inw(TCO1_CNT);
	val |= TCO_HALT_BIT;
	outw(val, TCO1_CNT);
	val = inw(TCO1_CNT);

	/* Clear STS */
	outw(TCO_TIMEOUT1_BIT, TCO1_STS);
	outw(TCO_TIMEOUT2_BIT, TCO2_STS);

	/* Set timeout */
	val = inw(TCOv2_TMR);
	val = (val & 0xfc00) | (((*timeout * 10) / 6) & 0x3ff);
	outw((val & 0x3ff), TCOv2_TMR);
	val = inw(TCOv2_TMR);

	/* Clear NO_REBOOT bit */
	tmp = read32((void *)(UINTN)PMC_GCR_PMC_CFG_REG);
	tmp &= ~ NO_REBOOT_BIT;
	write32((void *)(UINTN)PMC_GCR_PMC_CFG_REG, tmp);
	/*Read back and check */
	tmp = read32((void *)(UINTN)PMC_GCR_PMC_CFG_REG);
	if (tmp & NO_REBOOT_BIT)
		return EFI_DEVICE_ERROR;

	/* Reload */
	outw(0x1, TCO_RLD);

	/* Start TCO */
	val = inw(TCO1_CNT);
	val &= ~ TCO_HALT_BIT;
	outw(val, TCO1_CNT);
	val = inw(TCO1_CNT);
	if (val & TCO_HALT_BIT)
		return EFI_DEVICE_ERROR;

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS tco_wdt_disable (UINT32 timeout)
{
	UINT16 val = 0;
	UINT32 tmp = timeout;

	/* Halt TCO */
	val = inw(TCO1_CNT);
	val |= TCO_HALT_BIT;
	outw(val, TCO1_CNT);
	val = inw(TCO1_CNT);
	if (!(val & TCO_HALT_BIT))
		return EFI_DEVICE_ERROR;

	/* Set NO_REBOOT bit */
	tmp = read32((void *)(UINTN)PMC_GCR_PMC_CFG_REG);
	tmp |= NO_REBOOT_BIT;
	write32((void *)(UINTN)PMC_GCR_PMC_CFG_REG, tmp);
	/*Read back and check */
	tmp = read32((void *)(UINTN)PMC_GCR_PMC_CFG_REG);
	if (!(tmp & NO_REBOOT_BIT))
		return EFI_DEVICE_ERROR;

	return EFI_SUCCESS;
}
static EFI_STATUS tco_wdt_init(EFI_SYSTEM_TABLE *st)
{
	static EFI_TCO_RESET_PROTOCOL tco_wdt_default = {
		.EnableTcoReset = tco_wdt_enable,
		.DisableTcoReset = tco_wdt_disable
	};
	EFI_TCO_RESET_PROTOCOL *tco_wdt;

	return interface_init(st, &tco_wdt_guid, &handle,
			      &tco_wdt_default, sizeof(tco_wdt_default),
			      (void **)&tco_wdt);
}

static EFI_STATUS tco_wdt_exit(EFI_SYSTEM_TABLE *st)
{
	return interface_free(st, &tco_wdt_guid, handle);
}

ewdrv_t tco_wdt_drv = {
	.name = "tco_wdt",
	.description = "TCO watchdog Protocol",
	.init = tco_wdt_init,
	.exit = tco_wdt_exit
};
