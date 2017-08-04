/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 *
 * Author: Jérémy Compostella <jeremy.compostella@intel.com>
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


#include <arch/io.h>
#include <kconfig.h>
#include <libpayload-config.h>
#include <libpayload.h>

#include "cf9/cf9.h"

static EFIAPI EFI_STATUS
cf9_reset_system(EFI_RESET_TYPE ResetType,
		 __attribute__((__unused__)) EFI_STATUS ResetStatus,
		 __attribute__((__unused__)) UINTN DataSize,
		 __attribute__((__unused__)) CHAR16 *ResetData)
{
	UINT8 code;
	UINT8 cf9;
	UINT32 port = 0xcf9;

	if (ResetType == EfiResetShutdown)
		return EFI_UNSUPPORTED;

	switch (ResetType) {
	case EfiResetWarm:
		code = 0x06;
		break;

	case EfiResetCold:
		code = 0x0E;
		break;

	case EfiResetShutdown:
	default:
		return EFI_UNSUPPORTED;
	}

	cf9 = inb(port) & ~code;
	outb(cf9 | 2, port);
	udelay(50);

	outb(cf9 | code, port);
	udelay(500);

	return EFI_DEVICE_ERROR;
}

static EFI_RESET_SYSTEM saved_reset_rs;

static EFI_STATUS cf9_init(EFI_SYSTEM_TABLE *st)
{
	if (!st)
		return EFI_INVALID_PARAMETER;

	saved_reset_rs = st->RuntimeServices->ResetSystem;
	st->RuntimeServices->ResetSystem = cf9_reset_system;

	return EFI_SUCCESS;
}

static EFI_STATUS cf9_exit(EFI_SYSTEM_TABLE *st)
{
	if (!st)
		return EFI_INVALID_PARAMETER;

	st->RuntimeServices->ResetSystem = saved_reset_rs;

	return EFI_SUCCESS;
}

ewdrv_t cf9_drv = {
	.name = "cf9",
	.description = "Provide reset support based on CF9 IO port",
	.init = cf9_init,
	.exit = cf9_exit
};

