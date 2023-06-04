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

#include <kconfig.h>
#include <libpayload-config.h>
#include <libpayload.h>

#include "lprtc/lprtc.h"

static EFIAPI EFI_STATUS
lprtc_get_time(EFI_TIME *Time,
	       __attribute__((__unused__)) EFI_TIME_CAPABILITIES *Capabilities)
{
	struct tm now;

	if (!Time)
		return EFI_INVALID_PARAMETER;

	rtc_read_clock(&now);

	memset(Time, 0, sizeof(*Time));//NOLINT
	Time->Year = now.tm_year + 1900;
	Time->Month = now.tm_mon + 1;
	Time->Day = now.tm_mday;
	Time->Hour = now.tm_hour;
	Time->Minute = now.tm_min;
	Time->Second = now.tm_sec;
	Time->TimeZone = EFI_UNSPECIFIED_TIMEZONE;

	return EFI_SUCCESS;
}

static EFI_GET_TIME saved_gettime_rs;

static EFI_STATUS lprtc_init(EFI_SYSTEM_TABLE *st)
{
	if (!st)
		return EFI_INVALID_PARAMETER;

	saved_gettime_rs = st->RuntimeServices->GetTime;
	st->RuntimeServices->GetTime = lprtc_get_time;

	return EFI_SUCCESS;
}

static EFI_STATUS lprtc_exit(EFI_SYSTEM_TABLE *st)
{
	if (!st)
		return EFI_INVALID_PARAMETER;

	st->RuntimeServices->GetTime = saved_gettime_rs;

	return EFI_SUCCESS;
}

ewdrv_t lprtc_drv = {
	.name = "lprtc",
	.description = "Provide the GetTime runtime service support based \
on the libpayload rtc_read_clock() function.",
	.init = lprtc_init,
	.exit = lprtc_exit
};
