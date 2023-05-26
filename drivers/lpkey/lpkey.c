/*
 * Copyright (c) 2020, Intel Corporation
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
#include <ewlog.h>

#include "lpkey/lpkey.h"

static EFI_CHECK_EVENT saved_check_event;
static EFI_EVENT wait_for_key;

static EFIAPI EFI_STATUS
lpkey_check_event(EFI_EVENT Event)
{
	if (Event != wait_for_key)
		return saved_check_event(Event);

	return havekey() ? EFI_SUCCESS : EFI_NOT_READY;
}

static EFI_GUID guid = SIMPLE_TEXT_OUTPUT_PROTOCOL;

static EFI_STATUS lpkey_init(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;
	SIMPLE_INPUT_INTERFACE *input;

	if (!st)
		return EFI_INVALID_PARAMETER;

	ret = st->BootServices->LocateProtocol(&guid, NULL, (VOID **)&input);
	if (EFI_ERROR(ret)) {
		ewerr("Could not locate simple text input protocol");
		return ret;
	}

	wait_for_key = input->WaitForKey;
	saved_check_event = st->BootServices->CheckEvent;
	st->BootServices->CheckEvent = lpkey_check_event;

	return EFI_SUCCESS;
}


static EFI_STATUS lpkey_exit(EFI_SYSTEM_TABLE *st)
{
	if (!st)
		return EFI_INVALID_PARAMETER;

	st->BootServices->CheckEvent = saved_check_event;

	return EFI_SUCCESS;
}

ewdrv_t lpkey_drv = {
	.name = "lpkey",
	.description = "Provide key event support based on libpayload havekey()",
	.init = lpkey_init,
	.exit = lpkey_exit
};

