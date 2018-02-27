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
#include <smbios.h>
#include <ewarg.h>
#include <ewvar.h>
#include <libsmbios.h>

#include "abl/abl.h"

/* ABL hwver structure is defined at follow:
 * struct hwver {
 *         uint8_t cpu; // Cpu stepping
 *         uint8_t cpuNcores;
 *         uint16_t cpuFreq;
 *         uint8_t sku;
 *         uint8_t resetCause;
 *         uint16_t platformId;
 *         uint16_t moduleId;
 * };
 *
 * This structure is serialized as the "ABL.hwver" command line
 * parameter built as follow:
 * "$cpu,$cpuNcores,$cpuFreq,$sku,$resetCause,$platformId,$moduleId"
 */
static const char *get_cpu_stepping(const char *str)
{
	static char out[4];
	unsigned long stepping;
	char *data, *endptr;
	int ret;

	data = strdup(str);
	if (!data)
		return SMBIOS_UNDEFINED;

	data = strtok(data, ",");
	if (!data)
		return SMBIOS_UNDEFINED;

	stepping = strtoul(data, &endptr, 16);
	free(data);
	if (*endptr != '\0')
		return SMBIOS_UNDEFINED;

	if (stepping > 0xff)
		return SMBIOS_UNDEFINED;

	ret = snprintf(out, sizeof(out), "%lx", stepping);
	if (ret < 0)
		return SMBIOS_UNDEFINED;

	return out;
}

static const char *identity(const char *str)
{
	return str;
}

static const struct {
	const char *name;
	UINT8 type;
	UINT8 offset;
	const char *(*convert)(const char *);
} ARG_TO_SMBIOS[] = {
	{ "androidboot.serialno", 1, offsetof(SMBIOS_TYPE1, SerialNumber), identity },
	{ "ABL.version", 0, offsetof(SMBIOS_TYPE0, BiosVersion), identity },
	{ "androidboot.name", 1, offsetof(SMBIOS_TYPE1, ProductName), identity },
	{ "androidboot.brand", 2, offsetof(SMBIOS_TYPE2, Manufacturer), identity },
	{ "androidboot.device", 2, offsetof(SMBIOS_TYPE2, ProductName), identity },
	{ "ABL.hwver", 1, offsetof(SMBIOS_TYPE1, Version), get_cpu_stepping }
};

static EFI_STATUS set_smbios_fields(void)
{
	EFI_STATUS ret;
	size_t i;
	const char *val;

	for (i = 0; i < ARRAY_SIZE(ARG_TO_SMBIOS); i++) {
		val = ewarg_getval(ARG_TO_SMBIOS[i].name);
		if (!val)
			continue;

		ret = smbios_set(ARG_TO_SMBIOS[i].type,
				 ARG_TO_SMBIOS[i].offset,
				 ARG_TO_SMBIOS[i].convert(val));
		if (EFI_ERROR(ret))
			return ret;
	}

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
abl_exit_bs(__attribute__((__unused__)) EFI_HANDLE ImageHandle,
	    __attribute__((__unused__)) UINTN MapKey)
{
	/* On ABL platform there is no Boot state. */
	return EFI_SUCCESS;
}

extern ewvar_storage_t reboot_target_storage;
static EFI_EXIT_BOOT_SERVICES saved_exit_bs;

static EFI_STATUS abl_init(EFI_SYSTEM_TABLE *st)
{
	if (!st)
		return EFI_INVALID_PARAMETER;

	ewvar_register_storage(&reboot_target_storage);
	saved_exit_bs = st->BootServices->ExitBootServices;
	st->BootServices->ExitBootServices = abl_exit_bs;

	return set_smbios_fields();
}

static EFI_STATUS abl_exit(EFI_SYSTEM_TABLE *st)
{
	if (!st)
		return EFI_INVALID_PARAMETER;

	ewvar_unregister_storage();
	st->BootServices->ExitBootServices = saved_exit_bs;

	return EFI_SUCCESS;
}

ewdrv_t abl_drv = {
	.name = "abl",
	.description = "Automotive BootLoader support",
	.init = abl_init,
	.exit = abl_exit
};
