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

#include <efi.h>
#include <kconfig.h>
#include <libpayload-config.h>
#include <libpayload.h>
#include <libsmbios.h>
#include <smbios.h>
#include <stdint.h>

#include "mmc_serial/mmc_serial.h"

struct cid {
	uint8_t mid: 8;
	uint8_t reserved: 6;
	uint8_t cbx: 2;
	uint8_t oid: 8;
	char pnm[6];
	uint8_t prv;
	uint32_t psn;
	uint8_t mdt;
	uint8_t crc: 7;
	uint8_t unused: 1;
} __attribute__((packed));

/* Must be provided by a mmc driver */
extern int mmc_cid(uint8_t cid[16]);

/* The serial number is the concatenation of the CID PNM and PSN
 * fields. */
static char *build_serial(void)
{
	static char serial[15];
	struct cid cid;
	size_t i;
	uint32_t *tmp = (uint32_t *)&cid;
	int ret;

	ret = mmc_cid((uint8_t *)&cid);
	if (ret)
		return NULL;

	for (i = 0; i < sizeof(cid) / sizeof(*tmp); i++)
		tmp[i] = __builtin_bswap32(tmp[i]);

	memcpy(&serial[0], cid.pnm, sizeof(cid.pnm));
	snprintf(&serial[6], 9, "%08x", cid.psn);

	return serial;
}

static EFI_STATUS mmc_serial_init(__attribute__((__unused__)) EFI_SYSTEM_TABLE *st)
{
	char *serial;

	serial = build_serial();
	if (serial)
		smbios_set(1, offsetof(SMBIOS_TYPE1, SerialNumber), serial);

	return EFI_SUCCESS;
}

ewdrv_t mmc_serial_drv = {
	.name = "mmc_serial",
	.description = "Set the serial number of the SMBIOS table based on the MMC CID",
	.init = mmc_serial_init
};
