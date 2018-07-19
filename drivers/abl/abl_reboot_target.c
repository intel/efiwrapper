/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 *
 * Author: Guillaume Betous <guillaume.betous@intel.com>
 *         Ji Wu <ji.j.wu@intel.com>
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
#include <ewvar.h>
#include <ewlog.h>
#include <efilib.h>
#include "capsule_msg.h"

#define RTC_PORT(x)			(0x70 + (x))
#define LOADER_ENTRY_ONESHOT		L"LoaderEntryOneShot"
#define IFWI_CAPSULE_UPDATE 		L"IfwiCapsuleUpdate"

/* RTC read and write */
static inline unsigned char cmos_read_ext_bank(u8 addr)
{
	outb(addr, RTC_PORT(4));
	return inb(RTC_PORT(5));
}
#define CMOS_READ_EXT(a)		cmos_read_ext_bank(a)

static inline void cmos_write_ext_bank(u8 val, u8 addr)
{
	outb(addr, RTC_PORT(4));
	outb(val, RTC_PORT(5));
}
#define CMOS_WRITE_EXT(v, a) 		cmos_write_ext_bank(v, a)

struct nvram_reboot_cmd {
	char action;
	char target;
	char end;
	char padding;
} __attribute__((__packed__));

struct name2id {
	const CHAR16 *name;
	int id;
};

struct nvram_msg {
	char magic;
	char size;
	union _cdata_header cdata_header;
	char *cdata_payload;
	uint32_t cdata_payload_size;
	uint32_t crc;
} __attribute__((__packed__));

static const struct name2id NAME2ID[] = {
	{ L"",			0x00 },
	{ L"boot",		0x00 },
	{ L"bootloader",	0x01 },
	{ L"fastboot",		0x01 },
	{ L"recovery",		0x03 },
	{ L"dnx",		0x05 },
};

static size_t offset; /* memorize offset between each call */

extern size_t str16len(const CHAR16 *str);

static size_t write_data_to_nvram(char *data, size_t size)
{
	size_t i;

	for (i = 0; i < size; i++)
		CMOS_WRITE_EXT(*(data + i), NVRAM_START_ADDRESS + offset + i);

	offset += size;

	return i;
}

static void write_msg_to_nvram(struct nvram_msg *nvram_msg)
{
	/* Ensure to start from top : only one command expected */
	offset = 0;
	write_data_to_nvram((char*)nvram_msg,
			offsetof(struct nvram_msg, cdata_payload));
	write_data_to_nvram(nvram_msg->cdata_payload,
			nvram_msg->cdata_payload_size);
	write_data_to_nvram((char*)(&(nvram_msg->crc)), sizeof(nvram_msg->crc));
}

static EFI_STATUS reboot_target_name2id(const CHAR16 *name, int *id)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(NAME2ID); i++)
		if (str16len(NAME2ID[i].name) == str16len(name) &&
			!memcmp(NAME2ID[i].name, name, str16len(name) * sizeof(*name))) {
			if (id) {
				*id = NAME2ID[i].id;
				return EFI_SUCCESS;
			}
		}

	return EFI_NOT_FOUND;
}

static EFI_STATUS set_reboot_target(const CHAR16 *name)
{
	int id;
	struct nvram_msg msg;
	struct nvram_reboot_cmd reboot_cmd;
	union _cdata_header cdh;
	EFI_STATUS ret;

	if (!name)
		return EFI_INVALID_PARAMETER;

	ret = reboot_target_name2id(name, &id);
	if (EFI_ERROR(ret)) {
		ewerr("Error in %s: '%s' is not a valid target",
			__func__, (char*)name);
		return EFI_INVALID_PARAMETER;
	}
	if (id == 0) {
		ewdbg("Target 'boot' no need write to nvram.");
		return EFI_SUCCESS;
	}

	cdh.data = 0;
	cdh.length = 2; /* 2*32 bits, from header to padding */
	cdh.tag = CDATA_TAG_USER_CMD;

	memset(&reboot_cmd, 0, sizeof(reboot_cmd));
	memset(&msg, 0, sizeof(msg));
	msg.magic = NVRAM_VALID_FLAG;
	msg.cdata_header.data = cdh.data;
	reboot_cmd.action = USERCMD_ACTION;

	reboot_cmd.target = id;
	msg.cdata_payload = (char*)&reboot_cmd;
	msg.cdata_payload_size = sizeof(reboot_cmd);
	msg.size = offsetof(struct nvram_msg, cdata_payload) +
		sizeof(reboot_cmd) + sizeof(msg.crc);
	msg.crc = crc32c_msg((char *)&msg, offsetof(struct nvram_msg, cdata_payload), msg.cdata_payload, (size_t)msg.cdata_payload_size);

	write_msg_to_nvram(&msg);

	return EFI_SUCCESS;
}

static EFI_STATUS reboot_target_save(ewvar_t *var)
{
	const CHAR16* name;

	if (!var)
		return EFI_INVALID_PARAMETER;

	name = LOADER_ENTRY_ONESHOT;
	if (str16len(var->name) == str16len(name) &&
		!memcmp(var->name, name, str16len(name) * sizeof(*name))) {
		return set_reboot_target(var->data);
	}

	name = IFWI_CAPSULE_UPDATE;
	if (str16len(var->name) == str16len(name) &&
		!memcmp(var->name, name, str16len(name) * sizeof(*name))) {
		return capsule_store(var->data);
	}

	return EFI_SUCCESS;
}

ewvar_storage_t reboot_target_storage = {
	.load = NULL,
	.save = reboot_target_save,
	.delete = NULL
};
