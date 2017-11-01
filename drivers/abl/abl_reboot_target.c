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
#include "heci/heci_protocol.h"

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

/* ABL Conventions */
#define NVRAM_START_ADDRESS		0x10

#define _USERCMD_(cmd, len)		(((cmd) << 5) | ((len) & 0x1f))
#define USERCMD_END			_USERCMD_(0, 0)
#define USERCMD_ACTION			_USERCMD_(7, 1)
#define USERCMD_UPDATE_IFWI(len)	_USERCMD_(2, len)

#define CDATA_TAG_USER_CMD		0x4d
#define NVRAM_VALID_FLAG		0x12

#define CRC32C_POLYNOMIAL 		0x82F63B78 /* CRC32C Castagnoli */

union _cdata_header {
	uint32_t data;
	struct {
		unsigned ncond	: 2;
		unsigned length	: 10;
		unsigned flags	: 4;
		unsigned version: 4;
		unsigned tag	: 12;
	};
};

struct nvram_capsule_cmd {
	char action;
	char device;
	char partition;
	char file_name[1];
} __attribute__((__packed__));

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
	size_t cdata_payload_size;
	uint32_t crc;
} __attribute__((__packed__));

static const struct name2id NAME2ID[] = {
	{ L"",			0x00 },
	{ L"boot",		0x00 },
	{ L"bootloader",	0x01 },
	{ L"fastboot",		0x01 },
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

/*  Compute CRC for one byte (shift register-based: one bit at a time). */
static uint32_t crc32c_byte(uint32_t crc, unsigned byte)
{
	int i;
	uint32_t c;

	for (i = 0 ; i < 8 ; i += 1) {
		c = (crc ^ byte) & 1;
		if (c)
			crc = (crc >> 1) ^ CRC32C_POLYNOMIAL;
		else
			crc = (crc >> 1);
		byte >>= 1;
	}

	return crc;
}

/*  Compute CRC for a given buffer. */
static uint32_t crc32c_buf(uint32_t crc, const void *addr, unsigned len)
{
	unsigned i;

	for (i = 0 ; i < len ; i += 1)
		crc = crc32c_byte(crc, *(uint8_t *)(addr + i));

	return crc;
}

static uint32_t crc32c_msg(struct nvram_msg *nvram_msg)
{
	uint32_t crc;

	crc = crc32c_buf(~0, nvram_msg,
			offsetof(struct nvram_msg, cdata_payload));
	crc = crc32c_buf(crc, nvram_msg->cdata_payload,
			nvram_msg->cdata_payload_size);
	return crc;
}

enum capsule_device_type {
	EMMC = 2,
	SDCARD = 4
};

typedef union _MKHI_MESSAGE_HEADER {
	uint32_t Data;
	struct {
		uint32_t  GroupId : 8;
		uint32_t  Command : 7;
		uint32_t  IsResponse : 1;
		uint32_t  Reserved : 8;
		uint32_t  Result : 8;
	} Fields;
} MKHI_MESSAGE_HEADER;

/*
 * User command  message
 */
#define CSE_USRCMD_SIZE			128 // <64 or ==64 will send fail
typedef struct __attribute__( (packed) ) _HECI_USER_CMD_REQUEST
{
	MKHI_MESSAGE_HEADER  MKHIHeader;
	uint8_t sub_command;
	uint8_t data[CSE_USRCMD_SIZE];
} HECI_USER_CMD_REQUEST;

typedef struct _HECI_USER_CMD_RESPONSE {
	MKHI_MESSAGE_HEADER Header;
} HECI_USER_CMD_RESPONSE;

#define MBP_APP_ABL_SIG         	0x20
#define MBP_ITEM_ID_IAFW_IBB_SIG	0x7
#define BIOS_FIXED_HOST_ADDR    	0
static unsigned heci_send_user_command(uint8_t *data, uint8_t length)
{
	unsigned status;
	uint32_t HeciSendLength;
	uint32_t HeciRecvLength;
	HECI_USER_CMD_REQUEST *SendCommand;
	HECI_USER_CMD_RESPONSE *CommandResp;
	EFI_GUID guid = HECI_PROTOCOL_GUID;
	EFI_HECI_PROTOCOL *protocol = NULL;
	uint32_t SeCMode;
	uint8_t DataBuffer[sizeof(HECI_USER_CMD_REQUEST)];

	if (length == 0) {
		ewerr("No need Sending HeciSendUserCommandClear.");
		return 1;
	}
	if (length > CSE_USRCMD_SIZE)
		length = CSE_USRCMD_SIZE;

	status = LibLocateProtocol(&guid, (void **)&protocol);
	if (EFI_ERROR(status)) {
		ewerr("Failed to get heciprotocol");
		return 1;
	}

	status = uefi_call_wrapper(protocol->GetSeCMode, 1, &SeCMode);
	if (EFI_ERROR(status) || (SeCMode != SEC_MODE_NORMAL)) {
		ewerr("Failed to get hecisecmode");
		return 1;
	}
	ewdbg("HECI sec_mode %X", SeCMode);

	memset (DataBuffer, 0, sizeof(DataBuffer));
	SendCommand= (HECI_USER_CMD_REQUEST*)DataBuffer;
	SendCommand->MKHIHeader.Fields.GroupId = MBP_APP_ABL_SIG;
	SendCommand->MKHIHeader.Fields.Command = MBP_ITEM_ID_IAFW_IBB_SIG;
	SendCommand->sub_command = 1;
	memcpy(SendCommand->data, data, length);

	HeciSendLength = sizeof(HECI_USER_CMD_REQUEST);
	HeciRecvLength = sizeof(HECI_USER_CMD_RESPONSE);
	status = uefi_call_wrapper(protocol->SendwACK, 5, (UINT32 *)DataBuffer,
							   HeciSendLength, &HeciRecvLength,
							   BIOS_FIXED_HOST_ADDR, 0x7);
	if (status != 0) {
		ewerr("Heci send fail: %x", status);
		return status;
	}
	ewdbg("uefi_call_wrapper(SendwACK) =  %d", status);

	CommandResp = (HECI_USER_CMD_RESPONSE*)DataBuffer;
	ewdbg( "Group    =%08x\n", CommandResp->Header.Fields.GroupId);
	ewdbg( "Command  =%08x\n", CommandResp->Header.Fields.Command);
	ewdbg( "IsRespone=%08x\n", CommandResp->Header.Fields.IsResponse);
	ewdbg( "Result   =%08x\n", CommandResp->Header.Fields.Result);
	if (CommandResp->Header.Fields.Result != 0) {
		status = CommandResp->Header.Fields.Result;
		ewerr("Send cmd fail: %x", status);
	}

	return status;
}

static EFI_STATUS write_msg_to_cse(struct nvram_msg *msg)
{
	uint8_t *msg_buf, *msg_buf_p, msg_slen;
	unsigned status;

	msg_buf = malloc(CSE_USRCMD_SIZE);
	if (!msg_buf)
		return EFI_OUT_OF_RESOURCES;
	memset(msg_buf, 0, CSE_USRCMD_SIZE);
	msg_buf_p = msg_buf;

	msg_slen = offsetof(struct nvram_msg, cdata_payload);
	msg_buf_p = (uint8_t *)memcpy(msg_buf_p, (uint8_t *)msg, msg_slen) + msg_slen;
	msg_slen = msg->cdata_payload_size;
	msg_buf_p = (uint8_t *)memcpy(msg_buf_p, msg->cdata_payload, msg_slen) + msg_slen;
	msg_slen = sizeof(msg->crc);
	memcpy(msg_buf_p, (uint8_t *)(&(msg->crc)), msg_slen);

	status = heci_send_user_command(msg_buf, CSE_USRCMD_SIZE);

	free(msg_buf);
	return status;
}

static EFI_STATUS capsule_store(const char *buf)
{
	char name[32]; /* Length of capsule file name can't exceed 30. */
	int name_len, partition;
	enum capsule_device_type device;
	struct nvram_msg msg;
	struct nvram_capsule_cmd *capsule_cmd;
	unsigned char capsule_cmd_size;
	union _cdata_header cdh;
	EFI_STATUS status = EFI_SUCCESS;

	ewdbg("capsule buffer: %s", buf); /* Buffer format example: "m1:@0" */

	device = (buf[0] == 'm' ? EMMC : SDCARD);
	partition = buf[1] - '0';
	memset(name, 0, sizeof(name));
	strcpy(name, buf + 3); /* Number 3 is start index of name in buffer. */
	name_len = strlen(name);
	ewdbg("capsule parameters: DEVICE=%d PARTITION=%d NAME=%s",
		  device, partition, name);

	capsule_cmd_size = (offsetof(struct nvram_capsule_cmd, file_name) + name_len + 3) & ~3;

	cdh.data = 0;
	cdh.tag = CDATA_TAG_USER_CMD;
	cdh.length = (sizeof(cdh) + capsule_cmd_size) / 4;

	msg.magic = NVRAM_VALID_FLAG;
	msg.size = offsetof(struct nvram_msg, cdata_payload) + capsule_cmd_size + sizeof(msg.crc);
	msg.cdata_header.data = cdh.data;

	capsule_cmd = malloc(capsule_cmd_size);
	if (!capsule_cmd)
		return EFI_OUT_OF_RESOURCES;

	capsule_cmd->action = USERCMD_UPDATE_IFWI(name_len + 2);
	capsule_cmd->device = device;
	capsule_cmd->partition = partition;
	strncpy(capsule_cmd->file_name, name, name_len);
	msg.cdata_payload = (char *)capsule_cmd;
	msg.cdata_payload_size = capsule_cmd_size;
	msg.crc = crc32c_msg(&msg);

	status = write_msg_to_cse(&msg);

	free(capsule_cmd);
	return status;
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
	msg.crc = crc32c_msg(&msg);

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
