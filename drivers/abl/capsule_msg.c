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

#include <kconfig.h>
#include <libpayload-config.h>
#include <libpayload.h>
#include <ewvar.h>
#include <ewlog.h>
#include <efilib.h>
#include "capsule_msg.h"
#include "heci/heci_protocol.h"
#include <storage.h>

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

uint32_t crc32c_msg(const char *msg, UINTN offset, const void *addr, size_t len)
{
	uint32_t crc;

	crc = crc32c_buf(~0, msg, offset);
	crc = crc32c_buf(crc, addr, len);
	return crc;
}

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
	EFI_STATUS status;
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
		ewerr("Heci send fail: %x", (UINT32)status);
		return status;
	}
	ewdbg("uefi_call_wrapper(SendwACK) =  %d", (UINT32)status);

	CommandResp = (HECI_USER_CMD_RESPONSE*)DataBuffer;
	ewdbg( "Group    =%08x\n", CommandResp->Header.Fields.GroupId);
	ewdbg( "Command  =%08x\n", CommandResp->Header.Fields.Command);
	ewdbg( "IsRespone=%08x\n", CommandResp->Header.Fields.IsResponse);
	ewdbg( "Result   =%08x\n", CommandResp->Header.Fields.Result);
	if (CommandResp->Header.Fields.Result != 0) {
		status = CommandResp->Header.Fields.Result;
		ewerr("Send cmd fail: %x", (UINT32)status);
	}

	return status;
}

#ifdef CAPSULE4SBL
typedef enum {
	OsBootDeviceSata,
	OsBootDeviceSd,
	OsBootDeviceEmmc,
	OsBootDeviceUfs,
	OsBootDeviceSpi,
	OsBootDeviceUsb,
	OsBootDeviceNvme,
	OsBootDeviceMax
} SBL_OS_BOOT_MEDIUM_TYPE;

typedef enum  {
	EnumFileSystemTypeFat,
	EnumFileSystemTypeExt2,
	EnumFileSystemTypeAuto,
	EnumFileSystemMax
} SBL_OS_FILE_SYSTEM_TYPE;

static BOOLEAN capsule_request_slot_b;

static EFI_STATUS cse4sbl_capsule_msg_write(CSE_MSG *msg)
{
	uint8_t *msg_buf, *msg_buf_p, msg_slen;
	unsigned status;

	// It's temporary patch before SBL officially support HECI cmd trigger fw update"
	outb(0x40, 0x70);
	if (capsule_request_slot_b) {
		ewdbg("%s: Capsule requested, set CMOS 0x5B value for ICL KBL hack",__func__);
		outb(0x5B, 0x71);
	} else {
		ewdbg("%s: Capsule requested, set CMOS 0x5A value for ICL KBL hack",__func__);
		outb(0x5A, 0x71);
	}
	return EFI_SUCCESS;

	msg_buf = malloc(CSE_USRCMD_SIZE);
	if (!msg_buf)
		return EFI_OUT_OF_RESOURCES;
	memset(msg_buf, 0, CSE_USRCMD_SIZE);
	msg_buf_p = msg_buf;

	msg_slen = offsetof(CSE_MSG, cdata_payload);
	msg_buf_p = (uint8_t *)memcpy(msg_buf_p, (uint8_t *)msg, msg_slen) + msg_slen;
	msg_slen = msg->cdata_payload_size;
	msg_buf_p = (uint8_t *)memcpy(msg_buf_p, msg->cdata_payload, msg_slen) + msg_slen;

	status = heci_send_user_command(msg_buf, CSE_USRCMD_SIZE);

	free(msg_buf);
	return status;
}

static EFI_STATUS cse4sbl_capsule_cmd_create(CSE_CMD **cmd, size_t *cmd_size, const char *buf)
{
	char name[32]; /* Length of capsule file name can't exceed 30. */
	int partition;

	/* storage_type mapping */
	SBL_OS_BOOT_MEDIUM_TYPE device_map[] = {
		[STORAGE_EMMC]   = OsBootDeviceEmmc,
		[STORAGE_UFS]    = OsBootDeviceUfs,
		[STORAGE_SDCARD] = OsBootDeviceSd,
		[STORAGE_SATA]   = OsBootDeviceSata,
		[STORAGE_NVME]   = OsBootDeviceNvme,
		[STORAGE_ALL]    = OsBootDeviceEmmc
	};

	boot_dev_t *boot_dev;

	ewdbg("capsule buffer: %s", buf); /* Buffer format example: "m1:@0" */

	partition = buf[1] - '0';
	if (partition == 2)
		capsule_request_slot_b = TRUE;
	else
		capsule_request_slot_b = FALSE;
	memset(name, 0, sizeof(name));
	strncpy(name, buf + 3, sizeof(name) - 1); /* Number 3 is start index of name in buffer. */

	boot_dev = get_boot_media();

	ewdbg("capsule parameters: DEVICE=%d PARTITION=%d NAME=%s",
		  device_map[boot_dev->type], partition, name);
	if (name[0] != '@')
		return EFI_INVALID_PARAMETER;

	*cmd_size = sizeof(CSE_CMD);

	*cmd = malloc(*cmd_size);
	if (!(*cmd))
		return EFI_OUT_OF_RESOURCES;

	(*cmd)->dev_addr = boot_dev->diskbus;
	(*cmd)->dev_type = device_map[boot_dev->type];
	(*cmd)->fs_type = (uint8_t)EnumFileSystemTypeAuto;
	(*cmd)->hw_part = 0;
	(*cmd)->sw_part = partition;
	(*cmd)->file_path.image_lba_addr = strtoull(name+1, NULL, 16);
	(*cmd)->file_path.image_length = 0;
	(*cmd)->file_path.reserved = 0;
	return EFI_SUCCESS;
}

static int get_cse_msg_sum(CSE_MSG *msg, CSE_CMD *cmd, size_t cmd_size)
{
	size_t i = 0;
	char *p = NULL, sum = 0;

	p = (char *)msg;
	for (i = 0; i < offsetof(CSE_MSG, cdata_payload); ++i)
		sum += p[i];

	sum -= msg->check_sum;

	p = (char *)cmd;
	for (i = 0; i < cmd_size; ++i)
		sum += p[i];

	return -sum;
}

/* If multiple payload is needed, the cmd and cmd_size could be changed to array */
static EFI_STATUS cse4sbl_capsule_msg_create(CSE_MSG **msg, CSE_CMD *cmd, size_t cmd_size)
{
	union _cdata_header cdh;

	cdh.data = 0;
	cdh.tag = CDATA_TAG_USER_CMD;
	cdh.length = (sizeof(cdh) + cmd_size) / 4;

	*msg = malloc(sizeof(CSE_MSG));
	if (!(*msg))
		return EFI_OUT_OF_RESOURCES;

	(*msg)->magic = NVRAM_VALID_FLAG;
	(*msg)->total_size = offsetof(CSE_MSG, cdata_payload) + cmd_size;
	(*msg)->revision = 0;
	(*msg)->check_sum = get_cse_msg_sum(*msg, cmd, cmd_size);

	memset((*msg)->reserved, 0, 2);

	(*msg)->cdata_header.data = cdh.data;
	(*msg)->cdata_payload = (char *)cmd;
	(*msg)->cdata_payload_size = cmd_size;

	return EFI_SUCCESS;
}
#else
static EFI_STATUS cse4abl_capsule_msg_write(CSE_MSG *msg)
{
	uint8_t *msg_buf, *msg_buf_p, msg_slen;
	unsigned status;

	msg_buf = malloc(CSE_USRCMD_SIZE);
	if (!msg_buf)
		return EFI_OUT_OF_RESOURCES;
	memset(msg_buf, 0, CSE_USRCMD_SIZE);
	msg_buf_p = msg_buf;

	msg_slen = offsetof(CSE_MSG, cdata_payload);
	msg_buf_p = (uint8_t *)memcpy(msg_buf_p, (uint8_t *)msg, msg_slen) + msg_slen;
	msg_slen = msg->cdata_payload_size;
	msg_buf_p = (uint8_t *)memcpy(msg_buf_p, msg->cdata_payload, msg_slen) + msg_slen;
	msg_slen = sizeof(msg->crc);
	memcpy(msg_buf_p, (uint8_t *)(&(msg->crc)), msg_slen);

	status = heci_send_user_command(msg_buf, CSE_USRCMD_SIZE);

	free(msg_buf);
	return status;
}

enum abl_capsule_device_type {
	EMMC = 2,
	SDCARD = 4
};

static EFI_STATUS cse4abl_capsule_cmd_create(CSE_CMD **cmd, size_t *cmd_size, const char *buf)
{
	char name[32]; /* Length of capsule file name can't exceed 30. */
	int name_len, partition;
	enum abl_capsule_device_type device;

	ewdbg("capsule buffer: %s", buf); /* Buffer format example: "m1:@0" */

	device = (buf[0] == 'm' ? EMMC : SDCARD);
	partition = buf[1] - '0';
	memset(name, 0, sizeof(name));
	strncpy(name, buf + 3, sizeof(name) - 1); /* Number 3 is start index of name in buffer. */
	name_len = strlen(name) + 1;
	ewdbg("capsule parameters: DEVICE=%d PARTITION=%d NAME=%s",
		  device, partition, name);

	*cmd_size = (offsetof(CSE_CMD, file_name) + name_len + 3) & ~3;

	*cmd = malloc(*cmd_size);
	if (!(*cmd))
		return EFI_OUT_OF_RESOURCES;

	(*cmd)->action = USERCMD_UPDATE_IFWI(name_len + 2);
	(*cmd)->device = device;
	(*cmd)->partition = partition;
	strlcpy((*cmd)->file_name, name, name_len);
	return EFI_SUCCESS;
}

static EFI_STATUS cse4abl_capsule_msg_create(CSE_MSG **msg, CSE_CMD *cmd, size_t cmd_size)
{
	union _cdata_header cdh;

	cdh.data = 0;
	cdh.tag = CDATA_TAG_USER_CMD;
	cdh.length = (sizeof(cdh) + cmd_size) / 4;

	*msg = malloc(sizeof(CSE_MSG));
	if (!(*msg))
		return EFI_OUT_OF_RESOURCES;

	(*msg)->magic = NVRAM_VALID_FLAG;
	(*msg)->size = offsetof(CSE_MSG, cdata_payload) + cmd_size + sizeof((*msg)->crc);
	(*msg)->cdata_header.data = cdh.data;

	(*msg)->cdata_payload = (char *)cmd;
	(*msg)->cdata_payload_size = cmd_size;
	(*msg)->crc = crc32c_msg((char *)(*msg), offsetof(CSE_MSG, cdata_payload), (*msg)->cdata_payload, (size_t)(*msg)->cdata_payload_size);
	return EFI_SUCCESS;
}
#endif
static void capsule_free_all(CSE_CMD **cmd, CSE_MSG **msg)
{
	if (msg != NULL && *msg != NULL) {
		free(*msg);
		*msg = NULL;
	}
	if (cmd != NULL && *cmd != NULL) {
		free(*cmd);
		*cmd = NULL;
	}
}

static struct capsule_opt {
	EFI_STATUS (*cmd_create)(CSE_CMD **cmd, size_t *cmd_size, const char *buf);
	EFI_STATUS (*msg_create)(CSE_MSG **msg, CSE_CMD *cmd, size_t cmd_size);
	EFI_STATUS (*msg_write)(CSE_MSG *msg);
	void (*free_all)(CSE_CMD **cmd, CSE_MSG **msg);
} CAPSULE_OPT = {
#ifdef CAPSULE4SBL
	cse4sbl_capsule_cmd_create,
	cse4sbl_capsule_msg_create,
	cse4sbl_capsule_msg_write,
#else
	cse4abl_capsule_cmd_create,
	cse4abl_capsule_msg_create,
	cse4abl_capsule_msg_write,
#endif
	capsule_free_all,
};

EFI_STATUS capsule_store(const char *buf)
{
	CSE_MSG *capsule_msg = NULL;
	CSE_CMD *capsule_cmd = NULL;
	size_t capsule_cmd_size;
	EFI_STATUS ret = EFI_SUCCESS;

	ret = CAPSULE_OPT.cmd_create((CSE_CMD **)&capsule_cmd, &capsule_cmd_size, buf);
	if (EFI_ERROR(ret)) {
		ewerr("Error in %s: create capsule command failed!", __func__);
		goto error;
	}

	ret = CAPSULE_OPT.msg_create((CSE_MSG **)&capsule_msg, (CSE_CMD *)capsule_cmd, capsule_cmd_size);
	if (EFI_ERROR(ret)) {
		ewerr("Error in %s: create capsule message failed!", __func__);
		goto error;
	}

	ret = CAPSULE_OPT.msg_write((CSE_MSG *)capsule_msg);
	if (EFI_ERROR(ret)) {
		ewerr("Error in %s: write capsule message failed!", __func__);
		goto error;
	}

	CAPSULE_OPT.free_all((CSE_CMD **)&capsule_cmd, (CSE_MSG **)&capsule_msg);
	return EFI_SUCCESS;

error:
	CAPSULE_OPT.free_all((CSE_CMD **)&capsule_cmd, (CSE_MSG **)&capsule_msg);
	return ret;
}
