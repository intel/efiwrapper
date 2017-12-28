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

#ifndef _CAPSULE_MSG_H_
#define _CAPSULE_MSG_H_

#include <kconfig.h>
#include <libpayload-config.h>
#include <libpayload.h>
#include <ewvar.h>
#include <ewlog.h>
#include <efilib.h>

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

#ifdef CAPSULE4SBL
struct image_info {
	uint64_t image_lba_addr;
	uint32_t image_length;
	uint32_t reserved;
};

struct os_image_info {
	uint32_t dev_addr;
	uint8_t dev_type;
	uint8_t fs_type;
	uint8_t hw_part;
	uint8_t sw_part;
	struct image_info file_path;
} __attribute__((__packed__));

struct cse4sbl_msg {
	uint16_t magic;
	uint16_t total_size;
	char revision;
	char check_sum;
	char reserved[2];
	union _cdata_header cdata_header;
	char *cdata_payload;
	size_t cdata_payload_size;
} __attribute__((__packed__));

typedef struct cse4sbl_msg cse_msg;
typedef struct os_image_info cse_cmd;
#else
struct nvram_capsule_cmd {
	char action;
	char device;
	char partition;
	char file_name[1];
} __attribute__((__packed__));

struct nvram_msg {
	char magic;
	char size;
	union _cdata_header cdata_header;
	char *cdata_payload;
	size_t cdata_payload_size;
	uint32_t crc;
} __attribute__((__packed__));

typedef struct nvram_msg cse_msg;
typedef struct nvram_capsule_cmd cse_cmd;
uint32_t crc32c_msg(cse_msg *cse_msg);
#endif

EFI_STATUS capsule_store(const char *buf);

#endif /* ifndef _CAPSULE_MSG_H_ */
