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
/* SBL Conventions */
#define SIGNATURE_16(A, B)              ((A) | (B << 8))
#define SIGNATURE_32(A, B, C, D)        (SIGNATURE_16 (A, B) | (SIGNATURE_16 (C, D) << 16))
#define CFG_DATA_SIGNATURE              SIGNATURE_32('C', 'F', 'G', 'D')
#define MAX_FILE_LEN                    16
#define CDATA_CAPSULE_TAG               0xE00

typedef struct cdata_cond {
	uint32_t  Value;    // Bit masks on supported platforms
} __attribute__((__packed__)) cdata_cond_t;

typedef struct cdata_header {
	uint32_t  ncond          :  2;      // [1:0]   #of condition words present
	uint32_t  length         : 10;      // [11:2]  total size of item (in dwords)
	uint32_t  flags          :  4;      // [15:12] unused/reserved so far
	uint32_t  version        :  4;      // [19:16] item (payload) format version
	uint32_t  tag            : 12;      // [31:20] identifies item (in payload)
	struct cdata_cond        Condition;
} __attribute__((__packed__)) cdata_header_t ;

typedef struct cdata_blob {
	uint32_t  Signature;
	uint8_t   HeaderLength;
	uint8_t   Attribute;
	uint8_t   Reserved[2];
	uint32_t  UsedLength;
	uint32_t  TotalLength;
} __attribute__((__packed__)) cdata_blob_t;

typedef struct cse4sbl_capsule_cmd {
	uint32_t dev_addr;
	uint8_t  dev_type;
	uint8_t  hw_part;
	uint8_t  sw_part;
	uint8_t  fs_type;
	uint8_t  FileName[MAX_FILE_LEN];
	uint32_t LbaAddr;
} __attribute__((__packed__)) CSE_CMD;

typedef struct cse4sbl_capsule_msg {
	struct cdata_blob cdb;
	struct cdata_header cdh;
	struct cse4sbl_capsule_cmd cmd;
} __attribute__((__packed__)) CSE_MSG;
#else
typedef struct cse4abl_capsule_cmd {
	char action;
	char device;
	char partition;
	char file_name[1];
} __attribute__((__packed__)) CSE_CMD;

typedef struct cse4abl_capsule_msg {
	char magic;
	char size;
	union _cdata_header cdata_header;
	char *cdata_payload;
	uint32_t cdata_payload_size;
	uint32_t crc;
} __attribute__((__packed__)) CSE_MSG;
#endif

uint32_t crc32c_msg(const char *msg, UINTN offset, const void *addr, size_t len);
EFI_STATUS capsule_store(const char *buf);

#endif /* ifndef _CAPSULE_MSG_H_ */
