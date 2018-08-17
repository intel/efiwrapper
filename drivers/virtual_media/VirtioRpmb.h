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

#ifndef _VIRTIO_RPMB_H_
#define _VIRTIO_RPMB_H_

#include "Virtio.h"

#define MAX_COMMAND_RPMB		3

#pragma pack(1)
typedef struct {
	UINT8 stuff[196];
	UINT8 key_mac[32];
	UINT8 data[256];
	UINT8 nonce[16];
	UINT32 write_counter;
	UINT16 address;
	UINT16 block_count;
	UINT16 result;
	UINT16 req_resp;
} RPMB_DATA_FRAME;
#pragma pack()

typedef struct {
	UINT32 rpmb_flag;
	UINT32 n_rpmb_frame;
	RPMB_DATA_FRAME *addr_rpmb_frame;
} VIRTIO_RPMB_CMD;

typedef struct {
	UINT64 n_cmds;
	VIRTIO_RPMB_CMD cmds[MAX_COMMAND_RPMB + 1];
} VIRTIO_RPMB_IOCTL_SEQ_DATA;

typedef struct {
	UINT32 IoctlCmd;
	int Result;
	UINT8 Target;
	UINT8 Reserved[3];
} VIRTIO_RPMB_IOC;

struct rpmb_ioc_req_cmd {
	UINT64 req_type;
	VIRTIO_RPMB_CMD icmd;
	VIRTIO_RPMB_CMD ocmd;
};

struct rpmb_ioc_seq_cmd {
	UINT64 num_of_cmds;
	VIRTIO_RPMB_CMD cmds[0];
};

#define RPMB_IOC_SEQ_CMD 0xC008B552

#endif // _VIRTIO_RPMB_H_
