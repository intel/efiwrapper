/*
 * Copyright (c) 2016, Intel Corporation
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
 *
 */

#ifndef _LIFE_CYCLE_PROTOCOL_H_
#define _LIFE_CYCLE_PROTOCOL_H_

#include <efi.h>

#define EFI_LIFE_CYCLE_STATE_PROTOCOL_GUID \
	{0xf3c1138e, 0xcd89, 0x4e20,{0x9e, 0x68, 0x25, 0xa6, 0x76, 0x95, 0xa5, 0x6a}}

#define EFI_LIFE_CYCLE_STATE_PROTOCOL_REVISION1 0x00000001

typedef enum life_cycle_state {
	LC_STATE_MANUFACTURING,
	LC_STATE_ENDUSER,
	LC_STATE_RND,
	LC_STATE_CARE
} EFI_LIFE_CYCLE_STATE;

typedef struct _EFI_LIFE_CYCLE_STATE_PROTOCOL EFI_LIFE_CYCLE_STATE_PROTOCOL;

typedef
EFI_STATUS
(EFIAPI *EFI_GET_LIFE_CYCLE_STATE) (
	IN EFI_LIFE_CYCLE_STATE_PROTOCOL *This,
	OUT EFI_LIFE_CYCLE_STATE *LifeCycleState
	);

struct _EFI_LIFE_CYCLE_STATE_PROTOCOL {
	UINT32 Revision;
	EFI_GET_LIFE_CYCLE_STATE GetLifeCycleState;
} __attribute__((packed));

#endif	/* _LIFE_CYCLE_PROTOCOL_H_ */
