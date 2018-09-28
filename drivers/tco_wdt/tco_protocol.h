/*
 * Copyright (c) 2015, Intel Corporation
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

#ifndef _TCO_PROTOCOL_H_
#define _TCO_PROTOCOL_H_

#include <efi.h>

#define EFI_TCO_RESET_PROTOCOL_GUID                                     \
        {0xa6a79162, 0xe325, 0x4c30,{0xbc, 0xc3, 0x59, 0x37, 0x30, 0x64, 0xef, 0xb3}}

typedef struct _EFI_TCO_RESET_PROTOCOL EFI_TCO_RESET_PROTOCOL;

typedef
EFI_STATUS
(EFIAPI *EFI_TCO_RESET_PROTOCOL_ENABLE_WATCHDOG) (
        IN OUT UINT32 *RcrbGcsValue
        );

typedef
EFI_STATUS
(EFIAPI *EFI_TCO_RESET_PROTOCOL_DISABLE_WATCHDOG) (
        IN UINT32 RcrbGcsValue
        );

struct _EFI_TCO_RESET_PROTOCOL {
        EFI_TCO_RESET_PROTOCOL_ENABLE_WATCHDOG  EnableTcoReset;
        EFI_TCO_RESET_PROTOCOL_DISABLE_WATCHDOG DisableTcoReset;
};

#endif	/* _TCO_PROTOCOL_H_ */
