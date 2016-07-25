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

#ifndef _EFIWRAPPER_H_
#define _EFIWRAPPER_H_

#include <efi.h>
#include <efilib.h>
#include <efivar.h>

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(*x))
#endif

/* Memory management */
void *ew_malloc(unsigned int size);
void *ew_realloc(void *ptr, unsigned int size, unsigned int oldsize);
void *ew_calloc(unsigned long nmemb, unsigned long size);
void ew_free(void *ptr);

/* Tools */
void *ew_memset(void *s, int c, unsigned long n);
int ew_memcmp(const void *s1, const void *s2, unsigned long n);
int ew_guidcmp(EFI_GUID *g1, EFI_GUID *g2);
void *ew_memcpy(void *dest, const void *src, unsigned long n);
unsigned int ew_str16len(const CHAR16 *str);
char *ew_strdup(const char *s);
CHAR16 *ew_str16dup(const CHAR16 *str);
int ew_str16_to_str(const CHAR16 *str16, char *str);
CHAR16 *ew_str_to_str16_p(const char *str);
int ew_str_to_str16(const char *str, CHAR16 *str16);
unsigned long ew_strlen(const char *s);
CHAR16 *ew_str_to_str16_p(const char *str);
unsigned long ew_strtoul(const char *nptr, char **endptr, int base);
EFI_STATUS ew_str_to_guid(const char *str, EFI_GUID *g);

#endif	/* _EFIWRAPPER_H_ */
