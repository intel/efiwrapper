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

#include "efiwrapper.h"
#include "platform.h"

/* Memory management */
void *ew_malloc(unsigned int size)
{
	if (!platform)
		return NULL;

	return platform->malloc(size);
}

void *ew_realloc(void *ptr, unsigned int size, unsigned int oldsize)
{
	unsigned int i;
	char *newbuf;

	if (size < oldsize)
		return ptr;

	newbuf = ew_malloc(size);
	if (!newbuf)
		return NULL;

	for (i = 0; i < oldsize; i++)
		newbuf[i] = ((char *)ptr)[i];

	ew_free(ptr);
	return newbuf;
}

void *ew_calloc(unsigned long nmemb, unsigned long size)
{
	char *buf;
	unsigned long bufsize;

	bufsize = size * nmemb;
	buf = ew_malloc(bufsize);
	if (!buf)
		return NULL;

	return ew_memset(buf, 0, bufsize);
}

void ew_free(void *ptr)
{
	if (!platform)
		return;

	platform->free(ptr);
}

/* Tools */
void *ew_memset(void *s, int c, unsigned long n)
{
	char *buf = s;
	unsigned long i;

	if (!s)
		return NULL;

	for (i = 0; i < n; i++)
		buf[i] = c;

	return s;
}

int ew_memcmp(const void *s1, const void *s2, unsigned long n)
{
	unsigned char *b1 = (unsigned char *)s1;
	unsigned char *b2 = (unsigned char *)s2;
	unsigned long i;

	for (i = 0; i < n; i++)
		if (b1[i] != b2[i])
			return b1[i] - b2[i];

	return 0;
}

void *ew_memcpy(void *dest, const void *src, unsigned long n)
{
	unsigned char *destb = (unsigned char *)dest;
	unsigned char *srcb = (unsigned char *)src;
	unsigned long i;

	for (i = 0; i < n; i++)
		*destb++ = *srcb++;

	return dest;
}

int ew_guidcmp(EFI_GUID *g1, EFI_GUID *g2)
{
	return ew_memcmp(g1, g2, sizeof(*g1));
}

unsigned int ew_str16len(const CHAR16 *str)
{
	unsigned int len;

	for (len = 0; *str; len++)
		str++;

	return len;
}

char *ew_strdup(const char *s)
{
	char *dup;
	unsigned int size;

	size = ew_strlen(s) + 1;
	dup = ew_malloc(size);
	if (!dup)
		return NULL;

	ew_memcpy(dup, s, size);
	return dup;
}

CHAR16 *ew_str16dup(const CHAR16 *str)
{
	CHAR16 *dup;
	unsigned int size;

	size = (ew_str16len(str) + 1) * sizeof(*str);
	dup = ew_malloc(size);
	if (!dup)
		return NULL;

	ew_memcpy(dup, str, size);
	return dup;
}

int ew_str16_to_str(const CHAR16 *str16, char *str)
{
	unsigned int i;

	for (i = 0; str16[i] != '\0'; i++) {
		if (str16[i] > 0x7f)
			return -1;
		str[i] = str16[i];
	}

	str[i] = '\0';
	return i;
}

int ew_str_to_str16(const char *str, CHAR16 *str16)
{
	unsigned int i;

	for (i = 0; str[i] != '\0'; i++)
		str16[i] = str[i];

	str16[i] = '\0';
	return i;
}

CHAR16 *ew_str_to_str16_p(const char *str)
{
	CHAR16 *copy;

	copy = ew_malloc((ew_strlen(str) + 1) * sizeof(*copy));
	if (!copy)
		return NULL;

	ew_str_to_str16(str, copy);

	return copy;
}

unsigned long ew_strlen(const char *s)
{
	unsigned long len;

	for (len = 0; s[len]; len++)
		;

	return len;
}

static INTN to_digit(CHAR16 character, UINTN base)
{
	UINTN value = -1;

	if (character >= '0' && character <= '9')
		value = character - '0';
	else if (character >= 'a' && character <= 'z')
		value = 0xA + character - 'a';
	else if (character >= 'A' && character <= 'Z')
		value = 0xA + character - 'A';

	return value < base ? (INTN)value : -1;
}

unsigned long ew_strtoul(const char *nptr, char **endptr, int base)
{
	unsigned long value = 0;

	if (!nptr)
		goto out;

	if ((base == 0 || base == 16) &&
	    (ew_strlen(nptr) > 2 && nptr[0] == '0' && nptr[1] == 'x')) {
		nptr += 2;
		base = 16;
	}

	if (base == 0)
		base = 10;

	for (; *nptr != '\0' ; nptr++) {
		int t = to_digit(*nptr, base);
		if (t == -1)
			goto out;
		value = (value * base) + t;
	}

out:
	if (endptr)
		*endptr = (char *)nptr;
	return value;
}

EFI_STATUS ew_str_to_guid(const char *str, EFI_GUID *g)
{
	char value[3] = { '\0', '\0', '\0' };
	char *end;
	UINTN i;

	if (!str || !g)
		return EFI_INVALID_PARAMETER;

	g->Data1 = ew_strtoul(str, &end, 16);
	if (end - str != 8 || *end != '-')
		return EFI_INVALID_PARAMETER;

	str = end + 1;
	g->Data2 = ew_strtoul(str, &end, 16);
	if (end - str != 4 || *end != '-')
		return EFI_INVALID_PARAMETER;

	str = end + 1;
	g->Data3 = ew_strtoul(str, &end, 16);
	if (end - str != 4 || *end != '-')
		return EFI_INVALID_PARAMETER;

	str = end + 1;
	for (i = 0 ; i < 2; i++, str += 2) {
		value[0] = str[0];
		value[1] = str[1];
		g->Data4[i] = ew_strtoul(value, &end, 16);
		if (end != value + 2)
			return EFI_INVALID_PARAMETER;
	}

	if (*str != '-')
		return EFI_INVALID_PARAMETER;

	str++;
	for (i = 0 ; i < 6; i++, str += 2) {
		value[0] = str[0];
		value[1] = str[1];
		g->Data4[i + 2] = ew_strtoul(value, &end, 16);
		if (end != value + 2)
			return EFI_INVALID_PARAMETER;
	}

	return EFI_SUCCESS;
}
