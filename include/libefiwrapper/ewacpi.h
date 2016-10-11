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

#ifndef _EWACPI_H_
#define _EWACPI_H_

#include <efi.h>
#include <efiapi.h>

struct acpi_header {
	char		signature[4]; 		/* ASCII Table identifier */
	uint32_t	length;			/* Length of the table, including the header */
	char		revision;		/* Revision of the structure */
	char		checksum;		/* Sum of all fields must be 0 */
	char		oem_id[6];		/* ASCII OEM identifier */
	char		oem_table_id[8];	/* ASCII OEM table identifier */
	uint32_t	oem_revision;		/* OEM supplied revision number */
	char		creator_id[4];		/* Vendor ID of utility creator of the table */
	uint32_t	creator_revision; 	/* Revision of utility creator of the table */
} __attribute__((packed));

EFI_STATUS ewacpi_get_table(EFI_SYSTEM_TABLE *st, const char *name,
			    struct acpi_header **table);

#endif	/* _EWACPI_H_ */
