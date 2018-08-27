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
 *
 */

#ifndef _ACPI_TABLE_PROTOCOL_H_
#define _ACPI_TABLE_PROTOCOL_H_

#define EFI_ACPI_TABLE_PROTOCOL_GUID                                           \
  {                                                                            \
    0xffe06bdd, 0x6107, 0x46a6, {                                              \
      0x7b, 0xb2, 0x5a, 0x9c, 0x7e, 0xc5, 0x27, 0x5c                           \
    }                                                                          \
  }

typedef struct _EFI_ACPI_TABLE_PROTOCOL EFI_ACPI_TABLE_PROTOCOL;

typedef EFI_STATUS(EFIAPI *EFI_ACPI_TABLE_INSTALL_ACPI_TABLE)(
    IN EFI_ACPI_TABLE_PROTOCOL *This, IN VOID *AcpiTableBuffer,
    IN UINTN AcpiTableBufferSize, OUT UINTN *TableKey);

typedef EFI_STATUS(EFIAPI *EFI_ACPI_TABLE_UNINSTALL_ACPI_TABLE)(
    IN EFI_ACPI_TABLE_PROTOCOL *This, IN UINTN TableKey);

struct _EFI_ACPI_TABLE_PROTOCOL {
  EFI_ACPI_TABLE_INSTALL_ACPI_TABLE InstallAcpiTable;
  EFI_ACPI_TABLE_UNINSTALL_ACPI_TABLE UninstallAcpiTable;
};

#endif
