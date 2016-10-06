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

#include <interface.h>
#include <kconfig.h>
#include <libpayload-config.h>
#include <libpayload.h>

#include "lifecycle/LifeCycleProtocol.h"
#include "lifecycle/lifecycle.h"

#define HECI1_HFS 0x40

typedef union hfs1 {
	struct {
		u32 working_state: 4;	/* Current working state */
		u32 manuf_mode: 1;	/* Manufacturing mode */
		u32 part_tbl_status: 1;	/* Indicates status of flash
					 * partition table */
		u32 reserved: 25; 	/* Reserved for further use */
		u32 d0i3_support: 1;	/* Indicates D0i3 support */
	} field;
	u32 data;
} hfs1_t;

static struct {
	u16 vid;
	u16 did;
} SUPPORTED_DEVICES[] ={
	{ .vid = 0x8086, .did = 0x5a9a }
};

static EFIAPI EFI_STATUS
get_life_cycle_state(__attribute__((__unused__)) EFI_LIFE_CYCLE_STATE_PROTOCOL *This,
		     EFI_LIFE_CYCLE_STATE *LifeCycleState)
{
	pcidev_t pci_dev = 0;
	hfs1_t status;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(SUPPORTED_DEVICES); i++)
		if (pci_find_device(SUPPORTED_DEVICES[i].vid,
				    SUPPORTED_DEVICES[i].did,
				    &pci_dev))
			break;

	if (!pci_dev)
		return EFI_UNSUPPORTED;

	status.data = pci_read_config32(pci_dev, HECI1_HFS);

	if (status.field.manuf_mode)
		*LifeCycleState = LC_STATE_MANUFACTURING;
	else
		*LifeCycleState = LC_STATE_ENDUSER;

	return EFI_SUCCESS;
}

static EFI_GUID lifecycle_guid = EFI_LIFE_CYCLE_STATE_PROTOCOL_GUID;
static EFI_HANDLE handle;

static EFI_STATUS lifecycle_init(EFI_SYSTEM_TABLE *st)
{
	static EFI_LIFE_CYCLE_STATE_PROTOCOL lifecycle_default = {
		.Revision = EFI_LIFE_CYCLE_STATE_PROTOCOL_REVISION1,
		.GetLifeCycleState = get_life_cycle_state
	};
	EFI_LIFE_CYCLE_STATE_PROTOCOL *lifecycle;

	return interface_init(st, &lifecycle_guid, &handle,
			      &lifecycle_default, sizeof(lifecycle_default),
			      (void **)&lifecycle);
}

static EFI_STATUS lifecycle_exit(EFI_SYSTEM_TABLE *st)
{
	return interface_free(st, &lifecycle_guid, handle);
}

ewdrv_t lifecycle_drv = {
	.name = "lifecycle",
	.description = "Life Cycle Protocol",
	.init = lifecycle_init,
	.exit = lifecycle_exit
};
