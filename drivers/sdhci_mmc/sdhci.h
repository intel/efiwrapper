/******************************************************************************
 *
 * INTEL CONFIDENTIAL
 *
 * Copyright (c) 1999-2013 Intel Corporation All Rights Reserved.
 *
 * The source code contained or described herein and all documents related to
 * the source code (Material) are owned by Intel Corporation or its suppliers
 * or licensors. Title to the Material remains with Intel Corporation or its
 * suppliers and licensors. The Material contains trade secrets and proprietary
 * and confidential information of Intel or its suppliers and licensors. The
 * Material is protected by worldwide copyright and trade secret laws and
 * treaty provisions. No part of the Material may be used, copied, reproduced,
 * modified, published, uploaded, posted, transmitted, distributed, or
 * disclosed in any way without Intel's prior express written permission.
 *
 * No license under any patent, copyright, trade secret or other intellectual
 * property right is granted to or conferred upon you by disclosure or delivery
 * of the Materials, either expressly, by implication, inducement, estoppel or
 * otherwise. Any license under such intellectual property rights must be
 * express and approved by Intel in writing.
 *
 ******************************************************************************/

#ifndef _SDHCI_H_
#define _SDHCI_H_

#include <arch/io.h>

#include "mmc.h"
#include "pci.h"

/**
 * Generic SDHCI controller
 * @init_controller: controller initialization function
 * @send_cmd: send CMD to card
 * @wait_cmd_done: waits for a command to finish
 * @wait_boot_done: waits for boot protocol to finish
 * @boot_stop: stop boot process
 * @set_mode: set controller parameters
 * @execute_tuning: HS200 tuning procedure
 * @caps1, caps2: capabilities 1,2 registers
 * @voltage: Operating voltate
 * @f_max: maximum available frequency
 */
struct sdhci
{
	uintptr_t ioaddr;

	int  (*init_controller)(struct sdhci *host);
	void (*send_cmd)(struct mmc *m, struct cmd *c);
	int  (*wait_cmd_done)(struct mmc *m, struct cmd *c);
	int  (*wait_boot_done)(struct mmc *m, uintptr_t *dma_addr);
	void (*boot_stop)(struct mmc *m);
	int  (*set_mode)(struct mmc *m);
	EFI_STATUS (*execute_tuning)(struct mmc *m);

	uint32_t caps1;
	uint32_t caps2;

	unsigned voltage;
	unsigned f_max;
};

extern struct sdhci *sdhci_find_controller(pcidev_t);

#define sdhci_write32(h, o, v) write32((void *)((h)->ioaddr + o), v)
#define sdhci_write16(h, o, v) write16((void *)((h)->ioaddr + o), v)
#define sdhci_write8(h, o, v)  write8((void *)((h)->ioaddr + o), v)

#define sdhci_read32(h, o) read32((void *)((h)->ioaddr + o))
#define sdhci_read16(h, o) read16((void *)((h)->ioaddr + o))
#define sdhci_read8(h, o)  read8((void *)((h)->ioaddr + o))

#endif	/* _SDHCI_H_ */
