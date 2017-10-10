/** @file
  UfsPciHcPei driver is used to provide platform-dependent info, mainly UFS host controller
  MMIO base, to upper layer UFS drivers.

  Copyright (c) 2014, Intel Corporation. All rights reserved.<BR>
  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php.

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <pci/pci.h>

#include "UfsInternal.h"

/**
  Get the MMIO base address of UFS host controller.

  @param[in]  Private            A pointer to UFS_PEIM_HC_PRIVATE_DATA data structure.
  @param[in]  ControllerId       The ID of the UFS host controller.
  @param[out] MmioBar            Pointer to the UFS host controller MMIO base address.

  @retval EFI_SUCCESS            The operation succeeds.
  @retval EFI_INVALID_PARAMETER  The parameters are invalid.

**/
EFI_STATUS
EFIAPI
GetUfsHcMmioBar(
	IN     UFS_HC_PEI_PRIVATE_DATA       *Private,
	IN     UINT8                         ControllerId,
	OUT    UINTN                         *MmioBar
	)
{
	if ((Private == NULL) || (MmioBar == NULL)) {
		return EFI_INVALID_PARAMETER;
	}

	if (ControllerId >= Private->TotalUfsHcs) {
		return EFI_INVALID_PARAMETER;
	}

	*MmioBar = (UINTN)Private->UfsHcPciAddr[ControllerId];

	return EFI_SUCCESS;
}


/**
  The user code starts with this function.

  @param[out]  Private           A pointer to UFS_PEIM_HC_PRIVATE_DATA data structure.
  @param[in]   UfsHcPciBase      UFS Host Controller's PCI ConfigSpace Base address

  @retval EFI_SUCCESS            The driver is successfully initialized.
  @retval Others                 Can't initialize the driver.

**/
EFI_STATUS
EFIAPI
InitializeUfsHcPeim(
	OUT UFS_HC_PEI_PRIVATE_DATA    *Private,
	IN  pcidev_t pci_dev
	)
{
	ZeroMem(Private, sizeof(UFS_HC_PEI_PRIVATE_DATA));

	Private->Signature    = UFS_HC_PEI_SIGNATURE;
	Private->UfsHcPciAddr[Private->TotalUfsHcs] = (pci_read_config32(pci_dev, PCI_BASE_ADDRESS_0)) & (~0xf);
	DEBUG_UFS((EFI_D_VERBOSE, "UfsPciHcBase: 0x%08x\n", Private->UfsHcPciAddr[Private->TotalUfsHcs]));
	Private->TotalUfsHcs++;

	return EFI_SUCCESS;
}
