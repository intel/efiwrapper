/*++

  Copyright (c)  1999 - 2014 Intel Corporation. All rights reserved
  This software and associated documentation (if any) is furnished
  under a license and may only be used or copied in accordance
  with the terms of the license. Except as permitted by such
  license, no part of this software or documentation may be
  reproduced, stored in a retrieval system, or transmitted in any
  form or by any means without the express written consent of
  Intel Corporation.

  --*/

#ifndef _XDCI_UTILITY_H_
#define _XDCI_UTILITY_H_

#include "dw3/UsbDeviceLib.h"

VOID
PrintDeviceDescriptor (
	IN USB_DEVICE_DESCRIPTOR    *DevDesc
	);

VOID
PrintConfigDescriptor (
	IN EFI_USB_CONFIG_DESCRIPTOR    *ConfigDesc
	);

VOID
PrintInterfaceDescriptor (
	IN EFI_USB_INTERFACE_DESCRIPTOR    *IfDesc
	);

VOID
PrintEpDescriptor (
	IN EFI_USB_ENDPOINT_DESCRIPTOR    *EpDesc
	);

VOID
PrintEpCompDescriptor (
	IN EFI_USB_ENDPOINT_COMPANION_DESCRIPTOR    *EpDesc
	);

VOID
PrintStringDescriptor (
	IN USB_STRING_DESCRIPTOR    *StrDesc
	);

VOID
PrintDeviceRequest (
	IN EFI_USB_DEVICE_REQUEST    *DevReq
	);

#ifdef SUPPORT_SUPER_SPEED
VOID
PrintBOSDescriptor (
	IN EFI_USB_BOS_DESCRIPTOR    *BosDesc
	);
#endif

#endif
