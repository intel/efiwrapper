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

#ifndef _USB_DEVICE_H_
#define _USB_DEVICE_H_

/* @USB_DEV_CONFIG_PARAMS: Struct to be filled in with configuration
 * parameters and passed to the init routine for device controller
 */
typedef struct {
	USB_CONTROLLER_ID  ControllerId; // Controller ID of the core
	UINTN              BaseAddress; // Base address of the controller registers and on-chip memory
	UINT32             Flags;        // Initialization flags
	USB_SPEED          Speed;        // Desired USB bus speed
	USB_ROLE           Role;         // Default USB role
} USB_DEV_CONFIG_PARAMS;

typedef
EFI_STATUS
(EFIAPI *USB_DEVICE_CALLBACK_FUNC) (
	IN USB_DEVICE_CALLBACK_PARAM  *Param
	);

#endif /* _USB_DEVICE_H_ */
