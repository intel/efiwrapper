/** @file

  Virtio Rpmb Device specific type and macro definitions corresponding to the
  virtio-0.9.5 specification.

  Copyright (C) 2012, Red Hat, Inc.
  Copyright (c) 2012 - 2018, Intel Corporation. All rights reserved.<BR>

  This program and the accompanying materials are licensed and made available
  under the terms and conditions of the BSD License which accompanies this
  distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS, WITHOUT
  WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef _VIRTIO_RPMB_H_
#define _VIRTIO_RPMB_H_

#include "Virtio.h"

#define VIRTIO_RPMB_S_OK        0x00
#define VIRTIO_RPMB_S_IOERR     0x01
#define VIRTIO_RPMB_S_UNSUPP    0x02

#endif // _VIRTIO_RPMB_H_
