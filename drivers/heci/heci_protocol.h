/*
   This file contains an 'Intel Peripheral Driver' and is
   licensed for Intel CPUs and chipsets under the terms of your
   license agreement with Intel or your vendor.  This file may
   be modified by the user, subject to additional terms of the
   license agreement
*/
/*++

  Copyright (c)  1999 - 2017 Intel Corporation. All rights reserved
  This software and associated documentation (if any) is furnished
  under a license and may only be used or copied in accordance
  with the terms of the license. Except as permitted by such
  license, no part of this software or documentation may be
  reproduced, stored in a retrieval system, or transmitted in any
  form or by any means without the express written consent of
  Intel Corporation.

  --*/


/*++
  Module Name:

  Heci.h

  Abstract:

  Interface definition for EFI_HECI_PROTOCOL

  --*/

#ifndef _EFI_HECI_H_
#define _EFI_HECI_H_

#define HECI_PROTOCOL_GUID                                                \
		{ 0xcfb33810, 0x6e87, 0x4284, {0xb2, 0x3, 0xa6, 0x6a, 0xbe, 0x7, 0xf6, 0xe8 } }


/* #define EFI_HECI_PROTOCOL_GUID  HECI_PROTOCOL_GUID */

typedef struct _EFI_HECI_PROTOCOL EFI_HECI_PROTOCOL;

typedef
EFI_STATUS
(EFIAPI * EFI_HECI_SENDWACK) (
		IN OUT  uint32_t           *Message,
		IN OUT  uint32_t           Length,
		IN OUT  uint32_t           *RecLength,
		IN      uint8_t            HostAddress,
		IN      uint8_t            SECAddress
		);

typedef
EFI_STATUS
(EFIAPI * EFI_HECI_READ_MESSAGE) (
		IN      uint32_t           Blocking,
		IN      uint32_t           *MessageBody,
		IN OUT  uint32_t           *Length
		);

typedef
EFI_STATUS
(EFIAPI * EFI_HECI_SEND_MESSAGE) (
		IN      uint32_t           *Message,
		IN      uint32_t           Length,
		IN      uint8_t            HostAddress,
		IN      uint8_t            SECAddress
		);
typedef
EFI_STATUS
(EFIAPI * EFI_HECI_RESET) (
		VOID
		);

typedef
EFI_STATUS
(EFIAPI * EFI_HECI_INIT) (
		VOID
		);

typedef
EFI_STATUS
(EFIAPI * EFI_HECI_REINIT) (
		VOID
		);

typedef
EFI_STATUS
(EFIAPI * EFI_HECI_RESET_WAIT) (
		IN uint32_t   Delay
		);

typedef
EFI_STATUS
(EFIAPI * EFI_HECI_GET_SEC_STATUS) (
		IN uint32_t   *Status
		);

typedef
EFI_STATUS
(EFIAPI * EFI_HECI_GET_SEC_MODE) (
		IN uint32_t   *Mode
		);

typedef
EFI_STATUS
(EFIAPI * EFI_HECI_DISABLE_PG) (
		VOID
		);

typedef
EFI_STATUS
(EFIAPI * EFI_HECI_ENABLE_PG) (
		VOID
		);

typedef
EFI_STATUS
(EFIAPI * EFI_HECI_SUBMIT_COMMAND) (
		IN EFI_HECI_PROTOCOL  *This,
		IN uint32_t           InputParameterBlockSize,
		IN uint8_t            *InputParameterBlock,
		IN uint32_t           OutputParameterBlockSize,
		IN uint8_t            *OutputParameterBlock
		);

typedef struct _EFI_HECI_PROTOCOL {
	EFI_HECI_SENDWACK       SendwACK;
	EFI_HECI_READ_MESSAGE   ReadMsg;
	EFI_HECI_SEND_MESSAGE   SendMsg;
	EFI_HECI_RESET          ResetHeci;
	EFI_HECI_INIT           InitHeci;
	EFI_HECI_RESET_WAIT     SeCResetWait;
	EFI_HECI_REINIT         ReInitHeci;
	EFI_HECI_GET_SEC_STATUS  GetSeCStatus;
	EFI_HECI_GET_SEC_MODE    GetSeCMode;
	EFI_HECI_DISABLE_PG     DisableSeCPG;
	EFI_HECI_ENABLE_PG      EnableSeCPG;
	EFI_HECI_SUBMIT_COMMAND HeciSubmitCommand;
} EFI_HECI_PROTOCOL;


#pragma pack(1)

/* Abstract SEC Mode Definitions */
#define SEC_MODE_NORMAL        0x00

#define SEC_DEBUG_MODE_ALT_DIS 0x02
#define SEC_MODE_TEMP_DISABLED 0x03
#define SEC_MODE_SECOVER       0x04
#define SEC_MODE_FAILED        0x06

/* Abstract SEC Status definitions */
#define SEC_READY                    0x00
#define SEC_INITIALIZING             0x01
#define SEC_IN_RECOVERY_MODE         0x02
#define SEC_DISABLE_WAIT             0x06
#define SEC_TRANSITION               0x07
#define SEC_NOT_READY                0x0F
#define SEC_FW_INIT_COMPLETE         0x80
#define SEC_FW_BOOT_OPTIONS_PRESENT  0x100
#define SEC_FW_UPDATES_IN_PROGRESS   0x200

typedef struct {
	uint32_t  CodeMinor;
	uint32_t  CodeMajor;
	uint32_t  CodeBuildNo;
	uint32_t  CodeHotFix;
} SEC_VERSION_INFO;

#pragma pack()

/* HECI APIs */
extern EFI_GUID gEfiHeciProtocolGuid;

EFI_STATUS HeciHciSendMessage(uint8_t *pmsg, uint32_t MsgSize);

EFI_STATUS HeciHciRecvMessage(uint32_t *pbytesRead, uint8_t *prxBuff);

EFI_STATUS GetSeCFwVersion(SEC_VERSION_INFO *SeCVersion);

#endif /* _EFI_HECI_H_ */
