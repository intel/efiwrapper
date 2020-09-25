/*
 * Copyright (c) 2017, Intel Corporation
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
 */


#include <stdint.h>

#include <arch/io.h>
#include <kconfig.h>
#include <libpayload-config.h>
#include <libpayload.h>
#include <interface.h>

#include <efi.h>
#include <ewdrv.h>

#include "ewlog.h"
#include "heci_impl.h"
#include "heci_protocol.h"
#include "heci2_protocol.h"

#define UNUSED_PARAM        __attribute__((__unused__))

#define CPMS                19200
#define HPET_BASE_ADDRESS   0xFED00000
#define ClockCycles()       read32((void *)(HPET_BASE_ADDRESS + 0xf0))

static EFI_HECI_PROTOCOL *heci;

static void init_timer(void)
{
	uint32_t reg;
	/* Clear HPET Timer 0 Lower and Upper Comparator Value. */
	write32((void *)(HPET_BASE_ADDRESS + 0x108), 0);
	write32((void *)(HPET_BASE_ADDRESS + 0x10c), 0);

	/* Enable HPET main counter. */
	reg = read32((void *)(HPET_BASE_ADDRESS + 0x10));
	write32((void *)(HPET_BASE_ADDRESS + 0x10c), reg | 1);
}

static int wait_event(uint32_t timeout, int (*fun)(uint32_t), uint32_t arg)
{
	init_timer();
	uint32_t t0 = ClockCycles();
	uint32_t elapsed;
	int res;

	timeout *= CPMS;
	for (;;) {
		if (fun != NULL) {
			res = fun(arg);
			if (res > 0)
				return res;
		}

		if (!timeout)
			continue;

		elapsed = ClockCycles() - t0;
		if (elapsed >= timeout)
			return -1;
	}

	return 0;
}

static uint8_t heci_pci_read8(uint32_t reg)
{
	return (uint8_t)read8((void *)(HECI_PCI_DEV + reg));
}

static uint32_t heci_pci_read32(uint32_t reg)
{
	return read32((void *)(HECI_PCI_DEV + reg));
}

static void heci_pci_write32(uint32_t reg, uint32_t val)
{
	write32((void *)(HECI_PCI_DEV + reg), val);
}

static void heci_pci_set16(uint32_t reg, uint32_t val)
{
	uint16_t v;

	v = read16((void *)(HECI_PCI_DEV + reg));
	write16((void *)(HECI_PCI_DEV + reg), v | val);
}

static uint32_t heci_reg_read(uint32_t base, uint32_t offset)
{
	return read32((void *)(UINTN)(base + offset));
}

static void heci_reg_write(uint32_t base, uint32_t offset, uint32_t val)
{
	write32((void *)(UINTN)(base + offset), val);
}

static int is_dev_ready(uint32_t base)
{
	DEV_CTRL_REG reg;

	reg.data = heci_reg_read(base, SEC_CSR_HA);
	if (reg.bit.SEC_RDY_HRA == 0)
		return 0;

	return 1;
}

/* wait for dev ready */
static int is_dev_data_ready(uint32_t base)
{
	DEV_CTRL_REG reg;

	reg.data = heci_reg_read(base, SEC_CSR_HA);
	if (reg.bit.SEC_CBRP_HRA == reg.bit.SEC_CBWP_HRA)
		return 0;

	return 1;
}

static int is_interrupt_raised(uint32_t base)
{
	HOST_CTRL_REG reg;

	reg.data = heci_reg_read(base, H_CSR);
	if (reg.bit.H_IS == 0)
		return 0;

	return 1;
}

static int is_host_ready(uint32_t base)
{
	HOST_CTRL_REG reg;

	reg.data = heci_reg_read(base, H_CSR);
	if (reg.bit.H_RDY == 1)
		return 0;

	return  1;
}

/*
 * Verify that the HECI cmd and MBAR regs in its PCI cfg space are setup
 * properly and that the local mHeciContext variable matches this info.
 */
static uint32_t heci_get_base_addr(void)
{
	uint32_t u32HeciBase;
	uint32_t val;
	uint32_t mask;

	/* Read HECI_MBAR in case it has changed */
	val = heci_pci_read32(HECI_MBAR0) & 0xFFFFFFF0;

	heci_pci_write32(HECI_MBAR0, 0xFFFFFFFF);
	u32HeciBase = heci_pci_read32(HECI_MBAR0) & 0xFFFFFFF0;
	heci_pci_write32(HECI_MBAR0, val);
	u32HeciBase = heci_pci_read32(HECI_MBAR0) & 0xFFFFFFF0;
	ewdbg("HeciMemBase=%x\n", u32HeciBase);

	/* Check if HECI_MBAR is disabled */
	mask = (EFI_PCI_COMMAND_MEMORY_SPACE | EFI_PCI_COMMAND_BUS_MASTER);
	if ((heci_pci_read8(PCI_COMMAND_OFFSET) & mask) != mask)
		heci_pci_set16(PCI_COMMAND_OFFSET, mask | EFI_PCI_COMMAND_SERR);

	return u32HeciBase;
}


/*
 * Send HECI message implement
 */
static int heci_send_impl(uint32_t u32HeciBase, uint32_t *Message, uint32_t Length, uint8_t HostAddress, uint8_t DevAddr)
{
	uint32_t LeftSize;
	uint32_t MaxBuffer;
	uint32_t WriteSize;
	uint32_t Size;
	uint32_t Index;
	HECI_MSG_HDR head;
	HOST_CTRL_REG hcr;

	if (Message == NULL || Length == 0)
		return 1;

	ewdbg("heci_send Start\n");
	hcr.data = heci_reg_read(u32HeciBase, H_CSR);
	MaxBuffer = hcr.bit.H_CBD;

	/* The first DWORD used for send MessageHeader, so useable Buffer Size
	 * should be MaxBuffer -1;
	 */
	MaxBuffer -= 1;
	LeftSize = (Length + 3)/4;
	WriteSize = 0;
	hcr.bit.H_RDY = 1;
	heci_reg_write(u32HeciBase, H_CSR, hcr.data);
	while (LeftSize > 0) {
		ewdbg("Wait DEV, CSR %x\n", heci_reg_read(u32HeciBase, SEC_CSR_HA));

		if (wait_event(HECI_EVENT_TIMEOUT, is_dev_ready, u32HeciBase) < 0) {
			ewerr("Timeout waiting heci device\n");
			return 1;
		}

		hcr.data = heci_reg_read(u32HeciBase, H_CSR);
		hcr.bit.H_RDY = 1;
		hcr.bit.H_IE = 0;
		heci_reg_write(u32HeciBase, H_CSR, hcr.data);
		Size = (LeftSize > MaxBuffer) ? MaxBuffer : LeftSize;

		LeftSize -= Size;

		/* Prepare message header */
		head.data = 0;
		head.bit.sec_address = DevAddr;
		head.bit.host_address = HostAddress;
		head.bit.message_complete = (LeftSize > 0) ? 0 : 1;
		if (LeftSize > 0)
			head.bit.length = Size * sizeof(uint32_t);
		else
			head.bit.length = Length - WriteSize * sizeof(uint32_t);

		ewdbg("heci Message Header: %08x\n", head.data);
		heci_reg_write(u32HeciBase, H_CB_WW, head.data);
		for (Index = 0; Index < Size; Index++) {
			heci_reg_write(u32HeciBase, H_CB_WW, Message[Index + WriteSize]);
		}

		/* Send the Interrupt; */
		hcr.data = heci_reg_read(u32HeciBase, H_CSR);
		hcr.bit.H_IS = 1;
		hcr.bit.H_RDY = 1;
		hcr.bit.H_IE = 0;
		hcr.bit.H_IG = 1;
		heci_reg_write(u32HeciBase, H_CSR, hcr.data);

		WriteSize += Size;
		if (LeftSize > 0) {
			ewdbg("HostControlReg %x\n", heci_reg_read(u32HeciBase, SEC_CSR_HA));

			if (wait_event(HECI_EVENT_TIMEOUT, is_interrupt_raised, u32HeciBase)) {
				ewerr("Timeout waiting interrupt\n");
				return 1;
			}
		}
	}
	ewdbg("heci_send End\n");
	return 0;
}

static int heci_send(uint32_t *Message, uint32_t Length, uint8_t HostAddress, uint8_t DevAddr)
{
	int ret;
	uint32_t u32HeciBase;

	u32HeciBase = heci_get_base_addr();
	if (u32HeciBase == 0)
		return 1;

	ret = heci_send_impl(u32HeciBase, Message, Length, HostAddress, DevAddr);

	return ret;
}

/*
 * Receive HECI message
 */
static int heci_receive_impl(uint32_t u32HeciBase, uint32_t *Message, uint32_t *Length)
{
	uint32_t ReadSize = 0;
	uint32_t Index;
	uint32_t BufSize = 0;
	uint32_t value;
	HECI_MSG_HDR head;

	HOST_CTRL_REG hcr;

	ewdbg("heci_receive Start\n");

	if (Length != NULL)
		BufSize = *Length;
	while (1) {
		hcr.data = heci_reg_read(u32HeciBase, H_CSR);
		hcr.bit.H_RDY = 1;
		hcr.bit.H_IE = 0;
		heci_reg_write(u32HeciBase, H_CSR, hcr.data);
		ewdbg("Disable Interrupt, HCR: %08x\n", heci_reg_read(u32HeciBase, H_CSR));

		if (wait_event(HECI_EVENT_TIMEOUT, is_dev_data_ready, u32HeciBase) < 0)
			goto rx_failed;

		head.data = heci_reg_read(u32HeciBase, SEC_CB_RW);
		ewdbg("Get Message Header: %08x\n", head.data);
		for (Index = 0; Index < (uint32_t)((head.bit.length + 3)/4); Index++) {
			if (wait_event(HECI_EVENT_TIMEOUT, is_dev_data_ready, u32HeciBase) < 0)
				goto rx_failed;

			value = heci_reg_read(u32HeciBase, SEC_CB_RW);
			ewdbg("heci data[%x] = %08x\n", Index, value);
			if (Message != NULL && (BufSize == 0 || BufSize >= (uint32_t)(Index * 4)))
				Message[Index + ReadSize] = value;
		}

		if (Length != NULL) {
			if ((uint32_t)(Index * 4) > head.bit.length)
				*Length = head.bit.length;
			else
				*Length = (uint32_t)(Index * 4);
		}

		hcr.data = heci_reg_read(u32HeciBase, H_CSR);
		hcr.bit.H_IS = 1;
		hcr.bit.H_RDY = 1;
		hcr.bit.H_IE = 0;
		hcr.bit.H_IG = 1;
		heci_reg_write(u32HeciBase, H_CSR, hcr.data);
		if (head.bit.message_complete == 1)
			break;
	}
	ewdbg("heci_receive End\n");
	return 0;

rx_failed:
	ewerr("Timeout during data recv\n");
	return 1;
}

static int heci_receive(uint32_t *Message, uint32_t *Length)
{
	int ret;

	uint32_t u32HeciBase;

	u32HeciBase = heci_get_base_addr();
	if (u32HeciBase == 0)
		return 1;

	ret = heci_receive_impl(u32HeciBase, Message, Length);
	return ret;
}

/*
 * Reset HECI interface
 */
static int heci_reset_interface(void)
{
	HOST_CTRL_REG hcr;
	uint32_t u32HeciBase;

	u32HeciBase = heci_get_base_addr();
	if (u32HeciBase == 0)
		return 1;

	hcr.data = heci_reg_read(u32HeciBase, H_CSR);
	hcr.bit.H_IG = 1;
	hcr.bit.H_IS = 1;
	hcr.bit.H_RST = 1;
	ewdbg("Assert HECI interface reset\n");
	heci_reg_write(u32HeciBase, H_CSR, hcr.data);

	/* Make sure that the reset started */
	if (wait_event(HECI_RESET_TIMEOUT, is_host_ready, u32HeciBase) < 0) {
		ewerr("Timeout waiting reset\n");
		return 1;
	}

	/* Wait for SEC to perform reset */
	if (wait_event(HECI_RESET_TIMEOUT, is_dev_ready, u32HeciBase) < 0) {
		ewerr("Timeout waiting dev ready\n");
		return 1;
	}

	/* Make sure IS has been signaled on the HOST side */
	if (wait_event(HECI_RESET_TIMEOUT, is_interrupt_raised, u32HeciBase) < 0) {
		ewerr("Timeout waiting interrupt\n");
		return 1;
	}

	hcr.data = heci_reg_read(u32HeciBase, H_CSR);
	hcr.bit.H_RST = 0;
	hcr.bit.H_IG = 1;
	hcr.bit.H_RDY = 1;
	/* Enable host side interface */
	heci_reg_write(u32HeciBase, H_CSR, hcr.data);

	ewdbg("Reset HECI interface done\n");
	return 0;
}

/*
 * Reset HECI Interface
 */
static int heci_issue_reset(_Bool *reset_status)
{
	uint32_t status;
	uint32_t u32HeciBase;
	DEV_CTRL_REG DevCtrlReg;

	u32HeciBase = heci_get_base_addr();
	if (u32HeciBase == 0)
		return -1;

	if (reset_status != NULL)
		*reset_status = 0;

	DevCtrlReg.data = heci_reg_read(u32HeciBase, SEC_CSR_HA);
	if (DevCtrlReg.bit.SEC_RST_HRA == 1) {
		status = heci_reset_interface();
		if (status) {
			ewerr("Reset HECI failed: %d\n", status);
		} else {
			if (reset_status != NULL)
				*reset_status = 1;
		}
	}

	return 0;
}

static int heci_reset(void)
{
	return heci_issue_reset(NULL);
}

static EFI_STATUS EFIAPI HeciSendwACK(
		uint32_t           *Message,
		uint32_t           Length,
		uint32_t           *RecLength,
		uint8_t            HostAddress,
		uint8_t            SECAddress
		)
{
	int status = 0;
	uint32_t u32HeciBase;

	u32HeciBase = heci_get_base_addr();
	if (u32HeciBase == 0)
		return EFI_DEVICE_ERROR;

	heci_reset_interface();

	/* Send the message */
	status = heci_send_impl(u32HeciBase, Message, Length, HostAddress, SECAddress);
	if (status)
		return EFI_DEVICE_ERROR;

	/* Wait for ACK message */
	status = heci_receive_impl(u32HeciBase, Message, RecLength);
	if (status)
		return EFI_DEVICE_ERROR;

	return EFI_SUCCESS;
}

static EFI_STATUS EFIAPI HeciSendMsg(
		uint32_t         *Message,
		uint32_t         Length,
		uint8_t          HostAddress,
		uint8_t          SECAddress)
{
	int status = 0;

	status = heci_send(Message, Length, HostAddress, SECAddress);
	if (status)
		return EFI_DEVICE_ERROR;

	return EFI_SUCCESS;
}

static EFI_STATUS EFIAPI HeciReadMsg(
		UNUSED_PARAM uint32_t  Blocking,
		uint32_t  *MessageBody,
		uint32_t  *Length)
{
	int status = 0;

	status = heci_receive(MessageBody, Length);
	if (status)
		return EFI_DEVICE_ERROR;

	return EFI_SUCCESS;
}

static EFI_STATUS EFIAPI HeciResetHeci(VOID)
{
	int status = 0;

	status = heci_reset();
	if (status)
		return EFI_DEVICE_ERROR;

	return EFI_SUCCESS;
}


static EFI_STATUS EFIAPI HeciInitHeci(VOID)
{
	return HeciResetHeci();
}

static EFI_STATUS EFIAPI HeciReInitHeci(VOID)
{
	return HeciResetHeci();
}

static EFI_STATUS EFIAPI HeciGetSeCMode(uint32_t *Mode)
{
	uint32_t reg;
	uint32_t mode;

	reg = heci_pci_read32(HECI_SEC_FW_STS0);
	/* bit 16:19 - Management Engine Current Operation Mode */
	mode = (reg >> 16) & 0x0f;
	*Mode = mode;

	return EFI_SUCCESS;
}

static EFI_STATUS EFIAPI HeciSeCResetWait(UNUSED_PARAM uint32_t Delay)
{
	return EFI_UNSUPPORTED;
}

static EFI_STATUS EFIAPI HeciGetSeCStatus(UNUSED_PARAM uint32_t *Status)
{
	return EFI_UNSUPPORTED;
}

static EFI_STATUS EFIAPI HeciDisableSeCPG(VOID)
{
	return EFI_UNSUPPORTED;
}

static EFI_STATUS EFIAPI HeciEnableSeCPG(VOID)
{
	return EFI_UNSUPPORTED;
}

static EFI_STATUS EFIAPI HeciSubmitCommand(
		UNUSED_PARAM EFI_HECI_PROTOCOL   *This,
		UNUSED_PARAM uint32_t            InputParameterBlockSize,
		UNUSED_PARAM uint8_t             *InputParameterBlock,
		UNUSED_PARAM uint32_t            OutputParameterBlockSize,
		UNUSED_PARAM uint8_t             *OutputParameterBlock
		)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
heci2_send_w_ack(UNUSED_PARAM HECI2_DEVICE HeciDev,
		 UINT32 *Message,
		 UINT32 Length,
		 UINT32 *RecLength,
		 UINT8 HostAddress,
		 UINT8 MEAddress)
{
	return heci->SendwACK(Message, Length, RecLength, HostAddress, MEAddress);
}

static EFIAPI EFI_STATUS
heci2_read_msg(UNUSED_PARAM HECI2_DEVICE HeciDev,
	       UINT32 Blocking,
	       UINT32 *MessageBody,
	       UINT32 *Length)
{
	return heci->ReadMsg(Blocking, MessageBody, Length);
}

static EFIAPI EFI_STATUS
heci2_send_msg(UNUSED_PARAM HECI2_DEVICE HeciDev,
	       UINT32 *Message,
	       UINT32 Length,
	       UINT8 HostAddress,
	       UINT8 MEAddress)
{
	return heci->SendMsg(Message, Length, HostAddress, MEAddress);
}

static EFIAPI EFI_STATUS
heci2_reset_heci(UNUSED_PARAM HECI2_DEVICE HeciDev)
{
	return heci->ResetHeci();
}

static EFIAPI EFI_STATUS
heci2_init_heci(UNUSED_PARAM HECI2_DEVICE HeciDev)
{
	return heci->InitHeci();
}

static EFIAPI EFI_STATUS
heci2_me_reset_wait(UNUSED_PARAM HECI2_DEVICE HeciDev,
		    UINT32 Delay)
{
	return heci->SeCResetWait(Delay);
}

static EFIAPI EFI_STATUS
heci2_re_init_heci(UNUSED_PARAM HECI2_DEVICE HeciDev)
{
	return heci->ReInitHeci();
}

static EFIAPI EFI_STATUS
heci2_get_me_status(UNUSED_PARAM UINT32 *Status)
{
	return heci->GetSeCStatus(Status);
}

static EFIAPI EFI_STATUS
heci2_get_me_mode(UINT32 *Mode)
{
	return heci->GetSeCMode(Mode);
}

static EFI_GUID heci_guid = HECI_PROTOCOL_GUID;
static EFI_HANDLE handle;
static EFI_GUID heci2_guid = EFI_HECI2_PROTOCOL_GUID;
static EFI_HANDLE handle2;

static EFI_STATUS heci_init(EFI_SYSTEM_TABLE * st)
{
	static EFI_HECI_PROTOCOL heci_default = {
			.SendwACK = HeciSendwACK,
			.ReadMsg = HeciReadMsg,
			.SendMsg = HeciSendMsg,
			.ResetHeci = HeciResetHeci,
			.InitHeci = HeciInitHeci,
			.SeCResetWait = HeciSeCResetWait,
			.ReInitHeci = HeciReInitHeci,
			.GetSeCStatus = HeciGetSeCStatus,
			.GetSeCMode = HeciGetSeCMode,
			.DisableSeCPG = HeciDisableSeCPG,
			.EnableSeCPG = HeciEnableSeCPG,
			.HeciSubmitCommand = HeciSubmitCommand,
	};
	static struct EFI_HECI2_PROTOCOL_ heci2_default = {
		.SendwACK = heci2_send_w_ack,
		.ReadMsg = heci2_read_msg,
		.SendMsg = heci2_send_msg,
		.ResetHeci = heci2_reset_heci,
		.InitHeci = heci2_init_heci,
		.MeResetWait = heci2_me_reset_wait,
		.ReInitHeci = heci2_re_init_heci,
		.GetMeStatus = heci2_get_me_status,
		.GetMeMode = heci2_get_me_mode
	};
	EFI_STATUS ret;
	EFI_HECI2_PROTOCOL *heci2_drv;

	ret = interface_init(st, &heci_guid, &handle,
			     &heci_default, sizeof(heci_default),
			     (void **)&heci);
	if (EFI_ERROR(ret)) {
		ewerr("Failed to register HECI protocol");
		return ret;
	}

	ret = interface_init(st, &heci2_guid, &handle2,
			     &heci2_default, sizeof(heci2_default),
			     (void **)&heci2_drv);
	if (EFI_ERROR(ret)) {
		ewerr("Failed to register HECI2 protocol");
		interface_free(st, &heci_guid, handle);
	}

	return ret;
}

static EFI_STATUS heci_exit(EFI_SYSTEM_TABLE * st)
{
	return interface_free(st, &heci_guid, handle);
}

ewdrv_t heci_drv = {
	.name = "heci_driver",
	.description = "heci_driver",
	.init = heci_init,
	.exit = heci_exit
};


