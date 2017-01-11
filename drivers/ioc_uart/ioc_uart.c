/*
 * Copyright (c) 2017, Intel Corporation
 * All rights reserved.
 *
 * Author: jeremy.compostella@intel.com
 * Author: kui.wen@intel.com
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

#include <arch/io.h>
#include <kconfig.h>
#include <libpayload-config.h>
#include <libpayload.h>
#include <interface.h>

#include "ioc_uart/ioc_uart.h"
#include "ioc_uart/ioc_uart_protocol.h"
#include "ewlog.h"

#define CAN_STACK_READY				"T0000FFFF70A005555555555\r"
#define CAN_SUPPRES_HEART_BEAT_1MIN		"T0000FFFF701015555555555\r"
#define CAN_SUPPRES_HEART_BEAT_5MIN		"T0000FFFF701025555555555\r"
#define CAN_SUPPRES_HEART_BEAT_10MIN		"T0000FFFF701035555555555\r"
#define CAN_SUPPRES_HEART_BEAT_30MIN		"T0000FFFF701045555555555\r"
#define CAN_CONFIG_RESTART_SYSTEM		"T0000FFFF703015555555555\r"
#define CAN_CONFIG_SHUTDOWN_SYSTEM		"T0000FFFF703005555555555\r"
#define CAN_NUMBER_SUS_STAT_TOGGLES_3		"T0000FFFF705035555555555\r"
#define CAN_NUMBER_SUS_STAT_TOGGLES_2		"T0000FFFF705025555555555\r"
#define CAN_NUMBER_SUS_STAT_TOGGLES_1		"T0000FFFF705015555555555\r"
#define CAN_MSG_LENGTH   			25

#define SUPPRESS_HEART_BEAT_TIMEOUT_1_MIN	1
#define SUPPRESS_HEART_BEAT_TIMEOUT_5_MIN	5
#define SUPPRESS_HEART_BEAT_TIMEOUT_10_MIN	10
#define SUPPRESS_HEART_BEAT_TIMEOUT_30_MIN	30

#define NORTH					0xc5
#define GPIO_PADBAR               		0x500
#define RW_BARRIER()              		asm volatile ("" ::: "memory")
#define SBREG_BASE_ADDRESS        		0xF8000000  /* bar #0 */
#define _SB_MMIO_PORT_BASE(port)  		(SBREG_BASE_ADDRESS + ((port) << 16))
#define EC_BASE                   		0xE0000000
#define BDF_(b, d, f)             		((pci_device_t) (EC_BASE | ((b) << 20) | ((d) << 15) | ((f) << 12)))
#define BIT_( n)                  		(1U << (n))

#define R_UART_BAUD_THR           		0
#define R_UART_BAUD_LOW           		0
#define R_UART_BAUD_HIGH          		0x04
#define R_UART_IER                		0x04
#define R_UART_FCR                		0x08
#define B_UARY_FCR_TRFIFIE        		BIT_(0)
#define B_UARY_FCR_RESETRF        		BIT_(1)
#define B_UARY_FCR_RESETTF        		BIT_(2)
#define R_UART_LCR                		0x0C
#define B_UARY_LCR_DLAB           		BIT_(7)
#define R_UART_MCR                		0x10
#define B_UART_MCR_AFC            		0x20
#define R_UART_LSR                		0x14
#define B_UART_LSR_RXRDY          		BIT_(0)
#define B_UART_LSR_TXRDY          		BIT_(5)
#define B_UART_LSR_TEMT           		BIT_(6)
#define R_UART_MSR                		0x18
#define R_UART_SCR                		0x1C
#define R_UART_CLOCKS             		0x200     /* private clock configuration */
#define R_UART_RESETS             		0x204     /* software reset */

#define UART_CLK                  		64000000

#define UART_IPC_BASE_ADDRESS     		0xFC010000
#define BAUD_RATE                 		4000000

#define ClockCycles()             		*((volatile const uint32_t *) 0xfed000f0)  /* HPET_MCV */
#define CPMS                      		19200

#define TIMER_START(msec)         		(ClockCycles() + (msec) * CPMS)
#define TIMER_ELAPSED(tt)         		((int) (ClockCycles() - (tt)) >= 0)

static UINTN uart_base_addr;
static unsigned int old_state[7];
static char can_message_buf[CAN_MSG_LENGTH + 1];

union _pci_config_space {
	uint32_t cfg_word [16];
	/* common type 0/1 header part */
	struct _common_cfg_header {
		uint16_t vendor_id;
		uint16_t device_id;
		uint16_t command;
		uint16_t status;
		uint8_t  revision_id;
		uint8_t  class_code [3];
		uint8_t  cache_line_size;
		uint8_t  latency_timer;
		uint8_t  header_type;
		uint8_t  BIST;
	} cfg;
	struct {
		struct _common_cfg_header _common;
		uint32_t base_address [6];
		uint32_t cardbus_CIS_pointer;
		uint16_t subsystem_vendor_id;
		uint16_t subsystem_device_id;
		uint32_t expansion_ROM_base_address;
		uint8_t  capabilities_pointer;
		uint8_t  _reserved [7];
		uint8_t  interrupt_line;
		uint8_t  interrupt_pin;
		uint8_t  min_gnt;
		uint8_t  max_lat;
	} cfg0;
	struct {
		struct _common_cfg_header _common;
		uint32_t base_address [2];
		uint8_t  primary_bus_number;
		uint8_t  secondary_bus_number;
		uint8_t  subordinate_bus_number;
		uint8_t  secondary_latency_timer;
		uint8_t  io_base;
		uint8_t  io_limit;
		uint16_t secondary_status;
		uint16_t memory_base;
		uint16_t memory_limit;
		uint16_t prefetchable_memory_base;
		uint16_t prefetchable_memory_limit;
		uint32_t prefetchable_base_upper32;
		uint32_t prefetchable_limit_upper32;
		uint16_t io_base_upper16;
		uint16_t io_limit_upper16;
		uint8_t  capabilities_pointer;
		uint8_t  _reserved [3];
		uint32_t expansion_ROM_base_address;
		uint8_t  interrupt_line;
		uint8_t  interrupt_pin;
		uint16_t bridge_control;
	} cfg1;
};

typedef enum ignore_sus_stat_toggles {
	IGNORE_SUS_STAT_1,
	IGNORE_SUS_STAT_2,
	IGNORE_SUS_STAT_3
} EFI_IGNORE_SUS_STAT_TOGGLES;

typedef enum configure_shutdown_behaviour {
	RESTART_SYSTEM,
	SHUTDOWN_SYSTEM
} EFI_CONFIGURE_SHUTDOWN_BEHAVIOUR;


typedef volatile union _pci_config_space *pci_device_t;
extern size_t str16len(const CHAR16 *str);

static const struct _device_params
{
	pci_device_t      cfg;	/* PCI device (configuration space) */
	unsigned char     cfio_group;	/* CFIO community */
	unsigned short    cfio_offset;	/* Rx .DW0 register offset (.DW1, Tx .DW0, .DW1 follow) */
	unsigned int      cfio_values[4];	/* to configure Rx+Tx DW0+DW1 */
} device_params = {BDF_(0,24,1), NORTH, GPIO_PADBAR+0x0150, { 0x44000702, 0x304b, 0x44000700, 0x304c}};

static unsigned int msgbus32(unsigned int port, unsigned int reg)
{
	return read32((void *)(UINTN)(_SB_MMIO_PORT_BASE(port) + reg));
}

static void msgbus32_set(unsigned int port, unsigned int reg, unsigned int val)
{
	write32((void *)(UINTN)(_SB_MMIO_PORT_BASE(port) + reg), val);
}

static void init_uart_ioc(unsigned int saved[7])
{
	const struct _device_params *p = &device_params;
	unsigned int i;
	unsigned int lcr = 0;
	unsigned int divisor;

	for (i = 0 ; i < 4 ; i += 1) {
		saved[i] = msgbus32(p->cfio_group, p->cfio_offset + 4 * i);
		msgbus32_set(p->cfio_group, p->cfio_offset + 4 * i, p->cfio_values[i]);
	}

	saved[4] = read32((void *)(UINTN)((UINTN)p->cfg + 0x10));
	saved[5] = read16((void *)(UINTN)((UINTN)p->cfg + 0x4));

	uart_base_addr = read32((void *)(UINTN)((UINTN)p->cfg + 0x10)) & ~0xf;

	/* Take the UART out of reset, then update and enable the (M,N) clock divider. */
	write8((void *)(UINTN)(uart_base_addr + R_UART_RESETS), 0x00);
	write8((void *)(UINTN)(uart_base_addr +  R_UART_RESETS), 0x07);
	saved[6] = read32((void *)(UINTN)(uart_base_addr + R_UART_CLOCKS));
	write32((void *)(UINTN)(uart_base_addr + R_UART_CLOCKS), 0x00640081);
	write32((void *)(UINTN)(uart_base_addr +  R_UART_CLOCKS), 0x80640081);

	lcr |= 0x03;  /* 8 data bits */
	lcr &= 0xfb;  /* 1 stop bit */
	lcr &= 0xf7;  /* no parity */
	write8((void *)(UINTN)(uart_base_addr + R_UART_LCR), lcr | B_UARY_LCR_DLAB);

	divisor = UART_CLK / BAUD_RATE / 16;
	write8((void *)(UINTN)(uart_base_addr + R_UART_BAUD_HIGH), divisor >> 8);
	write8((void *)(UINTN)(uart_base_addr +  R_UART_BAUD_LOW),  divisor);

	/* switch back to bank 0 */
	write8((void *)(UINTN)(uart_base_addr + R_UART_LCR), lcr);

	/* enable & reset (receive and transmit) FIFO */
	write8((void *)(UINTN)(uart_base_addr + R_UART_MCR), B_UART_MCR_AFC);
	write8((void *)(UINTN)(uart_base_addr + R_UART_FCR), (B_UARY_FCR_TRFIFIE | B_UARY_FCR_RESETRF | B_UARY_FCR_RESETTF));
}

/*
**  Restore the original hardware state.
*/
static void ioc_uart_restore_device(unsigned int saved[7])
{
	const struct _device_params *p = &device_params;
	unsigned int to, i;

	/* Wait until the transmit FIFO is empty (ensure that they got our ACK) */
	while ((read8((void *)(UINTN)(uart_base_addr + R_UART_LSR)) & B_UART_LSR_TEMT) == 0)
		;

	/* Restore UART (M,N) clock divider */
	write32((void *)(UINTN)(uart_base_addr + R_UART_CLOCKS), saved[6]);

	/* Restore PCI configuration space registers */
	write32((void *)(UINTN)((UINTN)p->cfg + 0x10), saved[4]);
	write16((void *)(UINTN)((UINTN)p->cfg + 0x4), saved[5]);

	/* Wait a few milliconds before returning the CFIO pins to their
	original state; this avoids sending them a BREAK (i.e. an error)
	condition immediately after the acknowledge */
	to = TIMER_START(5);
	while (!TIMER_ELAPSED(to))
		;

	/* Restore CFIO pin mux */
	for (i = 0 ; i < 4; i += 1)
		msgbus32_set(p->cfio_group, p->cfio_offset + 4 * i, saved[i]);
}

/*
**  Send a single character to the IOC device.
*/
static void ioc_uart_send (int ch)
{
	unsigned lsr;
	static unsigned fifo_size = 16;

	lsr = read8((void *)(UINTN)(uart_base_addr + R_UART_LSR));
	while (fifo_size >= 16 && (lsr & B_UART_LSR_TXRDY) == 0)
		lsr = read8((void *)(UINTN)(uart_base_addr + R_UART_LSR));

	if ((lsr & B_UART_LSR_TXRDY) != 0)
		fifo_size = 0;

	write8((void *)(UINTN)(uart_base_addr + R_UART_BAUD_THR), ch);
	fifo_size += 1;
}

static void ioc_uart_send_data(const char *s, unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		ioc_uart_send(s[i]);
	}
}

static EFIAPI EFI_STATUS
set_suppress_heart_beat_timeout(__attribute__((__unused__)) IOC_UART_PROTOCOL *This,
		 UINT32 timeout)
{
	init_uart_ioc(old_state);
	switch (timeout) {
		case SUPPRESS_HEART_BEAT_TIMEOUT_1_MIN:
			strncpy((char *)can_message_buf, (char *)CAN_SUPPRES_HEART_BEAT_1MIN, CAN_MSG_LENGTH);
			break;
		case SUPPRESS_HEART_BEAT_TIMEOUT_5_MIN:
			strncpy((char *)can_message_buf, (char *)CAN_SUPPRES_HEART_BEAT_5MIN, CAN_MSG_LENGTH);
			break;
		case SUPPRESS_HEART_BEAT_TIMEOUT_10_MIN:
			strncpy((char *)can_message_buf, (char *)CAN_SUPPRES_HEART_BEAT_10MIN, CAN_MSG_LENGTH);
			break;
		case SUPPRESS_HEART_BEAT_TIMEOUT_30_MIN:
		default:
			strncpy((char *)can_message_buf, (char *)CAN_SUPPRES_HEART_BEAT_30MIN, CAN_MSG_LENGTH);
			break;
	}

	ioc_uart_send_data(can_message_buf, CAN_MSG_LENGTH);
	ioc_uart_restore_device(old_state);

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
set_shutdown_behaviour(EFI_CONFIGURE_SHUTDOWN_BEHAVIOUR shutdown)
{
	init_uart_ioc(old_state);
	switch (shutdown) {
		case RESTART_SYSTEM:
			strncpy((char *)can_message_buf, (char *)CAN_CONFIG_RESTART_SYSTEM, CAN_MSG_LENGTH);
			break;
		case SHUTDOWN_SYSTEM:
		default:
			strncpy((char *)can_message_buf, (char *)CAN_CONFIG_SHUTDOWN_SYSTEM, CAN_MSG_LENGTH);
			break;
	}

	ioc_uart_send_data(can_message_buf, CAN_MSG_LENGTH);
	ioc_uart_restore_device(old_state);

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
set_ignore_sus_stat_toggles(EFI_IGNORE_SUS_STAT_TOGGLES num_ignore_sus_stat)
{
	init_uart_ioc(old_state);
	switch (num_ignore_sus_stat) {
		case IGNORE_SUS_STAT_1:
			strncpy((char *)can_message_buf, (char *)CAN_NUMBER_SUS_STAT_TOGGLES_1, CAN_MSG_LENGTH);
			break;
		case IGNORE_SUS_STAT_2:
			strncpy((char *)can_message_buf, (char *)CAN_NUMBER_SUS_STAT_TOGGLES_2, CAN_MSG_LENGTH);
			break;
		case IGNORE_SUS_STAT_3:
			strncpy((char *)can_message_buf, (char *)CAN_NUMBER_SUS_STAT_TOGGLES_3, CAN_MSG_LENGTH);
			break;
		default:
			strncpy((char *)can_message_buf, (char *)CAN_NUMBER_SUS_STAT_TOGGLES_1, CAN_MSG_LENGTH);
			break;
	}

	ioc_uart_send_data(can_message_buf, CAN_MSG_LENGTH);
	ioc_uart_restore_device(old_state);

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
notify_ioc_cm_ready(__attribute__((__unused__)) IOC_UART_PROTOCOL *This)
{
	init_uart_ioc(old_state);
	strncpy((char *)can_message_buf, (char *)CAN_STACK_READY, CAN_MSG_LENGTH);
	ioc_uart_send_data(can_message_buf, CAN_MSG_LENGTH);
	strncpy((char *)can_message_buf, (char *)CAN_NUMBER_SUS_STAT_TOGGLES_3, CAN_MSG_LENGTH);
	ioc_uart_send_data(can_message_buf, CAN_MSG_LENGTH);
	ioc_uart_restore_device(old_state);

	return EFI_SUCCESS;
}

static void
ioc_reboot(EFI_RESET_TYPE ResetType, CHAR16 *ResetData)
{
	EFI_IGNORE_SUS_STAT_TOGGLES numberignoretoggles;
	size_t i;
	CHAR16 *fastboot_name[] = {L"bootloader", L"fastboot"};
	BOOLEAN is_fastboot = FALSE;

	if (ResetData) {
		for (i = 0; i < ARRAY_SIZE(fastboot_name); i++) {
			if ((str16len(fastboot_name[i]) == str16len(ResetData)) &&
			   !memcmp(fastboot_name[i], ResetData, str16len(ResetData) * sizeof(*ResetData))) {
				is_fastboot = TRUE;
				break;
			}
		}
	}

	if (is_fastboot ||  ResetType == EfiResetWarm)
		numberignoretoggles = IGNORE_SUS_STAT_2;
	else
		numberignoretoggles = IGNORE_SUS_STAT_1;

	set_ignore_sus_stat_toggles(numberignoretoggles);
	set_shutdown_behaviour(RESTART_SYSTEM);
}

static EFIAPI EFI_STATUS
cf9_reset_system(EFI_RESET_TYPE ResetType)
{
	UINT8 code;
	UINT8 cf9;
	UINT32 port = 0xcf9;

	if (ResetType == EfiResetShutdown)
		return EFI_UNSUPPORTED;

	switch (ResetType) {
	case EfiResetWarm:
		code = 0x06;
		break;

	case EfiResetCold:
		code = 0x0E;
		break;

	case EfiResetShutdown:
	default:
		return EFI_UNSUPPORTED;
	}

	cf9 = inb(port) & ~code;
	outb(cf9 | 2, port);
	udelay(50);

	outb(cf9 | code, port);
	udelay(500);

	return EFI_DEVICE_ERROR;
}

static EFIAPI EFI_STATUS
ioc_cf9_reset_system(EFI_RESET_TYPE ResetType,
		 __attribute__((__unused__)) EFI_STATUS ResetStatus,
		 __attribute__((__unused__)) UINTN DataSize,
		 CHAR16 *ResetData)
{
	ioc_reboot(ResetType, ResetData);

	return cf9_reset_system(ResetType);
}

static EFI_GUID ioc_uart_guid = EFI_IOC_UART_PROTOCOL_GUID;
static EFI_HANDLE handle;
static EFI_RESET_SYSTEM saved_reset_rs;

static EFI_STATUS ioc_uart_init(EFI_SYSTEM_TABLE *st)
{
	static IOC_UART_PROTOCOL ioc_uart_default = {
		.SetSuppressHeartBeatTimeout = set_suppress_heart_beat_timeout,
		.NotifyIOCCMReady = notify_ioc_cm_ready
	};
	IOC_UART_PROTOCOL *ioc_uart;

	saved_reset_rs = st->RuntimeServices->ResetSystem;
	st->RuntimeServices->ResetSystem = ioc_cf9_reset_system;

	return interface_init(st, &ioc_uart_guid, &handle,
			      &ioc_uart_default, sizeof(ioc_uart_default),
			      (void **)&ioc_uart);
}

static EFI_STATUS ioc_uart_exit(EFI_SYSTEM_TABLE *st)
{
	st->RuntimeServices->ResetSystem = saved_reset_rs;

	return interface_free(st, &ioc_uart_guid, handle);
}

ewdrv_t ioc_uart_drv = {
	.name = "ioc_uart",
	.description = "Provide reset support based on IOC CAN",
	.init = ioc_uart_init,
	.exit = ioc_uart_exit
};

