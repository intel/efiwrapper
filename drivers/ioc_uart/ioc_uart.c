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
#include <hwconfig.h>
#include <pci/pci.h>

#include "ioc_uart/ioc_uart.h"
#include "ioc_uart/ioc_uart_protocol.h"
#include "ewlog.h"

#ifndef IOC_USE_CBC  //default to use SLCAN
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
#define CAN_ENTER_IOC_BOOTLOADER		"T0000FFFF7FFAF5555555555\r"
#define CAN_MSG_LENGTH   			25
#else  //CBC frames
static char CBC_SUPPRES_HEART_BEAT_1MIN[]	=	{0x05,0x00,0x0E,0x04,0x60,0xEA,0x00,0x9F};
static char CBC_SUPPRES_HEART_BEAT_5MIN[]	=	{0x05,0x00,0x0E,0x04,0xE0,0x93,0x04,0x72};
static char CBC_SUPPRES_HEART_BEAT_10MIN[]	=	{0x05,0x00,0x0E,0x04,0xC0,0x27,0x09,0xF9};
static char CBC_SUPPRES_HEART_BEAT_30MIN[]	=	{0x05,0x00,0x0E,0x04,0x40,0x77,0x1B,0x17};
static char CBC_NUMBER_SUS_STAT_TOGGLES_2_HALT[]	=	{0x05,0x00,0x0E,0x02,0x00,0x05,0x00,0xE6};
static char CBC_NUMBER_SUS_STAT_TOGGLES_1_HALT[]	=	{0x05,0x00,0x0E,0x02,0x00,0x03,0x00,0xE8};
static char CBC_NUMBER_SUS_STAT_TOGGLES_2_REBOOT[]	=	{0x05,0x00,0x0E,0x02,0x00,0x06,0x00,0xE5};
static char CBC_NUMBER_SUS_STAT_TOGGLES_1_REBOOT[]	=	{0x05,0x00,0x0E,0x02,0x00,0x04,0x00,0xE7};
static char CBC_CONFIG_RESTART_SYSTEM[]	=	{0x05,0x00,0x0E,0x02,0x00,0x02,0x00,0xE9};
static char CBC_CONFIG_SHUTDOWN_SYSTEM[]	=	{0x05,0x00,0x0E,0x02,0x00,0x01,0x00,0xEA};
static char CBC_ENTER_IOC_BOOTLOADER[]	=	{0x05,0x00,0x20,0x01,0x30,0x10,0x80,0x1a,
						 0x05,0x00,0x20,0x01,0x30,0x10,0x80,0x11,
						 0x05,0x00,0x20,0x01,0x30,0x10,0x80,0x1a,
						 0x05,0x00,0x20,0x01,0x30,0x10,0x80,0x11};
#define CBC_ENTER_IOC_BOOTLOADER_MSG_NUM	4
#define CBC_MSG_LENGTH				8
#endif

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
#ifndef IOC_USE_CBC
static char can_message_buf[CAN_MSG_LENGTH + 1];
#else
static char * cbc_message_buf = NULL;
#endif
uint8_t frame_number_send, frame_number_received = 0;

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

static struct _device_params
{
	pcidev_t          cfg;	/* PCI device (configuration space) */
	unsigned char     cfio_group;	/* CFIO community */
	unsigned short    cfio_offset;	/* Rx .DW0 register offset (.DW1, Tx .DW0, .DW1 follow) */
	unsigned int      cfio_values[4];	/* to configure Rx+Tx DW0+DW1 */
} device_params = {0, NORTH, GPIO_PADBAR+0x0150, { 0x44000702, 0x304b, 0x44000700, 0x304c}};

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
	struct _device_params *p = &device_params;
	unsigned int i;
	unsigned int lcr = 0;
	unsigned int divisor;

	if (p->cfg == 0)
		pci_find_device(PCI_VENDOR_ID_INTEL, SERIAL_IOC_PCI_DID, &p->cfg);

	for (i = 0 ; i < 4 ; i += 1) {
		saved[i] = msgbus32(p->cfio_group, p->cfio_offset + 4 * i);
		msgbus32_set(p->cfio_group, p->cfio_offset + 4 * i, p->cfio_values[i]);
	}

	saved[4] = pci_read_config32(p->cfg, PCI_BASE_ADDRESS_0);
	saved[5] = pci_read_config16(p->cfg, PCI_COMMAND);

	uart_base_addr = pci_read_config32(p->cfg, PCI_BASE_ADDRESS_0) & PCI_BASE_ADDRESS_MEM_MASK;

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
	write8((void *)(UINTN)(uart_base_addr + R_UART_FCR), 0);
	write8((void *)(UINTN)(uart_base_addr + R_UART_FCR), (B_UARY_FCR_TRFIFIE | B_UARY_FCR_RESETRF | B_UARY_FCR_RESETTF));
}

/*
**  Restore the original hardware state.
*/
static void ioc_uart_restore_device(unsigned int saved[7])
{
	struct _device_params *p = &device_params;
	unsigned int to, i;

	if (p->cfg == 0)
		pci_find_device(PCI_VENDOR_ID_INTEL, SERIAL_IOC_PCI_DID, &p->cfg);

	/* Wait until the transmit FIFO is empty (ensure that they got our ACK) */
	while ((read8((void *)(UINTN)(uart_base_addr + R_UART_LSR)) & B_UART_LSR_TEMT) == 0)
		;

	/* Restore UART (M,N) clock divider */
	write32((void *)(UINTN)(uart_base_addr + R_UART_CLOCKS), saved[6]);

	/* Restore PCI configuration space registers */
	pci_write_config32(p->cfg, PCI_BASE_ADDRESS_0, saved[4]);
	pci_write_config16(p->cfg, PCI_COMMAND, saved[5]);

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

static void ioc_uart_send_data(char *s, unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		ioc_uart_send(s[i]);
	}
}

static int ioc_uart_recv(uint8_t *buffer, unsigned int len)
{
	unsigned pos = 0;
	unsigned lsr;

	for (;;) {
		lsr = read8((void *)(UINTN)(uart_base_addr + R_UART_LSR));

		/* ignore transmit status bits */
		lsr &= ~(B_UART_LSR_TXRDY | B_UART_LSR_TEMT);

		if ((lsr & ~B_UART_LSR_RXRDY) != 0) {
			/* yes, drain Rx FIFO */
			do {
				read8((void *)(UINTN)(uart_base_addr + R_UART_BAUD_THR));
				lsr = read8((void *)(UINTN)(uart_base_addr + R_UART_LSR));
			} while ((lsr & ~(B_UART_LSR_TXRDY | B_UART_LSR_TEMT)) != 0);

			return -1;
		}

		if (lsr != 0)
			buffer[pos++] = read8((void *)(UINTN)(uart_base_addr + R_UART_BAUD_THR));

		/* frame complete or time-out */
		if (pos >= len)
			return pos;
	}
}


static EFIAPI EFI_STATUS
set_suppress_heart_beat_timeout(__attribute__((__unused__)) IOC_UART_PROTOCOL *This,
		 UINT32 timeout)
{
	init_uart_ioc(old_state);
#ifndef IOC_USE_CBC
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
#else
	switch (timeout) {
		case SUPPRESS_HEART_BEAT_TIMEOUT_1_MIN:
			cbc_message_buf = CBC_SUPPRES_HEART_BEAT_1MIN;
			break;
		case SUPPRESS_HEART_BEAT_TIMEOUT_5_MIN:
			cbc_message_buf = CBC_SUPPRES_HEART_BEAT_5MIN;
			break;
		case SUPPRESS_HEART_BEAT_TIMEOUT_10_MIN:
			cbc_message_buf = CBC_SUPPRES_HEART_BEAT_10MIN;
			break;
		case SUPPRESS_HEART_BEAT_TIMEOUT_30_MIN:
		default:
			cbc_message_buf = CBC_SUPPRES_HEART_BEAT_30MIN;
			break;
	}
	ioc_uart_send_data(cbc_message_buf, CBC_MSG_LENGTH);
#endif
	ioc_uart_restore_device(old_state);

	return EFI_SUCCESS;
}
#ifndef IOC_USE_CBC
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
#else
static EFIAPI EFI_STATUS
set_ignore_sus_stat_toggles_shutdown_behaviour(EFI_IGNORE_SUS_STAT_TOGGLES num_ignore_sus_stat,EFI_CONFIGURE_SHUTDOWN_BEHAVIOUR shutdown)
{
	init_uart_ioc(old_state);
	switch (shutdown) {
		case RESTART_SYSTEM:
			if(IGNORE_SUS_STAT_3 == num_ignore_sus_stat)
				cbc_message_buf = CBC_NUMBER_SUS_STAT_TOGGLES_2_REBOOT;
			else if(IGNORE_SUS_STAT_2 == num_ignore_sus_stat)
				cbc_message_buf = CBC_NUMBER_SUS_STAT_TOGGLES_1_REBOOT;
			else
				cbc_message_buf = CBC_CONFIG_RESTART_SYSTEM;
			break;
		case SHUTDOWN_SYSTEM:
		default:
			if(IGNORE_SUS_STAT_3 == num_ignore_sus_stat)
				cbc_message_buf = CBC_NUMBER_SUS_STAT_TOGGLES_2_HALT;
			else if(IGNORE_SUS_STAT_2 == num_ignore_sus_stat)
				cbc_message_buf = CBC_NUMBER_SUS_STAT_TOGGLES_1_HALT;
			else
				cbc_message_buf = CBC_CONFIG_SHUTDOWN_SYSTEM;
			break;
	}

	ioc_uart_send_data(cbc_message_buf, CBC_MSG_LENGTH);
	ioc_uart_restore_device(old_state);

	return EFI_SUCCESS;
}
#endif

static EFIAPI EFI_STATUS
notify_ioc_cm_ready(__attribute__((__unused__)) IOC_UART_PROTOCOL *This)
{
	init_uart_ioc(old_state);
#ifndef IOC_USE_CBC
	strncpy((char *)can_message_buf, (char *)CAN_STACK_READY, CAN_MSG_LENGTH);
	ioc_uart_send_data(can_message_buf, CAN_MSG_LENGTH);
	strncpy((char *)can_message_buf, (char *)CAN_NUMBER_SUS_STAT_TOGGLES_3, CAN_MSG_LENGTH);
	ioc_uart_send_data(can_message_buf, CAN_MSG_LENGTH);
#else
#ifdef DEBUG
	printf("CBC Kmod-no handshake required\n");
#endif
#endif
	ioc_uart_restore_device(old_state);

	return EFI_SUCCESS;
}

static EFI_STATUS
ioc_reboot(EFI_RESET_TYPE ResetType, CHAR16 *ResetData)
{
	EFI_IGNORE_SUS_STAT_TOGGLES numberignoretoggles;
	size_t i;
	CHAR16 *fastboot_name[] = {L"bootloader", L"fastboot", L"recovery"};
	BOOLEAN is_fastboot = FALSE;
	EFI_STATUS ret;

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
#ifndef IOC_USE_CBC
	ret = set_ignore_sus_stat_toggles(numberignoretoggles);
	if (EFI_ERROR(ret)) {
		return ret;
	}

	ret = set_shutdown_behaviour(RESTART_SYSTEM);
#else
	ret = set_ignore_sus_stat_toggles_shutdown_behaviour(numberignoretoggles,RESTART_SYSTEM);
#endif
	return ret;
}

ias_frame init_frame(char command)
{
	ias_frame data;

	data.frame_counter = 0;
	data.payload_length = 0;
	data.command = command;
	memset(data.content, 0x0, IAS_MAX_PAYLOAD_LENGTH);//NOLINT
	data.checksum = 0;
	data.valid = 0;
	return data;
}

/*!
 * Converts given ASCII hex number to int
 * \param [in] *str - two ASCII string
 * \return integer value
 */
int c_to_i(const char *str)
{
	unsigned char device_id[2];

	device_id[0] = (int) str[0] < 58 ? (int) str[0] - 48 : tolower((int) str[0]) - 87;
	device_id[1] = (int) str[1] < 58 ? (int) str[1] - 48 : tolower((int) str[1]) - 87;

	return (unsigned int) ((device_id[0] << 4) | device_id[1]);
}

/*!
 * Send raw frame, no response for frame is expected
 * \param [in] tty_fd - file descriptor for the desired serial port
 * \param [in] *data - pointer to the frame containing data
 * \return value of write operations
 */
int send_frame_raw(ias_frame *data)
{
	int data_index = 0;
	uint8_t send_buffer[sizeof(ias_frame)];
	uint32_t send_index = 0;

	data->frame_counter = ++frame_number_send;
	data->checksum = data->frame_counter + data->payload_length + data->command;

	send_buffer[send_index++] = data->frame_counter;
	send_buffer[send_index++] = data->payload_length;
	send_buffer[send_index++] = data->command;

	for (data_index = 0; data_index < data->payload_length; data_index++) {
		send_buffer[send_index++] = data->content[data_index];
		data->checksum += data->content[data_index];
	}
	send_buffer[send_index++] = (data->checksum & 0xff);
	send_buffer[send_index++] = (data->checksum >> 8);

#ifdef DEBUG
	int a, b = 1;

	printf("------------------------------\n");
	printf("Send frame content\n");
	printf("------------------------------\n");
	printf("Counter: %"PRIu8", payload length: %"PRIu8", command: 0x%"PRIX8", checksum: %"PRIu16"\n",
			data->frame_counter, data->payload_length, data->command, data->checksum);
	for (a = 0; a < data->payload_length; a++) {
		printf(" 0x%02"PRIX8" ", data->content[a]);
		if (!(b++ % 8))
			printf("\n");
	}
	printf("\n");
#endif

	ioc_uart_send_data((char *)send_buffer, send_index);

	return 0;
}

/*!
 * Blocking function to receive a frame on UART
 * \param [in] tty_fd - file descriptor for serial port
 * \param [out] *frame - frame where received data is stored
 */
void receive_frame(ias_frame *frame, unsigned char skip_frame_counter)
{
	uint16_t checksum = 0;
	int i = 0;

	if (skip_frame_counter == 0)
		ioc_uart_recv(&frame->frame_counter, 1);
	else
		frame->frame_counter = 1;

	ioc_uart_recv(&frame->payload_length, 1);
	ioc_uart_recv(&frame->command, 1);

	if (frame->payload_length != 0)
		ioc_uart_recv(frame->content, frame->payload_length);

	ioc_uart_recv((unsigned char *) &frame->checksum, 1);
	frame->checksum <<= 8;
	ioc_uart_recv((unsigned char *) &frame->checksum, 1);

	checksum = frame->frame_counter + frame->payload_length + frame->command;
	for (i = 0; i < frame->payload_length; i++)
		checksum += frame->content[i];

#ifdef DEBUG
	int a, b = 1;

	printf("------------------------------\n");
	printf("Frame content\n");
	printf("------------------------------\n");
	printf("Counter: %"PRIu8", payload length: %"PRIu8", command: 0x%"PRIX8", checksum: %"PRIu16", checksum expected: %"PRIu16"\n",
			frame->frame_counter, frame->payload_length, frame->command, frame->checksum, checksum);
	for (a = 0; a < frame->payload_length; a++) {
		printf(" 0x%02"PRIX8" ", frame->content[a]);
		if (!(b++ % 8))
			printf("\n");
	}
	printf("\n");
#endif

	if (checksum == frame->checksum)
		frame->valid = 1;
	else
		frame->valid = 0;

	if ((frame->frame_counter == 0) && (frame->payload_length == 0) && (frame->command == 0)) {
		printf("ERROR: receive_frame - empty string received\n");
		frame->valid = 0;
	}

	frame_number_received++;
}

int send_frame(ias_frame *data)
{
	int return_value = 0;
	int attempts = 0;
	unsigned char done = 0;
	ias_frame rec_frame = init_frame(0x0);

	while (done == 0) {
		send_frame_raw(data);
		receive_frame(&rec_frame, 0);

		/* check if frame is valid and if no frame loss occurred */
		if (rec_frame.valid == 1) {
			if (rec_frame.frame_counter == frame_number_received) {
				/* acknowledge received */
				if (rec_frame.command == 0x06) {
					return_value = 0;
					done = 1;
				}
				/* not acknowledge received */
				else if (rec_frame.command == 0x07) {
					attempts++;
					printf("Retransmit was requested. Attempt: %i\n", attempts);
				} else if (rec_frame.command == 0x08) {
					done = 1;
					return_value = 3;
				} else {
					/* frame error detected */
					done = 1;
					return_value = 4;
				}
			} else {
				/* TODO */
				/* frame loss detected */
				done = 1;
				return_value = 1;
			}

		} else {
			/* TODO */
			/* frame not valid, normally send not acknowledge, but here it is already a acknowledge */
			done = 1;
			return_value = 2;
		}
	}

	return return_value;
} /* send_frame() */

ias_flash_result ias_ioc_handshake(ias_hardware_revision *hardware_revision)
{
	uint8_t received = 0;

	printf("Waiting for IOC bootloader\n");

	/* wait for charakter to receive */

	/* reset protokoll information */
	frame_number_send = 0;
	frame_number_received = 0;

	ioc_uart_recv(&received, 1);

	while (received != 0x55) {
		ioc_uart_recv(&received, 1);
	};

	if (received == 0x55) {
		printf("Bootloader request detected.\n");
		ioc_uart_send_data("\xAA", 1);


		while (received == 0x55) {
			ioc_uart_recv(&received, 1);
		};

		printf("Waiting for IOC information.\n\n");

		/* get frame first send frame with target information */
		ias_frame rec_frame = init_frame(0x0);

		receive_frame(&rec_frame, 1);
		if (rec_frame.valid == 1 && rec_frame.frame_counter == frame_number_received) {
			if (rec_frame.command == 0x01) {
				*hardware_revision = rec_frame.content[2];

				switch (*hardware_revision) {
				case e_ias_hardware_revision_fab_a:
					printf("Hardware revision: BfH - FAB A/ GR with FBL version 2.2\n");
					break;
				case e_ias_hardware_revision_fab_b:
					printf("Hardware revision: BfH - FAB B\n");
					break;
				case e_ias_hardware_revision_fab_c:
					printf("Hardware revision: BfH - FAB C\n");
					break;
				case e_ias_hardware_revision_gr_fab_a:
					printf("Hardware revision: GR FAB A/B\n");
					break;
				case e_ias_hardware_revision_gr_fab_b:
					printf("Hardware revision: GR FAB A/B\n");
					break;
				case e_ias_hardware_revision_gr_fab_c:
					printf("Hardware revision: GR FAB C\n");
					break;
				case e_ias_hardware_revision_gr_fab_d:
					printf("Hardware revision: GR FAB D\n");
					break;
				case e_ias_hardware_revision_sdc_fab_a:
					printf("Hardware revision: SDC FAB A\n");
					break;
				case e_ias_hardware_revision_carlake_fab_a:
					printf("Hardware revision: CAR LAKE FAB A\n");
					break;
				default:
					printf("Unknown hardware revision 0x%02X\n", *hardware_revision);
					break;
				}
				printf("Flash boot loader major-minor: %"PRIu8"-%"PRIu8"\n", rec_frame.content[0], rec_frame.content[1]);
				return e_ias_flash_result_ok;
			} else {
				printf("Received a frame that is illegal during handshake. No initial handshake possible. Application will exit.\n");
				return e_ias_flash_result_handshake_failed;
			}
		} else {
			printf("Frame not valid! No initial handshake possible. Application will exit.\n");
			return e_ias_flash_result_handshake_failed;
		}
	} else {
		printf("Received illegal request. Initial handshake failed.\n");
		return e_ias_flash_result_error;
	}
}

void ias_ioc_reset(void)
{
	ias_frame go_application = init_frame(0x20);

	go_application.command = 0x90;
	go_application.payload_length = 1;

	send_frame_raw(&go_application);
	printf("Restarting the IOC\n");
}

static ias_flash_result ias_flash_enter_fbl_mode(ias_hardware_revision *hardware_revision)
{
#ifndef IOC_USE_CBC
	printf("Restarting the IOC into the bootloader (slcan request)\n");
	strncpy((char *)can_message_buf, (char *)CAN_ENTER_IOC_BOOTLOADER, CAN_MSG_LENGTH);
	ioc_uart_send_data(can_message_buf, CAN_MSG_LENGTH);
#else
	unsigned int to;
	printf("Restarting the IOC into the bootloader (CBC request)\n");
	cbc_message_buf = CBC_ENTER_IOC_BOOTLOADER;
	ioc_uart_send_data(cbc_message_buf, CBC_MSG_LENGTH*CBC_ENTER_IOC_BOOTLOADER_MSG_NUM);
	to = TIMER_START(5);
	while (!TIMER_ELAPSED(to))
		;
#endif
	return ias_ioc_handshake(hardware_revision);
}

ias_flash_result ias_parse_file(uint8_t *file_content, uint32_t file_size, ias_frame_list **frame_list)
{
	uint32_t offset = 0;
	uint32_t i = 0;
	uint32_t record_length = 0;
	uint32_t line = 0;
	uint32_t calc_checksum = 0;
	uint32_t read_checksum = 0;
	uint8_t s3_record_offset = 0;

	ias_frame_list **next_frame_list_entry = frame_list;

	if (frame_list == NULL)
		return e_ias_flash_result_out_of_memory;

	while (offset < file_size) {
		++line;

		/* Search start of next record. */
		while ((offset + 1 < file_size)
				&& (file_content[offset] != 'S'))
			++offset;
		/* Check the record type (we only process S2 & S3 records). */
		if ((offset + 1 < file_size) && ((file_content[offset + 1] == '3')
					|| (file_content[offset + 1] == '2'))) {
			if (file_content[offset + 1] == '3')
				s3_record_offset = 1;
			else
				s3_record_offset = 0;

			/* Get the record length. */
			if ((offset + 4 + (s3_record_offset * 10)) < file_size) {
				record_length = c_to_i((char *)(&file_content[offset + 2]));
				if (record_length - 1 > IAS_MAX_PAYLOAD_LENGTH) {
					printf("ERROR: Payload too long (%"PRIu32")\n", record_length - 1);
					return e_ias_flash_result_parse_error;
				}
			}

			/* Ensure that a complete frame is remaining in the file. */
			if ((offset + 4 + (s3_record_offset * 3) + (2*record_length)) < file_size) {
				/* Calculate the checksum. */
				calc_checksum = 0;
				for (i = 0; i < record_length; ++i)
					calc_checksum += c_to_i((char *)(&file_content[offset + 2 + 2*i]));

				calc_checksum = (~calc_checksum) & 0x000000ff;

				/* Validate the checksum. */
				read_checksum = c_to_i((char *)(&file_content[offset + 2*record_length + 2]));
				if (calc_checksum != read_checksum) {
					printf("ERROR: Checksum mismatch in line %"PRIu32"\n", line);
					return e_ias_flash_result_checksum_error;
				}

				/* Initialize a new frame. */
				ias_frame s_frame = init_frame(0x0);

				/* Extract the address */
				s_frame.content[0] = c_to_i((char *) (&file_content[offset + 4]));
				s_frame.content[1] = c_to_i((char *) (&file_content[offset + 6]));
				s_frame.content[2] = c_to_i((char *) (&file_content[offset + 8]));
				if (s3_record_offset != 0)
					s_frame.content[3] = c_to_i((char *) (&file_content[offset + 10]));

				/* Extract data. */
				for (i = 0; i < record_length - 4; i++)
					s_frame.content[3 + s3_record_offset + i] = c_to_i((char *) (&file_content[offset + 10 + (s3_record_offset * 2) + i * 2]));

				/* Set the payload length. */
				s_frame.payload_length = record_length - 1;

				/* Add the frame to the frame list. */
				*next_frame_list_entry = (ias_frame_list *)malloc(sizeof(ias_frame_list));
				if ((*next_frame_list_entry) == NULL) {
					printf("ERROR: Allocating frame failed. Aborting!\n");
					return e_ias_flash_result_out_of_memory;
				}
				(*next_frame_list_entry)->next = NULL;
				memcpy(&(*next_frame_list_entry)->frame, &s_frame, sizeof((*next_frame_list_entry)->frame));//NOLINT

				/* Advance the next frame pointer */
				next_frame_list_entry = (ias_frame_list **) &(*next_frame_list_entry)->next;
			} else {
				printf("ERROR: Invalid S-Record, expecting more bytes than remaining\n");
				return e_ias_flash_result_parse_error;
			}
		}

		/* Skip beyond the next line break. */
		while ((offset < file_size) && (file_content[offset] != '\n'))
			++offset;

		if (offset < file_size)
			++offset;
	}

	return e_ias_flash_result_ok;
}


/*!
 * Sending whole bunch out of buffer, buffer has to be in srecord format with 128 byte lines
 * \param [in] - tty_fd - file descriptor for the desired serial port
 * \param [in] - *filecontent - pointer to buffer with file content
 * \param [in] - file_size - size of buffer
 * \param [in] - command - overwrite the default frame command
 * \return output if successful or not
 */
int flash_content(ias_frame_list const *frame_list, unsigned char command)
{
	int error = 0;
	uint32_t data_frame_counter = 1;
	uint32_t num_data_frames = 0;
	ias_frame s_frame;
	ias_frame_list const *current_frame = NULL;

	/* Count data frames. */
	current_frame = frame_list;
	while (current_frame != NULL) {
		++num_data_frames;
		current_frame = (ias_frame_list const *) current_frame->next;
	}

	/* Send data frames. */
	current_frame = frame_list;
	while ((current_frame != NULL) && (error == 0)) {
		if (((data_frame_counter % 50) == 0) ||
				(data_frame_counter == num_data_frames) ||
				(data_frame_counter < 10)) {
			if (data_frame_counter == 1)
				printf("Erasing flash.\n");
			else
				printf("Data frame number send: %4"PRIu32"|%4"PRIu32"\r", data_frame_counter, num_data_frames);
		}

		memcpy(&s_frame, &current_frame->frame, sizeof(s_frame));//NOLINT
		s_frame.command = command;

		/* Send the frame. */
		error = send_frame(&s_frame);
		if (error != 0)
			printf("Error occurred: %d\n", error);

		++data_frame_counter;
		current_frame = (ias_frame_list const *) current_frame->next;
	}
	printf("\n");

	return error;
}

int ias_flash_file(unsigned char *file_content, int file_size, unsigned char command)
{
	int error = 0;
	ias_frame_list *frame_list = NULL;
	ias_frame_list *frame_iterator = NULL;

	/* Try to parse the input file. */
	error = ias_parse_file(file_content, file_size, &frame_list);

	/* Try to flash the input file .*/
	if (error == 0) {
		error = flash_content(frame_list, command);

		if (error == 0)
			printf("\nFlashing was successful.\n");
		else if (error == 1)
			printf("An error occurred during flashing the new firmware. A frame loss was detected.\n");
		else if (error == 3)
			printf("An error occurred during flashing the new firmware. The number of retransmits exceeded the maximum or IOC detected frame loss. The IOC is in fail mode.\n");
	}

	/* Clean up allocated frames. */
	frame_iterator = frame_list;
	while (frame_iterator != NULL) {
		frame_list = frame_iterator;
		frame_iterator = (ias_frame_list *)frame_iterator->next;
		free(frame_list);
	}

	return error;
}

static EFI_STATUS EFIAPI ias_flash_ioc_firmware(__attribute__((__unused__)) IOC_UART_PROTOCOL * This,
		UINT8 *file_content,
		UINT32 file_size)

{
	ias_hardware_revision hardware_revision;

	init_uart_ioc(old_state);
	if (ias_flash_enter_fbl_mode(&hardware_revision))
		return EFI_UNSUPPORTED;

	ias_flash_file(file_content, file_size, 0x30);

	/* Start the newly flashed firmware. */
	ias_ioc_reset();

	ioc_uart_restore_device(old_state);

	return EFI_SUCCESS;
}

static EFI_GUID ioc_uart_guid = EFI_IOC_UART_PROTOCOL_GUID;
static EFI_HANDLE handle;
static EFI_RESET_SYSTEM saved_reset_rs;

static EFIAPI EFI_STATUS
ioc_reset_system(EFI_RESET_TYPE ResetType,
		 EFI_STATUS ResetStatus,
		 UINTN DataSize,
		 CHAR16 *ResetData)
{
	EFI_STATUS ret;
	ret = ioc_reboot(ResetType, ResetData);
	if (EFI_ERROR(ret)) {
		return ret;
	}

	return saved_reset_rs ? saved_reset_rs(ResetType, ResetStatus, DataSize, ResetData) : ret;
}

static EFI_STATUS ioc_uart_init(EFI_SYSTEM_TABLE *st)
{
	static IOC_UART_PROTOCOL ioc_uart_default = {
		.SetSuppressHeartBeatTimeout = set_suppress_heart_beat_timeout,
		.NotifyIOCCMReady = notify_ioc_cm_ready,
		.flash_ioc_firmware = ias_flash_ioc_firmware
	};
	IOC_UART_PROTOCOL *ioc_uart;

	saved_reset_rs = st->RuntimeServices->ResetSystem;
	st->RuntimeServices->ResetSystem = ioc_reset_system;

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

