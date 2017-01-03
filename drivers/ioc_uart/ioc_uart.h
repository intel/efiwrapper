/*
 * Copyright (c) 2017, Intel Corporation
 * All rights reserved.
 *
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

#ifndef _IOC_UART_H_
#define _IOC_UART_H_

#include <ewdrv.h>

extern ewdrv_t ioc_uart_drv;

typedef enum {
	e_ias_hardware_revision_fab_a = 0x0F,
	e_ias_hardware_revision_fab_b = 0x0E,
	e_ias_hardware_revision_fab_c = 0x0D,
	e_ias_hardware_revision_gr_fab_a = 0x07,
	e_ias_hardware_revision_gr_fab_b = 0x03,
	e_ias_hardware_revision_gr_fab_c = 0x05,
	e_ias_hardware_revision_gr_fab_d = 0x06,
	e_ias_hardware_revision_sdc_fab_a = 0x09,
	e_ias_hardware_revision_carlake_fab_a = 0x0A
}
ias_hardware_revision;

typedef enum {
	e_ias_flash_result_ok,
	e_ias_flash_result_timeout,
	e_ias_flash_result_handshake_failed,
	e_ias_flash_result_out_of_memory,
	e_ias_flash_result_parse_error,
	e_ias_flash_result_checksum_error,
	e_ias_flash_result_error
}
ias_flash_result;

#define IAS_MAX_PAYLOAD_LENGTH 255u

typedef struct ias_frame {
	uint8_t frame_counter;
	uint8_t payload_length;
	uint8_t command;
	uint8_t content[IAS_MAX_PAYLOAD_LENGTH];
	uint16_t checksum;
	uint8_t valid;
}
ias_frame;

typedef struct ias_frame_list {
	struct ias_frame_list *next;
	ias_frame frame;
}
ias_frame_list;

#endif	/* _IOC_UART_H_ */
