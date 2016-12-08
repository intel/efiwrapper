/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 *
 * Author: Jérémy Compostella <jeremy.compostella@intel.com>
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

#include <stdlib.h>
#include <pthread.h>
#include <ewlog.h>
#include <string.h>

#include "event.h"

typedef struct event {
	UINT32 type;
	EFI_TPL tpl;
	EFI_EVENT_NOTIFY notify;
	VOID *context;
	pthread_mutex_t lock;
	pthread_cond_t cond;
} event_t;

static EFIAPI EFI_STATUS
create_event(UINT32 Type,
	     EFI_TPL NotifyTpl,
	     EFI_EVENT_NOTIFY NotifyFunction,
	     VOID *NotifyContext,
	     EFI_EVENT *Event)
{
	event_t *event;
	int ret;

	if (!Event ||
	    (Type & EVT_NOTIFY_SIGNAL && Type & EVT_NOTIFY_WAIT) ||
	    (!NotifyFunction && (Type & EVT_NOTIFY_SIGNAL || Type & EVT_NOTIFY_WAIT)))
		return EFI_INVALID_PARAMETER;

	event = malloc(sizeof(*event));
	if (!event)
		return EFI_OUT_OF_RESOURCES;

	event->type = Type;
	event->tpl = NotifyTpl;
	event->notify = NotifyFunction;
	event->context = NotifyContext;
	if (event->type != EVT_NOTIFY_SIGNAL) {
		ret = pthread_mutex_init(&event->lock, NULL);
		if (ret)
			goto err;

		ret = pthread_cond_init(&event->cond, NULL);
		if (ret)
			goto err;
	}

	*Event = event;

	return EFI_SUCCESS;

err:
	free(event);
	return EFI_DEVICE_ERROR;

}

static EFIAPI EFI_STATUS
set_timer(__attribute__((__unused__)) EFI_EVENT Event,
	  __attribute__((__unused__)) EFI_TIMER_DELAY Type,
	  __attribute__((__unused__)) UINT64 TriggerTime)
{
	return EFI_UNSUPPORTED;
}

static EFIAPI EFI_STATUS
wait_for_event(UINTN NumberOfEvents,
	       EFI_EVENT *Event,
	       UINTN *Index)
{
	event_t *event;
	int ret;

	if (!Event || !Index)
		return EFI_INVALID_PARAMETER;

	if (NumberOfEvents != 1)
		return EFI_UNSUPPORTED;

	event = (event_t *)*Event;
	if (event->type == EVT_NOTIFY_SIGNAL)
		return EFI_INVALID_PARAMETER;

	ret = pthread_mutex_lock(&event->lock);
	if (ret)
		return EFI_DEVICE_ERROR;

        ret = pthread_cond_wait(&event->cond, &event->lock);
	if (ret)
		return EFI_DEVICE_ERROR;

        ret = pthread_mutex_unlock(&event->lock);
	if (ret)
		return EFI_DEVICE_ERROR;

	*Index = 0;

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
signal_event(EFI_EVENT Event)
{
	event_t *event = (event_t *)Event;
	int ret;

	if (!Event)
		return EFI_INVALID_PARAMETER;

	if (event->type == EVT_NOTIFY_SIGNAL) {
		event->notify(Event, event->context);
		return EFI_SUCCESS;
	}

	ret = pthread_mutex_lock(&event->lock);
	if (ret)
		return EFI_DEVICE_ERROR;

	ret = pthread_cond_signal(&event->cond);
	if (ret)
		return EFI_DEVICE_ERROR;

	ret = pthread_mutex_unlock(&event->lock);
	if (ret)
		return EFI_DEVICE_ERROR;

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
close_event(EFI_EVENT Event)
{
	free(Event);
	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
check_event(__attribute__((__unused__)) EFI_EVENT Event)
{
	return EFI_UNSUPPORTED;
}

static EFI_CREATE_EVENT saved_create_event;
static EFI_SET_TIMER saved_set_timer;
static EFI_WAIT_FOR_EVENT saved_wait_for_event;
static EFI_SIGNAL_EVENT saved_signal_event;
static EFI_CLOSE_EVENT saved_close_event;
static EFI_CHECK_EVENT saved_check_event;

static EFI_STATUS event_init(EFI_SYSTEM_TABLE *st)
{
	if (!st)
		return EFI_INVALID_PARAMETER;

	saved_create_event = st->BootServices->CreateEvent;
	saved_set_timer = st->BootServices->SetTimer;
	saved_wait_for_event = st->BootServices->WaitForEvent;
	saved_signal_event = st->BootServices->SignalEvent;
	saved_close_event = st->BootServices->CloseEvent;
	saved_check_event = st->BootServices->CheckEvent;

	st->BootServices->CreateEvent = create_event;
	st->BootServices->SetTimer = set_timer;
	st->BootServices->WaitForEvent = wait_for_event;
	st->BootServices->SignalEvent = signal_event;
	st->BootServices->CloseEvent = close_event;
	st->BootServices->CheckEvent = check_event;

	return EFI_SUCCESS;
}

static EFI_STATUS event_exit(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;

	if (!st)
		return EFI_INVALID_PARAMETER;

	st->BootServices->CreateEvent = saved_create_event;
	st->BootServices->SetTimer = saved_set_timer;
	st->BootServices->WaitForEvent = saved_wait_for_event;
	st->BootServices->SignalEvent = saved_signal_event;
	st->BootServices->CloseEvent = saved_close_event;
	st->BootServices->CheckEvent = saved_check_event;

	return EFI_SUCCESS;
}

ewdrv_t event_drv = {
	.name = "event",
	.description = "Event management for host",
	.init = event_init,
	.exit = event_exit
};
