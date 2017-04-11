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

#include <interface.h>
#include <efiwrapper.h>
#include <stdbool.h>
#include <pthread.h>
#include <string.h>
#include <ewlog.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>

#include "gop.h"

typedef struct gop {
	EFI_GRAPHICS_OUTPUT_PROTOCOL prot;
	bool running;
	Display *display;
	GC gc;
	Window win;
	Window root;
	XImage *img;
	pthread_t thread;
	pthread_cond_t ready;
	pthread_mutex_t lock;
} gop_t;

static const KeySym EXIT_KEYSYM = XK_End;
static EFI_GUID gop_guid = EFI_GRAPHICS_OUTPUT_PROTOCOL_GUID;
static EFI_HANDLE gop_handle;
static gop_t *gop;

static EFI_GRAPHICS_OUTPUT_MODE_INFORMATION info = {
	.Version = 1,
	.HorizontalResolution = 768,
	.VerticalResolution = 1024,
	.PixelFormat = PixelRedGreenBlueReserved8BitPerColor
};

static EFI_GRAPHICS_OUTPUT_PROTOCOL_MODE mode = {
	.MaxMode = 1,
	.Info = &info,
	.SizeOfInfo = sizeof(info)
};

static EFIAPI EFI_STATUS
query_mode(EFI_GRAPHICS_OUTPUT_PROTOCOL *This,
	   UINT32 ModeNumber,
	   UINTN *SizeOfInfo,
	   EFI_GRAPHICS_OUTPUT_MODE_INFORMATION **Info)
{
	if (!This || ModeNumber != 0 || !SizeOfInfo || !Info)
		return EFI_INVALID_PARAMETER;

	*SizeOfInfo = sizeof(info);
	*Info = &info;

	return EFI_SUCCESS;
}

static inline EFI_STATUS gop_lock(void)
{
	int ret;

	ret = pthread_mutex_lock(&gop->lock);
	if (ret) {
		ewerr("Failed to lock the GOP lock");
		return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}

static inline EFI_STATUS gop_unlock(void)
{
	int ret;

	ret = pthread_mutex_unlock(&gop->lock);
	if (ret) {
		ewerr("Failed to unlock the GOP lock");
		return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}

static EFI_STATUS wait_for_readiness(void)
{
	EFI_STATUS ret;
	int pret;

	ret = gop_lock();
	if (EFI_ERROR(ret))
		return ret;

        pret = pthread_cond_wait(&gop->ready, &gop->lock);
	if (ret)
		return EFI_DEVICE_ERROR;

	return gop_unlock();
}

static EFI_STATUS notify_is_ready(void)
{
	EFI_STATUS ret;
	int pret;

	ret = gop_lock();
	if (EFI_ERROR(ret))
		return ret;

	pret = pthread_cond_signal(&gop->ready);
	if (ret)
		return EFI_DEVICE_ERROR;

	return gop_unlock();
}

static XVisualInfo *get_visual_info(Display *display, int depth, int class)
{
	XVisualInfo template;
	int nitems;

	memset(&template, 0, sizeof(template));
	template.screen = DefaultScreen(display);
	template.class = class;
	template.depth = depth;

	return XGetVisualInfo(display, VisualScreenMask,
			      &template, &nitems);
}

static inline void put_image(void)
{
	XPutImage(gop->display, gop->win, gop->gc, gop->img,
		  0, 0, 0, 0,
		  info.HorizontalResolution,
		  info.VerticalResolution);
}

static void *draw(void *arg)
{
	EFI_STATUS ret;
	XVisualInfo *vi;
	XSetWindowAttributes attr;
	XEvent event;
	KeyCode exit_keycode;
	void *data;

	gop->display = XOpenDisplay(NULL);
	if (!gop->display) {
		ewerr("Failed to open X display");
		return NULL;
	}

	vi = get_visual_info(gop->display, 24, TrueColor);
	if (!vi) {
		ewerr("Failed to get appropriate visual info");
		return NULL;
	}

	gop->root = DefaultRootWindow(gop->display);

	attr.event_mask = ExposureMask | KeyPressMask;
	attr.colormap = XCreateColormap(gop->display, gop->root, vi->visual,
					AllocNone);
	gop->win = XCreateWindow(gop->display, gop->root, 0, 0,
				 info.HorizontalResolution,
				 info.VerticalResolution,
				 0, vi->depth,
				 InputOutput, vi->visual,
				 CWEventMask  | CWColormap, &attr);
	XStoreName(gop->display, gop->win, "EfiWrapper");
	XMapWindow(gop->display, gop->win);

	gop->gc = XCreateGC(gop->display, gop->win, 0, 0);
	exit_keycode = XKeysymToKeycode(gop->display, EXIT_KEYSYM);

	for (;;) {
		XNextEvent(gop->display, &event);

		if (event.type == Expose && event.xexpose.count == 0) {
			if (!gop->running) {
				data = malloc(sizeof(unsigned int) *
					      info.HorizontalResolution *
					      info.VerticalResolution);
				if (!data)
					return NULL;
				gop->img = XCreateImage(gop->display, CopyFromParent,
							vi->depth, ZPixmap, 0, data,
							info.HorizontalResolution,
							info.VerticalResolution,
							32, 0);
				if (!gop->img) {
					ewerr("Failed to create XImage");
					break;
				}
				gop->running = true;
				notify_is_ready();
				continue;
			}

			ret = gop_lock();
			if (EFI_ERROR(ret))
				break;

			put_image();

			ret = gop_unlock();
			if (EFI_ERROR(ret))
				break;
		} else if (event.type == KeyPress &&
			   event.xkey.keycode == exit_keycode)
			break;
	}


	ret = gop_lock();
	if (EFI_ERROR(ret))
		return NULL;

	if (gop->img) {
		XDestroyImage(gop->img);
		gop->img = NULL;
	}
	XUnmapWindow(gop->display, gop->win);
	XDestroyWindow(gop->display, gop->win);
	XFree(vi);
	XCloseDisplay(gop->display);

	ret = gop_unlock();
	if (EFI_ERROR(ret))
		return NULL;

	return NULL;
}

static EFI_STATUS stop_gop(void)
{
	Display *display;
	void *retval;
	int ret;
	XKeyEvent event = {
		.subwindow = None,
		.time = CurrentTime,
		.x = 1,
		.y = 1,
		.x_root = 1,
		.y_root = 1,
		.same_screen = True,
		.state = 0,
		.type = KeyPress
	};
	Status status;

	if (!gop || !gop->running)
		return EFI_SUCCESS;

	display = XOpenDisplay(NULL);
	if (!display)
		return EFI_DEVICE_ERROR;

	event.display = display;
	event.window = gop->win;
	event.root = gop->root;
	event.keycode = XKeysymToKeycode(gop->display, EXIT_KEYSYM);

	status = XSendEvent(event.display, event.window, True, KeyPressMask,
			    (XEvent *)&event);
	if (!status)
		return EFI_DEVICE_ERROR;

	XCloseDisplay(display);

	ret = pthread_join(gop->thread, &retval);
	if (ret) {
		ewerr("Failed to join gop thread, %s", strerror(ret));
		return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}

static EFIAPI EFI_STATUS
set_mode(EFI_GRAPHICS_OUTPUT_PROTOCOL *This,
	 UINT32 ModeNumber)
{
	EFI_STATUS ret;
	int pret;

	if (!This || (gop_t *)This != gop || ModeNumber != 0)
		return EFI_INVALID_PARAMETER;

	ret = stop_gop();
	if (EFI_ERROR(ret)) {
		ewerr("Failed to stop GOP thread");
		return ret;
	}

	gop->running = false;

	pret = pthread_mutex_init(&gop->lock, NULL);
	if (pret)
		return EFI_DEVICE_ERROR;

	pret = pthread_cond_init(&gop->ready, NULL);
	if (pret)
		return EFI_DEVICE_ERROR;

	XInitThreads();
	pret = pthread_create(&gop->thread, NULL, draw, NULL);
	if (pret)
		return EFI_DEVICE_ERROR;

	return wait_for_readiness();
}

static void fill_buffer(EFI_GRAPHICS_OUTPUT_BLT_PIXEL *color,
			UINTN destx,
			UINTN desty,
			UINTN width,
			UINTN height)
{
	size_t x, y;
	EFI_GRAPHICS_OUTPUT_BLT_PIXEL *data;

	data = (EFI_GRAPHICS_OUTPUT_BLT_PIXEL *)gop->img->data;
	for (y = desty; y < desty + height; y++)
		for (x = destx; x < destx + width; x++)
			data[y * info.HorizontalResolution + x] = *color;
}

static void copy_to_buffer(EFI_GRAPHICS_OUTPUT_BLT_PIXEL *buffer,
			   UINTN srcx,
			   UINTN srcy,
			   UINTN destx,
			   UINTN desty,
			   UINTN width,
			   UINTN height)
{
	size_t y, sy;
	EFI_GRAPHICS_OUTPUT_BLT_PIXEL *data;

	data = (EFI_GRAPHICS_OUTPUT_BLT_PIXEL *)gop->img->data;
	for (y = desty, sy = srcy; y < desty + height; y++, sy++)
		memcpy(&data[y * info.HorizontalResolution + destx],
		       &buffer[sy * width + srcx],
		       width * sizeof(*buffer));
}

static EFIAPI EFI_STATUS
blt(EFI_GRAPHICS_OUTPUT_PROTOCOL *This,
    EFI_GRAPHICS_OUTPUT_BLT_PIXEL *BltBuffer,
    EFI_GRAPHICS_OUTPUT_BLT_OPERATION BltOperation,
    UINTN SourceX,
    UINTN SourceY,
    UINTN DestinationX,
    UINTN DestinationY,
    UINTN Width,
    UINTN Height,
    __attribute__((__unused__)) UINTN Delta)
{
	EFI_STATUS ret = EFI_SUCCESS;
	EFI_STATUS unlock_ret;

	if (!This || (gop_t *)This != gop || !BltBuffer)
		return EFI_INVALID_PARAMETER;

	if (DestinationX + Width > info.HorizontalResolution ||
	    DestinationY + Height > info.VerticalResolution)
		return EFI_INVALID_PARAMETER;

	ret = gop_lock();
	if (EFI_ERROR(ret))
		return ret;

	if (!gop->img) {
		ret = EFI_NOT_STARTED;
		goto exit;
	}

	switch (BltOperation) {
	case EfiBltVideoFill:
		fill_buffer(BltBuffer, DestinationX, DestinationY,
			    Width, Height);
		break;

	case EfiBltBufferToVideo:
		copy_to_buffer(BltBuffer, SourceX, SourceY,
			       DestinationX, DestinationY,
			       Width, Height);
		break;

	default:
		ret = EFI_UNSUPPORTED;
	}

	if (EFI_ERROR(ret))
		goto exit;

	put_image();

exit:
	unlock_ret = gop_unlock();
	return EFI_ERROR(ret) ? ret : unlock_ret;
}

static EFI_STATUS gop_init(EFI_SYSTEM_TABLE *st)
{
	static gop_t gop_default = {
		.prot = {
			.QueryMode = query_mode,
			.SetMode = set_mode,
			.Blt = blt,
			.Mode = &mode
		}
	};

	if (!st)
		return EFI_INVALID_PARAMETER;

	if (gop_handle)
		return EFI_ALREADY_STARTED;

	return interface_init(st, &gop_guid, &gop_handle,
			      &gop_default, sizeof(gop_default),
			      (void **)&gop);
}

static EFI_STATUS gop_exit(EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;

	if (!st)
		return EFI_INVALID_PARAMETER;

	if (!gop_handle)
		return EFI_NOT_STARTED;

	ret = stop_gop();
	if (EFI_ERROR(ret)) {
		ewerr("Failed to stop GOP thread");
		return ret;
	}

	ret = interface_free(st, &gop_guid, gop_handle);
	if (EFI_ERROR(ret))
		return ret;

	gop = NULL;
	gop_handle = NULL;

	return EFI_SUCCESS;
}

ewdrv_t gop_drv = {
	.name = "gop",
	.description = "Graphics Output Protocol support based on Xlib",
	.init = gop_init,
	.exit = gop_exit
};
