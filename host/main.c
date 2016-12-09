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

#include <efi.h>
#include <efiapi.h>
#include <efiwrapper.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ewvar.h>
#include <ewdrv.h>
#include <ewlog.h>
#include <libgen.h>
#include <dlfcn.h>
#include <ewlog.h>
#include <ewlib.h>
#include <setjmp.h>
#include <string.h>

#include "disk.h"
#include "event.h"
#include "tcp4.h"
#include "fileio.h"

static ewdrv_t *host_drivers[] = {
	&disk_drv,
	&event_drv,
	&tcp4_drv,
	&fileio_drv,
	NULL
};
ewdrv_t **ew_drivers = host_drivers;

typedef EFI_STATUS (*efi_main_t)(EFI_HANDLE image, EFI_SYSTEM_TABLE *st);
static jmp_buf jmp;
static EFI_STATUS reset_status;
static char *cmdname;

static EFIAPI EFI_STATUS
reset_system(__attribute__((__unused__)) EFI_RESET_TYPE ResetType,
	     EFI_STATUS ResetStatus,
	     __attribute__((__unused__)) UINTN DataSize,
	     __attribute__((__unused__)) CHAR16 *ResetData)
{
	reset_status = ResetStatus;
	longjmp(jmp, 1);

	return EFI_SUCCESS;
}

static EFI_STATUS so_load_and_execute(const char *path,
				      EFI_HANDLE image,
				      EFI_SYSTEM_TABLE *st)
{
	EFI_STATUS ret;
	EFI_RESET_SYSTEM saved_reset_rs;
	void *dl;
	efi_main_t entry;
	int setjmpret;

	dl = dlopen(path, RTLD_NOW | RTLD_LOCAL);
	if (!dl) {
		ewerr("dlopen failed, %s", dlerror());
		return EFI_DEVICE_ERROR;
	}

	entry = dlsym(dl, "efi_main");
	if (!entry) {
		ewerr("Cannot find efi_main symbol");
		return EFI_COMPROMISED_DATA;
	}

	saved_reset_rs = st->RuntimeServices->ResetSystem;
	st->RuntimeServices->ResetSystem = reset_system;

	setjmpret = setjmp(jmp);
	if (setjmpret == 0)
		ret = entry(image, st);
	else
		ret = reset_status;

	st->RuntimeServices->ResetSystem = saved_reset_rs;
	dlclose(dl);
	return ret;
}

static void usage(int ret) __attribute__ ((noreturn));
static void usage(int ret)
{
	printf("Usage: %s [OPTIONS] <ELF shared library> [ARGS]\n",
	       cmdname);
	printf(" OPTIONS:\n");
	printf(" -h,--help			Print this help\n");
	printf(" --list-drivers			List available drivers\n");
	printf(" --disable-drivers=DRV1,DRV2	Disable drivers DRV1 and DRV2\n");
	exit(ret);
}

static void error(const char *fmt, ...) __attribute__ ((noreturn));
static void error(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vfprintf(stderr, fmt, args);
	va_end(args);

	usage(EXIT_FAILURE);
}

static void help(char *arg) __attribute__ ((noreturn));
static void help(__attribute__((__unused__)) char *arg)
{
	usage(EXIT_SUCCESS);
}

static void list_drivers(char *arg) __attribute__ ((noreturn));
static void list_drivers(__attribute__((__unused__)) char *arg)
{
	size_t i;

	printf("Drivers list:\n");
	for (i = 0; ew_drivers[i]; i++)
		printf("- %s: %s\n", ew_drivers[i]->name,
		       ew_drivers[i]->description);

	exit(EXIT_SUCCESS);
}

static void disable_driver(char *name)
{
	size_t i;
	bool found = false;

	for (i = 0; ew_drivers[i]; i++) {
		if (found || !strcmp(ew_drivers[i]->name, name)) {
			ew_drivers[i] = ew_drivers[i + 1];
			found = true;
		}
	}

	if (!found)
		error("'%s' driver not found\n", name);
}

static void disable_drivers(char *names)
{
	EFI_STATUS ret;
	char *saveptr, *name;

	name = strtok_r(names, ",", &saveptr);
	while (name) {
		disable_driver(name);
		name = strtok_r(NULL, ",", &saveptr);
	}
}

static struct option {
	const char *name;
	bool has_argument;
	void (*fun)(char *param);
} OPTIONS[] = {
	{ "-h", false, help },
	{ "--help", false, help },
	{ "--list-drivers", false, list_drivers },
	{ "--disable-drivers", true, disable_drivers }
};

static struct option *get_option(char *name, char **arg)
{
	size_t i, len;

	*arg = NULL;
	for (i = 0; i < ARRAY_SIZE(OPTIONS); i++) {
		len = strlen(OPTIONS[i].name);

		if (strncmp(OPTIONS[i].name, name, len))
			continue;

		if (name[len] == '\0' && !OPTIONS[i].has_argument)
			return &OPTIONS[i];

		if (name[len] == '=' && OPTIONS[i].has_argument) {
			*arg = &name[len + 1];
			return &OPTIONS[i];
		}
	}

	return NULL;
}

static void parse_options(int *argc, char ***argv)
{
	EFI_STATUS ret;
	char c;
	size_t i;
	struct option *option;
	char *cur, *arg;

	for (i = 1; i < (size_t)*argc; i++) {
		cur = (*argv)[i];

		if (*cur != '-')
			goto exit;

		option = get_option(cur, &arg);
		if (!option)
			error("Unknown option '%s'\n", cur);

		option->fun(arg);
	}

exit:
	*argc = *argc - i + 1;
	*argv = *argv +	i - 1;
}

int main(int argc, char **argv)
{
	EFI_HANDLE image = NULL;
	EFI_SYSTEM_TABLE *st;
	EFI_STATUS ret;

	cmdname = basename(argv[0]);

	parse_options(&argc, &argv);
	if (argc < 2)
		error("Not enough parameter\n");

	ret = efiwrapper_init(argc - 1, argv + 1, &st, &image);
	if (ret) {
		ewerr("efiwrapper library initialization failed");
		return EXIT_FAILURE;
	}

	ret = ewdrv_init(st);
	if (ret) {
		ewerr("drivers initialization failed");
		return EXIT_FAILURE;
	}

	ret = so_load_and_execute(argv[1], image, st);
	if (EFI_ERROR(ret))
		ewerr("%s load and execute failed, ret=0x%zx",
		      argv[1], (size_t)ret);

	ret = ewdrv_exit(st);
	if (ret)
		ewerr("drivers release failed");

	ret = efiwrapper_free(image);
	if (EFI_ERROR(ret))
		ewerr("efiwrapper library exit failed");

	return EFI_ERROR(ret) ? EXIT_FAILURE : EXIT_SUCCESS;
}
