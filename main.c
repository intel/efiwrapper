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
#include <external.h>
#include <ewvar.h>
#include <ewdrv.h>
#include <ewlog.h>
#include <storage.h>
/* Entry point */
int main(int argc, char **argv)
{
	EFI_HANDLE image = NULL;
	EFI_SYSTEM_TABLE *st;
	EFI_STATUS ret;

	ret = efiwrapper_init(argc, argv, &st, &image);
	if (ret) {
		ewerr("efiwrapper library initialization failed");
		return EXIT_FAILURE;
	}

	ret = ewdrv_init(st);
	if (ret) {
		ewerr("drivers initialization failed");
		return EXIT_FAILURE;
	}

	ret = efi_main(image, st);
	if (EFI_ERROR(ret))
		ewerr("The EFI program exited with error code: 0x%x", ret);

	ret = ewdrv_exit(st);
	if (ret)
		ewerr("drivers release failed");

	ret = efiwrapper_free(image);
	if (EFI_ERROR(ret))
		ewerr("efiwrapper library exit failed");

	return EFI_ERROR(ret) ? EXIT_FAILURE : EXIT_SUCCESS;
}
