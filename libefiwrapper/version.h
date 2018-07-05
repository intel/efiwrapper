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

#ifndef _VERSION_H_
#define _VERSION_H_

#if defined(USER)
#define BUILD_VARIANT           ""
#elif defined(USERDEBUG)
#define BUILD_VARIANT           "-userdebug"
#else
#define BUILD_VARIANT           "-eng"
#endif

#define STRINGIFY(x)	#x
#define TOSTRING(x)	STRINGIFY(x)
#define PASTE(a,b)	a##b
#define HEX(a)		PASTE(0x, a)

#define _EFIWRAPPER_MAJOR	02
#define _EFIWRAPPER_MINOR	02

#define EFIWRAPPER_MAJOR HEX(_EFIWRAPPER_MAJOR)
#define EFIWRAPPER_MINOR HEX(_EFIWRAPPER_MINOR)

#define EFIWRAPPER_VERSION ("efiwrapper-"			\
			    TOSTRING(_EFIWRAPPER_MAJOR) "."	\
			    TOSTRING(_EFIWRAPPER_MINOR) BUILD_VARIANT)

#endif	/* _VERSION_H_ */
