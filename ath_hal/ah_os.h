/*-
 * Copyright (c) 2002-2006 Sam Leffler, Errno Consulting
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 *
 * $Id$
 */
#ifndef _ATH_AH_OS_H_
#define _ATH_AH_OS_H_

/*
 * Atheros Hardware Access Layer (HAL) OS Dependent Definitions.
 */

/*
   MadWifi safe register operations:

	When hacking on registers directly, we need to use the macros below to
	avoid concurrent PCI access and abort mode errors.

	* ath_reg_read
	* ATH_REG_WRITE

   HAL-ONLY register operations:

	* _OS_REG_READ
	* _OS_REG_WRITE
	* OS_REG_READ
	* OS_REG_WRITE
	* ath_hal_reg_read.
	* ath_hal_reg_write

    When compiled in HAL:
	* We don't require locking overhead and function call except for
	  debugging.
	* All HAL operations are executed in the context of a MadWifi wrapper
	  call that holds the HAL lock.
	* Normally HAL is built with the non-modified version of this file, so
	  it doesn't have our funny macros anyway.

    When compiled in MadWifi:
	* The HAL wrapper API takes the HAL lock before invoking the HAL.
	* HAL access is already protected, and MadWifi must NOT access the
	  functions listed above.
*/

/*
 * When building the HAL proper, we use no GPL-licensed include files and must
 * define Linux types ourselves.  Please note that the definitions below don't
 * exactly match those in <linux/types.h>
 */
#ifndef _LINUX_TYPES_H
/* NB: ARM defaults to unsigned, so be explicit */
typedef signed char		int8_t;
typedef short			int16_t;
typedef int			int32_t;
typedef long long		int64_t;

typedef unsigned char		u_int8_t;
typedef unsigned short		u_int16_t;
typedef unsigned int		u_int32_t;
typedef unsigned long long	u_int64_t;

typedef unsigned int		size_t;
typedef unsigned int		u_int;
typedef	void*			va_list;
#endif				/* !_LINUX_TYPES_H */

struct ath_hal;

extern int ath_hal_dma_beacon_response_time;
extern int ath_hal_sw_beacon_response_time;
extern int ath_hal_additional_swba_backoff;

void __ahdecl ath_hal_printf(struct ath_hal *ah, HAL_BOOL prefer_alq, const char *fmt, ...)
	__attribute__ ((__format__ (__printf__, 3, 4)));
#ifdef AH_DEBUG_ALQ
void __ahdecl ath_hal_logprintf(struct ath_hal *ah, const char *fmt, ...)
	__attribute__ ((__format__ (__printf__, 2, 3)));
#endif

int __ahdecl ath_hal_memcmp(const void *a, const void *b, size_t n);
void *__ahdecl ath_hal_malloc(size_t size);
void __ahdecl ath_hal_free(void *p);

#ifndef abs
#define	abs(_a)			__builtin_abs(_a)
#endif

#ifndef labs
#define	labs(_a)		__builtin_labs(_a)
#endif

#define HAL_DEBUG_OFF			0
/* Show register accesses */
#define HAL_DEBUG_REGOPS 		1
/* Show decoded register dump (include name, etc) */
#define HAL_DEBUG_REGOPS_DECODED 	2
/* Show bit-fields where we put decode logic in */
#define HAL_DEBUG_REGOPS_BITFIELDS    	3
/* Add a read before a write to show 'changes', may have side-effects */
#define HAL_DEBUG_REGOPS_DELTAS	 	4

/* XXX: This should be stored per-device for proper multi-radio support */
extern const char *ath_hal_func;
extern const char *ath_hal_device;
extern int ath_hal_debug;
static inline void ath_hal_set_function(const char *name)
{
#ifdef AH_DEBUG
	ath_hal_func = name;
#endif
}
static inline void ath_hal_set_device(const char *name)
{
#ifdef AH_DEBUG
	ath_hal_device = name;
#endif
}

/*
 * Linux-specific attach/detach methods needed for module reference counting.
 *
 * NB: These are intentionally not marked __ahdecl since they are
 *     compiled with the default calling convention and are not called
 *     from within the HAL.
 */
extern struct ath_hal *_ath_hal_attach(u_int16_t devid, HAL_SOFTC,
				       HAL_BUS_TAG, HAL_BUS_HANDLE,
				       HAL_STATUS *);
extern void _ath_hal_detach(struct ath_hal *);

void
ath_hal_print_decoded_register(struct ath_hal *ah, 
			       const char *device_name,
			       u_int32_t address, u_int32_t oldval, 
			       u_int32_t newval, HAL_BOOL bitfields);
void
ath_hal_print_register(struct ath_hal *ah, 
			       const char *device_name,
			       u_int32_t address, u_int32_t value);

HAL_BOOL
ath_hal_lookup_register_name(struct ath_hal *ah, char *buf, int buflen, 
		u_int32_t address);

#endif				/* _ATH_AH_OSDEP_H_ */

