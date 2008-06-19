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

/* Delay n microseconds. */
extern void __ahdecl ath_hal_delay(int);
#define	OS_DELAY(_n)		ath_hal_delay(_n)

#define	OS_MEMZERO(_a, _n)	ath_hal_memzero((_a), (_n))
extern void __ahdecl ath_hal_memzero(void *, size_t);
#define	OS_MEMCPY(_d, _s, _n)	ath_hal_memcpy(_d,_s,_n)
extern void *__ahdecl ath_hal_memcpy(void *, const void *, size_t);

#ifndef abs
#define	abs(_a)			__builtin_abs(_a)
#endif

#ifndef labs
#define	labs(_a)		__builtin_labs(_a)
#endif

/* Byte order/swapping support. */
#define	AH_LITTLE_ENDIAN	1234
#define	AH_BIG_ENDIAN		4321

#ifndef AH_BYTE_ORDER
/*
 * When the .inc file is not available (e.g. when building in the kernel source
 * tree), look for some other way to determine the host byte order.
 */
#ifdef __LITTLE_ENDIAN
#define	AH_BYTE_ORDER	AH_LITTLE_ENDIAN
#endif
#ifdef __BIG_ENDIAN
#define	AH_BYTE_ORDER	AH_BIG_ENDIAN
#endif
#ifndef AH_BYTE_ORDER
#error "Do not know host byte order"
#endif
#endif				/* AH_BYTE_ORDER */

/*
 * The HAL programs big-endian platforms to use byte-swapped hardware registers.
 * This is done to avoid the byte swapping needed to access PCI devices.
 *
 * Many big-endian architectures provide I/O functions that avoid byte swapping.
 * We use them when possible.  Otherwise, we provide replacements.  The downside
 * or the replacements is that we may be byte-swapping data twice, so we try to
 * avoid it.
 *
 * We use raw access for Linux prior to 2.6.12.  For newer version, we need to
 * use ioread32() and iowrite32(), which would take care of indirect access to
 * the registers.
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,12)) && \
    (AH_BYTE_ORDER == AH_BIG_ENDIAN) && \
    !defined(CONFIG_GENERIC_IOMAP) && \
    !defined(CONFIG_PARISC) && \
    !(defined(CONFIG_PPC64) && \
      (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,14))) && \
    !defined(CONFIG_PPC_MERGE) && \
    !(defined(CONFIG_MIPS) && \
      (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,21)))
# ifndef iowrite32be
#  define iowrite32be(_val, _addr) iowrite32(swab32((_val)), (_addr))
# endif
# ifndef ioread32be
#  define ioread32be(_addr) swab32(ioread32((_addr)))
# endif
#endif

/*
 * The register accesses are done using target-specific functions when
 * debugging is enabled (AH_DEBUG) or it's explicitly requested for the target.
 *
 * The hardware registers use little-endian byte order natively.  Big-endian
 * systems are configured by HAL to byte-swap of register reads and writes.
 * However, the registers in the areas 0x4000-0x4fff and 0x7000-0x7fff are not
 * byte swapped!
 *
 * Since Linux I/O primitives default to little-endian operations, we only
 * need to suppress byte-swapping on big-endian systems outside the area used
 * by the PCI clock domain registers.
 */
#if (AH_BYTE_ORDER == AH_BIG_ENDIAN)
# define is_reg_le(__reg) ((0x4000 <= (__reg) && (__reg) < 0x5000) || \
			   (0x7000 <= (__reg) && (__reg) < 0x8000))
# if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,12)
#  define _OS_REG_WRITE(_ah, _reg, _val) do {			\
	 is_reg_le(_reg) ?					\
	  iowrite32((_val), (_ah)->ah_sh + (_reg)) :		\
	  iowrite32be((_val), (_ah)->ah_sh + (_reg));		\
	} while (0)
#  define _OS_REG_READ(_ah, _reg)				\
	(is_reg_le(_reg) ?					\
	  ioread32((_ah)->ah_sh + (_reg)) :			\
	  ioread32be((_ah)->ah_sh + (_reg)))
# else				/* Linux < 2.6.12 */
#  define _OS_REG_WRITE(_ah, _reg, _val) do {			\
	 writel(is_reg_le(_reg) ?				\
		 (_val) : cpu_to_le32(_val),			\
		 (_ah)->ah_sh + (_reg));			\
	} while (0)
#  define _OS_REG_READ(_ah, _reg)				\
	(is_reg_le(_reg) ?					\
	  readl((_ah)->ah_sh + (_reg)) :			\
	  cpu_to_le32(readl((_ah)->ah_sh + (_reg))))
# endif				/* Linux < 2.6.12 */
#else				/* Little endian */
# if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,12)
#  define _OS_REG_WRITE(_ah, _reg, _val) do {			\
	 iowrite32((_val), (_ah)->ah_sh + (_reg));		\
	} while (0)
#  define _OS_REG_READ(_ah, _reg)				\
	ioread32((_ah)->ah_sh + (_reg))
# else				/* Linux < 2.6.12 */
#  define _OS_REG_WRITE(_ah, _reg, _val) do {			\
	 writel((_val), (_ah)->ah_sh + (_reg));			\
	} while (0)
#  define _OS_REG_READ(_ah, _reg)				\
	readl((_ah)->ah_sh + (_reg))
# endif				/* Linux < 2.6.12 */
#endif				/* Little endian */

#define HAL_DEBUG_OFF			0
/* Show register accesses */
#define HAL_DEBUG_REGOPS 		1
/* Show decoded register dump (include name, etc) */
#define HAL_DEBUG_REGOPS_DECODED 	2
/* Show bit-fields where we put decode logic in */
#define HAL_DEBUG_REGOPS_BITFIELDS    	3
/* Add a read before a write to show 'changes', may have side-effects */
#define HAL_DEBUG_REGOPS_DELTAS	 	4

/*
 * The functions in this section are not intended to be invoked by MadWifi
 * driver code, but by the HAL.  They are NOT safe to call directly when the
 * sc->sc_hal_lock is not held.  Use ath_reg_read and ATH_REG_WRITE instead!
*/
#if defined(AH_DEBUG) || defined(AH_REGOPS_FUNC) || defined(AH_DEBUG_ALQ)
#define	OS_REG_WRITE(_ah, _reg, _val)	ath_hal_reg_write(_ah, _reg, _val)
#define	OS_REG_READ(_ah, _reg)		ath_hal_reg_read(_ah, _reg)
extern void __ahdecl ath_hal_reg_write(struct ath_hal *ah, u_int reg,
				       u_int32_t val);
extern u_int32_t __ahdecl ath_hal_reg_read(struct ath_hal *ah, u_int reg);
#else
#define OS_REG_WRITE(_ah, _reg, _val)	_OS_REG_WRITE(_ah, _reg, _val)
#define OS_REG_READ(_ah, _reg)		_OS_REG_READ(_ah, _reg)
#endif				/* AH_DEBUG || AH_REGFUNC || AH_DEBUG_ALQ */

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

#ifdef AH_DEBUG_ALQ
extern void __ahdecl OS_MARK(struct ath_hal *, u_int id, u_int32_t value);
#else
#define	OS_MARK(_ah, _id, _v)
#endif

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

