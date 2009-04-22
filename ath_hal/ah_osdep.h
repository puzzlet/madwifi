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
#ifndef _ATH_AH_OSDEP_H_
#define _ATH_AH_OSDEP_H_

#include <asm/io.h>
#include "ah_devid.h"

#define __ahdecl
#ifndef __packed
#define __packed	__attribute__((__packed__))
#endif
#ifndef __used
#define __used		__attribute__((__used__))
#endif

/* Replace void pointers from ah.h with safer specific types */
#define HAL_SOFTC struct ath_softc *
#define HAL_BUS_HANDLE void __iomem *
#define HAL_BUS_TAG struct ar531x_config *

/* Linker-assisted set support */
#define	__STRING(x)	#x

#define DECLARE_ah_chips \
struct ath_hal_chip *AR5210_chip_ptr __attribute__((__weak__));	\
struct ath_hal_chip *AR5211_chip_ptr __attribute__((__weak__));	\
struct ath_hal_chip *AR5212_chip_ptr __attribute__((__weak__));	\
struct ath_hal_chip *AR5312_chip_ptr __attribute__((__weak__));	\
struct ath_hal_chip *AR5416_chip_ptr __attribute__((__weak__));	\
struct ath_hal_chip *AR9160_chip_ptr __attribute__((__weak__));	\
struct ath_hal_chip *const *ah_chips_ptrs[] = {			\
	&AR5210_chip_ptr,					\
	&AR5211_chip_ptr,					\
	&AR5212_chip_ptr,					\
	&AR5312_chip_ptr,					\
	&AR5416_chip_ptr,					\
	&AR9160_chip_ptr,					\
	NULL							\
}

#define DECLARE_ah_rfs \
struct ath_hal_rf *RF2316_rf_ptr __attribute__((__weak__));	\
struct ath_hal_rf *RF2317_rf_ptr __attribute__((__weak__));	\
struct ath_hal_rf *RF2413_rf_ptr __attribute__((__weak__));	\
struct ath_hal_rf *RF2425_rf_ptr __attribute__((__weak__));	\
struct ath_hal_rf *RF5111_rf_ptr __attribute__((__weak__));	\
struct ath_hal_rf *RF5112_rf_ptr __attribute__((__weak__));	\
struct ath_hal_rf *RF5413_rf_ptr __attribute__((__weak__));	\
struct ath_hal_rf *const *ah_rfs_ptrs[] = {			\
	&RF2316_rf_ptr,						\
	&RF2317_rf_ptr,						\
	&RF2413_rf_ptr,						\
	&RF2425_rf_ptr,						\
	&RF5111_rf_ptr,						\
	&RF5112_rf_ptr,						\
	&RF5413_rf_ptr,						\
	NULL							\
}

#define OS_SET_DECLARE(set, ptype)				\
	DECLARE_##set

#define OS_DATA_SET(set, sym)					\
	typeof(sym) *sym##_ptr = &sym

#define OS_SET_FOREACH(pvar, set)				\
	typeof(pvar) *ppvar = set##_ptrs;			\
	for (pvar = *ppvar; pvar; pvar = *++ppvar) if (*pvar)

/* Byte order/swapping support. */
#define AH_LITTLE_ENDIAN	1234
#define AH_BIG_ENDIAN		4321

#ifndef AH_BYTE_ORDER
/*
 * When the .inc file is not available (e.g. when building in the kernel source
 * tree), look for some other way to determine the host byte order.
 */
#ifdef __LITTLE_ENDIAN
#define AH_BYTE_ORDER	AH_LITTLE_ENDIAN
#endif
#ifdef __BIG_ENDIAN
#define AH_BYTE_ORDER	AH_BIG_ENDIAN
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

/*
 * The functions in this section are not intended to be invoked by MadWifi
 * driver code, but by the HAL.  They are NOT safe to call directly when the
 * sc->sc_hal_lock is not held.  Use ath_reg_read and ATH_REG_WRITE instead!
*/
#if defined(AH_DEBUG) || defined(AH_REGOPS_FUNC) || defined(AH_DEBUG_ALQ)
#define OS_REG_WRITE(_ah, _reg, _val)	ath_hal_reg_write(_ah, _reg, _val)
#define OS_REG_READ(_ah, _reg)		ath_hal_reg_read(_ah, _reg)
extern void __ahdecl ath_hal_reg_write(struct ath_hal *ah, u_int reg,
				       u_int32_t val);
extern u_int32_t __ahdecl ath_hal_reg_read(struct ath_hal *ah, u_int reg);
#else
#define OS_REG_WRITE(_ah, _reg, _val)	_OS_REG_WRITE(_ah, _reg, _val)
#define OS_REG_READ(_ah, _reg)		_OS_REG_READ(_ah, _reg)
#endif				/* AH_DEBUG || AH_REGFUNC || AH_DEBUG_ALQ */

/* Delay n microseconds. */
extern void __ahdecl ath_hal_delay(int);
#define OS_DELAY(_n)		ath_hal_delay(_n)

#define OS_INLINE	__inline
#define OS_MEMZERO(_a, _n)	ath_hal_memzero((_a), (_n))
extern void __ahdecl ath_hal_memzero(void *, size_t);
#define OS_MEMCPY(_d, _s, _n)	ath_hal_memcpy(_d,_s,_n)
extern void *__ahdecl ath_hal_memcpy(void *, const void *, size_t);

#define __bswap16(val) __swab16(val)
#define __bswap32(val) __swab32(val)

#ifdef AH_DEBUG_ALQ
extern void __ahdecl OS_MARK(struct ath_hal *, u_int id, u_int32_t value);
#else
#define OS_MARK(_ah, _id, _v)
#endif

#define __DECONST(type, var) ((type)(unsigned long)(const void *)(var))
#define __printflike(fmtarg, firstvararg) \
	    __attribute__((__format__ (__printf__, fmtarg, firstvararg)))
#define __va_list va_list

#undef swap

#endif /* _ATH_AH_OSDEP_H_ */
