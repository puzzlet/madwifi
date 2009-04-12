/*-
 * Copyright (c) 2002-2005 Sam Leffler, Errno Consulting
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
#ifndef _ATH_COMPAT_H_
#define _ATH_COMPAT_H_

/* Compatibility with older Linux kernels */
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/time.h>
#include <linux/netdevice.h>
#include <linux/kernel.h>
#include <linux/kmod.h>
#endif

#include <linux/version.h>

#if !defined(__KERNEL__) || !defined (__bitwise)
#define __le16 u_int16_t
#define __le32 u_int32_t
#define __le64 u_int64_t
#define __be16 u_int16_t
#define __be32 u_int32_t
#define __be64 u_int64_t
#define __force
#endif

#ifndef container_of
#define container_of(ptr, type, member) ({				\
	    const typeof( ((type *)0)->member ) *__mptr = (ptr);	\
	    (type *)( (char *)__mptr - offsetof(type,member) );})
#endif

#ifndef list_for_each_entry_reverse
#define list_for_each_entry_reverse(pos, head, member)			\
	for (pos = list_entry((head)->prev, typeof(*pos), member);	\
	     prefetch(pos->member.prev), &pos->member != (head); 	\
	     pos = list_entry(pos->member.prev, typeof(*pos), member))
#endif

#ifndef NETDEV_TX_OK
#define NETDEV_TX_OK    0
#define NETDEV_TX_BUSY  1
#endif

/*
 * BSD/Linux compatibility shims.  These are used mainly to
 * minimize differences when importing necesary BSD code.
 */
#define	NBBY	8			/* number of bits/byte */

/* roundup() appears in Linux 2.6.18 */
#ifndef roundup
#define	roundup(x, y)	((((x)+((y)-1))/(y))*(y))  /* to any y */
#endif

#define	howmany(x, y)	(((x)+((y)-1))/(y))

/* roundup() and howmany() macros that works both with positive and negative
 * values. */
#define roundup_s(x,y) _roundup_s((signed)(x),(signed)(y))
#define howmany_s(x,y) _howmany_s((signed)(x),(signed)(y))
#define _roundup_s(x,y)	\
	((x) >= 0 ? (((x)+((y)-1))/(y))*(y) : ((x)/(y))*(y))
#define	_howmany_s(x, y)	\
	((x) >= 0 ? (((x)+((y)-1))/(y)) : ((x)/(y)))

/* Bit map related macros. */
#define	setbit(a,i)	((a)[(i)/NBBY] |= 1<<((i)%NBBY))
#define	clrbit(a,i)	((a)[(i)/NBBY] &= ~(1<<((i)%NBBY)))
#define	isset(a,i)	((a)[(i)/NBBY] & (1<<((i)%NBBY)))
#define	isclr(a,i)	(((a)[(i)/NBBY] & (1<<((i)%NBBY))) == 0)

#ifndef __packed
#define	__packed	__attribute__((__packed__))
#endif

#define	__offsetof(t,m)	offsetof(t,m)

/*
 * NetBSD/FreeBSD defines for file version.
 */
#define	__FBSDID(_s)
#define	__KERNEL_RCSID(_n,_s)

/*
 * Fixes for Linux API changes
 */
#ifdef __KERNEL__

#define KASSERT(exp, msg) do {			\
	if (unlikely(!(exp))) {			\
		printk msg;			\
		printk("\n");			\
		BUG();				\
	}					\
} while (0)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14)
#define gfp_t unsigned int

static inline void *_kzalloc(size_t size, gfp_t flags)
{
	void *p = kmalloc(size, flags);
	if (likely(p != NULL))
		memset(p, 0, size);
	return p;
}

#define kzalloc(size, flags) _kzalloc(size, flags)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
#define ATH_REGISTER_SYSCTL_TABLE(t) register_sysctl_table(t, 1)
#else
#define ATH_REGISTER_SYSCTL_TABLE(t) register_sysctl_table(t)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9)
#define __user
#define __kernel
#define __iomem
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
static inline int timeval_compare(struct timeval *lhs, struct timeval *rhs)
{
	if (lhs->tv_sec < rhs->tv_sec)
		return -1;
	if (lhs->tv_sec > rhs->tv_sec)
		return 1;
	return lhs->tv_usec - rhs->tv_usec;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
typedef unsigned long resource_size_t;
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
#define IRQF_SHARED SA_SHIRQ
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,27)
#define netdev_priv(_netdev) ((_netdev)->priv)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
#define skb_end_pointer(_skb) ((_skb)->end)
#define skb_tail_pointer(_skb) ((_skb)->tail)
#define skb_set_network_header(_skb, _offset) \
	do { (_skb)->nh.raw = (_skb)->data + (_offset); } while (0)
#define skb_reset_network_header(_skb) \
	do { (_skb)->nh.raw = (_skb)->data; } while (0)
#define skb_mac_header(_skb) ((_skb)->mac.raw)
#define skb_reset_mac_header(_skb) \
	do { (_skb)->mac.raw = (_skb)->data; } while (0)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
#define CTL_AUTO -2
#define DEV_ATH 9
#else
#define CTL_AUTO CTL_UNNUMBERED
#define DEV_ATH CTL_UNNUMBERED
#endif

/* __skb_append got a third parameter in 2.6.14 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14)
#define __skb_queue_after(_list, _old, _new)	__skb_append(_old, _new)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25)
#define __skb_queue_after(_list, _old, _new)	__skb_append(_old, _new, _list)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
#define request_module(_fmt, _modname) request_module(_modname)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
#ifndef spin_lock_irqsave_nested
#define spin_lock_irqsave_nested(_lock, _flags, _subclass) \
	spin_lock_irqsave(_lock, _flags)
#endif
#endif

#endif /* __KERNEL__ */

#endif /* _ATH_COMPAT_H_ */
