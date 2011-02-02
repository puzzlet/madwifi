/*-
 * Copyright (c) 2007 Michael Taylor, Apprion
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: ieee80211_linux.c 2829 2007-11-05 20:43:50Z mtaylor $
 */

#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif

/*
 * IEEE 802.11 support (Linux-specific code)
 */
#if !defined(AUTOCONF_INCLUDED) && !defined(CONFIG_LOCALVERSION)
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kmod.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/sysctl.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/vmalloc.h>
#include <linux/proc_fs.h>

#include <net/iw_handler.h>
#include <linux/wireless.h>
#include <linux/if_arp.h>		/* XXX for ARPHRD_* */

#include <asm/uaccess.h>

#include "if_media.h"
#include "if_ethersubr.h"

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_monitor.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,17)
#include <linux/device.h>
#endif

#undef alloc_skb
#undef dev_alloc_skb
#undef dev_kfree_skb
#undef dev_kfree_skb_any
#undef dev_kfree_skb_irq
#undef dev_queue_xmit
#undef kfree_skb
#undef kfree_skb_fast
#undef netif_rx
#undef pskb_copy
#undef skb_clone
#undef skb_copy
#undef skb_copy_expand
#undef skb_get
#undef skb_realloc_headroom
#undef skb_share_check
#undef skb_unshare
#undef vlan_hwaccel_rx

atomic_t skb_total_counter = ATOMIC_INIT(0);
EXPORT_SYMBOL(skb_total_counter);

#ifdef IEEE80211_DEBUG_REFCNT
atomic_t skb_refs_counter  = ATOMIC_INIT(0);
EXPORT_SYMBOL(skb_refs_counter);
#endif

/*******************************************************************************
 * Debug Helpers
 ******************************************************************************/

static void skb_print_message(
		int show_counter, 
		const struct sk_buff *skb, 
		const char *func, int line,
		const char *message,
		...)
{
	va_list args;
	char skb_count[32] = { '\0' };
	char expanded_message[256] = { '\0' };
	if (show_counter) {
#ifdef IEEE80211_DEBUG_REFCNT
		snprintf(skb_count, 
			 sizeof(skb_count), 
			 "[#SKB=%05d #REF=%05d] ", 
			 atomic_read(&skb_total_counter),
			 atomic_read(&skb_refs_counter));
#else
		snprintf(skb_count, 
			 sizeof(skb_count), 
			 "[#SKB=%05d] ", 
			 atomic_read(&skb_total_counter));
#endif

	}
	va_start(args, message);
	vsnprintf(expanded_message, sizeof(expanded_message), message, args);
	printk(KERN_DEBUG "%s: %s%s:%d %s\n",
		((skb != NULL) ? DEV_NAME(skb->dev) : "none"),
		skb_count, 
		func, line,
		expanded_message);
	va_end(args);
}

#ifdef IEEE80211_DEBUG_REFCNT

static void 
print_skb_refchange_message(
		const struct sk_buff *skb, int users_adjustment, 
		const char *func, int line);
static void 
print_skb_trackchange_message(
		const struct sk_buff *skb, int users_adjustment, 
		const char *func, int line,
		char *message);

/* Called automatically when an SKB reaches zero users, 
 * reporting any leaked node references. */
#ifdef IEEE80211_DEBUG_REFCNT_SKBDEST
static void skb_destructor(struct sk_buff *skb);
#endif
static void get_skb_description(char *dst, int dst_size, const char *label,
		const struct sk_buff *skb, int users_adjustment);

static struct sk_buff *
clean_clone_or_copy(struct sk_buff *skb);

static struct sk_buff *
track_skb(struct sk_buff *skb, int users_adjustment, 
		      const char *func, int line);

static struct sk_buff *
untrack_skb(struct sk_buff *skb, int users_adjustment, 
	      const char *func, int line);

#define UNREF_USE_KFREE_SKB 	   	0
#define UNREF_USE_DEV_KFREE_SKB_ANY   	1
#define UNREF_USE_DEV_KFREE_SKB 	2
#define UNREF_USE_DEV_KFREE_SKB_IRQ   	3

/* Assumes SKB is not yet freed at the time of the call and shows the new users
 * count as (users - 1). */
static void unref_skb(struct sk_buff *skb, int type,
	  const char *func, int line);

/* Assumes SKB reference counter has already been updated and reports count as
 * atomic_read(&skb->users). */
static struct sk_buff *
ref_skb(struct sk_buff *skb, 
	const char *func, int line);


#ifdef IEEE80211_DEBUG_REFCNT_SKBDEST
/* Destructor for reporting node reference leaks */
static void skb_destructor(struct sk_buff *skb) {
	/* Report any node reference leaks - caused by kernel net device queue 
	 * dropping buffer, rather than passing it to the driver. */
	if (SKB_NI(skb) != NULL) {
		printk(KERN_ERR "%s:%d - ERROR: non-NULL node pointer in %p, %p<" MAC_FMT ">!  "
				"Leak Detected!\n", 
		       __func__, __LINE__, 
		       skb, SKB_NI(skb), MAC_ADDR(SKB_NI(skb)->ni_macaddr));
		dump_stack();
	}
	if (SKB_CB(skb)->next_destructor != NULL) {
		SKB_CB(skb)->next_destructor(skb);
	}
}
EXPORT_SYMBOL(skb_destructor);
#endif /* #ifdef IEEE80211_DEBUG_REFCNT_SKBDEST */

static void get_skb_description(char *dst, int dst_size, const char *label, const struct sk_buff *skb, int users_adjustment) { 
	dst[0] = '\0';
	if (NULL != skb) {
		int adj_users = atomic_read(&skb->users) + users_adjustment;
		if (SKB_NI(skb) != NULL) {
			snprintf(dst, dst_size, 
				 " [%s%s%p,users=%d,node=%p<" MAC_FMT ">,aid=%d%s%s]",
				 label,
				 (label != NULL ? ": " : ""),
				 skb,
				 adj_users,
				 SKB_NI(skb),
				 MAC_ADDR(SKB_NI(skb)->ni_macaddr),
				 SKB_NI(skb)->ni_associd,
				 ((adj_users  < 0) ? " ** CORRUPTED **" : ""),
				 ((adj_users == 0) ? " ** RELEASED **" : "")
				 );
		}
		else {
			snprintf(dst, dst_size, 
				 " [%s%s%p,users=%d,node=NULL,aid=N/A%s%s]",
				 label,
				 (label != NULL ? ": " : ""),
				 skb,
				 adj_users,
				 ((adj_users  < 0) ? " ** CORRUPTED **" : ""),
				 ((adj_users == 0) ? " ** RELEASED **" : "")
				 );
		}
		dst[dst_size-1] = '\0';
	}
}

static void print_skb_refchange_message(
		const struct sk_buff *skb, int users_adjustment, 
		const char *func, int line)
{
	char skb_desc[128] = { '\0' };
	if (0 == (ath_debug_global & GLOBAL_DEBUG_SKB_REF))
		return;
	get_skb_description(skb_desc, sizeof(skb_desc),  
			     "skb",  skb,  users_adjustment);
	skb_print_message(0 /* no global count */, skb, 
			  func, line,
			  skb_desc);
}

static void print_skb_trackchange_message(
		const struct sk_buff *skb, int users_adjustment, 
		const char *func, int line,
		char *message)
{
	char skb_desc[128] = { '\0' };
	if (0 == (ath_debug_global & GLOBAL_DEBUG_SKB))
		return;
	get_skb_description(skb_desc, sizeof(skb_desc),  
			     "skb",  skb,  users_adjustment);
	skb_print_message(1 /* show global count */, skb, 
			  func, line,
			  "%s%s", skb_desc, message);
}

static struct sk_buff *
clean_clone_or_copy(struct sk_buff *skb) {
	if (skb != NULL)
		M_FLAG_CLR(skb, M_SKB_TRACKED);
	return skb;
}

static struct sk_buff *
track_skb(struct sk_buff *skb, int users_adjustment, 
		      const char *func, int line) 
{
	if (NULL == skb) {
		skb_print_message(0 /* show_counter */, 
			skb, func2, line2,
			"ERROR: NULL skb received.  Skipping.");
		return NULL;
	}
	if (M_FLAG_GET(skb, M_SKB_TRACKED)) {
		skb_print_message(0 /* show_counter */, 
			skb, func2, line2,
			"ERROR: Already tracked skb received.  Skipping.");
		dump_stack();
		return skb;
	}
	if (skb_shared(skb)) {
		skb_print_message(0 /* show_counter */, 
			skb, func2, line2,
			"ERROR: Shared skb received.  References leaked??");
		dump_stack();
	}
	atomic_inc(&skb_total_counter);
	atomic_inc(&skb_refs_counter);
	M_FLAG_SET(skb, M_SKB_TRACKED);
	print_skb_trackchange_message(skb, users_adjustment,
				      func2, line2, 
				      " is now ** TRACKED **");
#ifdef IEEE80211_DEBUG_REFCNT_SKBDEST
	/* Install our debug destructor, chaining to the original... */
	if (skb->destructor != skb_destructor) {
		SKB_CB(skb)->next_destructor = skb->destructor;
		skb->destructor = skb_destructor;
	}
#endif /* #ifdef IEEE80211_DEBUG_REFCNT_SKBDEST */
	return skb;
}

static struct sk_buff *
untrack_skb(struct sk_buff *skb, int users_adjustment, 
	      const char *func, int line)
{
	if (NULL == skb) {
		skb_print_message(0 /* show_counter */, 
			skb, func, line,
			"ERROR: NULL skb received.  No changes made.");
		return NULL;
	}
	if (!M_FLAG_GET(skb, M_SKB_TRACKED)) {
		skb_print_message(0 /* show_counter */, 
			skb, func, line,
			"ERROR: Untracked skb received.  No changes made.");
		return skb;
	}
	if (skb_shared(skb)) {
		skb_print_message(0 /* show_counter */, 
			skb, func, line,
			"ERROR: Shared skb received.  References leaked??");
	}
	atomic_dec(&skb_total_counter);
	atomic_dec(&skb_refs_counter);
	M_FLAG_CLR(skb, M_SKB_TRACKED);
#ifdef IEEE80211_DEBUG_REFCNT_SKBDEST
	/* Uninstall our debug destructor, restoring any original... */
	if (skb->destructor == skb_destructor) {
		skb->destructor = SKB_CB(skb)->next_destructor;
		SKB_CB(skb)->next_destructor = NULL;
	}
#endif /* #ifdef IEEE80211_DEBUG_REFCNT_SKBDEST */
	print_skb_trackchange_message(skb, users_adjustment,
				      func, line, 
				      " is now ** UNTRACKED **");
	return skb;
}

#define UNREF_USE_KFREE_SKB 	   	0
#define UNREF_USE_DEV_KFREE_SKB_ANY   	1
#define UNREF_USE_DEV_KFREE_SKB 	2
#define UNREF_USE_DEV_KFREE_SKB_IRQ   	3

/* Assumes SKB is not yet freed at the time of the call and shows the new users
 * count as (users - 1). */
static void
unref_skb(struct sk_buff *skb, int type,
	  const char *func, int line) 
{
	if (NULL == skb) {
		skb_print_message(0 /* show_counter */, 
			skb, func, line,
			"ERROR: NULL skb received.");
		dump_stack();
		return;
	}
	if (!M_FLAG_GET(skb, M_SKB_TRACKED)) {
		skb_print_message(0 /* show_counter */, 
			skb, func, line,
			"ERROR: Untracked skb received.  Probable duplicate free error!");
		dump_stack();
		return;
	}
	/* If free is unacceptable for current user count, report the error. */
	if (atomic_read(&skb->users) < 1) {
		skb_print_message(0 /* show_counter */, 
			skb, func, line,
			"ERROR: free an skb with %d users", 
			atomic_read(&skb->users));
		dump_stack();
		return;
	}

        if (skb_shared(skb)) {
		atomic_dec(&skb_refs_counter);
		print_skb_refchange_message(skb, -1, func2, line2);
	}
	else {
		if (SKB_NI(skb) != NULL) {
			printk(KERN_ERR "%s:%d - ERROR: non-NULL node pointer in %p, %p<" MAC_FMT ">!  "
					"Driver Leak Detected!\n", 
			       __func__, __LINE__, 
			       skb, SKB_NI(skb), MAC_ADDR(SKB_NI(skb)->ni_macaddr));
			dump_stack();
			/* Allow the leak and let programmer fix it, but do not
			 * report it again in the destructor. */
			SKB_NI(skb) = NULL;
		}
		untrack_skb(skb, -1, func, line);
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
	if ((in_irq() || irqs_disabled()) && 
			(type == UNREF_USE_KFREE_SKB || 
			 type == UNREF_USE_DEV_KFREE_SKB)) {
		skb_print_message(0 /* show_counter */, 
			skb, func, line,
			"ERROR: free an skb in interrupt context using a non-"
			"safe form of skb free function.");
		type = UNREF_USE_DEV_KFREE_SKB_ANY;
		dump_stack();
	}
#endif

	switch (type) {
	case UNREF_USE_DEV_KFREE_SKB_ANY:
		dev_kfree_skb_any(skb);
		break;
	case UNREF_USE_DEV_KFREE_SKB_IRQ:
		dev_kfree_skb_irq(skb);
		break;
	case UNREF_USE_DEV_KFREE_SKB:
		/* NOTE: dev_kfree_skb is a macro pointing to kfree_skb, so
		 * fallthrough... */
	case UNREF_USE_KFREE_SKB:
		/* fallthrough */
	default:
		kfree_skb(skb);
		break;
	}
}

/* Assumes SKB reference counter has already been updated and reports count as
 * atomic_read(&skb->users). */
static struct sk_buff *
ref_skb(struct sk_buff *skb, 
	const char *func, int line) 
{
	if (NULL == skb) {
		skb_print_message(0 /* show_counter */, 
			skb, func, line,
			"ERROR: NULL skb received.  No changes made.");
		dump_stack();
		return NULL;
	}
	if (!M_FLAG_GET(skb, M_SKB_TRACKED)) {
		skb_print_message(0 /* show_counter */, 
			skb, func, line,
			"ERROR: Untracked skb received.  Probable use after free!  "
			"No changes made.");
		dump_stack();
		return skb;
	}
	print_skb_refchange_message(skb, 0, func, line);
	return skb;
}

#endif /* #ifdef IEEE80211_DEBUG_REFCNT */

/*******************************************************************************
 * Public API
 ******************************************************************************/

/* ieee80211_dev_kfree_skb will release one reference from SKB.
 * If SKB refcount is going to zero:
 *  - Free the node reference and set it to null.
 *  - Break the linked list, clearing next skb's prev pointer if possible. */
void ieee80211_dev_kfree_skb(struct sk_buff **pskb) 
{
	struct sk_buff *skb;

	/* Do not fail on null, as we are going to use this in cleanup code. */
	if (!pskb || !(skb = *pskb))
		return;

	/* Release the SKB references, for fragments of chain that are
	 * unshared... starting at skb passed in. */
	if (skb->prev == NULL) {
		if (skb->next != NULL)
			skb->next->prev = NULL;
		skb->next = NULL;
	}

	if (SKB_NI(skb) != NULL)
		ieee80211_unref_node(&SKB_NI(skb));

#ifdef IEEE80211_DEBUG_REFCNT
	unref_skb(skb, UNREF_USE_DEV_KFREE_SKB_ANY, 
		  __func__, __LINE__);
#else
	dev_kfree_skb_any(skb);
#endif

	*pskb = NULL;
}

/* ieee80211_dev_kfree_skb_list will invoke ieee80211_dev_kfree_skb on each node in
 * a list of skbs, starting with the first. */
void
ieee80211_dev_kfree_skb_list(struct sk_buff **pskb) 
{
	struct sk_buff *skb, *tskb;

	/* Do not fail on null, as we are going to use this in cleanup code */
	if (!pskb || !(skb = *pskb))
		return;

	while (skb) {
		tskb = skb->next;

		ieee80211_dev_kfree_skb(&skb);
		skb = tskb;
	}

	*pskb = NULL;
}

struct sk_buff *
ieee80211_dev_alloc_skb(int size) 
{
	struct sk_buff *skb = dev_alloc_skb(size);
	if (skb == NULL) {
		skb_print_message(
				0 /* show_counter */, 
				NULL /* skb */, 
				__func__, __LINE__,
				"sk_buff allocation of size %u failed",
				size);
		return NULL;
	}

#ifdef IEEE80211_DEBUG_REFCNT
	return track_skb(skb, 0,  __func__, __LINE__);
#else
	return skb;
#endif
}

void
ieee80211_skb_track(struct sk_buff *skb) {
#ifdef IEEE80211_DEBUG_REFCNT
	track_skb(skb, 0 /* users_adjustment */, 
		  __func__, __LINE__);
#else
	/* Just a dumb counter, in no-debug builds */
	atomic_inc(&skb_total_counter);
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */
}

void
ieee80211_skb_untrack(struct sk_buff *skb) {
	/* Just a dumb counter, in no-debug builds */
	atomic_dec(&skb_total_counter);
}

#ifdef IEEE80211_DEBUG_REFCNT
int 
ieee80211_skb_counter(void) {
	return atomic_read(&skb_total_counter);
}

int
ieee80211_skb_references(void) {
	return atomic_read(&skb_refs_counter);
}
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */

/*******************************************************************************
 * skbuff leak/refcount debugging Replacement Functions
 * PUT last in order to avoid conflicts with use of original functions in
 * inline functions above this point.
 ******************************************************************************/

#ifdef IEEE80211_DEBUG_REFCNT

int  vlan_hwaccel_rx_debug(struct sk_buff *skb, 
			       struct vlan_group *grp, unsigned short vlan_tag, 
			       const char *func, int line) {
	return vlan_hwaccel_rx(
		untrack_skb(skb, 0, __func__, __LINE__),
		grp, vlan_tag);
}

int netif_rx_debug(struct sk_buff *skb, const char *func, int line) {
	return netif_rx(untrack_skb(skb, 0, __func__, __LINE__));
}

struct sk_buff *alloc_skb_debug(unsigned int length, gfp_t gfp_mask,
		const char *func, int line) {
	return track_skb(alloc_skb(length, gfp_mask),
			0 /* users_adjustment */,
			__func__, __LINE__);
}

struct sk_buff *dev_alloc_skb_debug(unsigned int length,
		    const char *func, int line)
{
	return track_skb(dev_alloc_skb(length),
			0 /* users_adjustment */,
			__func__, __LINE__);
}



struct sk_buff *skb_clone_debug(struct sk_buff *skb, gfp_t pri, 
			       const char *func, int line) 
{
	return track_skb(
			clean_clone_or_copy(skb_clone(skb, pri)),
			0 /* users_adjustment */,
			__func__, __LINE__);
}

struct sk_buff *skb_copy_debug(struct sk_buff *skb, gfp_t pri, 
	       const char *func, int line)
{
	return track_skb(
		clean_clone_or_copy(skb_copy(skb, pri)), 0 /* users_adjustment */,
			 __func__, __LINE__);
}

struct sk_buff *skb_get_debug(struct sk_buff *skb, 
	      const char *func, int line)
{
	return ref_skb(skb_get(skb), 
		       __func__, __LINE__);
}

struct sk_buff *skb_realloc_headroom_debug(struct sk_buff *skb, unsigned int headroom, 
			   const char *func, int line)
{
	/* skb_realloc_headroom ALWAYS returns a copy or a clone, refcount of
	 * new one is always zero and refcount of original is not touched. */
	return track_skb(
			clean_clone_or_copy(
				skb_realloc_headroom(skb, headroom)), 
			0 /* users_adjustment */,
			__func__, __LINE__);
}

struct sk_buff *pskb_copy_debug(struct sk_buff *skb, gfp_t pri,
		const char *func, int line)
{
	return track_skb(
			clean_clone_or_copy(pskb_copy(skb, pri)), 
			0 /* users_adjustment */,
			__func__, __LINE__);
}

int dev_queue_xmit_debug(struct sk_buff *skb,
		     const char *func, int line)
{
	return dev_queue_xmit(untrack_skb(skb, 0, __func__, __LINE__));
}

struct sk_buff *skb_share_check_debug(struct sk_buff *skb, gfp_t pri,
		      const char *func, int line)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
	might_sleep_if(pri & __GFP_WAIT);
#endif
	if (skb_shared(skb)) {
		struct sk_buff *nskb = track_skb(
			clean_clone_or_copy(skb_clone(skb, pri)), 
			0,
			__func__, __LINE__);
		unref_skb(skb, UNREF_USE_DEV_KFREE_SKB_ANY, 
			  __func__, __LINE__);
		skb = nskb;
	}
	return skb;
}

void  kfree_skb_fast_debug(struct sk_buff *skb, 
		     const char *func, int line)
{
	/* NOT so fast... */
	unref_skb(skb, UNREF_USE_DEV_KFREE_SKB_ANY, func, line, __func__, __LINE__);
}

struct sk_buff *skb_unshare_debug(struct sk_buff *skb, gfp_t pri,
		  const char *func, int line)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
	might_sleep_if(pri & __GFP_WAIT);
#endif
	if (skb_cloned(skb)) {
		struct sk_buff *nskb = track_skb(
			clean_clone_or_copy(skb_copy(skb, pri)), 0,
			__func__, __LINE__);
		unref_skb(skb, UNREF_USE_DEV_KFREE_SKB_ANY, 
			  __func__, __LINE__);
		skb = nskb;
	}
	return skb;
}

struct sk_buff *skb_copy_expand_debug(const struct sk_buff *skb, int newheadroom, 
		      int newtailroom, gfp_t gfp_mask, 
		      const char *func, int line)
{
	return track_skb(
		clean_clone_or_copy(
			skb_copy_expand(skb, newheadroom, newtailroom, gfp_mask)), 
			0 /* users_adjustment */,
			__func__, __LINE__);
}

EXPORT_SYMBOL(vlan_hwaccel_rx_debug);
EXPORT_SYMBOL(netif_rx_debug);
EXPORT_SYMBOL(alloc_skb_debug);
EXPORT_SYMBOL(dev_alloc_skb_debug);
EXPORT_SYMBOL(skb_clone_debug);
EXPORT_SYMBOL(skb_copy_debug);
EXPORT_SYMBOL(skb_get_debug);
EXPORT_SYMBOL(skb_realloc_headroom_debug);
EXPORT_SYMBOL(pskb_copy_debug);
EXPORT_SYMBOL(dev_queue_xmit_debug);
EXPORT_SYMBOL(skb_share_check_debug);
EXPORT_SYMBOL(kfree_skb_fast_debug);
EXPORT_SYMBOL(skb_unshare_debug);
EXPORT_SYMBOL(skb_copy_expand_debug);

EXPORT_SYMBOL(ieee80211_skb_counter);
EXPORT_SYMBOL(ieee80211_skb_references);
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */

EXPORT_SYMBOL(ieee80211_dev_alloc_skb);
EXPORT_SYMBOL(ieee80211_skb_untrack);
EXPORT_SYMBOL(ieee80211_dev_kfree_skb_list);
EXPORT_SYMBOL(ieee80211_dev_kfree_skb);
EXPORT_SYMBOL(ieee80211_skb_track);

