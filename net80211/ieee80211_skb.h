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
 * $Id: ieee80211_skb.h 2647 2007-08-09 08:43:58Z mtaylor $
 */
#ifndef _NET80211_IEEE80211_SKB_H_
#define _NET80211_IEEE80211_SKB_H_

/*******************************************************************************
 * Globals
 ******************************************************************************/

#ifdef IEEE80211_DEBUG_REFCNT

/* Count of currently tracked skbs */
extern atomic_t skb_total_counter;
/* Count of currently tracked skbs' references */
extern atomic_t skb_refs_counter;

#endif /* #ifdef IEEE80211_DEBUG_REFCNT */

/*******************************************************************************
 * Public API
 ******************************************************************************/

/* SKB_XX(...) macros will blow up if _skb is NULL (detect problems early) */
#define	SKB_CB(_skb) 		((struct ieee80211_cb *)(_skb)->cb)
#define	SKB_NI(_skb) 		(SKB_CB(_skb)->ni)

#define M_FLAG_SET(_skb, _flag) \
	(SKB_CB(_skb)->flags |= (_flag))
#define	M_FLAG_CLR(_skb, _flag) \
	(SKB_CB(_skb)->flags &= ~(_flag))
#define	M_FLAG_GET(_skb, _flag) \
	(SKB_CB(_skb)->flags & (_flag))
#define M_FLAG_KEEP_ONLY(_skb, _flag) \
	(SKB_CB(_skb)->flags &= (_flag))

#define	M_PWR_SAV_SET(skb) M_FLAG_SET((skb), M_PWR_SAV)
#define	M_PWR_SAV_CLR(skb) M_FLAG_CLR((skb), M_PWR_SAV)
#define	M_PWR_SAV_GET(skb) M_FLAG_GET((skb), M_PWR_SAV)

/* SKBs on the power save queue are tagged with an age and
 * timed out.  We reuse the hardware checksum field in the
 * mbuf packet header to store this data.
 * XXX: use private CB area. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
#define skb_age csum_offset
#else
#define skb_age csum
#endif

#define	M_AGE_SET(_skb,_v)	((_skb)->skb_age = (_v))
#define	M_AGE_GET(_skb)		((_skb)->skb_age)
#define	M_AGE_SUB(_skb,_adj)	((_skb)->skb_age -= (_adj))

/* ieee80211_dev_kfree_skb will release one reference from SKB.
 * If SKB refcount is going to zero:
 *  - Free the node reference and set it to null.
 *  - Break the linked list, clearing next skb's prev pointer if possible. */
void ieee80211_dev_kfree_skb(struct sk_buff **pskb);

static inline void ieee80211_skb_copy_noderef(struct sk_buff *src, 
		struct sk_buff *dst)
{
	if (SKB_NI(src) != NULL)
		SKB_NI(dst) = ieee80211_ref_node(SKB_NI(src));
}

/* ieee80211_dev_kfree_skb_list will invoke ieee80211_dev_kfree_skb on each node in
 * a list of skbs, starting with the first. */
void ieee80211_dev_kfree_skb_list(struct sk_buff **pskb);

struct sk_buff *ieee80211_dev_alloc_skb(int size);

void ieee80211_skb_track(struct sk_buff *skb);
void ieee80211_skb_untrack(struct sk_buff *skb);

#ifdef IEEE80211_DEBUG_REFCNT
int ieee80211_skb_counter(void);
int ieee80211_skb_references(void);
#else
#define ieee80211_skb_counter() (0)
#define ieee80211_skb_references() (0)
#endif

/*******************************************************************************
 * skbuff leak/refcount debugging Replacement Functions
 ******************************************************************************/

#ifdef IEEE80211_DEBUG_REFCNT

int vlan_hwaccel_rx_debug(struct sk_buff *skb, struct vlan_group *grp,
		unsigned short vlan_tag, const char *func, int line);
int netif_rx_debug(struct sk_buff *skb, const char *func, int line);
struct sk_buff *alloc_skb_debug(unsigned int length, gfp_t gfp_mask,
		const char *func, int line);
struct sk_buff *dev_alloc_skb_debug(unsigned int length,
		const char *func, int line);
struct sk_buff *skb_clone_debug(struct sk_buff *skb, gfp_t pri, 
		const char *func, int line);
struct sk_buff *skb_copy_debug(struct sk_buff *skb, gfp_t pri, 
		const char *func, int line);
struct sk_buff *skb_get_debug(struct sk_buff *skb, 
		const char *func, int line);
struct sk_buff *skb_realloc_headroom_debug(struct sk_buff *skb, unsigned int headroom, 
		const char *func, int line);
struct sk_buff *pskb_copy_debug(struct sk_buff *skb, gfp_t pri,
		const char *func, int line);
int dev_queue_xmit_debug(struct sk_buff *skb,
		const char *func, int line);
struct sk_buff *skb_share_check_debug(struct sk_buff *skb, gfp_t pri,
		const char *func, int line);
void kfree_skb_fast_debug(struct sk_buff *skb, 
		const char *func, int line);
struct sk_buff *skb_unshare_debug(struct sk_buff *skb, gfp_t pri,
		const char *func, int line);
struct sk_buff *skb_copy_expand_debug(const struct sk_buff *skb, int newheadroom, 
		int newtailroom, gfp_t gfp_mask, 
		const char *func, int line);

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

#define skb_unshare(_skb, _pri) \
	skb_unshare_debug(_skb, _pri, __func__, __LINE__)
#define skb_copy_expand(_skb, _newheadroom, _newtailroom, _gfp_mask) \
	skb_copy_expand_debug(_skb, _newheadroom, _newtailroom, _gfp_mask, __func__, __LINE__)
#define vlan_hwaccel_rx(_skb, _grp, _tag) \
	vlan_hwaccel_rx_debug(_skb, _grp, _tag, __func__, __LINE__)
#define netif_rx(_skb) \
	netif_rx_debug(_skb, __func__, __LINE__)
#define	alloc_skb(_length, _gfp_mask) \
	alloc_skb_debug(_length, _gfp_mask, __func__, __LINE__)
#define	dev_alloc_skb(_length) \
	dev_alloc_skb_debug(_length, __func__, __LINE__)
#define	dev_kfree_skb_irq(_skb) \
	unref_skb(_skb, UNREF_USE_DEV_KFREE_SKB_IRQ, __func__, __LINE__)
#define	dev_kfree_skb_any(_skb) \
	unref_skb(_skb, UNREF_USE_DEV_KFREE_SKB_ANY, __func__, __LINE__)
#define	dev_kfree_skb(_skb) \
	unref_skb(_skb, UNREF_USE_DEV_KFREE_SKB, __func__, __LINE__)
#define	kfree_skb(_skb) \
	unref_skb(_skb, UNREF_USE_KFREE_SKB, __func__, __LINE__)
#define skb_clone(_skb, _pri) \
	skb_clone_debug(_skb, _pri, __func__, __LINE__)
#define skb_share_check(_skb, _pri) \
	skb_share_check_debug(_skb, _pri, __func__, __LINE__)
#define kfree_skb_fast(_skb) \
	kfree_skb_fast_debug(_skb, __func__, __LINE__)
#define skb_realloc_headroom(_skb, _headroom) \
	skb_realloc_headroom_debug(_skb, _headroom, __func__, __LINE__)
#define pskb_copy(_skb, _pri) \
	pskb_copy_debug(_skb, _pri, __func__, __LINE__)
#define skb_get(_skb) \
	skb_get_debug(_skb, __func__, __LINE__)
#define skb_copy(_skb, _pri) \
	skb_copy_debug(_skb, _pri, __func__, __LINE__)
#define dev_queue_xmit(_skb) \
	dev_queue_xmit_debug(_skb, __func__, __LINE__)
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */

#endif /* _NET80211_IEEE80211_SKB_H_ */

