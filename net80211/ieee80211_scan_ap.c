/*-
 * Copyright (c) 2002-2005 Sam Leffler, Errno Consulting
 * Copyright (c) 2005  Matt Mackall <mpm@selenic.com>
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
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
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
 * $Id$
 */
#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif

/*
 * IEEE 802.11 ap scanning support.
 */
#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/init.h>
#include <linux/delay.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
/* Copied from Linux lib/sort.c */
static void u32_swap(void *a, void *b, int size)
{
	u32 t = *(u32 *)a;
	*(u32 *)a = *(u32 *)b;
	*(u32 *)b = t;
}

static void generic_swap(void *a, void *b, int size)
{
	char t;

	do {
		t = *(char *)a;
		*(char *)a++ = *(char *)b;
		*(char *)b++ = t;
	} while (--size > 0);
}

/**
 * sort - sort an array of elements
 * @base: pointer to data to sort
 * @num: number of elements
 * @size: size of each element
 * @cmp: pointer to comparison function
 * @swap: pointer to swap function or NULL
 *
 * This function does a heapsort on the given array. You may provide a
 * swap function optimized to your element type.
 *
 * Sorting time is O(n log n) both on average and worst-case. While
 * qsort is about 20% faster on average, it suffers from exploitable
 * O(n*n) worst-case behavior and extra memory requirements that make
 * it less suitable for kernel use.
 */

void sort(void *base, size_t num, size_t size,
	  int (*cmp)(const void *, const void *),
	  void (*swap)(void *, void *, int size))
{
	/* pre-scale counters for performance */
	int i = (num/2 - 1) * size, n = num * size, c, r;

	if (!swap)
		swap = (size == 4 ? u32_swap : generic_swap);

	/* heapify */
	for ( ; i >= 0; i -= size) {
		for (r = i; r * 2 + size < n; r  = c) {
			c = r * 2 + size;
			if (c < n - size && cmp(base + c, base + c + size) < 0)
				c += size;
			if (cmp(base + r, base + c) >= 0)
				break;
			swap(base + r, base + c, size);
		}
	}

	/* sort */
	for (i = n - size; i > 0; i -= size) {
		swap(base, base + i, size);
		for (r = 0; r * 2 + size < i; r = c) {
			c = r * 2 + size;
			if (c < i - size && cmp(base + c, base + c + size) < 0)
				c += size;
			if (cmp(base + r, base + c) >= 0)
				break;
			swap(base + r, base + c, size);
		}
	}
}
#else
#include <linux/sort.h>
#endif

#include "if_media.h"

#include <net80211/ieee80211_var.h>

#define	AP_PURGE_SCANS	2			/* age for purging entries (scans) */
#define RSSI_LPF_LEN	10
#define	RSSI_EP_MULTIPLIER	(1 << 7)	/* pow2 to optimize out * and / */
#define RSSI_IN(x)		((x) * RSSI_EP_MULTIPLIER)
#define LPF_RSSI(x, y, len)	(((x) * ((len) - 1) + (y)) / (len))
#define RSSI_LPF(x, y) do {						\
	if ((y) >= -20)							\
		x = LPF_RSSI((x), RSSI_IN((y)), RSSI_LPF_LEN);		\
} while (0)
#define	EP_RND(x, mul) \
	((((x)%(mul)) >= ((mul)/2)) ? howmany(x, mul) : (x)/(mul))
#define	RSSI_GET(x)	EP_RND(x, RSSI_EP_MULTIPLIER)
#define	AP_HASHSIZE	32
/* simple hash is enough for variation of macaddr */
#define	AP_HASH(addr)	\
	(((const u_int8_t *)(addr))[IEEE80211_ADDR_LEN - 1] % AP_HASHSIZE)
#define	SCAN_AP_LOCK_INIT(_st, _name)					\
	spin_lock_init(&(_st)->as_lock)
#define	SCAN_AP_LOCK_DESTROY(_st)
#define	SCAN_AP_LOCK_IRQ(_st) do {					\
	unsigned long __stlockflags;					\
	spin_lock_irqsave(&(_st)->as_lock, __stlockflags);
#define	SCAN_AP_UNLOCK_IRQ(_st)						\
	spin_unlock_irqrestore(&(_st)->as_lock, __stlockflags);		\
} while (0)
#define	SCAN_AP_UNLOCK_IRQ_EARLY(_st)					\
	spin_unlock_irqrestore(&(_st)->as_lock, __stlockflags);

#define	SCAN_AP_GEN_LOCK_INIT(_st, _name)				\
	spin_lock_init(&(_st)->as_scanlock)
#define	SCAN_AP_GEN_LOCK_DESTROY(_st)
#define	SCAN_AP_GEN_LOCK(_st)		spin_lock(&(_st)->as_scanlock);
#define	SCAN_AP_GEN_UNLOCK(_st)	spin_unlock(&(_st)->as_scanlock);

struct scan_entry {
	struct ieee80211_scan_entry base;
	TAILQ_ENTRY(scan_entry) se_list;
	LIST_ENTRY(scan_entry) se_hash;
	u_int8_t	se_seen;	/* seen during current scan */
	u_int8_t	se_notseen;	/* not seen in previous scans */
	u_int32_t se_avgrssi;		/* LPF rssi state */
	unsigned long se_lastupdate;	/* time of last update */
	unsigned long se_lastfail;	/* time of last failure */
	unsigned long se_lastassoc;	/* time of last association */
	u_int se_scangen;		/* iterator scan gen# */
};

struct ap_state {
	unsigned int as_vap_desired_mode;       /* Used for channel selection, 
						 * vap->iv_des_mode */
	unsigned int as_required_mode;          /* Used for channel selection, 
						 * filtered version of 
						 * as_vap_desired_mode */
	int as_maxrssi[IEEE80211_CHAN_MAX]; 	/* Used for channel selection */

	/* These fields are just for scan caching for returning responses to
	 * wireless extensions.  i.e. show peers, APs, etc. */
	spinlock_t as_lock;			/* on scan table */
	int as_newscan;				/* trigger for updating 
						 * seen/not-seen for aging */
	TAILQ_HEAD(, scan_entry) as_entry;	/* all entries */
	ATH_LIST_HEAD(, scan_entry) as_hash[AP_HASHSIZE];
	spinlock_t as_scanlock;			/* on as_scangen */
	u_int as_scangen;			/* gen# for iterator */

	struct IEEE80211_TQ_STRUCT as_actiontq;	/* tasklet for "action" */
	struct ieee80211_scan_entry as_selbss;	/* selected bss for action tasklet */
	int (*as_action)(struct ieee80211vap *, const struct ieee80211_scan_entry *);
};

static int ap_flush(struct ieee80211_scan_state *);
static void action_tasklet(IEEE80211_TQUEUE_ARG);
static struct ieee80211_channel *find11gchannel(struct ieee80211com *ic, 
		int i, int freq);

static const u_int chanflags[] = {
	IEEE80211_CHAN_B,	/* IEEE80211_MODE_AUTO */
	IEEE80211_CHAN_A,	/* IEEE80211_MODE_11A */
	IEEE80211_CHAN_B,	/* IEEE80211_MODE_11B */
	IEEE80211_CHAN_PUREG,	/* IEEE80211_MODE_11G */
	IEEE80211_CHAN_FHSS,	/* IEEE80211_MODE_FH */
	IEEE80211_CHAN_A,	/* IEEE80211_MODE_TURBO_A */ /* for turbo mode 
							      * look for AP in 
							      * normal channel 
							      */
	IEEE80211_CHAN_PUREG,	/* IEEE80211_MODE_TURBO_G */
	IEEE80211_CHAN_ST,	/* IEEE80211_MODE_TURBO_STATIC_A */
};

static const u_int16_t rcl1[] =		/* 8 FCC channel: 52, 56, 60, 64, 
					 *                36, 40, 44, 48 */
{ 5260, 5280, 5300, 5320, 5180, 5200, 5220, 5240 };
static const u_int16_t rcl2[] =		/* 4 MKK channels: 34, 38, 42, 46 */
{ 5170, 5190, 5210, 5230 };
static const u_int16_t rcl3[] =		/* 2.4Ghz ch: 1,6,11,7,13 */
{ 2412, 2437, 2462, 2442, 2472 };
static const u_int16_t rcl4[] =		/* 5 FCC channel: 149, 153, 161, 165 */
{ 5745, 5765, 5785, 5805, 5825 };
static const u_int16_t rcl7[] =		/* 11 ETSI channel: 100, 104, 108, 112,
					 *                  116, 120, 124, 128, 
					 *                  132, 136, 140 */
{ 5500, 5520, 5540, 5560, 5580, 5600, 5620, 5640, 5660, 5680, 5700 };
static const u_int16_t rcl8[] =		/* 2.4Ghz ch: 2,3,4,5,8,9,10,12 */
{ 2417, 2422, 2427, 2432, 2447, 2452, 2457, 2467 };
static const u_int16_t rcl9[] =		/* 2.4Ghz ch: 14 */
{ 2484 };
static const u_int16_t rcl10[] =	/* Added Korean channels 2312-2372 */
{ 2312, 2317, 2322, 2327, 2332, 2337, 2342, 2347, 2352, 2357, 2362, 2367, 2372 };
static const u_int16_t rcl11[] =	/* Added Japan channels in 4.9/5.0 spectrum */
{ 5040, 5060, 5080, 4920, 4940, 4960, 4980 };
#ifdef ATH_TURBO_SCAN
static const u_int16_t rcl5[] =		/* 3 static turbo channels */
{ 5210, 5250, 5290 };
static const u_int16_t rcl6[] =		/* 2 static turbo channels */
{ 5760, 5800 };
static const u_int16_t rcl6x[] =		/* 4 FCC3 turbo channels */
{ 5540, 5580, 5620, 5660 };
static const u_int16_t rcl12[] =		/* 2.4Ghz Turbo channel 6 */
{ 2437 };
static const u_int16_t rcl13[] =		/* dynamic Turbo channels */
{ 5200, 5240, 5280, 5765, 5805 };
#endif /* ATH_TURBO_SCAN */

struct scanlist {
	u_int16_t	mode;
	u_int16_t	count;
	const u_int16_t	*list;
};

#define	IEEE80211_MODE_TURBO_STATIC_A	IEEE80211_MODE_MAX
#define	X(a)	.count = ARRAY_SIZE(a), .list = a

static const struct scanlist staScanTable[] = {
	{ IEEE80211_MODE_11B,   		X(rcl3)  },
	{ IEEE80211_MODE_11A,   		X(rcl1)  },
	{ IEEE80211_MODE_11A,   		X(rcl2)  },
	{ IEEE80211_MODE_11B,   		X(rcl8)  },
	{ IEEE80211_MODE_11B,   		X(rcl9)  },
	{ IEEE80211_MODE_11A,   		X(rcl4)  },
#ifdef ATH_TURBO_SCAN
	{ IEEE80211_MODE_TURBO_STATIC_A,	X(rcl5)  },
	{ IEEE80211_MODE_TURBO_STATIC_A,	X(rcl6)  },
	{ IEEE80211_MODE_TURBO_A,		X(rcl6x) },
	{ IEEE80211_MODE_TURBO_A,		X(rcl13) },
#endif /* ATH_TURBO_SCAN */
	{ IEEE80211_MODE_11A,			X(rcl7)  },
	{ IEEE80211_MODE_11B,			X(rcl10) },
	{ IEEE80211_MODE_11A,			X(rcl11) },
#ifdef ATH_TURBO_SCAN
	{ IEEE80211_MODE_TURBO_G,		X(rcl12) },
#endif /* ATH_TURBO_SCAN */
	{ .list = NULL }
};

#undef X
/* This function must be invoked with locks acquired */
static void
add_channels(struct ieee80211com *ic,
	struct ieee80211_scan_state *ss,
	enum ieee80211_phymode mode, const u_int16_t freq[], int nfreq)
{
	struct ieee80211_channel *c, *cg;
	u_int modeflags;
	int i;

	KASSERT(mode < ARRAY_SIZE(chanflags), ("Unexpected mode %u", mode));
	modeflags = chanflags[mode];
	for (i = 0; i < nfreq; i++) {
		c = ieee80211_find_channel(ic, freq[i], modeflags);
		if ((c == NULL) || isclr(ic->ic_chan_active, c->ic_ieee))
			continue;
		if (mode == IEEE80211_MODE_AUTO) {
			/* XXX special-case 11b/g channels so we select
			 *     the g channel if both are present. */
			if (IEEE80211_IS_CHAN_B(c) &&
			    (cg = find11gchannel(ic, i, c->ic_freq)) != NULL)
				c = cg;
		}
		if (ss->ss_last >= IEEE80211_SCAN_MAX)
			break;
		ss->ss_chans[ss->ss_last++] = c;
	}
}

/* This function must be invoked with locks acquired */
static int
checktable(const struct scanlist *scan, const struct ieee80211_channel *c)
{
	int i;

	for (; scan->list != NULL; scan++) {
		for (i = 0; i < scan->count; i++)
			if (scan->list[i] == c->ic_freq)
				return 1;
	}
	return 0;
}

/*
 * Attach prior to any scanning work.
 */
static int
ap_attach(struct ieee80211_scan_state *ss)
{
	struct ap_state *as;

	_MOD_INC_USE(THIS_MODULE, return 0);

	MALLOC(as, struct ap_state *, sizeof(struct ap_state),
		M_80211_SCAN, M_NOWAIT | M_ZERO);
	if (as == NULL)
		return 0;
	SCAN_AP_LOCK_INIT(as, "scan_ap");
	SCAN_AP_GEN_LOCK_INIT(as, "scan_ap_gen");
	TAILQ_INIT(&as->as_entry);
	IEEE80211_INIT_TQUEUE(&as->as_actiontq, action_tasklet, ss);
	ss->ss_priv = as;
	ap_flush(ss);
	return 1;
}

/*
 * Cleanup any private state.
 */
static int
ap_detach(struct ieee80211_scan_state *ss)
{
	struct ap_state *as = ss->ss_priv;

	if (as != NULL) {
		ap_flush(ss);
		IEEE80211_CANCEL_TQUEUE(&as->as_actiontq);
		FREE(as, M_80211_SCAN);
	}

	_MOD_DEC_USE(THIS_MODULE);
	return 1;
}

/*
 * Flush all per-scan state.
 */
static int
ap_flush(struct ieee80211_scan_state *ss)
{
	struct ap_state *as = ss->ss_priv;
	struct scan_entry *se, *next;

	SCAN_AP_LOCK_IRQ(as);
	memset(as->as_maxrssi, 0, sizeof(as->as_maxrssi));
	TAILQ_FOREACH_SAFE(se, &as->as_entry, se_list, next) {
		TAILQ_REMOVE(&as->as_entry, se, se_list);
		LIST_REMOVE(se, se_hash);
		FREE(se, M_80211_SCAN);
	}
	ss->ss_last = 0;		/* ensure no channel will be picked */
	SCAN_AP_UNLOCK_IRQ(as);
	return 0;
}

/* This function must be invoked with locks acquired */
static void
saveie(u_int8_t **iep, const u_int8_t *ie)
{
	if (ie == NULL)
		*iep = NULL;
	else
		ieee80211_saveie(iep, ie);
}

/* This function must be invoked with locks acquired */
static struct ieee80211_channel *
find11gchannel(struct ieee80211com *ic, int i, int freq)
{
	struct ieee80211_channel *c;
	int j;

	/* The normal ordering in the channel list is b channel
	 * immediately followed by g so optimize the search for
	 * this.  We'll still do a full search just in case. */
	for (j = i + 1; j < ic->ic_nchans; j++) {
		c = &ic->ic_channels[j];
		if ((c->ic_freq == freq) && IEEE80211_IS_CHAN_ANYG(c))
			return c;
	}
	for (j = 0; j < i; j++) {
		c = &ic->ic_channels[j];
		if ((c->ic_freq == freq) && IEEE80211_IS_CHAN_ANYG(c))
			return c;
	}
	return NULL;
}

/*
 * Start an ap scan by populating the channel list.
 */
static int
ap_start(struct ieee80211_scan_state *ss, struct ieee80211vap *vap)
{
	struct ap_state *as 	    = ss->ss_priv;
	struct ieee80211com *ic     = NULL;
	const struct scanlist *sl   = NULL;
	struct ieee80211_channel *c = NULL;
	int i;
	unsigned int mode = 0;

	SCAN_AP_LOCK_IRQ(as);
	ic = vap->iv_ic;
	/* Determine mode flags to match, or leave zero for auto mode */
	as->as_vap_desired_mode = vap->iv_des_mode;
	as->as_required_mode    = 0;
	if (as->as_vap_desired_mode != IEEE80211_MODE_AUTO) {
		as->as_required_mode = chanflags[as->as_vap_desired_mode];
		if ((vap->iv_ath_cap & IEEE80211_ATHC_TURBOP) && 
		    (as->as_required_mode != IEEE80211_CHAN_ST)) {
			/* Fixup for dynamic turbo flags */
			if (as->as_vap_desired_mode == IEEE80211_MODE_11G)
				as->as_required_mode = IEEE80211_CHAN_108G;
			else
				as->as_required_mode = IEEE80211_CHAN_108A;
		}
	}

	ss->ss_last = 0;
	/* Use the table of ordered channels to construct the list
	 * of channels for scanning.  Any channels in the ordered
	 * list not in the master list will be discarded. */
	for (sl = staScanTable; sl->list != NULL; sl++) {
		mode = sl->mode;

		/* The scan table marks 2.4Ghz channels as b
		 * so if the desired mode is 11g, then use
		 * the 11b channel list but upgrade the mode. */
		if (as->as_vap_desired_mode &&
		    (as->as_vap_desired_mode != mode) && 
		    (as->as_vap_desired_mode == IEEE80211_MODE_11G) && 
		    (mode == IEEE80211_MODE_11B))
			mode = IEEE80211_MODE_11G;

		/* If we are in "AUTO" mode, upgrade the mode to auto. 
		 * This lets add_channels upgrade an 11b channel to 
		 * 11g if available. */
		if (!as->as_vap_desired_mode && (mode == IEEE80211_MODE_11B))
			mode = IEEE80211_MODE_AUTO;

		/* Add the list of the channels; any that are not
		 * in the master channel list will be discarded. */
		add_channels(ic, ss, mode, sl->list, sl->count);
	}

	/* Add the channels from the ic (from HAL) that are not present
	 * in the staScanTable, assuming they pass the sanity checks... */
	for (i = 0; i < ic->ic_nchans; i++) {
		c = &ic->ic_channels[i];

		/* XR is not supported on turbo channels */
		if (IEEE80211_IS_CHAN_TURBO(c) && vap->iv_flags & IEEE80211_F_XR)
			continue;

		/* Dynamic channels are scanned in base mode */
		if (!as->as_required_mode && !IEEE80211_IS_CHAN_ST(c))
			continue;

		/* Use any 11g channel instead of 11b one. */
		if (vap->iv_des_mode == IEEE80211_MODE_AUTO && 
		    IEEE80211_IS_CHAN_B(c) &&
		    find11gchannel(ic, i, c->ic_freq))
			continue;

		/* Do not add channels already put into the scan list by the
		 * scan table - these have already been filtered by mode
		 * and for whether they are in the active channel list. */
		if (checktable(staScanTable, c))
			continue;

		/* Make sure the channel is active */
		if ((c == NULL) || isclr(ic->ic_chan_active, c->ic_ieee))
			continue;

		/* Don't overrun */
		if (ss->ss_last >= IEEE80211_SCAN_MAX)
			break;

		ss->ss_chans[ss->ss_last++] = c;
	}
	ss->ss_next = 0;
	/* XXX tunables */
	ss->ss_mindwell = msecs_to_jiffies(200);	/* 200ms */
	ss->ss_maxdwell = msecs_to_jiffies(300);	/* 300ms */

#ifdef IEEE80211_DEBUG
	if (ieee80211_msg_scan(vap)) {
		printk("%s: scan set ", vap->iv_dev->name);
		ieee80211_scan_dump_channels(ss);
		printk(" dwell min %ld max %ld\n",
			ss->ss_mindwell, ss->ss_maxdwell);
	}
#endif /* IEEE80211_DEBUG */

	as->as_newscan = 1;
	SCAN_AP_UNLOCK_IRQ(as);
	return 0;
}

/*
 * Restart a bg scan.
 */
static int
ap_restart(struct ieee80211_scan_state *ss, struct ieee80211vap *vap)
{
	struct ap_state *as = ss->ss_priv;
	as->as_newscan = 1;
	return 0;
}

/*
 * Cancel an ongoing scan.
 */
static int
ap_cancel(struct ieee80211_scan_state *ss, struct ieee80211vap *vap)
{
	struct ap_state *as = ss->ss_priv;
	IEEE80211_CANCEL_TQUEUE(&as->as_actiontq);
	return 0;
}

/*
 * Record max rssi on channel.
 */
static int
ap_add(struct ieee80211_scan_state *ss, const struct ieee80211_scanparams *sp,
	const struct ieee80211_frame *wh, int subtype, int rssi, u_int64_t rtsf)
{
	struct ap_state *as              = ss->ss_priv;
	const u_int8_t *macaddr          = wh->i_addr2;
	struct ieee80211vap *vap         = ss->ss_vap;
	struct ieee80211com *ic          = vap->iv_ic;
	struct scan_entry *se            = NULL;
	struct ieee80211_scan_entry *ise = NULL;
	int hash = AP_HASH(macaddr);
	int chan;

	/* This section provides scan results to wireless extensions */
	SCAN_AP_LOCK_IRQ(as);

	chan = ieee80211_chan2ieee(ic, ic->ic_curchan);
	/* This is the only information used for channel selection by AP */
	if (rssi > as->as_maxrssi[chan])
		as->as_maxrssi[chan] = rssi;
	LIST_FOREACH(se, &as->as_hash[hash], se_hash)
		if (IEEE80211_ADDR_EQ(se->base.se_macaddr, macaddr) &&
		    (sp->ssid[1] == se->base.se_ssid[1]) &&
		    !memcmp(se->base.se_ssid + 2, sp->ssid + 2, 
			    se->base.se_ssid[1]))
			goto found;

	MALLOC(se, struct scan_entry *, sizeof(struct scan_entry),
		M_80211_SCAN, M_NOWAIT | M_ZERO);

	if (se == NULL) {
		SCAN_AP_UNLOCK_IRQ_EARLY(as);
		return 0;
	}

	se->se_scangen = as->as_scangen-1;
	IEEE80211_ADDR_COPY(se->base.se_macaddr, macaddr);
	TAILQ_INSERT_TAIL(&as->as_entry, se, se_list);
	LIST_INSERT_HEAD(&as->as_hash[hash], se, se_hash);

found:
	ise = &se->base;

	/* XXX: AP beaconing multiple SSID w/ same BSSID */
	if ((sp->ssid[1] != 0) &&
	    ((subtype == IEEE80211_FC0_SUBTYPE_PROBE_RESP) || 
	     (ise->se_ssid[1] == 0)))
		memcpy(ise->se_ssid, sp->ssid, 2 + sp->ssid[1]);

	memcpy(ise->se_rates, sp->rates, 
		IEEE80211_SANITISE_RATESIZE(2 + sp->rates[1]));
	if (sp->xrates != NULL) {
		memcpy(ise->se_xrates, sp->xrates, 
				IEEE80211_SANITISE_RATESIZE(2 + sp->xrates[1]));
	} else
		ise->se_xrates[1] = 0;

	IEEE80211_ADDR_COPY(ise->se_bssid, wh->i_addr3);

	/* Record RSSI data using extended precision LPF filter.*/
	if (se->se_lastupdate == 0)		/* First sample */
		se->se_avgrssi = RSSI_IN(rssi);
	else					/* Avg. w/ previous samples */
		RSSI_LPF(se->se_avgrssi, rssi);
	se->base.se_rssi = RSSI_GET(se->se_avgrssi);

	ise->se_rtsf = rtsf;
	memcpy(ise->se_tstamp.data, sp->tstamp, sizeof(ise->se_tstamp));
	ise->se_intval = sp->bintval;
	ise->se_capinfo = sp->capinfo;
	ise->se_chan = ic->ic_curchan;
	ise->se_fhdwell = sp->fhdwell;
	ise->se_fhindex = sp->fhindex;
	ise->se_erp = sp->erp;
	ise->se_timoff = sp->timoff;

	if (sp->tim != NULL) {
		const struct ieee80211_tim_ie *tim =
		    (const struct ieee80211_tim_ie *)sp->tim;
		ise->se_dtimperiod = tim->tim_period;
	}

	saveie(&ise->se_wme_ie, sp->wme);
	saveie(&ise->se_wpa_ie, sp->wpa);
	saveie(&ise->se_rsn_ie, sp->rsn);
	saveie(&ise->se_ath_ie, sp->ath);

	se->se_lastupdate = jiffies;		/* update time */
	se->se_seen = 1;
	se->se_notseen = 0;

	SCAN_AP_UNLOCK_IRQ(as);

	return 1;
}

struct pc_params {
	struct ieee80211vap *vap;
	struct ieee80211_scan_state *ss;
	int flags;
};

struct channel {
	struct ieee80211_channel *chan;
	int orig;
	struct pc_params *params;
};

/* This function must be invoked with locks acquired */
static int
pc_cmp_radar(struct ieee80211_channel *a, struct ieee80211_channel *b)
{
	/* a is better than b (return < 0) when b is marked while a is not */
	return !!IEEE80211_IS_CHAN_RADAR(a) - !!IEEE80211_IS_CHAN_RADAR(b);
}

/* This function must be invoked with locks acquired */
static int
pc_cmp_keepmode(struct pc_params *params, struct ieee80211_channel *a,
		struct ieee80211_channel *b)
{
	struct ieee80211com *ic = params->vap->iv_ic;
	struct ieee80211_channel *cur = ic->ic_bsschan;

	if (!(params->flags & IEEE80211_SCAN_KEEPMODE))
		return 0;

	/* a is better than b (return < 0) when (a, cur) have the same mode 
	 * and (b, cur) do not. */
	return
		!!IEEE80211_ARE_CHANS_SAME_MODE(b, cur) -
		!!IEEE80211_ARE_CHANS_SAME_MODE(a, cur);
}

/* This function must be invoked with locks acquired */
static int
pc_cmp_sc(struct ieee80211com *ic, struct ieee80211_channel *a,
		struct ieee80211_channel *b)
{
	/* a is better than b (return < 0) when a has more chan nodes than b */
	return
		ic->ic_chan_nodes[b->ic_ieee] -
		ic->ic_chan_nodes[a->ic_ieee];
}

/* This function must be invoked with locks acquired */
static int
pc_cmp_rssi(struct ap_state *as, struct ieee80211_channel *a,
		struct ieee80211_channel *b)
{
	/* a is better than b (return < 0) when a has rssi less than b */
	return
		as->as_maxrssi[a->ic_ieee] -
		as->as_maxrssi[b->ic_ieee];
}

/* This function must be invoked with locks acquired */
static int
pc_cmp_samechan(struct ieee80211com *ic, struct ieee80211_channel *a,
		struct ieee80211_channel *b)
{
	struct ieee80211_channel *ic_bsschan = ic->ic_bsschan;
	if (ic_bsschan == IEEE80211_CHAN_ANYC)
		return 0;
	/* a is better than b (return < 0) when a is current (and b is not) */
	return (b == ic_bsschan) - (a == ic_bsschan);
}

/* This function must be invoked with locks acquired */
static int
pc_cmp_orig(struct channel *a, struct channel *b)
{
	return a->orig - b->orig;
}

/* This function must be invoked with locks acquired */
static int
pc_cmp(const void *_a, const void *_b)
{
	struct ieee80211_channel *a = ((struct channel *)_a)->chan;
	struct ieee80211_channel *b = ((struct channel *)_b)->chan;
	struct pc_params *params = ((struct channel *)_a)->params;
	struct ieee80211com *ic = params->vap->iv_ic;
	int res;

#define EVALUATE_CRITERION(name, ...) do {			\
	if ((res = pc_cmp_##name(__VA_ARGS__)) != 0) 		\
		return res;					\
} while (0)

	EVALUATE_CRITERION(radar, a, b);
	EVALUATE_CRITERION(keepmode, params, a, b);
	EVALUATE_CRITERION(sc, ic, a, b);
	/* XXX: rssi useless? pick_channel evaluates it anyway */
	EVALUATE_CRITERION(rssi, params->ss->ss_priv, a, b);
	EVALUATE_CRITERION(samechan, ic, a, b);
	EVALUATE_CRITERION(orig, (struct channel *)_a, (struct channel *)_b);

#undef EVALUATE_CRITERION
	return res;
}

/* This function must be invoked with locks acquired */
static void
pc_swap(void *a, void *b, int n)
{
	struct ieee80211_channel *t = ((struct channel *)a)->chan;
	int i;

	((struct channel *)a)->chan = ((struct channel *)b)->chan;
	((struct channel *)b)->chan = t;

	i = ((struct channel *)a)->orig;
	((struct channel *)a)->orig = ((struct channel *)b)->orig;
	((struct channel *)b)->orig = i;

	/* (struct channel *)x->params doesn't have to be swapped, because it 
	 * is the same across all channels */
}

/* Pick a quiet channel to use for ap operation.
 * Must be invoked while we hold the locks. */
static struct ieee80211_channel *
pick_channel(struct ieee80211_scan_state *ss, struct ieee80211vap *vap, 
		u_int32_t flags)
{
	struct ieee80211com *ic = vap->iv_ic;
	unsigned int i, best_rssi;
	int ss_last = ss->ss_last;
	struct ieee80211_channel *best;
	struct ap_state *as = ss->ss_priv;
	struct channel chans[ss_last]; /* actually ss_last-1 is required */
	struct channel *c = NULL;
	struct pc_params params = { vap, ss, flags };
	int benefit = 0;
	int sta_assoc = 0;

	for (i = 0; i < ss_last; i++) {
		chans[i].chan = ss->ss_chans[i];
		chans[i].orig = i;
		chans[i].params = &params;
	}

	sort(chans, ss_last, sizeof(*chans), pc_cmp, pc_swap);

#ifdef IEEE80211_DEBUG
	for (i = 0; i < ss_last; i++) {
		int chan = ieee80211_chan2ieee(ic, chans[i].chan);
		IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN, "%s: channel %u, "
				"rssi %d, radar %d, cn %d, km %d\n",
				__func__, chan, as->as_maxrssi[chan], 
				IEEE80211_IS_CHAN_RADAR(chans[i].chan),
				ic->ic_chan_nodes[chans[i].chan->ic_ieee],
				!!IEEE80211_ARE_CHANS_SAME_MODE(chans[i].chan, 
					ic->ic_bsschan));
	}
#endif /* IEEE80211_DEBUG */

	best = NULL;
	best_rssi = 0xff; /* If signal is bigger than 0xff, we'd be melting. */

	for (i = 0; i < ss_last; i++) {
		c = &chans[i];
		benefit = best_rssi - as->as_maxrssi[c->chan->ic_ieee];
		sta_assoc = ic->ic_sta_assoc;

		/* Don't switch... */
		if (benefit <= 0)
			continue;

		/* Verify channel is not marked for non-occupancy */
		if (IEEE80211_IS_CHAN_RADAR(c->chan))
			continue;

		/* Do not select 802.11a ST if mode is specified and is not 
		 * 802.11a ST */
		if (as->as_required_mode &&
		    IEEE80211_IS_CHAN_STURBO(c->chan) &&
		    (as->as_vap_desired_mode != IEEE80211_MODE_TURBO_STATIC_A))
			continue;

		/* Verify mode matches any fixed mode specified */
		if ((c->chan->ic_flags & as->as_required_mode) != 
				as->as_required_mode)
			continue;

		if ((ic->ic_bsschan != NULL) &&
			(ic->ic_bsschan != IEEE80211_CHAN_ANYC)) {

			/* Make sure the channels are the same mode */
			if ((flags & IEEE80211_SCAN_KEEPMODE) &&
				!IEEE80211_ARE_CHANS_SAME_MODE(c->chan,
					ic->ic_bsschan))
				/* break the loop as the subsequent chans won't be 
				 * better */
				break;
		}

		if (sta_assoc != 0) {
			int sl = ic->ic_cn_total - 
				ic->ic_chan_nodes[c->chan->ic_ieee]; /* count */
			if (ic->ic_sc_algorithm == IEEE80211_SC_LOOSE) {
				int sl_max = ic->ic_sc_sldg * benefit;
				sl = 1000 * sl / sta_assoc; /* permil */
				IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
						"%s: chan %d, dB gained: %d, "
						"STAs lost: %d permil (max %d)\n",
						__func__, c->chan->ic_ieee, 
						benefit, sl, sl_max);
				if (sl > sl_max)
					continue;
			} else if (((ic->ic_sc_algorithm == 
						 IEEE80211_SC_TIGHT) ||
					(ic->ic_sc_algorithm == 
						 IEEE80211_SC_STRICT)) && 
					(sl > 0)) {
				/* Break the loop as the subsequent chans 
				 * won't be better. */
				break;
			}
		}
		best = c->chan;
		best_rssi = as->as_maxrssi[best->ic_ieee];
	}

	if (best != NULL) {
		i = best->ic_ieee;
		IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
				"%s: best: channel %u rssi %d\n",
				__func__, i, as->as_maxrssi[i]);
	}
	return best;
}

static int
ap_end(struct ieee80211_scan_state *ss, struct ieee80211vap *vap,
		int (*action)(struct ieee80211vap *, 
			const struct ieee80211_scan_entry *), 
		u_int32_t flags)
{
	struct ap_state *as = ss->ss_priv;
	struct ieee80211_channel *bestchan = NULL;
	struct ieee80211com *ic = NULL;
	int res = 1;

	SCAN_AP_LOCK_IRQ(as);

	KASSERT(vap->iv_opmode == IEEE80211_M_HOSTAP,
		("wrong opmode %u", vap->iv_opmode));

	ic = vap->iv_ic;
	bestchan = pick_channel(ss, vap, flags);
	if (bestchan == NULL) {
		if (ss->ss_last > 0) {
			/* no suitable channel, should not happen */
			printk(KERN_ERR "%s: %s: no suitable channel! "
				"(should not happen)\n", 
				DEV_NAME(vap->iv_dev), __func__);
		}
		res = 1; /* Do NOT restart scan */
	} else {
		struct ieee80211_scan_entry se;
		/* XXX: notify all VAPs? */
		/* if this is a dynamic turbo frequency , start with normal 
		 * mode first */
		if (IEEE80211_IS_CHAN_TURBO(bestchan) && 
				!IEEE80211_IS_CHAN_STURBO(bestchan)) {
			if ((bestchan = ieee80211_find_channel(ic, 
					bestchan->ic_freq, bestchan->ic_flags &
					~IEEE80211_CHAN_TURBO)) == NULL) {
				/* should never happen ?? */
				SCAN_AP_UNLOCK_IRQ_EARLY(as);
				return 0;
			}
		}
		memset(&se, 0, sizeof(se));
		se.se_chan = bestchan;

		as->as_action = ss->ss_ops->scan_default;
		if (action)
			as->as_action = action;
		as->as_selbss = se;

		/* Must defer action to avoid possible recursive call through 
		 * 80211 state machine, which would result in recursive 
		 * locking. */
		IEEE80211_SCHEDULE_TQUEUE(&as->as_actiontq);
		res = 1;
	}
	SCAN_AP_UNLOCK_IRQ(as);
	return res;
}

static void
ap_age(struct ieee80211_scan_state *ss)
{
	struct ap_state *as = ss->ss_priv;
	struct scan_entry *se, *next;

	SCAN_AP_LOCK_IRQ(as);
	TAILQ_FOREACH_SAFE(se, &as->as_entry, se_list, next) {
		if (se->se_notseen > AP_PURGE_SCANS) {
			TAILQ_REMOVE(&as->as_entry, se, se_list);
			LIST_REMOVE(se, se_hash);
			FREE(se, M_80211_SCAN);
		}
	}
	SCAN_AP_UNLOCK_IRQ(as);
}

static int
ap_iterate(struct ieee80211_scan_state *ss,
	ieee80211_scan_iter_func *f, void *arg)
{
	struct ap_state *as = ss->ss_priv;
	struct scan_entry *se;
	u_int gen;
	int res = 0;

	SCAN_AP_GEN_LOCK(as);
	gen = as->as_scangen++;
restart:
	SCAN_AP_LOCK_IRQ(as);
	TAILQ_FOREACH(se, &as->as_entry, se_list) {
		if (se->se_scangen != gen) {
			se->se_scangen = gen;
			/* update public state */
			se->base.se_age = jiffies - se->se_lastupdate;
			SCAN_AP_UNLOCK_IRQ_EARLY(as);

			res = (*f)(arg, &se->base);

			/* We probably ran out of buffer space. */
			if (res != 0)
				goto done;

			goto restart;
		}
	}

	SCAN_AP_UNLOCK_IRQ(as);

done:
	SCAN_AP_GEN_UNLOCK(as);

	return res;
}

static void
ap_assoc_success(struct ieee80211_scan_state *ss,
	const u_int8_t macaddr[IEEE80211_ADDR_LEN])
{
	/* should not be called */
}

static void
ap_assoc_fail(struct ieee80211_scan_state *ss,
	const u_int8_t macaddr[IEEE80211_ADDR_LEN], int reason)
{
	/* should not be called */
}

/*
 * Default action to execute when a scan entry is found for ap
 * mode.  Return 1 on success, 0 on failure
 */
static int
ap_default_action(struct ieee80211vap *vap,
	const struct ieee80211_scan_entry *se)
{
	ieee80211_create_ibss(vap, se->se_chan);

	return 1;
}

static void
action_tasklet(IEEE80211_TQUEUE_ARG data)
{
	struct ieee80211_scan_state *ss = (struct ieee80211_scan_state *)data;
	struct ap_state *as = (struct ap_state *)ss->ss_priv;
	struct ieee80211vap *vap = ss->ss_vap;
	SCAN_AP_LOCK_IRQ(as);
	if (as->as_newscan) {
		struct scan_entry *se;
		TAILQ_FOREACH(se, &as->as_entry, se_list) {
			/*
			 * If seen then reset and don't bump the count;
			 * otherwise bump the ``not seen'' count.  Note
			 * that this ensures that stations for which we
			 * see frames while not scanning but not during
			 * this scan will not be penalized.
			 */
			if (se->se_seen)
				se->se_seen = 0;
			else
				se->se_notseen++;
		}
		as->as_newscan = 0;
	}
	SCAN_AP_UNLOCK_IRQ(as);

	(*ss->ss_ops->scan_default)(vap, &as->as_selbss);
}

/*
 * Module glue.
 */
MODULE_AUTHOR("Errno Consulting, Sam Leffler");
MODULE_DESCRIPTION("802.11 wireless support: default ap scanner");
#ifdef MODULE_LICENSE
MODULE_LICENSE("Dual BSD/GPL");
#endif

static const struct ieee80211_scanner ap_default = {
	.scan_name		= "default",
	.scan_attach		= ap_attach,
	.scan_detach		= ap_detach,
	.scan_start		= ap_start,
	.scan_restart		= ap_restart,
	.scan_cancel		= ap_cancel,
	.scan_end		= ap_end,
	.scan_flush		= ap_flush,
	.scan_add		= ap_add,
	.scan_age		= ap_age,
	.scan_iterate		= ap_iterate,
	.scan_assoc_success	= ap_assoc_success,
	.scan_assoc_fail	= ap_assoc_fail,
	.scan_default		= ap_default_action,
};


static int __init
init_scanner_ap(void)
{
	ieee80211_scanner_register(IEEE80211_M_HOSTAP, &ap_default);
	return 0;
}
module_init(init_scanner_ap);

static void __exit
exit_scanner_ap(void)
{
	ieee80211_scanner_unregister_all(&ap_default);
}
module_exit(exit_scanner_ap);
