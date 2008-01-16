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

/*
 * Driver for the Atheros Wireless LAN controller.
 *
 * This software is derived from work of Atsushi Onoe; his contribution
 * is greatly appreciated.
 */
#include "opt_ah.h"

#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/random.h>
#include <linux/delay.h>
#include <linux/cache.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/if_arp.h>
#include <linux/rtnetlink.h>
#include <linux/time.h>
#include <asm/uaccess.h>

#include "if_ethersubr.h"		/* for ETHER_IS_MULTICAST */
#include "if_media.h"
#include "if_llc.h"

#define	AR_DEBUG

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_radiotap.h>
#include <net80211/ieee80211_monitor.h>
#include <net80211/ieee80211_rate.h>

#ifdef USE_HEADERLEN_RESV
#include <net80211/if_llc.h>
#endif

#include "net80211/if_athproto.h"
#include "if_athvar.h"

#include "ah_desc.h"

#include "ah_devid.h"			/* XXX to identify chipset */

#ifdef ATH_PCI		/* PCI BUS */
#include "if_ath_pci.h"
#endif			/* PCI BUS */
#ifdef ATH_AHB		/* AHB BUS */
#include "if_ath_ahb.h"
#endif			/* AHB BUS */

#include "ah.h"
#include "if_ath_hal.h"

#ifdef ATH_TX99_DIAG
#include "ath_tx99.h"
#endif

/* unaligned little endian access */
#define LE_READ_2(p)							\
	((u_int16_t)							\
	 ((((u_int8_t *)(p))[0]      ) | (((u_int8_t *)(p))[1] <<  8)))
#define LE_READ_4(p)							\
	((u_int32_t)							\
	 ((((u_int8_t *)(p))[0]      ) | (((u_int8_t *)(p))[1] <<  8) |	\
	  (((u_int8_t *)(p))[2] << 16) | (((u_int8_t *)(p))[3] << 24)))

/* Default rate control algorithm */
#ifdef CONFIG_ATHEROS_RATE_DEFAULT
#define DEF_RATE_CTL CONFIG_ATHEROS_RATE_DEFAULT
#else
#define DEF_RATE_CTL "sample"
#endif

enum {
	ATH_LED_TX,
	ATH_LED_RX,
	ATH_LED_POLL,
};

static struct ieee80211vap *ath_vap_create(struct ieee80211com *,
	const char *, int, int, struct net_device *);
static void ath_vap_delete(struct ieee80211vap *);
static int ath_init(struct net_device *);
static int ath_set_ack_bitrate(struct ath_softc *, int);
static int ath_reset(struct net_device *);
static void ath_fatal_tasklet(TQUEUE_ARG);
static void ath_rxorn_tasklet(TQUEUE_ARG);
static void ath_bmiss_tasklet(TQUEUE_ARG);
static void ath_bstuck_tasklet(TQUEUE_ARG);
static int ath_stop_locked(struct net_device *);
static int ath_stop(struct net_device *);
#if 0
static void ath_initkeytable(struct ath_softc *);
#endif
static ieee80211_keyix_t ath_key_alloc(struct ieee80211vap *, 
		const struct ieee80211_key *);
static int ath_key_delete(struct ieee80211vap *, const struct ieee80211_key *,
	struct ieee80211_node *);
static int ath_key_set(struct ieee80211vap *, const struct ieee80211_key *,
	const u_int8_t mac[IEEE80211_ADDR_LEN]);
static void ath_key_update_begin(struct ieee80211vap *);
static void ath_key_update_end(struct ieee80211vap *);
static void ath_mode_init(struct net_device *);
static void ath_setslottime(struct ath_softc *);
static void ath_updateslot(struct net_device *);
static int ath_beaconq_setup(struct ath_softc *);
static int ath_beacon_alloc(struct ath_softc *, struct ieee80211_node *);
#ifdef ATH_SUPERG_DYNTURBO
static void ath_beacon_dturbo_update(struct ieee80211vap *, int *, u_int8_t);
static void ath_beacon_dturbo_config(struct ieee80211vap *, u_int32_t);
static void ath_turbo_switch_mode(unsigned long);
static int ath_check_beacon_done(struct ath_softc *);
#endif
static void ath_beacon_send(struct ath_softc *, int *);
static void ath_beacon_config(struct ath_softc *, struct ieee80211vap *);
static void ath_beacon_return(struct ath_softc *, struct ath_buf *);
static void ath_beacon_free(struct ath_softc *);
static void ath_beacon_update_timers(struct ath_softc *, struct ieee80211vap *);
static int ath_desc_alloc(struct ath_softc *);
static void ath_desc_free(struct ath_softc *);
static void ath_desc_swap(struct ath_desc *);

#ifdef IEEE80211_DEBUG_REFCNT
static struct ieee80211_node *ath_node_alloc_debug(struct ieee80211vap *, 
		const char* func, int line);
static void ath_node_cleanup_debug(struct ieee80211_node *, const char* func, 
		int line);
static void ath_node_free_debug(struct ieee80211_node *, const char* func, 
		int line);
#else
static struct ieee80211_node *ath_node_alloc(struct ieee80211vap *);
static void ath_node_cleanup(struct ieee80211_node *);
static void ath_node_free(struct ieee80211_node *);
#endif

static u_int8_t ath_node_getrssi(const struct ieee80211_node *);
static int ath_rxbuf_init(struct ath_softc *, struct ath_buf *);
static void ath_recv_mgmt(struct ieee80211_node *, struct sk_buff *, int,
	int, u_int64_t);
static void ath_setdefantenna(struct ath_softc *, u_int);
static struct ath_txq *ath_txq_setup(struct ath_softc *, int, int);
static void ath_rx_tasklet(TQUEUE_ARG);
static int ath_hardstart(struct sk_buff *, struct net_device *);
static int ath_mgtstart(struct ieee80211com *, struct sk_buff *);
#ifdef ATH_SUPERG_COMP
static u_int32_t ath_get_icvlen(struct ieee80211_key *);
static u_int32_t ath_get_ivlen(struct ieee80211_key *);
static void ath_setup_comp(struct ieee80211_node *, int);
static void ath_comp_set(struct ieee80211vap *, struct ieee80211_node *, int);
#endif
static int ath_tx_setup(struct ath_softc *, int, int);
static int ath_wme_update(struct ieee80211com *);
static void ath_uapsd_flush(struct ieee80211_node *);
static void ath_tx_cleanupq(struct ath_softc *, struct ath_txq *);
static void ath_tx_cleanup(struct ath_softc *);
static void ath_tx_uapsdqueue(struct ath_softc *, struct ath_node *,
	struct ath_buf *);

static int ath_tx_start(struct net_device *, struct ieee80211_node *,
	struct ath_buf *, struct sk_buff *, int);
static void ath_tx_tasklet_q0(TQUEUE_ARG);
static void ath_tx_tasklet_q0123(TQUEUE_ARG);
static void ath_tx_tasklet(TQUEUE_ARG);
static void ath_tx_timeout(struct net_device *);
static void ath_tx_draintxq(struct ath_softc *, struct ath_txq *);
static int ath_chan_set(struct ath_softc *, struct ieee80211_channel *);
static void ath_draintxq(struct ath_softc *);
static __inline void ath_tx_txqaddbuf(struct ath_softc *, struct ieee80211_node *,
	struct ath_txq *, struct ath_buf *, struct ath_desc *, int);
static void ath_stoprecv(struct ath_softc *);
static int ath_startrecv(struct ath_softc *);
static void ath_flushrecv(struct ath_softc *);
static void ath_chan_change(struct ath_softc *, struct ieee80211_channel *);
static void ath_calibrate(unsigned long);
static void ath_mib_enable(unsigned long);
static int ath_newstate(struct ieee80211vap *, enum ieee80211_state, int);

static void ath_scan_start(struct ieee80211com *);
static void ath_scan_end(struct ieee80211com *);
static void ath_set_channel(struct ieee80211com *);
static void ath_set_coverageclass(struct ieee80211com *);
static u_int ath_mhz2ieee(struct ieee80211com *, u_int, u_int);
#ifdef ATH_SUPERG_FF
static int athff_can_aggregate(struct ath_softc *, struct ether_header *,
	struct ath_node *, struct sk_buff *, u_int16_t, int *);
#endif
static struct net_device_stats *ath_getstats(struct net_device *);
static void ath_setup_stationkey(struct ieee80211_node *);
static void ath_setup_stationwepkey(struct ieee80211_node *);
static void ath_setup_keycacheslot(struct ath_softc *, struct ieee80211_node *);
static void ath_newassoc(struct ieee80211_node *, int);
static int ath_getchannels(struct net_device *, u_int, HAL_BOOL, HAL_BOOL);
static void ath_led_event(struct ath_softc *, int);
static void ath_update_txpow(struct ath_softc *);

#ifdef ATH_REVERSE_ENGINEERING
/* Reverse engineering utility commands */
static void ath_registers_dump(struct ieee80211com *ic);
static void ath_registers_dump_delta(struct ieee80211com *ic);
static void ath_registers_mark(struct ieee80211com *ic);
static unsigned int ath_read_register(struct ieee80211com *ic, 
		unsigned int address, unsigned int* value);
static unsigned int ath_write_register(struct ieee80211com *ic, 
		unsigned int address, unsigned int value);
static void ath_ar5212_registers_dump(struct ath_softc *sc);
static void ath_print_register(const char* name, u_int32_t address, 
		u_int32_t v);
static void ath_print_register_delta(const char* name, u_int32_t address, 
		u_int32_t v_old, u_int32_t v_new);
#endif /* #ifdef ATH_REVERSE_ENGINEERING */

static int ath_set_mac_address(struct net_device *, void *);
static int ath_change_mtu(struct net_device *, int);
static int ath_ioctl(struct net_device *, struct ifreq *, int);

static int ath_rate_setup(struct net_device *, u_int);
static void ath_setup_subrates(struct net_device *);
#ifdef ATH_SUPERG_XR
static int ath_xr_rate_setup(struct net_device *);
static void ath_grppoll_txq_setup(struct ath_softc *, int, int);
static void ath_grppoll_start(struct ieee80211vap *, int);
static void ath_grppoll_stop(struct ieee80211vap *);
static u_int8_t ath_node_move_data(const struct ieee80211_node *);
static void ath_grppoll_txq_update(struct ath_softc *, int);
static void ath_grppoll_period_update(struct ath_softc *);
#endif
static void ath_setcurmode(struct ath_softc *, enum ieee80211_phymode);

static void ath_dynamic_sysctl_register(struct ath_softc *);
static void ath_dynamic_sysctl_unregister(struct ath_softc *);
static void ath_announce(struct net_device *);
static int ath_descdma_setup(struct ath_softc *, struct ath_descdma *,
	ath_bufhead *, const char *, int, int);
static void ath_descdma_cleanup(struct ath_softc *, struct ath_descdma *,
	ath_bufhead *, int);
static void ath_check_dfs_clear(unsigned long);
static const char *ath_get_hal_status_desc(HAL_STATUS status);
static int ath_rcv_dev_event(struct notifier_block *, unsigned long, void *);

#ifdef IEEE80211_DEBUG_REFCNT
#define ath_return_txbuf(_sc, _pbuf) \
	ath_return_txbuf_debug(_sc, _pbuf, __func__, __LINE__)
static void ath_return_txbuf_debug(struct ath_softc *sc, struct ath_buf **buf, 
		const char* func, int line);
#else /* #ifdef IEEE80211_DEBUG_REFCNT */
static void ath_return_txbuf(struct ath_softc *sc, struct ath_buf **buf);
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */

#ifdef IEEE80211_DEBUG_REFCNT
#define ath_return_txbuf_locked(_sc, _pbuf) \
	ath_return_txbuf_locked_debug(_sc, _pbuf, __func__, __LINE__)
static void ath_return_txbuf_locked_debug(struct ath_softc *sc, struct ath_buf **buf, 
		const char* func, int line);
#else /* #ifdef IEEE80211_DEBUG_REFCNT */
static void ath_return_txbuf_locked(struct ath_softc *sc, struct ath_buf **buf);
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */

#ifdef IEEE80211_DEBUG_REFCNT
#define ath_return_txbuf_list(_sc, _head) \
	ath_return_txbuf_list_debug(_sc, _head, __func__, __LINE__)
static void ath_return_txbuf_list_debug(struct ath_softc *sc, ath_bufhead *bfhead, 
		const char* func, int line);
#else /* #ifdef IEEE80211_DEBUG_REFCNT */
static void ath_return_txbuf_list(struct ath_softc *sc, ath_bufhead *bfhead);
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */

#ifdef IEEE80211_DEBUG_REFCNT
#define ath_return_txbuf_list_locked(_sc, _head) \
	ath_return_txbuf_list_locked_debug(_sc, _head, __func__, __LINE__)
static void ath_return_txbuf_list_locked_debug(struct ath_softc *sc, ath_bufhead *bfhead, 
		const char* func, int line);
#else /* #ifdef IEEE80211_DEBUG_REFCNT */
static void ath_return_txbuf_list_locked(struct ath_softc *sc, ath_bufhead *bfhead);
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */

#ifdef IEEE80211_DEBUG_REFCNT
#define cleanup_ath_buf(_sc, _buf, _dir) \
	cleanup_ath_buf_debug(_sc, _buf, _dir, __func__, __LINE__)
static struct ath_buf* cleanup_ath_buf_debug(struct ath_softc *sc, struct ath_buf *buf, 
		int direction, const char* func, int line);
#else /* #ifdef IEEE80211_DEBUG_REFCNT */
static struct ath_buf* cleanup_ath_buf(struct ath_softc *sc, struct ath_buf *buf, 
		int direction);
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */

/*
Regulatory agency testing - continuous transmit support
*/
static void txcont_on(struct ieee80211com *ic);
static void txcont_off(struct ieee80211com *ic);

static int ath_get_txcont(struct ieee80211com *);
static void ath_set_txcont(struct ieee80211com *, int);

static int ath_get_txcont_power(struct ieee80211com *);
static void ath_set_txcont_power(struct ieee80211com *, unsigned int);

static unsigned int ath_get_txcont_rate(struct ieee80211com *);
static void ath_set_txcont_rate(struct ieee80211com *ic, unsigned int new_rate);


static u_int32_t ath_get_clamped_maxtxpower(struct ath_softc *sc);
static u_int32_t ath_set_clamped_maxtxpower(struct ath_softc *sc, u_int32_t new_clamped_maxtxpower);
static u_int32_t ath_get_real_maxtxpower(struct ath_softc *sc);

/* calibrate every 30 secs in steady state but check every second at first. */
static int ath_calinterval = ATH_SHORT_CALINTERVAL;
static int ath_countrycode = CTRY_DEFAULT;	/* country code */
static int ath_outdoor = AH_FALSE;		/* enable outdoor use */
static int ath_xchanmode = AH_TRUE;		/* enable extended channels */
static char *autocreate = NULL;
static char *ratectl = DEF_RATE_CTL;
static int rfkill = 0;
static int tpc = 0;
static int countrycode = -1;
static int outdoor = -1;
static int xchanmode = -1;

static const char *hal_status_desc[] = {
	"No error",
	"No hardware present or device not yet supported",
	"Memory allocation failed",
	"Hardware didn't respond as expected",
	"EEPROM magic number invalid",
	"EEPROM version invalid",
	"EEPROM unreadable",
	"EEPROM checksum invalid",
	"EEPROM read problem",
	"EEPROM mac address invalid",
	"EEPROM size not supported",
	"Attempt to change write-locked EEPROM",
	"Invalid parameter to function",
	"Hardware revision not supported",
	"Hardware self-test failed",
	"Operation incomplete"
};

static struct notifier_block ath_event_block = {
	.notifier_call = ath_rcv_dev_event
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,52))
MODULE_PARM(countrycode, "i");
MODULE_PARM(outdoor, "i");
MODULE_PARM(xchanmode, "i");
MODULE_PARM(rfkill, "i");
#ifdef ATH_CAP_TPC
MODULE_PARM(tpc, "i");
#endif
MODULE_PARM(autocreate, "s");
MODULE_PARM(ratectl, "s");
#else
#include <linux/moduleparam.h>
module_param(countrycode, int, 0600);
module_param(outdoor, int, 0600);
module_param(xchanmode, int, 0600);
module_param(rfkill, int, 0600);
#ifdef ATH_CAP_TPC
module_param(tpc, int, 0600);
#endif
module_param(autocreate, charp, 0600);
module_param(ratectl, charp, 0600);
#endif
MODULE_PARM_DESC(countrycode, "Override default country code");
MODULE_PARM_DESC(outdoor, "Enable/disable outdoor use");
MODULE_PARM_DESC(xchanmode, "Enable/disable extended channel mode");
MODULE_PARM_DESC(rfkill, "Enable/disable RFKILL capability");
#ifdef ATH_CAP_TPC
MODULE_PARM_DESC(tpc, "Enable/disable per-packet transmit power control (TPC) "
		"capability");
#endif
MODULE_PARM_DESC(autocreate, "Create ath device in "
		"[sta|ap|wds|adhoc|ahdemo|monitor] mode. defaults to sta, use "
		"'none' to disable");
MODULE_PARM_DESC(ratectl, "Rate control algorithm [amrr|minstrel|onoe|sample], "
		"defaults to '" DEF_RATE_CTL "'");

static int	ath_debug = 0;
#ifdef AR_DEBUG
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,52))
MODULE_PARM(ath_debug, "i");
#else
module_param(ath_debug, int, 0600);
#endif
MODULE_PARM_DESC(ath_debug, "Load-time driver debug output enable");


static int	ieee80211_debug = 0;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,52))
MODULE_PARM(ieee80211_debug, "i");
#else
module_param(ieee80211_debug, int, 0600);
#endif
MODULE_PARM_DESC(ieee80211_debug, "Load-time 802.11 debug output enable");

static void ath_printrxbuf(const struct ath_buf *, int);
static void ath_printtxbuf(const struct ath_buf *, int);

#define	DFLAG_ISSET(sc, _m) \
	(((sc->sc_debug | ath_debug_global) & _m))
#define	IFF_DUMPPKTS(sc, _m) \
	DFLAG_ISSET(sc, (_m))
enum {
	ATH_DEBUG_XMIT		= 0x00000001,	/* basic xmit operation */
	ATH_DEBUG_XMIT_DESC	= 0x00000002,	/* xmit descriptors */
	ATH_DEBUG_RECV		= 0x00000004,	/* basic recv operation */
	ATH_DEBUG_RECV_DESC	= 0x00000008,	/* recv descriptors */
	ATH_DEBUG_RATE		= 0x00000010,	/* rate control */
	ATH_DEBUG_RESET		= 0x00000020,	/* reset processing */
	ATH_DEBUG_SKB_REF	= 0x00000040,	/* skb references */
	ATH_DEBUG_BEACON	= 0x00000080,	/* beacon handling */
	ATH_DEBUG_WATCHDOG	= 0x00000100,	/* watchdog timeout */
	ATH_DEBUG_INTR		= 0x00001000,	/* ISR */
	ATH_DEBUG_TX_PROC	= 0x00002000,	/* tx ISR proc */
	ATH_DEBUG_RX_PROC	= 0x00004000,	/* rx ISR proc */
	ATH_DEBUG_BEACON_PROC	= 0x00008000,	/* beacon ISR proc */
	ATH_DEBUG_CALIBRATE	= 0x00010000,	/* periodic calibration */
	ATH_DEBUG_KEYCACHE	= 0x00020000,	/* key cache management */
	ATH_DEBUG_STATE		= 0x00040000,	/* 802.11 state transitions */
	ATH_DEBUG_TSF		= 0x00080000,	/* timestamp processing */
	ATH_DEBUG_LED		= 0x00100000,	/* led management */
	ATH_DEBUG_FF		= 0x00200000,	/* fast frames */
	ATH_DEBUG_TURBO		= 0x00400000,	/* turbo/dynamic turbo */
	ATH_DEBUG_UAPSD		= 0x00800000,	/* uapsd */
	ATH_DEBUG_DOTH		= 0x01000000,	/* 11.h */
	ATH_DEBUG_DOTHFILT	= 0x02000000,	/* 11.h radar pulse filter algorithm */
	ATH_DEBUG_DOTHFILTVBSE	= 0x04000000,	/* 11.h radar pulse filter algorithm - 
						 * pulse level debug */
	ATH_DEBUG_DOTHFILTNOSC  = 0x08000000,	/* 11.h radar pulse filter algorithm - 
						 * disable short circuit of processing 
						 * after detection */
	ATH_DEBUG_DOTHPULSES    = 0x10000000,   /* 11.h radar pulse events */
	ATH_DEBUG_TXBUF         = 0x20000000,   /* TX buffer usage/leak debugging */
	ATH_DEBUG_SKB           = 0x40000000,   /* SKB usage/leak debugging [applies 
						 * to all vaps] */
	ATH_DEBUG_FATAL		= 0x80000000,	/* fatal errors */
	ATH_DEBUG_ANY		= 0xffffffff,
	ATH_DEBUG_GLOBAL	= (ATH_DEBUG_SKB|ATH_DEBUG_SKB_REF)
};
#define	DPRINTF(sc, _m, _fmt, ...) do {					\
	if (DFLAG_ISSET(sc, (_m))) {					\
		printk("%s: " _fmt, DEV_NAME(sc->sc_dev), __VA_ARGS__);	\
	} 								\
} while (0)
#define	KEYPRINTF(sc, ix, hk, mac) do {				\
	if (DFLAG_ISSET(sc, ATH_DEBUG_KEYCACHE))		\
		ath_keyprint(sc, __func__, ix, hk, mac);	\
} while (0)
#else /* defined(AR_DEBUG) */
#define	IFF_DUMPPKTS(sc, _m)		0
#define	DPRINTF(sc, _m, _fmt, ...)
#define	KEYPRINTF(sc, k, ix, mac)
#endif /* defined(AR_DEBUG) */

static atomic_t ath_buf_counter = ATOMIC_INIT(0);

#define ATH_SETUP_XR_VAP(sc,vap,rfilt)						\
	do {									\
		if (sc->sc_curchan.privFlags & CHANNEL_4MS_LIMIT)		\
			vap->iv_fragthreshold = XR_4MS_FRAG_THRESHOLD;		\
		else								\
			vap->iv_fragthreshold = vap->iv_xrvap->iv_fragthreshold;\
		if (!sc->sc_xrgrppoll) {					\
			ath_grppoll_txq_setup(sc, HAL_TX_QUEUE_DATA,		\
					GRP_POLL_PERIOD_NO_XR_STA(sc));		\
			ath_grppoll_start(vap, sc->sc_xrpollcount);		\
			ath_hal_setrxfilter(sc->sc_ah, 				\
					rfilt|HAL_RX_FILTER_XRPOLL);		\
		} \
	} while (0)

/*
 * Define the scheme that we select MAC address for multiple BSS on the same radio.
 * The very first VAP will just use the MAC address from the EEPROM.
 * For the next 3 VAPs, we set the U/L bit (bit 1) in MAC address,
 * and use the higher bits as the index of the VAP.
 */
#define ATH_SET_VAP_BSSID_MASK(bssid_mask)				\
	((bssid_mask)[0] &= ~(((ATH_BCBUF-1) << 2) | 0x02))
#define ATH_GET_VAP_ID(bssid)                   ((bssid)[0] >> 2)
#define ATH_SET_VAP_BSSID(bssid, id)					\
		do {							\
			if (id)						\
				(bssid)[0] |= (((id) << 2) | 0x02);	\
		} while (0)

int
ath_attach(u_int16_t devid, struct net_device *dev, HAL_BUS_TAG tag)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211vap *vap;
	struct ath_hal *ah;
	HAL_STATUS status;
	int error = 0;
	unsigned int i;
	int autocreatemode = IEEE80211_M_STA;
	u_int8_t csz;

	sc->devid = devid;
	ath_debug_global = (ath_debug & ATH_DEBUG_GLOBAL);
	sc->sc_debug 	 = (ath_debug & ~ATH_DEBUG_GLOBAL);
	DPRINTF(sc, ATH_DEBUG_ANY, "%s: devid 0x%x\n", __func__, devid);

	/*
	 * Cache line size is used to size and align various
	 * structures used to communicate with the hardware.
	 */
	bus_read_cachesize(sc, &csz);
	/* XXX assert csz is non-zero */
	sc->sc_cachelsz = csz << 2;		/* convert to bytes */

	ATH_LOCK_INIT(sc);
	ATH_HAL_LOCK_INIT(sc);
	ATH_TXBUF_LOCK_INIT(sc);
	ATH_RXBUF_LOCK_INIT(sc);

	ATH_INIT_TQUEUE(&sc->sc_rxtq,     ath_rx_tasklet,	dev);
	ATH_INIT_TQUEUE(&sc->sc_txtq,	  ath_tx_tasklet,	dev);
	ATH_INIT_TQUEUE(&sc->sc_bmisstq,  ath_bmiss_tasklet,	dev);
	ATH_INIT_TQUEUE(&sc->sc_bstucktq, ath_bstuck_tasklet,	dev);
	ATH_INIT_TQUEUE(&sc->sc_rxorntq,  ath_rxorn_tasklet,	dev);
	ATH_INIT_TQUEUE(&sc->sc_fataltq,  ath_fatal_tasklet,	dev);

	/*
	 * Attach the HAL and verify ABI compatibility by checking
	 * the HAL's ABI signature against the one the driver was
	 * compiled with.  A mismatch indicates the driver was
	 * built with an ah.h that does not correspond to the HAL
	 * module loaded in the kernel.
	 */
	ah = _ath_hal_attach(devid, sc, tag, sc->sc_iobase, &status);
	if (ah == NULL) {
		printk(KERN_ERR "%s: unable to attach hardware: '%s' "
			"(HAL status %u)\n",
			DEV_NAME(dev), ath_get_hal_status_desc(status), 
			status);
		error = ENXIO;
		goto bad;
	}
	if (ah->ah_abi != HAL_ABI_VERSION) {
		printk(KERN_ERR "%s: HAL ABI mismatch; "
			"driver expects 0x%x, HAL reports 0x%x\n",
			DEV_NAME(dev), HAL_ABI_VERSION, ah->ah_abi);
		error = ENXIO;		/* XXX */
		goto bad;
	}
	sc->sc_ah = ah;

	/*
	 * Check if the MAC has multi-rate retry support.
	 * We do this by trying to setup a fake extended
	 * descriptor.  MACs that don't have support will
	 * return false w/o doing anything.  MACs that do
	 * support it will return true w/o doing anything.
	 */
	sc->sc_mrretry = ath_hal_setupxtxdesc(ah, NULL, 0,0, 0,0, 0,0);

	/*
	 * Check if the device has hardware counters for PHY
	 * errors.  If so we need to enable the MIB interrupt
	 * so we can act on stat triggers.
	 */
	if (ath_hal_hwphycounters(ah))
		sc->sc_needmib = 1;

	/*
	 * Get the hardware key cache size.
	 */
	sc->sc_keymax = ath_hal_keycachesize(ah);
	if (sc->sc_keymax > ATH_KEYMAX) {
		printk("%s: Warning, using only %u entries in %u key cache\n",
			DEV_NAME(dev), ATH_KEYMAX, sc->sc_keymax);
		sc->sc_keymax = ATH_KEYMAX;
	}
	/*
	 * Reset the key cache since some parts do not
	 * reset the contents on initial power up.
	 */
	for (i = 0; i < sc->sc_keymax; i++)
		ath_hal_keyreset(ah, i);

	/*
	 * Collect the channel list using the default country
	 * code and including outdoor channels.  The 802.11 layer
	 * is responsible for filtering this list based on settings
	 * like the phy mode.
	 */
	if (countrycode != -1)
		ath_countrycode = countrycode;
	if (outdoor != -1)
		ath_outdoor = outdoor;
	if (xchanmode != -1)
		ath_xchanmode = xchanmode;
	error = ath_getchannels(dev, ath_countrycode,
			ath_outdoor, ath_xchanmode);
	if (error != 0)
		goto bad;

	ic->ic_country_code = ath_countrycode;
	ic->ic_country_outdoor = ath_outdoor;

	printk(KERN_INFO "ath_pci: switching rfkill capability %s\n",
		rfkill ? "on" : "off");
	ath_hal_setrfsilent(ah, rfkill);

	/*
	 * Setup rate tables for all potential media types.
	 */
	ath_rate_setup(dev, IEEE80211_MODE_11A);
	ath_rate_setup(dev, IEEE80211_MODE_11B);
	ath_rate_setup(dev, IEEE80211_MODE_11G);
	ath_rate_setup(dev, IEEE80211_MODE_TURBO_A);
	ath_rate_setup(dev, IEEE80211_MODE_TURBO_G);

	/* Setup for half/quarter rates */
	ath_setup_subrates(dev);

	/* NB: setup here so ath_rate_update is happy */
	ath_setcurmode(sc, IEEE80211_MODE_11A);

	/*
	 * Allocate tx+rx descriptors and populate the lists.
	 */
	error = ath_desc_alloc(sc);
	if (error != 0) {
		printk(KERN_ERR "%s: failed to allocate descriptors: %d\n",
			DEV_NAME(dev), error);
		goto bad;
	}

	/*
	 * Init ic_caps prior to queue init, since WME cap setting
	 * depends on queue setup.
	 */
	ic->ic_caps = 0;

	/*
	 * Allocate hardware transmit queues: one queue for
	 * beacon frames and one data queue for each QoS
	 * priority.  Note that the HAL handles resetting
	 * these queues at the needed time.
	 *
	 * XXX PS-Poll
	 */
	sc->sc_bhalq = ath_beaconq_setup(sc);
	if (sc->sc_bhalq == (u_int) -1) {
		printk(KERN_ERR "%s: unable to setup a beacon xmit queue!\n",
			DEV_NAME(dev));
		error = EIO;
		goto bad2;
	}
	/* CAB: Crap After Beacon - a beacon gated queue */
	sc->sc_cabq = ath_txq_setup(sc, HAL_TX_QUEUE_CAB, 0);
	if (sc->sc_cabq == NULL) {
		printk(KERN_ERR "%s: unable to setup CAB xmit queue!\n",
			DEV_NAME(dev));
		error = EIO;
		goto bad2;
	}
	/* NB: ensure BK queue is the lowest priority h/w queue */
	if (!ath_tx_setup(sc, WME_AC_BK, HAL_WME_AC_BK)) {
		printk(KERN_ERR "%s: unable to setup xmit queue for %s traffic!\n",
			DEV_NAME(dev), ieee80211_wme_acnames[WME_AC_BK]);
		error = EIO;
		goto bad2;
	}
	if (!ath_tx_setup(sc, WME_AC_BE, HAL_WME_AC_BE) ||
	    !ath_tx_setup(sc, WME_AC_VI, HAL_WME_AC_VI) ||
	    !ath_tx_setup(sc, WME_AC_VO, HAL_WME_AC_VO)) {
		/*
		 * Not enough hardware tx queues to properly do WME;
		 * just punt and assign them all to the same h/w queue.
		 * We could do a better job of this if, for example,
		 * we allocate queues when we switch from station to
		 * AP mode.
		 */
		if (sc->sc_ac2q[WME_AC_VI] != NULL)
			ath_tx_cleanupq(sc, sc->sc_ac2q[WME_AC_VI]);
		if (sc->sc_ac2q[WME_AC_BE] != NULL)
			ath_tx_cleanupq(sc, sc->sc_ac2q[WME_AC_BE]);
		sc->sc_ac2q[WME_AC_BE] = sc->sc_ac2q[WME_AC_BK];
		sc->sc_ac2q[WME_AC_VI] = sc->sc_ac2q[WME_AC_BK];
		sc->sc_ac2q[WME_AC_VO] = sc->sc_ac2q[WME_AC_BK];
	} else {
		/*
		 * Mark WME capability since we have sufficient
		 * hardware queues to do proper priority scheduling.
		 */
		ic->ic_caps |= IEEE80211_C_WME;
		sc->sc_uapsdq = ath_txq_setup(sc, HAL_TX_QUEUE_UAPSD, 0);
		if (sc->sc_uapsdq == NULL)
			DPRINTF(sc, ATH_DEBUG_UAPSD, 
				"%s: unable to setup UAPSD xmit queue!\n",
				__func__);
		else {
			ic->ic_caps |= IEEE80211_C_UAPSD;
			/*
			 * default UAPSD on if HW capable
			 */
			IEEE80211_COM_UAPSD_ENABLE(ic);
		}
	}
#ifdef ATH_SUPERG_XR
	ath_xr_rate_setup(dev);
	sc->sc_xrpollint = XR_DEFAULT_POLL_INTERVAL;
	sc->sc_xrpollcount = XR_DEFAULT_POLL_COUNT;
	strcpy(sc->sc_grppoll_str, XR_DEFAULT_GRPPOLL_RATE_STR);
	sc->sc_grpplq.axq_qnum = -1;
	sc->sc_xrtxq = ath_txq_setup(sc, HAL_TX_QUEUE_DATA, HAL_XR_DATA);
#endif

	/*
	 * Special case certain configurations.  Note the
	 * CAB queue is handled by these specially so don't
	 * include them when checking the txq setup mask.
	 */
	switch (sc->sc_txqsetup & ~((1 << sc->sc_cabq->axq_qnum) |
				(sc->sc_uapsdq ? 
				 (1 << sc->sc_uapsdq->axq_qnum) : 0))) {
	case 0x01:
		ATH_INIT_TQUEUE(&sc->sc_txtq, ath_tx_tasklet_q0, dev);
		break;
	case 0x0f:
		ATH_INIT_TQUEUE(&sc->sc_txtq, ath_tx_tasklet_q0123, dev);
		break;
	}

	sc->sc_setdefantenna = ath_setdefantenna;
	sc->sc_rc = ieee80211_rate_attach(sc, ratectl);
	if (sc->sc_rc == NULL) {
		error = EIO;
		goto bad2;
	}
	init_timer(&sc->sc_cal_ch);
	sc->sc_cal_ch.function = ath_calibrate;
	sc->sc_cal_ch.data = (unsigned long) dev;

	init_timer(&sc->sc_mib_enable);
	sc->sc_mib_enable.function = ath_mib_enable;
	sc->sc_mib_enable.data = (unsigned long) sc;

#ifdef ATH_SUPERG_DYNTURBO
	init_timer(&sc->sc_dturbo_switch_mode);
	sc->sc_dturbo_switch_mode.function = ath_turbo_switch_mode;
	sc->sc_dturbo_switch_mode.data = (unsigned long) dev;
#endif

	sc->sc_blinking = 0;
	sc->sc_ledstate = 1;
	sc->sc_ledon = 0;				/* low true */
	sc->sc_ledidle = msecs_to_jiffies(2700);	/* 2.7 sec */
	init_timer(&sc->sc_ledtimer);
	init_timer(&sc->sc_dfswaittimer);
	sc->sc_ledtimer.data = (unsigned long) sc;
	if (sc->sc_softled) {
		ath_hal_gpioCfgOutput(ah, sc->sc_ledpin);
		ath_hal_gpioset(ah, sc->sc_ledpin, !sc->sc_ledon);
	}

	/* NB: ether_setup is done by bus-specific code */
	dev->open = ath_init;
	dev->stop = ath_stop;
	dev->hard_start_xmit = ath_hardstart;
	dev->tx_timeout = ath_tx_timeout;
	dev->watchdog_timeo = 5 * HZ;
	dev->set_multicast_list = ath_mode_init;
	dev->do_ioctl = ath_ioctl;
	dev->get_stats = ath_getstats;
	dev->set_mac_address = ath_set_mac_address;
	dev->change_mtu = ath_change_mtu;
	dev->tx_queue_len = ATH_TXBUF - 1;		/* 1 for mgmt frame */
#ifdef USE_HEADERLEN_RESV
	dev->hard_header_len += sizeof(struct ieee80211_qosframe) +
				sizeof(struct llc) +
				IEEE80211_ADDR_LEN +
				IEEE80211_WEP_IVLEN +
				IEEE80211_WEP_KIDLEN;
#ifdef ATH_SUPERG_FF
	dev->hard_header_len += ATH_FF_MAX_HDR;
#endif
#endif
	dev->type = ARPHRD_IEEE80211;

	ic->ic_dev = dev;
	ic->ic_mgtstart = ath_mgtstart;
	ic->ic_init = ath_init;
	ic->ic_reset = ath_reset;
	ic->ic_newassoc = ath_newassoc;
	ic->ic_updateslot = ath_updateslot;
	atomic_set(&ic->ic_node_counter, 0);
	ic->ic_debug = 0;

	ic->ic_wme.wme_update = ath_wme_update;
	ic->ic_uapsd_flush = ath_uapsd_flush;

	/* XXX not right but it's not used anywhere important */
	ic->ic_phytype = IEEE80211_T_OFDM;
	ic->ic_opmode = IEEE80211_M_STA;
	sc->sc_opmode = HAL_M_STA;
	/*
	 * Set the Atheros Advanced Capabilities from station config before
	 * starting 802.11 state machine.  Currently, set only fast-frames
	 * capability.
	 */
	ic->ic_ath_cap = 0;
	sc->sc_fftxqmin = ATH_FF_TXQMIN;
#ifdef ATH_SUPERG_FF
	ic->ic_ath_cap |= (ath_hal_fastframesupported(ah) ? 
			IEEE80211_ATHC_FF : 0);
#endif
	ic->ic_ath_cap |= (ath_hal_burstsupported(ah) ? 
			IEEE80211_ATHC_BURST : 0);

#ifdef ATH_SUPERG_COMP
	ic->ic_ath_cap |= (ath_hal_compressionsupported(ah) ? 
			IEEE80211_ATHC_COMP : 0);
#endif

#ifdef ATH_SUPERG_DYNTURBO
	ic->ic_ath_cap |= (ath_hal_turboagsupported(ah, ath_countrycode) ? 
			(IEEE80211_ATHC_TURBOP | IEEE80211_ATHC_AR) : 0);
#endif
#ifdef ATH_SUPERG_XR
	ic->ic_ath_cap |= (ath_hal_xrsupported(ah) ? IEEE80211_ATHC_XR : 0);
#endif

	ic->ic_caps |= IEEE80211_C_IBSS	|	/* ibss, nee adhoc, mode */
		IEEE80211_C_HOSTAP	|	/* hostap mode */
		IEEE80211_C_MONITOR	|	/* monitor mode */
		IEEE80211_C_AHDEMO	|	/* adhoc demo mode */
		IEEE80211_C_SHPREAMBLE	|	/* short preamble supported */
		IEEE80211_C_SHSLOT	|	/* short slot time supported */
		IEEE80211_C_WPA		|	/* capable of WPA1+WPA2 */
		IEEE80211_C_BGSCAN;		/* capable of bg scanning */
	/* Query the HAL to figure out HW crypto. support. */
	if (ath_hal_ciphersupported(ah, HAL_CIPHER_WEP))
		ic->ic_caps |= IEEE80211_C_WEP;
	if (ath_hal_ciphersupported(ah, HAL_CIPHER_AES_OCB))
		ic->ic_caps |= IEEE80211_C_AES;
	if (ath_hal_ciphersupported(ah, HAL_CIPHER_AES_CCM))
		ic->ic_caps |= IEEE80211_C_AES_CCM;
	if (ath_hal_ciphersupported(ah, HAL_CIPHER_CKIP))
		ic->ic_caps |= IEEE80211_C_CKIP;
	if (ath_hal_ciphersupported(ah, HAL_CIPHER_TKIP)) {
		ic->ic_caps |= IEEE80211_C_TKIP;
		/*
		 * Check if h/w does the MIC and/or whether the
		 * separate key cache entries are required to
		 * handle both tx+rx MIC keys.
		 */
		if (ath_hal_ciphersupported(ah, HAL_CIPHER_MIC)) {
			ic->ic_caps |= IEEE80211_C_TKIPMIC;
			/*
			 * Check if h/w does MIC correctly when
			 * WMM is turned on.
			 */
			if (ath_hal_wmetkipmic(ah))
				ic->ic_caps |= IEEE80211_C_WME_TKIPMIC;
		}

		/*
		 * If the h/w supports storing tx+rx MIC keys
		 * in one cache slot automatically enable use.
		 */
		if (ath_hal_hastkipsplit(ah) ||
		    !ath_hal_settkipsplit(ah, AH_FALSE))
			sc->sc_splitmic = 1;
	}
	sc->sc_hasclrkey = ath_hal_ciphersupported(ah, HAL_CIPHER_CLR);
#if 0
	sc->sc_mcastkey = ath_hal_getmcastkeysearch(ah);
#endif
	/*
	 * Mark key cache slots associated with global keys
	 * as in use.  If we knew TKIP was not to be used we
	 * could leave the +32, +64, and +32+64 slots free.
	 */
	for (i = 0; i < IEEE80211_WEP_NKID; i++) {
		setbit(sc->sc_keymap, i);
		setbit(sc->sc_keymap, i+64);
		if (sc->sc_splitmic) {
			setbit(sc->sc_keymap, i+32);
			setbit(sc->sc_keymap, i+32+64);
		}
	}
	/*
	 * TPC support can be done either with a global cap or
	 * per-packet support.  The latter is not available on
	 * all parts.  We're a bit pedantic here as all parts
	 * support a global cap.
	 */
#ifdef ATH_CAP_TPC
	sc->sc_hastpc = ath_hal_hastpc(ah);
	if (tpc && !sc->sc_hastpc) {
		printk(KERN_WARNING "ath_pci: WARNING: per-packet transmit "
				"power control was requested, but is not "
				"supported by the hardware.\n");
		tpc = 0;
	}
	printk(KERN_INFO "ath_pci: switching per-packet transmit power "
			"control %s\n",
		tpc ? "on" : "off");
	ath_hal_settpc(ah, tpc);
#else
	sc->sc_hastpc = 0;
	tpc = 0; /* TPC is always zero, when compiled without ATH_CAP_TPC */
#endif
	if (sc->sc_hastpc || ath_hal_hastxpowlimit(ah))
		ic->ic_caps |= IEEE80211_C_TXPMGT;

	/*
	 * Default 11.h to start enabled.
	 */
	ic->ic_flags |= IEEE80211_F_DOTH;

	/*
	 * Check for misc other capabilities.
	 */
	if (ath_hal_hasbursting(ah))
		ic->ic_caps |= IEEE80211_C_BURST;
	sc->sc_hasbmask = ath_hal_hasbssidmask(ah);
	sc->sc_hastsfadd = ath_hal_hastsfadjust(ah);
	/*
	 * Indicate we need the 802.11 header padded to a
	 * 32-bit boundary for 4-address and QoS frames.
	 */
	ic->ic_flags |= IEEE80211_F_DATAPAD;

	/*
	 * Query the HAL about antenna support
	 * Enable rx fast diversity if HAL has support
	 */
	if (ath_hal_hasdiversity(ah)) {
		sc->sc_hasdiversity = 1;
		ath_hal_setdiversity(ah, AH_TRUE);
		sc->sc_diversity = 1;
	} else {
		sc->sc_hasdiversity = 0;
		sc->sc_diversity = 0;
		ath_hal_setdiversity(ah, AH_FALSE);
	}
	sc->sc_defant = ath_hal_getdefantenna(ah);

	/*
	 * Not all chips have the VEOL support we want to
	 * use with IBSS beacons; check here for it.
	 */
	sc->sc_hasveol = ath_hal_hasveol(ah);

	/* Interference mitigation/ambient noise immunity (ANI).
	 * In modes other than HAL_M_STA, it causes receive sensitivity
	 * problems for OFDM. */
	sc->sc_hasintmit = ath_hal_hasintmit(ah);

	/* get mac address from hardware */
	ath_hal_getmac(ah, ic->ic_myaddr);
	if (sc->sc_hasbmask) {
		ath_hal_getbssidmask(ah, sc->sc_bssidmask);
		ATH_SET_VAP_BSSID_MASK(sc->sc_bssidmask);
		ath_hal_setbssidmask(ah, sc->sc_bssidmask);
	}
	IEEE80211_ADDR_COPY(dev->dev_addr, ic->ic_myaddr);

	/* call MI attach routine. */
	ieee80211_ifattach(ic);
	/* override default methods */
#ifdef IEEE80211_DEBUG_REFCNT
	ic->ic_node_alloc_debug = ath_node_alloc_debug;
	sc->sc_node_free_debug = ic->ic_node_free_debug;
	ic->ic_node_free_debug = ath_node_free_debug;
	sc->sc_node_cleanup_debug = ic->ic_node_cleanup_debug;
	ic->ic_node_cleanup_debug = ath_node_cleanup_debug;
#else /* #ifdef IEEE80211_DEBUG_REFCNT */
	ic->ic_node_alloc = ath_node_alloc;
	sc->sc_node_free = ic->ic_node_free;
	ic->ic_node_free = ath_node_free;
	sc->sc_node_cleanup = ic->ic_node_cleanup;
	ic->ic_node_cleanup = ath_node_cleanup;
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */

	ic->ic_node_getrssi = ath_node_getrssi;
#ifdef ATH_SUPERG_XR
	ic->ic_node_move_data = ath_node_move_data;
#endif
	sc->sc_recv_mgmt = ic->ic_recv_mgmt;
	ic->ic_recv_mgmt = ath_recv_mgmt;

	ic->ic_vap_create = ath_vap_create;
	ic->ic_vap_delete = ath_vap_delete;


	ic->ic_set_txcont           = ath_set_txcont;
	ic->ic_get_txcont           = ath_get_txcont;

	ic->ic_set_txcont_power     = ath_set_txcont_power;
	ic->ic_get_txcont_power     = ath_get_txcont_power;

	ic->ic_set_txcont_rate      = ath_set_txcont_rate;
	ic->ic_get_txcont_rate      = ath_get_txcont_rate;

	ic->ic_scan_start = ath_scan_start;
	ic->ic_scan_end = ath_scan_end;
	ic->ic_set_channel = ath_set_channel;

#ifdef ATH_REVERSE_ENGINEERING
	ic->ic_read_register        = ath_read_register;
	ic->ic_write_register       = ath_write_register;
	ic->ic_registers_dump       = ath_registers_dump;
	ic->ic_registers_dump_delta = ath_registers_dump_delta;
	ic->ic_registers_mark       = ath_registers_mark;
#endif /* #ifdef ATH_REVERSE_ENGINEERING */

	ic->ic_set_coverageclass = ath_set_coverageclass;
	ic->ic_mhz2ieee = ath_mhz2ieee;

	if (register_netdev(dev)) {
		printk(KERN_ERR "%s: unable to register device\n", 
				DEV_NAME(dev));
		goto bad3;
	}
	/*
	 * Attach dynamic MIB vars and announce support
	 * now that we have a device name with unit number.
	 */
	ath_dynamic_sysctl_register(sc);
	ieee80211_announce(ic);
	ath_announce(dev);
#ifdef ATH_TX99_DIAG
	printk("%s: TX99 support enabled\n", DEV_NAME(dev));
#endif
	sc->sc_invalid = 0;

	if (autocreate) {
		if (!strcmp(autocreate, "none"))
			autocreatemode = -1;
		else if (!strcmp(autocreate, "sta"))
			autocreatemode = IEEE80211_M_STA;
		else if (!strcmp(autocreate, "ap"))
			autocreatemode = IEEE80211_M_HOSTAP;
		else if (!strcmp(autocreate, "adhoc"))
			autocreatemode = IEEE80211_M_IBSS;
		else if (!strcmp(autocreate, "ahdemo"))
			autocreatemode = IEEE80211_M_AHDEMO;
		else if (!strcmp(autocreate, "wds"))
			autocreatemode = IEEE80211_M_WDS;
		else if (!strcmp(autocreate, "monitor"))
			autocreatemode = IEEE80211_M_MONITOR;
		else {
			printk(KERN_INFO "Unknown autocreate mode: %s\n",
				autocreate);
			autocreatemode = -1;
		}
	}

	if (autocreatemode != -1) {
		rtnl_lock();
		vap = ieee80211_create_vap(ic, "ath%d", dev,
				autocreatemode, 0);
		rtnl_unlock();
		if (vap == NULL)
			printk(KERN_ERR "%s: autocreation of VAP failed.",
				DEV_NAME(dev));
	}

	return 0;
bad3:
	ieee80211_ifdetach(ic);
	ieee80211_rate_detach(sc->sc_rc);
bad2:
	ath_tx_cleanup(sc);
	ath_desc_free(sc);
bad:
	if (ah)
		_ath_hal_detach(ah);
	ATH_TXBUF_LOCK_DESTROY(sc);
	ATH_LOCK_DESTROY(sc);
	ATH_HAL_LOCK_DESTROY(sc);
	sc->sc_invalid = 1;

	return error;
}

int
ath_detach(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;

	HAL_INT tmp;
	DPRINTF(sc, ATH_DEBUG_ANY, "%s: flags %x\n", __func__, dev->flags);
	ath_stop(dev);

	ath_hal_setpower(sc->sc_ah, HAL_PM_AWAKE, AH_TRUE);

	sc->sc_invalid = 1;

	/*
	 * NB: the order of these is important:
	 * o call the 802.11 layer before detaching the HAL to
	 *   ensure callbacks into the driver to delete global
	 *   key cache entries can be handled
	 * o reclaim the tx queue data structures after calling
	 *   the 802.11 layer as we'll get called back to reclaim
	 *   node state and potentially want to use them
	 * o to cleanup the tx queues the HAL is called, so detach
	 *   it last
	 * Other than that, it's straightforward...
	 */
	ieee80211_ifdetach(&sc->sc_ic);

	ath_hal_intrset(ah, 0);		/* disable further intr's */
	ath_hal_getisr(ah, &tmp);	/* clear ISR */
	if (dev->irq) {
		free_irq(dev->irq, dev);
		dev->irq = 0;
	}
#ifdef ATH_TX99_DIAG
	if (sc->sc_tx99 != NULL)
		sc->sc_tx99->detach(sc->sc_tx99);
#endif
	ieee80211_rate_detach(sc->sc_rc);
	ath_desc_free(sc);
	ath_tx_cleanup(sc);
	_ath_hal_detach(ah);

	ath_dynamic_sysctl_unregister(sc);
	ATH_LOCK_DESTROY(sc);
	ATH_HAL_LOCK_DESTROY(sc);
	dev->stop = NULL; /* prevent calling ath_stop again */
	unregister_netdev(dev);
	return 0;
}

static struct ieee80211vap *
ath_vap_create(struct ieee80211com *ic, const char *name,
	int opmode, int flags, struct net_device *mdev)
{
	struct ath_softc *sc = ic->ic_dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct net_device *dev;
	struct ath_vap *avp;
	struct ieee80211vap *vap;
	int ic_opmode;

	if (ic->ic_dev->flags & IFF_RUNNING) {
		/* needs to disable hardware too */
		ath_hal_intrset(ah, 0);		/* disable interrupts */
		ath_draintxq(sc);		/* stop xmit side */
		ath_stoprecv(sc);		/* stop recv side */
	}
	/* XXX ic unlocked and race against add */
	switch (opmode) {
	case IEEE80211_M_STA:	/* ap+sta for repeater application */
		if (sc->sc_nstavaps != 0)  /* only one sta regardless */
			return NULL;
		if ((sc->sc_nvaps != 0) && (!(flags & IEEE80211_NO_STABEACONS)))
			return NULL;   /* If using station beacons, must first up */
		if (flags & IEEE80211_NO_STABEACONS) {
			sc->sc_nostabeacons = 1;
			ic_opmode = IEEE80211_M_HOSTAP;	/* Run with chip in AP mode */
		} else
			ic_opmode = opmode;
		break;
	case IEEE80211_M_IBSS:
		if ((sc->sc_nvaps != 0) && (ic->ic_opmode == IEEE80211_M_STA))
			return NULL;
		ic_opmode = opmode;
		break;
	case IEEE80211_M_AHDEMO:
	case IEEE80211_M_MONITOR:
		if (sc->sc_nvaps != 0 && (ic->ic_opmode != opmode)) {
			/* preserve existing mode */
			ic_opmode = ic->ic_opmode;
		} else
			ic_opmode = opmode;
		break;
	case IEEE80211_M_HOSTAP:
	case IEEE80211_M_WDS:
		/* permit multiple APs and/or WDS links */
		/* XXX sta+ap for repeater/bridge application */
		if ((sc->sc_nvaps != 0) && (ic->ic_opmode == IEEE80211_M_STA))
			return NULL;
		/* XXX not right, beacon buffer is allocated on RUN trans */
		if (opmode == IEEE80211_M_HOSTAP && STAILQ_EMPTY(&sc->sc_bbuf))
			return NULL;
		/*
		 * XXX Not sure if this is correct when operating only
		 * with WDS links.
		 */
		ic_opmode = IEEE80211_M_HOSTAP;

		break;
	default:
		return NULL;
	}

	if (sc->sc_nvaps >= ATH_BCBUF) {
		printk(KERN_WARNING "too many virtual APs (already got %d)\n", 
				sc->sc_nvaps);
		return NULL;
	}

	dev = alloc_etherdev(sizeof(struct ath_vap) + sc->sc_rc->arc_vap_space);
	if (dev == NULL) {
		/* XXX msg */
		return NULL;
	}

	avp = dev->priv;
	ieee80211_vap_setup(ic, dev, name, opmode, flags);
	/* override with driver methods */
	vap = &avp->av_vap;
	avp->av_newstate = vap->iv_newstate;
	vap->iv_newstate = ath_newstate;
	vap->iv_key_alloc = ath_key_alloc;
	vap->iv_key_delete = ath_key_delete;
	vap->iv_key_set = ath_key_set;
	vap->iv_key_update_begin = ath_key_update_begin;
	vap->iv_key_update_end = ath_key_update_end;
	if (sc->sc_default_ieee80211_debug) {
		/* User specified defaults for new VAPs were provided, so
		 * use those (only). */
		vap->iv_debug = (sc->sc_default_ieee80211_debug & ~IEEE80211_MSG_IC);
	}
	else {
		/* If no default VAP debug flags are passed, allow a few to
		 * transfer down from the driver to new VAPs so we can have load
		 * time debugging for VAPs too. */
		vap->iv_debug = 0 |
			((sc->sc_debug & ATH_DEBUG_RATE) ? IEEE80211_MSG_XRATE  : 0) | 
			((sc->sc_debug & ATH_DEBUG_XMIT) ? IEEE80211_MSG_OUTPUT : 0) | 
			((sc->sc_debug & ATH_DEBUG_RECV) ? IEEE80211_MSG_INPUT  : 0) |
			0
			;
	}
	ic->ic_debug = (sc->sc_default_ieee80211_debug & IEEE80211_MSG_IC);

#ifdef ATH_SUPERG_COMP
	vap->iv_comp_set = ath_comp_set;
#endif

	/* Let rate control register proc entries for the VAP */
	if (sc->sc_rc->ops->dynamic_proc_register)
		sc->sc_rc->ops->dynamic_proc_register(vap);

	/* XXX: VAPs emulate ethernet - true/false/good/bad? */
	dev->type = ARPHRD_ETHER;
	if (opmode == IEEE80211_M_MONITOR)
		/* Use RadioTAP interface type for monitor mode. */
		dev->type = ARPHRD_IEEE80211_RADIOTAP;

	if ((flags & IEEE80211_CLONE_BSSID) && sc->sc_hasbmask) {
		struct ieee80211vap *v;
		unsigned int id_mask, id;

		/*
		 * Hardware supports the bssid mask and a unique
		 * bssid was requested.  Assign a new mac address
		 * and expand our bssid mask to cover the active
		 * virtual APs with distinct addresses.
		 */

		/* do a full search to mark all the allocated VAPs */
		id_mask = 0;
		TAILQ_FOREACH(v, &ic->ic_vaps, iv_next)
			id_mask |= (1 << ATH_GET_VAP_ID(v->iv_myaddr));

		for (id = 1; id < ATH_BCBUF; id++) {
			/* get the first available slot */
			if ((id_mask & (1 << id)) == 0) {
				ATH_SET_VAP_BSSID(vap->iv_myaddr, id);
				break;
			}
		}
	}
	avp->av_bslot = -1;
	STAILQ_INIT(&avp->av_mcastq.axq_q);
	ATH_TXQ_LOCK_INIT(&avp->av_mcastq);
	if (IEEE80211_IS_MODE_BEACON(opmode)) {
		unsigned int slot;
		/*
		 * Allocate beacon state for hostap/ibss.  We know
		 * a buffer is available because of the check above.
		 */
		avp->av_bcbuf = STAILQ_FIRST(&sc->sc_bbuf);
		STAILQ_REMOVE_HEAD(&sc->sc_bbuf, bf_list);
		if (opmode == IEEE80211_M_HOSTAP || !sc->sc_hasveol) {
			/*
			 * Assign the VAP to a beacon xmit slot.  As
			 * above, this cannot fail to find one.
			 */
			avp->av_bslot = 0;
			for (slot = 0; slot < ATH_BCBUF; slot++)
				if (sc->sc_bslot[slot] == NULL) {
					/*
					 * XXX hack, space out slots to better
					 * deal with misses
					 */
					if (slot + 1 < ATH_BCBUF &&
					    sc->sc_bslot[slot+1] == NULL) {
						avp->av_bslot = slot + 1;
						break;
					}
					avp->av_bslot = slot;
					/* NB: keep looking for a double slot */
				}
			KASSERT(sc->sc_bslot[avp->av_bslot] == NULL,
				("beacon slot %u not empty?", avp->av_bslot));
			sc->sc_bslot[avp->av_bslot] = vap;
			sc->sc_nbcnvaps++;
		}
		if ((opmode == IEEE80211_M_HOSTAP) && (sc->sc_hastsfadd)) {
			/*
			 * Multiple VAPs are to transmit beacons and we
			 * have h/w support for TSF adjusting; enable use
			 * of staggered beacons.
			 */
			/* XXX check for beacon interval too small */
			sc->sc_stagbeacons = 1;
		}
	}
	if (sc->sc_hastsfadd)
		ath_hal_settsfadjust(sc->sc_ah, sc->sc_stagbeacons);
	SET_NETDEV_DEV(dev, ATH_GET_NETDEV_DEV(mdev));
	/* complete setup */
	(void) ieee80211_vap_attach(vap,
		ieee80211_media_change, ieee80211_media_status);

	ic->ic_opmode = ic_opmode;

	if (opmode != IEEE80211_M_WDS)
		sc->sc_nvaps++;

	if (opmode == IEEE80211_M_STA)
		sc->sc_nstavaps++;
	else if (opmode == IEEE80211_M_MONITOR)
		sc->sc_nmonvaps++;
	/*
	 * Adhoc demo mode is a pseudo mode; to the HAL it's
	 * just IBSS mode and the driver doesn't use management
	 * frames.  Other modes carry over directly to the HAL.
	 */
	if (ic->ic_opmode == IEEE80211_M_AHDEMO)
		sc->sc_opmode = HAL_M_IBSS;
	else
		sc->sc_opmode = (HAL_OPMODE) ic->ic_opmode;	/* NB: compatible */

#ifdef ATH_SUPERG_XR
	if (vap->iv_flags & IEEE80211_F_XR) {
		if (ath_descdma_setup(sc, &sc->sc_grppolldma, &sc->sc_grppollbuf,
				"grppoll", (sc->sc_xrpollcount + 1) * 
				HAL_ANTENNA_MAX_MODE, 1) != 0)
			printk("%s:grppoll Buf allocation failed \n", __func__);
		if (!sc->sc_xrtxq)
			sc->sc_xrtxq = ath_txq_setup(sc, HAL_TX_QUEUE_DATA, 
					HAL_XR_DATA);
		if (sc->sc_hasdiversity) {
			/* Save current diversity state if user destroys XR VAP */
			sc->sc_olddiversity = sc->sc_diversity;
			ath_hal_setdiversity(sc->sc_ah, 0);
			sc->sc_diversity = 0;
		}
	}
#endif
	if (ic->ic_dev->flags & IFF_RUNNING) {
		/* restart hardware */
		if (ath_startrecv(sc) != 0)	/* restart recv */
			printk("%s: %s: unable to start recv logic\n",
				DEV_NAME(dev), __func__);
		if (sc->sc_beacons)
			ath_beacon_config(sc, NULL);	/* restart beacons */
		ath_hal_intrset(ah, sc->sc_imask);
	}

	return vap;
}

static void
ath_vap_delete(struct ieee80211vap *vap)
{
	struct net_device *dev = vap->iv_ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ath_vap *avp = ATH_VAP(vap);
	int decrease = 1;
	unsigned int i;
	KASSERT(vap->iv_state == IEEE80211_S_INIT, ("VAP not stopped"));

	if (dev->flags & IFF_RUNNING) {
		/*
		 * Quiesce the hardware while we remove the VAP.  In
		 * particular we need to reclaim all references to the
		 * VAP state by any frames pending on the tx queues.
		 *
		 * XXX: Can we do this w/o affecting other VAPs?
		 */
		ath_hal_intrset(ah, 0);		/* disable interrupts */
		ath_draintxq(sc);		/* stop xmit side */
		ath_stoprecv(sc);		/* stop recv side */
	}

	/*
	 * Reclaim any pending mcast bufs on the VAP.
	 */
	ath_tx_draintxq(sc, &avp->av_mcastq);
	ATH_TXQ_LOCK_DESTROY(&avp->av_mcastq);

	/*
	 * Reclaim beacon state.  Note this must be done before
	 * VAP instance is reclaimed as we may have a reference
	 * to it in the buffer for the beacon frame.
	 */
	if (avp->av_bcbuf != NULL) {
		if (avp->av_bslot != -1) {
			sc->sc_bslot[avp->av_bslot] = NULL;
			sc->sc_nbcnvaps--;
		}
		ath_beacon_return(sc, avp->av_bcbuf);
		avp->av_bcbuf = NULL;
		if (sc->sc_nbcnvaps == 0)
			sc->sc_stagbeacons = 0;
	}

	if (vap->iv_opmode == IEEE80211_M_STA) {
		sc->sc_nstavaps--;
		sc->sc_nostabeacons = 0;
	} else if (vap->iv_opmode == IEEE80211_M_MONITOR)
		sc->sc_nmonvaps--;
	else if (vap->iv_opmode == IEEE80211_M_WDS)
		decrease = 0;

	ieee80211_vap_detach(vap);
	/* NB: memory is reclaimed through dev->destructor callback */
	if (decrease)
		sc->sc_nvaps--;

#ifdef ATH_SUPERG_XR
	/*
	 * If it's an XR VAP, free the memory allocated explicitly.
	 * Since the XR VAP is not registered, OS cannot free the memory.
	 */
	if (vap->iv_flags & IEEE80211_F_XR) {
		ath_grppoll_stop(vap);
		ath_descdma_cleanup(sc, &sc->sc_grppolldma, &sc->sc_grppollbuf, 
				BUS_DMA_FROMDEVICE);
		memset(&sc->sc_grppollbuf, 0, sizeof(sc->sc_grppollbuf));
		memset(&sc->sc_grppolldma, 0, sizeof(sc->sc_grppolldma));
		if (vap->iv_xrvap)
			vap->iv_xrvap->iv_xrvap = NULL;
		kfree(vap->iv_dev);
		ath_tx_cleanupq(sc, sc->sc_xrtxq);
		sc->sc_xrtxq = NULL;
		if (sc->sc_hasdiversity) {
			/* Restore diversity setting to old diversity setting */
			ath_hal_setdiversity(ah, sc->sc_olddiversity);
			sc->sc_diversity = sc->sc_olddiversity;
		}
	}
#endif

	for (i = 0; i < IEEE80211_APPIE_NUM_OF_FRAME; i++) {
		if (vap->app_ie[i].ie != NULL) {
			FREE(vap->app_ie[i].ie, M_DEVBUF);
			vap->app_ie[i].ie = NULL;
			vap->app_ie[i].length = 0;
		}
	}

	if (dev->flags & IFF_RUNNING) {
		/* Restart RX & TX machines if device is still running. */
		if (ath_startrecv(sc) != 0)		/* restart recv. */
			printk("%s: %s: unable to start recv logic\n",
				DEV_NAME(dev), __func__);
		if (sc->sc_beacons)
			ath_beacon_config(sc, NULL);	/* restart beacons */
		ath_hal_intrset(ah, sc->sc_imask);
	}
}

void
ath_suspend(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;

	DPRINTF(sc, ATH_DEBUG_ANY, "%s: flags %x\n", __func__, dev->flags);
	ath_stop(dev);
}

void
ath_resume(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;

	DPRINTF(sc, ATH_DEBUG_ANY, "%s: flags %x\n", __func__, dev->flags);
	ath_init(dev);
}

/* Extend 15-bit timestamp from RX descriptor to a full 64-bit TSF using the
 * provided hardware TSF. The result is the closest value relative to hardware
 * TSF. */

/* NB: Not all chipsets return the same precision rstamp */
static __inline u_int64_t
ath_extend_tsf(u_int64_t tsf, u_int32_t rstamp)
{
#define TSTAMP_MASK  0x7fff

	u_int64_t result;

	result = (tsf & ~TSTAMP_MASK) | rstamp;
	if (result > tsf) {
		if ((result - tsf) > (TSTAMP_MASK / 2))
			result -= (TSTAMP_MASK + 1);
	} else {
		if ((tsf - result) > (TSTAMP_MASK / 2))
			result += (TSTAMP_MASK + 1);
	}

	return result;
}

static void
ath_uapsd_processtriggers(struct ath_softc *sc)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ath_buf *bf;
	struct ath_desc *ds;
	struct ath_rx_status *rs;
	struct sk_buff *skb;
	struct ieee80211_node *ni;
	struct ath_node *an;
	struct ieee80211_qosframe *qwh;
	struct ath_txq *uapsd_xmit_q = sc->sc_uapsdq;
	struct ieee80211com *ic = &sc->sc_ic;
	int ac, retval;
	u_int32_t last_rs_tstamp = 0;
	int check_for_radar = 0;
	struct ath_buf *prev_rxbufcur;
	u_int8_t tid;
	u_int16_t frame_seq;
	u_int64_t hw_tsf;
	int count = 0;
	int rollover = 0;

#define	PA2DESC(_sc, _pa) \
	((struct ath_desc *)((caddr_t)(_sc)->sc_rxdma.dd_desc + \
		((_pa) - (_sc)->sc_rxdma.dd_desc_paddr)))

	/* XXXAPSD: build in check against max triggers we could see
	 *          based on ic->ic_uapsdmaxtriggers. */

	/* Do not move hw_tsf processing and noise processing out to the rx
	 * tasklet.  The ONLY place we can properly correct for TSF errors and
	 * get accurate noise floor information is in the interrupt handler.
	 * The HW returns a 15-bit TS on rx.  We get interrupts after multiple
	 * packets are queued up.  Sometimes (read often), the 15-bit counter
	 * in the hardware has rolled over one or more times.  We correct for
	 * this in the interrupt function and store the adjusted TSF in the
	 * buffer.
	 *
	 * We also store noise during interrupt, since HW does not log
	 * this per packet and the rx queue is too late. Multiple interrupts
	 * will have occurred, and the noise value at that point is totally
	 * unrelated to conditions during receiption.  This is as close as we
	 * get to reality.  This value is used in monitor mode and by tools like
	 * Wireshark and Kismet.
	 */
	hw_tsf = ath_hal_gettsf64(ah);
	ic->ic_channoise = ath_hal_get_channel_noise(ah, &(sc->sc_curchan));

	ATH_RXBUF_LOCK_IRQ(sc);
	if (sc->sc_rxbufcur == NULL)
		sc->sc_rxbufcur = STAILQ_FIRST(&sc->sc_rxbuf);

	prev_rxbufcur = sc->sc_rxbufcur;
	for (bf = prev_rxbufcur; bf; bf = STAILQ_NEXT(bf, bf_list)) {
		ds = bf->bf_desc;
		if (ds->ds_link == bf->bf_daddr) {
			/* NB: never process the self-linked entry at
			 * the end */
			break;
		}
		if (bf->bf_status & ATH_BUFSTATUS_DONE) {
			/* already processed this buffer (shouldn't
			 * occur if we change code to always process
			 * descriptors in rx intr handler - as opposed
			 * to sometimes processing in the rx tasklet) */
			continue;
		}
		skb = bf->bf_skb;
		if (skb == NULL) {
			printk("%s: no skbuff\n", __func__);
			continue;
		}

		/* XXXAPSD: consider new HAL call that does only the
		 *          subset of ath_hal_rxprocdesc we require
		 *          for trigger search. */

		/* NB: descriptor memory doesn't need to be sync'd
		 *     due to the way it was allocated. */

		/* Must provide the virtual address of the current
		 * descriptor, the physical address, and the virtual
		 * address of the next descriptor in the h/w chain.
		 * This allows the HAL to look ahead to see if the
		 * hardware is done with a descriptor by checking the
		 * done bit in the following descriptor and the address
		 * of the current descriptor the DMA engine is working
		 * on.  All this is necessary because of our use of
		 * a self-linked list to avoid rx overruns. */
		rs = &bf->bf_dsstatus.ds_rxstat;
		retval = ath_hal_rxprocdesc(ah, ds, bf->bf_daddr,
					    PA2DESC(sc, ds->ds_link),
					    hw_tsf, rs);
		if (HAL_EINPROGRESS == retval)
			break;

		/* update the per packet TSF with rs_tstamp */
		bf->bf_tsf     = rs->rs_tstamp;
		bf->bf_status |= ATH_BUFSTATUS_RXTSTAMP;
		count++;

		DPRINTF(sc, ATH_DEBUG_TSF,
			"%s: rs_tstamp=%10llx count=%d\n",
			DEV_NAME(sc->sc_dev),
			bf->bf_tsf, count);

		/* compute rollover */
		if (last_rs_tstamp > rs->rs_tstamp) {
			rollover++;
			DPRINTF(sc, ATH_DEBUG_TSF,
				"%s: %d rollover detected\n",
				DEV_NAME(sc->sc_dev),
				rollover);
		}

		last_rs_tstamp = rs->rs_tstamp;

		/* XXX: We do not support frames spanning multiple
		 *      descriptors */
		bf->bf_status |= ATH_BUFSTATUS_DONE;

		/* Capture noise per-interrupt, since it may change
		 * by the time the receive queue gets around to
		 * processing these buffers, and multiple interrupts
		 * may have occurred in the intervening timeframe. */
		bf->bf_channoise = ic->ic_channoise;

		if (rs->rs_status) {
			if ((HAL_RXERR_PHY == rs->rs_status) &&
			    (HAL_PHYERR_RADAR ==
			     (rs->rs_phyerr & 0x1f)) &&
			    (0 == (bf->bf_status &
				   ATH_BUFSTATUS_RADAR_DONE))) {
				check_for_radar = 1;
			}
			/* Skip past the error now */
			continue;
		}

		/* Prepare wireless header for examination */
		bus_dma_sync_single(sc->sc_bdev, bf->bf_skbaddr,
				sizeof(struct ieee80211_qosframe),
				BUS_DMA_FROMDEVICE);
		qwh = (struct ieee80211_qosframe *)skb->data;

		/* Find the node; it MUST be in the keycache. */
		if (rs->rs_keyix == HAL_RXKEYIX_INVALID ||
		    (ni = sc->sc_keyixmap[rs->rs_keyix]) == NULL) {
			/*
			 * XXX: this can occur if WEP mode is used for
			 *      non-Atheros clients (since we do not
			 *      know which of the 4 WEP keys will be
			 *      used at association time, so cannot
			 *      setup a key-cache entry.
			 *      The Atheros client can convey this in
			 *      the Atheros IE.)
			 *
			 *      The fix is to use the hash lookup on
			 *      the node here.
			 */
#if 0
			/* This print is very chatty, so removing for now. */
			DPRINTF(sc, ATH_DEBUG_UAPSD, "%s: U-APSD node (%s) "
				"has invalid keycache entry\n",
				__func__, ether_sprintf(qwh->i_addr2));
#endif
			continue;
		}

		if (!(ni->ni_flags & IEEE80211_NODE_UAPSD))
			continue;

		/*
		 * Must deal with change of state here, since otherwise
		 * there would be a race (on two quick frames from STA)
		 * between this code and the tasklet where we would:
		 *   - miss a trigger on entry to PS if we're already
		 *     trigger hunting
		 *   - generate spurious SP on exit (due to frame
		 *     following exit frame)
		 */
		if (((qwh->i_fc[1] & IEEE80211_FC1_PWR_MGT) ^
		     (ni->ni_flags & IEEE80211_NODE_PWR_MGT))) {
			/*
			 * NB: do not require lock here since this runs
			 * at intr "proper" time and cannot be
			 * interrupted by RX tasklet (code there has
			 * lock). May want to place a macro here
			 * (that does nothing) to make this more clear.
			 */
			ni->ni_flags |= IEEE80211_NODE_PS_CHANGED;
			ni->ni_pschangeseq = *(__le16 *)(&qwh->i_seq[0]);
			ni->ni_flags &= ~IEEE80211_NODE_UAPSD_SP;
			ni->ni_flags ^= IEEE80211_NODE_PWR_MGT;
			if (qwh->i_fc[1] & IEEE80211_FC1_PWR_MGT) {
				ni->ni_flags |=
					IEEE80211_NODE_UAPSD_TRIG;
				ic->ic_uapsdmaxtriggers++;
				WME_UAPSD_NODE_TRIGSEQINIT(ni);
				DPRINTF(sc, ATH_DEBUG_UAPSD,
					"%s: Node (%s) became U-APSD "
					"triggerable (%d)\n",
					__func__,
					ether_sprintf(qwh->i_addr2),
					ic->ic_uapsdmaxtriggers);
			} else {
				ni->ni_flags &=
					~IEEE80211_NODE_UAPSD_TRIG;
				ic->ic_uapsdmaxtriggers--;
				DPRINTF(sc, ATH_DEBUG_UAPSD,
					"%s: Node (%s) no longer U-APSD"
					" triggerable (%d)\n",
					__func__,
					ether_sprintf(qwh->i_addr2),
					ic->ic_uapsdmaxtriggers);
				/*
				 * XXX: Rapidly thrashing sta could get
				 * out-of-order frames due this flush
				 * placing frames on backlogged regular
				 * AC queue and re-entry to PS having
				 * fresh arrivals onto faster UPSD
				 * delivery queue. if this is a big
				 * problem we may need to drop these.
				 */
				ath_uapsd_flush(ni);
			}

			continue;
		}

		if (ic->ic_uapsdmaxtriggers == 0)
			continue;

		/* make sure the frame is QoS data/null */
		/* NB: with current sub-type definitions, the
		 * IEEE80211_FC0_SUBTYPE_QOS check, below,
		 * covers the QoS null case too.
		 */
		if (((qwh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) !=
		     IEEE80211_FC0_TYPE_DATA) ||
		     !(qwh->i_fc[0] & IEEE80211_FC0_SUBTYPE_QOS))
			continue;

		/*
		 * To be a trigger:
		 *   - node is in triggerable state
		 *   - QoS data/null frame with triggerable AC
		 */
		tid = qwh->i_qos[0] & IEEE80211_QOS_TID;
		ac = TID_TO_WME_AC(tid);
		if (!WME_UAPSD_AC_CAN_TRIGGER(ac, ni))
			continue;

		DPRINTF(sc, ATH_DEBUG_UAPSD,
			"%s: U-APSD trigger detected for node "
			"(%s) on AC %d\n",
			__func__,
			ether_sprintf(ni->ni_macaddr), ac);
		if (ni->ni_flags & IEEE80211_NODE_UAPSD_SP) {
			/* have trigger, but SP in progress,
			 * so ignore */
			DPRINTF(sc, ATH_DEBUG_UAPSD,
				"%s:   SP already in progress -"
				" ignoring\n",
				__func__);
			continue;
		}

		/*
		 * Detect duplicate triggers and drop if so.
		 */
		frame_seq = le16toh(*(__le16 *)qwh->i_seq);
		if ((qwh->i_fc[1] & IEEE80211_FC1_RETRY) &&
		    frame_seq == ni->ni_uapsd_trigseq[ac]) {
			DPRINTF(sc, ATH_DEBUG_UAPSD,
				"%s: dropped dup trigger, ac %d"
				", seq %d\n",
				__func__, ac, frame_seq);
			continue;
		}

		an = ATH_NODE(ni);

		/* start the SP */
		ATH_NODE_UAPSD_LOCK_IRQ(an);
		ni->ni_stats.ns_uapsd_triggers++;
		ni->ni_flags |= IEEE80211_NODE_UAPSD_SP;
		ni->ni_uapsd_trigseq[ac] = frame_seq;
		ATH_NODE_UAPSD_UNLOCK_IRQ(an);

		ATH_TXQ_LOCK_IRQ(uapsd_xmit_q);
		if (STAILQ_EMPTY(&an->an_uapsd_q)) {
			DPRINTF(sc, ATH_DEBUG_UAPSD,
				"%s: Queue empty, generating "
				"QoS NULL to send\n",
				__func__);
			/*
			 * Empty queue, so need to send QoS null
			 * on this ac. Make a call that will
			 * dump a QoS null onto the node's
			 * queue, then we can proceed as normal.
			 */
			ieee80211_send_qosnulldata(ni, ac);
		}

		if (STAILQ_FIRST(&an->an_uapsd_q)) {
			struct ath_buf *last_buf =
				STAILQ_LAST(&an->an_uapsd_q,
					    ath_buf, bf_list);
			struct ath_desc *last_desc =
				last_buf->bf_desc;
			struct ieee80211_qosframe *qwhl =
				(struct ieee80211_qosframe *)
				last_buf->bf_skb->data;
			/*
			 * NB: flip the bit to cause intr on the
			 * EOSP desc, which is the last one
			 */
			ath_hal_txreqintrdesc(sc->sc_ah,
					      last_desc);

			qwhl->i_qos[0] |= IEEE80211_QOS_EOSP;

			if (IEEE80211_VAP_EOSPDROP_ENABLED(ni->ni_vap)) {
				/* simulate lost EOSP */
				qwhl->i_addr1[0] |= 0x40;
			}

			/* more data bit only for EOSP frame */
			if (an->an_uapsd_overflowqdepth)
				qwhl->i_fc[1] |=
					IEEE80211_FC1_MORE_DATA;
			else if (IEEE80211_NODE_UAPSD_USETIM(ni))
				ni->ni_vap->iv_set_tim(ni, 0);

			ni->ni_stats.ns_tx_uapsd +=
				an->an_uapsd_qdepth;

			bus_dma_sync_single(sc->sc_bdev,
					    last_buf->bf_skbaddr,
					    sizeof(*qwhl),
					    BUS_DMA_TODEVICE);

			if (uapsd_xmit_q->axq_link) {
#ifdef AH_NEED_DESC_SWAP
				*uapsd_xmit_q->axq_link =
					cpu_to_le32(STAILQ_FIRST(&an->an_uapsd_q)->bf_daddr);
#else
				*uapsd_xmit_q->axq_link =
					STAILQ_FIRST(&an->an_uapsd_q)->bf_daddr;
#endif
			}
			/* below leaves an_uapsd_q NULL */
			STAILQ_CONCAT(&uapsd_xmit_q->axq_q,
				      &an->an_uapsd_q);
			uapsd_xmit_q->axq_link =
				&last_desc->ds_link;
			ath_hal_puttxbuf(sc->sc_ah,
				uapsd_xmit_q->axq_qnum,
				(STAILQ_FIRST(&uapsd_xmit_q->axq_q))->bf_daddr);

			ath_hal_txstart(sc->sc_ah,
					uapsd_xmit_q->axq_qnum);
		}
		an->an_uapsd_qdepth = 0;
		ATH_TXQ_UNLOCK_IRQ(uapsd_xmit_q);
	}
	sc->sc_rxbufcur = bf;

	/* SECOND PASS - FIX RX TIMESTAMPS */
	if (count > 0) {
		hw_tsf = ath_hal_gettsf64(ah);
		if (last_rs_tstamp > (hw_tsf & TSTAMP_MASK)) {
			rollover++;
			DPRINTF(sc, ATH_DEBUG_TSF,
				"%s: %d rollover detected for hw_tsf=%10llx\n",
				DEV_NAME(sc->sc_dev),
				rollover, hw_tsf);
		}

		last_rs_tstamp = 0;
		for (bf = prev_rxbufcur; bf; bf = STAILQ_NEXT(bf, bf_list)) {
			ds = bf->bf_desc;
			if (ds->ds_link == bf->bf_daddr) {
				/* NB: never process the self-linked entry at
				 * the end */
				break;
			}

			/* we only process buffers who needs RX timestamps
			 * adjustements */
			if  (bf->bf_status & ATH_BUFSTATUS_RXTSTAMP) {
				bf->bf_status &= ~ATH_BUFSTATUS_RXTSTAMP;

				/* update rollover */
				if (last_rs_tstamp > bf->bf_tsf)
					rollover--;

				/* update last_rs_tstamp */
				last_rs_tstamp = bf->bf_tsf;
				bf->bf_tsf = 
					(hw_tsf & ~TSTAMP_MASK) | bf->bf_tsf;
				bf->bf_tsf -= rollover * (TSTAMP_MASK + 1);

				DPRINTF(sc, ATH_DEBUG_TSF,
					"%s: bf_tsf=%10llx hw_tsf=%10llx\n",
					DEV_NAME(sc->sc_dev),
					bf->bf_tsf, hw_tsf);
			}
		}
	}

	ATH_RXBUF_UNLOCK_IRQ(sc);
#undef PA2DESC
}

/*
 * Interrupt handler.  Most of the actual processing is deferred.
 */
irqreturn_t
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
ath_intr(int irq, void *dev_id)
#else
ath_intr(int irq, void *dev_id, struct pt_regs *regs)
#endif
{
	struct net_device *dev = dev_id;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	HAL_INT status;
	int needmark;

	DPRINTF(sc, ATH_DEBUG_INTR, "%s:%d [flags 0x%x%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s] - "
				    "entered\n",
		__func__, __LINE__, 
		dev->flags,
		(dev->flags & IFF_UP) 		? " IFF_UP" 		: "",
		(dev->flags & IFF_BROADCAST)	? " IFF_BROADCAST"	: "",
		(dev->flags & IFF_DEBUG)	? " IFF_DEBUG"		: "",
		(dev->flags & IFF_LOOPBACK)	? " IFF_LOOPBACK"	: "",
		(dev->flags & IFF_POINTOPOINT)	? " IFF_POINTOPOINT"	: "",
		(dev->flags & IFF_NOTRAILERS)	? " IFF_NOTRAILERS"	: "",
		(dev->flags & IFF_RUNNING)	? " IFF_RUNNING"	: "",
		(dev->flags & IFF_NOARP)	? " IFF_NOARP"		: "",
		(dev->flags & IFF_PROMISC)	? " IFF_PROMISC"	: "",
		(dev->flags & IFF_ALLMULTI)	? " IFF_ALLMULTI"	: "",
		(dev->flags & IFF_MASTER)	? " IFF_MASTER"		: "",
		(dev->flags & IFF_SLAVE)	? " IFF_SLAVE"		: "",
		(dev->flags & IFF_MULTICAST)	? " IFF_MULTICAST"	: "",
		(dev->flags & IFF_PORTSEL)	? " IFF_PORTSEL"	: "",
		(dev->flags & IFF_AUTOMEDIA)	? " IFF_AUTOMEDIA"	: "",
		(dev->flags & IFF_DYNAMIC)	? " IFF_DYNAMIC"	: "");

	if (sc->sc_invalid) {
		/*
		 * The hardware is not ready/present, don't touch anything.
		 * Note this can happen early on if the IRQ is shared.
		 */
		return IRQ_NONE;
	}
	if (!ath_hal_intrpend(ah))		/* shared irq, not for us */
		return IRQ_NONE;
	if ((dev->flags & (IFF_RUNNING | IFF_UP)) != (IFF_RUNNING | IFF_UP)) {
		DPRINTF(sc, ATH_DEBUG_INTR, "%s: flags 0x%x\n",
			__func__, dev->flags);
		ath_hal_getisr(ah, &status);	/* clear ISR */
		ath_hal_intrset(ah, 0);		/* disable further intr's */
		return IRQ_HANDLED;
	}
	needmark = 0;
	/*
	 * Figure out the reason(s) for the interrupt.  Note
	 * that the HAL returns a pseudo-ISR that may include
	 * bits we haven't explicitly enabled so we mask the
	 * value to ensure we only process bits we requested.
	 */
	ath_hal_getisr(ah, &status);		/* NB: clears ISR too */
	DPRINTF(sc, ATH_DEBUG_INTR,
		"%s: status 0x%x%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s\n", __func__, status,
		(status & HAL_INT_RX)      	? " HAL_INT_RX"		: "",
		(status & HAL_INT_RXNOFRM) 	? " HAL_INT_RXNOFRM"	: "",
		(status & HAL_INT_TX)      	? " HAL_INT_TX"		: "",
		(status & HAL_INT_MIB)     	? " HAL_INT_MIB"	: "",
		(status & HAL_INT_RXPHY)   	? " HAL_INT_RXPHY"	: "",
		(status & HAL_INT_SWBA)    	? " HAL_INT_SWBA"	: "",
		(status & HAL_INT_RXDESC)	? " HAL_INT_RXDESC" 	: "",
		(status & HAL_INT_RXEOL)	? " HAL_INT_RXEOL"	: "",
		(status & HAL_INT_RXORN)	? " HAL_INT_RXORN"	: "",
		(status & HAL_INT_TXDESC)	? " HAL_INT_TXDESC"	: "",
		(status & HAL_INT_TXURN)	? " HAL_INT_TXURN"	: "",
		(status & HAL_INT_RXKCM)	? " HAL_INT_RXKCM"	: "",
		(status & HAL_INT_BMISS)	? " HAL_INT_BMISS"	: "",
		(status & HAL_INT_BNR)		? " HAL_INT_BNR"	: "",
		(status & HAL_INT_TIM)		? " HAL_INT_TIM"	: "",
		(status & HAL_INT_DTIM)		? " HAL_INT_DTIM"	: "",
		(status & HAL_INT_DTIMSYNC)	? " HAL_INT_DTIMSYNC"	: "",
		(status & HAL_INT_GPIO)		? " HAL_INT_GPIO"	: "",
		(status & HAL_INT_CABEND)	? " HAL_INT_CABEND"	: "",
		(status & HAL_INT_CST)		? " HAL_INT_CST"	: "",
		(status & HAL_INT_GTT)		? " HAL_INT_GTT"	: "",
		(status & HAL_INT_FATAL)	? " HAL_INT_FATAL"	: "",
		(status & HAL_INT_GLOBAL)	? " HAL_INT_GLOBAL"	: ""
		);

	status &= sc->sc_imask;			/* discard unasked for bits */
	if (status & HAL_INT_FATAL) {
		sc->sc_stats.ast_hardware++;
		ath_hal_intrset(ah, 0);		/* disable intr's until reset */
		ATH_SCHEDULE_TQUEUE(&sc->sc_fataltq, &needmark);
	} else if (status & HAL_INT_RXORN) {
		sc->sc_stats.ast_rxorn++;
		ath_hal_intrset(ah, 0);		/* disable intr's until reset */
		ATH_SCHEDULE_TQUEUE(&sc->sc_rxorntq, &needmark);
	} else {
		if (status & HAL_INT_SWBA) {
			struct ieee80211vap * vap;
			/* XXX: only one VAP */
			vap = TAILQ_FIRST(&sc->sc_ic.ic_vaps);

			if (vap->iv_opmode == IEEE80211_M_IBSS) {
				u_int64_t hw_tsf = ath_hal_gettsf64(ah);
				/* Update next beacon target time to keep 
				 * track of HW merges (TSF updates) we would 
				 * not notice otherwise. */
				sc->sc_nexttbtt += vap->iv_bss->ni_intval;
				DPRINTF(sc, ATH_DEBUG_INTR,
					"ath_intr HAL_INT_SWBA at tsf %10llx"
					" nexttbtt %10llx\n", hw_tsf,
					(u_int64_t)sc->sc_nexttbtt << 10);
			} else { /* AP mode */
				/* Software beacon alert - time to send a 
				 * beacon. Handle beacon transmission directly;
				 * deferring this is too slow to meet timing 
				 * constraints under load. */
				ath_beacon_send(sc, &needmark);
			}
		}
		if (status & HAL_INT_RXEOL) {
			/*
			 * NB: the hardware should re-read the link when
			 *     RXE bit is written, but it doesn't work at
			 *     least on older hardware revs.
			 */
			sc->sc_stats.ast_rxeol++;
		}
		if (status & HAL_INT_TXURN) {
			sc->sc_stats.ast_txurn++;
			/* bump tx trigger level */
			ath_hal_updatetxtriglevel(ah, AH_TRUE);
		}
		if (status & (HAL_INT_RX | HAL_INT_RXPHY)) {
			ath_uapsd_processtriggers(sc);
			ATH_SCHEDULE_TQUEUE(&sc->sc_rxtq, &needmark);
		}
		if (status & HAL_INT_TX) {
#ifdef ATH_SUPERG_DYNTURBO
			/*
			 * Check if the beacon queue caused the interrupt
			 * when a dynamic turbo switch
			 * is pending so we can initiate the change.
			 * XXX must wait for all VAPs' beacons
			 */

			if (sc->sc_dturbo_switch) {
				u_int32_t txqs = (1 << sc->sc_bhalq);
				ath_hal_gettxintrtxqs(ah, &txqs);
				if (txqs & (1 << sc->sc_bhalq)) {
					sc->sc_dturbo_switch = 0;
					/* Hack: defer switch for 10ms to 
					 * permit slow clients time to 
					 * track us.  This especially
					 * noticeable with Windows clients. */
					mod_timer(&sc->sc_dturbo_switch_mode,
							  jiffies + 
							  msecs_to_jiffies(10));
				}
			}
#endif
			ATH_SCHEDULE_TQUEUE(&sc->sc_txtq, &needmark);
		}
		if (status & HAL_INT_BMISS) {
			sc->sc_stats.ast_bmiss++;
			ATH_SCHEDULE_TQUEUE(&sc->sc_bmisstq, &needmark);
		}
		if (status & HAL_INT_MIB) {
			sc->sc_stats.ast_mib++;
			/* When the card receives lots of PHY errors, the MIB
			 * interrupt will fire at a very rapid rate. We will use
			 * a timer to enforce at least 1 jiffy delay between
			 * MIB interrupts. This should be unproblematic, since
			 * the hardware will continue to update the counters in 
			 * the mean time. */
			sc->sc_imask &= ~HAL_INT_MIB;
			ath_hal_intrset(ah, sc->sc_imask);
			mod_timer(&sc->sc_mib_enable, jiffies + 1);

			/* Let the HAL handle the event. */
			ath_hal_mibevent(ah, &sc->sc_halstats);
		}
	}
	if (needmark)
		mark_bh(IMMEDIATE_BH);
	return IRQ_HANDLED;
}

static void
ath_fatal_tasklet(TQUEUE_ARG data)
{
	struct net_device *dev = (struct net_device *)data;

	printk("%s: hardware error; resetting\n", DEV_NAME(dev));
	ath_reset(dev);
}

static void
ath_rxorn_tasklet(TQUEUE_ARG data)
{
	struct net_device *dev = (struct net_device *)data;

	printk("%s: rx FIFO overrun; resetting\n", DEV_NAME(dev));
	ath_reset(dev);
}

static void
ath_bmiss_tasklet(TQUEUE_ARG data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ath_softc *sc = dev->priv;

	if (time_before(jiffies, sc->sc_ic.ic_bmiss_guard)) {
		/* Beacon miss interrupt occured too short after last beacon
		 * timer configuration. Ignore it as it could be spurious. */
		DPRINTF(sc, ATH_DEBUG_ANY, "%s: ignored\n", __func__);
	} else {
		DPRINTF(sc, ATH_DEBUG_ANY, "%s\n", __func__);
		ieee80211_beacon_miss(&sc->sc_ic);
	}
}

static u_int
ath_chan2flags(struct ieee80211_channel *chan)
{
	u_int flags;
	static const u_int modeflags[] = {
		0,		/* IEEE80211_MODE_AUTO    */
		CHANNEL_A,	/* IEEE80211_MODE_11A     */
		CHANNEL_B,	/* IEEE80211_MODE_11B     */
		CHANNEL_PUREG,	/* IEEE80211_MODE_11G     */
		0,		/* IEEE80211_MODE_FH      */
		CHANNEL_108A,	/* IEEE80211_MODE_TURBO_A */
		CHANNEL_108G,	/* IEEE80211_MODE_TURBO_G */
	};

	flags = modeflags[ieee80211_chan2mode(chan)];

	if (IEEE80211_IS_CHAN_HALF(chan))
		flags |= CHANNEL_HALF;
	else if (IEEE80211_IS_CHAN_QUARTER(chan))
		flags |= CHANNEL_QUARTER;

	return flags;
}

/*
 * Context: process context
 */
static int
ath_init(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	HAL_STATUS status;
	int error = 0;

	ATH_LOCK(sc);

	DPRINTF(sc, ATH_DEBUG_RESET, "%s: mode %d\n", __func__, ic->ic_opmode);

	/*
	 * Stop anything previously setup.  This is safe
	 * whether this is the first time through or not.
	 */
	ath_stop_locked(dev);

#ifdef ATH_CAP_TPC
	/* Re-enable after suspend */
	ath_hal_settpc(ah, tpc);
#endif

	/* Whether we should enable h/w TKIP MIC */
	if ((ic->ic_caps & IEEE80211_C_WME) &&
	    ((ic->ic_caps & IEEE80211_C_WME_TKIPMIC) ||
	    !(ic->ic_flags & IEEE80211_F_WME))) {
		ath_hal_settkipmic(ah, AH_TRUE);
	} else {
		ath_hal_settkipmic(ah, AH_FALSE);
	}

	/*
	 * Flush the skbs allocated for receive in case the rx
	 * buffer size changes.  This could be optimized but for
	 * now we do it each time under the assumption it does
	 * not happen often.
	 */
	ath_flushrecv(sc);

	/*
	 * The basic interface to setting the hardware in a good
	 * state is ``reset''.  On return the hardware is known to
	 * be powered up and with interrupts disabled.  This must
	 * be followed by initialization of the appropriate bits
	 * and then setup of the interrupt mask.
	 */
	sc->sc_curchan.channel = ic->ic_curchan->ic_freq;
	sc->sc_curchan.channelFlags = ath_chan2flags(ic->ic_curchan);
	if (!ath_hal_reset(ah, sc->sc_opmode, &sc->sc_curchan, AH_FALSE, &status)) {
		printk("%s: unable to reset hardware: '%s' (HAL status %u) "
			"(freq %u flags 0x%x)\n", DEV_NAME(dev),
			ath_get_hal_status_desc(status), status,
			sc->sc_curchan.channel, sc->sc_curchan.channelFlags);
		error = -EIO;
		goto done;
	}

	if (sc->sc_softled)
		ath_hal_gpioCfgOutput(ah, sc->sc_ledpin);

	if ((sc->sc_opmode != HAL_M_STA) && sc->sc_hasintmit)
		ath_hal_setintmit(ah, 0);

	/*
	 * This is needed only to setup initial state
	 * but it's best done after a reset.
	 */
	ath_update_txpow(sc);

	/* Set the default RX antenna; it may get lost on reset. */
	ath_setdefantenna(sc, sc->sc_defant);

	/*
	 * Setup the hardware after reset: the key cache
	 * is filled as needed and the receive engine is
	 * set going.  Frame transmit is handled entirely
	 * in the frame output path; there's nothing to do
	 * here except setup the interrupt mask.
	 */
#if 0
	ath_initkeytable(sc);		/* XXX still needed? */
#endif
	if (ath_startrecv(sc) != 0) {
		printk("%s: unable to start recv logic\n", DEV_NAME(dev));
		error = -EIO;
		goto done;
	}
	/* Enable interrupts. */
	sc->sc_imask = HAL_INT_RX | HAL_INT_TX
		  | HAL_INT_RXEOL | HAL_INT_RXORN
		  | HAL_INT_FATAL | HAL_INT_GLOBAL;
	/*
	 * Enable MIB interrupts when there are hardware phy counters.
	 * Note we only do this (at the moment) for station mode.
	 */
	if (sc->sc_needmib && ic->ic_opmode == IEEE80211_M_STA)
		sc->sc_imask |= HAL_INT_MIB;
	ath_hal_intrset(ah, sc->sc_imask);

	/*
	 * The hardware should be ready to go now so it's safe
	 * to kick the 802.11 state machine as it's likely to
	 * immediately call back to us to send mgmt frames.
	 */
	ath_chan_change(sc, ic->ic_curchan);
	ath_set_ack_bitrate(sc, sc->sc_ackrate);
	dev->flags |= IFF_RUNNING;		/* we are ready to go */
	ieee80211_start_running(ic);		/* start all VAPs */
#ifdef ATH_TX99_DIAG
	if (sc->sc_tx99 != NULL)
		sc->sc_tx99->start(sc->sc_tx99);
#endif

done:
	ATH_UNLOCK(sc);
	return error;
}

/* Caller must lock ATH_LOCK
 *
 * Context: softIRQ
 */
static int
ath_stop_locked(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;

	DPRINTF(sc, ATH_DEBUG_RESET, "%s: invalid %u flags 0x%x\n",
		__func__, sc->sc_invalid, dev->flags);

	if (dev->flags & IFF_RUNNING) {
		/*
		 * Shutdown the hardware and driver:
		 *    stop output from above
		 *    reset 802.11 state machine
		 *	(sends station deassoc/deauth frames)
		 *    turn off timers
		 *    disable interrupts
		 *    clear transmit machinery
		 *    clear receive machinery
		 *    turn off the radio
		 *    reclaim beacon resources
		 *
		 * Note that some of this work is not possible if the
		 * hardware is gone (invalid).
		 */
#ifdef ATH_TX99_DIAG
		if (sc->sc_tx99 != NULL)
			sc->sc_tx99->stop(sc->sc_tx99);
#endif
		netif_stop_queue(dev);	/* XXX re-enabled by ath_newstate */
		dev->flags &= ~IFF_RUNNING;	/* NB: avoid recursion */
		ieee80211_stop_running(ic);	/* stop all VAPs */
		if (!sc->sc_invalid) {
			ath_hal_intrset(ah, 0);
			if (sc->sc_softled) {
				del_timer(&sc->sc_ledtimer);
				ath_hal_gpioset(ah, sc->sc_ledpin, !sc->sc_ledon);
				sc->sc_blinking = 0;
				sc->sc_ledstate = 1;
			}
		}
		ath_draintxq(sc);
		if (!sc->sc_invalid) {
			ath_stoprecv(sc);
		} else
			sc->sc_rxlink = NULL;
		ath_beacon_free(sc);		/* XXX needed? */
	} else
		ieee80211_stop_running(ic);	/* stop other VAPs */

	if (sc->sc_softled)
		ath_hal_gpioset(ah, sc->sc_ledpin, !sc->sc_ledon);

	return 0;
}

/*
 * Stop the device, grabbing the top-level lock to protect
 * against concurrent entry through ath_init (which can happen
 * if another thread does a system call and the thread doing the
 * stop is preempted).
 */
static int
ath_stop(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	int error;

	ATH_LOCK(sc);

	if (!sc->sc_invalid)
		ath_hal_setpower(sc->sc_ah, HAL_PM_AWAKE, AH_TRUE);

	error = ath_stop_locked(dev);

#if 0
	if (error == 0 && !sc->sc_invalid) {
		/*
		 * Set the chip in full sleep mode.  Note that we are
		 * careful to do this only when bringing the interface
		 * completely to a stop.  When the chip is in this state
		 * it must be carefully woken up or references to
		 * registers in the PCI clock domain may freeze the bus
		 * (and system).  This varies by chip and is mostly an
		 * issue with newer parts that go to sleep more quickly.
		 */
		ath_hal_setpower(sc->sc_ah, HAL_PM_FULL_SLEEP, AH_TRUE);
	}
#endif
	ATH_UNLOCK(sc);

	return error;
}

int
ar_device(int devid)
{
	switch (devid) {
	case AR5210_DEFAULT:
	case AR5210_PROD:
	case AR5210_AP:
		return 5210;
	case AR5211_DEFAULT:
	case AR5311_DEVID:
	case AR5211_LEGACY:
	case AR5211_FPGA11B:
		return 5211;
	case AR5212_DEFAULT:
	case AR5212_DEVID:
	case AR5212_FPGA:
	case AR5212_DEVID_IBM:
	case AR5212_AR5312_REV2:
	case AR5212_AR5312_REV7:
	case AR5212_AR2313_REV8:
	case AR5212_AR2315_REV6:
	case AR5212_AR2315_REV7:
	case AR5212_AR2317_REV1:
	case AR5212_DEVID_0014:
	case AR5212_DEVID_0015:
	case AR5212_DEVID_0016:
	case AR5212_DEVID_0017:
	case AR5212_DEVID_0018:
	case AR5212_DEVID_0019:
	case AR5212_AR2413:
	case AR5212_AR5413:
	case AR5212_AR5424:
	case AR5212_DEVID_FF19:
		return 5212;
	case AR5213_SREV_1_0:
	case AR5213_SREV_REG:
	case AR_SUBVENDOR_ID_NOG:
	case AR_SUBVENDOR_ID_NEW_A:
		return 5213;
	default:
		return 0; /* unknown */
	}
}


static int
ath_set_ack_bitrate(struct ath_softc *sc, int high)
{
	if (ar_device(sc->devid) == 5212 || ar_device(sc->devid) == 5213) {
		/* set ack to be sent at low bit-rate */
		/* registers taken from the OpenBSD 5212 HAL */
#define AR5K_AR5212_STA_ID1                     0x8004
#define AR5K_AR5212_STA_ID1_ACKCTS_6MB          0x01000000
#define AR5K_AR5212_STA_ID1_BASE_RATE_11B       0x02000000
		u_int32_t v = AR5K_AR5212_STA_ID1_BASE_RATE_11B | 
			AR5K_AR5212_STA_ID1_ACKCTS_6MB;
		if (high)
			ath_reg_write(sc, AR5K_AR5212_STA_ID1, 
					ath_reg_read(sc, AR5K_AR5212_STA_ID1) & ~v);
		else
			ath_reg_write(sc, AR5K_AR5212_STA_ID1, 
					ath_reg_read(sc, AR5K_AR5212_STA_ID1) | v);
#undef AR5K_AR5212_STA_ID1
#undef AR5K_AR5212_STA_ID1_BASE_RATE_11B
#undef AR5K_AR5212_STA_ID1_ACKCTS_6MB
		return 0;
	}
	return 1;
}

/*
 * Reset the hardware w/o losing operational state.  This is
 * basically a more efficient way of doing ath_stop, ath_init,
 * followed by state transitions to the current 802.11
 * operational state.  Used to recover from errors rx overrun
 * and to reset the hardware when rf gain settings must be reset.
 */
static int
ath_reset(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211_channel *c;
	HAL_STATUS status;

	/*
	 * Convert to a HAL channel description with the flags
	 * constrained to reflect the current operating mode.
	 */
	c = ic->ic_curchan;
	sc->sc_curchan.channel = c->ic_freq;
	sc->sc_curchan.channelFlags = ath_chan2flags(c);
	sc->sc_curchan.privFlags = 0;

	ath_hal_intrset(ah, 0);		/* disable interrupts */
	ath_draintxq(sc);		/* stop xmit side */
	ath_stoprecv(sc);		/* stop recv side */
	/* NB: indicate channel change so we do a full reset */
	if (!ath_hal_reset(ah, sc->sc_opmode, &sc->sc_curchan, AH_TRUE, &status))
		printk("%s: %s: unable to reset hardware: '%s' (HAL status %u)\n",
			DEV_NAME(dev), __func__, 
			ath_get_hal_status_desc(status), status);

	if ((sc->sc_opmode != HAL_M_STA) && sc->sc_hasintmit)
		ath_hal_setintmit(ah, 0);

	ath_update_txpow(sc);		/* update tx power state */
	if (ath_startrecv(sc) != 0)	/* restart recv */
		printk("%s: %s: unable to start recv logic\n",
			DEV_NAME(dev), __func__);
	if (sc->sc_softled)
		ath_hal_gpioCfgOutput(ah, sc->sc_ledpin);

	/*
	 * We may be doing a reset in response to an ioctl
	 * that changes the channel so update any state that
	 * might change as a result.
	 */
	ath_chan_change(sc, c);
	if (sc->sc_beacons)
		ath_beacon_config(sc, NULL);	/* restart beacons */
	ath_hal_intrset(ah, sc->sc_imask);
	ath_set_ack_bitrate(sc, sc->sc_ackrate);
	netif_wake_queue(dev);		/* restart xmit */
#ifdef ATH_SUPERG_XR
	/*
	 * restart the group polls.
	 */
	if (sc->sc_xrgrppoll) {
		struct ieee80211vap *vap;
		TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next)
			if (vap && (vap->iv_flags & IEEE80211_F_XR))
				break;
		ath_grppoll_stop(vap);
		ath_grppoll_start(vap, sc->sc_xrpollcount);
	}
#endif
	return 0;
}


/* Swap transmit descriptor.
 * if AH_NEED_DESC_SWAP flag is not defined this becomes a "null"
 * function.
 */
static __inline void
ath_desc_swap(struct ath_desc *ds)
{
#ifdef AH_NEED_DESC_SWAP
	ds->ds_link = cpu_to_le32(ds->ds_link);
	ds->ds_data = cpu_to_le32(ds->ds_data);
	ds->ds_ctl0 = cpu_to_le32(ds->ds_ctl0);
	ds->ds_ctl1 = cpu_to_le32(ds->ds_ctl1);
	ds->ds_hw[0] = cpu_to_le32(ds->ds_hw[0]);
	ds->ds_hw[1] = cpu_to_le32(ds->ds_hw[1]);
#endif
}

/*
 * Insert a buffer on a txq
 *
 */
static __inline void
ath_tx_txqaddbuf(struct ath_softc *sc, struct ieee80211_node *ni,
	struct ath_txq *txq, struct ath_buf *bf,
	struct ath_desc *lastds, int framelen)
{
	struct ath_hal *ah = sc->sc_ah;

	/*
	 * Insert the frame on the outbound list and
	 * pass it on to the hardware.
	 */
	ATH_TXQ_LOCK_IRQ(txq);
	if (ni && ni->ni_vap && txq == &ATH_VAP(ni->ni_vap)->av_mcastq) {
		/*
		 * The CAB queue is started from the SWBA handler since
		 * frames only go out on DTIM and to avoid possible races.
		 */
		ath_hal_intrset(ah, sc->sc_imask & ~HAL_INT_SWBA);
		ATH_TXQ_INSERT_TAIL(txq, bf, bf_list);
		DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: MC txq [%d] depth = %d\n", 
				__func__, txq->axq_qnum, txq->axq_depth);
		if (txq->axq_link != NULL) {
#ifdef AH_NEED_DESC_SWAP
			*txq->axq_link = cpu_to_le32(bf->bf_daddr);
#else
			*txq->axq_link = bf->bf_daddr;
#endif
			DPRINTF(sc, ATH_DEBUG_XMIT, "%s: link[%u](%p)=%llx (%p)\n",
				__func__,
				txq->axq_qnum, txq->axq_link,
				ito64(bf->bf_daddr), bf->bf_desc);
		}
		txq->axq_link = &lastds->ds_link;
		/* We do not start tx on this queue as it will be done as
		"CAB" data at DTIM intervals. */
		ath_hal_intrset(ah, sc->sc_imask);
	} else {
		ATH_TXQ_INSERT_TAIL(txq, bf, bf_list);
		DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: UC txq [%d] depth = %d\n", 
				__func__, txq->axq_qnum, txq->axq_depth);
		if (txq->axq_link == NULL) {
			ath_hal_puttxbuf(ah, txq->axq_qnum, bf->bf_daddr);
			DPRINTF(sc, ATH_DEBUG_XMIT, "%s: TXDP[%u] = %llx (%p)\n",
				__func__,
				txq->axq_qnum, ito64(bf->bf_daddr), bf->bf_desc);
		} else {
#ifdef AH_NEED_DESC_SWAP
			*txq->axq_link = cpu_to_le32(bf->bf_daddr);
#else
			*txq->axq_link = bf->bf_daddr;
#endif
			DPRINTF(sc, ATH_DEBUG_XMIT, "%s: link[%u] (%p)=%llx (%p)\n",
				__func__,
				txq->axq_qnum, txq->axq_link,
				ito64(bf->bf_daddr), bf->bf_desc);
		}
		txq->axq_link = &lastds->ds_link;
		ath_hal_txstart(ah, txq->axq_qnum);
		sc->sc_dev->trans_start = jiffies;
	}
	ATH_TXQ_UNLOCK_IRQ(txq);

	sc->sc_devstats.tx_packets++;
	sc->sc_devstats.tx_bytes += framelen;
}

static int
dot11_to_ratecode(struct ath_softc *sc, const HAL_RATE_TABLE *rt, int dot11)
{
	int index = sc->sc_rixmap[dot11 & IEEE80211_RATE_VAL];
	if (index >= 0 && index < rt->rateCount)
		return rt->info[index].rateCode;

	return rt->info[sc->sc_minrateix].rateCode;
}


static int
ath_tx_startraw(struct net_device *dev, struct ath_buf *bf, struct sk_buff *skb)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211_phy_params *ph = (struct ieee80211_phy_params *)
		(SKB_CB(skb) + sizeof(struct ieee80211_cb));
	const HAL_RATE_TABLE *rt;
	unsigned int pktlen, hdrlen, try0, power;
	HAL_PKT_TYPE atype;
	u_int flags;
	u_int8_t antenna, txrate;
	struct ath_txq *txq = NULL;
	struct ath_desc *ds = NULL;
	struct ieee80211_frame *wh;

	wh = (struct ieee80211_frame *)skb->data;
	try0 = ph->try0;
	rt = sc->sc_currates;
	txrate = dot11_to_ratecode(sc, rt, ph->rate0);
	power = ph->power > 60 ? 60 : ph->power;
	hdrlen = ieee80211_anyhdrsize(wh);
	pktlen = skb->len + IEEE80211_CRC_LEN;

	flags = HAL_TXDESC_INTREQ | HAL_TXDESC_CLRDMASK; /* XXX needed for crypto errs */

	bf->bf_skbaddr = bus_map_single(sc->sc_bdev,
					skb->data, pktlen, BUS_DMA_TODEVICE);
	DPRINTF(sc, ATH_DEBUG_XMIT, "%s: skb %p [data %p len %u] skbaddr %llx\n",
		__func__, skb, skb->data, skb->len, ito64(bf->bf_skbaddr));

	bf->bf_skb = skb;
	KASSERT((bf->bf_node == NULL), ("Detected node reference leak"));
#ifdef ATH_SUPERG_FF
	bf->bf_numdescff = 0;
#endif

	/* setup descriptors */
	ds = bf->bf_desc;
	rt = sc->sc_currates;
	KASSERT(rt != NULL, ("no rate table, mode %u", sc->sc_curmode));

	if (IEEE80211_IS_MULTICAST(wh->i_addr1)) {
		flags |= HAL_TXDESC_NOACK;	/* no ack on broad/multicast */
		sc->sc_stats.ast_tx_noack++;
		try0 = 1;
	}
	atype = HAL_PKT_TYPE_NORMAL;		/* default */
	txq = sc->sc_ac2q[skb->priority & 0x3];


	flags |= HAL_TXDESC_INTREQ;
	antenna = sc->sc_txantenna;

	/* XXX check return value? */
	ath_hal_setuptxdesc(ah, ds,
			    pktlen,			/* packet length */
			    hdrlen,			/* header length */
			    atype,			/* Atheros packet type */
			    power,			/* txpower */
			    txrate, try0,		/* series 0 rate/tries */
			    HAL_TXKEYIX_INVALID,	/* key cache index */
			    antenna,			/* antenna mode */
			    flags,			/* flags */
			    0,				/* rts/cts rate */
			    0,				/* rts/cts duration */
			    0,				/* comp icv len */
			    0,				/* comp iv len */
			    ATH_COMP_PROC_NO_COMP_NO_CCS /* comp scheme */
			   );

	if (ph->try1) {
		ath_hal_setupxtxdesc(sc->sc_ah, ds,
			dot11_to_ratecode(sc, rt, ph->rate1), ph->try1,
			dot11_to_ratecode(sc, rt, ph->rate2), ph->try2,
			dot11_to_ratecode(sc, rt, ph->rate3), ph->try3
			);
	}
	bf->bf_flags = flags;	/* record for post-processing */

	ds->ds_link = 0;
	ds->ds_data = bf->bf_skbaddr;

	ath_hal_filltxdesc(ah, ds,
			   skb->len,	/* segment length */
			   AH_TRUE,	/* first segment */
			   AH_TRUE,	/* last segment */
			   ds		/* first descriptor */
			   );

	/* NB: The desc swap function becomes void,
	 * if descriptor swapping is not enabled
	 */
	ath_desc_swap(ds);

	DPRINTF(sc, ATH_DEBUG_XMIT, "%s: Q%d: %08x %08x %08x %08x %08x %08x\n",
		__func__, M_FLAG_GET(skb, M_UAPSD) ? 0 : txq->axq_qnum, 
		ds->ds_link, ds->ds_data,
		ds->ds_ctl0, ds->ds_ctl1, 
		ds->ds_hw[0], ds->ds_hw[1]);

	ath_tx_txqaddbuf(sc, NULL, txq, bf, ds, pktlen);
	return 0;
}

#ifdef ATH_SUPERG_FF
/* Flush FF staging queue. */
static int
ath_ff_neverflushtestdone(struct ath_txq *txq, struct ath_buf *bf)
{
	return 0;
}

static int
ath_ff_ageflushtestdone(struct ath_txq *txq, struct ath_buf *bf)
{
	if ((txq->axq_totalqueued - bf->bf_queueage) < ATH_FF_STAGEQAGEMAX)
		return 1;

	return 0;
}

/* Caller must not hold ATH_TXQ_LOCK_IRQ and ATH_TXBUF_LOCK_IRQ
 *
 * Context: softIRQ
 */
static void
ath_ffstageq_flush(struct ath_softc *sc, struct ath_txq *txq,
	int (*ath_ff_flushdonetest)(struct ath_txq *txq, struct ath_buf *bf))
{
	struct ath_buf *bf_ff = NULL;
	unsigned int pktlen;
	int framecnt;

	for (;;) {
		ATH_TXQ_LOCK_IRQ(txq);

		bf_ff = TAILQ_LAST(&txq->axq_stageq, axq_headtype);
		if ((!bf_ff) || ath_ff_flushdonetest(txq, bf_ff)) {
			ATH_TXQ_UNLOCK_IRQ_EARLY(txq);
			return;
		}

		KASSERT(ATH_NODE(bf_ff->bf_node)->an_tx_ffbuf[bf_ff->bf_skb->priority],
			("no bf_ff on staging queue %p", bf_ff));
		ATH_NODE(bf_ff->bf_node)->an_tx_ffbuf[bf_ff->bf_skb->priority] = NULL;
		TAILQ_REMOVE(&txq->axq_stageq, bf_ff, bf_stagelist);

		ATH_TXQ_UNLOCK_IRQ(txq);

		/* encap and xmit */
		bf_ff->bf_skb = ieee80211_encap(bf_ff->bf_node, bf_ff->bf_skb, 
				&framecnt);
		if (bf_ff->bf_skb == NULL) {
			DPRINTF(sc, ATH_DEBUG_XMIT | ATH_DEBUG_FF,
				"%s: discard, encapsulation failure\n", __func__);
			sc->sc_stats.ast_tx_encap++;
			goto bad;
		}
		pktlen = bf_ff->bf_skb->len;	/* NB: don't reference skb below */
		if (ath_tx_start(sc->sc_dev, bf_ff->bf_node, bf_ff, 
					bf_ff->bf_skb, 0) == 0)
			continue;
	bad:
		ath_return_txbuf(sc, &bf_ff);
	}
}
#endif

static inline
u_int32_t
ath_get_buffers_available(void)
{
	return ATH_TXBUF - atomic_read(&ath_buf_counter);
}

#ifdef IEEE80211_DEBUG_REFCNT
/* NOTE: This function is valid in non-debug configurations, just not used. */
static inline
u_int32_t
ath_get_buffer_count(void)
{
	return atomic_read(&ath_buf_counter);
}
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */

static 
struct ath_buf*
#ifdef IEEE80211_DEBUG_REFCNT
_take_txbuf_locked_debug(struct ath_softc *sc, int for_management, 
			const char *func, int line)
#else
_take_txbuf_locked(struct ath_softc *sc, int for_management)
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */
{
	struct ath_buf* bf = NULL;
	ATH_TXBUF_LOCK_ASSERT(sc);
	/* Reserve at least ATH_TXBUF_MGT_RESERVED buffers for management frames */
	if (ath_get_buffers_available() <= ATH_TXBUF_MGT_RESERVED) {
		/* Stop the queue, we are full */
		DPRINTF(sc, ATH_DEBUG_XMIT, "%s: stopping queuing of additional "
					    "frames.  Insufficient free "
					    "buffers.\n", __func__);
		sc->sc_stats.ast_tx_qstop++;
		netif_stop_queue(sc->sc_dev);
		sc->sc_devstopped = 1;
		ATH_SCHEDULE_TQUEUE(&sc->sc_txtq, NULL);
	}

	/* Only let us go further if management frame, or there are enough */
	if (for_management || (ath_get_buffers_available() > ATH_TXBUF_MGT_RESERVED)) {
		bf = STAILQ_FIRST(&sc->sc_txbuf);
		if (bf) {
			STAILQ_REMOVE_HEAD(&sc->sc_txbuf, bf_list);
			/* This should be redundant, unless someone illegally 
			 * accessed the buffer after returning it. */
#ifdef IEEE80211_DEBUG_REFCNT
			cleanup_ath_buf_debug(sc, bf, BUS_DMA_TODEVICE, func, line);
#else
			cleanup_ath_buf(sc, bf, BUS_DMA_TODEVICE);
#endif
			atomic_inc(&ath_buf_counter);
#ifdef IEEE80211_DEBUG_REFCNT
			DPRINTF(sc, ATH_DEBUG_TXBUF, 
				"[TXBUF=%03d/%03d] %s:%d -> %s:%d took txbuf %p.\n", 
				ath_get_buffer_count(), ATH_TXBUF,
				func, line, __func__, __LINE__,
				bf);
#endif
		}
		else {
			DPRINTF(sc, ATH_DEBUG_ANY, 
				"%s: discard %s, no xmit buffers available.\n", 
				__func__,
				for_management ? "management frame" : "frame");
			sc->sc_stats.ast_tx_nobuf++;
		}
	}

	return bf;
}

static
struct ath_buf*
#ifdef IEEE80211_DEBUG_REFCNT
_take_txbuf_debug(struct ath_softc *sc, int for_management,
			  const char *func, int line) {
#else
_take_txbuf(struct ath_softc *sc, int for_management) {
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */
	struct ath_buf* bf = NULL;
	ATH_TXBUF_LOCK_IRQ(sc);
#ifdef IEEE80211_DEBUG_REFCNT
	bf = _take_txbuf_locked_debug(sc, for_management, func, line);
#else
	bf = _take_txbuf_locked(sc, for_management);
#endif
	ATH_TXBUF_UNLOCK_IRQ(sc);
	return bf;
}

#ifdef IEEE80211_DEBUG_REFCNT

#define ath_take_txbuf_locked(_sc) \
	_take_txbuf_locked_debug(_sc, 0, __func__, __LINE__)
#define ath_take_txbuf_locked_debug(_sc, _func, _line) \
	_take_txbuf_locked_debug(_sc, 0, _func, _line)

#define ath_take_txbuf_mgmt_locked(_sc) \
	_take_txbuf_locked_debug(_sc, 1, __func__, __LINE__)
#define ath_take_txbuf_mgmt_locked_debug(_sc, _func, _line) \
	_take_txbuf_locked_debug(_sc, 1, _func, _line)

#define ath_take_txbuf(_sc) \
	_take_txbuf_debug(_sc, 0, __func__, __LINE__)
#define ath_take_txbuf_debug(_sc, _func, _line) \
	_take_txbuf_debug(_sc, 0, _func, _line)

#define ath_take_txbuf_mgmt(_sc) \
	_take_txbuf_debug(_sc, 1, __func__, __LINE__)
#define ath_take_txbuf_mgmt_debug(_sc, _func, _line) \
	_take_txbuf_debug(_sc, 1, _func, _line)

#else /* #ifdef IEEE80211_DEBUG_REFCNT */

#define ath_take_txbuf_locked(_sc) \
	_take_txbuf_locked(_sc, 0)
#define ath_take_txbuf_locked_debug(_sc, _func, _line) \
	_take_txbuf_locked(_sc, 0)

#define ath_take_txbuf_mgmt_locked(_sc) \
	_take_txbuf_locked(_sc, 1)
#define ath_take_txbuf_mgmt_locked_debug(_sc, _func, _line) \
	_take_txbuf_locked(_sc, 1)

#define ath_take_txbuf(_sc) \
	_take_txbuf(_sc, 0)
#define ath_take_txbuf_debug(_sc, _func, _line) \
	_take_txbuf(_sc, 0)

#define ath_take_txbuf_mgmt(_sc) \
	_take_txbuf(_sc, 1)
#define ath_take_txbuf_mgmt_debug(_sc, _func, _line) \
	_take_txbuf(_sc, 1)

#endif /* #ifdef IEEE80211_DEBUG_REFCNT */


/*
 * Transmit a data packet.  On failure caller is
 * assumed to reclaim the resources.
 *
 * Context: process context with BHs disabled
 * It mut return either NETDEV_TX_OK or NETDEV_TX_BUSY
 */
static int
ath_hardstart(struct sk_buff *skb, struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211_node *ni = NULL;
	struct ath_buf *bf = NULL;
	struct ether_header *eh;
	ath_bufhead bf_head;
	struct ath_buf *tbf, *tempbf;
	struct sk_buff *tskb;
	int framecnt;
	int requeue = 0;
#ifdef ATH_SUPERG_FF
	unsigned int pktlen;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_node *an;
	struct ath_txq *txq = NULL;
	int ff_flush;
	struct ieee80211vap *vap;
#endif

	/* If an skb is passed in directly from the kernel, 
	 * we take responsibility for the reference */
	ieee80211_skb_track(skb);

	if ((dev->flags & IFF_RUNNING) == 0 || sc->sc_invalid) {
		DPRINTF(sc, ATH_DEBUG_XMIT,
			"%s: discard, invalid %d flags %x\n",
			__func__, sc->sc_invalid, dev->flags);
		sc->sc_stats.ast_tx_invalid++;
		return -ENETDOWN;
	}

	STAILQ_INIT(&bf_head);

	if (SKB_CB(skb)->flags & M_RAW) {
		bf = ath_take_txbuf(sc);
		if (bf == NULL)
			goto hardstart_fail;
		ath_tx_startraw(dev, bf, skb);
		return NETDEV_TX_OK;
	}

	eh = (struct ether_header *)skb->data;
	ni = SKB_CB(skb)->ni;		/* NB: always passed down by 802.11 layer */
	if (ni == NULL) {
		/* NB: this happens if someone marks the underlying device up */
		DPRINTF(sc, ATH_DEBUG_XMIT,
			"%s: discard, no node in cb\n", __func__);
		goto hardstart_fail;
	}
#ifdef ATH_SUPERG_FF
	vap = ni->ni_vap;

	if (M_FLAG_GET(skb, M_UAPSD)) {
		/* bypass FF handling */
		bf = ath_take_txbuf(sc);
		if (bf == NULL)
			goto hardstart_fail;
		goto ff_bypass;
	}

	/*
	 * Fast frames check.
	 */
	ATH_FF_MAGIC_CLR(skb);
	an = ATH_NODE(ni);

	txq = sc->sc_ac2q[skb->priority];

	if (txq->axq_depth > TAIL_DROP_COUNT) {
		sc->sc_stats.ast_tx_discard++;
		/* queue is full, let the kernel backlog the skb */
		netif_stop_queue(dev);
		requeue = 1;
               
		goto hardstart_fail;
	}
#endif

	/* If the skb data is shared, we will copy it so we can strip padding
	 * without affecting any other bridge ports. */
	if (skb_cloned(skb)) {
		/* Remember the original SKB so we can free up our references */
		struct sk_buff *skb_orig = skb;
		skb = skb_copy(skb, GFP_ATOMIC);
		if (skb == NULL) {
			requeue = 1;
			netif_stop_queue(dev);
			goto hardstart_fail;
		}
		/* If the clone works, bump the reference count for our copy. */
		SKB_CB(skb)->ni = ieee80211_ref_node(SKB_CB(skb_orig)->ni);
		ieee80211_dev_kfree_skb(&skb_orig);
	} else {
		if (SKB_CB(skb)->ni != NULL) 
			ieee80211_unref_node(&SKB_CB(skb)->ni);
		skb_orphan(skb);
	}

	eh = (struct ether_header *) skb->data;

#ifdef ATH_SUPERG_FF
	/* NB: use this lock to protect an->an_tx_ffbuf (and txq->axq_stageq)
	 *     in athff_can_aggregate() call too. */
	ATH_TXQ_LOCK_IRQ(txq);
	if (athff_can_aggregate(sc, eh, an, skb, 
				vap->iv_fragthreshold, &ff_flush)) {
		if (an->an_tx_ffbuf[skb->priority]) { /* i.e., frame on the staging queue */
			bf = an->an_tx_ffbuf[skb->priority];

			/* get (and remove) the frame from staging queue */
			TAILQ_REMOVE(&txq->axq_stageq, bf, bf_stagelist);
			an->an_tx_ffbuf[skb->priority] = NULL;

			/*
			 * chain skbs and add FF magic
			 *
			 * NB: the arriving skb should not be on a list (skb->list),
			 *     so "re-using" the skb next field should be OK.
			 */
			bf->bf_skb->next = skb;
			skb->next = NULL;
			skb = bf->bf_skb;
			ATH_FF_MAGIC_PUT(skb);

			DPRINTF(sc, ATH_DEBUG_XMIT | ATH_DEBUG_FF,
				"%s: aggregating fast-frame\n", __func__);
		} else {
			/* NB: Careful grabbing the TX_BUF lock since still 
			 *     holding the TXQ lock.  This could be avoided 
			 *     by always obtaining the TXBuf earlier, but 
			 *     the "if" portion of this "if/else" clause would 
			 *     then need to give the buffer back. */
			bf = ath_take_txbuf(sc);
			if (bf == NULL) {
				ATH_TXQ_UNLOCK_IRQ_EARLY(txq);
				goto hardstart_fail;
			}
			DPRINTF(sc, ATH_DEBUG_XMIT | ATH_DEBUG_FF,
				"%s: adding to fast-frame stage Q\n", __func__);

			bf->bf_skb = skb;
			bf->bf_node = ieee80211_ref_node(ni);
			bf->bf_queueage = txq->axq_totalqueued;
			an->an_tx_ffbuf[skb->priority] = bf;

			TAILQ_INSERT_HEAD(&txq->axq_stageq, bf, bf_stagelist);

			ATH_TXQ_UNLOCK_IRQ_EARLY(txq);

			return NETDEV_TX_OK;
		}
	} else {
		if (ff_flush) {
			struct ath_buf *bf_ff = an->an_tx_ffbuf[skb->priority];
			int success = 0;

			TAILQ_REMOVE(&txq->axq_stageq, bf_ff, bf_stagelist);
			an->an_tx_ffbuf[skb->priority] = NULL;

			/* NB: ath_tx_start -> ath_tx_txqaddbuf uses ATH_TXQ_LOCK too */
			ATH_TXQ_UNLOCK_IRQ_EARLY(txq);

			/* Encap. and transmit */
			bf_ff->bf_skb = ieee80211_encap(ni, bf_ff->bf_skb, 
					&framecnt);
			if (bf_ff->bf_skb == NULL) {
				DPRINTF(sc, ATH_DEBUG_XMIT,
					"%s: discard, fast-frame flush encap. "
					"failure\n", __func__);
				sc->sc_stats.ast_tx_encap++;
			} else {
				pktlen = bf_ff->bf_skb->len;	/* NB: don't reference skb below */
				if (!ath_tx_start(dev, ni, bf_ff, 
							bf_ff->bf_skb, 0))
					success = 1;
			}

			if (!success) {
				DPRINTF(sc, ATH_DEBUG_XMIT | ATH_DEBUG_FF,
					"%s: fast-frame stageq flush "
					"failure\n", __func__);
				ath_return_txbuf(sc, &bf_ff);
			}
			bf = ath_take_txbuf(sc);
			if (bf == NULL) {
				goto hardstart_fail;
			}

			goto ff_flush_done;
		}
		/* XXX: out-of-order condition only occurs for AP mode and 
		 *      multicast.  But, there may be no valid way to get 
		 *      this condition. */
		else if (an->an_tx_ffbuf[skb->priority]) {
			DPRINTF(sc, ATH_DEBUG_XMIT | ATH_DEBUG_FF,
				"%s: Out-Of-Order fast-frame\n", __func__);
		}

		bf = ath_take_txbuf(sc);
		if (bf == NULL) {
			ATH_TXQ_UNLOCK_IRQ_EARLY(txq);
			goto hardstart_fail;
		}
	}

	ATH_TXQ_UNLOCK_IRQ(txq);

ff_flush_done:
ff_bypass:

#else /* ATH_SUPERG_FF */

	bf = ath_take_txbuf(sc);
	if (bf == NULL) {
		goto hardstart_fail;
	}

#endif /* ATH_SUPERG_FF */

	/*
	 * Encapsulate the packet for transmission.
	 */
	skb = ieee80211_encap(ni, skb, &framecnt);
	if (skb == NULL) {
		DPRINTF(sc, ATH_DEBUG_XMIT,
			"%s: discard, encapsulation failure\n", __func__);
		sc->sc_stats.ast_tx_encap++;
		goto hardstart_fail;
	}

	if (framecnt > 1) {
		unsigned int bfcnt;

		/*
		 *  Allocate 1 ath_buf for each frame given 1 was
		 *  already alloc'd
		 */
		ATH_TXBUF_LOCK_IRQ(sc);
		for (bfcnt = 1; bfcnt < framecnt; ++bfcnt) {
			tbf = ath_take_txbuf_locked(sc);
			if (tbf == NULL)
				break;
			tbf->bf_node = ieee80211_ref_node(SKB_CB(skb)->ni);
			STAILQ_INSERT_TAIL(&bf_head, tbf, bf_list);
		}

		if (bfcnt != framecnt) {
			ath_return_txbuf_list_locked(sc, &bf_head);
			ATH_TXBUF_UNLOCK_IRQ_EARLY(sc);
			STAILQ_INIT(&bf_head);
			goto hardstart_fail;
		}
		ATH_TXBUF_UNLOCK_IRQ(sc);

		while (((bf = STAILQ_FIRST(&bf_head)) != NULL) && (skb != NULL)) {
			unsigned int nextfraglen = 0;

			STAILQ_REMOVE_HEAD(&bf_head, bf_list);
			tskb = skb->next;
			skb->next = NULL;
			if (tskb)
				nextfraglen = tskb->len;

			if (ath_tx_start(dev, ni, bf, skb, nextfraglen) != 0) {
				STAILQ_INSERT_TAIL(&bf_head, bf, bf_list);
				skb->next = tskb;
				goto hardstart_fail;
			}
			skb = tskb;
		}
	} else {
		if (ath_tx_start(dev, ni, bf, skb, 0) != 0) {
			STAILQ_INSERT_TAIL(&bf_head, bf, bf_list);
			goto hardstart_fail;
		}
	}

	ni = NULL;

#ifdef ATH_SUPERG_FF
	/* flush out stale FF from staging Q for applicable operational modes. */
	/* XXX: ADHOC mode too? */
	if (txq && ic->ic_opmode == IEEE80211_M_HOSTAP)
		ath_ffstageq_flush(sc, txq, ath_ff_ageflushtestdone);
#endif

	return NETDEV_TX_OK;

hardstart_fail:
	/* Clear all SKBs from the buffers, we will clear them separately IF
	 * we do not requeue them. */
	ATH_TXBUF_LOCK_IRQ(sc);
	STAILQ_FOREACH_SAFE(tbf, &bf_head, bf_list, tempbf) {
		tbf->bf_skb = NULL;
	}
	ATH_TXBUF_UNLOCK_IRQ(sc);
	/* Release the buffers, now that skbs are disconnected */
	ath_return_txbuf_list(sc, &bf_head);
	/* Pass control of the skb to the caller (i.e., resources are their 
	 * problem). */
	if (requeue) {
		/* Stop tracking again we are giving it back*/
		ieee80211_skb_untrack(skb);
		return NETDEV_TX_BUSY;
	}
	/* Now free the SKBs */
	ieee80211_dev_kfree_skb_list(&skb);
	return NETDEV_TX_OK;	
}

/*
 * Transmit a management frame.  On failure we reclaim the skbuff.
 * Note that management frames come directly from the 802.11 layer
 * and do not honor the send queue flow control.  Need to investigate
 * using priority queuing so management frames can bypass data.
 *
 * Context: hwIRQ and softIRQ
 */
static int
ath_mgtstart(struct ieee80211com *ic, struct sk_buff *skb)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	struct ath_buf *bf = NULL;
	int error;

	if ((dev->flags & IFF_RUNNING) == 0 || sc->sc_invalid) {
		DPRINTF(sc, ATH_DEBUG_XMIT,
			"%s: discard, invalid %d flags %x\n",
			__func__, sc->sc_invalid, dev->flags);
		sc->sc_stats.ast_tx_invalid++;
		error = -ENETDOWN;
		goto bad;
	}
	/*
	 * Grab a TX buffer and associated resources.
	 */
	bf = ath_take_txbuf_mgmt(sc);
	if (bf == NULL) {
		printk("ath_mgtstart: discard, no xmit buf\n");
		sc->sc_stats.ast_tx_nobufmgt++;
		error = -ENOBUFS;
		goto bad;
	}

	/*
	 * NB: the referenced node pointer is in the
	 * control block of the sk_buff.  This is
	 * placed there by ieee80211_mgmt_output because
	 * we need to hold the reference with the frame.
	 */
	error = ath_tx_start(dev, SKB_CB(skb)->ni, bf, skb, 0);
	if (error)
		goto bad;

	sc->sc_stats.ast_tx_mgmt++;
	return 0;
bad:
	ath_return_txbuf(sc, &bf);
	return error;
}

#ifdef AR_DEBUG
static void
ath_keyprint(struct ath_softc *sc, const char *tag, u_int ix,
	const HAL_KEYVAL *hk, const u_int8_t mac[IEEE80211_ADDR_LEN])
{
	static const char *ciphers[] = {
		"WEP",
		"AES-OCB",
		"AES-CCM",
		"CKIP",
		"TKIP",
		"CLR",
	};
	unsigned int i, n;

	printk("%s: [%02u] %-7s ", tag, ix, ciphers[hk->kv_type]);
	for (i = 0, n = hk->kv_len; i < n; i++)
		printk("%02x", hk->kv_val[i]);
	printk(" mac %s", ether_sprintf(mac));
	if (hk->kv_type == HAL_CIPHER_TKIP) {
		printk(" %s ", sc->sc_splitmic ? "mic" : "rxmic");
		for (i = 0; i < sizeof(hk->kv_mic); i++)
			printk("%02x", hk->kv_mic[i]);
#if HAL_ABI_VERSION > 0x06052200
		if (!sc->sc_splitmic) {
			printk(" txmic ");
			for (i = 0; i < sizeof(hk->kv_txmic); i++)
				printk("%02x", hk->kv_txmic[i]);
		}
#endif
	}
	printk("\n");
}
#endif

/*
 * Set a TKIP key into the hardware.  This handles the
 * potential distribution of key state to multiple key
 * cache slots for TKIP.
 */
static int
ath_keyset_tkip(struct ath_softc *sc, const struct ieee80211_key *k,
	HAL_KEYVAL *hk, const u_int8_t mac[IEEE80211_ADDR_LEN])
{
#define	IEEE80211_KEY_XR	(IEEE80211_KEY_XMIT | IEEE80211_KEY_RECV)
	static const u_int8_t zerobssid[IEEE80211_ADDR_LEN];
	struct ath_hal *ah = sc->sc_ah;

	KASSERT(k->wk_cipher->ic_cipher == IEEE80211_CIPHER_TKIP,
		("got a non-TKIP key, cipher %u", k->wk_cipher->ic_cipher));
	if ((k->wk_flags & IEEE80211_KEY_XR) == IEEE80211_KEY_XR) {
		if (sc->sc_splitmic) {
			/*
			 * TX key goes at first index, RX key at the rx index.
			 * The HAL handles the MIC keys at index+64.
			 */
			memcpy(hk->kv_mic, k->wk_txmic, sizeof(hk->kv_mic));
			KEYPRINTF(sc, k->wk_keyix, hk, zerobssid);
			if (!ath_hal_keyset(ah, ATH_KEY(k->wk_keyix), hk, 
						zerobssid, AH_FALSE))
				return 0;

			memcpy(hk->kv_mic, k->wk_rxmic, sizeof(hk->kv_mic));
			KEYPRINTF(sc, k->wk_keyix + 32, hk, mac);
			/* XXX delete tx key on failure? */
			return ath_hal_keyset(ah, ATH_KEY(k->wk_keyix + 32), 
					hk, mac, AH_FALSE);
		} else {
			/*
			 * Room for both TX+RX MIC keys in one key cache
			 * slot, just set key at the first index; the HAL
			 * will handle the reset.
			 */
			memcpy(hk->kv_mic, k->wk_rxmic, sizeof(hk->kv_mic));
#if HAL_ABI_VERSION > 0x06052200
			memcpy(hk->kv_txmic, k->wk_txmic, sizeof(hk->kv_txmic));
#endif
			KEYPRINTF(sc, k->wk_keyix, hk, mac);
			return ath_hal_keyset(ah, ATH_KEY(k->wk_keyix), hk, 
					mac, AH_FALSE);
		}
	} else if (k->wk_flags & IEEE80211_KEY_XR) {
		/*
		 * TX/RX key goes at first index.
		 * The HAL handles the MIC keys are index+64.
		 */
		memcpy(hk->kv_mic, k->wk_flags & IEEE80211_KEY_XMIT ?
			k->wk_txmic : k->wk_rxmic, sizeof(hk->kv_mic));
		KEYPRINTF(sc, k->wk_keyix, hk, mac);
		return ath_hal_keyset(ah, ATH_KEY(k->wk_keyix), hk, mac, 
				AH_FALSE);
	}
	return 0;
#undef IEEE80211_KEY_XR
}

/*
 * Set a net80211 key into the hardware.  This handles the
 * potential distribution of key state to multiple key
 * cache slots for TKIP with hardware MIC support.
 */
static int
ath_keyset(struct ath_softc *sc, const struct ieee80211_key *k,
	const u_int8_t mac0[IEEE80211_ADDR_LEN],
	struct ieee80211_node *bss)
{
	static const u_int8_t ciphermap[] = {
		HAL_CIPHER_WEP,		/* IEEE80211_CIPHER_WEP */
		HAL_CIPHER_TKIP,	/* IEEE80211_CIPHER_TKIP */
		HAL_CIPHER_AES_OCB,	/* IEEE80211_CIPHER_AES_OCB */
		HAL_CIPHER_AES_CCM,	/* IEEE80211_CIPHER_AES_CCM */
		(u_int8_t) -1,		/* 4 is not allocated */
		HAL_CIPHER_CKIP,	/* IEEE80211_CIPHER_CKIP */
		HAL_CIPHER_CLR,		/* IEEE80211_CIPHER_NONE */
	};
	struct ath_hal *ah = sc->sc_ah;
	const struct ieee80211_cipher *cip = k->wk_cipher;
	u_int8_t gmac[IEEE80211_ADDR_LEN];
	const u_int8_t *mac;
	HAL_KEYVAL hk;

	memset(&hk, 0, sizeof(hk));
	/*
	 * Software crypto uses a "clear key" so non-crypto
	 * state kept in the key cache are maintained and
	 * so that rx frames have an entry to match.
	 */
	if ((k->wk_flags & IEEE80211_KEY_SWCRYPT) == 0) {
		KASSERT(cip->ic_cipher < ARRAY_SIZE(ciphermap),
			("invalid cipher type %u", cip->ic_cipher));
		hk.kv_type = ciphermap[cip->ic_cipher];
		hk.kv_len = k->wk_keylen;
		memcpy(hk.kv_val, k->wk_key, k->wk_keylen);
	} else
		hk.kv_type = HAL_CIPHER_CLR;

	if ((k->wk_flags & IEEE80211_KEY_GROUP) && sc->sc_mcastkey) {
		/*
		 * Group keys on hardware that supports multicast frame
		 * key search use a mac that is the sender's address with
		 * the high bit set instead of the app-specified address.
		 */
		IEEE80211_ADDR_COPY(gmac, bss->ni_macaddr);
		gmac[0] |= 0x80;
		mac = gmac;
	} else
		mac = mac0;

	if (hk.kv_type == HAL_CIPHER_TKIP &&
	    (k->wk_flags & IEEE80211_KEY_SWMIC) == 0) {
		return ath_keyset_tkip(sc, k, &hk, mac);
	} else {
		KEYPRINTF(sc, k->wk_keyix, &hk, mac);
		return ath_hal_keyset(ah, ATH_KEY(k->wk_keyix), &hk, mac, 
				AH_FALSE);
	}
}

/*
 * Allocate tx/rx key slots for TKIP.  We allocate two slots for
 * each key, one for decrypt/encrypt and the other for the MIC.
 */
static ieee80211_keyix_t
key_alloc_2pair(struct ath_softc *sc)
{
	u_int i;
	ieee80211_keyix_t keyix;

	KASSERT(sc->sc_splitmic, ("key cache !split"));
	/* XXX could optimize */
	for (i = 0; i < ARRAY_SIZE(sc->sc_keymap) / 4; i++) {
		u_int8_t b = sc->sc_keymap[i];
		if (b != 0xff) {
			/*
			 * One or more slots in this byte are free.
			 */
			keyix = i * NBBY;
			while (b & 1) {
		again:
				keyix++;
				b >>= 1;
			}
			/* XXX IEEE80211_KEY_XMIT | IEEE80211_KEY_RECV */
			if (isset(sc->sc_keymap, keyix + 32) ||
			    isset(sc->sc_keymap, keyix + 64) ||
			    isset(sc->sc_keymap, keyix + 32 + 64)) {
				/* full pair unavailable */
				/* XXX statistic */
				if (keyix == (i + 1) * NBBY) {
					/* no slots were appropriate, advance */
					continue;
				}
				goto again;
			}
			setbit(sc->sc_keymap, keyix);
			setbit(sc->sc_keymap, keyix + 64);
			setbit(sc->sc_keymap, keyix + 32);
			setbit(sc->sc_keymap, keyix + 32 + 64);
			DPRINTF(sc, ATH_DEBUG_KEYCACHE,
				"%s: key pair %u,%u %u,%u\n",
				__func__, keyix, keyix + 64,
				keyix + 32, keyix + 32 + 64);
			return keyix;
		}
	}
	DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s: out of pair space\n", __func__);
	return IEEE80211_KEYIX_NONE;
}

/*
 * Allocate tx/rx key slots for TKIP.  We allocate two slots for
 * each key, one for decrypt/encrypt and the other for the MIC.
 */
static ieee80211_keyix_t
key_alloc_pair(struct ath_softc *sc)
{
	u_int i;
	ieee80211_keyix_t keyix;

	KASSERT(!sc->sc_splitmic, ("key cache split"));
	/* XXX could optimize */
	for (i = 0; i < ARRAY_SIZE(sc->sc_keymap)/4; i++) {
		u_int8_t b = sc->sc_keymap[i];
		if (b != 0xff) {
			/*
			 * One or more slots in this byte are free.
			 */
			keyix = i * NBBY;
			while (b & 1) {
		again:
				keyix++;
				b >>= 1;
			}
			if (isset(sc->sc_keymap, keyix + 64)) {
				/* full pair unavailable */
				/* XXX statistic */
				if (keyix == (i + 1) * NBBY) {
					/* no slots were appropriate, advance */
					continue;
				}
				goto again;
			}
			setbit(sc->sc_keymap, keyix);
			setbit(sc->sc_keymap, keyix + 64);
			DPRINTF(sc, ATH_DEBUG_KEYCACHE,
				"%s: key pair %u,%u\n",
				__func__, keyix, keyix + 64);
			return keyix;
		}
	}
	DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s: out of pair space\n", __func__);
	return IEEE80211_KEYIX_NONE;
}

/*
 * Allocate a single key cache slot.
 */
static ieee80211_keyix_t
key_alloc_single(struct ath_softc *sc)
{
	u_int i;
	ieee80211_keyix_t keyix;

	/* XXX: try i, i + 32, i + 64, i + 32 + 64 to minimize key pair conflicts */
	for (i = 0; i < ARRAY_SIZE(sc->sc_keymap); i++) {
		u_int8_t b = sc->sc_keymap[i];
		if (b != 0xff) {
			/*
			 * One or more slots are free.
			 */
			keyix = i * NBBY;
			while (b & 1)
				keyix++, b >>= 1;
			setbit(sc->sc_keymap, keyix);
			DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s: key %u\n",
				__func__, keyix);
			return keyix;
		}
	}
	DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s: out of space\n", __func__);
	return IEEE80211_KEYIX_NONE;
}

/*
 * Allocate one or more key cache slots for a unicast key.  The
 * key itself is needed only to identify the cipher.  For hardware
 * TKIP with split cipher+MIC keys we allocate two key cache slot
 * pairs so that we can setup separate TX and RX MIC keys.  Note
 * that the MIC key for a TKIP key at slot i is assumed by the
 * hardware to be at slot i+64.  This limits TKIP keys to the first
 * 64 entries.
 */
static ieee80211_keyix_t
ath_key_alloc(struct ieee80211vap *vap, const struct ieee80211_key *k)
{
	struct net_device *dev = vap->iv_ic->ic_dev;
	struct ath_softc *sc = dev->priv;

	/*
	 * Group key allocation must be handled specially for
	 * parts that do not support multicast key cache search
	 * functionality.  For those parts the key id must match
	 * the h/w key index so lookups find the right key.  On
	 * parts w/ the key search facility we install the sender's
	 * MAC address (with the high bit set) and let the hardware
	 * find the key w/o using the key id.  This is preferred as
	 * it permits us to support multiple users for adhoc and/or
	 * multi-station operation.
	 */
	if ((k->wk_flags & IEEE80211_KEY_GROUP) && !sc->sc_mcastkey) {
		int i;
		ieee80211_keyix_t keyix = IEEE80211_KEYIX_NONE;

		for (i = 0; i < IEEE80211_WEP_NKID; i++) {
			if (k == &vap->iv_nw_keys[i]) {
				keyix = i;
				break;
			}
		}
		if (keyix == IEEE80211_KEYIX_NONE) {
			/* should not happen */
			DPRINTF(sc, ATH_DEBUG_KEYCACHE,
				"%s: bogus group key\n", __func__);
			return IEEE80211_KEYIX_NONE;
		}

		/* XXX: We pre-allocate the global keys so have no way 
		 * to check if they've already been allocated. */
		return keyix;
	}
	/*
	 * We allocate two pair for TKIP when using the h/w to do
	 * the MIC.  For everything else, including software crypto,
	 * we allocate a single entry.  Note that s/w crypto requires
	 * a pass-through slot on the 5211 and 5212.  The 5210 does
	 * not support pass-through cache entries and we map all
	 * those requests to slot 0.
	 *
	 * Allocate 1 pair of keys for WEP case. Make sure the key
	 * is not a shared-key.
	 */
	if (k->wk_flags & IEEE80211_KEY_SWCRYPT)
		return key_alloc_single(sc);
	else if (k->wk_cipher->ic_cipher == IEEE80211_CIPHER_TKIP &&
		(k->wk_flags & IEEE80211_KEY_SWMIC) == 0) {
		if (sc->sc_splitmic)
			return key_alloc_2pair(sc);
		else
			return key_alloc_pair(sc);
	} else
		return key_alloc_single(sc);
}

/*
 * Delete an entry in the key cache allocated by ath_key_alloc.
 */
static int
ath_key_delete(struct ieee80211vap *vap, const struct ieee80211_key *k,
				struct ieee80211_node *ninfo)
{
	struct net_device *dev = vap->iv_ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211_node *ni = NULL;
	const struct ieee80211_cipher *cip = k->wk_cipher;
	ieee80211_keyix_t keyix = k->wk_keyix;
	unsigned int rxkeyoff = 0;

	DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s: delete key %u\n", __func__, keyix);

	ath_hal_keyreset(ah, keyix);
	/*
	 * Check the key->node map and flush any ref.
	 */
	ni = sc->sc_keyixmap[keyix];
	if (ni != NULL) {
		ieee80211_unref_node(&ni);
		sc->sc_keyixmap[keyix] = NULL;
	}
	/*
	 * Handle split tx/rx keying required for TKIP with h/w MIC.
	 */
	if (cip->ic_cipher == IEEE80211_CIPHER_TKIP &&
	    (k->wk_flags & IEEE80211_KEY_SWMIC) == 0 && sc->sc_splitmic) {
		ath_hal_keyreset(ah, keyix + 32);	/* RX key */
		ni = sc->sc_keyixmap[keyix + 32];
		if (ni != NULL) {			/* as above... */
			ieee80211_unref_node(&ni);
			sc->sc_keyixmap[keyix + 32] = NULL;
		}
	}

	/* Remove receive key entry if one exists for static WEP case */
	if (ninfo != NULL) {
		rxkeyoff = ninfo->ni_rxkeyoff;
		if (rxkeyoff != 0) {
			ninfo->ni_rxkeyoff = 0;
			ath_hal_keyreset(ah, keyix + rxkeyoff);
			ni = sc->sc_keyixmap[keyix + rxkeyoff];
			if (ni != NULL) {	/* as above... */
				ieee80211_unref_node(&ni);
				sc->sc_keyixmap[keyix + rxkeyoff] = NULL;
			}
		}
	}

	if (keyix >= IEEE80211_WEP_NKID) {
		/*
		 * Don't touch keymap entries for global keys so
		 * they are never considered for dynamic allocation.
		 */
		clrbit(sc->sc_keymap, keyix);
		if (cip->ic_cipher == IEEE80211_CIPHER_TKIP &&
		    (k->wk_flags & IEEE80211_KEY_SWMIC) == 0) {
			clrbit(sc->sc_keymap, keyix + 64);	/* TX key MIC */
			if (sc->sc_splitmic) {
				/* +32 for RX key, +32+64 for RX key MIC */
				clrbit(sc->sc_keymap, keyix + 32);
				clrbit(sc->sc_keymap, keyix + 32 + 64);
			}
		}

		if (rxkeyoff != 0)
			clrbit(sc->sc_keymap, keyix + rxkeyoff);	/* RX Key */
	}
	return 1;
}

/*
 * Set the key cache contents for the specified key.  Key cache
 * slot(s) must already have been allocated by ath_key_alloc.
 */
static int
ath_key_set(struct ieee80211vap *vap, const struct ieee80211_key *k,
	const u_int8_t mac[IEEE80211_ADDR_LEN])
{
	struct net_device *dev = vap->iv_ic->ic_dev;
	struct ath_softc *sc = dev->priv;

	return ath_keyset(sc, k, mac, vap->iv_bss);
}

/*
 * Block/unblock tx+rx processing while a key change is done.
 * We assume the caller serializes key management operations
 * so we only need to worry about synchronization with other
 * uses that originate in the driver.
 */
static void
ath_key_update_begin(struct ieee80211vap *vap)
{
	struct net_device *dev = vap->iv_ic->ic_dev;
	struct ath_softc *sc = dev->priv;

	DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s:\n", __func__);
	/*
	 * When called from the rx tasklet we cannot use
	 * tasklet_disable because it will block waiting
	 * for us to complete execution.
	 *
	 * XXX Using in_softirq is not right since we might
	 * be called from other soft irq contexts than
	 * ath_rx_tasklet.
	 */
	if (!in_softirq())
		tasklet_disable(&sc->sc_rxtq);
	netif_stop_queue(dev);
}

static void
ath_key_update_end(struct ieee80211vap *vap)
{
	struct net_device *dev = vap->iv_ic->ic_dev;
	struct ath_softc *sc = dev->priv;

	DPRINTF(sc, ATH_DEBUG_KEYCACHE, "%s:\n", __func__);
	netif_start_queue(dev);
	if (!in_softirq())		/* NB: see above */
		tasklet_enable(&sc->sc_rxtq);
}

/*
 * Calculate the receive filter according to the
 * operating mode and state:
 *
 * o always accept unicast, broadcast, and multicast traffic
 * o maintain current state of phy error reception (the HAL
 *   may enable phy error frames for noise immunity work)
 * o probe request frames are accepted only when operating in
 *   hostap, adhoc, or monitor modes
 * o enable promiscuous mode according to the interface state
 * o accept beacons:
 *   - when operating in adhoc mode so the 802.11 layer creates
 *     node table entries for peers,
 *   - when operating in station mode for collecting rssi data when
 *     the station is otherwise quiet, or
 *   - when operating as a repeater so we see repeater-sta beacons
 *   - when scanning
 */
static u_int32_t
ath_calcrxfilter(struct ath_softc *sc)
{
#define	RX_FILTER_PRESERVE	(HAL_RX_FILTER_PHYERR | HAL_RX_FILTER_PHYRADAR)
	struct ieee80211com *ic = &sc->sc_ic;
	struct net_device *dev = ic->ic_dev;
	struct ath_hal *ah = sc->sc_ah;
	u_int32_t rfilt;

	rfilt = (ath_hal_getrxfilter(ah) & RX_FILTER_PRESERVE) |
		 HAL_RX_FILTER_UCAST | HAL_RX_FILTER_BCAST |
		 HAL_RX_FILTER_MCAST;
	if (ic->ic_opmode != IEEE80211_M_STA)
		rfilt |= HAL_RX_FILTER_PROBEREQ;
	if (ic->ic_opmode != IEEE80211_M_HOSTAP && (dev->flags & IFF_PROMISC))
		rfilt |= HAL_RX_FILTER_PROM;
	if (ic->ic_opmode == IEEE80211_M_STA ||
	    sc->sc_opmode == HAL_M_IBSS ||	/* NB: AHDEMO too */
	    (sc->sc_nostabeacons) || sc->sc_scanning)
		rfilt |= HAL_RX_FILTER_BEACON;
	if (sc->sc_nmonvaps > 0)
		rfilt |= (HAL_RX_FILTER_CONTROL | HAL_RX_FILTER_BEACON |
			  HAL_RX_FILTER_PROBEREQ | HAL_RX_FILTER_PROM);
	return rfilt;
#undef RX_FILTER_PRESERVE
}

/*
 * Merge multicast addresses from all VAPs to form the
 * hardware filter.  Ideally we should only inspect our
 * own list and the 802.11 layer would merge for us but
 * that's a bit difficult so for now we put the onus on
 * the driver.
 */
static void
ath_merge_mcast(struct ath_softc *sc, u_int32_t mfilt[2])
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211vap *vap;
	struct dev_mc_list *mc;
	u_int32_t val;
	u_int8_t pos;

	mfilt[0] = mfilt[1] = 0;
	/* XXX locking */
	TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) {
		struct net_device *dev = vap->iv_dev;
		for (mc = dev->mc_list; mc; mc = mc->next) {
			/* calculate XOR of eight 6-bit values */
			val = LE_READ_4(mc->dmi_addr + 0);
			pos = (val >> 18) ^ (val >> 12) ^ (val >> 6) ^ val;
			val = LE_READ_4(mc->dmi_addr + 3);
			pos ^= (val >> 18) ^ (val >> 12) ^ (val >> 6) ^ val;
			pos &= 0x3f;
			mfilt[pos / 32] |= (1 << (pos % 32));
		}
	}
}

static void
ath_mode_init(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	u_int32_t rfilt, mfilt[2];

	/* configure rx filter */
	rfilt = ath_calcrxfilter(sc);
	ath_hal_setrxfilter(ah, rfilt);

	/* configure bssid mask */
	if (sc->sc_hasbmask)
		ath_hal_setbssidmask(ah, sc->sc_bssidmask);

	/* configure operational mode */
	ath_hal_setopmode(ah);

	/* calculate and install multicast filter */
	if ((dev->flags & IFF_ALLMULTI) == 0)
		ath_merge_mcast(sc, mfilt);
	else
		mfilt[0] = mfilt[1] = ~0;
	ath_hal_setmcastfilter(ah, mfilt[0], mfilt[1]);
	DPRINTF(sc, ATH_DEBUG_STATE,
	    "%s: RX filter 0x%x, MC filter %08x:%08x\n",
	    __func__, rfilt, mfilt[0], mfilt[1]);
}

/*
 * Set the slot time based on the current setting.
 */
static void
ath_setslottime(struct ath_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;

	if (sc->sc_slottimeconf > 0) /* manual override */
		ath_hal_setslottime(ah, sc->sc_slottimeconf);
	else if (ic->ic_flags & IEEE80211_F_SHSLOT)
		ath_hal_setslottime(ah, HAL_SLOT_TIME_9);
	else
		ath_hal_setslottime(ah, HAL_SLOT_TIME_20);
	sc->sc_updateslot = OK;
}

/*
 * Callback from the 802.11 layer to update the
 * slot time based on the current setting.
 */
static void
ath_updateslot(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;

	/*
	 * When not coordinating the BSS, change the hardware
	 * immediately.  For other operation we defer the change
	 * until beacon updates have propagated to the stations.
	 */
	if (ic->ic_opmode == IEEE80211_M_HOSTAP)
		sc->sc_updateslot = UPDATE;
	else if (dev->flags & IFF_RUNNING)
		ath_setslottime(sc);
}

#ifdef ATH_SUPERG_DYNTURBO
/*
 * Dynamic turbo support.
 * XXX much of this could be moved up to the net80211 layer.
 */

/*
 * Configure dynamic turbo state on beacon setup.
 */
static void
ath_beacon_dturbo_config(struct ieee80211vap *vap, u_int32_t intval)
{
#define	IS_CAPABLE(vap) \
	(vap->iv_bss && (vap->iv_bss->ni_ath_flags & (IEEE80211_ATHC_TURBOP)) == \
		(IEEE80211_ATHC_TURBOP))
	struct ieee80211com *ic = vap->iv_ic;
	struct ath_softc *sc = ic->ic_dev->priv;

	if (ic->ic_opmode == IEEE80211_M_HOSTAP && IS_CAPABLE(vap)) {

		/* Dynamic Turbo is supported on this channel. */
		sc->sc_dturbo = 1;
		sc->sc_dturbo_tcount = 0;
		sc->sc_dturbo_switch = 0;
		sc->sc_ignore_ar = 0;

		/* Set the initial ATHC_BOOST capability. */
		if (ic->ic_bsschan->ic_flags & CHANNEL_TURBO)
			ic->ic_ath_cap |=  IEEE80211_ATHC_BOOST;
		else
			ic->ic_ath_cap &= ~IEEE80211_ATHC_BOOST;

		/*
		 * Calculate time & bandwidth thresholds
		 *
		 * sc_dturbo_base_tmin  :  ~70 seconds
		 * sc_dturbo_turbo_tmax : ~120 seconds
		 *
		 * NB: scale calculated values to account for staggered
		 *     beacon handling
		 */
		sc->sc_dturbo_base_tmin  = 70  * 1024 / ic->ic_lintval;
		sc->sc_dturbo_turbo_tmax = 120 * 1024 / ic->ic_lintval;
		sc->sc_dturbo_turbo_tmin = 5 * 1024 / ic->ic_lintval;
		/* convert the thresholds from BW/sec to BW/beacon period */
		sc->sc_dturbo_bw_base    = ATH_TURBO_DN_THRESH/(1024/ic->ic_lintval);
		sc->sc_dturbo_bw_turbo   = ATH_TURBO_UP_THRESH/(1024/ic->ic_lintval);
		/* time in hold state in number of beacon */
		sc->sc_dturbo_hold_max   = (ATH_TURBO_PERIOD_HOLD * 1024)/ic->ic_lintval;
	} else {
		sc->sc_dturbo = 0;
		ic->ic_ath_cap &= ~IEEE80211_ATHC_BOOST;
	}
#undef IS_CAPABLE
}

/*
 * Update dynamic turbo state at SWBA.  We assume we care
 * called only if dynamic turbo has been enabled (sc_turbo).
 */
static void
ath_beacon_dturbo_update(struct ieee80211vap *vap, int *needmark, u_int8_t dtim)
{
	struct ieee80211com *ic = vap->iv_ic;
	struct ath_softc *sc = ic->ic_dev->priv;
	u_int32_t bss_traffic;

	if (sc->sc_ignore_ar) {
		/*
		 * Ignore AR for this beacon; a dynamic turbo
		 * switch just happened and the information
		 * is invalid.  Notify AR support of the channel
		 * change.
		 */
		sc->sc_ignore_ar = 0;
#if 0 /* HAL 0.9.20.3 has no arEnable method */
		ath_hal_ar_enable(sc->sc_ah);
#endif
	}
	sc->sc_dturbo_tcount++;
	/*
	 * Calculate BSS traffic over the previous interval.
	 */
	bss_traffic = (sc->sc_devstats.tx_bytes + sc->sc_devstats.rx_bytes)
		    - sc->sc_dturbo_bytes;
	sc->sc_dturbo_bytes = sc->sc_devstats.tx_bytes
			    + sc->sc_devstats.rx_bytes;
	if (ic->ic_ath_cap & IEEE80211_ATHC_BOOST) {
		/* Before switching to base mode, make sure that the 
		 * conditions (low RSSI, low BW) to switch mode hold for some 
		 * time and time in turbo exceeds minimum turbo time. */
		if ((sc->sc_dturbo_tcount >= sc->sc_dturbo_turbo_tmin) &&
		    (sc->sc_dturbo_hold == 0) &&
		    (bss_traffic < sc->sc_dturbo_bw_base || !sc->sc_rate_recn_state)) {
			sc->sc_dturbo_hold = 1;
		} else {
			if (sc->sc_dturbo_hold &&
			   bss_traffic >= sc->sc_dturbo_bw_turbo && sc->sc_rate_recn_state) {
				/* out of hold state */
				sc->sc_dturbo_hold = 0;
				sc->sc_dturbo_hold_count = sc->sc_dturbo_hold_max;
			}
		}
		if (sc->sc_dturbo_hold && sc->sc_dturbo_hold_count)
			sc->sc_dturbo_hold_count--;
		/*
		 * Current Mode: Turbo (i.e. BOOST)
		 *
		 * Transition to base occurs when one of the following
		 * is true:
		 *    1. its a DTIM beacon.
		 *    2. Maximum time in BOOST has elapsed (120 secs).
		 *    3. Channel is marked with interference
		 *    4. Average BSS traffic falls below 4Mbps
		 *    5. RSSI cannot support at least 18 Mbps rate
		 * XXX do bw checks at true beacon interval?
		 */
		if (dtim &&
			(sc->sc_dturbo_tcount >= sc->sc_dturbo_turbo_tmax ||
			 ((vap->iv_bss->ni_ath_flags & IEEE80211_ATHC_AR) &&
			  (sc->sc_curchan.privFlags & CHANNEL_INTERFERENCE) &&
			  IEEE80211_IS_CHAN_2GHZ(ic->ic_curchan)) ||
			 !sc->sc_dturbo_hold_count)) {
			DPRINTF(sc, ATH_DEBUG_TURBO, "%s: Leaving turbo\n",
				__func__);
			ic->ic_ath_cap &= ~IEEE80211_ATHC_BOOST;
			vap->iv_bss->ni_ath_flags &= ~IEEE80211_ATHC_BOOST;
			sc->sc_dturbo_tcount = 0;
			sc->sc_dturbo_switch = 1;
		}
	} else {
		/*
		 * Current Mode: BASE
		 *
		 * Transition to Turbo (i.e. BOOST) when all of the
		 * following are true:
		 *
		 * 1. its a DTIM beacon.
		 * 2. Dwell time at base has exceeded minimum (70 secs)
		 * 3. Only DT-capable stations are associated
		 * 4. Channel is marked interference-free.
		 * 5. BSS data traffic averages at least 6Mbps
		 * 6. RSSI is good enough to support 36Mbps
		 * XXX do bw+rssi checks at true beacon interval?
		 */
		if (dtim &&
			(sc->sc_dturbo_tcount >= sc->sc_dturbo_base_tmin &&
			 (ic->ic_dt_sta_assoc != 0 &&
			  ic->ic_sta_assoc == ic->ic_dt_sta_assoc) &&
			 ((vap->iv_bss->ni_ath_flags & IEEE80211_ATHC_AR) == 0 ||
			  (sc->sc_curchan.privFlags & CHANNEL_INTERFERENCE) == 0) &&
			 bss_traffic >= sc->sc_dturbo_bw_turbo &&
			 sc->sc_rate_recn_state)) {
			DPRINTF(sc, ATH_DEBUG_TURBO, "%s: Entering turbo\n",
				__func__);
			ic->ic_ath_cap |= IEEE80211_ATHC_BOOST;
			vap->iv_bss->ni_ath_flags |= IEEE80211_ATHC_BOOST;
			sc->sc_dturbo_tcount = 0;
			sc->sc_dturbo_switch = 1;
			sc->sc_dturbo_hold = 0;
			sc->sc_dturbo_hold_count = sc->sc_dturbo_hold_max;
		}
	}
}


static int
ath_check_beacon_done(struct ath_softc *sc)
{
	struct ieee80211vap *vap = NULL;
	struct ath_vap *avp;
	struct ath_buf *bf;
	struct sk_buff *skb;
	struct ath_desc *ds;
	struct ath_tx_status *ts;
	struct ath_hal *ah = sc->sc_ah;
	unsigned int slot;

	/*
	 * check if the last beacon went out with the mode change flag set.
	 */
	for (slot = 0; slot < ATH_BCBUF; slot++) {
		if (sc->sc_bslot[slot]) {
			vap = sc->sc_bslot[slot];
			break;
		}
	}
	if (!vap)
		 return 0;
	avp = ATH_VAP(vap);
	bf = avp->av_bcbuf;
	skb = bf->bf_skb;
	ds = bf->bf_desc;
	ts = &bf->bf_dsstatus.ds_txstat;

	return (ath_hal_txprocdesc(ah, ds, ts) != HAL_EINPROGRESS);

}

/*
 * Effect a turbo mode switch when operating in dynamic
 * turbo mode. wait for beacon to go out before switching.
 */
static void
ath_turbo_switch_mode(unsigned long data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	unsigned int newflags;

	KASSERT(ic->ic_opmode == IEEE80211_M_HOSTAP,
		("unexpected operating mode %d", ic->ic_opmode));

	DPRINTF(sc, ATH_DEBUG_STATE, "dynamic turbo switch to %s mode\n",
		ic->ic_ath_cap & IEEE80211_ATHC_BOOST ? "turbo" : "base");

	if (!ath_check_beacon_done(sc)) {
		/*
		 * beacon did not go out. reschedule tasklet.
		 */
		mod_timer(&sc->sc_dturbo_switch_mode, jiffies + msecs_to_jiffies(2));
		return;
	}

	/* TBD: DTIM adjustments, delay CAB queue tx until after transmit */
	newflags = ic->ic_bsschan->ic_flags;
	if (ic->ic_ath_cap & IEEE80211_ATHC_BOOST) {
		if (IEEE80211_IS_CHAN_2GHZ(ic->ic_bsschan)) {
			/*
			 * Ignore AR next beacon. the AR detection
			 * code detects the traffic in normal channel
			 * from stations during transition delays
			 * between AP and station.
			 */
			sc->sc_ignore_ar = 1;
#if 0 /* HAL 0.9.20.3 has no arDisable method */
			ath_hal_ar_disable(sc->sc_ah);
#endif
		}
		newflags |= IEEE80211_CHAN_TURBO;
	} else
		newflags &= ~IEEE80211_CHAN_TURBO;
	ieee80211_dturbo_switch(ic, newflags);
	/* XXX ieee80211_reset_erp? */
}
#endif /* ATH_SUPERG_DYNTURBO */

/*
 * Setup a h/w transmit queue for beacons.
 */
static int
ath_beaconq_setup(struct ath_softc *sc)
{
	HAL_TXQ_INFO qi;
	struct ath_txq *txq;
	int qnum;

	memset(&qi, 0, sizeof(qi));
	qi.tqi_aifs = 1;
	qi.tqi_cwmin = 0;
	qi.tqi_cwmax = 0;
#ifdef ATH_SUPERG_DYNTURBO
	qi.tqi_qflags = HAL_TXQ_TXDESCINT_ENABLE;
#endif
	/* NB: don't enable any interrupts */
	qnum = ath_hal_setuptxqueue(sc->sc_ah, HAL_TX_QUEUE_BEACON, &qi);
	txq = &sc->sc_txq[qnum];
	memset(txq, 0, sizeof(struct ath_txq));
	txq->axq_qnum		= qnum;
	STAILQ_INIT(&txq->axq_q);
	ATH_TXQ_LOCK_INIT(txq);
	TAILQ_INIT(&txq->axq_stageq);
	return qnum;
}

/*
 * Configure IFS parameter for the beacon queue.
 */
static int
ath_beaconq_config(struct ath_softc *sc)
{
#define	ATH_EXPONENT_TO_VALUE(v)	((1<<v)-1)
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	HAL_TXQ_INFO qi;

	ath_hal_gettxqueueprops(ah, sc->sc_bhalq, &qi);
	if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
		/*
		 * Always burst out beacon and CAB traffic.
		 */
		qi.tqi_aifs = 1;
		qi.tqi_cwmin = 0;
		qi.tqi_cwmax = 0;
	} else {
		struct wmeParams *wmep =
			&ic->ic_wme.wme_chanParams.cap_wmeParams[WME_AC_BE];
		/*
		 * Adhoc mode; important thing is to use 2x cwmin.
		 */
		qi.tqi_aifs = wmep->wmep_aifsn;
		qi.tqi_cwmin = 2 * ATH_EXPONENT_TO_VALUE(wmep->wmep_logcwmin);
		qi.tqi_cwmax = ATH_EXPONENT_TO_VALUE(wmep->wmep_logcwmax);
	}

	DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
		"%s: ath_hal_settxqueuepropos tqi_aifs:%d tqi_cwmin:%d tqi_cwmax:%d\n",
		__func__, qi.tqi_aifs, qi.tqi_cwmin, qi.tqi_cwmax);
	if (!ath_hal_settxqueueprops(ah, sc->sc_bhalq, &qi)) {
		printk("%s: unable to update h/w beacon queue parameters\n",
			DEV_NAME(sc->sc_dev));
		return 0;
	} else {
		DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"%s: ath_hal_resettxqueue sc_bhalq:%d\n",
			__func__, sc->sc_bhalq);
		ath_hal_resettxqueue(ah, sc->sc_bhalq);	/* push to h/w */
		return 1;
	}
#undef ATH_EXPONENT_TO_VALUE
}

/*
 * Allocate and setup an initial beacon frame.
 *
 * Context: softIRQ
 */
static int
ath_beacon_alloc(struct ath_softc *sc, struct ieee80211_node *ni)
{
	struct ath_vap *avp = ATH_VAP(ni->ni_vap);
	struct ieee80211_frame *wh;
	struct ath_buf *bf;
	struct sk_buff *skb;

	DPRINTF(sc, ATH_DEBUG_BEACON, "%s: in\n", __func__);

	/*
	 * release the previous beacon's skb if it already exists.
	 */
	bf = avp->av_bcbuf;
	cleanup_ath_buf(sc, bf, BUS_DMA_TODEVICE);

	/*
	 * NB: the beacon data buffer must be 32-bit aligned;
	 * we assume the mbuf routines will return us something
	 * with this alignment (perhaps should assert).
	 */
	skb = ieee80211_beacon_alloc(ni, &avp->av_boff);
	if (skb == NULL) {
		DPRINTF(sc, ATH_DEBUG_BEACON, "%s: cannot get sk_buff\n",
			__func__);
		sc->sc_stats.ast_be_nobuf++;
		return -ENOMEM;
	}

	/*
	 * Calculate a TSF adjustment factor required for
	 * staggered beacons.  Note that we assume the format
	 * of the beacon frame leaves the tstamp field immediately
	 * following the header.
	 */
	if (sc->sc_stagbeacons && avp->av_bslot > 0) {
		uint64_t tuadjust;
		__le64 tsfadjust;
		/*
		 * The beacon interval is in TUs; the TSF in usecs.
		 * We figure out how many TUs to add to align the
		 * timestamp then convert to TSF units and handle
		 * byte swapping before writing it in the frame.
		 * The hardware will then add this each time a beacon
		 * frame is sent.  Note that we align VAPs 1..N
		 * and leave VAP 0 untouched.  This means VAP 0
		 * has a timestamp in one beacon interval while the
		 * others get a timestamp aligned to the next interval.
		 */
		tuadjust = (ni->ni_intval * (ATH_BCBUF - avp->av_bslot)) / ATH_BCBUF;
		tsfadjust = cpu_to_le64(tuadjust << 10);	/* TU->TSF */

		DPRINTF(sc, ATH_DEBUG_BEACON,
			"%s: %s beacons, bslot %d intval %u tsfadjust(Kus) %llu\n",
			__func__, sc->sc_stagbeacons ? "stagger" : "burst",
			avp->av_bslot, ni->ni_intval, (unsigned long long) tuadjust);

		wh = (struct ieee80211_frame *)skb->data;
		memcpy(&wh[1], &tsfadjust, sizeof(tsfadjust));
	}

	bf->bf_node = ieee80211_ref_node(ni);
	bf->bf_skbaddr = bus_map_single(sc->sc_bdev,
		skb->data, skb->len, BUS_DMA_TODEVICE);
	bf->bf_skb = skb;

	DPRINTF(sc, ATH_DEBUG_BEACON, "%s: out\n", __func__);

	return 0;
}

/*
 * Setup the beacon frame for transmit.
 *
 * If the part supports the ``virtual EOL'' mechanism in the xmit descriptor,
 * we can use it to periodically send the beacon frame w/o having to do
 * setup. Otherwise we have to explicitly submit the beacon frame at each SWBA
 * interrupt. In order to minimize change, we always use the SWBA interrupt
 * mechanism.
 */
static void
ath_beacon_setup(struct ath_softc *sc, struct ath_buf *bf)
{
#define	USE_SHPREAMBLE(_ic) \
	(((_ic)->ic_flags & (IEEE80211_F_SHPREAMBLE | IEEE80211_F_USEBARKER))\
		== IEEE80211_F_SHPREAMBLE)
	struct ieee80211com *ic = bf->bf_node->ni_ic;
	struct sk_buff *skb = bf->bf_skb;
	struct ath_hal *ah = sc->sc_ah;
	struct ath_desc *ds;
	unsigned int flags;
	int antenna = sc->sc_txantenna;
	const HAL_RATE_TABLE *rt;
	u_int8_t rix, rate;
	unsigned int ctsrate = 0, ctsduration = 0;

	DPRINTF(sc, ATH_DEBUG_BEACON_PROC, "%s: skbaddr %p len %u\n",
		__func__, skb, skb->len);

	/* setup descriptors */
	ds = bf->bf_desc;

	flags = HAL_TXDESC_NOACK;
#ifdef ATH_SUPERG_DYNTURBO
	if (sc->sc_dturbo_switch)
		flags |= HAL_TXDESC_INTREQ;
#endif

	if (ic->ic_opmode == IEEE80211_M_IBSS && sc->sc_hasveol) {
		ds->ds_link = bf->bf_daddr;	/* self-linked */
		flags |= HAL_TXDESC_VEOL;
		/*
		 * Let hardware handle antenna switching if txantenna is not set
		 */
	} else {
		ds->ds_link = 0;
		/*
		 * Switch antenna every beacon if txantenna is not set
		 * Should only switch every beacon period, not for all
		 * SWBAs
		 * XXX: assumes two antennae
		 */
		if (antenna == 0) {
			if (sc->sc_stagbeacons)
				antenna = ((sc->sc_stats.ast_be_xmit / 
							sc->sc_nbcnvaps) & 1 ? 2 : 1);
			else
				antenna = (sc->sc_stats.ast_be_xmit & 1 ? 2 : 1);
		}
	}

	ds->ds_data = bf->bf_skbaddr;
	/*
	 * Calculate rate code.
	 * XXX everything at min xmit rate
	 */
	rix = sc->sc_minrateix;
	rt = sc->sc_currates;
	rate = rt->info[rix].rateCode;
	if (USE_SHPREAMBLE(ic))
		rate |= rt->info[rix].shortPreamble;
#ifdef ATH_SUPERG_XR
	if (bf->bf_node->ni_vap->iv_flags & IEEE80211_F_XR) {
		u_int8_t cix;
		unsigned int pktlen;
		pktlen = skb->len + IEEE80211_CRC_LEN;
		cix = rt->info[sc->sc_protrix].controlRate;
		/* for XR VAP use different RTSCTS rates and calculate duration */
		ctsrate = rt->info[cix].rateCode;
		if (USE_SHPREAMBLE(ic))
			ctsrate |= rt->info[cix].shortPreamble;
		flags |= HAL_TXDESC_CTSENA;
		rt = sc->sc_xr_rates;
		ctsduration = ath_hal_computetxtime(ah, rt, pktlen,
			IEEE80211_XR_DEFAULT_RATE_INDEX, AH_FALSE);
		rate = rt->info[IEEE80211_XR_DEFAULT_RATE_INDEX].rateCode;
	}
#endif
	ath_hal_setuptxdesc(ah, ds,
		skb->len + IEEE80211_CRC_LEN,	/* frame length */
		sizeof(struct ieee80211_frame), /* header length */
		HAL_PKT_TYPE_BEACON,		/* Atheros packet type */
		bf->bf_node->ni_txpower,        /* txpower XXX */
		rate, 1,			/* series 0 rate/tries */
		HAL_TXKEYIX_INVALID,		/* no encryption */
		antenna,			/* antenna mode */
		flags,				/* no ack, veol for beacons */
		ctsrate,			/* rts/cts rate */
		ctsduration,			/* rts/cts duration */
		0,				/* comp icv len */
		0,				/* comp iv len */
		ATH_COMP_PROC_NO_COMP_NO_CCS	/* comp scheme */
	);

	/* NB: beacon's BufLen must be a multiple of 4 bytes */
	ath_hal_filltxdesc(ah, ds,
		roundup(skb->len, 4),	/* buffer length */
		AH_TRUE,		/* first segment */
		AH_TRUE,		/* last segment */
		ds			/* first descriptor */
	);

	/* NB: The desc swap function becomes void,
	 * if descriptor swapping is not enabled
	 */
	ath_desc_swap(ds);
#undef USE_SHPREAMBLE
}

/*
 * Generate beacon frame and queue cab data for a VAP.
 */
static struct ath_buf *
ath_beacon_generate(struct ath_softc *sc, struct ieee80211vap *vap, int *needmark)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ath_buf *bf;
	struct ath_vap *avp;
	struct sk_buff *skb;
	unsigned int curlen, ncabq;
	u_int8_t tim_bitctl;

	if (vap->iv_state != IEEE80211_S_RUN) {
		DPRINTF(sc, ATH_DEBUG_BEACON_PROC, "%s: skip VAP in %s state\n",
			__func__, ieee80211_state_name[vap->iv_state]);
		return NULL;
	}
#ifdef ATH_SUPERG_XR
	if (vap->iv_flags & IEEE80211_F_XR) {
		vap->iv_xrbcnwait++;
		/* wait for XR_BEACON_FACTOR times before sending the beacon */
		if (vap->iv_xrbcnwait < IEEE80211_XR_BEACON_FACTOR)
			return NULL;
		vap->iv_xrbcnwait = 0;
	}
#endif
	avp = ATH_VAP(vap);
	if (avp->av_bcbuf == NULL) {
		DPRINTF(sc, ATH_DEBUG_ANY, "%s: avp=%p av_bcbuf=%p\n",
			 __func__, avp, avp->av_bcbuf);
		return NULL;
	}
	bf = avp->av_bcbuf;

#ifdef ATH_SUPERG_DYNTURBO
	/*
	 * If we are using dynamic turbo, update the
	 * capability info and arrange for a mode change
	 * if needed.
	 */
	if (sc->sc_dturbo) {
		u_int8_t dtim;
		dtim = ((avp->av_boff.bo_tim[2] == 1) ||
			(avp->av_boff.bo_tim[3] == 1));
		ath_beacon_dturbo_update(vap, needmark, dtim);
	}
#endif
	/*
	 * Update dynamic beacon contents.  If this returns
	 * non-zero then we need to remap the memory because
	 * the beacon frame changed size (probably because
	 * of the TIM bitmap).
	 */
	skb = bf->bf_skb;
	curlen = skb->len;
	ncabq = avp->av_mcastq.axq_depth;
	if (ieee80211_beacon_update(bf->bf_node, &avp->av_boff, skb, ncabq)) {
		bus_unmap_single(sc->sc_bdev,
			bf->bf_skbaddr, curlen, BUS_DMA_TODEVICE);
		bf->bf_skbaddr = 0;

		bf->bf_skbaddr = bus_map_single(sc->sc_bdev,
			skb->data, skb->len, BUS_DMA_TODEVICE);
	}

	/*
	 * if the CABQ traffic from previous DTIM is pending and the current
	 * beacon is also a DTIM.
	 *  1) if there is only one VAP let the cab traffic continue.
	 *  2) if there are more than one VAP and we are using staggered
	 *     beacons, then drain the cabq by dropping all the frames in
	 *     the cabq so that the current VAPs CAB traffic can be scheduled.
	 * XXX: Need to handle the last MORE_DATA bit here.
	 */
	tim_bitctl = ((struct ieee80211_tim_ie *)avp->av_boff.bo_tim)->tim_bitctl;
	if (ncabq && (tim_bitctl & BITCTL_BUFD_MCAST) && sc->sc_cabq->axq_depth) {
		if (sc->sc_nvaps > 1 && sc->sc_stagbeacons) {
			ath_tx_draintxq(sc, sc->sc_cabq);
			DPRINTF(sc, ATH_DEBUG_BEACON,
				"%s: flush previous cabq traffic\n", __func__);
		}
	}

	/*
	 * Construct tx descriptor.
	 */
	ath_beacon_setup(sc, bf);

	bus_dma_sync_single(sc->sc_bdev,
		bf->bf_skbaddr, bf->bf_skb->len, BUS_DMA_TODEVICE);

	/*
	 * Enable the CAB queue before the beacon queue to
	 * ensure cab frames are triggered by this beacon.
	 * We only set BITCTL_BUFD_MCAST bit when its DTIM */
	if (tim_bitctl & BITCTL_BUFD_MCAST) {
		struct ath_txq *cabq = sc->sc_cabq;
		struct ath_buf *bfmcast;
		/*
		 * Move everything from the VAPs mcast queue
		 * to the hardware cab queue.
		 */
		ATH_TXQ_LOCK_IRQ(&avp->av_mcastq);
		ATH_TXQ_LOCK_IRQ_INSIDE(cabq);
		bfmcast = STAILQ_FIRST(&avp->av_mcastq.axq_q);
		/* link the descriptors */
		if (cabq->axq_link == NULL)
			ath_hal_puttxbuf(ah, cabq->axq_qnum, bfmcast->bf_daddr);
		else {
#ifdef AH_NEED_DESC_SWAP
			*cabq->axq_link = cpu_to_le32(bfmcast->bf_daddr);
#else
			*cabq->axq_link = bfmcast->bf_daddr;
#endif
		}

		/* Set the MORE_DATA bit for each packet except the last one */
		STAILQ_FOREACH(bfmcast, &avp->av_mcastq.axq_q, bf_list) {
			if (bfmcast != STAILQ_LAST(&avp->av_mcastq.axq_q, 
						ath_buf, bf_list))
				((struct ieee80211_frame *)
				 bfmcast->bf_skb->data)->i_fc[1] |= 
					IEEE80211_FC1_MORE_DATA;
		}

		/* append the private VAP mcast list to the cabq */
		ATH_TXQ_MOVE_MCASTQ(&avp->av_mcastq, cabq);
		/* NB: gated by beacon so safe to start here */
		if (cabq->axq_depth > 0) {
			if (!ath_hal_txstart(ah, cabq->axq_qnum)) {
				DPRINTF(sc, ATH_DEBUG_TX_PROC,
					"%s: failed to start CABQ\n",
					__func__);
			}
		}
		ATH_TXQ_UNLOCK_IRQ_INSIDE(cabq);
		ATH_TXQ_UNLOCK_IRQ(&avp->av_mcastq);
	}

	return bf;
}

/*
 * Transmit one or more beacon frames at SWBA.  Dynamic
 * updates to the frame contents are done as needed and
 * the slot time is also adjusted based on current state.
 */
static void
ath_beacon_send(struct ath_softc *sc, int *needmark)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211vap *vap;
	struct ath_buf *bf;
	unsigned int slot;
	u_int32_t bfaddr = 0;
	u_int32_t n_beacon;


	/*
	 * Check if the previous beacon has gone out.  If
	 * not don't try to post another, skip this period
	 * and wait for the next.  Missed beacons indicate
	 * a problem and should not occur.  If we miss too
	 * many consecutive beacons reset the device.
	 */
	if ((n_beacon = ath_hal_numtxpending(ah, sc->sc_bhalq)) != 0 ) {
		sc->sc_bmisscount++;
		/* XXX: 802.11h needs the chanchange IE countdown decremented.
		 *      We should consider adding a net80211 call to indicate
		 *      a beacon miss so appropriate action could be taken
		 *      (in that layer).
		 */
		DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"%s: missed %u consecutive beacons (n_beacon=%u)\n",
			__func__, sc->sc_bmisscount, n_beacon);
		if (sc->sc_bmisscount > BSTUCK_THRESH)
			ATH_SCHEDULE_TQUEUE(&sc->sc_bstucktq, needmark);
		return;
	}
	if (sc->sc_bmisscount != 0) {
		DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"%s: resume beacon xmit after %u misses\n",
			__func__, sc->sc_bmisscount);
		sc->sc_bmisscount = 0;
	}

	/*
	 * Generate beacon frames.  If we are sending frames
	 * staggered then calculate the slot for this frame based
	 * on the tsf to safeguard against missing an swba.
	 * Otherwise we are bursting all frames together and need
	 * to generate a frame for each VAP that is up and running.
	 */
	if (sc->sc_stagbeacons) {		/* staggered beacons */
		struct ieee80211com *ic = &sc->sc_ic;
		u_int64_t tsf;
		u_int32_t tsftu;

		tsf = ath_hal_gettsf64(ah);
		tsftu = tsf >> 10; /* NB: 64 -> 32: See note far above. */
		slot = ((tsftu % ic->ic_lintval) * ATH_BCBUF) / ic->ic_lintval;
		vap = sc->sc_bslot[(slot + 1) % ATH_BCBUF];
		DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"%s: slot %d [tsf %llu tsftu %llu intval %u] vap %p\n",
			__func__, slot, (unsigned long long)tsf, 
			(unsigned long long)tsftu, ic->ic_lintval, vap);
		bfaddr = 0;
		if (vap != NULL) {
			bf = ath_beacon_generate(sc, vap, needmark);
			if (bf != NULL)
				bfaddr = bf->bf_daddr;
		}
	} else {				/* burst'd beacons */
		u_int32_t *bflink = NULL;

		/* XXX: rotate/randomize order? */
		for (slot = 0; slot < ATH_BCBUF; slot++) {
			if ((vap = sc->sc_bslot[slot]) != NULL) {
				if ((bf = ath_beacon_generate(
						sc, vap, 
						needmark)) != NULL) {
					if (bflink != NULL)
#ifdef AH_NEED_DESC_SWAP
						*bflink = cpu_to_le32(bf->bf_daddr);
#else
						*bflink = bf->bf_daddr;
#endif
					else /* For the first bf, save bf_addr for later */
						bfaddr = bf->bf_daddr;

					bflink = &bf->bf_desc->ds_link;
				}
			}
		}
		if (bflink != NULL)
			*bflink = 0;		/* link of last frame */

		if(!bfaddr) {
			DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
				"%s: bursted beacons failed to set bfaddr!!\n", __func__);
		}

	}

	/*
	 * Handle slot time change when a non-ERP station joins/leaves
	 * an 11g network.  The 802.11 layer notifies us via callback,
	 * we mark updateslot, then wait one beacon before effecting
	 * the change.  This gives associated stations at least one
	 * beacon interval to note the state change.
	 *
	 * NB: The slot time change state machine is clocked according
	 *     to whether we are bursting or staggering beacons.  We
	 *     recognize the request to update and record the current
	 *     slot then don't transition until that slot is reached
	 *     again.  If we miss a beacon for that slot then we'll be
	 *     slow to transition but we'll be sure at least one beacon
	 *     interval has passed.  When bursting slot is always left
	 *     set to ATH_BCBUF so this check is a no-op.
	 */
	/* XXX locking */
	if (sc->sc_updateslot == UPDATE) {
		sc->sc_updateslot = COMMIT;	/* commit next beacon */
		sc->sc_slotupdate = slot;
	} else if ((sc->sc_updateslot == COMMIT) && (sc->sc_slotupdate == slot))
		ath_setslottime(sc);		/* commit change to hardware */

	if ((!sc->sc_stagbeacons || slot == 0) && (!sc->sc_diversity)) {
		unsigned int otherant;
		/*
		 * Check recent per-antenna transmit statistics and flip
		 * the default rx antenna if noticeably more frames went out
		 * on the non-default antenna.  Only do this if rx diversity
		 * is off.
		 * XXX assumes 2 antennae
		 */
		otherant = sc->sc_defant & 1 ? 2 : 1;
		if (sc->sc_ant_tx[otherant] > sc->sc_ant_tx[sc->sc_defant] + 
				ATH_ANTENNA_DIFF) {
			DPRINTF(sc, ATH_DEBUG_BEACON,
				"%s: flip defant to %u, %u > %u\n",
				__func__, otherant, sc->sc_ant_tx[otherant],
				sc->sc_ant_tx[sc->sc_defant]);
			ath_setdefantenna(sc, otherant);
		}
		sc->sc_ant_tx[1] = sc->sc_ant_tx[2] = 0;
	}

	if (bfaddr != 0) {
		/*
		 * Stop any current DMA and put the new frame(s) on the queue.
		 * This should never fail since we check above that no frames
		 * are still pending on the queue.
		 */
		DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"%s: ath_hal_stoptxdma sc_bhalq:%d\n",
			__func__, sc->sc_bhalq);
		if (!ath_hal_stoptxdma(ah, sc->sc_bhalq)) {
			DPRINTF(sc, ATH_DEBUG_ANY,
				"%s: beacon queue %u did not stop?\n",
				__func__, sc->sc_bhalq);
			/* NB: the HAL still stops DMA, so proceed */
		}
		/* NB: cabq traffic should already be queued and primed */
		DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"%s: ath_hal_puttxbuf sc_bhalq:%d bfaddr:%x\n",
			__func__, sc->sc_bhalq, bfaddr);
		ath_hal_puttxbuf(ah, sc->sc_bhalq, bfaddr);

		DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"%s: ath_hal_txstart sc_bhalq:%d\n",
			__func__, sc->sc_bhalq);
		ath_hal_txstart(ah, sc->sc_bhalq);

		sc->sc_stats.ast_be_xmit++;		/* XXX per-VAP? */
	}
}

/*
 * Reset the hardware after detecting beacons have stopped.
 */
static void
ath_bstuck_tasklet(TQUEUE_ARG data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ath_softc *sc = dev->priv;
	/*
	 * XXX:if the bmisscount is cleared while the
	 *     tasklet execution is pending, the following
	 *     check will be true, in which case return
	 *     without resetting the driver.
	 */
	if (sc->sc_bmisscount <= BSTUCK_THRESH)
		return;
	printk("%s: stuck beacon; resetting (bmiss count %u)\n",
		DEV_NAME(dev), sc->sc_bmisscount);
	ath_reset(dev);
}


/*
 * Startup beacon transmission for adhoc mode when
 * they are sent entirely by the hardware using the
 * self-linked descriptor + veol trick.
 */
static void
ath_beacon_config(struct ath_softc *sc, struct ieee80211vap *vap)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ath_buf *bf;
	struct ath_vap *avp;
	struct sk_buff *skb;

	avp = ATH_VAP(vap);
	if (avp == NULL || avp->av_bcbuf == NULL) {
		DPRINTF(sc, ATH_DEBUG_ANY, "%s: avp=%p av_bcbuf=%p\n",
			 __func__, avp, avp != NULL ? avp->av_bcbuf : NULL);
		return;
	}
	bf = avp->av_bcbuf;

	ath_beacon_update_timers(sc, vap);

	/*
	 * Update dynamic beacon contents.  If this returns
	 * non-zero then we need to remap the memory because
	 * the beacon frame changed size (probably because
	 * of the TIM bitmap).
	 */
	skb = bf->bf_skb;
	if (ieee80211_beacon_update(bf->bf_node, &avp->av_boff, skb, 0)) {
		bus_unmap_single(sc->sc_bdev,
			bf->bf_skbaddr, bf->bf_skb->len, BUS_DMA_TODEVICE);
		bf->bf_skbaddr = bus_map_single(sc->sc_bdev,
			skb->data, skb->len, BUS_DMA_TODEVICE);
	}

	/*
	 * Construct tx descriptor.
	 */
	ath_beacon_setup(sc, bf);
	ath_beaconq_config(sc);

	bus_dma_sync_single(sc->sc_bdev,
		bf->bf_skbaddr, bf->bf_skb->len, BUS_DMA_TODEVICE);

	/* NB: caller is known to have already stopped tx DMA */
	ath_hal_puttxbuf(ah, sc->sc_bhalq, bf->bf_daddr);
	ath_hal_txstart(ah, sc->sc_bhalq);
	DPRINTF(sc, ATH_DEBUG_BEACON_PROC, "%s: TXDP%u = %llx (%p)\n", __func__,
		sc->sc_bhalq, ito64(bf->bf_daddr), bf->bf_desc);
}


/*
 * Reclaim beacon resources and return buffer to the pool.
 */
static void
ath_beacon_return(struct ath_softc *sc, struct ath_buf *bf)
{
	cleanup_ath_buf(sc, bf, BUS_DMA_TODEVICE);
	STAILQ_INSERT_TAIL(&sc->sc_bbuf, bf, bf_list);
}

/*
 * Reclaim all beacon resources.
 */
static void
ath_beacon_free(struct ath_softc *sc)
{
	struct ath_buf *bf;
	STAILQ_FOREACH(bf, &sc->sc_bbuf, bf_list)
		cleanup_ath_buf(sc, bf, BUS_DMA_TODEVICE);
}

/*
 * Configure the beacon and sleep timers.
 *
 * When operating as an AP/IBSS this resets the TSF and sets up the hardware to
 * notify us when we need to issue beacons.
 *
 * When operating in station mode this sets up the beacon timers according to
 * the timestamp of the last received beacon and the current TSF, configures
 * PCF and DTIM handling, programs the sleep registers so the hardware will
 * wake up in time to receive beacons, and configures the beacon miss handling
 * so we'll receive a BMISS interrupt when we stop seeing beacons from the AP
 * we've associated with.
 *
 * Note : TBTT is Target Beacon Transmission Time (see IEEE 802.11-1999: 4 &
 * 11.2.1.3).
 *
 * Note: TSF is right shifter by 10 and then put into a 32-bit int, which will 
 * truncate. This does not affect the calculation as long as no more than one 
 * overflow/wraparound occurs between beacons. This is not going to happen as
 * (2^(32 + 10 - 1) - 1)us is a really long time.
 */
static void
ath_beacon_update_timers(struct ath_softc *sc, struct ieee80211vap *vap)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211_node *ni;
	u_int64_t tsf, hw_tsf;
	u_int32_t tsftu, hw_tsftu;
	u_int32_t intval, nexttbtt = 0;
	int reset_tsf = 0;

	if (vap == NULL)
		vap = TAILQ_FIRST(&ic->ic_vaps);   /* XXX */

	ni = vap->iv_bss;

	hw_tsf = ath_hal_gettsf64(ah);
	tsf = le64_to_cpu(ni->ni_tstamp.tsf);
	hw_tsftu = hw_tsf >> 10;
	tsftu = tsf >> 10; /* NB: 64 -> 32. See note above. */

	/* We should reset hw TSF only once, so we increment
	 * ni_tstamp.tsf to avoid resetting the hw TSF multiple
	 * times */
	if (tsf == 0) {
		reset_tsf = 1;
		ni->ni_tstamp.tsf = cpu_to_le64(1);
	}

	/* XXX: Conditionalize multi-bss support? */
	if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
		/* For multi-bss ap support beacons are either staggered
		 * evenly over N slots or burst together.  For the former
		 * arrange for the SWBA to be delivered for each slot.
		 * Slots that are not occupied will generate nothing. */
		/* NB: the beacon interval is kept internally in TUs */
		intval = ic->ic_lintval & HAL_BEACON_PERIOD;
		if (sc->sc_stagbeacons)
			intval /= ATH_BCBUF;	/* for staggered beacons */
		if ((sc->sc_nostabeacons) &&
		    (vap->iv_opmode == IEEE80211_M_HOSTAP))
			reset_tsf = 1;
	} else
		intval = ni->ni_intval & HAL_BEACON_PERIOD;

#define	FUDGE	3
	sc->sc_syncbeacon = 0;

	if (reset_tsf) {
		/* We just created the interface and TSF will be reset to
		 * zero, so next beacon will be sent at the next intval
		 * time */
		nexttbtt = intval;
	} else if (intval) {	/* NB: can be 0 for monitor mode */
		if (tsf == 1) {
			/* We have not received any beacons or probe
			 * responses. Since a beacon should be sent
			 * every 'intval' ms, we compute the next
			 * beacon timestamp using the hardware TSF. We
			 * ensure that it is at least FUDGE TUs ahead
			 * of the current TSF. Otherwise, we use the
			 * next beacon timestamp again */
 			nexttbtt = roundup(hw_tsftu + FUDGE, intval);
		} 
#if 0
		else {
			if (tsf > hw_tsf) {
				/* We received a beacon, but the HW TSF has
				 * not been updated (otherwise hw_tsf > tsf)
				 * We cannot use the hardware TSF, so we
				 * wait to synchronize beacons again. */
				sc->sc_syncbeacon = 1;
				goto ath_beacon_config_debug;
			} else {
				/* Normal case: we received a beacon to which
				 * we have synchronized. Make sure that nexttbtt
				 * is at least FUDGE TU ahead of hw_tsf */
				nexttbtt = tsftu + roundup(hw_tsftu + FUDGE - 
						tsftu, intval);
			}
		}
#endif
	}

	if (ic->ic_opmode == IEEE80211_M_STA &&	!(sc->sc_nostabeacons)) {
		HAL_BEACON_STATE bs;
		int dtimperiod, dtimcount;
		int cfpperiod, cfpcount;

		/* Setup DTIM and CTP parameters according to last
		 * beacon we received (which may not have
		 * happened). */
		dtimperiod = vap->iv_dtim_period;
		if (dtimperiod <= 0)		/* NB: 0 if not known */
			dtimperiod = 1;
		dtimcount = vap->iv_dtim_count;
		if (dtimcount >= dtimperiod)	/* NB: sanity check */
			dtimcount = 0;		/* XXX? */
		cfpperiod = 1;			/* NB: no PCF support yet */
		cfpcount = 0;

		/* Pull nexttbtt forward to reflect the current
		 * TSF and calculate dtim+cfp state for the result. */
		nexttbtt = tsftu;
		if (nexttbtt == 0)		/* e.g. for ap mode */
			nexttbtt = intval;
		do {
			nexttbtt += intval;
			if (--dtimcount < 0) {
				dtimcount = dtimperiod - 1;
				if (--cfpcount < 0)
					cfpcount = cfpperiod - 1;
			}
		} while (nexttbtt < hw_tsftu + FUDGE);
#undef FUDGE
		memset(&bs, 0, sizeof(bs));
		bs.bs_intval = intval;
		bs.bs_nexttbtt = nexttbtt;
		bs.bs_dtimperiod = dtimperiod * intval;
		bs.bs_nextdtim = bs.bs_nexttbtt + dtimcount * intval;
		bs.bs_cfpperiod = cfpperiod * bs.bs_dtimperiod;
		bs.bs_cfpnext = bs.bs_nextdtim + cfpcount * bs.bs_dtimperiod;
		bs.bs_cfpmaxduration = 0;
#if 0
		/*
		 * The 802.11 layer records the offset to the DTIM
		 * bitmap while receiving beacons; use it here to
		 * enable h/w detection of our AID being marked in
		 * the bitmap vector (to indicate frames for us are
		 * pending at the AP).
		 * XXX do DTIM handling in s/w to WAR old h/w bugs
		 * XXX enable based on h/w rev for newer chips
		 */
		bs.bs_timoffset = ni->ni_timoff;
#endif
		/*
		 * Calculate the number of consecutive beacons to miss
		 * before taking a BMISS interrupt.  The configuration
		 * is specified in TU so we only need calculate based
		 * on the beacon interval.  Note that we clamp the
		 * result to at most 10 beacons.
		 */
		bs.bs_bmissthreshold = howmany(ic->ic_bmisstimeout, intval);
		if (bs.bs_bmissthreshold > 10)
			bs.bs_bmissthreshold = 10;
		else if (bs.bs_bmissthreshold < 2)
			bs.bs_bmissthreshold = 2;

		/*
		 * Calculate sleep duration.  The configuration is
		 * given in ms.  We ensure a multiple of the beacon
		 * period is used.  Also, if the sleep duration is
		 * greater than the DTIM period then it makes senses
		 * to make it a multiple of that.
		 *
		 * XXX fixed at 100ms
		 */
		bs.bs_sleepduration =
			roundup(IEEE80211_MS_TO_TU(100), bs.bs_intval);
		if (bs.bs_sleepduration > bs.bs_dtimperiod)
			bs.bs_sleepduration = roundup(bs.bs_sleepduration, 
					bs.bs_dtimperiod);

		DPRINTF(sc, ATH_DEBUG_BEACON,
			"%s: tsf %llu tsf:tu %u intval %u nexttbtt %u dtim %u "
			"nextdtim %u bmiss %u sleep %u cfp:period %u "
			"maxdur %u next %u timoffset %u\n",
			__func__,
			(unsigned long long)tsf, tsftu,
			bs.bs_intval,
			bs.bs_nexttbtt,
			bs.bs_dtimperiod,
			bs.bs_nextdtim,
			bs.bs_bmissthreshold,
			bs.bs_sleepduration,
			bs.bs_cfpperiod,
			bs.bs_cfpmaxduration,
			bs.bs_cfpnext,
			bs.bs_timoffset
		);

		ic->ic_bmiss_guard = jiffies +
			IEEE80211_TU_TO_JIFFIES(bs.bs_intval * 
					bs.bs_bmissthreshold);

		ath_hal_intrset(ah, 0);
		ath_hal_beacontimers(ah, &bs);
		sc->sc_imask |= HAL_INT_BMISS;
		ath_hal_intrset(ah, sc->sc_imask);
	} else {
		ath_hal_intrset(ah, 0);
		if (reset_tsf)
			intval |= HAL_BEACON_RESET_TSF;
		if (IEEE80211_IS_MODE_BEACON(ic->ic_opmode)) {
			/*
			 * In AP/IBSS mode we enable the beacon timers and
			 * SWBA interrupts to prepare beacon frames.
			 */
			intval |= HAL_BEACON_ENA;
			sc->sc_imask |= HAL_INT_SWBA;
		}
#ifdef ATH_SUPERG_DYNTURBO
		ath_beacon_dturbo_config(vap, intval &
				~(HAL_BEACON_RESET_TSF | HAL_BEACON_ENA));
#endif
		sc->sc_nexttbtt = nexttbtt;
		ath_hal_beaconinit(ah, nexttbtt, intval);
		sc->sc_bmisscount = 0;
		ath_hal_intrset(ah, sc->sc_imask);
	}
	sc->sc_syncbeacon = 0;

#if 0
ath_beacon_config_debug:
#endif
	/* We print all debug messages here, in order to preserve the
	 * time critical aspect of this function */
	DPRINTF(sc, ATH_DEBUG_BEACON,
		"%s: ni=%p tsf=%llu hw_tsf=%llu tsftu=%u hw_tsftu=%u\n",
		__func__, ni, 
		(unsigned long long)tsf, (unsigned long long)hw_tsf, 
		tsftu, hw_tsftu);

	if (reset_tsf) {
		/* We just created the interface */
		DPRINTF(sc, ATH_DEBUG_BEACON, "%s: first beacon\n", __func__);
	} else {
		if (tsf == 1) {
			/* We did not receive any beacons or probe response */
			DPRINTF(sc, ATH_DEBUG_BEACON,
				"%s: no beacon received...\n", __func__);
		} else {
			if (tsf > hw_tsf) {
				/* We did receive a beacon but the hw TSF has 
				 * not been updated - must have been a 
				 * different BSSID */
				DPRINTF(sc, ATH_DEBUG_BEACON,
					"%s: beacon received, but TSF has not "
					"been updated\n", __func__);
			} else {
				/* We did receive a beacon, normal case */
				DPRINTF(sc, ATH_DEBUG_BEACON,
					"%s: beacon received, TSF and timers "
					"synchronized\n", __func__);
			}
		}
	}

	DPRINTF(sc, ATH_DEBUG_BEACON, 
		"%s: nexttbtt=%10x intval=%u%s%s imask=%s%s\n", __func__, 
		nexttbtt, intval & HAL_BEACON_PERIOD,
		intval & HAL_BEACON_ENA       ? " HAL_BEACON_ENA"       : "",
		intval & HAL_BEACON_RESET_TSF ? " HAL_BEACON_RESET_TSF" : "",
		sc->sc_imask & HAL_INT_BMISS  ? " HAL_INT_BMISS"        : "",
		sc->sc_imask & HAL_INT_SWBA   ? " HAL_INT_SWBA"         : "");
}

static int
ath_descdma_setup(struct ath_softc *sc,
	struct ath_descdma *dd, ath_bufhead *head,
	const char *name, int nbuf, int ndesc)
{
#define	DS2PHYS(_dd, _ds) \
	((_dd)->dd_desc_paddr + ((caddr_t)(_ds) - (caddr_t)(_dd)->dd_desc))
	struct ath_desc *ds;
	struct ath_buf *bf;
	unsigned int i, bsize;
	int error;

	DPRINTF(sc, ATH_DEBUG_RESET, "%s: %s DMA: %u buffers %u desc/buf\n",
		__func__, name, nbuf, ndesc);

	dd->dd_name = name;
	dd->dd_desc_len = sizeof(struct ath_desc) * nbuf * ndesc;

	/* allocate descriptors */
	dd->dd_desc = bus_alloc_consistent(sc->sc_bdev,
		dd->dd_desc_len, &dd->dd_desc_paddr);
	if (dd->dd_desc == NULL) {
		error = -ENOMEM;
		goto fail;
	}
	ds = dd->dd_desc;
	DPRINTF(sc, ATH_DEBUG_RESET, "%s: %s DMA map: %p (%lu) -> %llx (%lu)\n",
		__func__, dd->dd_name, ds, (u_long) dd->dd_desc_len,
		ito64(dd->dd_desc_paddr), /*XXX*/ (u_long) dd->dd_desc_len);

	/* allocate buffers */
	bsize = sizeof(struct ath_buf) * nbuf;
	bf = kmalloc(bsize, GFP_KERNEL);
	if (bf == NULL) {
		error = -ENOMEM;
		goto fail2;
	}
	memset(bf, 0, bsize);
	dd->dd_bufptr = bf;

	STAILQ_INIT(head);
	for (i = 0; i < nbuf; i++, bf++, ds += ndesc) {
		bf->bf_desc = ds;
		bf->bf_daddr = DS2PHYS(dd, ds);
		STAILQ_INSERT_TAIL(head, bf, bf_list);
	}
	return 0;
fail2:
	bus_free_consistent(sc->sc_bdev, dd->dd_desc_len,
		dd->dd_desc, dd->dd_desc_paddr);
fail:
	memset(dd, 0, sizeof(*dd));
	return error;
#undef DS2PHYS
}

static void
ath_descdma_cleanup(struct ath_softc *sc,
	struct ath_descdma *dd, ath_bufhead *head, int direction)
{
	struct ath_buf *bf;
	STAILQ_FOREACH(bf, head, bf_list)
		cleanup_ath_buf(sc, bf, direction);

	/* Free memory associated with descriptors */
	bus_free_consistent(sc->sc_bdev, dd->dd_desc_len,
		dd->dd_desc, dd->dd_desc_paddr);
	STAILQ_INIT(head);
	kfree(dd->dd_bufptr);
	memset(dd, 0, sizeof(*dd));
}

static int
ath_desc_alloc(struct ath_softc *sc)
{
	int error;

	error = ath_descdma_setup(sc, &sc->sc_rxdma, &sc->sc_rxbuf,
			"rx", ATH_RXBUF, 1);
	if (error != 0)
		return error;

	error = ath_descdma_setup(sc, &sc->sc_txdma, &sc->sc_txbuf,
			"tx", ATH_TXBUF, ATH_TXDESC);
	if (error != 0) {
		ath_descdma_cleanup(sc, &sc->sc_rxdma, &sc->sc_rxbuf,
			BUS_DMA_FROMDEVICE);
		return error;
	}

	/* XXX allocate beacon state together with VAP */
	error = ath_descdma_setup(sc, &sc->sc_bdma, &sc->sc_bbuf,
			"beacon", ATH_BCBUF, 1);
	if (error != 0) {
		ath_descdma_cleanup(sc, &sc->sc_txdma, &sc->sc_txbuf,
			BUS_DMA_TODEVICE);
		ath_descdma_cleanup(sc, &sc->sc_rxdma, &sc->sc_rxbuf,
			BUS_DMA_FROMDEVICE);
		return error;
	}
	return 0;
}

static void
ath_desc_free(struct ath_softc *sc)
{
	if (sc->sc_bdma.dd_desc_len != 0)
		ath_descdma_cleanup(sc, &sc->sc_bdma, &sc->sc_bbuf,
			BUS_DMA_TODEVICE);
	if (sc->sc_txdma.dd_desc_len != 0)
		ath_descdma_cleanup(sc, &sc->sc_txdma, &sc->sc_txbuf,
			BUS_DMA_TODEVICE);
	if (sc->sc_rxdma.dd_desc_len != 0)
		ath_descdma_cleanup(sc, &sc->sc_rxdma, &sc->sc_rxbuf,
			BUS_DMA_FROMDEVICE);
}

static struct ieee80211_node *
#ifdef IEEE80211_DEBUG_REFCNT
ath_node_alloc_debug(struct ieee80211vap *vap, const char* func, int line)
#else
ath_node_alloc(struct ieee80211vap *vap)
#endif 
{
	struct ath_softc *sc = vap->iv_ic->ic_dev->priv;
	const size_t space = sizeof(struct ath_node) + sc->sc_rc->arc_space;
	struct ath_node *an = kmalloc(space, GFP_ATOMIC);
	if (an != NULL) {
		memset(an, 0, space);
		an->an_decomp_index = INVALID_DECOMP_INDEX;
		an->an_avgrssi = ATH_RSSI_DUMMY_MARKER;
		an->an_halstats.ns_avgbrssi = ATH_RSSI_DUMMY_MARKER;
		an->an_halstats.ns_avgrssi = ATH_RSSI_DUMMY_MARKER;
		an->an_halstats.ns_avgtxrssi = ATH_RSSI_DUMMY_MARKER;
		/*
		 * ath_rate_node_init needs a vap pointer in node
		 * to decide which mgt rate to use
		 */
		an->an_node.ni_vap = vap;
		sc->sc_rc->ops->node_init(sc, an);
		/* U-APSD init */
		STAILQ_INIT(&an->an_uapsd_q);
		an->an_uapsd_qdepth = 0;
		STAILQ_INIT(&an->an_uapsd_overflowq);
		an->an_uapsd_overflowqdepth = 0;
		ATH_NODE_UAPSD_LOCK_INIT(an);
		return &an->an_node;
	} else {
		return NULL;
	}
}

static void
#ifdef IEEE80211_DEBUG_REFCNT
ath_node_cleanup_debug(struct ieee80211_node *ni, const char* func, int line)
#else
ath_node_cleanup(struct ieee80211_node *ni)
#endif
{
	struct ieee80211com *ic = ni->ni_ic;
	struct ath_softc *sc = ni->ni_ic->ic_dev->priv;
	struct ath_node *an = ATH_NODE(ni);
	struct ath_buf *bf;

	/*
	 * U-APSD cleanup
	 */
	ATH_NODE_UAPSD_LOCK_IRQ(an);
	if (ni->ni_flags & IEEE80211_NODE_UAPSD_TRIG) {
		ni->ni_flags &= ~IEEE80211_NODE_UAPSD_TRIG;
		ic->ic_uapsdmaxtriggers--;
		ni->ni_flags &= ~IEEE80211_NODE_UAPSD_SP;
	}
	ATH_NODE_UAPSD_UNLOCK_IRQ(an);

	while (an->an_uapsd_qdepth) {
		bf = STAILQ_FIRST(&an->an_uapsd_q);
		STAILQ_REMOVE_HEAD(&an->an_uapsd_q, bf_list);
#ifdef IEEE80211_DEBUG_REFCNT
		ath_return_txbuf_debug(sc, &bf, func, line);
#else /* #ifdef IEEE80211_DEBUG_REFCNT */
		ath_return_txbuf(sc, &bf);
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */
		an->an_uapsd_qdepth--;
	}

	while (an->an_uapsd_overflowqdepth) {
		bf = STAILQ_FIRST(&an->an_uapsd_overflowq);
		STAILQ_REMOVE_HEAD(&an->an_uapsd_overflowq, bf_list);
#ifdef IEEE80211_DEBUG_REFCNT
		ath_return_txbuf_debug(sc, &bf, func, line);
#else /* #ifdef IEEE80211_DEBUG_REFCNT */
		ath_return_txbuf(sc, &bf);
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */
		an->an_uapsd_overflowqdepth--;
	}

	/* Clean up node-specific rate things - this currently appears to 
	 * always be a no-op */
	sc->sc_rc->ops->node_cleanup(sc, ATH_NODE(ni));

	ATH_NODE_UAPSD_LOCK_IRQ(an);
#ifdef IEEE80211_DEBUG_REFCNT
	sc->sc_node_cleanup_debug(ni, func, line);
#else /* #ifdef IEEE80211_DEBUG_REFCNT */
	sc->sc_node_cleanup(ni);
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */
	ATH_NODE_UAPSD_UNLOCK_IRQ(an);
}

static void
#ifdef IEEE80211_DEBUG_REFCNT
ath_node_free_debug(struct ieee80211_node *ni, const char* func, int line)
#else
ath_node_free(struct ieee80211_node *ni)
#endif
{
	struct ath_softc *sc = ni->ni_ic->ic_dev->priv;

#ifdef IEEE80211_DEBUG_REFCNT
	sc->sc_node_free_debug(ni, func, line);
#else
	sc->sc_node_free(ni);
#endif

#ifdef ATH_SUPERG_XR
	ath_grppoll_period_update(sc);
#endif
}

static u_int8_t
ath_node_getrssi(const struct ieee80211_node *ni)
{
#define	HAL_EP_RND(x, mul) \
	((((x) % (mul)) >= ((mul) / 2)) ? ((x) + ((mul) - 1)) / 	\
	 (mul) : (x)/(mul))
	u_int32_t avgrssi = ATH_NODE_CONST(ni)->an_avgrssi;
	int32_t rssi;

	/*
	 * When only one frame is received there will be no state in
	 * avgrssi so fallback on the value recorded by the 802.11 layer.
	 */
	if (avgrssi != ATH_RSSI_DUMMY_MARKER)
		rssi = HAL_EP_RND(avgrssi, HAL_RSSI_EP_MULTIPLIER);
	else
		rssi = ni->ni_rssi;
	/* NB: theoretically we shouldn't need this, but be paranoid */
	return rssi < 0 ? 0 : rssi > 127 ? 127 : rssi;
#undef HAL_EP_RND
}


#ifdef ATH_SUPERG_XR
/*
 * Stops the txqs and moves data between XR and Normal queues.
 * Also adjusts the rate info in the descriptors.
 */

static u_int8_t
ath_node_move_data(const struct ieee80211_node *ni)
{
#ifdef NOT_YET
	struct ath_txq *txq = NULL;
	struct ieee80211com *ic = ni->ni_ic;
	struct ath_softc *sc = ic->ic_dev->priv;
	struct ath_buf *bf, *prev, *bf_tmp, *bf_tmp1;
	struct ath_hal *ah = sc->sc_ah;
	struct sk_buff *skb = NULL;
	struct ath_desc *ds;
	struct ath_tx_status *ts;
	HAL_STATUS status;
	unsigned int index;


	if (ni->ni_vap->iv_flags & IEEE80211_F_XR) {
		struct ath_txq tmp_q;
		memset(&tmp_q, 0, sizeof(tmp_q));
		STAILQ_INIT(&tmp_q.axq_q);
		/*
		 * move data from Normal txqs to XR queue.
		 */
		printk("move data from NORMAL to XR\n");
		/*
		 * collect all the data towards the node
		 * in to the tmp_q.
		 */
		index = WME_AC_VO;
		while (index >= WME_AC_BE && txq != sc->sc_ac2q[index]) {
			txq = sc->sc_ac2q[index];

			ATH_TXQ_LOCK_IRQ(txq);
			ath_hal_stoptxdma(ah, txq->axq_qnum);
			bf = prev = STAILQ_FIRST(&txq->axq_q);
			/*
			 * skip all the buffers that are done
			 * until the first one that is in progress
			 */
			while (bf) {
#ifdef ATH_SUPERG_FF
				ds = &bf->bf_desc[bf->bf_numdescff];
#else
				ds = bf->bf_desc;	/* NB: last descriptor */
#endif
				ts = &ts->bf_dsstatus.ds_txstat;
				status = ath_hal_txprocdesc(ah, ds, ts);
				if (status == HAL_EINPROGRESS)
					break;
				prev = bf;
				bf = STAILQ_NEXT(bf, bf_list);
			}

			/* save the pointer to the last buf that's done */
			if (prev == bf)
				bf_tmp = NULL;
			else
				bf_tmp = prev;
			while (bf) {
				if (ni == bf->bf_node) {
					if (prev == bf) {
						ATH_TXQ_REMOVE_HEAD(txq, 
								bf_list);
						STAILQ_INSERT_TAIL(
								&tmp_q.axq_q, 
								bf, bf_list
								);
						bf = STAILQ_FIRST(&txq->axq_q);
						prev = bf;
					} else {
						STAILQ_REMOVE_AFTER(
								&(txq->axq_q), 
								prev, bf_list
								);
						txq->axq_depth--;
						STAILQ_INSERT_TAIL(&tmp_q.axq_q, 
								bf, bf_list);
						bf = STAILQ_NEXT(prev, bf_list);
						/*
						 * after deleting the node
						 * link the descriptors
						 */
#ifdef ATH_SUPERG_FF
						ds = &prev->bf_desc[prev->bf_numdescff];
#else
						/* NB: last descriptor */
						ds = prev->bf_desc;
#endif
#ifdef AH_NEED_DESC_SWAP
						ds->ds_link = cpu_to_le32(bf->bf_daddr);
#else
						ds->ds_link = bf->bf_daddr;
#endif
					}
				} else {
					prev = bf;
					bf = STAILQ_NEXT(bf, bf_list);
				}
			}
			/*
			 * if the last buf was deleted.
			 * set the pointer to the last descriptor.
			 */
			bf = STAILQ_FIRST(&txq->axq_q);
			if (bf) {
				if (prev) {
					bf = STAILQ_NEXT(prev, bf_list);
					if (!bf) { /* prev is the last one on 
						    * the list */
#ifdef ATH_SUPERG_FF
						ds = &prev->bf_desc[prev->bf_numdescff];
#else
						/* NB: last descriptor */
						ds = prev->bf_desc;
#endif
						ts = &bf->bf_dsstatus.ds_txstat;
						status = ath_hal_txprocdesc(
								ah, ds, ts
								);
						if (status == HAL_EINPROGRESS)
							txq->axq_link = 
								&ds->ds_link;
						else
							txq->axq_link = NULL;
					}
				}
			} else
				txq->axq_link = NULL;

			ATH_TXQ_UNLOCK_IRQ(txq);

			/*
			 * restart the DMA from the first
			 * buffer that was not DMA'd.
			 */
			if (bf_tmp)
				bf = STAILQ_NEXT(bf_tmp, bf_list);
			else
				bf = STAILQ_FIRST(&txq->axq_q);
			if (bf) {
				ath_hal_puttxbuf(ah, txq->axq_qnum, 
						bf->bf_daddr);
				ath_hal_txstart(ah, txq->axq_qnum);
			}
		}
		/*
		 * queue them on to the XR txqueue.
		 * can not directly put them on to the XR txq. since the
		 * skb data size may be greater than the XR fragmentation
		 * threshold size.
		 */
		bf  = STAILQ_FIRST(&tmp_q.axq_q);
		index = 0;
		while (bf) {
			skb = bf->bf_skb;
			bf->bf_skb = NULL;
			ath_return_txbuf(sc, &bf);
			/* Untrack because ath_hardstart will restart tracking */
			ieee80211_skb_untrack(skb);
			ath_hardstart(skb, sc->sc_dev);
			ATH_TXQ_REMOVE_HEAD(&tmp_q, bf_list);
			bf = STAILQ_FIRST(&tmp_q.axq_q);
			index++;
		}
		DPRINTF(sc, ATH_DEBUG_XMIT_PROC, 
				"moved %d buffers from NORMAL to XR\n", index);
	} else {
		struct ath_txq wme_tmp_qs[WME_AC_VO+1];
		struct ath_txq *wmeq = NULL, *prevq;
		struct ieee80211_frame *wh;
		struct ath_desc *ds = NULL;
		unsigned int count = 0;

		/*
		 * move data from XR txq to Normal txqs.
		 */
		DPRINTF(sc, ATH_DEBUG_XMIT_PROC, 
				"move buffers from XR to NORMAL\n");
		memset(&wme_tmp_qs, 0, sizeof(wme_tmp_qs));
		for (index = 0; index <= WME_AC_VO; index++)
			STAILQ_INIT(&wme_tmp_qs[index].axq_q);
		txq = sc->sc_xrtxq;

		ATH_TXQ_LOCK_IRQ(txq);
		ath_hal_stoptxdma(ah, txq->axq_qnum);
		bf = prev = STAILQ_FIRST(&txq->axq_q);
		/*
		 * skip all the buffers that are done
		 * until the first one that is in progress
		 */
		while (bf) {
#ifdef ATH_SUPERG_FF
			ds = &bf->bf_desc[bf->bf_numdescff];
#else
			ds = bf->bf_desc;		/* NB: last descriptor */
#endif
			ts = &bf->bf_dsstatus.ds_txstat;
			status = ath_hal_txprocdesc(ah, ds, ts);
			if (status == HAL_EINPROGRESS)
				break;
			prev= bf;
			bf = STAILQ_NEXT(bf, bf_list);
		}
		/*
		 * save the pointer to the last buf that's
		 * done
		 */
		if (prev == bf)
			bf_tmp1 = NULL;
		else
			bf_tmp1 = prev;
		/*
		 * collect all the data in to four temp SW queues.
		 */
		while (bf) {
			if (ni == bf->bf_node) {
				if (prev == bf) {
					STAILQ_REMOVE_HEAD(&txq->axq_q, bf_list);
					bf_tmp=bf;
					bf = STAILQ_FIRST(&txq->axq_q);
					prev = bf;
				} else {
					STAILQ_REMOVE_AFTER(&(txq->axq_q), 
							prev, bf_list);
					bf_tmp=bf;
					bf = STAILQ_NEXT(prev, bf_list);
				}
				count++;
				skb = bf_tmp->bf_skb;
				wh = (struct ieee80211_frame *)skb->data;
				if (wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_QOS) {
					/* XXX validate skb->priority, remove 
					 * mask */
					wmeq = &wme_tmp_qs[skb->priority & 0x3];
				} else
					wmeq = &wme_tmp_qs[WME_AC_BE];
				STAILQ_INSERT_TAIL(&wmeq->axq_q, bf_tmp, bf_list);
				ds = bf_tmp->bf_desc;
				/*
				 * link the descriptors
				 */
				if (wmeq->axq_link != NULL) {
#ifdef AH_NEED_DESC_SWAP
					*wmeq->axq_link = 
						cpu_to_le32(bf_tmp->bf_daddr);
#else
					*wmeq->axq_link = bf_tmp->bf_daddr;
#endif
					DPRINTF(sc, ATH_DEBUG_XMIT, 
							"%s: link[%u](%p)=%p (%p)\n",
							__func__,
							wmeq->axq_qnum, wmeq->axq_link,
							(caddr_t)bf_tmp->bf_daddr, 
							bf_tmp->bf_desc);
				}
				wmeq->axq_link = &ds->ds_link;
				/*
				 * update the rate information
				 */
			} else {
				prev = bf;
				bf = STAILQ_NEXT(bf, bf_list);
			}
		}
		/*
		 * reset the axq_link pointer to the last descriptor.
		 */
		bf = STAILQ_FIRST(&txq->axq_q);
		if (bf) {
			if (prev) {
				bf = STAILQ_NEXT(prev, bf_list);
				if (!bf) { /* prev is the last one on the list */
#ifdef ATH_SUPERG_FF
					ds = &prev->bf_desc[prev->bf_numdescff];
#else
					/* NB: last descriptor */
					ds = prev->bf_desc;
#endif
					ts = &bf->bf_dsstatus.ds_txstat;
					status = ath_hal_txprocdesc(ah, ds, ts);
					if (status == HAL_EINPROGRESS)
						txq->axq_link = &ds->ds_link;
					else
						txq->axq_link = NULL;
				}
			}
		} else {
			/*
			 * if the list is empty reset the pointer.
			 */
			txq->axq_link = NULL;
		}
		ATH_TXQ_UNLOCK_IRQ(txq);

		/*
		 * restart the DMA from the first
		 * buffer that was not DMA'd.
		 */
		if (bf_tmp1)
			bf = STAILQ_NEXT(bf_tmp1, bf_list);
		else
			bf = STAILQ_FIRST(&txq->axq_q);

		if (bf) {
			ath_hal_puttxbuf(ah, txq->axq_qnum, bf->bf_daddr);
			ath_hal_txstart(ah, txq->axq_qnum);
		}

		/* Move (concat.) the lists from the temp. SW queues in to
		 * WME queues. */
		index = WME_AC_VO;
		while (index >= WME_AC_BE) {
			txq = sc->sc_ac2q[index];

			ATH_TXQ_LOCK_IRQ(txq);
			ath_hal_stoptxdma(ah, txq->axq_qnum);

			while ((txq == sc->sc_ac2q[index]) && 
					(index >= WME_AC_BE)) {
				wmeq = &wme_tmp_qs[index];
				bf = STAILQ_FIRST(&wmeq->axq_q);
				if (bf) {
					ATH_TXQ_MOVE_Q(wmeq, txq);
					if (txq->axq_link != NULL) {
#ifdef AH_NEED_DESC_SWAP
						*(txq->axq_link) = 
							cpu_to_le32(bf->bf_daddr);
#else
						*(txq->axq_link) = bf->bf_daddr;
#endif
					}
				}
				index--;
			}

			/* Find the first buffer to be DMA'd. */
			bf = STAILQ_FIRST(&txq->axq_q);
			while (bf) {
#ifdef ATH_SUPERG_FF
				ds = &bf->bf_desc[bf->bf_numdescff];
#else
				ds = bf->bf_desc;	/* NB: last descriptor */
#endif
				ts = &bf->bf_dsstatus.ds_txstat;
				status = ath_hal_txprocdesc(ah, ds, ts);
				if (status == HAL_EINPROGRESS)
					break;
				bf = STAILQ_NEXT(bf, bf_list);
			}

			if (bf) {
				ath_hal_puttxbuf(ah, txq->axq_qnum, bf->bf_daddr);
				ath_hal_txstart(ah, txq->axq_qnum);
			}

			ATH_TXQ_UNLOCK_IRQ(txq);
		}

		DPRINTF(sc, ATH_DEBUG_XMIT_PROC, 
				"moved %d buffers from XR to NORMAL\n"m count);
	}
#endif
	return 0;
}
#endif

static struct sk_buff *
#ifdef IEEE80211_DEBUG_REFCNT
#define ath_alloc_skb(_size, _align) \
	ath_alloc_skb_debug(_size, _align, __func__, __LINE__)
ath_alloc_skb_debug(u_int size, u_int align, const char* func, int line)
#else
ath_alloc_skb(u_int size, u_int align)
#endif
{
	struct sk_buff *skb;
	u_int off;

#ifdef IEEE80211_DEBUG_REFCNT
	skb = ieee80211_dev_alloc_skb_debug(size + align - 1, func, line);
#else
	skb = ieee80211_dev_alloc_skb(size + align - 1);
#endif
	if (skb != NULL) {
		off = ((unsigned long) skb->data) % align;
		if (off != 0)
			skb_reserve(skb, align - off);
	}
	return skb;
}

static int
ath_rxbuf_init(struct ath_softc *sc, struct ath_buf *bf)
{
	struct ath_hal *ah = sc->sc_ah;
	struct sk_buff *skb;
	struct ath_desc *ds;

#if 0
	/* Free the prior skb, if present */
	if (bf->bf_skb != NULL) {
		ieee80211_dev_kfree_skb(&bf->bf_skb);
		if (bf->bf_skbaddr != 0) {
			bus_unmap_single(sc->sc_bdev, bf->bf_skbaddr,
				sc->sc_rxbufsize, BUS_DMA_FROMDEVICE);
			bf->bf_skbaddr = 0;
		}
	}
#endif

	skb = bf->bf_skb;
	if (skb == NULL) {
		if (sc->sc_nmonvaps > 0) {
			u_int off;
			unsigned int extra = A_MAX(sizeof(struct ath_rx_radiotap_header),
						   A_MAX(sizeof(struct wlan_ng_prism2_header),
							 ATHDESC_HEADER_SIZE));

			/*
			 * Allocate buffer for monitor mode with space for the
			 * wlan-ng style physical layer header at the start.
			 */
			skb = ieee80211_dev_alloc_skb(sc->sc_rxbufsize + 
					extra + sc->sc_cachelsz - 1);
			if (skb == NULL) {
				sc->sc_stats.ast_rx_nobuf++;
				return -ENOMEM;
			}
			/*
			 * Reserve space for the Prism header.
 			 */
 			skb_reserve(skb, sizeof(struct wlan_ng_prism2_header));
			/*
			 * Align to cache line.
			 */
			off = ((unsigned long) skb->data) % sc->sc_cachelsz;
			if (off != 0)
				skb_reserve(skb, sc->sc_cachelsz - off);
		} else {
			/*
			 * Cache-line-align.  This is important (for the
			 * 5210 at least) as not doing so causes bogus data
			 * in rx'd frames.
			 */
			skb = ath_alloc_skb(sc->sc_rxbufsize, sc->sc_cachelsz);
			if (skb == NULL) {
				DPRINTF(sc, ATH_DEBUG_ANY,
					"%s: skbuff alloc of size %u failed\n",
					__func__, sc->sc_rxbufsize);
				sc->sc_stats.ast_rx_nobuf++;
				return -ENOMEM;
			}
		}
		skb->dev = sc->sc_dev;
		bf->bf_skb = skb;
		bf->bf_skbaddr = bus_map_single(sc->sc_bdev,
			skb->data, sc->sc_rxbufsize, BUS_DMA_FROMDEVICE);
	}

	/*
	 * Setup descriptors.  For receive we always terminate
	 * the descriptor list with a self-linked entry so we'll
	 * not get overrun under high load (as can happen with a
	 * 5212 when ANI processing enables PHY error frames).
	 *
	 * To ensure the last descriptor is self-linked we create
	 * each descriptor as self-linked and add it to the end.  As
	 * each additional descriptor is added the previous self-linked
	 * entry is ``fixed'' naturally.  This should be safe even
	 * if DMA is happening.  When processing RX interrupts we
	 * never remove/process the last, self-linked, entry on the
	 * descriptor list.  This ensures the hardware always has
	 * someplace to write a new frame.
	 */
	ds = bf->bf_desc;
	ds->ds_link = bf->bf_daddr;		/* link to self */
	ds->ds_data = bf->bf_skbaddr;
	ath_hal_setuprxdesc(ah, ds,
		skb_tailroom(skb),		/* buffer size */
		0);
	if (sc->sc_rxlink != NULL)
		*sc->sc_rxlink = bf->bf_daddr;
	sc->sc_rxlink = &ds->ds_link;
	return 0;
}

/* This function calculates the presence of, and then removes any padding
 * bytes between the frame header and frame body, and returns a modified 
 * SKB. If padding is removed and copy_skb is specified, then a new SKB is 
 * created, otherwise the same SKB is used.
 *
 * NB: MAY ALLOCATE */
static struct sk_buff *
ath_skb_removepad(struct sk_buff *skb, unsigned int copy_skb)
{
	struct sk_buff *tskb = skb;
	struct ieee80211_frame *wh = (struct ieee80211_frame *)skb->data;
	unsigned int padbytes = 0, headersize = 0;

	/* Only non-control frames have bodies, and hence padding. */
	if (IEEE80211_FRM_HAS_BODY(wh)) {
		headersize = ieee80211_anyhdrsize(wh);
		padbytes = roundup(headersize, 4) - headersize;
		if (padbytes > 0) {
			if (copy_skb) {
				/* Copy skb and remove HW pad bytes */
				tskb = skb_copy(skb, GFP_ATOMIC);
				if (tskb == NULL)
					return NULL;
				/* Reference any node from the source skb. */
				if (SKB_CB(skb)->ni != NULL)
					SKB_CB(tskb)->ni = ieee80211_ref_node(
							SKB_CB(skb)->ni);
			}
			memmove(tskb->data + padbytes, tskb->data, headersize);
			skb_pull(tskb, padbytes);
		}
  	}
	return tskb;
}

/*
 * Add a prism2 header to a received frame and
 * dispatch it to capture tools like kismet.
 */
static void
ath_capture(struct net_device *dev, const struct ath_buf *bf,
		struct sk_buff *skb, u_int64_t tsf, unsigned int tx)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct sk_buff *tskb = NULL;
  
  	KASSERT(ic->ic_flags & IEEE80211_F_DATAPAD,
  		("data padding not enabled?"));
  
	if (sc->sc_nmonvaps <= 0)
		return;

	/* Never copy the SKB, as it is ours on the RX side, and this is the 
	 * last process on the TX side and we only modify our own headers. */
	tskb = ath_skb_removepad(skb, 0 /* Copy SKB */);
	if (tskb == NULL)
		return;
	
	ieee80211_input_monitor(ic, tskb, bf, tx, tsf, sc);
}

/*
 * Intercept management frames to collect beacon RSSI data and to do
 * ibss merges. This function is called for all management frames,
 * including those belonging to other BSS.
 */
static void
ath_recv_mgmt(struct ieee80211_node *ni, struct sk_buff *skb,
	int subtype, int rssi, u_int64_t rtsf)
{
	struct ath_softc *sc = ni->ni_ic->ic_dev->priv;
	struct ieee80211vap *vap = ni->ni_vap;
	u_int64_t hw_tsf, beacon_tsf;
	u_int32_t hw_tu;

	DPRINTF(sc, ATH_DEBUG_BEACON,
		"%s: vap:%p[" MAC_FMT "] ni:%p[" MAC_FMT "]\n",
		__func__, vap, MAC_ADDR(vap->iv_bss->ni_bssid),
		ni, MAC_ADDR(ni->ni_macaddr));

	/*
	 * Call up first so subsequent work can use information
	 * potentially stored in the node (e.g. for ibss merge).
	 */
	sc->sc_recv_mgmt(ni, skb, subtype, rssi, rtsf);

	switch (subtype) {
	case IEEE80211_FC0_SUBTYPE_BEACON:
		/* update rssi statistics for use by the HAL */
		ATH_RSSI_LPF(ATH_NODE(ni)->an_halstats.ns_avgbrssi, rssi);
		if ((sc->sc_syncbeacon ||
		    (vap->iv_flags_ext & IEEE80211_FEXT_APPIE_UPDATE)) &&
		     ni == vap->iv_bss && vap->iv_state == IEEE80211_S_RUN) {
			/*
			 * Resync beacon timers using the tsf of the
			 * beacon frame we just received.
			 */
			vap->iv_flags_ext &= ~IEEE80211_FEXT_APPIE_UPDATE;
			ath_beacon_update_timers(sc, vap);
			DPRINTF(sc, ATH_DEBUG_BEACON,
				"%s updated beacon timers\n", __func__);
		}
		/* fall thru... */
	case IEEE80211_FC0_SUBTYPE_PROBE_RESP:
		if (vap->iv_opmode == IEEE80211_M_IBSS &&
		    vap->iv_state == IEEE80211_S_RUN) {
			/* Don't merge if we have a desired BSSID */
			if (vap->iv_flags & IEEE80211_F_DESBSSID)
				break;

			/* Handle IBSS merge as needed; check the TSF on the
			 * frame before attempting the merge. The 802.11 spec.
			 * says the station should change its BSSID to match
			 * the oldest station with the same SSID, where oldest
			 * is determined by the TSF. Note that hardware
			 * reconfiguration happens through callback to
			 * ath_newstate as the state machine will go from
			 * RUN -> RUN when this happens. */
			hw_tsf = ath_hal_gettsf64(sc->sc_ah);
			hw_tu  = hw_tsf >> 10;
			beacon_tsf = le64_to_cpu(ni->ni_tstamp.tsf);

			DPRINTF(sc, ATH_DEBUG_BEACON,
				"beacon received at %10llx "
				"beacon_tsf %10llx (%lld), "
				"current hw TSF %10llx (%lld)\n",
				rtsf, beacon_tsf,
				beacon_tsf - rtsf,
				hw_tsf, hw_tsf - beacon_tsf);

			if (rtsf <= beacon_tsf) {
				DPRINTF(sc, ATH_DEBUG_BEACON,
					"trying ibss merge: rtsf %10llx "
					"beacon's tsf %10llx\n",
					rtsf, beacon_tsf);
				ieee80211_ibss_merge(ni);
			}

			/* Check sc_nexttbtt */
			if (sc->sc_nexttbtt < hw_tu) {
				DPRINTF(sc, ATH_DEBUG_BEACON,
					"sc_nexttbtt (%8x TU) is in the past "
					"(tsf %8x TU), updating timers\n",
					sc->sc_nexttbtt, hw_tu);
				ath_beacon_update_timers(sc, vap);
			}
		}
		break;
	}
}

static void
ath_setdefantenna(struct ath_softc *sc, u_int antenna)
{
	struct ath_hal *ah = sc->sc_ah;

	/* XXX block beacon interrupts */
	ath_hal_setdefantenna(ah, antenna);
	if (sc->sc_defant != antenna)
		sc->sc_stats.ast_ant_defswitch++;
	sc->sc_defant = antenna;
	sc->sc_rxotherant = 0;
}

static void
ath_rx_tasklet(TQUEUE_ARG data)
{
#define	PA2DESC(_sc, _pa) \
	((struct ath_desc *)((caddr_t)(_sc)->sc_rxdma.dd_desc + \
		((_pa) - (_sc)->sc_rxdma.dd_desc_paddr)))
	struct net_device *dev = (struct net_device *)data;
	struct ath_buf *bf;
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	struct ath_desc *ds;
	struct ath_rx_status *rs;
	struct sk_buff *skb = NULL;
	struct ieee80211_node *ni;
	unsigned int len;
	int type;
	u_int phyerr;

	DPRINTF(sc, ATH_DEBUG_RX_PROC, "%s\n", __func__);
	do {
		bf = STAILQ_FIRST(&sc->sc_rxbuf);
		if (bf == NULL) {		/* XXX ??? can this happen */
			printk("%s: no buffer (%s)\n", DEV_NAME(dev), __func__);
			break;
		}

		/*
		 * Descriptors are now processed at in the first-level
		 * interrupt handler to support U-APSD trigger search.
		 * This must also be done even when U-APSD is not active to support
		 * other error handling that requires immediate attention.
		 * We check bf_status to find out if the bf's descriptors have
		 * been processed by the HAL.
		 */
		if (!(bf->bf_status & ATH_BUFSTATUS_DONE))
			break;

		ds = bf->bf_desc;
		if (ds->ds_link == bf->bf_daddr) {
			/* NB: never process the self-linked entry at the end */
			break;
		}
		skb = bf->bf_skb;
		if (skb == NULL) {
			printk("%s: no skbuff (%s)\n", DEV_NAME(dev), __func__);
			continue;
		}

#ifdef AR_DEBUG
		if (sc->sc_debug & ATH_DEBUG_RECV_DESC)
			ath_printrxbuf(bf, 1);
#endif
		rs = &bf->bf_dsstatus.ds_rxstat;

		if(rs->rs_rssi < 0)
			rs->rs_rssi = 0;

		len = rs->rs_datalen;
		/* DMA sync. dies spectacularly if len == 0 */
		if (len == 0)
			goto rx_next;

		if (rs->rs_more) {
			/*
			 * Frame spans multiple descriptors; this
			 * cannot happen yet as we don't support
			 * jumbograms.  If not in monitor mode,
			 * discard the frame.
			 */
#ifndef ERROR_FRAMES
			/*
			 * Enable this if you want to see
			 * error frames in Monitor mode.
			 */
			if (ic->ic_opmode != IEEE80211_M_MONITOR) {
				sc->sc_stats.ast_rx_toobig++;
				goto rx_next;
			}
#endif
			/* fall thru for monitor mode handling... */
		} else if (rs->rs_status != 0) {
			if (rs->rs_status & HAL_RXERR_CRC)
				sc->sc_stats.ast_rx_crcerr++;
			if (rs->rs_status & HAL_RXERR_FIFO)
				sc->sc_stats.ast_rx_fifoerr++;
			if (rs->rs_status & HAL_RXERR_PHY) {
				sc->sc_stats.ast_rx_phyerr++;
				phyerr = rs->rs_phyerr & 0x1f;
				sc->sc_stats.ast_rx_phy[phyerr]++;
			}
			if (rs->rs_status & HAL_RXERR_DECRYPT) {
				/*
				 * Decrypt error.  If the error occurred
				 * because there was no hardware key, then
				 * let the frame through so the upper layers
				 * can process it.  This is necessary for 5210
				 * parts which have no way to setup a ``clear''
				 * key cache entry.
				 *
				 * XXX do key cache faulting
				 */
				if (rs->rs_keyix == HAL_RXKEYIX_INVALID)
					goto rx_accept;
				sc->sc_stats.ast_rx_badcrypt++;
			}
			if (rs->rs_status & HAL_RXERR_MIC) {
				sc->sc_stats.ast_rx_badmic++;
				/*
				 * Do minimal work required to hand off
				 * the 802.11 header for notification.
				 */
				/* XXX frag's and QoS frames */
				if (len >= sizeof (struct ieee80211_frame)) {
					bus_dma_sync_single(sc->sc_bdev,
					    bf->bf_skbaddr, len,
					    BUS_DMA_FROMDEVICE);
#if 0
/* XXX revalidate MIC, lookup ni to find VAP */
					ieee80211_notify_michael_failure(ic,
					    (struct ieee80211_frame *)skb->data,
					    sc->sc_splitmic ?
					        rs->rs_keyix - 32 : rs->rs_keyix
					);
#endif
				}
			}
			/*
			 * Reject error frames if we have no vaps that
			 * are operating in monitor mode.
			 */
			if (sc->sc_nmonvaps == 0)
				goto rx_next;
		}
rx_accept:
		/*
		 * Sync and unmap the frame.  At this point we're
		 * committed to passing the sk_buff somewhere so
		 * clear buf_skb; this means a new sk_buff must be
		 * allocated when the rx descriptor is setup again
		 * to receive another frame.
		 */
		bus_dma_sync_single(sc->sc_bdev,
			bf->bf_skbaddr, len, BUS_DMA_FROMDEVICE);

		bus_unmap_single(sc->sc_bdev, bf->bf_skbaddr,
			sc->sc_rxbufsize, BUS_DMA_FROMDEVICE);
		bf->bf_skbaddr = 0;

		bf->bf_skb = NULL;

		sc->sc_stats.ast_ant_rx[rs->rs_antenna]++;
		sc->sc_devstats.rx_packets++;
		sc->sc_devstats.rx_bytes += len;

		skb_put(skb, len);
		skb->protocol = __constant_htons(ETH_P_CONTROL);

#if 0
		if (len < IEEE80211_ACK_LEN) {
			DPRINTF(sc, ATH_DEBUG_RECV,
					"%s: runt packet %d\n", __func__, len);
			sc->sc_stats.ast_rx_tooshort++;
			ieee80211_dev_kfree_skb(&skb);
			goto rx_next;
			if (sc->sc_ic.ic_opmode == IEEE80211_M_MONITOR) {
				/* no other VAPs need the packet */
				ieee80211_dev_kfree_skb(&skb);
				goto rx_next;
			}
		}
#endif
		ath_capture(dev, bf, skb, bf->bf_tsf, 0 /* RX */);

		/*
		 * Finished monitor mode handling, now reject
		 * error frames before passing to other vaps
		 */
		if (rs->rs_status != 0) {
			ieee80211_dev_kfree_skb(&skb);
			goto rx_next;
		}

		/* remove the CRC */
		skb_trim(skb, skb->len - IEEE80211_CRC_LEN);

		/*
		 * From this point on we assume the frame is at least
		 * as large as ieee80211_frame_min; verify that.
		 */
		if (len < IEEE80211_MIN_LEN) {
			DPRINTF(sc, ATH_DEBUG_RECV, "%s: short packet %d\n",
				__func__, len);
			sc->sc_stats.ast_rx_tooshort++;
			ieee80211_dev_kfree_skb(&skb);
			goto rx_next;
		}

		/*
		 * Normal receive.
		 */

		if (IFF_DUMPPKTS(sc, ATH_DEBUG_RECV))
			ieee80211_dump_pkt(ic, skb->data, skb->len,
				   sc->sc_hwmap[rs->rs_rate].ieeerate,
				   rs->rs_rssi);

		{
			struct ieee80211_frame * wh =
				(struct ieee80211_frame *) skb->data;

			/* only print beacons */

			if ((skb->len >= sizeof(struct ieee80211_frame)) &&
			    ((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK)
			     == IEEE80211_FC0_TYPE_MGT) &&
			    ((wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK)
			     == IEEE80211_FC0_SUBTYPE_BEACON)) {

				DPRINTF(sc, ATH_DEBUG_BEACON,
					"%s: RA:" MAC_FMT " TA:" MAC_FMT
					" BSSID:" MAC_FMT "\n",
					__func__, MAC_ADDR(wh->i_addr1),
					MAC_ADDR(wh->i_addr2),
					MAC_ADDR(wh->i_addr3));
			}
		}

		/*
		 * Locate the node for sender, track state, and then
		 * pass the (referenced) node up to the 802.11 layer
		 * for its use.  If the sender is unknown spam the
		 * frame; it'll be dropped where it's not wanted.
		 */
		if (rs->rs_keyix != HAL_RXKEYIX_INVALID &&
		    (ni = sc->sc_keyixmap[rs->rs_keyix]) != NULL) {
			/* Fast path: node is present in the key map;
			 * grab a reference for processing the frame. */
			ni = ieee80211_ref_node(ni);
			ATH_RSSI_LPF(ATH_NODE(ni)->an_avgrssi, rs->rs_rssi);
			type = ieee80211_input(ni, skb, rs->rs_rssi, bf->bf_tsf);
			ieee80211_unref_node(&ni);
		} else {
			/*
			 * No key index or no entry, do a lookup and
			 * add the node to the mapping table if possible.
			 */
			ni = ieee80211_find_rxnode(ic,
				(const struct ieee80211_frame_min *)skb->data);
			if (ni != NULL) {
				struct ath_node *an = ATH_NODE(ni);
				ieee80211_keyix_t keyix;

				ATH_RSSI_LPF(an->an_avgrssi, rs->rs_rssi);
				type = ieee80211_input(ni, skb, rs->rs_rssi, bf->bf_tsf);
				/*
				 * If the station has a key cache slot assigned
				 * update the key->node mapping table.
				 */
				keyix = ni->ni_ucastkey.wk_keyix;
				if (keyix != IEEE80211_KEYIX_NONE &&
				    sc->sc_keyixmap[keyix] == NULL)
					sc->sc_keyixmap[keyix] = ieee80211_ref_node(ni);
				an = NULL;
				ieee80211_unref_node(&ni);
			} else
				type = ieee80211_input_all(ic, skb, rs->rs_rssi, bf->bf_tsf);
		}

		if (sc->sc_diversity) {
			/*
			 * When using hardware fast diversity, change the default rx
			 * antenna if rx diversity chooses the other antenna 3
			 * times in a row.
			 */
			if (sc->sc_defant != rs->rs_antenna) {
				if (++sc->sc_rxotherant >= 3)
					ath_setdefantenna(sc, rs->rs_antenna);
			} else
				sc->sc_rxotherant = 0;
		}
		if (sc->sc_softled) {
			/*
			 * Blink for any data frame.  Otherwise do a
			 * heartbeat-style blink when idle.  The latter
			 * is mainly for station mode where we depend on
			 * periodic beacon frames to trigger the poll event.
			 */
			if (type == IEEE80211_FC0_TYPE_DATA) {
				sc->sc_rxrate = rs->rs_rate;
				ath_led_event(sc, ATH_LED_RX);
			} else if (jiffies - sc->sc_ledevent >= sc->sc_ledidle)
				ath_led_event(sc, ATH_LED_POLL);
		}
rx_next:
		ATH_RXBUF_LOCK_IRQ(sc);
		STAILQ_REMOVE_HEAD(&sc->sc_rxbuf, bf_list);
		ATH_RXBUF_RESET(bf);
		STAILQ_INSERT_TAIL(&sc->sc_rxbuf, bf, bf_list);
		ATH_RXBUF_UNLOCK_IRQ(sc);
	} while (ath_rxbuf_init(sc, bf) == 0);

	/* rx signal state monitoring */
	ath_hal_rxmonitor(ah, &sc->sc_halstats, &sc->sc_curchan);
#undef PA2DESC
}

#ifdef ATH_SUPERG_XR

static void
ath_grppoll_period_update(struct ath_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	u_int16_t interval;
	u_int16_t xrsta;
	u_int16_t normalsta;
	u_int16_t allsta;

	xrsta = ic->ic_xr_sta_assoc;

	/*
	 * if no stations are in XR mode.
	 * use default poll interval.
	 */
	if (xrsta == 0) {
		if (sc->sc_xrpollint != XR_DEFAULT_POLL_INTERVAL) {
			sc->sc_xrpollint = XR_DEFAULT_POLL_INTERVAL;
			ath_grppoll_txq_update(sc, XR_DEFAULT_POLL_INTERVAL);
		}
		return;
	}

	allsta = ic->ic_sta_assoc;
	/*
	 * if all the stations are in XR mode.
	 * use minimum poll interval.
	 */
	if (allsta == xrsta) {
		if (sc->sc_xrpollint != XR_MIN_POLL_INTERVAL) {
			sc->sc_xrpollint = XR_MIN_POLL_INTERVAL;
			ath_grppoll_txq_update(sc, XR_MIN_POLL_INTERVAL);
		}
		return;
	}

	normalsta = allsta-xrsta;
	/*
	 * if stations are in both XR and normal mode.
	 * use some fudge factor.
	 */
	interval = XR_DEFAULT_POLL_INTERVAL -
	    ((XR_DEFAULT_POLL_INTERVAL - XR_MIN_POLL_INTERVAL) * xrsta) / 
	    (normalsta * XR_GRPPOLL_PERIOD_FACTOR);
	if (interval < XR_MIN_POLL_INTERVAL)
		interval = XR_MIN_POLL_INTERVAL;

	if (sc->sc_xrpollint != interval) {
		sc->sc_xrpollint = interval;
		ath_grppoll_txq_update(sc, interval);
	}

	/*
	 * XXX: what if stations go to sleep?
	 * ideally the interval should be adjusted dynamically based on
	 * xr and normal upstream traffic.
	 */
}

/*
 * update grppoll period.
 */
static void
ath_grppoll_txq_update(struct ath_softc *sc, int period)
{
	struct ath_hal *ah = sc->sc_ah;
	HAL_TXQ_INFO qi;
	struct ath_txq *txq = &sc->sc_grpplq;

	if (sc->sc_grpplq.axq_qnum == -1)
		return;

	memset(&qi, 0, sizeof(qi));
	qi.tqi_subtype = 0;
	qi.tqi_aifs = XR_AIFS;
	qi.tqi_cwmin = XR_CWMIN_CWMAX;
	qi.tqi_cwmax = XR_CWMIN_CWMAX;
	qi.tqi_compBuf = 0;
	qi.tqi_cbrPeriod = IEEE80211_TU_TO_MS(period) * 1000; /* usec */
	qi.tqi_cbrOverflowLimit = 2;
	ath_hal_settxqueueprops(ah, txq->axq_qnum, &qi);
	ath_hal_resettxqueue(ah, txq->axq_qnum); /* push to h/w */
}

/*
 * Setup grppoll  h/w transmit queue.
 */
static void
ath_grppoll_txq_setup(struct ath_softc *sc, int qtype, int period)
{
	struct ath_hal *ah = sc->sc_ah;
	HAL_TXQ_INFO qi;
	int qnum;
	u_int compbufsz = 0;
	char *compbuf = NULL;
	dma_addr_t compbufp = 0;
	struct ath_txq *txq = &sc->sc_grpplq;

	memset(&qi, 0, sizeof(qi));
	qi.tqi_subtype = 0;
	qi.tqi_aifs = XR_AIFS;
	qi.tqi_cwmin = XR_CWMIN_CWMAX;
	qi.tqi_cwmax = XR_CWMIN_CWMAX;
	qi.tqi_compBuf = 0;
	qi.tqi_cbrPeriod = IEEE80211_TU_TO_MS(period) * 1000; /* usec */
	qi.tqi_cbrOverflowLimit = 2;

	if (sc->sc_grpplq.axq_qnum == -1) {
		qnum = ath_hal_setuptxqueue(ah, qtype, &qi);
		if (qnum == -1)
			return ;
		if (qnum >= ARRAY_SIZE(sc->sc_txq)) {
			printk("%s: HAL qnum %u out of range, max %u!\n",
				   DEV_NAME(sc->sc_dev), qnum,
				   (unsigned)ARRAY_SIZE(sc->sc_txq));
			ath_hal_releasetxqueue(ah, qnum);
			return;
		}

		txq->axq_qnum = qnum;
	}
	txq->axq_link = NULL;
	STAILQ_INIT(&txq->axq_q);
	ATH_TXQ_LOCK_INIT(txq);
	txq->axq_depth = 0;
	txq->axq_totalqueued = 0;
	txq->axq_intrcnt = 0;
	TAILQ_INIT(&txq->axq_stageq);
	txq->axq_compbuf = compbuf;
	txq->axq_compbufsz = compbufsz;
	txq->axq_compbufp = compbufp;
	ath_hal_resettxqueue(ah, txq->axq_qnum); /* push to h/w */
}

/*
 * Setup group poll frames on the group poll queue.
 */
static void ath_grppoll_start(struct ieee80211vap *vap, int pollcount)
{
	unsigned int i, amode;
	unsigned int flags = 0;
	unsigned int pktlen = 0;
	ath_keyix_t keyix = HAL_TXKEYIX_INVALID;
	unsigned int pollsperrate, pos;
	struct sk_buff *skb = NULL;
	struct ath_buf *bf, *head = NULL;
	struct ieee80211com *ic = vap->iv_ic;
	struct ath_softc *sc = ic->ic_dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	u_int8_t rate;
	unsigned int ctsrate = 0, ctsduration = 0;
	const HAL_RATE_TABLE *rt;
	u_int8_t cix, rtindex = 0;
	u_int type;
	struct ath_txq *txq = &sc->sc_grpplq;
	struct ath_desc *ds = NULL;
	int rates[XR_NUM_RATES];
	u_int8_t ratestr[16], numpollstr[16];
	struct rate_to_str_map {
		u_int8_t str[4];
		int ratekbps;
	};

	static const struct rate_to_str_map ratestrmap[] = {
		{"0.25",    250},
		{ ".25",    250},
		{ "0.5",    500},
		{  ".5",    500},
		{   "1",   1000},
		{   "3",   3000},
		{   "6",   6000},
		{   "?",      0},
	};

#define MAX_GRPPOLL_RATE 5
#define	USE_SHPREAMBLE(_ic) \
	(((_ic)->ic_flags & (IEEE80211_F_SHPREAMBLE | IEEE80211_F_USEBARKER)) \
		== IEEE80211_F_SHPREAMBLE)

	if (sc->sc_xrgrppoll)
		return;


	memset(&rates, 0, sizeof(rates));
	pos = 0;
	while (sscanf(&(sc->sc_grppoll_str[pos]), "%s %s", 
				ratestr, numpollstr) == 2) {
		unsigned int rtx = 0;
		while (ratestrmap[rtx].ratekbps != 0) {
			if (strcmp(ratestrmap[rtx].str, ratestr) == 0)
				break;
			rtx++;
		}
		sscanf(numpollstr, "%d", &(rates[rtx]));
		pos += strlen(ratestr) + strlen(numpollstr) + 2;
	}
	if (!sc->sc_grppolldma.dd_bufptr) {
		printk("grppoll_start: grppoll Buf allocation failed\n");
		return;
	}
	rt = sc->sc_currates;
	cix = rt->info[sc->sc_protrix].controlRate;
	ctsrate = rt->info[cix].rateCode;
	if (USE_SHPREAMBLE(ic))
			ctsrate |= rt->info[cix].shortPreamble;
	rt = sc->sc_xr_rates;
	/* queue the group polls for each antenna mode. set the right keycache 
	 * index for the broadcast packets. this will ensure that if the first 
	 * poll does not elicit a single chirp from any XR station, hardware 
	 * will not send the subsequent polls. */
	pollsperrate = 0;
	for (amode = HAL_ANTENNA_FIXED_A; amode < HAL_ANTENNA_MAX_MODE; amode++) {
		for (i = 0; i < (pollcount + 1); i++) {
			flags = HAL_TXDESC_NOACK;
			rate = rt->info[rtindex].rateCode;
			/*
			 * except for the last one every thing else is a CF poll.
			 * last one is  the CF End frame.
			 */

			if (i == pollcount) {
				skb = ieee80211_getcfframe(vap, 
						IEEE80211_FC0_SUBTYPE_CF_END);
				rate = ctsrate;
				ctsduration = ath_hal_computetxtime(ah,
					sc->sc_currates, pktlen, sc->sc_protrix, 
					AH_FALSE);
			} else {
				skb = ieee80211_getcfframe(vap, 
						IEEE80211_FC0_SUBTYPE_CFPOLL);
				pktlen = skb->len + IEEE80211_CRC_LEN;
				/* The very first group poll ctsduration 
				 * should be enough to allow an auth frame 
				 * from station. This is to pass the wifi 
				 * testing (as some stations in testing do 
				 * not honor CF_END and rely on CTS duration). */
				if (i == 0 && amode == HAL_ANTENNA_FIXED_A) {
					ctsduration = ath_hal_computetxtime(ah,	
							rt, pktlen, rtindex,
							AH_FALSE) + 	/* CF-Poll time */
						(XR_AIFS + 
						 (XR_CWMIN_CWMAX * XR_SLOT_DELAY)) +
						ath_hal_computetxtime(ah, rt,
							2 * (sizeof(struct ieee80211_frame_min) + 6),
							IEEE80211_XR_DEFAULT_RATE_INDEX,
							AH_FALSE) +	/* Auth packet time */
						ath_hal_computetxtime(ah, rt,
							IEEE80211_ACK_LEN,
							IEEE80211_XR_DEFAULT_RATE_INDEX,
							AH_FALSE); 	/* ACK. frame time */
				} else {
					ctsduration = ath_hal_computetxtime(ah, rt,
							pktlen, rtindex,
							AH_FALSE) +	/* CF-Poll time */
						(XR_AIFS + (XR_CWMIN_CWMAX * XR_SLOT_DELAY)) + 
						ath_hal_computetxtime(ah, rt,
							XR_FRAGMENTATION_THRESHOLD,
							IEEE80211_XR_DEFAULT_RATE_INDEX,
							AH_FALSE) +	/* Data packet time */
						ath_hal_computetxtime(ah, rt,
							IEEE80211_ACK_LEN,
							IEEE80211_XR_DEFAULT_RATE_INDEX,
							AH_FALSE);	/* ACK frame time */
				}
				if ((vap->iv_flags & IEEE80211_F_PRIVACY) &&
						(keyix == HAL_TXKEYIX_INVALID)) {
					struct ieee80211_key *k;
					k = ieee80211_crypto_encap(vap->iv_bss, skb);
					if (k)
						keyix = ATH_KEY(k->wk_keyix);
				}
			}

			ATH_TXBUF_LOCK_IRQ(sc);
			bf = STAILQ_FIRST(&sc->sc_grppollbuf);
			if (bf != NULL)
				STAILQ_REMOVE_HEAD(&sc->sc_grppollbuf, bf_list);
			else {
				DPRINTF(sc, ATH_DEBUG_XMIT, 
						"%s: No more TxBufs\n", __func__);
				ATH_TXBUF_UNLOCK_IRQ_EARLY(sc);
				return;
			}
			/* XXX use a counter and leave at least one for mgmt frames */
			if (STAILQ_EMPTY(&sc->sc_grppollbuf)) {
				DPRINTF(sc, ATH_DEBUG_XMIT, 
						"%s: No more TxBufs left\n", __func__);
				ATH_TXBUF_UNLOCK_IRQ_EARLY(sc);
				return;
			}
			ATH_TXBUF_UNLOCK_IRQ(sc);

			bf->bf_skbaddr = bus_map_single(sc->sc_bdev,
				skb->data, skb->len, BUS_DMA_TODEVICE);
			bf->bf_skb = skb;
			ATH_TXQ_INSERT_TAIL(txq, bf, bf_list);
			ds = bf->bf_desc;
			ds->ds_data = bf->bf_skbaddr;
			if (i == pollcount && amode == (HAL_ANTENNA_MAX_MODE - 1)) {
				type = HAL_PKT_TYPE_NORMAL;
				flags |= (HAL_TXDESC_CLRDMASK | HAL_TXDESC_VEOL);
			} else {
				flags |= HAL_TXDESC_CTSENA;
				type = HAL_PKT_TYPE_GRP_POLL;
			}
			if (i == 0 && amode == HAL_ANTENNA_FIXED_A) {
				flags |= HAL_TXDESC_CLRDMASK;
				head = bf;
			}
			ath_hal_setuptxdesc(ah, ds,
				skb->len + IEEE80211_CRC_LEN,	/* frame length */
				sizeof(struct ieee80211_frame), /* header length */
				type,				/* Atheros packet type */
				ic->ic_txpowlimit,		/* max txpower */
				rate, 0,			/* series 0 rate/tries */
				keyix,				/* key index */
				amode,				/* antenna mode */
				flags,
				ctsrate,			/* rts/cts rate */
				ctsduration,			/* rts/cts duration */
				0,				/* comp icv len */
				0,				/* comp iv len */
				ATH_COMP_PROC_NO_COMP_NO_CCS	/* comp scheme */
				);
			ath_hal_filltxdesc(ah, ds,
				roundup(skb->len, 4),	/* buffer length */
				AH_TRUE,		/* first segment */
				AH_TRUE,		/* last segment */
				ds			/* first descriptor */
				);
			/* NB: The desc swap function becomes void,
			 * if descriptor swapping is not enabled */
			ath_desc_swap(ds);
			if (txq->axq_link) {
#ifdef AH_NEED_DESC_SWAP
				*txq->axq_link = cpu_to_le32(bf->bf_daddr);
#else
				*txq->axq_link = bf->bf_daddr;
#endif
			}
			txq->axq_link = &ds->ds_link;
			pollsperrate++;
			if (pollsperrate > rates[rtindex]) {
				rtindex = (rtindex + 1) % MAX_GRPPOLL_RATE;
				pollsperrate = 0;
			}
		}
	}
	/* make it circular */
#ifdef AH_NEED_DESC_SWAP
	ds->ds_link = cpu_to_le32(head->bf_daddr);
#else
	ds->ds_link = head->bf_daddr;
#endif
	/* start the queue */
	ath_hal_puttxbuf(ah, txq->axq_qnum, head->bf_daddr);
	ath_hal_txstart(ah, txq->axq_qnum);
	sc->sc_xrgrppoll = 1;
#undef USE_SHPREAMBLE
}

static void ath_grppoll_stop(struct ieee80211vap *vap)
{
	struct ieee80211com *ic = vap->iv_ic;
	struct ath_softc *sc = ic->ic_dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ath_txq *txq = &sc->sc_grpplq;
	struct ath_buf *bf;


	if (!sc->sc_xrgrppoll)
		return;
	ath_hal_stoptxdma(ah, txq->axq_qnum);

	/* move the grppoll bufs back to the grppollbuf */
	for (;;) {
		ATH_TXQ_LOCK_IRQ(txq);
		bf = STAILQ_FIRST(&txq->axq_q);
		if (bf == NULL) {
			txq->axq_link = NULL;
			ATH_TXQ_UNLOCK_IRQ_EARLY(txq);
			goto bf_fail;
		}
		ATH_TXQ_REMOVE_HEAD(txq, bf_list);
		ATH_TXQ_UNLOCK_IRQ(txq);

		cleanup_ath_buf(sc, bf, BUS_DMA_TODEVICE);
		ATH_TXBUF_LOCK_IRQ(sc);
		STAILQ_INSERT_TAIL(&sc->sc_grppollbuf, bf, bf_list);
		ATH_TXBUF_UNLOCK_IRQ(sc);
	}
bf_fail:

	STAILQ_INIT(&txq->axq_q);
	txq->axq_depth = 0;
	txq->axq_totalqueued = 0;
	txq->axq_intrcnt = 0;
	TAILQ_INIT(&txq->axq_stageq);
	sc->sc_xrgrppoll = 0;
}
#endif

/*
 * Setup a h/w transmit queue.
 */
static struct ath_txq *
ath_txq_setup(struct ath_softc *sc, int qtype, int subtype)
{
	struct ath_hal *ah = sc->sc_ah;
	HAL_TXQ_INFO qi;
	int qnum;
	u_int compbufsz = 0;
	char *compbuf = NULL;
	dma_addr_t compbufp = 0;

	memset(&qi, 0, sizeof(qi));
	qi.tqi_subtype = subtype;
	qi.tqi_aifs = HAL_TXQ_USEDEFAULT;
	qi.tqi_cwmin = HAL_TXQ_USEDEFAULT;
	qi.tqi_cwmax = HAL_TXQ_USEDEFAULT;
	qi.tqi_compBuf = 0;
#ifdef ATH_SUPERG_XR
	if (subtype == HAL_XR_DATA) {
		qi.tqi_aifs  = XR_DATA_AIFS;
		qi.tqi_cwmin = XR_DATA_CWMIN;
		qi.tqi_cwmax = XR_DATA_CWMAX;
	}
#endif

#ifdef ATH_SUPERG_COMP
	/* allocate compression scratch buffer for data queues */
	if (((qtype == HAL_TX_QUEUE_DATA)|| (qtype == HAL_TX_QUEUE_UAPSD))
	    && ath_hal_compressionsupported(ah)) {
		compbufsz = roundup(HAL_COMP_BUF_MAX_SIZE,
			HAL_COMP_BUF_ALIGN_SIZE) + HAL_COMP_BUF_ALIGN_SIZE;
		compbuf = (char *)bus_alloc_consistent(sc->sc_bdev,
			compbufsz, &compbufp);
		if (compbuf == NULL)
			sc->sc_ic.ic_ath_cap &= ~IEEE80211_ATHC_COMP;
		else
			qi.tqi_compBuf = (u_int32_t)compbufp;
	}
#endif
	/*
	 * Enable interrupts only for EOL and DESC conditions.
	 * We mark tx descriptors to receive a DESC interrupt
	 * when a tx queue gets deep; otherwise waiting for the
	 * EOL to reap descriptors.  Note that this is done to
	 * reduce interrupt load and this only defers reaping
	 * descriptors, never transmitting frames.  Aside from
	 * reducing interrupts this also permits more concurrency.
	 * The only potential downside is if the tx queue backs
	 * up in which case the top half of the kernel may backup
	 * due to a lack of tx descriptors.
	 *
	 * The UAPSD queue is an exception, since we take a desc-
	 * based intr on the EOSP frames.
	 */
	if (qtype == HAL_TX_QUEUE_UAPSD)
		qi.tqi_qflags = HAL_TXQ_TXDESCINT_ENABLE;
	else
		qi.tqi_qflags = HAL_TXQ_TXEOLINT_ENABLE | 
			HAL_TXQ_TXDESCINT_ENABLE;
	qnum = ath_hal_setuptxqueue(ah, qtype, &qi);
	if (qnum == -1) {
		/*
		 * NB: don't print a message, this happens
		 * normally on parts with too few tx queues
		 */
#ifdef ATH_SUPERG_COMP
		if (compbuf) {
			bus_free_consistent(sc->sc_bdev, compbufsz,
				compbuf, compbufp);
		}
#endif
		return NULL;
	}
	if (qnum >= ARRAY_SIZE(sc->sc_txq)) {
		printk("%s: HAL qnum %u out of range, max %u!\n",
			DEV_NAME(sc->sc_dev), qnum,
			(unsigned)ARRAY_SIZE(sc->sc_txq));
#ifdef ATH_SUPERG_COMP
		if (compbuf) {
			bus_free_consistent(sc->sc_bdev, compbufsz,
				compbuf, compbufp);
		}
#endif
		ath_hal_releasetxqueue(ah, qnum);
		return NULL;
	}
	if (!ATH_TXQ_SETUP(sc, qnum)) {
		struct ath_txq *txq = &sc->sc_txq[qnum];

		txq->axq_qnum = qnum;
		txq->axq_link = NULL;
		STAILQ_INIT(&txq->axq_q);
		ATH_TXQ_LOCK_INIT(txq);
		txq->axq_depth = 0;
		txq->axq_totalqueued = 0;
		txq->axq_intrcnt = 0;
		TAILQ_INIT(&txq->axq_stageq);
		txq->axq_compbuf = compbuf;
		txq->axq_compbufsz = compbufsz;
		txq->axq_compbufp = compbufp;
		sc->sc_txqsetup |= 1 << qnum;
	}
	return &sc->sc_txq[qnum];
}

/*
 * Setup a hardware data transmit queue for the specified
 * access control.  The HAL may not support all requested
 * queues in which case it will return a reference to a
 * previously setup queue.  We record the mapping from ACs
 * to H/W queues for use by ath_tx_start and also track
 * the set of H/W queues being used to optimize work in the
 * transmit interrupt handler and related routines.
 */
static int
ath_tx_setup(struct ath_softc *sc, int ac, int haltype)
{
	struct ath_txq *txq;

	if (ac >= ARRAY_SIZE(sc->sc_ac2q)) {
		printk("%s: AC %u out of range, max %u!\n",
		       DEV_NAME(sc->sc_dev), ac,
		       (unsigned)ARRAY_SIZE(sc->sc_ac2q));
		return 0;
	}
	txq = ath_txq_setup(sc, HAL_TX_QUEUE_DATA, haltype);
	if (txq != NULL) {
		sc->sc_ac2q[ac] = txq;
		return 1;
	} else
		return 0;
}

/*
 * Update WME parameters for a transmit queue.
 */
static int
ath_txq_update(struct ath_softc *sc, struct ath_txq *txq, int ac)
{
#define	ATH_EXPONENT_TO_VALUE(v)	((1<<v)-1)
#define	ATH_TXOP_TO_US(v)		(v<<5)
	struct ieee80211com *ic = &sc->sc_ic;
	struct wmeParams *wmep = &ic->ic_wme.wme_chanParams.cap_wmeParams[ac];
	struct ath_hal *ah = sc->sc_ah;
	HAL_TXQ_INFO qi;

	ath_hal_gettxqueueprops(ah, txq->axq_qnum, &qi);
	qi.tqi_aifs = wmep->wmep_aifsn;
	qi.tqi_cwmin = ATH_EXPONENT_TO_VALUE(wmep->wmep_logcwmin);
	qi.tqi_cwmax = ATH_EXPONENT_TO_VALUE(wmep->wmep_logcwmax);
	qi.tqi_burstTime = ATH_TXOP_TO_US(wmep->wmep_txopLimit);

	if (!ath_hal_settxqueueprops(ah, txq->axq_qnum, &qi)) {
		printk("%s: unable to update hardware queue "
			"parameters for %s traffic!\n",
			DEV_NAME(sc->sc_dev), ieee80211_wme_acnames[ac]);
		return 0;
	} else {
		ath_hal_resettxqueue(ah, txq->axq_qnum); /* push to h/w */
		return 1;
	}
#undef ATH_TXOP_TO_US
#undef ATH_EXPONENT_TO_VALUE
}

/*
 * Callback from the 802.11 layer to update WME parameters.
 */
static int
ath_wme_update(struct ieee80211com *ic)
{
	struct ath_softc *sc = ic->ic_dev->priv;

	if (sc->sc_uapsdq)
		ath_txq_update(sc, sc->sc_uapsdq, WME_AC_VO);

	return !ath_txq_update(sc, sc->sc_ac2q[WME_AC_BE], WME_AC_BE) ||
	    !ath_txq_update(sc, sc->sc_ac2q[WME_AC_BK], WME_AC_BK) ||
	    !ath_txq_update(sc, sc->sc_ac2q[WME_AC_VI], WME_AC_VI) ||
	    !ath_txq_update(sc, sc->sc_ac2q[WME_AC_VO], WME_AC_VO) ? EIO : 0;
}

/*
 * Callback from 802.11 layer to flush a node's U-APSD queues
 */
static void
ath_uapsd_flush(struct ieee80211_node *ni)
{
	struct ath_node *an = ATH_NODE(ni);
	struct ath_buf *bf;
	struct ath_softc *sc = ni->ni_ic->ic_dev->priv;
	struct ath_txq *txq;

	ATH_NODE_UAPSD_LOCK_IRQ(an);
	/*
	 * NB: could optimize for successive runs from the same AC
	 *     if we can assume that is the most frequent case.
	 */
	while (an->an_uapsd_qdepth) {
		bf = STAILQ_FIRST(&an->an_uapsd_q);
		STAILQ_REMOVE_HEAD(&an->an_uapsd_q, bf_list);
		bf->bf_desc->ds_link = 0;
		txq = sc->sc_ac2q[bf->bf_skb->priority & 0x3];
		ath_tx_txqaddbuf(sc, ni, txq, bf, bf->bf_desc, bf->bf_skb->len);
		an->an_uapsd_qdepth--;
	}

	while (an->an_uapsd_overflowqdepth) {
		bf = STAILQ_FIRST(&an->an_uapsd_overflowq);
		STAILQ_REMOVE_HEAD(&an->an_uapsd_overflowq, bf_list);
		bf->bf_desc->ds_link = 0;
		txq = sc->sc_ac2q[bf->bf_skb->priority & 0x3];
		ath_tx_txqaddbuf(sc, ni, txq, bf, bf->bf_desc, bf->bf_skb->len);
		an->an_uapsd_overflowqdepth--;
	}
	if (IEEE80211_NODE_UAPSD_USETIM(ni))
		ni->ni_vap->iv_set_tim(ni, 0);
	ATH_NODE_UAPSD_UNLOCK_IRQ(an);
}

/*
 * Reclaim resources for a setup queue.
 */
static void
ath_tx_cleanupq(struct ath_softc *sc, struct ath_txq *txq)
{

#ifdef ATH_SUPERG_COMP
	/* Release compression buffer */
	if (txq->axq_compbuf) {
		bus_free_consistent(sc->sc_bdev, txq->axq_compbufsz,
			txq->axq_compbuf, txq->axq_compbufp);
		txq->axq_compbuf = NULL;
	}
#endif
	ath_hal_releasetxqueue(sc->sc_ah, txq->axq_qnum);
	ATH_TXQ_LOCK_DESTROY(txq);
	sc->sc_txqsetup &= ~(1 << txq->axq_qnum);
}

/*
 * Reclaim all tx queue resources.
 */
static void
ath_tx_cleanup(struct ath_softc *sc)
{
	unsigned int i;

	ATH_TXBUF_LOCK_DESTROY(sc);
	for (i = 0; i < HAL_NUM_TX_QUEUES; i++)
		if (ATH_TXQ_SETUP(sc, i))
			ath_tx_cleanupq(sc, &sc->sc_txq[i]);
}

#ifdef ATH_SUPERG_COMP
static u_int32_t
ath_get_icvlen(struct ieee80211_key *k)
{
	const struct ieee80211_cipher *cip = k->wk_cipher;

	if (cip->ic_cipher == IEEE80211_CIPHER_AES_CCM ||
		cip->ic_cipher == IEEE80211_CIPHER_AES_OCB)
		return AES_ICV_FIELD_SIZE;

	return WEP_ICV_FIELD_SIZE;
}

static u_int32_t
ath_get_ivlen(struct ieee80211_key *k)
{
	const struct ieee80211_cipher *cip = k->wk_cipher;
	u_int32_t ivlen;

	ivlen = WEP_IV_FIELD_SIZE;

	if (cip->ic_cipher == IEEE80211_CIPHER_AES_CCM ||
		cip->ic_cipher == IEEE80211_CIPHER_AES_OCB)
		ivlen += EXT_IV_FIELD_SIZE;

	return ivlen;
}
#endif

/*
 * Get transmit rate index using rate in Kbps
 */
static __inline int
ath_tx_findindex(const HAL_RATE_TABLE *rt, int rate)
{
	unsigned int i, ndx = 0;

	for (i = 0; i < rt->rateCount; i++) {
		if (rt->info[i].rateKbps == rate) {
			ndx = i;
			break;
		}
	}

	return ndx;
}

/*
 * XXX: Needs external locking!
 */
static void
ath_tx_uapsdqueue(struct ath_softc *sc, struct ath_node *an, struct ath_buf *bf)
{
	struct ath_buf *lastbuf;

	/* case the delivery queue just sent and can move overflow q over */
	if (an->an_uapsd_qdepth == 0 && an->an_uapsd_overflowqdepth != 0) {
		DPRINTF(sc, ATH_DEBUG_UAPSD,
			"%s: delivery Q empty, replacing with overflow Q\n",
			__func__);
		STAILQ_CONCAT(&an->an_uapsd_q, &an->an_uapsd_overflowq);
		an->an_uapsd_qdepth = an->an_uapsd_overflowqdepth;
		an->an_uapsd_overflowqdepth = 0;
	}

	/* most common case - room on delivery q */
	if (an->an_uapsd_qdepth < an->an_node.ni_uapsd_maxsp) {
		/* add to delivery q */
		if ((lastbuf = STAILQ_LAST(&an->an_uapsd_q, ath_buf, bf_list))) {
#ifdef AH_NEED_DESC_SWAP
			lastbuf->bf_desc->ds_link = cpu_to_le32(bf->bf_daddr);
#else
			lastbuf->bf_desc->ds_link = bf->bf_daddr;
#endif
		}
		STAILQ_INSERT_TAIL(&an->an_uapsd_q, bf, bf_list);
		an->an_uapsd_qdepth++;
		DPRINTF(sc, ATH_DEBUG_UAPSD,
				"%s: added AC %d frame to delivery Q, "
				"new depth = %d\n", __func__, 
				bf->bf_skb->priority, an->an_uapsd_qdepth);
		return;
	}

	/* check if need to make room on overflow queue */
	if (an->an_uapsd_overflowqdepth == an->an_node.ni_uapsd_maxsp) {
		/*
		 *  pop oldest from delivery queue and cleanup
		 */
		lastbuf = STAILQ_FIRST(&an->an_uapsd_q);
		STAILQ_REMOVE_HEAD(&an->an_uapsd_q, bf_list);
		ath_return_txbuf(sc, &lastbuf);

		/*
		 *  move oldest from overflow to delivery
		 */
		lastbuf = STAILQ_FIRST(&an->an_uapsd_overflowq);
		STAILQ_REMOVE_HEAD(&an->an_uapsd_overflowq, bf_list);
		an->an_uapsd_overflowqdepth--;
		STAILQ_INSERT_TAIL(&an->an_uapsd_q, lastbuf, bf_list);
		DPRINTF(sc, ATH_DEBUG_UAPSD,
			"%s: delivery and overflow Qs full, dropped oldest\n",
			__func__);
	}

	/* add to overflow q */
	if ((lastbuf = STAILQ_LAST(&an->an_uapsd_overflowq, ath_buf, bf_list))) {
#ifdef AH_NEED_DESC_SWAP
		lastbuf->bf_desc->ds_link = cpu_to_le32(bf->bf_daddr);
#else
		lastbuf->bf_desc->ds_link = bf->bf_daddr;
#endif
	}
	STAILQ_INSERT_TAIL(&an->an_uapsd_overflowq, bf, bf_list);
	an->an_uapsd_overflowqdepth++;
	DPRINTF(sc, ATH_DEBUG_UAPSD, "%s: added AC %d to overflow Q, "
		"new depth = %d\n", __func__, 
		bf->bf_skb->priority, an->an_uapsd_overflowqdepth);

	return;
}

static int
ath_tx_start(struct net_device *dev, struct ieee80211_node *ni, 
		struct ath_buf *bf, struct sk_buff *skb, int nextfraglen)
{
#define	MIN(a,b)	((a) < (b) ? (a) : (b))
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = ni->ni_ic;
	struct ieee80211vap *vap = ni->ni_vap;
	struct ath_hal *ah = sc->sc_ah;
	int isprot, ismcast, istxfrag;
	unsigned int try0, hdrlen, pktlen, comp = ATH_COMP_PROC_NO_COMP_NO_CCS;
	ath_keyix_t keyix;
	u_int8_t rix, txrate, ctsrate;
	u_int32_t ivlen = 0, icvlen = 0;
	u_int8_t cix = 0xff;
	struct ath_desc *ds = NULL;
	struct ath_txq *txq = NULL;
	struct ieee80211_frame *wh;
	u_int subtype, flags, ctsduration;
	HAL_PKT_TYPE atype;
	const HAL_RATE_TABLE *rt;
	HAL_BOOL shortPreamble;
	struct ath_node *an;
	struct ath_vap *avp = ATH_VAP(vap);
	u_int8_t antenna;
	struct ieee80211_mrr mrr;

	wh = (struct ieee80211_frame *)skb->data;
	isprot = wh->i_fc[1] & IEEE80211_FC1_PROT;
	ismcast = IEEE80211_IS_MULTICAST(wh->i_addr1);
	hdrlen = ieee80211_anyhdrsize(wh);
	istxfrag = (wh->i_fc[1] & IEEE80211_FC1_MORE_FRAG) ||
		(((le16toh(*(__le16 *)&wh->i_seq[0]) >>
		IEEE80211_SEQ_FRAG_SHIFT) & IEEE80211_SEQ_FRAG_MASK) > 0);

	pktlen = skb->len;
#ifdef ATH_SUPERG_FF
	{
		struct sk_buff *skbtmp = skb;
		while ((skbtmp = skbtmp->next))
			pktlen += skbtmp->len;
	}
#endif
	/*
	 * Packet length must not include any
	 * pad bytes; deduct them here.
	 */
	pktlen -= (hdrlen & 3);

	if (isprot) {
		const struct ieee80211_cipher *cip;
		struct ieee80211_key *k;

		/*
		 * Construct the 802.11 header+trailer for an encrypted
		 * frame. The only reason this can fail is because of an
		 * unknown or unsupported cipher/key type.
		 */

		/* FFXXX: change to handle linked skbs */
		k = ieee80211_crypto_encap(ni, skb);
		if (k == NULL) {
			/*
			 * This can happen when the key is yanked after the
			 * frame was queued.  Just discard the frame; the
			 * 802.11 layer counts failures and provides
			 * debugging/diagnostics.
			 */
			return -EIO;
		}
		/*
		 * Adjust the packet + header lengths for the crypto
		 * additions and calculate the h/w key index.  When
		 * a s/w mic is done the frame will have had any mic
		 * added to it prior to entry so skb->len above will
		 * account for it. Otherwise we need to add it to the
		 * packet length.
		 */
		cip = k->wk_cipher;
		hdrlen += cip->ic_header;
		pktlen += cip->ic_header + cip->ic_trailer;
		if ((k->wk_flags & IEEE80211_KEY_SWMIC) == 0) {
			if (!istxfrag)
				pktlen += cip->ic_miclen;
			else
				if (cip->ic_cipher != IEEE80211_CIPHER_TKIP)
					pktlen += cip->ic_miclen;
		}
		keyix = ATH_KEY(k->wk_keyix);

#ifdef ATH_SUPERG_COMP
		icvlen = ath_get_icvlen(k) / 4;
		ivlen = ath_get_ivlen(k) / 4;
#endif
		/* packet header may have moved, reset our local pointer */
		wh = (struct ieee80211_frame *)skb->data;
	} else if (ni->ni_ucastkey.wk_cipher == &ieee80211_cipher_none) {
		/*
		 * Use station key cache slot, if assigned.
		 */
		keyix = ATH_KEY(ni->ni_ucastkey.wk_keyix);
	} else
		keyix = HAL_TXKEYIX_INVALID;

	pktlen += IEEE80211_CRC_LEN;

	/*
	 * Load the DMA map so any coalescing is done.  This
	 * also calculates the number of descriptors we need.
	 */
#ifndef ATH_SUPERG_FF
	bf->bf_skbaddr = bus_map_single(sc->sc_bdev,
		skb->data, pktlen, BUS_DMA_TODEVICE);
	DPRINTF(sc, ATH_DEBUG_XMIT, "%s: skb %p [data %p len %u] skbaddr %llx\n",
		__func__, skb, skb->data, skb->len, ito64(bf->bf_skbaddr));
#else /* ATH_SUPERG_FF case */
	bf->bf_skbaddr = bus_map_single(sc->sc_bdev,
		skb->data, skb->len, BUS_DMA_TODEVICE);
	DPRINTF(sc, ATH_DEBUG_XMIT, "%s: skb %p [data %p len %u] skbaddr %llx\n",
		__func__, skb, skb->data, skb->len, ito64(bf->bf_skbaddr));
	/* NB: ensure skb->len had been updated for each skb so we don't need pktlen */
	{
		struct sk_buff *skbtmp = skb;
		unsigned int i = 0;

		while ((skbtmp = skbtmp->next)) {
			bf->bf_skbaddrff[i] = bus_map_single(sc->sc_bdev,
				skbtmp->data, skbtmp->len, BUS_DMA_TODEVICE);
			DPRINTF(sc, ATH_DEBUG_XMIT, "%s: skb%d (FF) %p "
				"[data %p len %u] skbaddr %llx\n", __func__, 
				i, skbtmp, skbtmp->data, skbtmp->len,
				ito64(bf->bf_skbaddrff[i]));
			i++;
		}
		bf->bf_numdescff = i;
	}
#endif /* ATH_SUPERG_FF */
	bf->bf_skb = skb;
	bf->bf_node = ieee80211_ref_node(ni);

	/* setup descriptors */
	ds = bf->bf_desc;
#ifdef ATH_SUPERG_XR
	if (vap->iv_flags & IEEE80211_F_XR)
		rt = sc->sc_xr_rates;
	else
		rt = sc->sc_currates;
#else
	rt = sc->sc_currates;
#endif
	KASSERT(rt != NULL, ("no rate table, mode %u", sc->sc_curmode));

	/*
	 * NB: the 802.11 layer marks whether or not we should
	 * use short preamble based on the current mode and
	 * negotiated parameters.
	 */
	if ((ic->ic_flags & IEEE80211_F_SHPREAMBLE) &&
	    (ni->ni_capinfo & IEEE80211_CAPINFO_SHORT_PREAMBLE)) {
		shortPreamble = AH_TRUE;
		sc->sc_stats.ast_tx_shortpre++;
	} else
		shortPreamble = AH_FALSE;

	an = ATH_NODE(ni);
	flags = HAL_TXDESC_CLRDMASK;		/* XXX needed for crypto errs */
	/*
	 * Calculate Atheros packet type from IEEE80211 packet header,
	 * setup for rate calculations, and select h/w transmit queue.
	 */
	switch (wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) {
	case IEEE80211_FC0_TYPE_MGT:
		subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;
		if (subtype == IEEE80211_FC0_SUBTYPE_BEACON)
			atype = HAL_PKT_TYPE_BEACON;
		else if (subtype == IEEE80211_FC0_SUBTYPE_PROBE_RESP)
			atype = HAL_PKT_TYPE_PROBE_RESP;
		else if (subtype == IEEE80211_FC0_SUBTYPE_ATIM)
			atype = HAL_PKT_TYPE_ATIM;
		else
			atype = HAL_PKT_TYPE_NORMAL;	/* XXX */
		rix = sc->sc_minrateix;
		txrate = rt->info[rix].rateCode;
		if (shortPreamble)
			txrate |= rt->info[rix].shortPreamble;
		try0 = ATH_TXMAXTRY;

		if (ni->ni_flags & IEEE80211_NODE_QOS) {
			/* NB: force all management frames to highest queue */
			txq = sc->sc_ac2q[WME_AC_VO];
		} else
			txq = sc->sc_ac2q[WME_AC_BE];
		break;
	case IEEE80211_FC0_TYPE_CTL:
		atype = HAL_PKT_TYPE_PSPOLL;	/* stop setting of duration */
		rix = sc->sc_minrateix;
		txrate = rt->info[rix].rateCode;
		if (shortPreamble)
			txrate |= rt->info[rix].shortPreamble;
		try0 = ATH_TXMAXTRY;

		if (ni->ni_flags & IEEE80211_NODE_QOS) {
			/* NB: force all ctl frames to highest queue */
			txq = sc->sc_ac2q[WME_AC_VO];
		} else
			txq = sc->sc_ac2q[WME_AC_BE];
		break;
	case IEEE80211_FC0_TYPE_DATA:
		atype = HAL_PKT_TYPE_NORMAL;		/* default */

		if (ismcast) {
			rix = ath_tx_findindex(rt, vap->iv_mcast_rate);
			txrate = rt->info[rix].rateCode;
			if (shortPreamble)
				txrate |= rt->info[rix].shortPreamble;
			/*
			 * ATH_TXMAXTRY disables Multi-rate retries, which
			 * isn't applicable to mcast packets and overrides
			 * the desired transmission rate for mcast traffic.
			 */
			try0 = ATH_TXMAXTRY;
		} else {
			/*
			 * Data frames; consult the rate control module.
			 */
			sc->sc_rc->ops->findrate(sc, an, shortPreamble, skb->len,
				&rix, &try0, &txrate);

			/* Ratecontrol sometimes returns invalid rate index */
			if (rix != 0xff)
				an->an_prevdatarix = rix;
			else
				rix = an->an_prevdatarix;
		}

		if (M_FLAG_GET(skb, M_UAPSD)) {
			/* U-APSD frame, handle txq later */
			break;
		}

		/*
		 * Default all non-QoS traffic to the best-effort queue.
		 */
		if (wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_QOS) {
			/* XXX validate skb->priority, remove mask */
			txq = sc->sc_ac2q[skb->priority & 0x3];
			if (ic->ic_wme.wme_wmeChanParams.cap_wmeParams[skb->priority].wmep_noackPolicy) {
				flags |= HAL_TXDESC_NOACK;
				sc->sc_stats.ast_tx_noack++;
			}
		} else
			txq = sc->sc_ac2q[WME_AC_BE];
		break;
	default:
		printk("%s: bogus frame type 0x%x (%s)\n", DEV_NAME(dev),
			wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK, __func__);
		/* XXX statistic */
		return -EIO;
	}

#ifdef ATH_SUPERG_XR
	if (vap->iv_flags & IEEE80211_F_XR) {
		txq = sc->sc_xrtxq;
		if (!txq)
			txq = sc->sc_ac2q[WME_AC_BK];
		flags |= HAL_TXDESC_CTSENA;
		cix = rt->info[sc->sc_protrix].controlRate;
	}
#endif
	/*
	 * When servicing one or more stations in power-save mode (or)
	 * if there is some mcast data waiting on mcast queue
	 * (to prevent out of order delivery of mcast/bcast packets)
	 * multicast frames must be buffered until after the beacon.
	 * We use the private mcast queue for that.
	 */
	if (ismcast && (vap->iv_ps_sta || avp->av_mcastq.axq_depth)) {
		txq = &avp->av_mcastq;
		/* XXX? more bit in 802.11 frame header */
	}

	/*
	 * Calculate miscellaneous flags.
	 */
	if (ismcast) {
		flags |= HAL_TXDESC_NOACK;	/* no ack on broad/multicast */
		sc->sc_stats.ast_tx_noack++;
		try0 = ATH_TXMAXTRY;    /* turn off multi-rate retry for multicast traffic */
	 } else if (pktlen > vap->iv_rtsthreshold) {
#ifdef ATH_SUPERG_FF
		/* we could refine to only check that the frame of interest
		 * is a FF, but this seems inconsistent.
		 */
		if (!(vap->iv_ath_cap & ni->ni_ath_flags & IEEE80211_ATHC_FF)) {
#endif
			flags |= HAL_TXDESC_RTSENA;	/* RTS based on frame length */
			cix = rt->info[rix].controlRate;
			sc->sc_stats.ast_tx_rts++;
#ifdef ATH_SUPERG_FF
		}
#endif
	}

	/*
	 * If 802.11g protection is enabled, determine whether
	 * to use RTS/CTS or just CTS.  Note that this is only
	 * done for OFDM unicast frames.
	 */
	if ((ic->ic_flags & IEEE80211_F_USEPROT) &&
	    rt->info[rix].phy == IEEE80211_T_OFDM &&
	    (flags & HAL_TXDESC_NOACK) == 0) {
		/* XXX fragments must use CCK rates w/ protection */
		if (ic->ic_protmode == IEEE80211_PROT_RTSCTS)
			flags |= HAL_TXDESC_RTSENA;
		else if (ic->ic_protmode == IEEE80211_PROT_CTSONLY)
			flags |= HAL_TXDESC_CTSENA;

		if (istxfrag)
			/*
			 *  if Tx fragment, it would be desirable to
			 *  use highest CCK rate for RTS/CTS.
			 *  However, stations farther away may detect it
			 *  at a lower CCK rate. Therefore, use the
			 *  configured protect rate, which is 2 Mbps
			 *  for 11G.
			 */
			cix = rt->info[sc->sc_protrix].controlRate;
		else
			cix = rt->info[sc->sc_protrix].controlRate;
		sc->sc_stats.ast_tx_protect++;
	}

	/*
	 * Calculate duration.  This logically belongs in the 802.11
	 * layer but it lacks sufficient information to calculate it.
	 */
	if ((flags & HAL_TXDESC_NOACK) == 0 &&
	    (wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) != IEEE80211_FC0_TYPE_CTL) {
		u_int16_t dur;

		/* XXX: not right with fragmentation. */
		if (shortPreamble)
			dur = rt->info[rix].spAckDuration;
		else
			dur = rt->info[rix].lpAckDuration;

		if (wh->i_fc[1] & IEEE80211_FC1_MORE_FRAG) {
			dur += dur;  /* Add additional 'SIFS + ACK' */

			/*
			** Compute size of next fragment in order to compute
			** durations needed to update NAV.
			** The last fragment uses the ACK duration only.
			** Add time for next fragment.
			*/
			dur += ath_hal_computetxtime(ah, rt, nextfraglen,
				rix, shortPreamble);
		}

		if (istxfrag) {
			/*
			**  Force hardware to use computed duration for next
			**  fragment by disabling multi-rate retry, which
			**  updates duration based on the multi-rate
			**  duration table.
			*/
			try0 = ATH_TXMAXTRY;
		}

		wh->i_dur = cpu_to_le16(dur);
	}

	/*
	 * Calculate RTS/CTS rate and duration if needed.
	 */
	ctsduration = 0;
	if (flags & (HAL_TXDESC_RTSENA|HAL_TXDESC_CTSENA)) {
		/*
		 * CTS transmit rate is derived from the transmit rate
		 * by looking in the h/w rate table.  We must also factor
		 * in whether or not a short preamble is to be used.
		 */
		/* NB: cix is set above where RTS/CTS is enabled */
		KASSERT(cix != 0xff, ("cix not setup"));
		ctsrate = rt->info[cix].rateCode;
		/*
		 * Compute the transmit duration based on the frame
		 * size and the size of an ACK frame.  We call into the
		 * HAL to do the computation since it depends on the
		 * characteristics of the actual PHY being used.
		 *
		 * NB: CTS is assumed the same size as an ACK so we can
		 *     use the precalculated ACK durations.
		 */
		if (shortPreamble) {
			ctsrate |= rt->info[cix].shortPreamble;
			if (flags & HAL_TXDESC_RTSENA)		/* SIFS + CTS */
				ctsduration += rt->info[cix].spAckDuration;
			ctsduration += ath_hal_computetxtime(ah,
				rt, pktlen, rix, AH_TRUE);
			if ((flags & HAL_TXDESC_NOACK) == 0)	/* SIFS + ACK */
				ctsduration += rt->info[rix].spAckDuration;
		} else {
			if (flags & HAL_TXDESC_RTSENA)		/* SIFS + CTS */
				ctsduration += rt->info[cix].lpAckDuration;
			ctsduration += ath_hal_computetxtime(ah,
				rt, pktlen, rix, AH_FALSE);
			if ((flags & HAL_TXDESC_NOACK) == 0)	/* SIFS + ACK */
				ctsduration += rt->info[rix].lpAckDuration;
		}
		/*
		 * Must disable multi-rate retry when using RTS/CTS.
		 */
		try0 = ATH_TXMAXTRY;
	} else
		ctsrate = 0;

	if (IFF_DUMPPKTS(sc, ATH_DEBUG_XMIT))
		/* FFXXX: need multi-skb version to dump entire FF */
		ieee80211_dump_pkt(ic, skb->data, skb->len,
			sc->sc_hwmap[txrate].ieeerate, -1);

	/*
	 * Determine if a tx interrupt should be generated for
	 * this descriptor.  We take a tx interrupt to reap
	 * descriptors when the h/w hits an EOL condition or
	 * when the descriptor is specifically marked to generate
	 * an interrupt.  We periodically mark descriptors in this
	 * way to ensure timely replenishing of the supply needed
	 * for sending frames.  Deferring interrupts reduces system
	 * load and potentially allows more concurrent work to be
	 * done, but if done too aggressively, it can cause senders
	 * to backup.
	 *
	 * NB: use >= to deal with sc_txintrperiod changing
	 *     dynamically through sysctl.
	 */
	if (!M_FLAG_GET(skb, M_UAPSD) &&
		++txq->axq_intrcnt >= sc->sc_txintrperiod) {
		flags |= HAL_TXDESC_INTREQ;
		txq->axq_intrcnt = 0;
	}

#ifdef ATH_SUPERG_COMP
	if (ATH_NODE(ni)->an_decomp_index != INVALID_DECOMP_INDEX &&
	    !ismcast &&
	    ((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_DATA) &&
	    ((wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK) != IEEE80211_FC0_SUBTYPE_NODATA)) {
		if (pktlen > ATH_COMP_THRESHOLD)
			comp = ATH_COMP_PROC_COMP_OPTIMAL;
		else
			comp = ATH_COMP_PROC_NO_COMP_ADD_CCS;
	}
#endif

	/*
	 * sc_txantenna == 0 means transmit diversity mode.
	 * sc_txantenna == 1 or sc_txantenna == 2 means the user has selected
	 * the first or second antenna port.
	 * If the user has set the txantenna, use it for multicast frames too.
	 */
	if (ismcast && !sc->sc_txantenna) {
		antenna = sc->sc_mcastantenna + 1;
		sc->sc_mcastantenna = (sc->sc_mcastantenna + 1) & 0x1;
	} else
		antenna = sc->sc_txantenna;

	if (txrate == 0) {
		/* Drop frame, if the rate is 0.
		 * Otherwise this may lead to the continuous transmission of
		 * noise. */
		printk("%s: invalid TX rate %u (%s: %u)\n", DEV_NAME(dev),
			txrate, __func__, __LINE__);
		return -EIO;
	}
	
	DPRINTF(sc, ATH_DEBUG_XMIT, "%s: set up txdesc: pktlen %d hdrlen %d "
		"atype %d txpower %d txrate %d try0 %d keyix %d ant %d flags %x "
		"ctsrate %d ctsdur %d icvlen %d ivlen %d comp %d\n",
		__func__, pktlen, hdrlen, atype, MIN(ni->ni_txpower, 60), txrate,
		try0, keyix, antenna, flags, ctsrate, ctsduration, icvlen, ivlen,
		comp);

	/*
	 * Formulate first tx descriptor with tx controls.
	 */
	/* XXX check return value? */
	ath_hal_setuptxdesc(ah, ds,
			    pktlen,			/* packet length */
			    hdrlen,			/* header length */
			    atype,			/* Atheros packet type */
			    MIN(ni->ni_txpower, 60),	/* txpower */
			    txrate, try0,		/* series 0 rate/tries */
			    keyix,			/* key cache index */
			    antenna,			/* antenna mode */
			    flags,			/* flags */
			    ctsrate,			/* rts/cts rate */
			    ctsduration,		/* rts/cts duration */
			    icvlen,			/* comp icv len */
			    ivlen,			/* comp iv len */
			    comp			/* comp scheme */
		);
	bf->bf_flags = flags;	/* record for post-processing */

	/*
	 * Setup the multi-rate retry state only when we're
	 * going to use it.  This assumes ath_hal_setuptxdesc
	 * initializes the descriptors (so we don't have to)
	 * when the hardware supports multi-rate retry and
	 * we don't use it.
	 */
	if (try0 != ATH_TXMAXTRY) {
		sc->sc_rc->ops->get_mrr(sc, an, shortPreamble, skb->len, rix,
					&mrr);
		/* Explicitly disable retries, if the retry rate is 0.
		 * Otherwise this may lead to the continuous transmission of
		 * noise. */
		if (!mrr.rate1) mrr.retries1 = 0;
		if (!mrr.rate2) mrr.retries2 = 0;
		if (!mrr.rate3) mrr.retries3 = 0;

		DPRINTF(sc, ATH_DEBUG_XMIT, "%s: set up multi rate/retry "
			"1:%d/%d 2:%d/%d 3:%d/%d\n", __func__,
			mrr.rate1, mrr.retries1, mrr.rate2, mrr.retries2,
			mrr.rate3, mrr.retries3);

		ath_hal_setupxtxdesc(sc->sc_ah, ds, mrr.rate1, mrr.retries1,
				     mrr.rate2, mrr.retries2,
				     mrr.rate3, mrr.retries3);
	}

#ifndef ATH_SUPERG_FF
	ds->ds_link = 0;
	ds->ds_data = bf->bf_skbaddr;

	ath_hal_filltxdesc(ah, ds,
			   skb->len,	/* segment length */
			   AH_TRUE,	/* first segment */
			   AH_TRUE,	/* last segment */
			   ds		/* first descriptor */
		);

	/* NB: The desc swap function becomes void,
	 * if descriptor swapping is not enabled
	 */
	ath_desc_swap(ds);

	DPRINTF(sc, ATH_DEBUG_XMIT, "%s: Q%d: %08x %08x %08x %08x %08x %08x\n",
	    __func__, M_FLAG_GET(skb, M_UAPSD) ? 0 : txq->axq_qnum, ds->ds_link, ds->ds_data,
	ds->ds_ctl0, ds->ds_ctl1, ds->ds_hw[0], ds->ds_hw[1]);
#else /* ATH_SUPERG_FF */
	{
		struct sk_buff *skbtmp = skb;
		struct ath_desc *ds0 = ds;
		unsigned int i;

		ds->ds_data = bf->bf_skbaddr;
		ds->ds_link = (skb->next == NULL) ? 0 : bf->bf_daddr + sizeof(*ds);

		ath_hal_filltxdesc(ah, ds,
			skbtmp->len,		/* segment length */
			AH_TRUE,		/* first segment */
			(skbtmp->next == NULL), /* last segment */
			ds			/* first descriptor */
		);

		/* NB: The desc swap function becomes void,
		 * if descriptor swapping is not enabled
		 */
		ath_desc_swap(ds);

		DPRINTF(sc, ATH_DEBUG_XMIT, "%s: Q%d: (ds)%p (lk)%08x (d)%08x "
			"(c0)%08x (c1)%08x %08x %08x\n", __func__, 
			M_FLAG_GET(skb, M_UAPSD) ? 0 : txq->axq_qnum,
			ds, ds->ds_link, ds->ds_data, ds->ds_ctl0, ds->ds_ctl1,
			ds->ds_hw[0], ds->ds_hw[1]);
		for (i = 0, skbtmp = skbtmp->next; 
				i < bf->bf_numdescff; i++, 
				skbtmp = skbtmp->next) {
			ds++;
			ds->ds_link = (skbtmp->next == NULL) ? 
				0 : bf->bf_daddr + (sizeof(*ds) * (i + 2));
			ds->ds_data = bf->bf_skbaddrff[i];
			ath_hal_filltxdesc(ah, ds,
				skbtmp->len,		/* segment length */
				AH_FALSE,		/* first segment */
				(skbtmp->next == NULL), /* last segment */
				ds0			/* first descriptor */
			);

			/* NB: The desc swap function becomes void,
			 * if descriptor swapping is not enabled
			 */
			ath_desc_swap(ds);

			DPRINTF(sc, ATH_DEBUG_XMIT, 
				"%s: Q%d: %08x %08x %08x %08x %08x %08x\n",
				__func__, M_FLAG_GET(skb, M_UAPSD) ? 
					0 : txq->axq_qnum,
				ds->ds_link, ds->ds_data, ds->ds_ctl0,
				ds->ds_ctl1, ds->ds_hw[0], ds->ds_hw[1]);
		}
	}
#endif

	if (M_FLAG_GET(skb, M_UAPSD)) {
		/* must lock against interrupt-time processing (i.e., not just tasklet) */
		ATH_NODE_UAPSD_LOCK_IRQ(an);
		DPRINTF(sc, ATH_DEBUG_UAPSD, 
			"%s: Qing U-APSD data frame for node %s \n",
			__func__, ether_sprintf(an->an_node.ni_macaddr));
		ath_tx_uapsdqueue(sc, an, bf);
		if (IEEE80211_NODE_UAPSD_USETIM(ni) && (an->an_uapsd_qdepth == 1))
			vap->iv_set_tim(ni, 1);
		ATH_NODE_UAPSD_UNLOCK_IRQ(an);

		return 0;
	}

	ath_tx_txqaddbuf(sc, PASS_NODE(ni), txq, bf, ds, pktlen);
	return 0;
#undef MIN
}

/*
 * Process completed xmit descriptors from the specified queue.
 * Should only be called from tasklet context
 */
static void
ath_tx_processq(struct ath_softc *sc, struct ath_txq *txq)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ath_buf *bf = NULL;
	struct ath_desc *ds = NULL;
	struct ath_tx_status *ts = NULL;
	struct ieee80211_node *ni = NULL;
	struct ath_node *an = NULL;
	unsigned int sr, lr;
	HAL_STATUS status;
	int uapsdq = 0;
	u_int64_t tsf = 0; /* Only needed for monitor mode */

	DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: tx queue %d (0x%x), link %p\n", 
			__func__,
		txq->axq_qnum, ath_hal_gettxbuf(sc->sc_ah, txq->axq_qnum),
		txq->axq_link);

	if (sc->sc_nmonvaps > 0)
		tsf = ath_hal_gettsf64(ah);

	if (txq == sc->sc_uapsdq) {
		DPRINTF(sc, ATH_DEBUG_UAPSD, "%s: reaping U-APSD txq\n", 
				__func__);
		uapsdq = 1;
	}

	for (;;) {
		ATH_TXQ_LOCK_IRQ(txq);

		txq->axq_intrcnt = 0; /* reset periodic desc intr count */
		bf = STAILQ_FIRST(&txq->axq_q);
		if (bf == NULL) {
			txq->axq_link = NULL;
			ATH_TXQ_UNLOCK_IRQ_EARLY(txq);
			goto bf_fail;
		}

#ifdef ATH_SUPERG_FF
		ds = &bf->bf_desc[bf->bf_numdescff];
		DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: frame's last desc: %p\n",
			__func__, ds);
#else
		ds = bf->bf_desc;		/* NB: last descriptor */
#endif
		ts = &bf->bf_dsstatus.ds_txstat;
		status = ath_hal_txprocdesc(ah, ds, ts);
#ifdef AR_DEBUG
		if (sc->sc_debug & ATH_DEBUG_XMIT_DESC)
			ath_printtxbuf(bf, status == HAL_OK);
#endif
		if (status == HAL_EINPROGRESS) {
			ATH_TXQ_UNLOCK_IRQ_EARLY(txq);
			goto bf_fail;
		}

		ATH_TXQ_REMOVE_HEAD(txq, bf_list);
		ATH_TXQ_UNLOCK_IRQ(txq);

		ni = bf->bf_node;
		if (ni != NULL) {
			an = ATH_NODE(ni);
			if (ts->ts_status == 0) {
				u_int8_t txant = ts->ts_antenna;
				sc->sc_stats.ast_ant_tx[txant]++;
				sc->sc_ant_tx[txant]++;
#ifdef ATH_SUPERG_FF
				if (bf->bf_numdescff > 0)
					ni->ni_vap->iv_stats.is_tx_ffokcnt++;
#endif
				if (ts->ts_rate & HAL_TXSTAT_ALTRATE)
					sc->sc_stats.ast_tx_altrate++;
				sc->sc_stats.ast_tx_rssi = ts->ts_rssi;
				ATH_RSSI_LPF(an->an_halstats.ns_avgtxrssi,
					ts->ts_rssi);
				if (bf->bf_skb->priority == WME_AC_VO ||
				    bf->bf_skb->priority == WME_AC_VI)
					ni->ni_ic->ic_wme.wme_hipri_traffic++;
				ni->ni_inact = ni->ni_inact_reload;
			} else {
#ifdef ATH_SUPERG_FF
				if (bf->bf_numdescff > 0)
					ni->ni_vap->iv_stats.is_tx_fferrcnt++;
#endif
				if (ts->ts_status & HAL_TXERR_XRETRY) {
					sc->sc_stats.ast_tx_xretries++;
					if (ni->ni_flags & IEEE80211_NODE_UAPSD_TRIG) {
						ni->ni_stats.ns_tx_eosplost++;
						DPRINTF(sc, ATH_DEBUG_UAPSD,
							"%s: frame in SP "
							"retried out, "
							"possible EOSP "
							"stranded!!!\n",
							__func__);
					}
				}
				if (ts->ts_status & HAL_TXERR_FIFO)
					sc->sc_stats.ast_tx_fifoerr++;
				if (ts->ts_status & HAL_TXERR_FILT)
					sc->sc_stats.ast_tx_filtered++;
			}
			sr = ts->ts_shortretry;
			lr = ts->ts_longretry;
			sc->sc_stats.ast_tx_shortretry += sr;
			sc->sc_stats.ast_tx_longretry += lr;
			/*
			 * Hand the descriptor to the rate control algorithm
			 * if the frame wasn't dropped for filtering or sent
			 * w/o waiting for an ack.  In those cases the rssi
			 * and retry counts will be meaningless.
			 */
			if ((ts->ts_status & HAL_TXERR_FILT) == 0 &&
			    (bf->bf_flags & HAL_TXDESC_NOACK) == 0)
				sc->sc_rc->ops->tx_complete(sc, an, bf);
		}

		bus_unmap_single(sc->sc_bdev, bf->bf_skbaddr,
				 bf->bf_skb->len, BUS_DMA_TODEVICE);
		bf->bf_skbaddr = 0;

		if (ni && uapsdq) {
			/* detect EOSP for this node */
			struct ieee80211_qosframe *qwh = 
				(struct ieee80211_qosframe *)bf->bf_skb->data;
			an = ATH_NODE(ni);
			KASSERT(ni != NULL, ("Processing U-APSD txq for "
						"ath_buf with no node!\n"));
			if (qwh->i_qos[0] & IEEE80211_QOS_EOSP) {
				DPRINTF(sc, ATH_DEBUG_UAPSD, 
					"%s: EOSP detected for node (%s) on desc %p\n",
					__func__, ether_sprintf(ni->ni_macaddr), ds);
				ATH_NODE_UAPSD_LOCK_IRQ(an);
				ni->ni_flags &= ~IEEE80211_NODE_UAPSD_SP;
				if ((an->an_uapsd_qdepth == 0) && 
						(an->an_uapsd_overflowqdepth != 0)) {
					STAILQ_CONCAT(&an->an_uapsd_q, 
							&an->an_uapsd_overflowq);
					an->an_uapsd_qdepth = 
						an->an_uapsd_overflowqdepth;
					an->an_uapsd_overflowqdepth = 0;
				}
				ATH_NODE_UAPSD_UNLOCK_IRQ(an);
			}
		}

		{
			struct ieee80211_frame *wh = 
				(struct ieee80211_frame *)bf->bf_skb->data;
			if ((ts->ts_seqnum << IEEE80211_SEQ_SEQ_SHIFT) & 
					~IEEE80211_SEQ_SEQ_MASK) {
				DPRINTF(sc, ATH_DEBUG_TX_PROC, 
					"%s: h/w assigned sequence number is "
					"not sane (%d), ignoring it\n", 
					__func__, ts->ts_seqnum);
			} else {
				DPRINTF(sc, ATH_DEBUG_TX_PROC, 
					"%s: updating frame's sequence number "
					"from %d to %d\n", __func__,
				        ((le16toh(*(__le16 *)&wh->i_seq[0]) & 
					 	IEEE80211_SEQ_SEQ_MASK)) >> 
					IEEE80211_SEQ_SEQ_SHIFT,
				        ts->ts_seqnum);

				*(__le16 *)&wh->i_seq[0] = htole16(
					ts->ts_seqnum << IEEE80211_SEQ_SEQ_SHIFT |
					(le16toh(*(__le16 *)&wh->i_seq[0]) & 
					 ~IEEE80211_SEQ_SEQ_MASK));
			}
		}

		{
			u_int32_t tstamp;
			/* Extend tstamp to a full TSF.
			 * TX descriptor contains the transmit time in TUs,
			 * (bits 25-10 of the TSF). */
#define TSTAMP_TX_MASK	((2 ^ (27 - 1)) - 1)	/* First 27 bits. */

			tstamp = ts->ts_tstamp << 10;
			bf->bf_tsf = ((bf->bf_tsf & ~TSTAMP_TX_MASK) | tstamp);
			if ((bf->bf_tsf & TSTAMP_TX_MASK) < tstamp)
				bf->bf_tsf -= TSTAMP_TX_MASK + 1;
		}

		{
			struct sk_buff *tskb = NULL, *skb = bf->bf_skb;
#ifdef ATH_SUPERG_FF
			unsigned int i;
#endif

			/* ath_capture modifies skb data; must be last process
			 * in TX path. */
			tskb = skb->next;
			DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: free skb %p\n", 
					__func__, bf->bf_skb);
			ath_capture(sc->sc_dev, bf, skb, bf->bf_tsf, 1 /* TX */);
			skb = tskb;

#ifdef ATH_SUPERG_FF
			/* Handle every skb after the first one - these are FF 
			 * extra buffers */
			for (i = 0; i < bf->bf_numdescff; i++) {
				tskb = skb->next;
				DPRINTF(sc, ATH_DEBUG_TX_PROC, "%s: capture/free skb %p\n",
					__func__, skb);
				ath_capture(sc->sc_dev, bf, skb, bf->bf_tsf, 1 /* TX */);
				skb = tskb;
			}
			bf->bf_numdescff = 0;
#endif
		}

		bf->bf_skb = NULL;
		ni = NULL;
		ath_return_txbuf(sc, &bf);
	}
bf_fail:
#ifdef ATH_SUPERG_FF
	/* flush ff staging queue if buffer low */
	if (txq->axq_depth <= sc->sc_fftxqmin - 1) {
		/* NB: consider only flushing a preset number based on age. */
		ath_ffstageq_flush(sc, txq, ath_ff_neverflushtestdone);
	}
#else
	;
#endif /* ATH_SUPERG_FF */
}

static __inline int
txqactive(struct ath_hal *ah, int qnum)
{
	u_int32_t txqs = 1 << qnum;
	ath_hal_gettxintrtxqs(ah, &txqs);
	return (txqs & (1 << qnum));
}

/*
 * Deferred processing of transmit interrupt; special-cased
 * for a single hardware transmit queue (e.g. 5210 and 5211).
 */
static void
ath_tx_tasklet_q0(TQUEUE_ARG data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ath_softc *sc = dev->priv;

	if (txqactive(sc->sc_ah, 0))
		ath_tx_processq(sc, &sc->sc_txq[0]);
	if (txqactive(sc->sc_ah, sc->sc_cabq->axq_qnum))
		ath_tx_processq(sc, sc->sc_cabq);

	netif_wake_queue(dev);

	if (sc->sc_softled)
		ath_led_event(sc, ATH_LED_TX);
}

/*
 * Deferred processing of transmit interrupt; special-cased
 * for four hardware queues, 0-3 (e.g. 5212 w/ WME support).
 */
static void
ath_tx_tasklet_q0123(TQUEUE_ARG data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ath_softc *sc = dev->priv;

	/*
	 * Process each active queue.
	 */
	if (txqactive(sc->sc_ah, 0))
		ath_tx_processq(sc, &sc->sc_txq[0]);
	if (txqactive(sc->sc_ah, 1))
		ath_tx_processq(sc, &sc->sc_txq[1]);
	if (txqactive(sc->sc_ah, 2))
		ath_tx_processq(sc, &sc->sc_txq[2]);
	if (txqactive(sc->sc_ah, 3))
		ath_tx_processq(sc, &sc->sc_txq[3]);
	if (txqactive(sc->sc_ah, sc->sc_cabq->axq_qnum))
		ath_tx_processq(sc, sc->sc_cabq);
#ifdef ATH_SUPERG_XR
	if (sc->sc_xrtxq && txqactive(sc->sc_ah, sc->sc_xrtxq->axq_qnum))
		ath_tx_processq(sc, sc->sc_xrtxq);
#endif
	if (sc->sc_uapsdq && txqactive(sc->sc_ah, sc->sc_uapsdq->axq_qnum))
		ath_tx_processq(sc, sc->sc_uapsdq);

	netif_wake_queue(dev);

	if (sc->sc_softled)
		ath_led_event(sc, ATH_LED_TX);
}

/*
 * Deferred processing of transmit interrupt.
 */
static void
ath_tx_tasklet(TQUEUE_ARG data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ath_softc *sc = dev->priv;
	unsigned int i;

	/* Process each active queue. This includes sc_cabq, sc_xrtq and
	 * sc_uapsdq */
	for (i = 0; i < HAL_NUM_TX_QUEUES; i++)
		if (ATH_TXQ_SETUP(sc, i) && txqactive(sc->sc_ah, i))
			ath_tx_processq(sc, &sc->sc_txq[i]);

	netif_wake_queue(dev);

	if (sc->sc_softled)
		ath_led_event(sc, ATH_LED_TX);
}

static void
ath_tx_timeout(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;


	DPRINTF(sc, ATH_DEBUG_WATCHDOG, "%s: %sRUNNING %svalid\n",
		__func__, (dev->flags & IFF_RUNNING) ? "" : "!",
		sc->sc_invalid ? "in" : "");

	if ((dev->flags & IFF_RUNNING) && !sc->sc_invalid) {
		sc->sc_stats.ast_watchdog++;
		ath_reset(dev);	/* Avoid taking a semaphore in ath_init */
	}
}

/*
 * Context: softIRQ and hwIRQ
 */
static void
ath_tx_draintxq(struct ath_softc *sc, struct ath_txq *txq)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ath_buf *bf;
	/*
	 * NB: this assumes output has been stopped and
	 *     we do not need to block ath_tx_tasklet
	 */
	for (;;) {
		ATH_TXQ_LOCK_IRQ(txq);
		bf = STAILQ_FIRST(&txq->axq_q);
		if (bf == NULL) {
			txq->axq_link = NULL;
			ATH_TXQ_UNLOCK_IRQ_EARLY(txq);
			return;
		}
		ATH_TXQ_REMOVE_HEAD(txq, bf_list);
		ATH_TXQ_UNLOCK_IRQ(txq);
#ifdef AR_DEBUG
		if (sc->sc_debug & ATH_DEBUG_RESET)
			ath_printtxbuf(bf, ath_hal_txprocdesc(ah, bf->bf_desc, 
						&bf->bf_dsstatus.ds_txstat) == HAL_OK);
#endif /* AR_DEBUG */

		ath_return_txbuf(sc, &bf);
	}
}

static void
ath_tx_stopdma(struct ath_softc *sc, struct ath_txq *txq)
{
	struct ath_hal *ah = sc->sc_ah;

	(void) ath_hal_stoptxdma(ah, txq->axq_qnum);
	DPRINTF(sc, ATH_DEBUG_RESET, "%s: tx queue [%u] 0x%x, link %p\n",
		__func__, txq->axq_qnum,
		ath_hal_gettxbuf(ah, txq->axq_qnum), txq->axq_link);
}

/*
 * Drain the transmit queues and reclaim resources.
 */
static void
ath_draintxq(struct ath_softc *sc)
{
	struct ath_hal *ah = sc->sc_ah;
	unsigned int i;

	/* XXX return value */
	if (!sc->sc_invalid) {
		DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"%s: ath_hal_stoptxdma sc_bhalq:%d\n",
			__func__, sc->sc_bhalq);
		(void) ath_hal_stoptxdma(ah, sc->sc_bhalq);
		DPRINTF(sc, ATH_DEBUG_RESET, "%s: beacon queue 0x%x\n",
			__func__, ath_hal_gettxbuf(ah, sc->sc_bhalq));
		for (i = 0; i < HAL_NUM_TX_QUEUES; i++)
			if (ATH_TXQ_SETUP(sc, i))
				ath_tx_stopdma(sc, &sc->sc_txq[i]);
	}
	sc->sc_dev->trans_start = jiffies;
	netif_start_queue(sc->sc_dev);		/* XXX move to callers */
	for (i = 0; i < HAL_NUM_TX_QUEUES; i++)
		if (ATH_TXQ_SETUP(sc, i))
			ath_tx_draintxq(sc, &sc->sc_txq[i]);
}

/*
 * Disable the receive h/w in preparation for a reset.
 */
static void
ath_stoprecv(struct ath_softc *sc)
{
#define	PA2DESC(_sc, _pa) \
	((struct ath_desc *)((caddr_t)(_sc)->sc_rxdma.dd_desc + \
		((_pa) - (_sc)->sc_rxdma.dd_desc_paddr)))
	struct ath_hal *ah = sc->sc_ah;
	u_int64_t tsf;

	ath_hal_stoppcurecv(ah);	/* disable PCU */
	ath_hal_setrxfilter(ah, 0);	/* clear recv filter */
	ath_hal_stopdmarecv(ah);	/* disable DMA engine */
	mdelay(3);			/* 3 ms is long enough for 1 frame */
	tsf = ath_hal_gettsf64(ah);
#ifdef AR_DEBUG
	if (sc->sc_debug & (ATH_DEBUG_RESET | ATH_DEBUG_FATAL)) {
		struct ath_buf *bf;

		printk("ath_stoprecv: rx queue 0x%x, link %p\n",
			ath_hal_getrxbuf(ah), sc->sc_rxlink);
		STAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list) {
			struct ath_desc *ds = bf->bf_desc;
			struct ath_rx_status *rs = &bf->bf_dsstatus.ds_rxstat;
			HAL_STATUS status = ath_hal_rxprocdesc(ah, ds,
				bf->bf_daddr, PA2DESC(sc, ds->ds_link), tsf, rs);
			if (status == HAL_OK || (sc->sc_debug & ATH_DEBUG_FATAL))
				ath_printrxbuf(bf, status == HAL_OK);
		}
	}
#endif
	sc->sc_rxlink = NULL;		/* just in case */
#undef PA2DESC
}

/*
 * Enable the receive h/w following a reset.
 */
static int
ath_startrecv(struct ath_softc *sc)
{
	struct ath_hal *ah = sc->sc_ah;
	struct net_device *dev = sc->sc_dev;
	struct ath_buf *bf;

	/*
	 * Cisco's VPN software requires that drivers be able to
	 * receive encapsulated frames that are larger than the MTU.
	 * Since we can't be sure how large a frame we'll get, setup
	 * to handle the larges on possible.
	 */
#ifdef ATH_SUPERG_FF
	sc->sc_rxbufsize = roundup(ATH_FF_MAX_LEN, sc->sc_cachelsz);
#else
	sc->sc_rxbufsize = roundup(IEEE80211_MAX_LEN, sc->sc_cachelsz);
#endif
	DPRINTF(sc, ATH_DEBUG_RESET, "%s: mtu %u cachelsz %u rxbufsize %u\n",
		__func__, dev->mtu, sc->sc_cachelsz, sc->sc_rxbufsize);

	sc->sc_rxlink = NULL;
	STAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list) {
		int error = ath_rxbuf_init(sc, bf);
		ATH_RXBUF_RESET(bf);
		if (error < 0)
			return error;
	}

	sc->sc_rxbufcur = NULL;

	bf = STAILQ_FIRST(&sc->sc_rxbuf);
	ath_hal_putrxbuf(ah, bf->bf_daddr);
	ath_hal_rxena(ah);		/* enable recv descriptors */
	ath_mode_init(dev);		/* set filters, etc. */
	ath_hal_startpcurecv(ah);	/* re-enable PCU/DMA engine */
	return 0;
}

/*
 * Flush skbs allocated for receiving.
 */
static void
ath_flushrecv(struct ath_softc *sc)
{
	struct ath_buf *bf;
	STAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list)
		cleanup_ath_buf(sc, bf, BUS_DMA_FROMDEVICE);
}

/*
 * Update internal state after a channel change.
 */
static void
ath_chan_change(struct ath_softc *sc, struct ieee80211_channel *chan)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct net_device *dev = sc->sc_dev;
	enum ieee80211_phymode mode;

	mode = ieee80211_chan2mode(chan);

	ath_rate_setup(dev, mode);
	ath_setcurmode(sc, mode);
	/* Reset noise floor on channelc hange and let ieee layer know */
	ic->ic_channoise = ath_hal_get_channel_noise(sc->sc_ah, 
			&(sc->sc_curchan));

#ifdef notyet
	/*
	 * Update BPF state.
	 */
	sc->sc_tx_th.wt_chan_freq = sc->sc_rx_th.wr_chan_freq =
		htole16(chan->ic_freq);
	sc->sc_tx_th.wt_chan_flags = sc->sc_rx_th.wr_chan_flags =
		htole16(chan->ic_flags);
#endif
	if (ic->ic_curchanmaxpwr == 0)
		ic->ic_curchanmaxpwr = chan->ic_maxregpower;
}

/*
 * Set/change channels.  If the channel is really being changed,
 * it's done by resetting the chip.  To accomplish this we must
 * first cleanup any pending DMA, then restart stuff after a la
 * ath_init.
 */
static int
ath_chan_set(struct ath_softc *sc, struct ieee80211_channel *chan)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	struct net_device *dev = sc->sc_dev;
	HAL_CHANNEL hchan;
	u_int8_t tswitch = 0;
	u_int8_t channel_change_required = 0;
	struct timeval tv;

	/*
	 * Convert to a HAL channel description with
	 * the flags constrained to reflect the current
	 * operating mode.
	 */
	memset(&hchan, 0, sizeof(HAL_CHANNEL));
	hchan.channel = chan->ic_freq;
	hchan.channelFlags = ath_chan2flags(chan);
	KASSERT(hchan.channel != 0,
		("bogus channel %u/0x%x", hchan.channel, hchan.channelFlags));
	do_gettimeofday(&tv);
	DPRINTF(sc, ATH_DEBUG_RESET | ATH_DEBUG_DOTH, 
		"%s: %3d (%4d MHz) -> %3d (%4d MHz) -- Time: %ld.%06ld\n",
		__func__, ath_hal_mhz2ieee(ah, sc->sc_curchan.channel,
		sc->sc_curchan.channelFlags), sc->sc_curchan.channel,
		ath_hal_mhz2ieee(ah, hchan.channel, hchan.channelFlags),
		hchan.channel,
		tv.tv_sec,
		tv.tv_usec
		);

	/* check if it is turbo mode switch */
	if (hchan.channel == sc->sc_curchan.channel &&
	   (hchan.channelFlags & IEEE80211_CHAN_TURBO) != 
	   (sc->sc_curchan.channelFlags & IEEE80211_CHAN_TURBO))
		tswitch = 1;


	channel_change_required = hchan.channel != sc->sc_curchan.channel ||
		hchan.channelFlags != sc->sc_curchan.channelFlags ||
		tswitch;

	if (channel_change_required) {
		HAL_STATUS status;

		/* To switch channels clear any pending DMA operations;
		 * wait long enough for the RX fifo to drain, reset the
		 * hardware at the new frequency, and then re-enable
		 * the relevant bits of the h/w. */
		ath_hal_intrset(ah, 0);	/* disable interrupts */
		ath_draintxq(sc);	/* clear pending tx frames */
		ath_stoprecv(sc);	/* turn off frame recv */

		/* Set coverage class */
		if (sc->sc_scanning || !IEEE80211_IS_CHAN_A(chan))
			ath_hal_setcoverageclass(sc->sc_ah, 0, 0);
		else
			ath_hal_setcoverageclass(sc->sc_ah, ic->ic_coverageclass, 0);

		do_gettimeofday(&tv);

		DPRINTF(sc, ATH_DEBUG_DOTH, 
			"RADAR CHANNEL channel:%u jiffies:%lu\n",
			hchan.channel,
			jiffies);

		/* ath_hal_reset with chanchange = AH_TRUE doesn't seem to
		 * completely reset the state of the card.  According to
		 * reports from ticket #1106, kismet and aircrack people they
		 * needed to do the reset with chanchange = AH_FALSE in order
		 * to receive traffic when peforming high velocity channel
		 * changes. */
		if (!ath_hal_reset(ah, sc->sc_opmode, &hchan, AH_TRUE, &status)   ||
		    !ath_hal_reset(ah, sc->sc_opmode, &hchan, AH_FALSE, &status)) {
			printk("%s: %s: unable to reset channel %u (%u MHz) "
				"flags 0x%x '%s' (HAL status %u)\n",
				DEV_NAME(dev), __func__,
				ieee80211_chan2ieee(ic, chan), chan->ic_freq,
				hchan.channelFlags,
				ath_get_hal_status_desc(status), status);
			return -EIO;
		}

		if (sc->sc_softled)
			ath_hal_gpioCfgOutput(ah, sc->sc_ledpin);

		if ((sc->sc_opmode != HAL_M_STA) && sc->sc_hasintmit)
			ath_hal_setintmit(ah, 0);

		sc->sc_curchan = hchan;
		ath_update_txpow(sc);		/* update tx power state */

		/* Change channels and update the h/w rate map
		 * if we're switching; e.g. 11a to 11b/g. */
		ath_chan_change(sc, chan);
		 /* Re-enable rx framework. */
		if (ath_startrecv(sc) != 0) {
			printk("%s: %s: unable to restart recv logic\n",
				DEV_NAME(dev), __func__);
			return -EIO;
		}

		do_gettimeofday(&tv);
		/*
		 * Change channels and update the h/w rate map
		 * if we're switching; e.g. 11a to 11b/g.
		 */
		ath_chan_change(sc, chan);
		if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
			if (sc->sc_curchan.privFlags & CHANNEL_DFS) {
				if (!(sc->sc_curchan.privFlags & 
							CHANNEL_DFS_CLEAR)) {
					/* set the timeout to normal */
					dev->watchdog_timeo = 120 * HZ; 
					if (sc->sc_dfswait)
						del_timer_sync(&sc->sc_dfswaittimer);
					DPRINTF(sc, ATH_DEBUG_STATE, "%s: %s: "
							"start dfs wait period\n",
							__func__, DEV_NAME(dev));
					sc->sc_dfswait = 1;
					sc->sc_dfswaittimer.function = 
						ath_check_dfs_clear;
					sc->sc_dfswaittimer.expires =
						jiffies + 
						(ATH_DFS_WAIT_SHORT_POLL_PERIOD * HZ);
					sc->sc_dfswaittimer.data = 
						(unsigned long)sc;
					add_timer(&sc->sc_dfswaittimer);
				}
			} else if (sc->sc_dfswait == 1)
				mod_timer(&sc->sc_dfswaittimer, jiffies + 2);
		}
		/*
		 * re configure beacons when it is a turbo mode switch.
		 * HW seems to turn off beacons during turbo mode switch.
		 */
		if (sc->sc_beacons && tswitch)
			ath_beacon_config(sc, NULL);
		/*
		 * Re-enable interrupts.
		 */
		ath_hal_intrset(ah, sc->sc_imask);
	}
	return 0;
}

/*
 * Enable MIB interrupts again, after the ISR disabled them
 * to slow down the rate of PHY error reporting.
 */
static void
ath_mib_enable(unsigned long arg)
{
	struct ath_softc *sc = (struct ath_softc *)arg;

	sc->sc_imask |= HAL_INT_MIB;
	ath_hal_intrset(sc->sc_ah, sc->sc_imask);
}

/*
 * Periodically recalibrate the PHY to account
 * for temperature/environment changes.
 */
static void
ath_calibrate(unsigned long arg)
{
	struct net_device *dev = (struct net_device *)arg;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	/* u_int32_t nchans; */
	HAL_BOOL isIQdone = AH_FALSE;

	sc->sc_stats.ast_per_cal++;
	DPRINTF(sc, ATH_DEBUG_CALIBRATE, 
			"%s: channel %u/%x - periodic recalibration\n", 
			__func__, sc->sc_curchan.channel, 
			sc->sc_curchan.channelFlags);

	if (ath_hal_getrfgain(ah) == HAL_RFGAIN_NEED_CHANGE) {
		/*
		 * Rfgain is out of bounds, reset the chip
		 * to load new gain values.
		 */
		int txcont_was_active = sc->sc_txcont;
		DPRINTF(sc, ATH_DEBUG_RESET | ATH_DEBUG_CALIBRATE | ATH_DEBUG_DOTH,
			"%s: Forcing reset() for (ath_hal_getrfgain(ah) == "
			"HAL_RFGAIN_NEED_CHANGE)\n",  __func__);
		sc->sc_stats.ast_per_rfgain++;
		ath_reset(dev);
		/* Turn txcont back on as necessary */
		if (txcont_was_active)
			ath_set_txcont(ic, txcont_was_active);

	}
	if (!ath_hal_calibrate(ah, &sc->sc_curchan, &isIQdone)) {
		DPRINTF(sc, ATH_DEBUG_ANY,
			"%s: calibration of channel %u failed\n",
			__func__, sc->sc_curchan.channel);
		sc->sc_stats.ast_per_calfail++;
	}

	ath_hal_process_noisefloor(ah);
	if (isIQdone == AH_TRUE) {
		/* Unless user has overridden calibration interval,
		 * upgrade to less frequent calibration */
		if (ath_calinterval == ATH_SHORT_CALINTERVAL)
			ath_calinterval = ATH_LONG_CALINTERVAL;
	}
	else {
		/* Unless user has overridden calibration interval,
		 * downgrade to more frequent calibration */
		if (ath_calinterval == ATH_LONG_CALINTERVAL)
			ath_calinterval = ATH_SHORT_CALINTERVAL;
	}

	DPRINTF(sc, ATH_DEBUG_CALIBRATE, "%s: channel %u/%x -- IQ %s.\n",
		__func__, sc->sc_curchan.channel, sc->sc_curchan.channelFlags,
		isIQdone ? "done" : "not done");

	sc->sc_cal_ch.expires = jiffies + (ath_calinterval * HZ);
	add_timer(&sc->sc_cal_ch);
}

static void
ath_scan_start(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	u_int32_t rfilt;

	/* XXX calibration timer? */

	sc->sc_scanning = 1;
	sc->sc_syncbeacon = 0;
	rfilt = ath_calcrxfilter(sc);
	ath_hal_setrxfilter(ah, rfilt);
	ath_hal_setassocid(ah, dev->broadcast, 0);

	DPRINTF(sc, ATH_DEBUG_STATE, "%s: RX filter 0x%x bssid %s aid 0\n",
		 __func__, rfilt, ether_sprintf(dev->broadcast));
}

static void
ath_scan_end(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	u_int32_t rfilt;

	sc->sc_scanning = 0;
	rfilt = ath_calcrxfilter(sc);
	ath_hal_setrxfilter(ah, rfilt);
	ath_hal_setassocid(ah, sc->sc_curbssid, sc->sc_curaid);

	DPRINTF(sc, ATH_DEBUG_STATE, "%s: RX filter 0x%x bssid %s aid 0x%x\n",
		 __func__, rfilt, ether_sprintf(sc->sc_curbssid),
		 sc->sc_curaid);
}

static void
ath_set_channel(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;

	(void) ath_chan_set(sc, ic->ic_curchan);
	/*
	 * If we are returning to our bss channel then mark state
	 * so the next recv'd beacon's TSF will be used to sync the
	 * beacon timers.  Note that since we only hear beacons in
	 * sta/ibss mode this has no effect in other operating modes.
	 */
	if (!sc->sc_scanning && ic->ic_curchan == ic->ic_bsschan)
		sc->sc_syncbeacon = 1;
}

static void
ath_set_coverageclass(struct ieee80211com *ic)
{
	struct ath_softc *sc = ic->ic_dev->priv;

	ath_hal_setcoverageclass(sc->sc_ah, ic->ic_coverageclass, 0);

	return;
}

static u_int
ath_mhz2ieee(struct ieee80211com *ic, u_int freq, u_int flags)
{
	struct ath_softc *sc = ic->ic_dev->priv;

	return (ath_hal_mhz2ieee(sc->sc_ah, freq, flags));
}


/*
 * Context: softIRQ and process context
 */
static int
ath_newstate(struct ieee80211vap *vap, enum ieee80211_state nstate, int arg)
{
	struct ath_vap *avp = ATH_VAP(vap);
	struct ieee80211com *ic = vap->iv_ic;
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211_node *ni, *wds_ni;
	unsigned int i;
	int error, stamode;
	u_int32_t rfilt = 0;
	struct ieee80211vap *tmpvap;
	static const HAL_LED_STATE leds[] = {
		HAL_LED_INIT,	/* IEEE80211_S_INIT */
		HAL_LED_SCAN,	/* IEEE80211_S_SCAN */
		HAL_LED_AUTH,	/* IEEE80211_S_AUTH */
		HAL_LED_ASSOC,	/* IEEE80211_S_ASSOC */
		HAL_LED_RUN,	/* IEEE80211_S_RUN */
	};

	DPRINTF(sc, ATH_DEBUG_STATE, "%s: %p[%s] %s -> %s\n", __func__,
		vap, vap->iv_nickname,
		ieee80211_state_name[vap->iv_state],
		ieee80211_state_name[nstate]);

	del_timer(&sc->sc_cal_ch);		/* periodic calibration timer */

	ath_hal_setledstate(ah, leds[nstate]);	/* set LED */
	netif_stop_queue(dev);			/* before we do anything else */

	if (nstate == IEEE80211_S_INIT) {
		/*
		 * if there is no VAP left in RUN state
		 * disable beacon interrupts.
		 */
		TAILQ_FOREACH(tmpvap, &ic->ic_vaps, iv_next) {
			if ((tmpvap != vap) && 
					(tmpvap->iv_state == IEEE80211_S_RUN))
				break;
		}
		if (!tmpvap) {
			sc->sc_imask &= ~(HAL_INT_SWBA | HAL_INT_BMISS);
			/*
			 * Disable interrupts.
			 */
			ath_hal_intrset(ah, sc->sc_imask &~ HAL_INT_GLOBAL);
			sc->sc_beacons = 0;
		}
		/*
		 * Notify the rate control algorithm.
		 */
		sc->sc_rc->ops->newstate(vap, nstate);
		sc->sc_txcont = 0;
		sc->sc_txcont_rate = 0;
		sc->sc_txcont_power = -1;

		goto done;
	}
	ni = vap->iv_bss;

	rfilt = ath_calcrxfilter(sc);
	stamode = (vap->iv_opmode == IEEE80211_M_STA ||
		   vap->iv_opmode == IEEE80211_M_IBSS ||
		   vap->iv_opmode == IEEE80211_M_AHDEMO);
	if (stamode && nstate == IEEE80211_S_RUN) {
		sc->sc_curaid = ni->ni_associd;
		IEEE80211_ADDR_COPY(sc->sc_curbssid, ni->ni_bssid);
		DPRINTF(sc, ATH_DEBUG_BEACON,
			"%s: sc_curbssid " MAC_FMT " from " MAC_FMT "\n",
			__func__, MAC_ADDR(sc->sc_curbssid),
			MAC_ADDR(ni->ni_macaddr));
	} else
		sc->sc_curaid = 0;

	DPRINTF(sc, ATH_DEBUG_STATE, "%s: RX filter 0x%x bssid %s aid 0x%x\n",
		 __func__, rfilt, ether_sprintf(sc->sc_curbssid),
		 sc->sc_curaid);

	ath_hal_setrxfilter(ah, rfilt);
	if (stamode) {
		DPRINTF(sc, ATH_DEBUG_BEACON,
			"%s: setassocid " MAC_FMT "\n",
			__func__, MAC_ADDR(sc->sc_curbssid));
		ath_hal_setassocid(ah, sc->sc_curbssid, sc->sc_curaid);
	}

	if ((vap->iv_opmode != IEEE80211_M_STA) &&
			(vap->iv_flags & IEEE80211_F_PRIVACY)) {
		for (i = 0; i < IEEE80211_WEP_NKID; i++)
			if (ath_hal_keyisvalid(ah, i))
				ath_hal_keysetmac(ah, i, ni->ni_bssid);
	}

	/*
	 * Notify the rate control algorithm so rates
	 * are setup should ath_beacon_alloc be called.
	 */
	sc->sc_rc->ops->newstate(vap, nstate);

	if (vap->iv_opmode == IEEE80211_M_MONITOR) {
		/* nothing to do */;
	} else if (nstate == IEEE80211_S_RUN) {
		DPRINTF(sc, ATH_DEBUG_STATE,
			"%s(RUN): ic_flags=0x%08x iv=%d bssid=%s "
			"capinfo=0x%04x chan=%d\n",
			 __func__,
			 vap->iv_flags,
			 ni->ni_intval,
			 ether_sprintf(ni->ni_bssid),
			 ni->ni_capinfo,
			 ieee80211_chan2ieee(ic, ni->ni_chan));

		switch (vap->iv_opmode) {
		case IEEE80211_M_HOSTAP:
		case IEEE80211_M_IBSS:
			/*
			 * Allocate and setup the beacon frame.
			 *
			 * Stop any previous beacon DMA. This may be necessary,
			 * for example, when an ibss merge causes
			 * reconfiguration; there will be a state transition
			 * from RUN->RUN that means we may be called with
			 * beacon transmission active. In this case,
			 * ath_beacon_alloc() is simply skipped to avoid race
			 * condition with SWBA interrupts calling
			 * ath_beacon_generate()
			 */
			DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
				"%s: ath_hal_stoptxdma sc_bhalq:%d\n",
				__func__, sc->sc_bhalq);
			ath_hal_stoptxdma(ah, sc->sc_bhalq);

			/* Set default key index for static wep case */
			ni->ni_ath_defkeyindex = IEEE80211_INVAL_DEFKEY;
			if (((vap->iv_flags & IEEE80211_F_WPA) == 0) &&
			    (ni->ni_authmode != IEEE80211_AUTH_8021X) &&
			    (vap->iv_def_txkey != IEEE80211_KEYIX_NONE)) {
				ni->ni_ath_defkeyindex = vap->iv_def_txkey;
			}

			error = ath_beacon_alloc(sc, ni);
			if (error < 0)
				goto bad;

			/*
			 * if the turbo flags have changed, then beacon and turbo
			 * need to be reconfigured.
			 */
			if ((sc->sc_dturbo && !(vap->iv_ath_cap & IEEE80211_ATHC_TURBOP)) ||
				(!sc->sc_dturbo && (vap->iv_ath_cap & IEEE80211_ATHC_TURBOP)))
				sc->sc_beacons = 0;
			/*
			 * if it is the first AP VAP moving to RUN state then beacon
			 * needs to be reconfigured.
			 */
			TAILQ_FOREACH(tmpvap, &ic->ic_vaps, iv_next) {
				if ((tmpvap != vap) && 
						(tmpvap->iv_state == IEEE80211_S_RUN) &&
						(tmpvap->iv_opmode == IEEE80211_M_HOSTAP))
					break;
			}
			if (!tmpvap)
				sc->sc_beacons = 0;
			break;
		case IEEE80211_M_STA:
#ifdef ATH_SUPERG_COMP
			/* Have we negotiated compression? */
			if (!(vap->iv_ath_cap & ni->ni_ath_flags & IEEE80211_NODE_COMP))
				ni->ni_ath_flags &= ~IEEE80211_NODE_COMP;
#endif
			/* Allocate a key cache slot to the station. */
			ath_setup_keycacheslot(sc, ni);

			/*
			 * Record negotiated dynamic turbo state for
			 * use by rate control modules.
			 */
			sc->sc_dturbo =
				(ni->ni_ath_flags & IEEE80211_ATHC_TURBOP) != 0;
			break;
		case IEEE80211_M_WDS:
			wds_ni = ieee80211_find_txnode(vap, vap->wds_mac);
			if (wds_ni != NULL) {
				/* XXX no rate negotiation; just dup */
				wds_ni->ni_rates = vap->iv_bss->ni_rates;
				/* Depending on the sequence of bringing up devices
				 * it's possible the rates of the root BSS isn't
				 * filled yet. */
				if ((vap->iv_ic->ic_newassoc != NULL) &&
				    (wds_ni->ni_rates.rs_nrates != 0)) {
					/* Fill in the rates based on our own rates
					 * we rely on the rate selection mechanism
					 * to find out which rates actually work! */
					vap->iv_ic->ic_newassoc(wds_ni, 1);
				}
				ieee80211_unref_node(&wds_ni);
			}
			break;
		default:
			break;
		}

		ath_hal_process_noisefloor(ah);
		/*
		 * Configure the beacon and sleep timers.
		 */
		if (!sc->sc_beacons && vap->iv_opmode!=IEEE80211_M_WDS) {
			ath_beacon_config(sc, vap);
			DPRINTF(sc, ATH_DEBUG_STATE | ATH_DEBUG_BEACON | ATH_DEBUG_BEACON_PROC,
			"%s: Beacons reconfigured by %p[%s]!\n",
			__func__, vap, vap->iv_nickname);

			sc->sc_beacons = 1;
		}

		/*
		 * Reset rssi stats; maybe not the best place...
		 */
		sc->sc_halstats.ns_avgbrssi = ATH_RSSI_DUMMY_MARKER;
		sc->sc_halstats.ns_avgrssi = ATH_RSSI_DUMMY_MARKER;
		sc->sc_halstats.ns_avgtxrssi = ATH_RSSI_DUMMY_MARKER;
		/* if it is a DFS channel and has not been checked for radar
		 * do not let the 80211 state machine to go to RUN state. */
		if (sc->sc_dfswait && vap->iv_opmode == IEEE80211_M_HOSTAP) {
			/* push the VAP to RUN state once DFS is cleared */
			DPRINTF(sc, ATH_DEBUG_STATE, "%s: %s: VAP  -> DFS_WAIT\n",
				__func__, DEV_NAME(dev));
			avp->av_dfswait_run = 1;
			return 0;
		}
	} else {
		if (sc->sc_dfswait &&
			vap->iv_opmode == IEEE80211_M_HOSTAP &&
			sc->sc_dfswaittimer.data == (unsigned long)vap) {
			del_timer_sync(&sc->sc_dfswaittimer);
			sc->sc_dfswait = 0;
			DPRINTF(sc, ATH_DEBUG_STATE, "%s: %s: VAP  out of DFS_WAIT\n",
				__func__, DEV_NAME(dev));
		}

		/* XXX: if it is SCAN state, disable beacons. */
		if (nstate == IEEE80211_S_SCAN) {
			sc->sc_imask &= ~(HAL_INT_SWBA | HAL_INT_BMISS);
			ath_hal_intrset(ah, sc->sc_imask);
			/* need to reconfigure the beacons when it moves to RUN */
			sc->sc_beacons = 0;
		}
		avp->av_dfswait_run = 0; /* reset the dfs wait flag */
	}
done:
	/* Invoke the parent method to complete the work. */
	error = avp->av_newstate(vap, nstate, arg);

	/* Finally, restart the periodic recalibration timer */
	if (nstate == IEEE80211_S_RUN)
  		mod_timer(&sc->sc_cal_ch, jiffies + (ath_calinterval * HZ));
  
  #ifdef ATH_SUPERG_XR
	if ((vap->iv_flags & IEEE80211_F_XR) &&
			(nstate == IEEE80211_S_RUN))
  		ATH_SETUP_XR_VAP(sc, vap, rfilt);
	if ((vap->iv_flags & IEEE80211_F_XR) && 
			(nstate == IEEE80211_S_INIT) && (sc->sc_xrgrppoll))
		ath_grppoll_stop(vap);
#endif
bad:
	netif_start_queue(dev);
	dev->watchdog_timeo = 5 * HZ;		/* set the timeout to normal */
	return error;
}

/* periodically checks for the HAL to set
 * CHANNEL_DFS_CLEAR flag on current channel.
 * if the flag is set and a VAP is waiting for it, push
 * transition the VAP to RUN state.
 *
 * Context: Timer (softIRQ) */
static void
ath_check_dfs_clear(unsigned long data)
{
	struct ath_softc *sc = (struct ath_softc *)data;
	struct ieee80211com *ic = &sc->sc_ic;
	struct net_device *dev = sc->sc_dev;
	struct ieee80211vap *vap ;
	HAL_CHANNEL hchan;

	if (!sc->sc_dfswait) return;

	/* if still need to wait */
	ath_hal_radar_wait(sc->sc_ah, &hchan);

	if (hchan.privFlags & CHANNEL_INTERFERENCE)
		return;

	if ((hchan.privFlags & CHANNEL_DFS_CLEAR) ||
	    (!(hchan.privFlags & CHANNEL_DFS))) {
		sc->sc_curchan.privFlags |= CHANNEL_DFS_CLEAR;
		sc->sc_dfswait = 0;
		TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) {
			struct ath_vap *avp = ATH_VAP(vap);
			if (avp->av_dfswait_run) {
				/* re alloc beacons to update new channel info */
				int error;
				error = ath_beacon_alloc(sc, vap->iv_bss);
				if (error < 0) {
					/* XXX */
					return;
				}
				DPRINTF(sc, ATH_DEBUG_STATE, 
					"%s: %s: VAP DFS_WAIT -> RUN\n",
					__func__, DEV_NAME(dev));
				avp->av_newstate(vap, IEEE80211_S_RUN, 0);
				/* start calibration timer */
				mod_timer(&sc->sc_cal_ch, jiffies + 
						(ath_calinterval * HZ));
#ifdef ATH_SUPERG_XR
				if (vap->iv_flags & IEEE80211_F_XR) {
					u_int32_t rfilt = 0;
					rfilt = ath_calcrxfilter(sc);
					ATH_SETUP_XR_VAP(sc, vap, rfilt);
				}
#endif
				avp->av_dfswait_run = 0;
			}
		}
		/* start the device */
		netif_start_queue(dev);
		dev->watchdog_timeo = 5 * HZ;	/* set the timeout to normal */
	} else {
		/* fire the timer again */
		sc->sc_dfswaittimer.expires = 
			jiffies + (ATH_DFS_WAIT_SHORT_POLL_PERIOD * HZ);
		sc->sc_dfswaittimer.data = (unsigned long)sc;
		add_timer(&sc->sc_dfswaittimer);
	}

}

#ifdef ATH_SUPERG_COMP
/* Enable/Disable de-compression mask for given node.
 * The routine is invoked after addition or deletion of the
 * key.
 */
static void
ath_comp_set(struct ieee80211vap *vap, struct ieee80211_node *ni, int en)
{
	ath_setup_comp(ni, en);
	return;
}

/* Set up decompression engine for this node. */
static void
ath_setup_comp(struct ieee80211_node *ni, int enable)
{
#define	IEEE80211_KEY_XR	(IEEE80211_KEY_XMIT | IEEE80211_KEY_RECV)
	struct ieee80211vap *vap = ni->ni_vap;
	struct ath_softc *sc = vap->iv_ic->ic_dev->priv;
	struct ath_node *an = ATH_NODE(ni);
	ieee80211_keyix_t keyix;

	if (enable) {
		/* Have we negotiated compression? */
		if (!(ni->ni_ath_flags & IEEE80211_NODE_COMP))
			return;

		/* No valid key? */
		if (ni->ni_ucastkey.wk_keyix == IEEE80211_KEYIX_NONE)
			return;

		/* Setup decompression mask.
		 * For TKIP and split MIC case, recv. keyix is at 32 offset
		 * from tx key.
		 */
		if ((ni->ni_wpa_ie != NULL) &&
		    (ni->ni_rsn.rsn_ucastcipher == IEEE80211_CIPHER_TKIP) &&
		    sc->sc_splitmic) {
			if ((ni->ni_ucastkey.wk_flags & IEEE80211_KEY_XR)
							== IEEE80211_KEY_XR)
				keyix = ni->ni_ucastkey.wk_keyix + 32;
			else
				keyix = ni->ni_ucastkey.wk_keyix;
		} else
			keyix = ni->ni_ucastkey.wk_keyix + ni->ni_rxkeyoff;

		ath_hal_setdecompmask(sc->sc_ah, ATH_KEY(keyix), 1);
		an->an_decomp_index = keyix;
	} else {
		if (an->an_decomp_index != INVALID_DECOMP_INDEX) {
			ath_hal_setdecompmask(sc->sc_ah, an->an_decomp_index, 0);
			an->an_decomp_index = INVALID_DECOMP_INDEX;
		}
	}

	return;
#undef IEEE80211_KEY_XR
}
#endif

/*
 * Allocate a key cache slot to the station so we can
 * setup a mapping from key index to node. The key cache
 * slot is needed for managing antenna state and for
 * compression when stations do not use crypto.  We do
 * it unilaterally here; if crypto is employed this slot
 * will be reassigned.
 */
static void
ath_setup_stationkey(struct ieee80211_node *ni)
{
	struct ieee80211vap *vap = ni->ni_vap;
	struct ath_softc *sc = vap->iv_ic->ic_dev->priv;
	ieee80211_keyix_t keyix;

	keyix = ath_key_alloc(vap, &ni->ni_ucastkey);
	if (keyix == IEEE80211_KEYIX_NONE) {
		/*
		 * Key cache is full; we'll fall back to doing
		 * the more expensive lookup in software.  Note
		 * this also means no h/w compression.
		 */
		/* XXX msg+statistic */
		return;
	} else {
		ni->ni_ucastkey.wk_keyix = keyix;
		/* NB: this will create a pass-thru key entry */
		ath_keyset(sc, &ni->ni_ucastkey, ni->ni_macaddr, vap->iv_bss);

#ifdef ATH_SUPERG_COMP
		/* Enable de-compression logic */
		ath_setup_comp(ni, 1);
#endif
	}

	return;
}

/* Setup WEP key for the station if compression is negotiated.
 * When station and AP are using same default key index, use single key
 * cache entry for receive and transmit, else two key cache entries are
 * created. One for receive with MAC address of station and one for transmit
 * with NULL mac address. On receive key cache entry de-compression mask
 * is enabled.
 */
static void
ath_setup_stationwepkey(struct ieee80211_node *ni)
{
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211_key *ni_key;
	struct ieee80211_key tmpkey;
	struct ieee80211_key *rcv_key, *xmit_key;
	unsigned int i;
	ieee80211_keyix_t txkeyidx, rxkeyidx = IEEE80211_KEYIX_NONE;
	u_int8_t null_macaddr[IEEE80211_ADDR_LEN] = {0, 0, 0, 0, 0, 0};

	KASSERT(ni->ni_ath_defkeyindex < IEEE80211_WEP_NKID,
		("got invalid node key index 0x%x", ni->ni_ath_defkeyindex));
	KASSERT(vap->iv_def_txkey < IEEE80211_WEP_NKID,
		("got invalid vap def key index 0x%x", vap->iv_def_txkey));

	/* Allocate a key slot first */
	if (!ieee80211_crypto_newkey(vap,
		IEEE80211_CIPHER_WEP,
		IEEE80211_KEY_XMIT|IEEE80211_KEY_RECV,
		&ni->ni_ucastkey))
		goto error;

	txkeyidx = ni->ni_ucastkey.wk_keyix;
	xmit_key = &vap->iv_nw_keys[vap->iv_def_txkey];

	/* Do we need separate rx key? */
	if (ni->ni_ath_defkeyindex != vap->iv_def_txkey) {
		ni->ni_ucastkey.wk_keyix = IEEE80211_KEYIX_NONE;
		if (!ieee80211_crypto_newkey(vap,
			IEEE80211_CIPHER_WEP,
			IEEE80211_KEY_XMIT|IEEE80211_KEY_RECV,
			&ni->ni_ucastkey)) {
			ni->ni_ucastkey.wk_keyix = txkeyidx;
			ieee80211_crypto_delkey(vap, &ni->ni_ucastkey, ni);
			goto error;
		}
		rxkeyidx = ni->ni_ucastkey.wk_keyix;
		ni->ni_ucastkey.wk_keyix = txkeyidx;

		rcv_key = &vap->iv_nw_keys[ni->ni_ath_defkeyindex];
	} else {
		rcv_key = xmit_key;
		rxkeyidx = txkeyidx;
	}

	/* Remember receive key offset */
	ni->ni_rxkeyoff = rxkeyidx - txkeyidx;

	/* Setup xmit key */
	ni_key = &ni->ni_ucastkey;
	if (rxkeyidx != txkeyidx)
		ni_key->wk_flags = IEEE80211_KEY_XMIT;
	else
		ni_key->wk_flags = IEEE80211_KEY_XMIT|IEEE80211_KEY_RECV;

	ni_key->wk_keylen = xmit_key->wk_keylen;
	for (i = 0; i < IEEE80211_TID_SIZE; i++)
		ni_key->wk_keyrsc[i] = xmit_key->wk_keyrsc[i];
	ni_key->wk_keytsc = 0;
	memset(ni_key->wk_key, 0, sizeof(ni_key->wk_key));
	memcpy(ni_key->wk_key, xmit_key->wk_key, xmit_key->wk_keylen);
	ieee80211_crypto_setkey(vap, &ni->ni_ucastkey,
		(rxkeyidx == txkeyidx) ? ni->ni_macaddr:null_macaddr, ni);

	if (rxkeyidx != txkeyidx) {
		/* Setup recv key */
		ni_key = &tmpkey;
		ni_key->wk_keyix = rxkeyidx;
		ni_key->wk_flags = IEEE80211_KEY_RECV;
		ni_key->wk_keylen = rcv_key->wk_keylen;
		for (i = 0; i < IEEE80211_TID_SIZE; i++)
			ni_key->wk_keyrsc[i] = rcv_key->wk_keyrsc[i];
		ni_key->wk_keytsc = 0;
		ni_key->wk_cipher = rcv_key->wk_cipher;
		ni_key->wk_private = rcv_key->wk_private;
		memset(ni_key->wk_key, 0, sizeof(ni_key->wk_key));
		memcpy(ni_key->wk_key, rcv_key->wk_key, rcv_key->wk_keylen);
		ieee80211_crypto_setkey(vap, &tmpkey, ni->ni_macaddr, ni);
	}

	return;

error:
	ni->ni_ath_flags &= ~IEEE80211_NODE_COMP;
	return;
}

/* Create a keycache entry for given node in clearcase as well as static wep.
 * Handle compression state if required.
 * For non clearcase/static wep case, the key is plumbed by hostapd.
 */
static void
ath_setup_keycacheslot(struct ath_softc *sc, struct ieee80211_node *ni)
{
	struct ieee80211vap *vap = ni->ni_vap;

	if (ni->ni_ucastkey.wk_keyix != IEEE80211_KEYIX_NONE)
		ieee80211_crypto_delkey(vap, &ni->ni_ucastkey, ni);

	/* Only for clearcase and WEP case */
	if ((vap->iv_flags & IEEE80211_F_PRIVACY) == 0 ||
		(ni->ni_ath_defkeyindex != IEEE80211_INVAL_DEFKEY)) {

		if ((vap->iv_flags & IEEE80211_F_PRIVACY) == 0) {
			KASSERT(ni->ni_ucastkey.wk_keyix == IEEE80211_KEYIX_NONE,
				("new node with a ucast key already setup (keyix %u)",
				  ni->ni_ucastkey.wk_keyix));
			/* NB: 5210 has no passthru/clr key support */
			if (sc->sc_hasclrkey)
				ath_setup_stationkey(ni);
		} else
			ath_setup_stationwepkey(ni);
	}

	return;
}

/*
 * Setup driver-specific state for a newly associated node.
 * Note that we're called also on a re-associate, the isnew
 * param tells us if this is the first time or not.
 */
static void
ath_newassoc(struct ieee80211_node *ni, int isnew)
{
	struct ieee80211com *ic = ni->ni_ic;
	struct ieee80211vap *vap = ni->ni_vap;
	struct ath_softc *sc = ic->ic_dev->priv;

	sc->sc_rc->ops->newassoc(sc, ATH_NODE(ni), isnew);

	/* are we supporting compression? */
	if (!(vap->iv_ath_cap & ni->ni_ath_flags & IEEE80211_NODE_COMP))
		ni->ni_ath_flags &= ~IEEE80211_NODE_COMP;

	/* disable compression for TKIP */
	if ((ni->ni_ath_flags & IEEE80211_NODE_COMP) &&
		(ni->ni_wpa_ie != NULL) &&
		(ni->ni_rsn.rsn_ucastcipher == IEEE80211_CIPHER_TKIP))
		ni->ni_ath_flags &= ~IEEE80211_NODE_COMP;

	ath_setup_keycacheslot(sc, ni);
#ifdef ATH_SUPERG_XR
	if (1) {
		struct ath_node *an = ATH_NODE(ni);
		if (ic->ic_ath_cap & an->an_node.ni_ath_flags & IEEE80211_ATHC_XR)
			an->an_minffrate = ATH_MIN_FF_RATE;
		else
			an->an_minffrate = 0;
		ath_grppoll_period_update(sc);
	}
#endif
}

static int
ath_getchannels(struct net_device *dev, u_int cc,
	HAL_BOOL outdoor, HAL_BOOL xchanmode)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	HAL_CHANNEL *chans;
	unsigned int i;
	u_int nchan;

	chans = kmalloc(IEEE80211_CHAN_MAX * sizeof(HAL_CHANNEL), GFP_KERNEL);
	if (chans == NULL) {
		printk("%s: unable to allocate channel table\n", DEV_NAME(dev));
		return -ENOMEM;
	}
	if (!ath_hal_init_channels(ah, chans, IEEE80211_CHAN_MAX, &nchan,
	    ic->ic_regclassids, IEEE80211_REGCLASSIDS_MAX, &ic->ic_nregclass,
	    cc, HAL_MODE_ALL, outdoor, xchanmode)) {
		u_int32_t rd;

		ath_hal_getregdomain(ah, &rd);
		printk("%s: unable to collect channel list from HAL; "
			"regdomain likely %u country code %u\n",
			DEV_NAME(dev), rd, cc);
		kfree(chans);
		return -EINVAL;
	}
	/*
	 * Convert HAL channels to ieee80211 ones.
	 */
	printk(KERN_INFO "HAL returned %d channels.\n", nchan);
	for (i = 0; i < nchan; i++) {

		HAL_CHANNEL *c = &chans[i];
		struct ieee80211_channel *ichan = &ic->ic_channels[i];

		/* Force re-check.
		 * XXX: Unclear whether regs say you can avoid the channel
		 * availability check if you've already performed it on the
		 * channel within some more brief interval. */
		c->privFlags		&= ~CHANNEL_DFS_CLEAR;

		/* Initialize all fields of ieee80211_channel here */

		ichan->ic_freq		= c->channel;
		ichan->ic_flags		= c->channelFlags;
		ichan->ic_ieee		= ath_hal_mhz2ieee(ah,
							   c->channel,
							   c->channelFlags);
		ichan->ic_maxregpower	= c->maxRegTxPower;	/* dBm */
		ichan->ic_maxpower	= c->maxTxPower;	/* 1/2 dBm */
		ichan->ic_minpower	= c->minTxPower;	/* 1/2 dBm */
		ichan->ic_non_occupancy_period.tv_sec  = 0;
		ichan->ic_non_occupancy_period.tv_usec = 0;

		printk(KERN_INFO "Channel %3d (%4d MHz) Max Tx Power %d dBm%s [%d hw %d reg] Flags%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s\n"
			, ichan->ic_ieee
			, c->channel
			, c->maxRegTxPower > (c->maxTxPower/2) ? (c->maxTxPower/2) : c->maxRegTxPower
			, ( c->maxRegTxPower == (c->maxTxPower/2) ? "" : ( (c->maxRegTxPower > (c->maxTxPower/2)) ? " (hw limited)" : " (reg limited)" ) )
			, (c->maxTxPower/2)
			, c->maxRegTxPower
			, (c->channelFlags & 0x0001               ? " CF & (1<<0)"          :       "" /* undocumented */)
			, (c->channelFlags & CHANNEL_CW_INT       ? " CF_CW_INTERFERENCE"   :       "" /* CW interference detected on channel */)
			, (c->channelFlags & 0x0004               ? " CF & (1<<2)"          :       "" /* undocumented */)
			, (c->channelFlags & 0x0008               ? " CF & (1<<3)"          :       "" /* undocumented */)
			, (c->channelFlags & CHANNEL_TURBO        ? " CF_TURBO"             :       "" /* Turbo channel */)
			, (c->channelFlags & CHANNEL_CCK          ? " CF_CCK"               :       "" /* CCK channel */)
			, (c->channelFlags & CHANNEL_OFDM         ? " CF_OFDM"              :       "" /* OFDM channel */)
			, (c->channelFlags & CHANNEL_2GHZ         ? " CF_2GHZ"              :       "" /* 2GHz spectrum channel. */)
			, (c->channelFlags & CHANNEL_5GHZ         ? " CF_5GHZ"              :       "" /* 5GHz spectrum channel */)
			, (c->channelFlags & CHANNEL_PASSIVE      ? " CF_PASSIVE_SCAN_ONLY" :       "" /* Only passive scan allowed */)
			, (c->channelFlags & CHANNEL_DYN          ? " CF_DYNAMIC_TURBO"     :       "" /* Dynamic CCK-OFDM channel */)
			, (c->channelFlags & CHANNEL_XR           ? " CF_FHSS"              :       "" /* GFSK channel (FHSS  PHY) */)
			, (c->channelFlags & IEEE80211_CHAN_RADAR ? " CF_RADAR_SEEN"        :       "" /* Radar found on channel */)
			, (c->channelFlags & CHANNEL_STURBO       ? " CF_STATIC_TURBO"      :       "" /* 11a static turbo channel only  */)
			, (c->channelFlags & CHANNEL_HALF         ? " CF_HALF_RATE"         :       "" /* Half rate channel */)
			, (c->channelFlags & CHANNEL_QUARTER      ? " CF_QUARTER_RATE"      :       "" /* Quarter rate channel */)
			, (c->privFlags    & CHANNEL_INTERFERENCE ? " PF_INTERFERENCE"      :       "" /* Software use: channel interference used for as AR as well as RADAR interference detection */)
			, (c->privFlags    & CHANNEL_DFS          ? " PF_DFS_REQUIRED"      :       "" /* DFS required on channel */)
			, (c->privFlags    & CHANNEL_4MS_LIMIT    ? " PF_4MS_LIMIT"         :       "" /* 4msec packet limit on this channel */)
			, (c->privFlags    & CHANNEL_DFS_CLEAR    ? " PF_DFS_CLEAR"         :       "" /* if channel has been checked for DFS */)
			, (c->privFlags    & 0x0010               ? " PF & (1<<4)"          :       "" /* undocumented */)
			, (c->privFlags    & 0x0020               ? " PF & (1<<5)"          :       "" /* undocumented */)
			, (c->privFlags    & 0x0040               ? " PF & (1<<6)"          :       "" /* undocumented */)
			, (c->privFlags    & 0x0080               ? " PF & (1<<7)"          :       "" /* undocumented */)
			);
	}
	ic->ic_nchans = nchan;
	kfree(chans);
	return 0;
}

static void
ath_led_done(unsigned long arg)
{
	struct ath_softc *sc = (struct ath_softc *)arg;

	sc->sc_blinking = 0;
}

/*
 * Turn the LED off: flip the pin and then set a timer so no
 * update will happen for the specified duration.
 */
static void
ath_led_off(unsigned long arg)
{
	struct ath_softc *sc = (struct ath_softc *)arg;

	ath_hal_gpioset(sc->sc_ah, sc->sc_ledpin, !sc->sc_ledon);
	sc->sc_ledtimer.function = ath_led_done;
	sc->sc_ledtimer.expires = jiffies + sc->sc_ledoff;
	add_timer(&sc->sc_ledtimer);
}

/*
 * Blink the LED according to the specified on/off times.
 */
static void
ath_led_blink(struct ath_softc *sc, int on, int off)
{
	DPRINTF(sc, ATH_DEBUG_LED, "%s: on %u off %u\n", __func__, on, off);
	ath_hal_gpioset(sc->sc_ah, sc->sc_ledpin, sc->sc_ledon);
	sc->sc_blinking = 1;
	sc->sc_ledoff = off;
	sc->sc_ledtimer.function = ath_led_off;
	sc->sc_ledtimer.expires = jiffies + on;
	add_timer(&sc->sc_ledtimer);
}

static void
ath_led_event(struct ath_softc *sc, int event)
{

	sc->sc_ledevent = jiffies;	/* time of last event */
	if (sc->sc_blinking)		/* don't interrupt active blink */
		return;
	switch (event) {
	case ATH_LED_POLL:
		ath_led_blink(sc, sc->sc_hwmap[0].ledon,
			sc->sc_hwmap[0].ledoff);
		break;
	case ATH_LED_TX:
		ath_led_blink(sc, sc->sc_hwmap[sc->sc_txrate].ledon,
			sc->sc_hwmap[sc->sc_txrate].ledoff);
		break;
	case ATH_LED_RX:
		ath_led_blink(sc, sc->sc_hwmap[sc->sc_rxrate].ledon,
			sc->sc_hwmap[sc->sc_rxrate].ledoff);
		break;
	}
}

static void
set_node_txpower(void *arg, struct ieee80211_node *ni)
{
	int *value = (int *)arg;
	ni->ni_txpower = *value;
}

/* The HAL supports a maxtxpow which is something we can configure to be the
minimum of the regulatory constraint and the limits of the radio.
XXX: this function needs some locking to avoid being called twice/interrupted
Returns the value actually stored. */
static u_int32_t
ath_set_clamped_maxtxpower(struct ath_softc *sc, u_int32_t new_clamped_maxtxpower)
{
	(void)ath_hal_settxpowlimit(sc->sc_ah, new_clamped_maxtxpower);
	return ath_get_clamped_maxtxpower(sc);
}

/* The HAL supports a maxtxpow which is something we can configure to be the
minimum of the regulatory constraint and the limits of the radio.
XXX: this function needs some locking to avoid being called twice/interrupted */
static u_int32_t
ath_get_clamped_maxtxpower(struct ath_softc *sc)
{
	u_int32_t clamped_maxtxpower;
	(void)ath_hal_getmaxtxpow(sc->sc_ah, &clamped_maxtxpower);
	return clamped_maxtxpower;
}

/* XXX: this function needs some locking to avoid being called twice/interrupted */
/* 1.  Save the currently specified maximum txpower (as clamped by madwifi)
 * 2.  Determine the real maximum txpower the card can support by
 *     setting a value that exceeds the maximum range (by one) and
 *     finding out what it limits us to.
 * 3.  Restore the saved maxtxpower value we had previously specified */
static u_int32_t
ath_get_real_maxtxpower(struct ath_softc *sc)
{

	u_int32_t saved_clamped_maxtxpower;
	u_int32_t real_maxtxpower;

	saved_clamped_maxtxpower = ath_get_clamped_maxtxpower(sc);
	real_maxtxpower = ath_set_clamped_maxtxpower(sc, IEEE80211_TXPOWER_MAX + 1);
	ath_set_clamped_maxtxpower(sc, saved_clamped_maxtxpower);
	return real_maxtxpower;
}


/* XXX: this function needs some locking to avoid being called twice/interrupted */
static void
ath_update_txpow(struct ath_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211vap *vap = NULL;
	struct ath_hal *ah = sc->sc_ah;
	u_int32_t prev_clamped_maxtxpower = 0;
	u_int32_t new_clamped_maxtxpower = 0;

	/* Determine the previous value of maxtxpower */
	prev_clamped_maxtxpower = ath_get_clamped_maxtxpower(sc);
	/* Determine the real maximum txpower the card can support */
	ic->ic_txpowlimit = ath_get_real_maxtxpower(sc);
	/* Grab the new maxtxpower setting (which may have changed) */
	new_clamped_maxtxpower = ic->ic_newtxpowlimit;
	/* Make sure the change is within limits, clamp it otherwise */
	if (ic->ic_newtxpowlimit > ic->ic_txpowlimit)
		new_clamped_maxtxpower = ic->ic_txpowlimit;
	/* * Search for the VAP that needs a txpow change, if any */
	TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) {
		if (!tpc || ic->ic_newtxpowlimit != vap->iv_bss->ni_txpower) {
			vap->iv_bss->ni_txpower = new_clamped_maxtxpower;
			ieee80211_iterate_nodes(&vap->iv_ic->ic_sta, set_node_txpower, &new_clamped_maxtxpower);
		}
	}

	/* Store the assigned (clamped) maximum txpower and update the HAL */
	sc->sc_curtxpow = new_clamped_maxtxpower;
	if (new_clamped_maxtxpower != prev_clamped_maxtxpower)
		ath_hal_settxpowlimit(ah, new_clamped_maxtxpower);
}


#ifdef ATH_SUPERG_XR
static int
ath_xr_rate_setup(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	const HAL_RATE_TABLE *rt;
	struct ieee80211_rateset *rs;
	unsigned int i, maxrates;
	sc->sc_xr_rates = ath_hal_getratetable(ah, HAL_MODE_XR);
	rt = sc->sc_xr_rates;
	if (rt == NULL)
		return 0;
	if (rt->rateCount > IEEE80211_RATE_MAXSIZE) {
		DPRINTF(sc, ATH_DEBUG_ANY,
			"%s: rate table too small (%u > %u)\n",
			__func__, rt->rateCount, IEEE80211_RATE_MAXSIZE);
		maxrates = IEEE80211_RATE_MAXSIZE;
	} else
		maxrates = rt->rateCount;
	rs = &ic->ic_sup_xr_rates;
	for (i = 0; i < maxrates; i++)
		rs->rs_rates[i] = rt->info[i].dot11Rate;
	rs->rs_nrates = maxrates;
	return 1;
}
#endif

/* Setup half/quarter rate table support */
static void
ath_setup_subrates(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	const HAL_RATE_TABLE *rt;
	struct ieee80211_rateset *rs;
	unsigned int i, maxrates;

	sc->sc_half_rates = ath_hal_getratetable(ah, HAL_MODE_11A_HALF_RATE);
	rt = sc->sc_half_rates;
	if (rt != NULL) {
		if (rt->rateCount > IEEE80211_RATE_MAXSIZE) {
			DPRINTF(sc, ATH_DEBUG_ANY,
				"%s: rate table too small (%u > %u)\n",
			       __func__, rt->rateCount, IEEE80211_RATE_MAXSIZE);
			maxrates = IEEE80211_RATE_MAXSIZE;
		} else
			maxrates = rt->rateCount;
		rs = &ic->ic_sup_half_rates;
		for (i = 0; i < maxrates; i++)
			rs->rs_rates[i] = rt->info[i].dot11Rate;
		rs->rs_nrates = maxrates;
	}

	sc->sc_quarter_rates = ath_hal_getratetable(ah, HAL_MODE_11A_QUARTER_RATE);
	rt = sc->sc_quarter_rates;
	if (rt != NULL) {
		if (rt->rateCount > IEEE80211_RATE_MAXSIZE) {
			DPRINTF(sc, ATH_DEBUG_ANY,
				"%s: rate table too small (%u > %u)\n",
			       __func__, rt->rateCount, IEEE80211_RATE_MAXSIZE);
			maxrates = IEEE80211_RATE_MAXSIZE;
		} else
			maxrates = rt->rateCount;
		rs = &ic->ic_sup_quarter_rates;
		for (i = 0; i < maxrates; i++)
			rs->rs_rates[i] = rt->info[i].dot11Rate;
		rs->rs_nrates = maxrates;
	}
}

static int
ath_rate_setup(struct net_device *dev, u_int mode)
{
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	const HAL_RATE_TABLE *rt;
	struct ieee80211_rateset *rs;
	unsigned int i, maxrates;

	switch (mode) {
	case IEEE80211_MODE_11A:
		sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_11A);
		break;
	case IEEE80211_MODE_11B:
		sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_11B);
		break;
	case IEEE80211_MODE_11G:
		sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_11G);
		break;
	case IEEE80211_MODE_TURBO_A:
		sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_TURBO);
		break;
	case IEEE80211_MODE_TURBO_G:
		sc->sc_rates[mode] = ath_hal_getratetable(ah, HAL_MODE_108G);
		break;
	default:
		DPRINTF(sc, ATH_DEBUG_ANY, "%s: invalid mode %u\n",
			__func__, mode);
		return 0;
	}
	rt = sc->sc_rates[mode];
	if (rt == NULL)
		return 0;
	if (rt->rateCount > IEEE80211_RATE_MAXSIZE) {
		DPRINTF(sc, ATH_DEBUG_ANY,
			"%s: rate table too small (%u > %u)\n",
			__func__, rt->rateCount, IEEE80211_RATE_MAXSIZE);
		maxrates = IEEE80211_RATE_MAXSIZE;
	} else
		maxrates = rt->rateCount;
	rs = &ic->ic_sup_rates[mode];
	for (i = 0; i < maxrates; i++)
		rs->rs_rates[i] = rt->info[i].dot11Rate;
	rs->rs_nrates = maxrates;
	return 1;
}

static void
ath_setcurmode(struct ath_softc *sc, enum ieee80211_phymode mode)
{
	/* NB: on/off times from the Atheros NDIS driver, w/ permission */
	static const struct {
		u_int		rate;		/* tx/rx 802.11 rate */
		u_int16_t	timeOn;		/* LED on time (ms) */
		u_int16_t	timeOff;	/* LED off time (ms) */
	} blinkrates[] = {
		{ 108,  40,  10 },
		{  96,  44,  11 },
		{  72,  50,  13 },
		{  48,  57,  14 },
		{  36,  67,  16 },
		{  24,  80,  20 },
		{  22, 100,  25 },
		{  18, 133,  34 },
		{  12, 160,  40 },
		{  10, 200,  50 },
		{   6, 240,  58 },
		{   4, 267,  66 },
		{   2, 400, 100 },
		{   0, 500, 130 },
	};
	const HAL_RATE_TABLE *rt;
	unsigned int i, j;

	memset(sc->sc_rixmap, 0xff, sizeof(sc->sc_rixmap));
	rt = sc->sc_rates[mode];
	KASSERT(rt != NULL, ("no h/w rate set for phy mode %u", mode));
	for (i = 0; i < rt->rateCount; i++)
		sc->sc_rixmap[rt->info[i].dot11Rate & IEEE80211_RATE_VAL] = i;
	memset(sc->sc_hwmap, 0, sizeof(sc->sc_hwmap));
	for (i = 0; i < 32; i++) {
		u_int8_t ix = rt->rateCodeToIndex[i];
		if (ix == 0xff) {
			sc->sc_hwmap[i].ledon = msecs_to_jiffies(500);
			sc->sc_hwmap[i].ledoff = msecs_to_jiffies(130);
			continue;
		}
		sc->sc_hwmap[i].ieeerate =
			rt->info[ix].dot11Rate & IEEE80211_RATE_VAL;
		if (rt->info[ix].shortPreamble ||
		    rt->info[ix].phy == IEEE80211_T_OFDM)
			sc->sc_hwmap[i].flags |= IEEE80211_RADIOTAP_F_SHORTPRE;
		/* setup blink rate table to avoid per-packet lookup */
		for (j = 0; j < ARRAY_SIZE(blinkrates) - 1; j++)
			if (blinkrates[j].rate == sc->sc_hwmap[i].ieeerate)
				break;
		/* NB: this uses the last entry if the rate isn't found */
		/* XXX beware of overflow */
		sc->sc_hwmap[i].ledon = msecs_to_jiffies(blinkrates[j].timeOn);
		sc->sc_hwmap[i].ledoff = msecs_to_jiffies(blinkrates[j].timeOff);
	}
	sc->sc_currates = rt;
	sc->sc_curmode = mode;
	/*
	 * All protection frames are transmitted at 2Mb/s for
	 * 11g, otherwise at 1Mb/s.
	 * XXX select protection rate index from rate table.
	 */
	sc->sc_protrix = (mode == IEEE80211_MODE_11G ? 1 : 0);
	/* rate index used to send mgt frames */
	sc->sc_minrateix = 0;
}

#ifdef ATH_SUPERG_FF
static u_int32_t
athff_approx_txtime(struct ath_softc *sc, struct ath_node *an, struct sk_buff *skb)
{
	u_int32_t txtime;
	u_int32_t framelen;

	/*
	 * Approximate the frame length to be transmitted. A swag to add
	 * the following maximal values to the skb payload:
	 *   - 32: 802.11 encap + CRC
	 *   - 24: encryption overhead (if wep bit)
	 *   - 4 + 6: fast-frame header and padding
	 *   - 16: 2 LLC FF tunnel headers
	 *   - 14: 1 802.3 FF tunnel header (skb already accounts for 2nd)
	 */
	framelen = skb->len + 32 + 4 + 6 + 16 + 14;
	if (sc->sc_ic.ic_flags & IEEE80211_F_PRIVACY)
		framelen += 24;
	if (an->an_tx_ffbuf[skb->priority])
		framelen += an->an_tx_ffbuf[skb->priority]->bf_skb->len;

	txtime = ath_hal_computetxtime(sc->sc_ah, sc->sc_currates, framelen,
		an->an_prevdatarix, AH_FALSE);

	return txtime;
}
/*
 * Determine if a data frame may be aggregated via ff tunneling.
 *
 *  NB: allowing EAPOL frames to be aggregated with other unicast traffic.
 *      Do 802.1x EAPOL frames proceed in the clear? Then they couldn't
 *      be aggregated with other types of frames when encryption is on?
 *
 *  NB: assumes lock on an_tx_ffbuf effectively held by txq lock mechanism.
 */
static int
athff_can_aggregate(struct ath_softc *sc, struct ether_header *eh,
		    struct ath_node *an, struct sk_buff *skb, 
		    u_int16_t fragthreshold, int *flushq)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_txq *txq = sc->sc_ac2q[skb->priority];
	struct ath_buf *ffbuf = an->an_tx_ffbuf[skb->priority];
	u_int32_t txoplimit;

#define US_PER_4MS 4000
#define	MIN(a,b)	((a) < (b) ? (a) : (b))

	*flushq = AH_FALSE;

	if (fragthreshold < 2346)
		return AH_FALSE;

	if ((!ffbuf) && (txq->axq_depth < sc->sc_fftxqmin))
		return AH_FALSE;
	if (!(ic->ic_ath_cap & an->an_node.ni_ath_flags & IEEE80211_ATHC_FF))
		return AH_FALSE;
	if (!(ic->ic_opmode == IEEE80211_M_STA ||
		  ic->ic_opmode == IEEE80211_M_HOSTAP))
		return AH_FALSE;
	if ((ic->ic_opmode == IEEE80211_M_HOSTAP) &&
		  ETHER_IS_MULTICAST(eh->ether_dhost))
		return AH_FALSE;

#ifdef ATH_SUPERG_XR
	if (sc->sc_currates->info[an->an_prevdatarix].rateKbps < an->an_minffrate)
		return AH_FALSE;
#endif
	txoplimit = IEEE80211_TXOP_TO_US(
		ic->ic_wme.wme_chanParams.cap_wmeParams[skb->priority].wmep_txopLimit);

	/* if the 4 msec limit is set on the channel, take it into account */
	if (sc->sc_curchan.privFlags & CHANNEL_4MS_LIMIT)
		txoplimit = MIN(txoplimit, US_PER_4MS);

	if (txoplimit != 0 && athff_approx_txtime(sc, an, skb) > txoplimit) {
		DPRINTF(sc, ATH_DEBUG_XMIT | ATH_DEBUG_FF,
			"%s: FF TxOp violation\n", __func__);
		if (ffbuf)
			*flushq = AH_TRUE;
		return AH_FALSE;
	}

	return AH_TRUE;

#undef US_PER_4MS
#undef MIN
}
#endif

#ifdef AR_DEBUG

static void
ath_printrxbuf(const struct ath_buf *bf, int done)
{
	const struct ath_rx_status *rs = &bf->bf_dsstatus.ds_rxstat;
	const struct ath_desc *ds = bf->bf_desc;
	u_int8_t status = done ? rs->rs_status : 0;
	printk("R (%p %llx) %08x %08x %08x %08x %08x %08x%s%s%s%s%s%s%s%s%s\n",
	    ds, ito64(bf->bf_daddr),
	    ds->ds_link, ds->ds_data,
	    ds->ds_ctl0, ds->ds_ctl1,
	    ds->ds_hw[0], ds->ds_hw[1],
	    status                              ? ""            	: " OK",
	    status & HAL_RXERR_CRC 		? " ERR_CRC" 		: "",
	    status & HAL_RXERR_PHY 		? " ERR_PHY" 		: "",
	    status & HAL_RXERR_FIFO 		? " ERR_FIFO"		: "",
	    status & HAL_RXERR_DECRYPT	 	? " ERR_DECRYPT"	: "",
	    status & HAL_RXERR_MIC 		? " ERR_MIC" 		: "",
	    status & 0x20 			? " (1<<5)" 		: "",
	    status & 0x40 			? " (1<<6)"		: "",
	    status & 0x80 			? " (1<<7)"		: "");
}

static void
ath_printtxbuf(const struct ath_buf *bf, int done)
{
	const struct ath_tx_status *ts = &bf->bf_dsstatus.ds_txstat;
	const struct ath_desc *ds = bf->bf_desc;
	u_int8_t status = done ? ts->ts_status : 0;
	printk("T (%p %llx) %08x %08x %08x %08x %08x %08x %08x %08x%s%s%s%s%s%s%s%s%s\n",
	    ds, ito64(bf->bf_daddr),
	    ds->ds_link, ds->ds_data,
	    ds->ds_ctl0, ds->ds_ctl1,
	    ds->ds_hw[0], ds->ds_hw[1], ds->ds_hw[2], ds->ds_hw[3],
	    status 				? "" 			: " OK",
	    status & HAL_TXERR_XRETRY		? " ERR_XRETRY" 	: "",
	    status & HAL_TXERR_FILT		? " ERR_FILT" 		: "",
	    status & HAL_TXERR_FIFO 		? " ERR_FIFO" 		: "",
	    status & HAL_TXERR_XTXOP 		? " ERR_XTXOP" 		: "",
	    status & HAL_TXERR_DESC_CFG_ERR 	? " ERR_DESC_CFG_ERR" 	: "",
	    status & HAL_TXERR_DATA_UNDERRUN	? " ERR_DATA_UNDERRUN" 	: "",
	    status & HAL_TXERR_DELIM_UNDERRUN	? " ERR_DELIM_UNDERRUN" : "",
	    status & 0x80 			? " (1<<7)" 		: "");
}
#endif /* AR_DEBUG */

/*
 * Return netdevice statistics.
 */
static struct net_device_stats *
ath_getstats(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct net_device_stats *stats = &sc->sc_devstats;

	/* update according to private statistics */
	stats->tx_errors = sc->sc_stats.ast_tx_xretries
			 + sc->sc_stats.ast_tx_fifoerr
			 + sc->sc_stats.ast_tx_filtered;
	stats->tx_dropped = sc->sc_stats.ast_tx_nobuf
			+ sc->sc_stats.ast_tx_encap
			+ sc->sc_stats.ast_tx_nonode
			+ sc->sc_stats.ast_tx_nobufmgt;
	stats->rx_errors = sc->sc_stats.ast_rx_fifoerr
			+ sc->sc_stats.ast_rx_badcrypt
			+ sc->sc_stats.ast_rx_badmic;
	stats->rx_dropped = sc->sc_stats.ast_rx_tooshort;
	stats->rx_crc_errors = sc->sc_stats.ast_rx_crcerr;

	return stats;
}

static int
ath_set_mac_address(struct net_device *dev, void *addr)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;
	struct sockaddr *mac = addr;
	int error = 0;

	if (netif_running(dev)) {
		DPRINTF(sc, ATH_DEBUG_ANY,
			"%s: cannot set address; device running\n", __func__);
		return -EBUSY;
	}
	DPRINTF(sc, ATH_DEBUG_ANY, "%s: %.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n",
		__func__,
		mac->sa_data[0], mac->sa_data[1], mac->sa_data[2],
		mac->sa_data[3], mac->sa_data[4], mac->sa_data[5]);

	ATH_LOCK(sc);
	/* XXX not right for multiple VAPs */
	IEEE80211_ADDR_COPY(ic->ic_myaddr, mac->sa_data);
	IEEE80211_ADDR_COPY(dev->dev_addr, mac->sa_data);
	ath_hal_setmac(ah, dev->dev_addr);
	if ((dev->flags & IFF_RUNNING) && !sc->sc_invalid) {
		error = ath_reset(dev);
	}
	ATH_UNLOCK(sc);

	return error;
}

static int
ath_change_mtu(struct net_device *dev, int mtu)
{
	struct ath_softc *sc = dev->priv;
	int error = 0;

	if (!(ATH_MIN_MTU < mtu && mtu <= ATH_MAX_MTU)) {
		DPRINTF(sc, ATH_DEBUG_ANY, "%s: invalid %d, min %u, max %u\n",
			__func__, mtu, ATH_MIN_MTU, ATH_MAX_MTU);
		return -EINVAL;
	}
	DPRINTF(sc, ATH_DEBUG_ANY, "%s: %d\n", __func__, mtu);

	ATH_LOCK(sc);
	dev->mtu = mtu;
	if ((dev->flags & IFF_RUNNING) && !sc->sc_invalid) {
		/* NB: the rx buffers may need to be reallocated */
		tasklet_disable(&sc->sc_rxtq);
		error = ath_reset(dev);
		tasklet_enable(&sc->sc_rxtq);
	}
	ATH_UNLOCK(sc);

	return error;
}

/*
 * Diagnostic interface to the HAL.  This is used by various
 * tools to do things like retrieve register contents for
 * debugging.  The mechanism is intentionally opaque so that
 * it can change frequently w/o concern for compatibility.
 */
static int
ath_ioctl_diag(struct ath_softc *sc, struct ath_diag *ad)
{
	struct ath_hal *ah = sc->sc_ah;
	u_int id = ad->ad_id & ATH_DIAG_ID;
	void *indata = NULL;
	void *outdata = NULL;
	u_int32_t insize = ad->ad_in_size;
	u_int32_t outsize = ad->ad_out_size;
	int error = 0;

	if (ad->ad_id & ATH_DIAG_IN) {
		/*
		 * Copy in data.
		 */
		indata = kmalloc(insize, GFP_KERNEL);
		if (indata == NULL) {
			error = -ENOMEM;
			goto bad;
		}
		if (copy_from_user(indata, ad->ad_in_data, insize)) {
			error = -EFAULT;
			goto bad;
		}
	}
	if (ad->ad_id & ATH_DIAG_DYN) {
		/*
		 * Allocate a buffer for the results (otherwise the HAL
		 * returns a pointer to a buffer where we can read the
		 * results).  Note that we depend on the HAL leaving this
		 * pointer for us to use below in reclaiming the buffer;
		 * may want to be more defensive.
		 */
		outdata = kmalloc(outsize, GFP_KERNEL);
		if (outdata == NULL) {
			error = -ENOMEM;
			goto bad;
		}
	}
	if (ath_hal_getdiagstate(ah, id, indata, insize, &outdata, &outsize)) {
		if (outsize < ad->ad_out_size)
			ad->ad_out_size = outsize;
		if (outdata &&
		    copy_to_user(ad->ad_out_data, outdata, ad->ad_out_size))
			error = -EFAULT;
	} else
		error = -EINVAL;
bad:
	if ((ad->ad_id & ATH_DIAG_IN) && indata != NULL)
		kfree(indata);
	if ((ad->ad_id & ATH_DIAG_DYN) && outdata != NULL)
		kfree(outdata);
	return error;
}

static int
ath_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	int error;

	ATH_LOCK(sc);
	switch (cmd) {
	case SIOCGATHSTATS:
		sc->sc_stats.ast_tx_packets = sc->sc_devstats.tx_packets;
		sc->sc_stats.ast_rx_packets = sc->sc_devstats.rx_packets;
		sc->sc_stats.ast_rx_rssi = ieee80211_getrssi(ic);
		if (copy_to_user(ifr->ifr_data, &sc->sc_stats, 
					sizeof (sc->sc_stats)))
			error = -EFAULT;
		else
			error = 0;
		break;
	case SIOCGATHDIAG:
		if (!capable(CAP_NET_ADMIN))
			error = -EPERM;
		else
			error = ath_ioctl_diag(sc, (struct ath_diag *)ifr);
		break;
	case SIOCETHTOOL:
		if (copy_from_user(&cmd, ifr->ifr_data, sizeof(cmd)))
			error = -EFAULT;
		else
			error = ath_ioctl_ethtool(sc, cmd, ifr->ifr_data);
		break;
	case SIOC80211IFCREATE:
		error = ieee80211_ioctl_create_vap(ic, ifr, dev);
		break;
	default:
		error = -EINVAL;
		break;
	}
	ATH_UNLOCK(sc);
	return error;
}

/*
 * Sysctls are split into ``static'' and ``dynamic'' tables.
 * The former are defined at module load time and are used
 * control parameters common to all devices.  The latter are
 * tied to particular device instances and come and go with
 * each device.  The split is currently a bit tenuous; many of
 * the static ones probably should be dynamic but having them
 * static (e.g. debug) means they can be set after a module is
 * loaded and before bringing up a device.  The alternative
 * is to add module parameters.
 */

/*
 * Dynamic (i.e. per-device) sysctls.  These are automatically
 * mirrored in /proc/sys.
 */
enum {
	ATH_SLOTTIME		= 1,
	ATH_ACKTIMEOUT		= 2,
	ATH_CTSTIMEOUT		= 3,
	ATH_SOFTLED		= 4,
	ATH_LEDPIN		= 5,
	ATH_COUNTRYCODE		= 6,
	ATH_REGDOMAIN		= 7,
	ATH_DEBUG		= 8,
	ATH_TXANTENNA		= 9,
	ATH_RXANTENNA		= 10,
	ATH_DIVERSITY		= 11,
	ATH_TXINTRPERIOD	= 12,
	ATH_FFTXQMIN		= 13,
	ATH_XR_POLL_PERIOD	= 14,
	ATH_XR_POLL_COUNT	= 15,
	ATH_ACKRATE		= 16,
};

static int
ATH_SYSCTL_DECL(ath_sysctl_halparam, ctl, write, filp, buffer, lenp, ppos)
{
	struct ath_softc *sc = ctl->extra1;
	struct ath_hal *ah = sc->sc_ah;
	u_int val;
	int ret = 0;

	ctl->data = &val;
	ctl->maxlen = sizeof(val);
	ATH_LOCK(sc);
	if (write) {
		ret = ATH_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer, 
				lenp, ppos);
		if (ret == 0) {
			switch ((long)ctl->extra2) {
			case ATH_SLOTTIME:
				if (val > 0) {
					if (!ath_hal_setslottime(ah, val))
						ret = -EINVAL;
					else
						sc->sc_slottimeconf = val;
				} else {
					/* disable manual override */
					sc->sc_slottimeconf = 0;
					ath_setslottime(sc);
				}
				break;
			case ATH_ACKTIMEOUT:
				if (!ath_hal_setacktimeout(ah, val))
					ret = -EINVAL;
				break;
			case ATH_CTSTIMEOUT:
				if (!ath_hal_setctstimeout(ah, val))
					ret = -EINVAL;
				break;
			case ATH_SOFTLED:
				if (val != sc->sc_softled) {
					if (val)
						ath_hal_gpioCfgOutput(ah, 
								sc->sc_ledpin);
					ath_hal_gpioset(ah, sc->sc_ledpin, 
							!sc->sc_ledon);
					sc->sc_softled = val;
				}
				break;
			case ATH_LEDPIN:
				/* XXX validate? */
				sc->sc_ledpin = val;
				break;
			case ATH_DEBUG:
				sc->sc_debug 	 = (val & ~ATH_DEBUG_GLOBAL);
				ath_debug_global = (val &  ATH_DEBUG_GLOBAL);
				break;
			case ATH_TXANTENNA:
				/*
				 * antenna can be:
				 * 0 = transmit diversity
				 * 1 = antenna port 1
				 * 2 = antenna port 2
				 */
				if (val > 2)
					ret = -EINVAL;
				else
					sc->sc_txantenna = val;
				break;
			case ATH_RXANTENNA:
				/*
				 * antenna can be:
				 * 0 = receive diversity
				 * 1 = antenna port 1
				 * 2 = antenna port 2
				 */
				if (val > 2)
					ret = -EINVAL;
				else
					ath_setdefantenna(sc, val);
				break;
			case ATH_DIVERSITY:
				/*
				 * 0 = disallow use of diversity
				 * 1 = allow use of diversity
				 */
				if (val > 1) {
					ret = -EINVAL;
					break;
				}
				/* Don't enable diversity if XR is enabled */
				if (((!sc->sc_hasdiversity) || 
						 (sc->sc_xrtxq != NULL)) && 
						val) {
					ret = -EINVAL;
					break;
				}
				sc->sc_diversity = val;
				ath_hal_setdiversity(ah, val);
				break;
			case ATH_TXINTRPERIOD:
				/* XXX: validate? */
				sc->sc_txintrperiod = val;
				break;
			case ATH_FFTXQMIN:
				/* XXX validate? */
				sc->sc_fftxqmin = val;
				break;
#ifdef ATH_SUPERG_XR
			case ATH_XR_POLL_PERIOD:
				if (val > XR_MAX_POLL_INTERVAL)
					val = XR_MAX_POLL_INTERVAL;
				else if (val < XR_MIN_POLL_INTERVAL)
					val = XR_MIN_POLL_INTERVAL;
				sc->sc_xrpollint = val;
				break;

			case ATH_XR_POLL_COUNT:
				if (val > XR_MAX_POLL_COUNT)
					val = XR_MAX_POLL_COUNT;
				else if (val < XR_MIN_POLL_COUNT)
					val = XR_MIN_POLL_COUNT;
				sc->sc_xrpollcount = val;
				break;
#endif
			case ATH_ACKRATE:
				sc->sc_ackrate = val;
				ath_set_ack_bitrate(sc, sc->sc_ackrate);
				break;
			default:
				ret = -EINVAL;
				break;
			}
		}
	} else {
		switch ((long)ctl->extra2) {
		case ATH_SLOTTIME:
			val = ath_hal_getslottime(ah);
			break;
		case ATH_ACKTIMEOUT:
			val = ath_hal_getacktimeout(ah);
			break;
		case ATH_CTSTIMEOUT:
			val = ath_hal_getctstimeout(ah);
			break;
		case ATH_SOFTLED:
			val = sc->sc_softled;
			break;
		case ATH_LEDPIN:
			val = sc->sc_ledpin;
			break;
		case ATH_COUNTRYCODE:
			ath_hal_getcountrycode(ah, &val);
			break;
		case ATH_REGDOMAIN:
			ath_hal_getregdomain(ah, &val);
			break;
		case ATH_DEBUG:
			val = sc->sc_debug | ath_debug_global;
			break;
		case ATH_TXANTENNA:
			val = sc->sc_txantenna;
			break;
		case ATH_RXANTENNA:
			val = ath_hal_getdefantenna(ah);
			break;
		case ATH_DIVERSITY:
			val = sc->sc_diversity;
			break;
		case ATH_TXINTRPERIOD:
			val = sc->sc_txintrperiod;
			break;
		case ATH_FFTXQMIN:
			val = sc->sc_fftxqmin;
			break;
#ifdef ATH_SUPERG_XR
		case ATH_XR_POLL_PERIOD:
			val=sc->sc_xrpollint;
			break;
		case ATH_XR_POLL_COUNT:
			val=sc->sc_xrpollcount;
			break;
#endif
		case ATH_ACKRATE:
			val = sc->sc_ackrate;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		if (!ret) {
			ret = ATH_SYSCTL_PROC_DOINTVEC(ctl, write, filp, 
					buffer, lenp, ppos);
		}
	}
	ATH_UNLOCK(sc);
	return ret;
}

static int mincalibrate = 1;		/* once a second */
static int maxint = 0x7fffffff;		/* 32-bit big */

static const ctl_table ath_sysctl_template[] = {
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "slottime",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam,
	  .extra2	= (void *)ATH_SLOTTIME,
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "acktimeout",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam,
	  .extra2	= (void *)ATH_ACKTIMEOUT,
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "ctstimeout",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam,
	  .extra2	= (void *)ATH_CTSTIMEOUT,
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "softled",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam,
	  .extra2	= (void *)ATH_SOFTLED,
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "ledpin",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam,
	  .extra2	= (void *)ATH_LEDPIN,
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "countrycode",
	  .mode		= 0444,
	  .proc_handler	= ath_sysctl_halparam,
	  .extra2	= (void *)ATH_COUNTRYCODE,
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "regdomain",
	  .mode		= 0444,
	  .proc_handler	= ath_sysctl_halparam,
	  .extra2	= (void *)ATH_REGDOMAIN,
	},
#ifdef AR_DEBUG
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "debug",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam,
	  .extra2	= (void *)ATH_DEBUG,
	},
#endif
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "txantenna",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam,
	  .extra2	= (void *)ATH_TXANTENNA,
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "rxantenna",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam,
	  .extra2	= (void *)ATH_RXANTENNA,
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "diversity",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam,
	  .extra2	= (void *)ATH_DIVERSITY,
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "txintrperiod",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam,
	  .extra2	= (void *)ATH_TXINTRPERIOD,
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "fftxqmin",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam,
	  .extra2	= (void *)ATH_FFTXQMIN,
	},
#ifdef ATH_SUPERG_XR
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "xrpollperiod",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam,
	  .extra2	= (void *)ATH_XR_POLL_PERIOD,
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "xrpollcount",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam,
	  .extra2	= (void *)ATH_XR_POLL_COUNT,
	},
#endif
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "ackrate",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam,
	  .extra2	= (void *)ATH_ACKRATE,
	},
	{ 0 }
};

static void
ath_dynamic_sysctl_register(struct ath_softc *sc)
{
	unsigned int i, space;
	char *dev_name = NULL;

	space = 5 * sizeof(struct ctl_table) + sizeof(ath_sysctl_template);
	sc->sc_sysctls = kmalloc(space, GFP_KERNEL);
	if (sc->sc_sysctls == NULL) {
		printk("%s: no memory for sysctl table!\n", __func__);
		return;
	}

	/*
	 * We want to reserve space for the name of the device separate
	 * from the net_device structure, because when the name is changed
	 * it is changed in the net_device structure and the message given
	 * out.  Thus we won't know what the name it used to be if we rely
	 * on it.
	 */
	dev_name = kmalloc((strlen(DEV_NAME(sc->sc_dev)) + 1) * sizeof(char), GFP_KERNEL);
	if (dev_name == NULL) {
		printk("%s: no memory for device name storage!\n", __func__);
		return;
	}
	strncpy(dev_name, DEV_NAME(sc->sc_dev), strlen(DEV_NAME(sc->sc_dev)) + 1);

	/* setup the table */
	memset(sc->sc_sysctls, 0, space);
	sc->sc_sysctls[0].ctl_name = CTL_DEV;
	sc->sc_sysctls[0].procname = "dev";
	sc->sc_sysctls[0].mode = 0555;
	sc->sc_sysctls[0].child = &sc->sc_sysctls[2];
	/* [1] is NULL terminator */
	sc->sc_sysctls[2].ctl_name = CTL_AUTO;
	sc->sc_sysctls[2].procname = dev_name;
	sc->sc_sysctls[2].mode = 0555;
	sc->sc_sysctls[2].child = &sc->sc_sysctls[4];
	/* [3] is NULL terminator */
	/* copy in pre-defined data */
	memcpy(&sc->sc_sysctls[4], ath_sysctl_template,
		sizeof(ath_sysctl_template));

	/* add in dynamic data references */
	for (i = 4; sc->sc_sysctls[i].procname; i++)
		if (sc->sc_sysctls[i].extra1 == NULL)
			sc->sc_sysctls[i].extra1 = sc;

	/* and register everything */
	sc->sc_sysctl_header = ATH_REGISTER_SYSCTL_TABLE(sc->sc_sysctls);
	if (!sc->sc_sysctl_header) {
		printk("%s: failed to register sysctls!\n", DEV_NAME(sc->sc_dev));
		kfree(dev_name);
		kfree(sc->sc_sysctls);
		sc->sc_sysctls = NULL;
	}

	/* initialize values */
	ath_debug_global = (ath_debug & ATH_DEBUG_GLOBAL);
	sc->sc_debug 	 = (ath_debug & ~ATH_DEBUG_GLOBAL);
	sc->sc_default_ieee80211_debug = ieee80211_debug;
	sc->sc_txantenna = 0;		/* default to auto-selection */
	sc->sc_txintrperiod = ATH_TXQ_INTR_PERIOD;
}

static void
ath_dynamic_sysctl_unregister(struct ath_softc *sc)
{
	if (sc->sc_sysctl_header) {
		unregister_sysctl_table(sc->sc_sysctl_header);
		sc->sc_sysctl_header = NULL;
	}
	if (sc->sc_sysctls && sc->sc_sysctls[2].procname) {
		kfree(sc->sc_sysctls[2].procname);
		sc->sc_sysctls[2].procname = NULL;
	}
	if (sc->sc_sysctls) {
		kfree(sc->sc_sysctls);
		sc->sc_sysctls = NULL;
	}
}

/*
 * Announce various information on device/driver attach.
 */
static void
ath_announce(struct net_device *dev)
{
#define	HAL_MODE_DUALBAND	(HAL_MODE_11A|HAL_MODE_11B)
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	u_int modes, cc;

	printk("%s: mac %d.%d phy %d.%d", DEV_NAME(dev),
		ah->ah_macVersion, ah->ah_macRev,
		ah->ah_phyRev >> 4, ah->ah_phyRev & 0xf);
	/*
	 * Print radio revision(s).  We check the wireless modes
	 * to avoid falsely printing revs for inoperable parts.
	 * Dual-band radio revs are returned in the 5 GHz rev number.
	 */
	ath_hal_getcountrycode(ah, &cc);
	modes = ath_hal_getwirelessmodes(ah, cc);
	if ((modes & HAL_MODE_DUALBAND) == HAL_MODE_DUALBAND) {
		if (ah->ah_analog5GhzRev && ah->ah_analog2GhzRev)
			printk(" 5 GHz radio %d.%d 2 GHz radio %d.%d",
				ah->ah_analog5GhzRev >> 4,
				ah->ah_analog5GhzRev & 0xf,
				ah->ah_analog2GhzRev >> 4,
				ah->ah_analog2GhzRev & 0xf);
		else
			printk(" radio %d.%d", ah->ah_analog5GhzRev >> 4,
				ah->ah_analog5GhzRev & 0xf);
	} else
		printk(" radio %d.%d", ah->ah_analog5GhzRev >> 4,
			ah->ah_analog5GhzRev & 0xf);
	printk("\n");
	if (1/*bootverbose*/) {
		unsigned int i;
		for (i = 0; i <= WME_AC_VO; i++) {
			struct ath_txq *txq = sc->sc_ac2q[i];
			printk("%s: Use hw queue %u for %s traffic\n",
				DEV_NAME(dev), txq->axq_qnum,
				ieee80211_wme_acnames[i]);
		}
		printk("%s: Use hw queue %u for CAB traffic\n", DEV_NAME(dev),
			sc->sc_cabq->axq_qnum);
		printk("%s: Use hw queue %u for beacons\n", DEV_NAME(dev),
			sc->sc_bhalq);
	}
#undef HAL_MODE_DUALBAND
}

/*
 * Static (i.e. global) sysctls.  Note that the HAL sysctls
 * are located under ours by sharing the setting for DEV_ATH.
 */
static ctl_table ath_static_sysctls[] = {
#ifdef AR_DEBUG
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "debug",
	  .mode		= 0644,
	  .data		= &ath_debug,
	  .maxlen	= sizeof(ath_debug),
	  .proc_handler	= proc_dointvec
	},
#endif
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "countrycode",
	  .mode		= 0444,
	  .data		= &ath_countrycode,
	  .maxlen	= sizeof(ath_countrycode),
	  .proc_handler	= proc_dointvec
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "outdoor",
	  .mode		= 0444,
	  .data		= &ath_outdoor,
	  .maxlen	= sizeof(ath_outdoor),
	  .proc_handler	= proc_dointvec
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "xchanmode",
	  .mode		= 0444,
	  .data		= &ath_xchanmode,
	  .maxlen	= sizeof(ath_xchanmode),
	  .proc_handler	= proc_dointvec
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "calibrate",
	  .mode		= 0644,
	  .data		= &ath_calinterval,
	  .maxlen	= sizeof(ath_calinterval),
	  .extra1	= &mincalibrate,
	  .extra2	= &maxint,
	  .proc_handler	= proc_dointvec_minmax
	},
	{ 0 }
};
static ctl_table ath_ath_table[] = {
	{ .ctl_name	= DEV_ATH,
	  .procname	= "ath",
	  .mode		= 0555,
	  .child	= ath_static_sysctls
	}, { 0 }
};
static ctl_table ath_root_table[] = {
	{ .ctl_name	= CTL_DEV,
	  .procname	= "dev",
	  .mode		= 0555,
	  .child	= ath_ath_table
	}, { 0 }
};
static struct ctl_table_header *ath_sysctl_header;

void
ath_sysctl_register(void)
{
	static int initialized = 0;

	if (!initialized) {
		register_netdevice_notifier(&ath_event_block);
		ath_sysctl_header = ATH_REGISTER_SYSCTL_TABLE(ath_root_table);
		initialized = 1;
	}
}

void
ath_sysctl_unregister(void)
{
	unregister_netdevice_notifier(&ath_event_block);
	if (ath_sysctl_header)
		unregister_sysctl_table(ath_sysctl_header);
}

static const char*
ath_get_hal_status_desc(HAL_STATUS status)
{
	if ((status > 0) && (status < (sizeof(hal_status_desc) / 
					sizeof(char *))))
		return hal_status_desc[status];
	else
		return "";
}

/* Adjust the ratecode used for continuous transmission to the closest rate 
 * to the one specified (rounding down) */
static int
ath_get_txcont_adj_ratecode(struct ath_softc *sc)
{
	const HAL_RATE_TABLE *rt    = sc->sc_currates;
	int closest_rate_ix         = sc->sc_minrateix;
	int j;

	if (0 != sc->sc_txcont_rate) {
		/* Find closest rate to specified rate */
		for (j = sc->sc_minrateix; j < rt->rateCount; j++) {
			if (((sc->sc_txcont_rate * 1000) >= 
					 rt->info[j].rateKbps) &&
					(rt->info[j].rateKbps >= 
					 rt->info[closest_rate_ix].rateKbps)) {
				closest_rate_ix = j;
			}
		}
	}
	/* Diagnostic */
	if (0 == sc->sc_txcont_rate) {
		printk(KERN_INFO "%s: %s: Using default rate of %dM.\n", DEV_NAME(sc->sc_dev),
				__func__, (rt->info[closest_rate_ix].rateKbps / 1000));
	} else if (sc->sc_txcont_rate == (rt->info[closest_rate_ix].rateKbps / 1000)) {
		printk(KERN_INFO "%s: %s: Using requested rate of %dM.\n",
				DEV_NAME(sc->sc_dev), __func__, sc->sc_txcont_rate);
	} else {
		printk(KERN_INFO "%s: %s: Rounded down requested rate of %dM to %dM.\n",
				DEV_NAME(sc->sc_dev), __func__, sc->sc_txcont_rate,
				(rt->info[closest_rate_ix].rateKbps / 1000));
	}
	return rt->info[closest_rate_ix].rateCode;
}

/*
Configure the radio for continuous transmission
*/
static void
txcont_configure_radio(struct ieee80211com *ic)
{
	struct net_device           *dev = ic->ic_dev;
	struct ath_softc            *sc = dev->priv;
	struct ath_hal              *ah = sc->sc_ah;
	struct ieee80211_wme_state  *wme = &ic->ic_wme;
	struct ieee80211vap         *vap = TAILQ_FIRST(&ic->ic_vaps);

	HAL_STATUS status;
	int q;

	if (IFF_RUNNING != (ic->ic_dev->flags & IFF_RUNNING)) {
		printk(KERN_ERR "%s: %s: Cannot enable txcont when interface is"
			" not in running state.\n", DEV_NAME(dev), __func__);
		sc->sc_txcont = 0;
		return;
	}

	ath_hal_intrset(ah, 0);
	
	{
		int ac;

		/* Prepare to reconfigure */
		ic->ic_caps  &= ~IEEE80211_C_SHPREAMBLE;
		ic->ic_coverageclass = 0;
		ic->ic_flags &= ~IEEE80211_F_DOTH;
		ic->ic_flags &= ~IEEE80211_F_SHPREAMBLE;
		ic->ic_flags &= ~IEEE80211_F_TXPOW_FIXED;
		ic->ic_flags |= IEEE80211_F_USEBARKER;
		ic->ic_flags_ext &= ~IEEE80211_FEXT_COUNTRYIE;
		ic->ic_flags_ext &= ~IEEE80211_FEXT_MARKDFS;
		ic->ic_flags_ext &= ~IEEE80211_FEXT_REGCLASS;
		ic->ic_protmode = IEEE80211_PROT_NONE;
		ic->ic_roaming = IEEE80211_ROAMING_DEVICE;
		vap->iv_flags &= ~IEEE80211_F_BGSCAN;
		vap->iv_flags &= ~IEEE80211_F_DROPUNENC;
		vap->iv_flags &= ~IEEE80211_F_NOBRIDGE;
		vap->iv_flags &= ~IEEE80211_F_PRIVACY;
		vap->iv_flags |= IEEE80211_F_PUREG;
		vap->iv_flags |= IEEE80211_F_WME;
		vap->iv_flags_ext &= ~IEEE80211_FEXT_UAPSD;
		vap->iv_flags_ext &= ~IEEE80211_FEXT_WDS;
		vap->iv_ic->ic_flags |= IEEE80211_F_WME; /* XXX needed by ic_reset */
		vap->iv_mcast_rate = 54000;
		vap->iv_uapsdinfo = 0;
		vap->iv_ath_cap |= IEEE80211_ATHC_BURST;
		vap->iv_ath_cap |= IEEE80211_ATHC_FF;
		vap->iv_ath_cap &= ~IEEE80211_ATHC_AR;
		vap->iv_ath_cap &= ~IEEE80211_ATHC_COMP;
		vap->iv_des_ssid[0].len = 0;
		vap->iv_des_nssid = 1;
		sc->sc_txantenna = sc->sc_defant = sc->sc_mcastantenna = sc->sc_rxotherant = 1;
		sc->sc_diversity = 0;
		memset(vap->iv_des_ssid[0].ssid, 0, IEEE80211_ADDR_LEN);
		ath_hal_setdiversity(sc->sc_ah, 0);

		for (ac = 0; ac < WME_NUM_AC; ac++) {
			/* AIFSN = 1 */
			wme->wme_wmeBssChanParams.cap_wmeParams[ac].wmep_aifsn   =
			    wme->wme_bssChanParams.cap_wmeParams[ac].wmep_aifsn  =
			    wme->wme_wmeChanParams.cap_wmeParams[ac].wmep_aifsn  =
			    wme->wme_chanParams.cap_wmeParams[ac].wmep_aifsn     =
			    1;

			/*  CWMIN = 1 */
			wme->wme_wmeBssChanParams.cap_wmeParams[ac].wmep_logcwmin  =
			    wme->wme_bssChanParams.cap_wmeParams[ac].wmep_logcwmin =
			    wme->wme_wmeChanParams.cap_wmeParams[ac].wmep_logcwmin =
			    wme->wme_chanParams.cap_wmeParams[ac].wmep_logcwmin    =
			    1;

			/*  CWMAX = 1 */
			wme->wme_wmeBssChanParams.cap_wmeParams[ac].wmep_logcwmax  =
			    wme->wme_bssChanParams.cap_wmeParams[ac].wmep_logcwmax =
			    wme->wme_wmeChanParams.cap_wmeParams[ac].wmep_logcwmax =
			    wme->wme_chanParams.cap_wmeParams[ac].wmep_logcwmax    =
			    1;

			/*  ACM = 1 */
			wme->wme_wmeBssChanParams.cap_wmeParams[ac].wmep_acm       =
			    wme->wme_bssChanParams.cap_wmeParams[ac].wmep_acm      =
			    0;

			/*  NOACK = 1 */
			wme->wme_wmeChanParams.cap_wmeParams[ac].wmep_noackPolicy  =
			    wme->wme_chanParams.cap_wmeParams[ac].wmep_noackPolicy =
			    1;

			/*  TXOPLIMIT = 8192 */
			wme->wme_wmeBssChanParams.cap_wmeParams[ac].wmep_txopLimit  =
			    wme->wme_bssChanParams.cap_wmeParams[ac].wmep_txopLimit =
			    wme->wme_wmeChanParams.cap_wmeParams[ac].wmep_txopLimit =
			    wme->wme_chanParams.cap_wmeParams[ac].wmep_txopLimit    =
			    IEEE80211_US_TO_TXOP(8192);
		}
		ieee80211_cancel_scan(vap);	/* anything current */
		ieee80211_wme_updateparams(vap);
		/*  reset the WNIC */
		if (!ath_hal_reset(ah, sc->sc_opmode, 
					&sc->sc_curchan, AH_TRUE, &status)) {
			printk(KERN_ERR "%s: ath_hal_reset failed: '%s' "
					"(HAL status %u) in %s at %s:%d\n",
					DEV_NAME(dev), 
					ath_get_hal_status_desc(status),
					status,  __func__, __FILE__, __LINE__);
		}

		ath_update_txpow(sc);

#ifdef ATH_SUPERG_DYNTURBO
		/*  Turn on dynamic turbo if necessary -- before we get into 
		 *  our own implementation -- and before we configures */
		if (!IEEE80211_IS_CHAN_STURBO(ic->ic_bsschan) &&
				(IEEE80211_ATHC_TURBOP &
					TAILQ_FIRST(&ic->ic_vaps)->iv_ath_cap) &&
				(IEEE80211_IS_CHAN_ANYG(ic->ic_bsschan) ||
				 IEEE80211_IS_CHAN_A(ic->ic_bsschan))) {
			u_int32_t newflags = ic->ic_bsschan->ic_flags;
			if (IEEE80211_ATHC_TURBOP & 
						TAILQ_FIRST(&ic->ic_vaps)->iv_ath_cap) {
				DPRINTF(sc, ATH_DEBUG_TURBO, 
					"%s: Enabling dynamic turbo...\n",
					__func__);
				ic->ic_ath_cap |= IEEE80211_ATHC_BOOST;
				sc->sc_ignore_ar = 1;
				newflags |= IEEE80211_CHAN_TURBO;
			} else {
				DPRINTF(sc, ATH_DEBUG_TURBO, 
					"%s: Disabling dynamic turbo...\n",
					__func__);
				ic->ic_ath_cap &= ~IEEE80211_ATHC_BOOST;
				newflags &= ~IEEE80211_CHAN_TURBO;
			}
			ieee80211_dturbo_switch(ic, newflags);
			/*  Keep interupts off, just in case... */
			ath_hal_intrset(ah, 0);
		}
#endif /* #ifdef ATH_SUPERG_DYNTURBO */
		/* clear pending tx frames picked up after reset */
		ath_draintxq(sc);
		/* stop receive side */
		ath_stoprecv(sc);
		ath_hal_setrxfilter(ah, 0);
		ath_hal_setmcastfilter(ah, 0, 0);
		ath_set_ack_bitrate(sc, sc->sc_ackrate);
		netif_wake_queue(dev);		/* restart xmit */

		if (ar_device(sc->devid) == 5212 || ar_device(sc->devid) == 5213) {
			/* registers taken from openhal */
#define AR5K_AR5212_TXCFG				0x0030
#define AR5K_AR5212_TXCFG_TXCONT_ENABLE			0x00000080
#define AR5K_AR5212_RSSI_THR				0x8018
#define AR5K_AR5212_PHY_NF				0x9864
#define AR5K_AR5212_ADDAC_TEST				0x8054
#define AR5K_AR5212_DIAG_SW				0x8048
#define AR5K_AR5212_DIAG_SW_IGNOREPHYCS			0x00100000
#define AR5K_AR5212_DIAG_SW_IGNORENAV			0x00200000
#define AR5K_AR5212_DCU_GBL_IFS_SIFS			0x1030
#define AR5K_AR5212_DCU_GBL_IFS_SIFS_M			0x0000ffff
#define AR5K_AR5212_DCU_GBL_IFS_EIFS			0x10b0
#define AR5K_AR5212_DCU_GBL_IFS_EIFS_M			0x0000ffff
#define AR5K_AR5212_DCU_GBL_IFS_SLOT			0x1070
#define AR5K_AR5212_DCU_GBL_IFS_SLOT_M			0x0000ffff
#define AR5K_AR5212_DCU_GBL_IFS_MISC			0x10f0
#define	AR5K_AR5212_DCU_GBL_IFS_MISC_LFSR_SLICE		0x00000007
#define	AR5K_AR5212_DCU_GBL_IFS_MISC_TURBO_MODE		0x00000008
#define	AR5K_AR5212_DCU_GBL_IFS_MISC_SIFS_DUR_USEC	0x000003f0
#define	AR5K_AR5212_DCU_GBL_IFS_MISC_USEC_DUR		0x000ffc00
#define	AR5K_AR5212_DCU_GBL_IFS_MISC_DCU_ARB_DELAY	0x00300000
#define	AR5K_AR5212_DCU_MISC_POST_FR_BKOFF_DIS		0x00200000
#define	AR5K_AR5212_DCU_CHAN_TIME_DUR			0x000fffff
#define	AR5K_AR5212_DCU_CHAN_TIME_ENABLE		0x00100000
#define	AR5K_AR5212_QCU(_n, _a)		                (((_n) << 2) + _a)
#define	AR5K_AR5212_DCU(_n, _a)		                AR5K_AR5212_QCU(_n, _a)
#define AR5K_AR5212_DCU_MISC(_n)			AR5K_AR5212_DCU(_n, 0x1100)
#define AR5K_AR5212_DCU_CHAN_TIME(_n)			AR5K_AR5212_DCU(_n, 0x10c0)
			/* NB: This section of direct hardware access contains
			 * a continuous transmit mode derived by reverse
			 * engineering. Many of the settings may be unnecessary
			 * to achieve the end result. Additional testing,
			 * selectively commenting out register writes below may
			 * result in simpler code with the same results. */

			/*  Set RSSI threshold to extreme, hear nothing */
			OS_REG_WRITE(ah, AR5K_AR5212_RSSI_THR, 0xffffffff);
			/*  Blast away at noise floor, assuming AGC has
			 *  already set it... we want to trash it. */
			OS_REG_WRITE(ah, AR5K_AR5212_PHY_NF,   0xffffffff);
			/* Enable continuous transmit mode / DAC test mode */
			OS_REG_WRITE(ah, AR5K_AR5212_ADDAC_TEST,
					OS_REG_READ(ah, AR5K_AR5212_ADDAC_TEST) | 1);
			/* Ignore real and virtual carrier sensing, and reception */
			OS_REG_WRITE(ah, AR5K_AR5212_DIAG_SW,
					OS_REG_READ(ah, AR5K_AR5212_DIAG_SW) |
					AR5K_AR5212_DIAG_SW_IGNOREPHYCS |
					AR5K_AR5212_DIAG_SW_IGNORENAV);
			/*  Set SIFS to rediculously small value...  */
			OS_REG_WRITE(ah, AR5K_AR5212_DCU_GBL_IFS_SIFS,
					(OS_REG_READ(ah, 
						     AR5K_AR5212_DCU_GBL_IFS_SIFS) &
					 ~AR5K_AR5212_DCU_GBL_IFS_SIFS_M) | 1);
			/*  Set EIFS to rediculously small value...  */
			OS_REG_WRITE(ah, AR5K_AR5212_DCU_GBL_IFS_EIFS,
					(OS_REG_READ(ah, 
						     AR5K_AR5212_DCU_GBL_IFS_EIFS) &
					 ~AR5K_AR5212_DCU_GBL_IFS_EIFS_M) | 1);
			/*  Set slot time to rediculously small value...  */
			OS_REG_WRITE(ah, AR5K_AR5212_DCU_GBL_IFS_SLOT,
					(OS_REG_READ(ah, 
						     AR5K_AR5212_DCU_GBL_IFS_SLOT) &
					 ~AR5K_AR5212_DCU_GBL_IFS_SLOT_M) | 1);
			OS_REG_WRITE(ah, AR5K_AR5212_DCU_GBL_IFS_MISC,
			    OS_REG_READ(ah, AR5K_AR5212_DCU_GBL_IFS_MISC) &
			    ~AR5K_AR5212_DCU_GBL_IFS_MISC_SIFS_DUR_USEC &
			    ~AR5K_AR5212_DCU_GBL_IFS_MISC_USEC_DUR &
			    ~AR5K_AR5212_DCU_GBL_IFS_MISC_DCU_ARB_DELAY &
			    ~AR5K_AR5212_DCU_GBL_IFS_MISC_LFSR_SLICE);

			/*  Disable queue backoff (default was like 256 or 0x100) */
			for (q = 0; q < 4; q++) {
				OS_REG_WRITE(ah, AR5K_AR5212_DCU_MISC(q), 
						AR5K_AR5212_DCU_MISC_POST_FR_BKOFF_DIS);
				/*  Set the channel time (burst time) to the 
				 *  highest setting the register can take, 
				 *  forget this compliant 8192 limit... */
				OS_REG_WRITE(ah, AR5K_AR5212_DCU_CHAN_TIME(q), 
						AR5K_AR5212_DCU_CHAN_TIME_ENABLE | 
						AR5K_AR5212_DCU_CHAN_TIME_DUR);
			}
			/*  Set queue full to continuous */
			OS_REG_WRITE(ah, AR5K_AR5212_TXCFG, OS_REG_READ(ah, 
						AR5K_AR5212_TXCFG) | 
					AR5K_AR5212_TXCFG_TXCONT_ENABLE);
#undef AR5K_AR5212_TXCFG
#undef AR5K_AR5212_TXCFG_TXCONT_ENABLE
#undef AR5K_AR5212_RSSI_THR
#undef AR5K_AR5212_PHY_NF
#undef AR5K_AR5212_ADDAC_TEST
#undef AR5K_AR5212_DIAG_SW
#undef AR5K_AR5212_DIAG_SW_IGNOREPHYCS
#undef AR5K_AR5212_DIAG_SW_IGNORENAV
#undef AR5K_AR5212_DCU_GBL_IFS_SIFS
#undef AR5K_AR5212_DCU_GBL_IFS_SIFS_M
#undef AR5K_AR5212_DCU_GBL_IFS_EIFS
#undef AR5K_AR5212_DCU_GBL_IFS_EIFS_M
#undef AR5K_AR5212_DCU_GBL_IFS_SLOT
#undef AR5K_AR5212_DCU_GBL_IFS_SLOT_M
#undef AR5K_AR5212_DCU_GBL_IFS_MISC
#undef AR5K_AR5212_DCU_GBL_IFS_MISC_LFSR_SLICE
#undef AR5K_AR5212_DCU_GBL_IFS_MISC_TURBO_MODE
#undef AR5K_AR5212_DCU_GBL_IFS_MISC_SIFS_DUR_USEC
#undef AR5K_AR5212_DCU_GBL_IFS_MISC_USEC_DUR
#undef AR5K_AR5212_DCU_GBL_IFS_MISC_DCU_ARB_DELAY
#undef AR5K_AR5212_DCU_MISC_POST_FR_BKOFF_DIS
#undef AR5K_AR5212_DCU_CHAN_TIME_DUR
#undef AR5K_AR5212_DCU_CHAN_TIME_ENABLE
#undef AR5K_AR5212_QCU
#undef AR5K_AR5212_DCU
#undef AR5K_AR5212_DCU_MISC
#undef AR5K_AR5212_DCU_CHAN_TIME
		}

		/* Disable beacons and beacon miss interrupts */
		sc->sc_beacons = 0;
		sc->sc_imask &= ~(HAL_INT_SWBA | HAL_INT_BMISS);

		/* Enable continuous transmit register bit */
		sc->sc_txcont = 1;
	}
	ath_hal_intrset(ah, sc->sc_imask);
}

/* Queue a self-looped packet for the specified hardware queue. */
static void
txcont_queue_packet(struct ieee80211com *ic, struct ath_txq* txq)
{
	struct net_device *dev             = ic->ic_dev;
	struct ath_softc *sc               = dev->priv;
	struct ath_hal *ah                 = sc->sc_ah;
	struct ath_buf *bf                 = NULL;
	struct sk_buff *skb                = NULL;
	unsigned int i;
	/* maximum supported size, subtracting headers and required slack */
	unsigned int datasz                = 4028;
	struct ieee80211_frame* wh         = NULL;
	unsigned char *data                = NULL;
	unsigned char *crc                 = NULL;

	if (IFF_RUNNING != (ic->ic_dev->flags & IFF_RUNNING) || 
			(0 == sc->sc_txcont)) {
		printk(KERN_ERR "%s: %s: Refusing to queue self linked frame "
				"when txcont is not enabled.\n", 
				DEV_NAME(dev), __func__);
		return;
	}

	ath_hal_intrset(ah, 0);
	{
		bf  = ath_take_txbuf(sc);
		skb = alloc_skb(datasz + sizeof(struct ieee80211_frame) + 
				IEEE80211_CRC_LEN, GFP_ATOMIC);
		if (NULL == skb) {
			printk(KERN_ERR "%s: %s: alloc_skb returned null!\n", 
					DEV_NAME(dev), __func__);
			BUG();
		}
		wh  = (struct ieee80211_frame*)skb_put(skb, 
				sizeof(struct ieee80211_frame));
		if (NULL == bf) {
			printk(KERN_ERR "%s: %s: STAILQ_FIRST(&sc->sc_txbuf) "
					"returned null!\n", 
					DEV_NAME(dev), __func__);
			BUG();
		}

		/*  Define the SKB format */
		data   = skb_put(skb, datasz);

		/*  NB: little endian */

		/*  11110000 (protocol = 00, type = 00 "management",
		 *  subtype = 1111 "reserved/undocumented" */
		wh->i_fc[0]    = 0xf0;
		/* leave out to/from DS, frag., retry, pwr mgt, more data,
		 * protected frame, and order bit */
		wh->i_fc[1]    = 0x00;
		/* NB: Duration is left at zero, for broadcast frames. */
		wh->i_dur      = 0;
		/*  DA (destination address) */
		wh->i_addr1[0] = wh->i_addr1[1] = wh->i_addr1[2] =
			wh->i_addr1[3] = wh->i_addr1[4] = wh->i_addr1[5] = 0xff;
		/*  BSSID */
		wh->i_addr2[0] = wh->i_addr2[1] = wh->i_addr2[2] =
			wh->i_addr2[3] = wh->i_addr2[4] = wh->i_addr2[5] = 0xff;
		/*  SA (source address) */
		wh->i_addr3[0] = wh->i_addr3[1] = wh->i_addr3[2] =
			wh->i_addr3[3] = wh->i_addr3[4] = wh->i_addr3[5] = 0x00;
		/*  Sequence is zero for now, let the hardware assign this or
		 *  not, depending on how we setup flags (below) */
		wh->i_seq[0]   = 0x00;
		wh->i_seq[1]   = 0x00;

		/*  Initialize the data */
		if (datasz % 4)	BUG();
		for (i = 0; i < datasz; i+=4) {
			data[i+0] = 0x00;
			data[i+1] = 0xff;
			data[i+2] = 0x00;
			data[i+3] = 0xff;
		}

		/*  Add space for the CRC */
		crc    = skb_put(skb, IEEE80211_CRC_LEN);

		/*  Clear  */
		crc[0] = crc[1] = crc[2] = crc[3] = 0x00;

		/*  Initialize the selfed-linked frame */
		bf->bf_skb      = skb;
		bf->bf_skbaddr  = bus_map_single(sc->sc_bdev, bf->bf_skb->data,
				bf->bf_skb->len, BUS_DMA_TODEVICE);
		bf->bf_flags    = HAL_TXDESC_CLRDMASK | HAL_TXDESC_NOACK;
		bf->bf_node     = NULL;
		bf->bf_desc->ds_link = bf->bf_daddr;
		bf->bf_desc->ds_data = bf->bf_skbaddr;

		ath_hal_setuptxdesc(ah,
		    bf->bf_desc,			/* the descriptor */
		    skb->len,				/* packet length */
		    sizeof(struct ieee80211_frame),	/* header length */
		    HAL_PKT_TYPE_NORMAL,	   	/* Atheros packet type */
		    sc->sc_txcont_power,	   	/* txpower in 0.5dBm 
							 * increments, range 0-n 
							 * depending upon card 
							 * typically 60-100 max */
		    ath_get_txcont_adj_ratecode(sc),	/* series 0 rate */
		    0,					/* series 0 retries */
		    HAL_TXKEYIX_INVALID,		/* key cache index */
		    sc->sc_txantenna,			/* antenna mode */
		    bf->bf_flags,			/* flags */
		    0,					/* rts/cts rate */
		    0,					/* rts/cts duration */
		    0,					/* comp icv len */
		    0,					/* comp iv len */
		    ATH_COMP_PROC_NO_COMP_NO_CCS	/* comp scheme */
		    );

		ath_hal_filltxdesc(ah,
		    bf->bf_desc,	/* Descriptor to fill */
		    skb->len,		/* buffer length */
		    AH_TRUE,		/* is first segment */
		    AH_TRUE,		/* is last segment */
		    bf->bf_desc		/* first descriptor */
		    );

		/*  Byteswap (as necessary) */
		ath_desc_swap(bf->bf_desc);
		/*  queue the self-linked frame */
		ath_tx_txqaddbuf(sc, NULL,	/* node */
		    txq,			/* hardware queue */
		    bf,				/* atheros buffer */
		    bf->bf_desc,		/* last descriptor */
		    bf->bf_skb->len		/* frame length */
		    );
		ath_hal_txstart(ah, txq->axq_qnum);
	}
	ath_hal_intrset(ah, sc->sc_imask);
}

/* Turn on continuous transmission */
static void
txcont_on(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;

	if (IFF_RUNNING != (ic->ic_dev->flags & IFF_RUNNING)) {
		printk(KERN_ERR "%s: %s: Cannot enable txcont when"
				" interface is not in running state.\n", 
				DEV_NAME(dev), __func__);
		sc->sc_txcont = 0;
		return;
	}

	txcont_configure_radio(ic);
	txcont_queue_packet(ic, sc->sc_ac2q[WME_AC_BE]);
	txcont_queue_packet(ic, sc->sc_ac2q[WME_AC_BK]);
	txcont_queue_packet(ic, sc->sc_ac2q[WME_AC_VI]);
	txcont_queue_packet(ic, sc->sc_ac2q[WME_AC_VO]);
}

/* Turn off continuous transmission */
static void
txcont_off(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;

	if (TAILQ_FIRST(&ic->ic_vaps)->iv_opmode != IEEE80211_M_WDS)
		sc->sc_beacons = 1;
	ath_reset(sc->sc_dev);

	sc->sc_txcont = 0;
}

/* Is continuous transmission mode enabled?  It may not actually be
 * transmitting if the interface is down, for example. */
static int
ath_get_txcont(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	return sc->sc_txcont;
}

/* Set transmission mode on/off... but know that it may not actually start if
 * the interface is down, for example. */
static void
ath_set_txcont(struct ieee80211com *ic, int on)
{
	on ? txcont_on(ic) : txcont_off(ic);
}

/* Set the transmission power to be used during continuous transmission in
 * units of 0.5dBm ranging from 0 to 127. */
static void
ath_set_txcont_power(struct ieee80211com *ic, unsigned int txpower)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	int new_txcont_power = txpower > IEEE80211_TXPOWER_MAX ? 
		IEEE80211_TXPOWER_MAX : txpower;
	if (sc->sc_txcont_power != new_txcont_power) {
		/*  update */
		sc->sc_txcont_power = new_txcont_power;
		/*  restart continuous transmit if necessary */
		if (sc->sc_txcont) {
			txcont_on(ic);
		}
	}
}

/* See ath_set_txcont_power for details. */
static int
ath_get_txcont_power(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	/* VERY conservative default */
	return sc->sc_txcont_power ? sc->sc_txcont_power : 0;
}

/* Set the transmission rate to be used for continuous transmissions(in Mbps) */
	static void
ath_set_txcont_rate(struct ieee80211com *ic, unsigned int new_rate)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	if (sc->sc_txcont_rate != new_rate) {
		/*  NOTE: This value is sanity checked and dropped down to 
		 *  closest rate in txcont_on. */
		sc->sc_txcont_rate = new_rate;
		/*  restart continuous transmit if necessary */
		if (sc->sc_txcont) {
			txcont_on(ic);
		}
	}
}

/* See ath_set_txcont_rate for details. */
	static unsigned int
ath_get_txcont_rate(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	return sc->sc_txcont_rate ? sc->sc_txcont_rate : 0;
}

static int
ath_rcv_dev_event(struct notifier_block *this, unsigned long event,
	void *ptr)
{
	struct net_device *dev = (struct net_device *)ptr;
	struct ath_softc *sc = (struct ath_softc *)dev->priv;

	if (!dev || !sc || dev->open != &ath_init)
		return 0;

	switch (event) {
	case NETDEV_CHANGENAME:
		ath_dynamic_sysctl_unregister(sc);
		ath_dynamic_sysctl_register(sc);
		return NOTIFY_DONE;
	default:
		break;
	}
	return 0;
}

/* For any addresses we wish to get a symbolic representation of (i.e. flag
 * names) we can add it to this helper function and a subsequent line is
 * printed with the status in symbolic form. */
#ifdef ATH_REVERSE_ENGINEERING
static void
ath_print_register_details(const char* name, u_int32_t address, u_int32_t v)
{
/* constants from openhal ar5212reg.h */
#define AR5K_AR5212_PHY_ERR_FIL		    0x810c
#define AR5K_AR5212_PHY_ERR_FIL_RADAR	0x00000020
#define AR5K_AR5212_PHY_ERR_FIL_OFDM	0x00020000
#define AR5K_AR5212_PHY_ERR_FIL_CCK     0x02000000
#define AR5K_AR5212_PIMR		    0x00a0
#define AR5K_AR5212_PISR		    0x0080
#define AR5K_AR5212_PIMR_RXOK		0x00000001
#define AR5K_AR5212_PIMR_RXDESC		0x00000002
#define AR5K_AR5212_PIMR_RXERR		0x00000004
#define AR5K_AR5212_PIMR_RXNOFRM	0x00000008
#define AR5K_AR5212_PIMR_RXEOL		0x00000010
#define AR5K_AR5212_PIMR_RXORN		0x00000020
#define AR5K_AR5212_PIMR_TXOK		0x00000040
#define AR5K_AR5212_PIMR_TXDESC		0x00000080
#define AR5K_AR5212_PIMR_TXERR		0x00000100
#define AR5K_AR5212_PIMR_TXNOFRM	0x00000200
#define AR5K_AR5212_PIMR_TXEOL		0x00000400
#define AR5K_AR5212_PIMR_TXURN		0x00000800
#define AR5K_AR5212_PIMR_MIB		0x00001000
#define AR5K_AR5212_PIMR_SWI		0x00002000
#define AR5K_AR5212_PIMR_RXPHY		0x00004000
#define AR5K_AR5212_PIMR_RXKCM		0x00008000
#define AR5K_AR5212_PIMR_SWBA		0x00010000
#define AR5K_AR5212_PIMR_BRSSI		0x00020000
#define AR5K_AR5212_PIMR_BMISS		0x00040000
#define AR5K_AR5212_PIMR_HIUERR		0x00080000
#define AR5K_AR5212_PIMR_BNR		0x00100000
#define AR5K_AR5212_PIMR_RXCHIRP	0x00200000
#define AR5K_AR5212_PIMR_TIM		0x00800000
#define AR5K_AR5212_PIMR_BCNMISC	0x00800000
#define AR5K_AR5212_PIMR_GPIO		0x01000000
#define AR5K_AR5212_PIMR_QCBRORN	0x02000000
#define AR5K_AR5212_PIMR_QCBRURN	0x04000000
#define AR5K_AR5212_PIMR_QTRIG		0x08000000

	if (address == AR5K_AR5212_PHY_ERR_FIL) {
		printk(KERN_DEBUG "%18s info:%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s"
				"%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s\n",
		       (name == strstr(name, "AR5K_AR5212_") ? 
			(name + strlen("AR5K_AR5212_")) : 
			name),
		       (v & (1 << 31)                ? " (1 << 31)"     : ""),
		       (v & (1 << 30)                ? " (1 << 30)"     : ""),
		       (v & (1 << 29)                ? " (1 << 29)"     : ""),
		       (v & (1 << 28)                ? " (1 << 28)"     : ""),
		       (v & (1 << 27)                ? " (1 << 27)"     : ""),
		       (v & (1 << 26)                ? " (1 << 26)"     : ""),
		       (v & AR5K_AR5212_PHY_ERR_FIL_CCK  ? " CCK"       : ""),
		       (v & (1 << 24)                ? " (1 << 24)"     : ""),
		       (v & (1 << 23)                ? " (1 << 23)"     : ""),
		       (v & (1 << 22)                ? " (1 << 22)"     : ""),
		       (v & (1 << 21)                ? " (1 << 21)"     : ""),
		       (v & (1 << 20)                ? " (1 << 20)"     : ""),
		       (v & (1 << 19)                ? " (1 << 19)"     : ""),
		       (v & (1 << 18)                ? " (1 << 18)"     : ""),
		       (v & AR5K_AR5212_PHY_ERR_FIL_OFDM ? " OFDM"      : ""),
		       (v & (1 << 16)                ? " (1 << 16)"     : ""),
		       (v & (1 << 15)                ? " (1 << 15)"     : ""),
		       (v & (1 << 14)                ? " (1 << 14)"     : ""),
		       (v & (1 << 13)                ? " (1 << 13)"     : ""),
		       (v & (1 << 12)                ? " (1 << 12)"     : ""),
		       (v & (1 << 11)                ? " (1 << 11)"     : ""),
		       (v & (1 << 10)                ? " (1 << 10)"     : ""),
		       (v & (1 <<  9)                ? " (1 <<  9)"     : ""),
		       (v & (1 <<  8)                ? " (1 <<  8)"     : ""),
		       (v & (1 <<  7)                ? " (1 <<  7)"     : ""),
		       (v & (1 <<  6)                ? " (1 <<  6)"     : ""),
		       (v & AR5K_AR5212_PHY_ERR_FIL_RADAR ? " RADAR"    : ""),
		       (v & (1 <<  4)                ? " (1 <<  4)"     : ""),
		       (v & (1 <<  3)                ? " (1 <<  3)"     : ""),
		       (v & (1 <<  2)                ? " (1 <<  2)"     : ""),
		       (v & (1 <<  1)                ? " (1 <<  1)"     : ""),
		       (v & (1 <<  0)                ? " (1 <<  0)"     : "")
		      );
	}
	if (address == AR5K_AR5212_PISR || address == AR5K_AR5212_PIMR) {
		printk(KERN_DEBUG "%18s info:%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s"
				"%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s\n",
			(name == strstr(name, "AR5K_AR5212_") ? 
			 (name + strlen("AR5K_AR5212_")) : 
			 name),
			(v & HAL_INT_GLOBAL           ?  " HAL_INT_GLOBAL" : ""),
			(v & HAL_INT_FATAL            ?  " HAL_INT_FATAL"  : ""),
			(v & (1 << 29)                ?  " (1  << 29)"     : ""),
			(v & (1 << 28)                ?  " (1  << 28)"     : ""),
			(v & AR5K_AR5212_PIMR_RXOK    ?  " RXOK"           : ""),
			(v & AR5K_AR5212_PIMR_RXDESC  ?  " RXDESC"         : ""),
			(v & AR5K_AR5212_PIMR_RXERR   ?  " RXERR"          : ""),
			(v & AR5K_AR5212_PIMR_RXNOFRM ?  " RXNOFRM"        : ""),
			(v & AR5K_AR5212_PIMR_RXEOL   ?  " RXEOL"          : ""),
			(v & AR5K_AR5212_PIMR_RXORN   ?  " RXORN"          : ""),
			(v & AR5K_AR5212_PIMR_TXOK    ?  " TXOK"           : ""),
			(v & AR5K_AR5212_PIMR_TXDESC  ?  " TXDESC"         : ""),
			(v & AR5K_AR5212_PIMR_TXERR   ?  " TXERR"          : ""),
			(v & AR5K_AR5212_PIMR_TXNOFRM ?  " TXNOFRM"        : ""),
			(v & AR5K_AR5212_PIMR_TXEOL   ?  " TXEOL"          : ""),
			(v & AR5K_AR5212_PIMR_TXURN   ?  " TXURN"          : ""),
			(v & AR5K_AR5212_PIMR_MIB     ?  " MIB"            : ""),
			(v & AR5K_AR5212_PIMR_SWI     ?  " SWI"            : ""),
			(v & AR5K_AR5212_PIMR_RXPHY   ?  " RXPHY"          : ""),
			(v & AR5K_AR5212_PIMR_RXKCM   ?  " RXKCM"          : ""),
			(v & AR5K_AR5212_PIMR_SWBA    ?  " SWBA"           : ""),
			(v & AR5K_AR5212_PIMR_BRSSI   ?  " BRSSI"          : ""),
			(v & AR5K_AR5212_PIMR_BMISS   ?  " BMISS"          : ""),
			(v & AR5K_AR5212_PIMR_HIUERR  ?  " HIUERR"         : ""),
			(v & AR5K_AR5212_PIMR_BNR     ?  " BNR"            : ""),
			(v & AR5K_AR5212_PIMR_RXCHIRP ?  " RXCHIRP"        : ""),
			(v & AR5K_AR5212_PIMR_TIM     ?  " TIM"            : ""),
			(v & AR5K_AR5212_PIMR_BCNMISC ?  " BCNMISC"        : ""),
			(v & AR5K_AR5212_PIMR_GPIO    ?  " GPIO"           : ""),
			(v & AR5K_AR5212_PIMR_QCBRORN ?  " QCBRORN"        : ""),
			(v & AR5K_AR5212_PIMR_QCBRURN ?  " QCBRURN"        : ""),
			(v & AR5K_AR5212_PIMR_QTRIG   ?  " QTRIG"          : "")
			);
	}
#undef AR5K_AR5212_PHY_ERR_FIL
#undef AR5K_AR5212_PHY_ERR_FIL_RADAR
#undef AR5K_AR5212_PHY_ERR_FIL_OFDM
#undef AR5K_AR5212_PHY_ERR_FIL_CCK
#undef AR5K_AR5212_PIMR
#undef AR5K_AR5212_PISR
#undef AR5K_AR5212_PIMR_RXOK
#undef AR5K_AR5212_PIMR_RXDESC
#undef AR5K_AR5212_PIMR_RXERR
#undef AR5K_AR5212_PIMR_RXNOFRM
#undef AR5K_AR5212_PIMR_RXEOL
#undef AR5K_AR5212_PIMR_RXORN
#undef AR5K_AR5212_PIMR_TXOK
#undef AR5K_AR5212_PIMR_TXDESC
#undef AR5K_AR5212_PIMR_TXERR
#undef AR5K_AR5212_PIMR_TXNOFRM
#undef AR5K_AR5212_PIMR_TXEOL
#undef AR5K_AR5212_PIMR_TXURN
#undef AR5K_AR5212_PIMR_MIB
#undef AR5K_AR5212_PIMR_SWI
#undef AR5K_AR5212_PIMR_RXPHY
#undef AR5K_AR5212_PIMR_RXKCM
#undef AR5K_AR5212_PIMR_SWBA
#undef AR5K_AR5212_PIMR_BRSSI
#undef AR5K_AR5212_PIMR_BMISS
#undef AR5K_AR5212_PIMR_HIUERR
#undef AR5K_AR5212_PIMR_BNR
#undef AR5K_AR5212_PIMR_RXCHIRP
#undef AR5K_AR5212_PIMR_TIM
#undef AR5K_AR5212_PIMR_BCNMISC
#undef AR5K_AR5212_PIMR_GPIO
#undef AR5K_AR5212_PIMR_QCBRORN
#undef AR5K_AR5212_PIMR_QCBRURN
#undef AR5K_AR5212_PIMR_QTRIG
}
#endif /* #ifdef ATH_REVERSE_ENGINEERING */

/* Print out a register with name, address and value in hex and binary. If
 * v_old and v_new are the same we just dump the binary out (zeros are listed
 * using dots for easier reading). If v_old and v_new are NOT the same, we
 * indicate which bits were activated or de-activated using differnet
 * characters than 1. */
#ifdef ATH_REVERSE_ENGINEERING
static void
ath_print_register_delta(const char* name, u_int32_t address, u_int32_t v_old, 
		u_int32_t v_new)
{
#define BIT_UNCHANGED_ON  "1"
#define BIT_UNCHANGED_OFF "."
#define BIT_CHANGED_ON    "+"
#define BIT_CHANGED_OFF   "-"
#define NYBBLE_SEPARATOR   ""
#define BYTE_SEPARATOR    " "
#define BIT_STATUS(_shift) \
	(((v_old & (1 << _shift)) == (v_new & (1 << _shift))) ? \
		(v_new & (1 << _shift) ? 			\
		 BIT_UNCHANGED_ON : BIT_UNCHANGED_OFF) :	\
		(v_new & (1 << _shift) ? 			\
		 BIT_CHANGED_ON : BIT_CHANGED_OFF))

	/* Used for formatting hex data with spacing */
	static char nybbles[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', 
		'9', 'a', 'b', 'c', 'd', 'e', 'f'};
	char address_string[10] = "";

	if (address != 0xffffffff) {
		address_string[0] = '*';
		address_string[1] = '0';
		address_string[2] = 'x';
		address_string[3] = nybbles[(address >> 12) & 0x0f];
		address_string[4] = nybbles[(address >>  8) & 0x0f];
		address_string[5] = nybbles[(address >>  4) & 0x0f];
		address_string[6] = nybbles[(address >>  0) & 0x0f];
		address_string[7] = '=';
		address_string[9] = '\0';
	}
	printk(KERN_DEBUG
		"%23s: %s0x%08x%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s"
			"%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s\n",
		(name == strstr(name, "AR5K_AR5212_") ? 
		 (name+strlen("AR5K_AR5212_")) : 
		 name),
		address_string,
		v_new,
		"  ",
		BIT_STATUS(31),
		BIT_STATUS(30),
		BIT_STATUS(29),
		BIT_STATUS(28),
		NYBBLE_SEPARATOR,
		BIT_STATUS(27),
		BIT_STATUS(26),
		BIT_STATUS(25),
		BIT_STATUS(24),
		BYTE_SEPARATOR,
		BIT_STATUS(23),
		BIT_STATUS(22),
		BIT_STATUS(21),
		BIT_STATUS(20),
		NYBBLE_SEPARATOR,
		BIT_STATUS(19),
		BIT_STATUS(18),
		BIT_STATUS(17),
		BIT_STATUS(16),
		BYTE_SEPARATOR,
		BIT_STATUS(15),
		BIT_STATUS(14),
		BIT_STATUS(13),
		BIT_STATUS(12),
		NYBBLE_SEPARATOR,
		BIT_STATUS(11),
		BIT_STATUS(10),
		BIT_STATUS( 9),
		BIT_STATUS( 8),
		BYTE_SEPARATOR,
		BIT_STATUS( 7),
		BIT_STATUS( 6),
		BIT_STATUS( 5),
		BIT_STATUS( 4),
		NYBBLE_SEPARATOR,
		BIT_STATUS( 3),
		BIT_STATUS( 2),
		BIT_STATUS( 1),
		BIT_STATUS( 0),
		""
		);
#undef BIT_UNCHANGED_ON
#undef BIT_UNCHANGED_OFF
#undef BIT_CHANGED_ON
#undef BIT_CHANGED_OFF
#undef NYBBLE_SEPARATOR
#undef BYTE_SEPARATOR
#undef BIT_STATUS
}
#endif /* #ifdef ATH_REVERSE_ENGINEERING */

/* Lookup a friendly name for a register address (for any we have nicknames
 * for). Names were taken from openhal ar5212regs.h. Return AH_TRUE if the
 * name is a known ar5212 register, and AH_FALSE otherwise. */
#ifdef ATH_REVERSE_ENGINEERING
static HAL_BOOL
ath_lookup_register_name(struct ath_softc *sc, char* buf, int buflen, 
		u_int32_t address) {
	const char* static_label = NULL;
	memset(buf, 0, buflen);

	if ((ar_device(sc->devid) == 5212) || (ar_device(sc->devid) == 5213)) {
		/* Handle Static Register Labels (unique stuff we know about) */
		switch (address) {
		case 0x0008: static_label = "CR";                     break;
		case 0x000c: static_label = "RXDP";                   break;
		case 0x0014: static_label = "CFG";                    break;
		case 0x0024: static_label = "IER";                    break;
		case 0x0030: static_label = "TXCFG";                  break;
		case 0x0034: static_label = "RXCFG";                  break;
		case 0x0040: static_label = "MIBC";                   break;
		case 0x0044: static_label = "TOPS";                   break;
		case 0x0048: static_label = "RXNOFRM";                break;
		case 0x004c: static_label = "TXNOFRM";                break;
		case 0x0050: static_label = "RPGTO";                  break;
		case 0x0054: static_label = "RFCNT";                  break;
		case 0x0058: static_label = "MISC";                   break;
		case 0x0080: static_label = "PISR";                   break;
		case 0x0084: static_label = "SISR0";                  break;
		case 0x0088: static_label = "SISR1";                  break;
		case 0x008c: static_label = "SISR2";                  break;
		case 0x0090: static_label = "SISR3";                  break;
		case 0x0094: static_label = "SISR4";                  break;
		case 0x00a0: static_label = "PIMR";                   break;
		case 0x00a4: static_label = "SIMR0";                  break;
		case 0x00a8: static_label = "SIMR1";                  break;
		case 0x00ac: static_label = "SIMR2";                  break;
		case 0x00b0: static_label = "SIMR3";                  break;
		case 0x00b4: static_label = "SIMR4";                  break;
		case 0x0400: static_label = "DCM_ADDR";               break;
		case 0x0404: static_label = "DCM_DATA";               break;
		case 0x0420: static_label = "DCCFG";                  break;
		case 0x0600: static_label = "CCFG";                   break;
		case 0x0604: static_label = "CCFG_CUP";               break;
		case 0x0610: static_label = "CPC0";                   break;
		case 0x0614: static_label = "CPC1";                   break;
		case 0x0618: static_label = "CPC2";                   break;
		case 0x061c: static_label = "CPC3";                   break;
		case 0x0620: static_label = "CPCORN";                 break;
		case 0x0800: static_label = "QCU_TXDP(0)";            break;
		case 0x0804: static_label = "QCU_TXDP(1)";            break;
		case 0x0808: static_label = "QCU_TXDP(2)";            break;
		case 0x080c: static_label = "QCU_TXDP(3)";            break;
		case 0x0810: static_label = "QCU_TXDP(4)";            break;
		case 0x0814: static_label = "QCU_TXDP(5)";            break;
		case 0x0818: static_label = "QCU_TXDP(6)";            break;
		case 0x081c: static_label = "QCU_TXDP(7)";            break;
		case 0x0820: static_label = "QCU_TXDP(8)";            break;
		case 0x0824: static_label = "QCU_TXDP(9)";            break;
		case 0x0840: static_label = "QCU_TXE";                break;
		case 0x0880: static_label = "QCU_TXD";                break;
		case 0x08c0: static_label = "QCU_CBRCFG(0)";          break;
		case 0x08c4: static_label = "QCU_CBRCFG(1)";          break;
		case 0x08c8: static_label = "QCU_CBRCFG(2)";          break;
		case 0x08cc: static_label = "QCU_CBRCFG(3)";          break;
		case 0x08d0: static_label = "QCU_CBRCFG(4)";          break;
		case 0x08d4: static_label = "QCU_CBRCFG(5)";          break;
		case 0x08d8: static_label = "QCU_CBRCFG(6)";          break;
		case 0x08dc: static_label = "QCU_CBRCFG(7)";          break;
		case 0x08e0: static_label = "QCU_CBRCFG(8)";          break;
		case 0x08e4: static_label = "QCU_CBRCFG(9)";          break;
		case 0x0900: static_label = "QCU_RDYTIMECFG(0)";      break;
		case 0x0904: static_label = "QCU_RDYTIMECFG(1)";      break;
		case 0x0908: static_label = "QCU_RDYTIMECFG(2)";      break;
		case 0x090c: static_label = "QCU_RDYTIMECFG(3)";      break;
		case 0x0910: static_label = "QCU_RDYTIMECFG(4)";      break;
		case 0x0914: static_label = "QCU_RDYTIMECFG(5)";      break;
		case 0x0918: static_label = "QCU_RDYTIMECFG(6)";      break;
		case 0x091c: static_label = "QCU_RDYTIMECFG(7)";      break;
		case 0x0920: static_label = "QCU_RDYTIMECFG(8)";      break;
		case 0x0924: static_label = "QCU_RDYTIMECFG(9)";      break;
		case 0x0940: static_label = "QCU_ONESHOTARM_SET(0)";  break;
		case 0x0944: static_label = "QCU_ONESHOTARM_SET(1)";  break;
		case 0x0948: static_label = "QCU_ONESHOTARM_SET(2)";  break;
		case 0x094c: static_label = "QCU_ONESHOTARM_SET(3)";  break;
		case 0x0950: static_label = "QCU_ONESHOTARM_SET(4)";  break;
		case 0x0954: static_label = "QCU_ONESHOTARM_SET(5)";  break;
		case 0x0958: static_label = "QCU_ONESHOTARM_SET(6)";  break;
		case 0x095c: static_label = "QCU_ONESHOTARM_SET(7)";  break;
		case 0x0960: static_label = "QCU_ONESHOTARM_SET(8)";  break;
		case 0x0964: static_label = "QCU_ONESHOTARM_SET(9)";  break;
		case 0x0980: static_label = "QCU_ONESHOTARM_CLR(0)";  break;
		case 0x0984: static_label = "QCU_ONESHOTARM_CLR(1)";  break;
		case 0x0988: static_label = "QCU_ONESHOTARM_CLR(2)";  break;
		case 0x098c: static_label = "QCU_ONESHOTARM_CLR(3)";  break;
		case 0x0990: static_label = "QCU_ONESHOTARM_CLR(4)";  break;
		case 0x0994: static_label = "QCU_ONESHOTARM_CLR(5)";  break;
		case 0x0998: static_label = "QCU_ONESHOTARM_CLR(6)";  break;
		case 0x099c: static_label = "QCU_ONESHOTARM_CLR(7)";  break;
		case 0x09a0: static_label = "QCU_ONESHOTARM_CLR(8)";  break;
		case 0x09a4: static_label = "QCU_ONESHOTARM_CLR(9)";  break;
		case 0x09c0: static_label = "QCU_MISC(0)";            break;
		case 0x09c4: static_label = "QCU_MISC(1)";            break;
		case 0x09c8: static_label = "QCU_MISC(2)";            break;
		case 0x09cc: static_label = "QCU_MISC(3)";            break;
		case 0x09d0: static_label = "QCU_MISC(4)";            break;
		case 0x09d4: static_label = "QCU_MISC(5)";            break;
		case 0x09d8: static_label = "QCU_MISC(6)";            break;
		case 0x09dc: static_label = "QCU_MISC(7)";            break;
		case 0x09e0: static_label = "QCU_MISC(8)";            break;
		case 0x09e4: static_label = "QCU_MISC(9)";            break;
		case 0x0a00: static_label = "QCU_STS(0)";             break;
		case 0x0a04: static_label = "QCU_STS(1)";             break;
		case 0x0a08: static_label = "QCU_STS(2)";             break;
		case 0x0a0c: static_label = "QCU_STS(3)";             break;
		case 0x0a10: static_label = "QCU_STS(4)";             break;
		case 0x0a14: static_label = "QCU_STS(5)";             break;
		case 0x0a18: static_label = "QCU_STS(6)";             break;
		case 0x0a1c: static_label = "QCU_STS(7)";             break;
		case 0x0a20: static_label = "QCU_STS(8)";             break;
		case 0x0a24: static_label = "QCU_STS(9)";             break;
		case 0x0a40: static_label = "QCU_RDYTIMESHDN";        break;
		case 0x0b00: static_label = "QCU_CBB_SELECT";         break;
		case 0x0b04: static_label = "QCU_CBB_ADDR";           break;
		case 0x0b08: static_label = "QCU_CBCFG";              break;
		case 0x1000: static_label = "DCU_QCUMASK(9)";         break;
		case 0x1004: static_label = "DCU_QCUMASK(0)";         break;
		case 0x1008: static_label = "DCU_QCUMASK(1)";         break;
		case 0x100c: static_label = "DCU_QCUMASK(2)";         break;
		case 0x1010: static_label = "DCU_QCUMASK(3)";         break;
		case 0x1014: static_label = "DCU_QCUMASK(4)";         break;
		case 0x1018: static_label = "DCU_QCUMASK(5)";         break;
		case 0x101c: static_label = "DCU_QCUMASK(6)";         break;
		case 0x1020: static_label = "DCU_QCUMASK(7)";         break;
		case 0x1024: static_label = "DCU_QCUMASK(8)";         break;
		case 0x1030: static_label = "DCU_GBL_IFS_SIFS";       break;
		case 0x1038: static_label = "DCU_TX_FILTER";          break;
		case 0x1040: static_label = "DCU_LCL_IFS(0)";         break;
		case 0x1044: static_label = "DCU_LCL_IFS(1)";         break;
		case 0x1048: static_label = "DCU_LCL_IFS(2)";         break;
		case 0x104c: static_label = "DCU_LCL_IFS(3)";         break;
		case 0x1050: static_label = "DCU_LCL_IFS(4)";         break;
		case 0x1054: static_label = "DCU_LCL_IFS(5)";         break;
		case 0x1058: static_label = "DCU_LCL_IFS(6)";         break;
		case 0x105c: static_label = "DCU_LCL_IFS(7)";         break;
		case 0x1060: static_label = "DCU_LCL_IFS(8)";         break;
		case 0x1064: static_label = "DCU_LCL_IFS(9)";         break;
		case 0x1070: static_label = "DCU_GBL_IFS_SLOT";       break;
		case 0x1080: static_label = "DCU_RETRY_LIMIT(0)";     break;
		case 0x1084: static_label = "DCU_RETRY_LIMIT(1)";     break;
		case 0x1088: static_label = "DCU_RETRY_LIMIT(2)";     break;
		case 0x108c: static_label = "DCU_RETRY_LIMIT(3)";     break;
		case 0x1090: static_label = "DCU_RETRY_LIMIT(4)";     break;
		case 0x1094: static_label = "DCU_RETRY_LIMIT(5)";     break;
		case 0x1098: static_label = "DCU_RETRY_LIMIT(6)";     break;
		case 0x109c: static_label = "DCU_RETRY_LIMIT(7)";     break;
		case 0x10a0: static_label = "DCU_RETRY_LIMIT(8)";     break;
		case 0x10a4: static_label = "DCU_RETRY_LIMIT(9)";     break;
		case 0x10b0: static_label = "DCU_GBL_IFS_EIFS";       break;
		case 0x10c0: static_label = "DCU_CHAN_TIME(0)";       break;
		case 0x10c4: static_label = "DCU_CHAN_TIME(1)";       break;
		case 0x10c8: static_label = "DCU_CHAN_TIME(2)";       break;
		case 0x10cc: static_label = "DCU_CHAN_TIME(3)";       break;
		case 0x10d0: static_label = "DCU_CHAN_TIME(4)";       break;
		case 0x10d4: static_label = "DCU_CHAN_TIME(5)";       break;
		case 0x10d8: static_label = "DCU_CHAN_TIME(6)";       break;
		case 0x10dc: static_label = "DCU_CHAN_TIME(7)";       break;
		case 0x10e0: static_label = "DCU_CHAN_TIME(8)";       break;
		case 0x10e4: static_label = "DCU_CHAN_TIME(9)";       break;
		case 0x10f0: static_label = "DCU_GBL_IFS_MISC";       break;
		case 0x1100: static_label = "DCU_MISC(0)";            break;
		case 0x1104: static_label = "DCU_MISC(1)";            break;
		case 0x1108: static_label = "DCU_MISC(2)";            break;
		case 0x110c: static_label = "DCU_MISC(3)";            break;
		case 0x1110: static_label = "DCU_MISC(4)";            break;
		case 0x1114: static_label = "DCU_MISC(5)";            break;
		case 0x1118: static_label = "DCU_MISC(6)";            break;
		case 0x111c: static_label = "DCU_MISC(7)";            break;
		case 0x1120: static_label = "DCU_MISC(8)";            break;
		case 0x1124: static_label = "DCU_MISC(9)";            break;
		case 0x1140: static_label = "DCU_SEQ_NUM(0)";         break;
		case 0x1144: static_label = "DCU_SEQ_NUM(1)";         break;
		case 0x1148: static_label = "DCU_SEQ_NUM(2)";         break;
		case 0x114c: static_label = "DCU_SEQ_NUM(3)";         break;
		case 0x1150: static_label = "DCU_SEQ_NUM(4)";         break;
		case 0x1154: static_label = "DCU_SEQ_NUM(5)";         break;
		case 0x1158: static_label = "DCU_SEQ_NUM(6)";         break;
		case 0x115c: static_label = "DCU_SEQ_NUM(7)";         break;
		case 0x1160: static_label = "DCU_SEQ_NUM(8)";         break;
		case 0x1164: static_label = "DCU_SEQ_NUM(9)";         break;
		case 0x1230: static_label = "DCU_FP";                 break;
		case 0x1270: static_label = "DCU_TXP";                break;
		case 0x143c: static_label = "DCU_TX_FILTER_CLR";      break;
		case 0x147c: static_label = "DCU_TX_FILTER_SET";      break;
		case 0x4000: static_label = "RESET_CONTROL";          break;
		case 0x4004: static_label = "SLEEP_CONTROL";          break;
		case 0x4008: static_label = "INTERRUPT_PENDING";      break;
		case 0x400c: static_label = "SLEEP_FORCE";            break;
		case 0x4010: static_label = "PCICFG";                 break;
		case 0x4014: static_label = "GPIOCR";                 break;
		case 0x4018: static_label = "GPIODO";                 break;
		case 0x401c: static_label = "GPIODI";                 break;
		case 0x4020: static_label = "SREV";                   break;
		case 0x6000: static_label = "EEPROM_BASE";            break;
		case 0x6004: static_label = "EEPROM_DATA";            break;
		case 0x6008: static_label = "EEPROM_CMD";             break;
		case 0x6010: static_label = "EEPROM_CFG";             break;
		case 0x8000: static_label = "STA_ID0";                break;
		case 0x8004: static_label = "STA_ID1";                break;
		case 0x8008: static_label = "BSS_ID0";                break;
		case 0x800c: static_label = "BSS_ID1";                break;
		case 0x8010: static_label = "SLOT_TIME";              break;
		case 0x8014: static_label = "TIME_OUT";               break;
		case 0x8018: static_label = "RSSI_THR";               break;
		case 0x801c: static_label = "USEC";                   break;
		case 0x8020: static_label = "BEACON";                 break;
		case 0x8024: static_label = "CFP_PERIOD";             break;
		case 0x8028: static_label = "TIMER0";                 break;
		case 0x802c: static_label = "TIMER1";                 break;
		case 0x8030: static_label = "TIMER2";                 break;
		case 0x8034: static_label = "TIMER3";                 break;
		case 0x8038: static_label = "CFP_DUR";                break;
		case 0x803c: static_label = "RX_FILTER";              break;
		case 0x8040: static_label = "MCAST_FIL0";             break;
		case 0x8044: static_label = "MCAST_FIL1";             break;
		case 0x8048: static_label = "DIAG_SW";                break;
		case 0x804c: static_label = "TSF_L32";                break;
		case 0x8050: static_label = "TSF_U32";                break;
		case 0x8054: static_label = "ADDAC_TEST";             break;
		case 0x8058: static_label = "DEFAULT_ANTENNA";        break;
		case 0x8080: static_label = "LAST_TSTP";              break;
		case 0x8084: static_label = "NAV";                    break;
		case 0x8088: static_label = "RTS_OK";                 break;
		case 0x808c: static_label = "RTS_FAIL";               break;
		case 0x8090: static_label = "ACK_FAIL";               break;
		case 0x8094: static_label = "FCS_FAIL";               break;
		case 0x8098: static_label = "BEACON_CNT";             break;
		case 0x80c0: static_label = "XRMODE";                 break;
		case 0x80c4: static_label = "XRDELAY";                break;
		case 0x80c8: static_label = "XRTIMETOUT";             break;
		case 0x80cc: static_label = "XRCHIRP";                break;
		case 0x80d0: static_label = "XRSTOMP";                break;
		case 0x80d4: static_label = "SLEEP0";                 break;
		case 0x80d8: static_label = "SLEEP1";                 break;
		case 0x80dc: static_label = "SLEEP2";                 break;
		case 0x80e0: static_label = "BSS_IDM0";               break;
		case 0x80e4: static_label = "BSS_IDM1";               break;
		case 0x80e8: static_label = "TXPC";                   break;
		case 0x80ec: static_label = "PROFCNT_TX";             break;
		case 0x80f0: static_label = "PROFCNT_RX";             break;
		case 0x80f4: static_label = "PROFCNT_RXCLR";          break;
		case 0x80f8: static_label = "PROFCNT_CYCLE";          break;
		case 0x8104: static_label = "TSF_PARM";               break;
		case 0x810c: static_label = "PHY_ERR_FIL";            break;
		case 0x9800: static_label = "PHY(0)";                 break;
		case 0x9804: static_label = "PHY_TURBO";              break;
		case 0x9808: static_label = "PHY_AGC";                break;
		case 0x9814: static_label = "PHY_TIMING_3";           break;
		case 0x9818: static_label = "PHY_CHIP_ID";            break;
		case 0x981c: static_label = "PHY_ACTIVE";             break;
		case 0x9860: static_label = "PHY_AGCCTL";             break;
		case 0x9864: static_label = "PHY_NF";                 break;
		case 0x9870: static_label = "PHY_SCR";                break;
		case 0x9874: static_label = "PHY_SLMT";               break;
		case 0x9878: static_label = "PHY_SCAL";               break;
		case 0x987c: static_label = "PHY_PLL";                break;
		case 0x9914: static_label = "PHY_RX_DELAY";           break;
		case 0x9920: static_label = "PHY_IQ";                 break;
		case 0x9930: static_label = "PHY_PAPD_PROBE";         break;
		case 0x9934: static_label = "PHY_TXPOWER_RATE1";      break;
		case 0x9938: static_label = "PHY_TXPOWER_RATE2";      break;
		case 0x993c: static_label = "PHY_TXPOWER_RATE_MAX";   break;
		case 0x9944: static_label = "PHY_FC";                 break;
		case 0x9954: static_label = "PHY_RADAR";              break;
		case 0x9960: static_label = "PHY_ANT_SWITCH_TABLE_0"; break;
		case 0x9964: static_label = "PHY_ANT_SWITCH_TABLE_1"; break;
		case 0x99f0: static_label = "PHY_SCLOCK";             break;
		case 0x99f4: static_label = "PHY_SDELAY";             break;
		case 0x99f8: static_label = "PHY_SPENDING";           break;
		case 0x9c10: static_label = "PHY_IQRES_CAL_PWR_I";    break;
		case 0x9c14: static_label = "PHY_IQRES_CAL_PWR_Q";    break;
		case 0x9c18: static_label = "PHY_IQRES_CAL_CORR";     break;
		case 0x9c1c: static_label = "PHY_CURRENT_RSSI";       break;
		case 0xa200: static_label = "PHY_MODE";               break;
		case 0xa204: static_label = "PHY_CCKTXCTL";           break;
		case 0xa20c: static_label = "PHY_GAIN_2GHZ";          break;
		case 0xa234: static_label = "PHY_TXPOWER_RATE3";      break;
		case 0xa238: static_label = "PHY_TXPOWER_RATE4";      break;
		default:
			break;
		}
		if (static_label) {
			snprintf(buf, buflen, static_label);
			return AH_TRUE;
		}

		/* Handle Key Table */
		if ((address >= 0x8800) && (address < 0x9800)) {
#define keytable_entry_reg_count (8)
#define keytable_entry_size      (keytable_entry_reg_count * sizeof(u_int32_t))
			int key = ((address - 0x8800) / keytable_entry_size);
			int reg = ((address - 0x8800) % keytable_entry_size) / 
				sizeof(u_int32_t);
			char* format = NULL;
			switch (reg) {
			case 0: format = "KEY(%3d).KEYBITS[031:000]"; break;
			case 1: format = "KEY(%3d).KEYBITS[047:032]"; break;
			case 2: format = "KEY(%3d).KEYBITS[079:048]"; break;
			case 3: format = "KEY(%3d).KEYBITS[095:080]"; break;
			case 4: format = "KEY(%3d).KEYBITS[127:096]"; break;
			case 5: format = "KEY(%3d).TYPE............"; break;
			case 6: format = "KEY(%3d).MAC[32:01]......"; break;
			case 7: format = "KEY(%3d).MAC[47:33]......"; break;
			default:
				BUG();
			}
			snprintf(buf, buflen, format, key);
#undef keytable_entry_reg_count
#undef keytable_entry_size
			return AH_TRUE;
		}

		/* Handle Rate Duration Table */
		if (address >= 0x8700 && address < 0x8780) {
			snprintf(buf, buflen, "RATE(%2d).DURATION",
					((address - 0x8700) / sizeof(u_int32_t)));
			return AH_TRUE;
		}

		/* Handle txpower Table */
		if (address >= 0xa180 && address < 0xa200) {
			snprintf(buf, buflen, "PCDAC_TXPOWER(%2d)",
					((address - 0xa180) / sizeof(u_int32_t)));
			return AH_TRUE;
		}
	}

	/* Everything else... */
	snprintf(buf, buflen, UNKNOWN_NAME);
	return AH_FALSE;
}
#endif /* #ifdef ATH_REVERSE_ENGINEERING */

/* Print out a single register name/address/value in hex and binary */
#ifdef ATH_REVERSE_ENGINEERING
static void
ath_print_register(const char* name, u_int32_t address, u_int32_t v)
{
	ath_print_register_delta(name, address, v, v);
	ath_print_register_details(name, address, v);
}
#endif /* #ifdef ATH_REVERSE_ENGINEERING */

/* A filter for hiding the addresses we don't think are very interesting or
 * which have adverse side effects. Return AH_TRUE if the address should be
 * exlucded, and AH_FALSE otherwise. */
#ifdef ATH_REVERSE_ENGINEERING
static HAL_BOOL
ath_regdump_filter(struct ath_softc *sc, u_int32_t address) {
#ifndef ATH_REVERSE_ENGINEERING_WITH_NO_FEAR
	char buf[MAX_REGISTER_NAME_LEN];
#endif
	#define UNFILTERED AH_FALSE
	#define FILTERED   AH_TRUE

	if ((ar_device(sc->devid) != 5212) && (ar_device(sc->devid) != 5213)) 
		return FILTERED;
	/* Addresses with side effects are never dumped out by bulk debug 
	 * dump routines. */
	if ((address >= 0x00c0) && (address <= 0x00df)) return FILTERED;
	if ((address >= 0x143c) && (address <= 0x143f)) return FILTERED;
	/* PCI timing registers are not interesting */
	if ((address >= 0x4000) && (address <= 0x5000)) return FILTERED;

#ifndef ATH_REVERSE_ENGINEERING_WITH_NO_FEAR
	/* We are being conservative, and do not want to access addresses that
	 * may crash the system, so we will only consider addresses we know
	 * the names of from previous reverse engineering efforts (AKA
	 * openHAL). */
	return (AH_TRUE == ath_lookup_register_name(sc, buf, 
				MAX_REGISTER_NAME_LEN, address)) ?
		UNFILTERED : FILTERED;
#else /* #ifndef ATH_REVERSE_ENGINEERING_WITH_NO_FEAR */

	return UNFILTERED;
#endif /* #ifndef ATH_REVERSE_ENGINEERING_WITH_NO_FEAR */
	#undef UNFILTERED
	#undef FILTERED
}
#endif /* #ifdef ATH_REVERSE_ENGINEERING */

/* Dump any Atheros registers we think might be interesting. */
#ifdef ATH_REVERSE_ENGINEERING
static void
ath_ar5212_registers_dump(struct ath_softc *sc) {
	char name[MAX_REGISTER_NAME_LEN];
	unsigned int address = MIN_REGISTER_ADDRESS;
	unsigned int value   = 0;

	do {
		if (ath_regdump_filter(sc, address))
			continue;
		ath_lookup_register_name(sc, name, 
				MAX_REGISTER_NAME_LEN, address);
		value = ath_reg_read(sc, address);
		ath_print_register(name, address, value);
	} while ((address += 4) < MAX_REGISTER_ADDRESS);
}
#endif /* #ifdef ATH_REVERSE_ENGINEERING */

/* Dump any changes that were made to Atheros registers we think might be
 * interesting, since the last call to ath_ar5212_registers_mark. */
#ifdef ATH_REVERSE_ENGINEERING
static void
ath_ar5212_registers_dump_delta(struct ath_softc *sc)
{
	unsigned int address = MIN_REGISTER_ADDRESS;
	unsigned int value   = 0;
	char name[MAX_REGISTER_NAME_LEN];
	unsigned int *p_old  = 0;

	do {
		if (ath_regdump_filter(sc, address))
			continue;
		value = ath_reg_read(sc, address);
		p_old = (unsigned int*)&sc->register_snapshot[address];
		if (*p_old != value) {
			ath_lookup_register_name(sc, name, 
					MAX_REGISTER_NAME_LEN, address);
			ath_print_register_delta(name, address, *p_old, value);
			ath_print_register_details(name, address, value);
		}
	} while ((address += 4) < MAX_REGISTER_ADDRESS);
}
#endif /* #ifdef ATH_REVERSE_ENGINEERING */

/* Mark the current values of all Atheros registers we think might be
 * interesting, so any changes can be dumped out by a subsequent call to
 * ath_ar5212_registers_dump_delta. */
#ifdef ATH_REVERSE_ENGINEERING
static void
ath_ar5212_registers_mark(struct ath_softc *sc)
{
	unsigned int address = MIN_REGISTER_ADDRESS;

	do {
		*((unsigned int*)&sc->register_snapshot[address]) =
			ath_regdump_filter(sc, address) ?
			0x0 : ath_reg_read(sc, address);
	} while ((address += 4) < MAX_REGISTER_ADDRESS);
}
#endif /* #ifdef ATH_REVERSE_ENGINEERING */

/* Read an Atheros register...for reverse engineering. */
#ifdef ATH_REVERSE_ENGINEERING
static unsigned int
ath_read_register(struct ieee80211com *ic, unsigned int address, 
		unsigned int* value)
{
	struct ath_softc *sc = ic->ic_dev->priv;
	if (address >= MAX_REGISTER_ADDRESS) {
		printk(KERN_ERR "%s: %s: Illegal Atheros register access "
				"attempted: 0x%04x >= 0x%04x\n",
				DEV_NAME(sc->sc_dev), __func__, address,
				MAX_REGISTER_ADDRESS);
		return 1;
	}
	if (address % 4) {
		printk(KERN_ERR "%s: %s: Illegal Atheros register access "
				"attempted: 0x%04x %% 4 != 0\n",
				DEV_NAME(sc->sc_dev), __func__, address);
		return 1;
	}
	*value = ath_reg_read(sc, address);
	printk(KERN_DEBUG "*0x%04x -> 0x%08x\n", address, *value);
	return 0;
}
#endif /* #ifdef ATH_REVERSE_ENGINEERING */

/* Write to a Atheros register...for reverse engineering.
 * XXX: known issue with iwpriv argument handling.  It only knows how to
 * handle signed 32-bit integers and seems to get confused if you are writing
 * 0xffffffff or something. Using the signed integer equivalent always works,
 * but for some reason 0xffffffff is just as likely to give you something else
 * at the moment. */
#ifdef ATH_REVERSE_ENGINEERING
static unsigned int
ath_write_register(struct ieee80211com *ic, unsigned int address, 
		unsigned int value)
{
	struct ath_softc *sc = ic->ic_dev->priv;
	if (address >= MAX_REGISTER_ADDRESS) {
		printk(KERN_ERR "%s: %s: Illegal Atheros register access "
				"attempted: 0x%04x >= 0x%04x\n",
				DEV_NAME(sc->sc_dev), __func__, address,
				MAX_REGISTER_ADDRESS);
		return 1;
	}
	if (address % 4) {
		printk(KERN_ERR "%s: %s: Illegal Atheros register access "
				"attempted: 0x%04x %% 4 != 0\n",
				DEV_NAME(sc->sc_dev), __func__, address);
		return 1;
	}
	ath_reg_write(sc, address, value);
	printk(KERN_DEBUG "*0x%04x <- 0x%08x = 0x%08x\n", address, value,
			ath_reg_read(sc, address));
	return 0;
}
#endif /* #ifdef ATH_REVERSE_ENGINEERING */

/* Dump out Atheros registers (excluding known duplicate mappings, 
 * unmapped zones, etc.) */
#ifdef ATH_REVERSE_ENGINEERING
static void
ath_registers_dump(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	ath_ar5212_registers_dump(sc);
}
#endif /* #ifdef ATH_REVERSE_ENGINEERING */

/* Make a copy of significant registers in the Atheros chip for later
 * comparison and dump with ath_registers_dump_delta */
#ifdef ATH_REVERSE_ENGINEERING
static void
ath_registers_mark(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	ath_ar5212_registers_mark(sc);
}
#endif /* #ifdef ATH_REVERSE_ENGINEERING */

/* Dump out any registers changed since the last call to 
 * ath_registers_mark */
#ifdef ATH_REVERSE_ENGINEERING
static void
ath_registers_dump_delta(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	ath_ar5212_registers_dump_delta(sc);
}
#endif /* #ifdef ATH_REVERSE_ENGINEERING */

/* Caller must have the TXBUF_LOCK */
static void
#ifdef IEEE80211_DEBUG_REFCNT
ath_return_txbuf_locked_debug(struct ath_softc *sc, struct ath_buf **bf, 
		const char* func, int line) 
#else
ath_return_txbuf_locked(struct ath_softc *sc, struct ath_buf **bf) 
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */
{
	struct ath_buf *bfaddr;
	ATH_TXBUF_LOCK_ASSERT(sc);

	if ((bf == NULL) || ((*bf) == NULL)) 
		return;
	bfaddr = *bf;
#ifdef IEEE80211_DEBUG_REFCNT
	cleanup_ath_buf_debug(sc, (*bf), BUS_DMA_TODEVICE, func, line);
#else
	cleanup_ath_buf(sc, (*bf), BUS_DMA_TODEVICE);
#endif
	STAILQ_INSERT_TAIL(&sc->sc_txbuf, (*bf), bf_list);
	atomic_dec(&ath_buf_counter);
#ifdef IEEE80211_DEBUG_REFCNT
	DPRINTF(sc, ATH_DEBUG_TXBUF, 
		"[TXBUF=%03d/%03d] %s:%d -> %s:%d returned txbuf %p.\n", 
		ath_get_buffer_count(), ATH_TXBUF,
		func, line, __func__, __LINE__, bfaddr);
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */
	if (sc->sc_devstopped) {
		++sc->sc_reapcount;
		if (sc->sc_reapcount > ATH_TXBUF_FREE_THRESHOLD) {
			
			netif_start_queue(sc->sc_dev);
			DPRINTF(sc, ATH_DEBUG_ANY,
			    "%s: restarting queue.\n",
			    __func__);
			
			sc->sc_reapcount = 0;
			sc->sc_devstopped = 0;
		}
		else 
			ATH_SCHEDULE_TQUEUE(&sc->sc_txtq, NULL);
	}

	*bf = NULL;
}

/* Takes the TXBUF_LOCK */
static void 
#ifdef IEEE80211_DEBUG_REFCNT
ath_return_txbuf_debug(struct ath_softc *sc, struct ath_buf **bf, 
		const char* func, int line) 
#else
ath_return_txbuf(struct ath_softc *sc, struct ath_buf **bf) 
#endif 
{
	ATH_TXBUF_LOCK_IRQ(sc);
#ifdef IEEE80211_DEBUG_REFCNT
	ath_return_txbuf_locked_debug(sc, bf, func, line);
#else
	ath_return_txbuf_locked(sc, bf);
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */
	ATH_TXBUF_UNLOCK_IRQ(sc);
}

/* Takes the lock */
static void 
#ifdef IEEE80211_DEBUG_REFCNT
ath_return_txbuf_list_debug(struct ath_softc *sc, ath_bufhead *bfhead, 
		const char* func, int line) 
#else
ath_return_txbuf_list(struct ath_softc *sc, ath_bufhead *bfhead) 
#endif
{
	if (!bfhead)
		return;
	ATH_TXBUF_LOCK_IRQ(sc);
	if (!STAILQ_EMPTY(bfhead)) {
		struct ath_buf* tbf;
		struct ath_buf* tempbf;
		STAILQ_FOREACH_SAFE(tbf, bfhead, bf_list, tempbf) {
#ifdef IEEE80211_DEBUG_REFCNT
			ath_return_txbuf_locked_debug(sc, &tbf, func, line);
#else
			ath_return_txbuf_locked(sc, &tbf);
#endif
		}
	}
	ATH_TXBUF_UNLOCK_IRQ(sc);
	STAILQ_INIT(bfhead);
}
/* Caller must have the lock */
static void 
#ifdef IEEE80211_DEBUG_REFCNT
ath_return_txbuf_list_locked_debug(struct ath_softc *sc, ath_bufhead *bfhead, 
		const char* func, int line)
#else
ath_return_txbuf_list_locked(struct ath_softc *sc, ath_bufhead *bfhead)
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */
{
	ATH_TXBUF_LOCK_ASSERT(sc);
	if (!bfhead)
		return;
	
	if (!STAILQ_EMPTY(bfhead)) {
		struct ath_buf* tbf;
		struct ath_buf* tempbf;
		STAILQ_FOREACH_SAFE(tbf, bfhead, bf_list, tempbf) {
#ifdef IEEE80211_DEBUG_REFCNT
			ath_return_txbuf_locked_debug(sc, &tbf, func, line);
#else
			ath_return_txbuf_locked(sc, &tbf);
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */
		}
	}
	STAILQ_INIT(bfhead);
}

static struct ath_buf* 
#ifdef IEEE80211_DEBUG_REFCNT
cleanup_ath_buf_debug(struct ath_softc *sc, struct ath_buf *bf, int direction, 
		const char* func, int line) 
#else /* #ifdef IEEE80211_DEBUG_REFCNT */
cleanup_ath_buf(struct ath_softc *sc, struct ath_buf *bf, int direction) 
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */
{
	if (bf == NULL) 
		return bf;

	if (bf->bf_skbaddr) {
		bus_unmap_single(
			sc->sc_bdev,
			bf->bf_skbaddr, 
			(direction == BUS_DMA_FROMDEVICE ? 
				sc->sc_rxbufsize : bf->bf_skb->len),
			direction);
		bf->bf_skbaddr = 0;
		bf->bf_desc->ds_link = 0;
		bf->bf_desc->ds_data = 0;
	}

#ifdef ATH_SUPERG_FF
	{
		unsigned int i = 0;
		struct sk_buff* next_ffskb = NULL;
		/* Start with the second skb for FF */
		struct sk_buff* ffskb = bf->bf_skb ? 
			bf->bf_skb->next : NULL;
		while (ffskb) {
			next_ffskb = ffskb->next;

			if (bf->bf_skbaddrff[i] != 0) {
				bus_unmap_single(
					sc->sc_bdev,
					bf->bf_skbaddrff[i], 
					(direction == BUS_DMA_TODEVICE ? 
					 sc->sc_rxbufsize : ffskb->len), 
					direction);
				bf->bf_skbaddrff[i] = 0;
			}

			ffskb = next_ffskb;
			i++;
		}
		memset(bf->bf_skbaddrff, 0, sizeof(bf->bf_skbaddrff));
		bf->bf_numdescff = 0;
	}
#endif /* ATH_SUPERG_FF */

	if (bf->bf_node != NULL) {
#ifdef IEEE80211_DEBUG_REFCNT
		ieee80211_unref_node_debug(&bf->bf_node, func, line);
#else
		ieee80211_unref_node(&bf->bf_node);
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */
	}

	bf->bf_flags = 0;
	if (bf->bf_desc) {
		bf->bf_desc->ds_link = 0;
		bf->bf_desc->ds_data = 0;
	}

	if (bf->bf_skb != NULL)
		ieee80211_dev_kfree_skb_list(&bf->bf_skb);

	return bf;
}

