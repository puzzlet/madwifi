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
#include "if_ath_debug.h"
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
#include <linux/bitops.h>
#include <linux/types.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/if_arp.h>
#include <linux/rtnetlink.h>
#include <linux/time.h>
#include <asm/uaccess.h>

#include "if_ethersubr.h"		/* for ETHER_IS_MULTICAST */
#include "if_media.h"
#include "if_llc.h"

#include <net80211/ieee80211_radiotap.h>
#include <net80211/ieee80211_var.h>
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
#include "if_ath_radar.h"

#ifdef ATH_TX99_DIAG
#include "ath_tx99.h"
#endif

#include "ah_os.h"

#ifndef MAX
# define MAX(a, b)	(((a) > (b))? (a) : (b))
#endif
#ifndef MIN
# define MIN(a, b)	(((a) < (b))? (a) : (b))
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
static int ath_reset(struct net_device *);
static void ath_fatal_tasklet(TQUEUE_ARG);
static void ath_rxorn_tasklet(TQUEUE_ARG);
static void ath_bmiss_tasklet(TQUEUE_ARG);
static void ath_bstuck_tasklet(TQUEUE_ARG);
static int ath_stop_locked(struct net_device *);
static int ath_stop(struct net_device *);
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
static void ath_setctstimeout(struct ath_softc *);
static void ath_setacktimeout(struct ath_softc *);
static void ath_updateslot(struct net_device *);
static int ath_beaconq_setup(struct ath_softc *);
static int ath_beacon_alloc(struct ath_softc *, struct ieee80211_node *);
#ifdef ATH_SUPERG_DYNTURBO
static void ath_beacon_dturbo_update(struct ieee80211vap *, int *, u_int8_t);
static void ath_beacon_dturbo_config(struct ieee80211vap *, u_int32_t);
static void ath_turbo_switch_mode(unsigned long);
static int ath_check_beacon_done(struct ath_softc *);
#endif
static void ath_beacon_send(struct ath_softc *, int *, uint64_t hw_tsf);
static void ath_beacon_return(struct ath_softc *, struct ath_buf *);
static void ath_beacon_free(struct ath_softc *);
static void ath_beacon_config(struct ath_softc *, struct ieee80211vap *);
static int ath_desc_alloc(struct ath_softc *);
static void ath_desc_free(struct ath_softc *);

static struct ieee80211_node *ath_node_alloc(struct ieee80211vap *);
static void ath_node_cleanup(struct ieee80211_node *);
static void ath_node_free(struct ieee80211_node *);

static u_int8_t ath_node_getrssi(const struct ieee80211_node *);
static struct sk_buff *ath_rxbuf_take_skb(struct ath_softc *, struct ath_buf *);
static int ath_rxbuf_init(struct ath_softc *, struct ath_buf *);
static int ath_recv_mgmt(struct ieee80211vap *, struct ieee80211_node *,
	struct sk_buff *, int, int, u_int64_t);
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
static void ath_tx_txqaddbuf(struct ath_softc *, struct ieee80211_node *,
	struct ath_txq *, struct ath_buf *, int);
static void ath_stoprecv(struct ath_softc *);
static int ath_startrecv(struct ath_softc *);
static void ath_flushrecv(struct ath_softc *);
static void ath_chan_change(struct ath_softc *, struct ieee80211_channel *);
static void ath_calibrate(unsigned long);
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
		unsigned int address, unsigned int *value);
static unsigned int ath_write_register(struct ieee80211com *ic, 
		unsigned int address, unsigned int value);
static void ath_ar5212_registers_dump(struct ath_softc *sc);
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
static const char *ath_get_hal_status_desc(HAL_STATUS status);
static int ath_rcv_dev_event(struct notifier_block *, unsigned long, void *);

static void ath_return_txbuf(struct ath_softc *sc, struct ath_buf **buf);
static void ath_return_txbuf_locked(struct ath_softc *sc, struct ath_buf **buf);

static void ath_return_txbuf_list(struct ath_softc *sc, ath_bufhead *bfhead);
static void ath_return_txbuf_list_locked(struct ath_softc *sc, ath_bufhead *bfhead);

static struct ath_buf *cleanup_ath_buf(struct ath_softc *sc,
		struct ath_buf *buf, int direction);

/* Regulatory agency testing - continuous transmit support */
static void txcont_on(struct ieee80211com *ic);
static void txcont_off(struct ieee80211com *ic);

static int ath_get_txcont(struct ieee80211com *);
static void ath_set_txcont(struct ieee80211com *, int);

static int ath_get_txcont_power(struct ieee80211com *);
static void ath_set_txcont_power(struct ieee80211com *, unsigned int);

static unsigned int ath_get_txcont_rate(struct ieee80211com *);
static void ath_set_txcont_rate(struct ieee80211com *ic, unsigned int new_rate);

/* 802.11h DFS support functions */
static void ath_dfs_cac_completed(unsigned long);
static void ath_interrupt_dfs_cac(struct ath_softc *sc, const char *reason);

static inline int ath_chan_unavail(struct ath_softc *sc);

#define ath_cac_running_dbgmsg(_sc)	\
	_ath_cac_running_dbgmsg((_sc), __func__)
#define ath_chan_unavail_dbgmsg(_sc)	\
	_ath_chan_unavail_dbgmsg((_sc), __func__)
static inline int _ath_cac_running_dbgmsg(struct ath_softc *sc,
		const char *func);
static inline int _ath_chan_unavail_dbgmsg(struct ath_softc *sc,
		const char *func);

/* 802.11h DFS testing functions */
static int ath_get_dfs_testmode(struct ieee80211com *);
static void ath_set_dfs_testmode(struct ieee80211com *, int);

static unsigned int ath_get_dfs_excl_period(struct ieee80211com *);
static void ath_set_dfs_excl_period(struct ieee80211com *, 
		unsigned int seconds);

static unsigned int ath_get_dfs_cac_time(struct ieee80211com *);
static void ath_set_dfs_cac_time(struct ieee80211com *, unsigned int seconds);

static unsigned int ath_test_radar(struct ieee80211com *);
static unsigned int ath_dump_hal_map(struct ieee80211com *ic);

static u_int32_t ath_get_clamped_maxtxpower(struct ath_softc *sc);
static u_int32_t ath_set_clamped_maxtxpower(struct ath_softc *sc, 
		u_int32_t new_clamped_maxtxpower);

static void ath_scanbufs(struct ath_softc *sc);
static int ath_debug_iwpriv(struct ieee80211com *ic, 
		unsigned int param, unsigned int value);

static u_int32_t ath_get_real_maxtxpower(struct ath_softc *sc);
static int ath_txq_check(struct ath_softc *sc, struct ath_txq *txq, const char *msg);

static int ath_countrycode = CTRY_DEFAULT;	/* country code */
static int ath_outdoor = AH_FALSE;		/* enable outdoor use */
static int ath_xchanmode = AH_TRUE;		/* enable extended channels */
static int ath_maxvaps = ATH_MAXVAPS_DEFAULT;   /* set default maximum vaps */
static char *autocreate = "sta";
static char *ratectl = DEF_RATE_CTL;
static int rfkill = 0;
static int hal_tpc = 0;
static int intmit = 0;
static int countrycode = CTRY_DEFAULT;
static int maxvaps = ATH_MAXVAPS_DEFAULT;
static int outdoor = 0;
static int xchanmode = 0;
static int beacon_cal = 1;

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
MODULE_PARM(beacon_cal, "i");
MODULE_PARM(countrycode, "i");
MODULE_PARM(maxvaps, "i");
MODULE_PARM(outdoor, "i");
MODULE_PARM(xchanmode, "i");
MODULE_PARM(rfkill, "i");
#ifdef ATH_CAP_TPC
MODULE_PARM(hal_tpc, "i");
#endif
MODULE_PARM(autocreate, "s");
MODULE_PARM(ratectl, "s");
MODULE_PARM(intmit, "i");
#else
#include <linux/moduleparam.h>
module_param(beacon_cal, int, 0600);
module_param(countrycode, int, 0600);
module_param(maxvaps, int, 0600);
module_param(outdoor, int, 0600);
module_param(xchanmode, int, 0600);
module_param(rfkill, int, 0600);
#ifdef ATH_CAP_TPC
module_param(hal_tpc, int, 0600);
#endif
module_param(autocreate, charp, 0600);
module_param(ratectl, charp, 0600);
module_param(intmit, int, 0600);
#endif
MODULE_PARM_DESC(countrycode, "Override default country code.  Default is 0.");
MODULE_PARM_DESC(maxvaps, "Maximum VAPs.  Default is 4.");
MODULE_PARM_DESC(outdoor, "Enable/disable outdoor use.  Default is 0.");
MODULE_PARM_DESC(xchanmode, "Enable/disable extended channel mode.");
MODULE_PARM_DESC(rfkill, "Enable/disable RFKILL capability.  Default is 0.");
#ifdef ATH_CAP_TPC
MODULE_PARM_DESC(hal_tpc, "Disables manual per-packet transmit power control and "
		"lets this be managed by the HAL.  Default is OFF.");
#endif
MODULE_PARM_DESC(autocreate, "Create ath device in "
		"[sta|ap|wds|adhoc|ahdemo|monitor] mode. defaults to sta, use "
		"'none' to disable");
MODULE_PARM_DESC(ratectl, "Rate control algorithm [amrr|minstrel|onoe|sample], "
		"defaults to '" DEF_RATE_CTL "'");
MODULE_PARM_DESC(intmit, "Enable interference mitigation by default.  Default is 0.");

static int	ath_debug = 0;
#ifdef AR_DEBUG
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,52))
MODULE_PARM(ath_debug, "i");
#else
module_param(ath_debug, int, 0600);
#endif
MODULE_PARM_DESC(ath_debug, "Load-time driver debug output enable");
static void ath_printrxbuf(const struct ath_buf *, int);
static void ath_printtxbuf(const struct ath_buf *, int);
#endif /* defined(AR_DEBUG) */

static int	ieee80211_debug = 0;
#ifdef AR_DEBUG
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,52))
MODULE_PARM(ieee80211_debug, "i");
#else
module_param(ieee80211_debug, int, 0600);
#endif
MODULE_PARM_DESC(ieee80211_debug, "Load-time 802.11 debug output enable");
#endif /* defined(AR_DEBUG) */

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
	((bssid_mask)[0] &= ~(((ath_maxvaps-1) << 2) | 0x02))
#define ATH_GET_VAP_ID(bssid)                   ((bssid)[0] >> 2)
#define ATH_SET_VAP_BSSID(bssid, id)					\
		do {							\
			if (id)						\
				(bssid)[0] |= (((id) << 2) | 0x02);	\
		} while (0)

/* Initialize ath_softc structure */

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

	/* Allocate space for dynamically determined maximum VAP count */
	sc->sc_bslot = 
		kzalloc(ath_maxvaps * sizeof(struct ieee80211vap), GFP_KERNEL);

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
	ATH_BBUF_LOCK_INIT(sc);
	ATH_GBUF_LOCK_INIT(sc);

	atomic_set(&sc->sc_txbuf_counter, 0);

	ATH_INIT_TQUEUE(&sc->sc_rxtq,		ath_rx_tasklet,		dev);
	ATH_INIT_TQUEUE(&sc->sc_txtq,		ath_tx_tasklet,		dev);
	ATH_INIT_TQUEUE(&sc->sc_bmisstq,	ath_bmiss_tasklet,	dev);
	ATH_INIT_TQUEUE(&sc->sc_bstucktq,	ath_bstuck_tasklet,	dev);
	ATH_INIT_TQUEUE(&sc->sc_rxorntq,	ath_rxorn_tasklet,	dev);
	ATH_INIT_TQUEUE(&sc->sc_fataltq,	ath_fatal_tasklet,	dev);

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
	 * TPC support can be done either with a global cap or
	 * per-packet support.  The latter is not available on
	 * all parts.  We're a bit pedantic here as all parts
	 * support a global cap.
	 */
#ifdef ATH_CAP_TPC
	sc->sc_hastpc = ath_hal_hastpc(ah);
	if (hal_tpc && !sc->sc_hastpc) {
		WPRINTF(sc, "HAL managed transmit power control (TPC) "
				"was requested, but is not "
				"supported by the HAL.\n");
		hal_tpc = 0;
	}
	DPRINTF(sc, ATH_DEBUG_ANY, "HAL managed transmit power control (TPC) "
		"%s.\n", hal_tpc ? "enabled" : "disabled");
	ath_hal_settpc(ah, hal_tpc);
#else
	sc->sc_hastpc = 0;
	hal_tpc = 0; /* TPC is always zero, when compiled without ATH_CAP_TPC */
#endif
	/*
	 * Init ic_caps prior to queue init, since WME cap setting
	 * depends on queue setup.
	 */
	ic->ic_caps = 0;

	if (ath_hal_hastxpowlimit(ah)) {
		ic->ic_caps |= IEEE80211_C_TXPMGT;
	}
	/* Interference mitigation/ambient noise immunity (ANI).
	 * In modes other than HAL_M_STA, it causes receive sensitivity
	 * problems for OFDM. */
	sc->sc_hasintmit = ath_hal_hasintmit(ah);
	sc->sc_useintmit = (intmit && sc->sc_hasintmit);
	if (!sc->sc_hasintmit && intmit) {
		WPRINTF(sc, "Interference mitigation was requested, but is not"
				"supported by the HAL/hardware.\n");
		intmit = 0; /* Stop use in future ath_attach(). */
	}
	else {
		ath_hal_setintmit(ah, sc->sc_useintmit);
		DPRINTF(sc, ATH_DEBUG_ANY, "Interference mitigation is "
			"supported.  Currently %s.\n",
			(sc->sc_useintmit ? "enabled" : "disabled"));
	}

	sc->sc_dmasize_stomp = 0;

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
	sc->sc_needmib = ath_hal_hwphycounters(ah) &&
		sc->sc_hasintmit &&
		sc->sc_useintmit;

	/*
	 * Get the hardware key cache size.
	 */
	sc->sc_keymax = ath_hal_keycachesize(ah);
	if (sc->sc_keymax > ATH_KEYMAX) {
		WPRINTF(sc, "Using only %u entries in %u key cache.\n",
			ATH_KEYMAX, sc->sc_keymax);
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
	if (maxvaps != -1) {
		ath_maxvaps = maxvaps;
		if (ath_maxvaps < ATH_MAXVAPS_MIN)
			ath_maxvaps = ATH_MAXVAPS_MIN;
		else if (ath_maxvaps > ATH_MAXVAPS_MAX)
			ath_maxvaps = ATH_MAXVAPS_MAX;
	}
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

	IPRINTF(sc, "Switching rfkill capability %s.\n",
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
		EPRINTF(sc, "Failed to allocate descriptors. Error %d.\n",
			error);
		goto bad;
	}

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
		EPRINTF(sc, "Unable to setup a beacon xmit queue!\n");
		error = EIO;
		goto bad2;
	}
	/* CAB: Crap After Beacon - a beacon gated queue */
	sc->sc_cabq = ath_txq_setup(sc, HAL_TX_QUEUE_CAB, 0);
	if (sc->sc_cabq == NULL) {
		EPRINTF(sc, "Unable to setup CAB transmit queue!\n");
		error = EIO;
		goto bad2;
	}
	/* NB: ensure BK queue is the lowest priority h/w queue */
	if (!ath_tx_setup(sc, WME_AC_BK, HAL_WME_AC_BK)) {
		EPRINTF(sc, "Unable to setup transmit queue for %s traffic!\n",
			ieee80211_wme_acnames[WME_AC_BK]);
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
				"Unable to setup UAPSD transmit queue!\n");
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
	sc->sc_calinterval_sec = ATH_SHORT_CALINTERVAL_SECS;
	sc->sc_lastcal = jiffies;

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

	/* initialize DFS related variables */
	sc->sc_dfswait = 0;
	sc->sc_dfs_cac = 0;
	sc->sc_dfs_testmode = 0;

	init_timer(&sc->sc_dfs_cac_timer);
	sc->sc_dfs_cac_timer.function = 
		ath_dfs_cac_completed; 
	sc->sc_dfs_cac_timer.data = (unsigned long) sc ; 

	sc->sc_dfs_cac_period = ATH_DFS_WAIT_MIN_PERIOD;
	sc->sc_dfs_excl_period = ATH_DFS_AVOID_MIN_PERIOD;

	/* initialize radar stuff */
	ath_rp_init(sc);

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
	dev->tx_queue_len = ATH_TXBUF - ATH_TXBUF_MGT_RESERVED;
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
	sc->sc_debug = (ath_debug & ~ATH_DEBUG_GLOBAL);
	sc->sc_default_ieee80211_debug = ieee80211_debug;

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

	/* If hardware has multicast key search capabilities, disable it. It
	 * is not currently supported by the driver. It seems we can properly
	 * enable things in the HAL, however multicast packets are still
	 * decoded using keys located at index 0..3, so current code is no
	 * longer working */
	if (ath_hal_hasmcastkeysearch(ah)) {
		ath_hal_setmcastkeysearch(ah, AH_FALSE);
	}
	sc->sc_mcastkey  = ath_hal_getmcastkeysearch(ah);

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

	/* Query the HAL about antenna support
	 * Enable RX fast diversity if HAL has support. */
	sc->sc_hasdiversity = sc->sc_diversity = !!ath_hal_hasdiversity(ah);
	ath_hal_setdiversity(ah, sc->sc_diversity);

	sc->sc_rxantenna = ath_hal_getdefantenna(ah);
	sc->sc_txantenna = 0;	/* default to auto-selection */

	/*
	 * Not all chips have the VEOL support we want to
	 * use with IBSS beacons; check here for it.
	 */
	sc->sc_hasveol = ath_hal_hasveol(ah);

	sc->sc_txintrperiod = ATH_TXQ_INTR_PERIOD;

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
	ic->ic_node_alloc = ath_node_alloc;
	sc->sc_node_free = ic->ic_node_free;
	ic->ic_node_free = ath_node_free;
	sc->sc_node_cleanup = ic->ic_node_cleanup;
	ic->ic_node_cleanup = ath_node_cleanup;

	ic->ic_node_getrssi = ath_node_getrssi;
#ifdef ATH_SUPERG_XR
	ic->ic_node_move_data = ath_node_move_data;
#endif
	sc->sc_recv_mgmt = ic->ic_recv_mgmt;
	ic->ic_recv_mgmt = ath_recv_mgmt;

	ic->ic_vap_create = ath_vap_create;
	ic->ic_vap_delete = ath_vap_delete;

	ic->ic_test_radar		= ath_test_radar;
	ic->ic_dump_hal_map		= ath_dump_hal_map;

	ic->ic_set_dfs_testmode		= ath_set_dfs_testmode;
	ic->ic_get_dfs_testmode		= ath_get_dfs_testmode;

	ic->ic_set_txcont		= ath_set_txcont;
	ic->ic_get_txcont		= ath_get_txcont;

	ic->ic_set_txcont_power		= ath_set_txcont_power;
	ic->ic_get_txcont_power		= ath_get_txcont_power;

	ic->ic_set_txcont_rate		= ath_set_txcont_rate;
	ic->ic_get_txcont_rate		= ath_get_txcont_rate;

	ic->ic_scan_start		= ath_scan_start;
	ic->ic_scan_end			= ath_scan_end;
	ic->ic_set_channel		= ath_set_channel;

#ifdef ATH_REVERSE_ENGINEERING
	ic->ic_read_register		= ath_read_register;
	ic->ic_write_register		= ath_write_register;
	ic->ic_registers_dump		= ath_registers_dump;
	ic->ic_registers_dump_delta	= ath_registers_dump_delta;
	ic->ic_registers_mark		= ath_registers_mark;
#endif /* #ifdef ATH_REVERSE_ENGINEERING */
	ic->ic_debug_ath_iwpriv		= ath_debug_iwpriv;

	ic->ic_set_coverageclass = ath_set_coverageclass;
	ic->ic_mhz2ieee = ath_mhz2ieee;

	/* DFS radar avoidance channel availability check time (in seconds) */
	ic->ic_set_dfs_cac_time = ath_set_dfs_cac_time;
	ic->ic_get_dfs_cac_time = ath_get_dfs_cac_time;

	/* DFS radar avoidance channel use delay */
	ic->ic_set_dfs_excl_period = ath_set_dfs_excl_period;
	ic->ic_get_dfs_excl_period = ath_get_dfs_excl_period;

	if (register_netdev(dev)) {
		EPRINTF(sc, "Unable to register device\n");
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
	IPRINTF(sc, "TX99 support enabled\n");
#endif
	sc->sc_invalid = 0;

	if (autocreate) {
		if (!strcmp(autocreate, "none") || !strlen(autocreate))
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
			DPRINTF(sc, ATH_DEBUG_ANY,
				"Unknown autocreate mode: %s\n", autocreate);
			autocreatemode = -1;
		}
	}

	if (autocreatemode != -1) {
		rtnl_lock();
		vap = ieee80211_create_vap(ic, "ath%d", dev,
				autocreatemode, 0);
		rtnl_unlock();
		if (vap == NULL)
			EPRINTF(sc, "Autocreation of %s VAP failed.", autocreate);
	}

	sc->sc_rp_lasttsf	= 0;
	sc->sc_last_tsf		= 0;

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
	ATH_RXBUF_LOCK_DESTROY(sc);
	ATH_BBUF_LOCK_DESTROY(sc);
	ATH_GBUF_LOCK_DESTROY(sc);
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
	DPRINTF(sc, ATH_DEBUG_ANY, "flags=%x\n", dev->flags);
	ath_stop(dev);

	ath_hal_setpower(sc->sc_ah, HAL_PM_AWAKE, AH_TRUE);
	/* Flush the radar task if it's scheduled */
	if (sc->sc_dfs_cac)
		flush_scheduled_work();

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
	kfree(sc->sc_bslot);
	sc->sc_bslot = NULL;

	/* free radar pulse stuff */
	ath_rp_done(sc);

	ath_dynamic_sysctl_unregister(sc);
	ATH_LOCK_DESTROY(sc);
	ATH_TXBUF_LOCK_DESTROY(sc);
	ATH_RXBUF_LOCK_DESTROY(sc);
	ATH_BBUF_LOCK_DESTROY(sc);
	ATH_GBUF_LOCK_DESTROY(sc);
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
		/* If we already have an AP VAP, we can still add a station VAP
		 * but we must not attempt to re-use the hardware beacon timers
		 * since the AP is already using them, and we must stay in AP 
		 * opmode. */
		if (sc->sc_nvaps != 0) {
			flags |= IEEE80211_USE_SW_BEACON_TIMERS;
			sc->sc_nostabeacons = 1;
			ic_opmode = IEEE80211_M_HOSTAP;	/* Run with chip in AP mode */
		} else {
			ic_opmode = opmode;
		}
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

	if (sc->sc_nvaps >= ath_maxvaps) {
		EPRINTF(sc, "Too many virtual APs (%d already exist).\n", 
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

	if (flags & IEEE80211_CLONE_BSSID) {
		if (sc->sc_hasbmask) {
			struct ieee80211vap *v;
			uint64_t id_mask = 0;
			unsigned int id;

			/* Hardware supports the BSSID mask and a unique
			 * BSSID was requested.  Assign a new MAC address
			 * and expand our BSSID mask to cover the active
			 * virtual APs with distinct addresses. */
			/* Do a full search to mark all the allocated VAPs. */
			TAILQ_FOREACH(v, &ic->ic_vaps, iv_next)
				id_mask |= (1 << ATH_GET_VAP_ID(v->iv_myaddr));

			for (id = 1; id < ath_maxvaps; id++) {
				/* Get the first available slot. */
				if ((id_mask & (1 << id)) == 0) {
					ATH_SET_VAP_BSSID(vap->iv_myaddr, id);
					ATH_SET_VAP_BSSID(vap->iv_bssid, id);
					break;
				}
			}
		} else {
			EPRINTF(sc, "Unique BSSID requested on HW that does"
				"does not support the necessary features.");
		}
	}
	avp->av_bslot = -1;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)
	atomic_set(&avp->av_beacon_alloc, 0);
#else	
	clear_bit(0, &avp->av_beacon_alloc);
#endif
	STAILQ_INIT(&avp->av_mcastq.axq_q);
	ATH_TXQ_LOCK_INIT(&avp->av_mcastq);
	if (IEEE80211_IS_MODE_BEACON(opmode)) {
		unsigned int slot;
		/* Allocate beacon state for hostap/ibss.  We know
		 * a buffer is available because of the check above. */
		avp->av_bcbuf = STAILQ_FIRST(&sc->sc_bbuf);
		STAILQ_REMOVE_HEAD(&sc->sc_bbuf, bf_list);
		
		/* Assign the VAP to a beacon xmit slot.  As
		 * above, this cannot fail to find one. */
		avp->av_bslot = 0;
		for (slot = 0; slot < ath_maxvaps; slot++)
			if (sc->sc_bslot[slot] == NULL) {
				/* XXX: Hack, space out slots to better
				 * deal with misses. */
				if (slot + 1 < ath_maxvaps &&
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

		if ((opmode == IEEE80211_M_HOSTAP) && (sc->sc_hastsfadd)) {
			/*
			 * Multiple VAPs are to transmit beacons and we
			 * have h/w support for TSF adjusting; enable use
			 * of staggered beacons.
			 */
			/* XXX check for beacon interval too small */
			if (ath_maxvaps > 4) {
				DPRINTF(sc, ATH_DEBUG_BEACON, 
						"Staggered beacons are not "
						"possible with maxvaps set "
						"to %d.\n", ath_maxvaps);
				sc->sc_stagbeacons = 0;
			} else {
				sc->sc_stagbeacons = 1;
			}
		}
		DPRINTF(sc, ATH_DEBUG_BEACON, "sc->sc_stagbeacons %sabled\n", 
				(sc->sc_stagbeacons ? "en" : "dis"));
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
			EPRINTF(sc, "DMA setup failed\n");
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
			EPRINTF(sc, "Unable to start receive logic.\n");
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
			EPRINTF(sc, "Unable to start receive logic.\n");
		if (sc->sc_beacons)
			ath_beacon_config(sc, NULL);	/* restart beacons */
		ath_hal_intrset(ah, sc->sc_imask);
	}
}

void
ath_suspend(struct net_device *dev)
{
	DPRINTF(((struct ath_softc *)dev->priv), ATH_DEBUG_ANY, "flags=%x\n", dev->flags);
	ath_stop(dev);
}

void
ath_resume(struct net_device *dev)
{
	DPRINTF(((struct ath_softc *)dev->priv), ATH_DEBUG_ANY, "flags=%x\n", dev->flags);
	ath_init(dev);
}

/* NB: Int. mit. was not implemented so that it could be enabled/disabled,
 * and actually in 0.9.30.13 HAL it really can't even be disabled because
 * it will start adjusting registers even when we turn off the capability
 * in the HAL.
 *
 * NB: This helper function basically clobbers all the related registers
 * if we have disabled int. mit. cap, allowing us to turn it on and off and
 * work around the bug preventing it from being disabled. */
static inline void ath_override_intmit_if_disabled(struct ath_softc *sc)
{
	/* Restore int. mit. registers if they were turned off. */
	if (sc->sc_hasintmit && !sc->sc_useintmit)
		ath_hal_restore_default_intmit(sc->sc_ah);
	/* Sanity check... remove later. */
	if (!sc->sc_useintmit) {
		ath_hal_verify_default_intmit(sc->sc_ah);
		/* If we don't have int. mit. and we don't have DFS on channel,
		 * it is safe to filter error packets. */
		if (!ath_radar_is_dfs_required(sc, &sc->sc_curchan)) {
			ath_hal_setrxfilter(sc->sc_ah,
				ath_hal_getrxfilter(sc->sc_ah) & 
				~HAL_RX_FILTER_PHYERR);
		}
	}
	else {
		/* Make sure that we have errors in RX filter because ANI needs
		 * them. */
		ath_hal_setrxfilter(sc->sc_ah, 
			ath_hal_getrxfilter(sc->sc_ah) | HAL_RX_FILTER_PHYERR);
	}
}

static inline HAL_BOOL ath_hw_puttxbuf(struct ath_softc *sc, u_int qnum,
				u_int32_t txdp, const char *msg)
{
	HAL_BOOL result;
	u_int32_t txdp_2;

	result = ath_hal_puttxbuf(sc->sc_ah, qnum, txdp);
	if (!result) {
		DPRINTF(sc, ATH_DEBUG_WATCHDOG,
			"TXQ%d: BUG failed to set TXDP:%08x\n",
			qnum, txdp);
		ath_txq_check(sc, &sc->sc_txq[qnum], msg);
		return result;
	}

	txdp_2 = ath_hal_gettxbuf(sc->sc_ah, qnum);
	if (txdp_2 != txdp) {
		DPRINTF(sc, ATH_DEBUG_WATCHDOG,
			"TXQ%d: BUG failed to set TXDP:%08x (is %08x)\n",
			qnum, txdp, txdp_2);
		ath_txq_check(sc, &sc->sc_txq[qnum], msg);
	}

	return result;
}

/* If channel change is successful, sc->sc_curchan is updated with the new
 * channel */

static HAL_BOOL ath_hw_reset(struct ath_softc *sc, HAL_OPMODE opmode,
		HAL_CHANNEL *channel, HAL_BOOL bChannelChange,
		HAL_STATUS *status)
{
	HAL_BOOL ret;
	unsigned long __axq_lockflags[HAL_NUM_TX_QUEUES];
	struct ath_txq * txq;
	int i;
 	u_int8_t old_privFlags = sc->sc_curchan.privFlags;

	/* ath_hal_reset() resets all TXDP pointers, so we need to
	 * lock all TXQ to avoid race condition with
	 * ath_tx_txqaddbuf() and ath_tx_processq() */

	for (i = 0; i < HAL_NUM_TX_QUEUES; i++) {
		if (ATH_TXQ_SETUP(sc, i)) {
			txq = &sc->sc_txq[i];
			spin_lock_irqsave(&txq->axq_lock, __axq_lockflags[i]);
		}
	}

	ret = ath_hal_reset(sc->sc_ah, sc->sc_opmode, channel, bChannelChange, status);

	/* Restore all TXDP pointers, if appropriate, and unlock in
	 * the reverse order we locked */
	for (i = HAL_NUM_TX_QUEUES - 1; i >= 0; i--) {
		/* only take care of configured TXQ */
		if (ATH_TXQ_SETUP(sc, i)) {
			struct ath_buf *bf;
			u_int32_t txdp;

			txq = &sc->sc_txq[i];

			/* Check that TXDP is NULL */
			txdp = ath_hal_gettxbuf(sc->sc_ah, txq->axq_qnum);
			if (txdp != 0) {
				DPRINTF(sc, ATH_DEBUG_WATCHDOG,
						"TXQ%d: BUG TXDP:%08x is "
						"not NULL\n",
						txq->axq_qnum, txdp);
			}

			/* We restore TXDP to the first "In Progress" TX
			 * descriptor. We skip "Done" TX descriptors in
			 * order to avoid sending duplicate packets */
			STAILQ_FOREACH(bf, &txq->axq_q, bf_list) {
				if (ath_hal_txprocdesc(sc->sc_ah,
						       bf->bf_desc,
					&bf->bf_dsstatus.ds_txstat) ==
				    HAL_EINPROGRESS) {
					DPRINTF(sc, ATH_DEBUG_WATCHDOG,
						"TXQ%d: restoring"
						" TXDP:%08llx\n",
 						txq->axq_qnum,
 						(u_int64_t)bf->bf_daddr);
					ath_hw_puttxbuf(sc, txq->axq_qnum,
							bf->bf_daddr,
							__func__);
					ath_hal_txstart(sc->sc_ah,
							txq->axq_qnum);
				}
			}
			spin_unlock_irqrestore(&txq->axq_lock,
					__axq_lockflags[i]);
		}
	}

	/* On failure, we return immediately */
	if (!ret)
		return ret;

 	/* Do the same as in ath_getchannels() */
 	ath_radar_correct_dfs_flags(sc, channel);
 
 	/* Restore CHANNEL_DFS_CLEAR and CHANNEL_INTERFERENCE flags */
#define CHANNEL_DFS_FLAGS	(CHANNEL_DFS_CLEAR|CHANNEL_INTERFERENCE)
	channel->privFlags = (channel->privFlags & ~CHANNEL_DFS_FLAGS) |
 		(old_privFlags & CHANNEL_DFS_FLAGS);
 
	/* On success, we update sc->sc_curchan which can be needed by other
	 * functions below , like ath_radar_update() at least */
	sc->sc_curchan = *channel;

#ifdef ATH_CAP_TPC
	if (sc->sc_hastpc && (hal_tpc != ath_hal_gettpc(sc->sc_ah))) {
		EPRINTF(sc, "TPC HAL capability out of sync.  Got %d!\n",
				ath_hal_gettpc(sc->sc_ah));
		ath_hal_settpc(sc->sc_ah, hal_tpc);
	}
#endif
#if 0 /* Setting via HAL does not work, so it is done manually below. */
	if (sc->sc_hasintmit)
		ath_hal_setintmit(sc->sc_ah, sc->sc_useintmit);
#endif
	ath_override_intmit_if_disabled(sc);
	if (sc->sc_dmasize_stomp)
		ath_hal_set_dmasize_pcie(sc->sc_ah);
	if (sc->sc_softled)
		ath_hal_gpioCfgOutput(sc->sc_ah, sc->sc_ledpin);
	ath_update_txpow(sc);		/* Update TX power state. */
	ath_hal_setdiversity(sc->sc_ah, sc->sc_diversity);
	ath_setdefantenna(sc, sc->sc_rxantenna);
	/* XXX: Any other clobbered features? */

	ath_radar_update(sc);
	ath_rp_flush(sc);

	return ret;
}

/* Returns true if we can transmit any frames : this is not the case if :
 * - we are on a DFS channel
 * - 802.11h is enabled
 * - Channel Availability Check is not done or a radar has been detected
 */
static int
ath_chan_unavail(struct ath_softc *sc)
{
	return sc->sc_dfs_cac ||
		((sc->sc_curchan.privFlags & CHANNEL_DFS) &&
		 (sc->sc_curchan.privFlags & CHANNEL_INTERFERENCE));
}

static inline int
_ath_cac_running_dbgmsg(struct ath_softc *sc, const char *func)
{
	int b = sc->sc_dfs_cac;
	if (b)
		DPRINTF(sc, ATH_DEBUG_DOTH,
				"%s: Invoked a transmit function during DFS "
				"channel availability check!\n", func);
	return b;
}

static inline int
_ath_chan_unavail_dbgmsg(struct ath_softc *sc, const char *func)
{
	int b = ath_chan_unavail(sc);
	if (b)
		DPRINTF(sc, ATH_DEBUG_DOTH,
				"%s: Invoked a transmit function during DFS "
				"channel availability check OR while radar "
				"interference is detected!\n", func);
	return b;
}


/* Extend 15-bit timestamp from RX descriptor to a full 64-bit TSF using the
 * provided hardware TSF. The result is the closest value relative to hardware
 * TSF. */

/* NB: Not all chipsets return the same precision rstamp */
static __inline u_int64_t
ath_extend_tsf(u_int64_t tsf, u_int32_t rstamp)
{
#define TSTAMP_RX_MASK  0x7fff

	u_int64_t result;

	result = (tsf & ~TSTAMP_RX_MASK) | rstamp;
	if (result > tsf) {
		if ((result - tsf) > (TSTAMP_RX_MASK / 2))
			result -= (TSTAMP_RX_MASK + 1);
	} else {
		if ((tsf - result) > (TSTAMP_RX_MASK / 2))
			result += (TSTAMP_RX_MASK + 1);
	}

	return result;
}

/* Swap transmit descriptor pointer for HW. */
static __inline u_int32_t
ath_ds_link_swap(u_int32_t dp)
{
#ifdef AH_NEED_DESC_SWAP
	return swab32(dp);
#else
	return (dp);
#endif
}

static __inline u_int32_t
ath_get_last_ds_link(struct ath_txq *txq)
{
	struct ath_desc *ds = ath_txq_last_desc(txq);
	return (ds ? ds->ds_link : 0);
}

static void
ath_intr_process_rx_descriptors(struct ath_softc *sc, int *pneedmark, u_int64_t hw_tsf)
{
	struct ath_hal *ah = sc->sc_ah;
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
	 * We collect the first hw_tsf in the ath_intr (the interrupt handler)
	 * so it can be passed to some helper functions... later in this 
	 * function, however, we read it again to perform rollover adjustments.
	 *
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

	ATH_RXBUF_LOCK_IRQ(sc);
	if (sc->sc_rxbufcur == NULL)
		sc->sc_rxbufcur = STAILQ_FIRST(&sc->sc_rxbuf);

	prev_rxbufcur = sc->sc_rxbufcur;
	/* FIRST PASS - PROCESS DESCRIPTORS */
	{
		struct ath_buf *bf;
		for (bf = prev_rxbufcur; bf; bf = STAILQ_NEXT(bf, bf_list)) {
			ds = bf->bf_desc;
			if (ds->ds_link == bf->bf_daddr) {
				/* NB: never process the self-linked entry at
				 * the end */
				break;
			}
			if (bf->bf_status & ATH_BUFSTATUS_RXDESC_DONE) {
				/* already processed this buffer (shouldn't
				 * occur if we change code to always process
				 * descriptors in rx intr handler - as opposed
				 * to sometimes processing in the rx tasklet) */
				continue;
			}
			skb = bf->bf_skb;
			if (skb == NULL) {
				EPRINTF(sc, "Dropping; skb is NULL in received ath_buf.\n");
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
			count ++;

			DPRINTF(sc, ATH_DEBUG_TSF,
				"rs_tstamp=%10llx count=%d\n",
				bf->bf_tsf, count);

			/* compute rollover */
			if (last_rs_tstamp > rs->rs_tstamp) {
				rollover ++;
				DPRINTF(sc, ATH_DEBUG_TSF,
					"%d rollover detected\n",
					rollover);
			}

			last_rs_tstamp = rs->rs_tstamp;

			/* Do not allow negative RSSI values */
			if (rs->rs_rssi < 0)
				rs->rs_rssi = 0;

			/* Capture noise per-interrupt, since it may change
			 * by the time the receive queue gets around to
			 * processing these buffers, and multiple interrupts
			 * may have occurred in the intervening timeframe. */
			bf->bf_channoise = ic->ic_channoise;
			if ((HAL_RXERR_PHY == rs->rs_status) &&
			    (HAL_PHYERR_RADAR == (rs->rs_phyerr & 0x1f)) &&
			    !(bf->bf_status & ATH_BUFSTATUS_RADAR_DONE) &&
			    (ic->ic_flags & IEEE80211_F_DOTH))
				check_for_radar = 1;

			/* XXX: We do not support frames spanning multiple
			 *      descriptors */
			bf->bf_status |= ATH_BUFSTATUS_RXDESC_DONE;

			if (rs->rs_status) {
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
				DPRINTF(sc, ATH_DEBUG_UAPSD, "U-APSD node (" MAC_FMT ") "
					"has invalid keycache entry\n",
					MAC_ADDR(qwh->i_addr2));
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
			if ((!!(qwh->i_fc[1] & IEEE80211_FC1_PWR_MGT) !=
			     !!(ni->ni_flags & IEEE80211_NODE_PWR_MGT))) {
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
						"Node (" MAC_FMT ") became U-APSD "
						"triggerable (%d)\n",
						MAC_ADDR(qwh->i_addr2),
						ic->ic_uapsdmaxtriggers);
				} else {
					ni->ni_flags &=
						~IEEE80211_NODE_UAPSD_TRIG;
					ic->ic_uapsdmaxtriggers--;
					DPRINTF(sc, ATH_DEBUG_UAPSD,
						"Node (" MAC_FMT ") no longer U-APSD"
						" triggerable (%d)\n",
						MAC_ADDR(qwh->i_addr2),
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

			/* If we are supposed to be not listening or
			 * transmitting, don't do uapsd triggers */
			if (!ath_cac_running_dbgmsg(sc)) {
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
					"U-APSD trigger detected for node "
					"(" MAC_FMT ") on AC %d\n",
					MAC_ADDR(ni->ni_macaddr), ac);
				if (ni->ni_flags & IEEE80211_NODE_UAPSD_SP) {
					/* have trigger, but SP in progress,
					 * so ignore */
					DPRINTF(sc, ATH_DEBUG_UAPSD,
						"SP already in progress -"
						" ignoring\n");
					continue;
				}

				/*
				 * Detect duplicate triggers and drop if so.
				 */
				frame_seq = le16toh(*(__le16 *)qwh->i_seq);
				if ((qwh->i_fc[1] & IEEE80211_FC1_RETRY) &&
				    frame_seq == ni->ni_uapsd_trigseq[ac]) {
					DPRINTF(sc, ATH_DEBUG_UAPSD,
						"Dropped dup trigger, ac %d"
						", seq %d\n",
						ac, frame_seq);
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
						"Queue empty; generating "
						"QoS NULL to send\n");
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

  
					ATH_TXQ_LINK_DESC(uapsd_xmit_q,
							STAILQ_FIRST(
							&an->an_uapsd_q));
					/* Below leaves an_uapsd_q NULL. */
					STAILQ_CONCAT(&uapsd_xmit_q->axq_q,
						      &an->an_uapsd_q);
					ath_hw_puttxbuf(sc,
						uapsd_xmit_q->axq_qnum,
						(STAILQ_FIRST(&uapsd_xmit_q->axq_q))->bf_daddr,
						__func__);

					ath_hal_txstart(sc->sc_ah,
							uapsd_xmit_q->axq_qnum);
				}
				an->an_uapsd_qdepth = 0;
				ATH_TXQ_UNLOCK_IRQ(uapsd_xmit_q);
			}
		}
		sc->sc_rxbufcur = bf;
	}

	/* SECOND PASS - FIX RX TIMESTAMPS */
	if (count > 0) {
		struct ath_buf * bf;

		hw_tsf = ath_hal_gettsf64(ah);
		if (last_rs_tstamp > (hw_tsf & TSTAMP_RX_MASK)) {
			rollover++;
			DPRINTF(sc, ATH_DEBUG_TSF,
				"%d rollover detected for hw_tsf=%10llx\n",
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
					(hw_tsf & ~TSTAMP_RX_MASK) | bf->bf_tsf;
				bf->bf_tsf -= rollover * (TSTAMP_RX_MASK + 1);

				DPRINTF(sc, ATH_DEBUG_TSF,
					"bf_tsf=%10llx hw_tsf=%10llx\n",
					bf->bf_tsf, hw_tsf);

				if (bf->bf_tsf < sc->sc_last_tsf) {
					DPRINTF(sc, ATH_DEBUG_TSF, 
						"TSF error: bf_tsf=%10llx "
						"sc_last_tsf=%10llx\n",
						bf->bf_tsf,
						sc->sc_last_tsf);
				}
				sc->sc_last_tsf = bf->bf_tsf;
			}
		}
	}

	/* Process radar after we have done everything else */
	if (check_for_radar) {
		/* Collect pulse events */
		struct ath_buf *p;
		for (p = prev_rxbufcur; p; p = STAILQ_NEXT(p, bf_list)) {
			ds = p->bf_desc;

			/* NB: never process the self-linked entry at end */
			if (ds->ds_link == p->bf_daddr)
				break;
			/* should have already been processed, above */
			if (0 == (p->bf_status & ATH_BUFSTATUS_RXDESC_DONE))
				continue;
			/* should not have already been processed for radar */
			if (p->bf_status & ATH_BUFSTATUS_RADAR_DONE)
				continue;
			skb = p->bf_skb;
			if (skb == NULL)
				continue;
			rs = &p->bf_dsstatus.ds_rxstat;
			if ((HAL_RXERR_PHY == rs->rs_status) &&
			    (HAL_PHYERR_RADAR == (rs->rs_phyerr & 0x1f)) &&
			    (0 == (p->bf_status & ATH_BUFSTATUS_RADAR_DONE))) {
				/* Sync the contents of the buffer in the case
				 * of radar errors so we will get the pulse
				 * width */
				if (rs->rs_datalen != 0) {
					bus_dma_sync_single(sc->sc_bdev,
							    p->bf_skbaddr,
							    rs->rs_datalen,
							    BUS_DMA_FROMDEVICE);
				}
				/* record the radar pulse event */
				ath_rp_record (
					sc, p->bf_tsf,
					rs->rs_rssi,
					(rs->rs_datalen ? skb->data[0] : 0),
					0 /* not simulated */);
#if 0
				DPRINTF(sc, ATH_DEBUG_DOTH,
					"RADAR PULSE channel:%u "
					"jiffies:%lu fulltsf:%llu "
					"fulltsf_high49:%llu tstamp:%u "
					"bf_tsf:%llu bf_tsf_high49:%llu "
					"bf_tsf_low15:%llu rssi:%u width:%u\n",
					sc->sc_curchan.channel,
					jiffies, p->bf_tsf,
					p->bf_tsf & ~TSTAMP_MASK,
					p->bf_tsf &  TSTAMP_MASK,
					p->bf_tsf,
					p->bf_tsf & ~TSTAMP_MASK,
					p->bf_tsf &  TSTAMP_MASK,
					rs->rs_rssi,
					skb->data[0]);
#endif
				sc->sc_rp_lasttsf = p->bf_tsf;
				p->bf_status |= ATH_BUFSTATUS_RADAR_DONE;
			}
		}
		/* radar pulses have been found,
		 * check them against known patterns */
		ATH_SCHEDULE_TQUEUE(&sc->sc_rp_tq, NULL);
	}

	/* If we got something to process, schedule rx queue to handle it */
	if (count)
		ATH_SCHEDULE_TQUEUE(&sc->sc_rxtq, pneedmark);
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
	u_int64_t hw_tsf = 0;
	HAL_INT status;
	int needmark;

	DPRINTF(sc, ATH_DEBUG_INTR, 
		"dev->flags=0x%x%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s]\n",
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
		/* The hardware is not ready/present, don't touch anything.
		 * Note this can happen early on if the IRQ is shared. */
		return IRQ_NONE;
	}
	if (!ath_hal_intrpend(ah))		/* shared irq, not for us */
		return IRQ_NONE;
	if ((dev->flags & (IFF_RUNNING | IFF_UP)) != (IFF_RUNNING | IFF_UP)) {
		DPRINTF(sc, ATH_DEBUG_INTR, "Flags=0x%x\n", dev->flags);
		ath_hal_getisr(ah, &status);	/* clear ISR */
		ath_hal_intrset(ah, 0);		/* disable further intr's */
		return IRQ_HANDLED;
	}
	needmark = 0;

	/* Figure out the reason(s) for the interrupt.  Note
	 * that the HAL returns a pseudo-ISR that may include
	 * bits we haven't explicitly enabled so we mask the
	 * value to ensure we only process bits we requested. */
	ath_hal_getisr(ah, &status);		/* NB: clears ISR too */
	DPRINTF(sc, ATH_DEBUG_INTR,
		"ISR=0x%x%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s\n", 
		status,
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
	/* As soon as we know we have a real interrupt we intend to service, 
	 * we will check to see if we need an initial hardware TSF reading. 
	 * Normally we would just populate this all the time to keep things
	 * clean, but this function (ath_hal_gettsf64) has been observed to be 
	 * VERY slow and hurting performance.  There's nothing we can do for it. */
	if (status & (HAL_INT_RX | HAL_INT_RXPHY | HAL_INT_SWBA))
		hw_tsf = ath_hal_gettsf64(ah);

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

			/* Updates sc_nexttbtt */
			vap = TAILQ_FIRST(&sc->sc_ic.ic_vaps);
			sc->sc_nexttbtt += vap->iv_bss->ni_intval;

			DPRINTF(sc, ATH_DEBUG_BEACON,
				"ath_intr HAL_INT_SWBA at "
				"tsf %10llx nexttbtt %10llx\n",
				hw_tsf, (u_int64_t)sc->sc_nexttbtt << 10);

			/* Software beacon alert--time to send a beacon.
			 * Handle beacon transmission directly; deferring
			 * this is too slow to meet timing constraints
			 * under load. */
			if (!sc->sc_dfs_cac)
				ath_beacon_send(sc, &needmark, hw_tsf);
			else {
				sc->sc_beacons = 0;
				sc->sc_imask &= ~(HAL_INT_SWBA | HAL_INT_BMISS);
				ath_hal_intrset(ah, sc->sc_imask);
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
			/* NB: Will schedule rx tasklet if necessary. */
			ath_intr_process_rx_descriptors(sc, &needmark, hw_tsf);
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
			if (!sc->sc_dfs_cac)
				ATH_SCHEDULE_TQUEUE(&sc->sc_bmisstq, &needmark);
			else {
				sc->sc_beacons = 0;
				sc->sc_imask &= ~(HAL_INT_SWBA | HAL_INT_BMISS);
				ath_hal_intrset(ah, sc->sc_imask);
			}
		}
		if (status & HAL_INT_MIB) {
			sc->sc_stats.ast_mib++;
			/* If we aren't doing interference mitigation and we get
			 * a lot of MIB events we can safely skip them.
			 * However, we must never throttle them DURING interference
			 * mitigation calibration sequence as it depends on this
			 * hook to advance the calibration sequence / alg.
			 */
			if (!sc->sc_useintmit) {
				sc->sc_imask &= ~HAL_INT_MIB;
				ath_hal_intrset(ah, sc->sc_imask);
			}

			/* Let the HAL handle the event. */
			ath_hal_mibevent(ah, &sc->sc_halstats);
			ath_override_intmit_if_disabled(sc);
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
	struct ath_softc *sc = dev->priv;

	EPRINTF(sc, "Hardware error; resetting.\n");
	ath_reset(dev);
}

static void
ath_rxorn_tasklet(TQUEUE_ARG data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ath_softc *sc = dev->priv;

	EPRINTF(sc, "Receive FIFO overrun; resetting.\n");
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
		DPRINTF(sc, ATH_DEBUG_ANY, "Beacon miss ignored\n");
	} else {
		DPRINTF(sc, ATH_DEBUG_ANY, "Too many beacon misses\n");
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

	DPRINTF(sc, ATH_DEBUG_RESET, "mode %d\n", ic->ic_opmode);

	/*
	 * Stop anything previously setup.  This is safe
	 * whether this is the first time through or not.
	 */
	ath_stop_locked(dev);

#ifdef ATH_CAP_TPC
	/* Re-enable after suspend */
	ath_hal_settpc(ah, hal_tpc);
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
	if (!ath_hw_reset(sc, sc->sc_opmode, &sc->sc_curchan, AH_FALSE, &status)) {
		EPRINTF(sc, "unable to reset hardware: '%s' (HAL status %u) "
			"(freq %u flags 0x%x)\n", 
			ath_get_hal_status_desc(status), status,
			sc->sc_curchan.channel, sc->sc_curchan.channelFlags);
		error = -EIO;
		goto done;
	}

	/*
	 * Setup the hardware after reset: the key cache
	 * is filled as needed and the receive engine is
	 * set going.  Frame transmit is handled entirely
	 * in the frame output path; there's nothing to do
	 * here except setup the interrupt mask.
	 */
	if (ath_startrecv(sc) != 0) {
		EPRINTF(sc, "Unable to start receive logic.\n");
		error = -EIO;
		goto done;
	}
	/* Enable interrupts. */
	sc->sc_imask = HAL_INT_RX | HAL_INT_TX
		| HAL_INT_RXEOL | HAL_INT_RXORN
		| HAL_INT_FATAL | HAL_INT_GLOBAL
		| (sc->sc_needmib ? HAL_INT_MIB : 0);

	/* Push changes to sc_imask to hardware */
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

	DPRINTF(sc, ATH_DEBUG_RESET, "invalid=%u flags=0x%x\n",
		sc->sc_invalid, dev->flags);

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
		if (!sc->sc_invalid) {
			del_timer_sync(&sc->sc_dfs_cac_timer);
			if (!sc->sc_beacon_cal)
				del_timer_sync(&sc->sc_cal_ch);
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

static void ath_set_beacon_cal(struct ath_softc *sc, int val)
{
	if (sc->sc_beacon_cal == !!val)
		return;

	if (val) {
		del_timer(&sc->sc_cal_ch);
	} else {
		mod_timer(&sc->sc_cal_ch, jiffies + (sc->sc_calinterval_sec * HZ));
	}
	sc->sc_beacon_cal = (val && beacon_cal);
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
	if (!ath_hw_reset(sc, sc->sc_opmode, &sc->sc_curchan, AH_TRUE, &status))
		EPRINTF(sc, "Unable to reset hardware: '%s' (HAL status %u)\n",
			ath_get_hal_status_desc(status), status);

	if (ath_startrecv(sc) != 0)	/* restart recv */
		EPRINTF(sc, "Unable to start receive logic.\n");

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
	/* Restart the reaper task, once, after each reset. */
	ATH_SCHEDULE_TQUEUE(&sc->sc_txtq, &needmark);
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

static void
ath_txq_dump(struct ath_softc *sc, struct ath_txq *txq)
{
	int j;
	struct ath_buf *bf;

	DPRINTF(sc, ATH_DEBUG_WATCHDOG, "txq:%p : axq_qnum:%u axq_depth:%d "
			"TXDP:%08x\n",
			txq, txq->axq_qnum, txq->axq_depth, 
			ath_hal_gettxbuf(sc->sc_ah, txq->axq_qnum));

	j = 0;
	STAILQ_FOREACH(bf, &txq->axq_q, bf_list) {
		HAL_STATUS status = 
			ath_hal_txprocdesc(sc->sc_ah,
					   bf->bf_desc,
					   &bf->bf_dsstatus.ds_txstat);
		DPRINTF(sc, ATH_DEBUG_WATCHDOG, "  [%3u] bf_daddr:%08llx "
			"ds_link:%08x %s\n",
			j++,
			(u_int64_t)bf->bf_daddr, bf->bf_desc->ds_link,
			status == HAL_EINPROGRESS ? "pending" : "done");
	}
}

/* Check TXDP (HW queue head) and SW queue head. This function assumes
 * that axq_lock are held (ie ATH_TXQ_LOCK_IRQ has been called) */
static int
ath_txq_check(struct ath_softc *sc, struct ath_txq *txq, const char *msg)
{
	struct ath_hal * ah = sc->sc_ah;
	struct ath_buf *bf;
	u_int32_t txdp;
	int sw_head_printed = 0;
	int hw_head_printed = 0;

	txdp = ath_hal_gettxbuf(ah, txq->axq_qnum);

	STAILQ_FOREACH(bf, &txq->axq_q, bf_list) {
		if (!sw_head_printed)
			sw_head_printed = 1;
		if (!hw_head_printed && txdp == bf->bf_daddr)
			hw_head_printed = 1;
	}

	if (sw_head_printed && !hw_head_printed) {
		DPRINTF(sc, ATH_DEBUG_WATCHDOG, "TXQ%d: BUG TXDP:%08x not "
				"in queue (%d elements) [%s]\n",
				txq->axq_qnum, txdp, txq->axq_depth, msg);
		ath_txq_dump(sc, txq);
		return 0;
	}

	return 1;
}

/*
 * Insert a buffer on a txq
 */
static __inline void
ath_tx_txqaddbuf(struct ath_softc *sc, struct ieee80211_node *ni,
	struct ath_txq *txq, struct ath_buf *bf, int framelen)
{
	struct ath_hal *ah = sc->sc_ah;
#ifdef ATH_SUPERG_FF
	struct ath_desc	*ds = bf->bf_desc;

	/* Go to the last descriptor.
	 * NB: This code assumes that the descriptors for a buf are allocated,
	 *     contiguously. This assumption is made elsewhere too. */
	ds += bf->bf_numdescff;
#endif

	if (ath_cac_running_dbgmsg(sc))
		return;

	/* Insert the frame on the outbound list and
	 * pass it on to the hardware. */
	ATH_TXQ_LOCK_IRQ(txq);
	if (ni && ni->ni_vap && txq == &ATH_VAP(ni->ni_vap)->av_mcastq) {
		/* The CAB queue is started from the SWBA handler since
		 * frames only go out on DTIM and to avoid possible races. */
		ath_hal_intrset(ah, sc->sc_imask & ~HAL_INT_SWBA);

		ATH_TXQ_LINK_DESC(txq, bf);
		ATH_TXQ_INSERT_TAIL(txq, bf, bf_list); 
		DPRINTF(sc, ATH_DEBUG_XMIT, 
				"link[%u]=%08llx (%p)\n",
				txq->axq_qnum, 
  				(u_int64_t)bf->bf_daddr, bf->bf_desc);

		/* We do not start tx on this queue as it will be done as
		 * "CAB" data at DTIM intervals. */
		ath_hal_intrset(ah, sc->sc_imask);
	} else {
		ATH_TXQ_LINK_DESC(txq, bf);
		if (!STAILQ_FIRST(&txq->axq_q)) {
			ath_hw_puttxbuf(sc, txq->axq_qnum, bf->bf_daddr, __func__);
		}
		ATH_TXQ_INSERT_TAIL(txq, bf, bf_list);
		
		DPRINTF(sc, ATH_DEBUG_TX_PROC, "UC txq [%d] depth = %d\n", 
				txq->axq_qnum, txq->axq_depth);
		DPRINTF(sc, ATH_DEBUG_XMIT,
				"link[%u] (%08x)=%08llx (%p)\n",
				txq->axq_qnum, 
				ath_get_last_ds_link(txq),
				(u_int64_t)bf->bf_daddr, bf->bf_desc);

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
	struct ieee80211_phy_params *ph = &(SKB_CB(skb)->phy); 
	const HAL_RATE_TABLE *rt;
	unsigned int pktlen, hdrlen, try0, power;
	HAL_PKT_TYPE atype;
	u_int flags;
	u_int8_t txrate;
	struct ath_txq *txq = NULL;
	struct ath_desc *ds = NULL;
	struct ieee80211_frame *wh;

	wh = (struct ieee80211_frame *)skb->data;
	try0 = ph->try[0];
	rt = sc->sc_currates;
	txrate = dot11_to_ratecode(sc, rt, ph->rate[0]);
	power = ph->power > 60 ? 60 : ph->power;
	hdrlen = ieee80211_anyhdrsize(wh);
	pktlen = skb->len + IEEE80211_CRC_LEN;

	flags = HAL_TXDESC_INTREQ | HAL_TXDESC_CLRDMASK; /* XXX needed for crypto errs */

	bf->bf_skbaddr = bus_map_single(sc->sc_bdev,
					skb->data, pktlen, BUS_DMA_TODEVICE);
	DPRINTF(sc, ATH_DEBUG_XMIT, "skb=%p [data %p len %u] skbaddr %08llx\n",
		skb, skb->data, skb->len, (u_int64_t)bf->bf_skbaddr);

	bf->bf_skb = skb;
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

	/* XXX check return value? */
	ath_hal_setuptxdesc(ah, ds,
			    pktlen,			/* packet length */
			    hdrlen,			/* header length */
			    atype,			/* Atheros packet type */
			    power,			/* txpower */
			    txrate, try0,		/* series 0 rate/tries */
			    HAL_TXKEYIX_INVALID,	/* key cache index */
			    sc->sc_txantenna,		/* antenna mode */
			    flags,			/* flags */
			    0,				/* rts/cts rate */
			    0,				/* rts/cts duration */
			    0,				/* comp icv len */
			    0,				/* comp iv len */
			    ATH_COMP_PROC_NO_COMP_NO_CCS /* comp scheme */
			   );

	if (ph->try[1]) {
		ath_hal_setupxtxdesc(sc->sc_ah, ds,
			dot11_to_ratecode(sc, rt, ph->rate[1]), ph->try[1],
			dot11_to_ratecode(sc, rt, ph->rate[2]), ph->try[2],
			dot11_to_ratecode(sc, rt, ph->rate[3]), ph->try[3]
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

	DPRINTF(sc, ATH_DEBUG_XMIT, "Q%d: %08x %08x %08x %08x %08x %08x\n",
		M_FLAG_GET(skb, M_UAPSD) ? 0 : txq->axq_qnum, 
		ds->ds_link, ds->ds_data,
		ds->ds_ctl0, ds->ds_ctl1, 
		ds->ds_hw[0], ds->ds_hw[1]);

	ath_tx_txqaddbuf(sc, NULL, txq, bf, pktlen);
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

		KASSERT(ATH_BUF_AN(bf_ff)->an_tx_ffbuf[bf_ff->bf_skb->priority],
			("no bf_ff on staging queue %p", bf_ff));
		ATH_BUF_AN(bf_ff)->an_tx_ffbuf[bf_ff->bf_skb->priority] = NULL;
		TAILQ_REMOVE(&txq->axq_stageq, bf_ff, bf_stagelist);

		ATH_TXQ_UNLOCK_IRQ(txq);

		/* encap and xmit */
		bf_ff->bf_skb = ieee80211_encap(ATH_BUF_NI(bf_ff), bf_ff->bf_skb, 
				&framecnt);
		if (bf_ff->bf_skb == NULL) {
			DPRINTF(sc, ATH_DEBUG_XMIT | ATH_DEBUG_FF,
				"Dropping; encapsulation failure\n");
			sc->sc_stats.ast_tx_encap++;
			goto bad;
		}
		pktlen = bf_ff->bf_skb->len;	/* NB: don't reference skb below */
		if (ath_tx_start(sc->sc_dev, ATH_BUF_NI(bf_ff), bf_ff, 
					bf_ff->bf_skb, 0) == 0)
			continue;
	bad:
		ath_return_txbuf(sc, &bf_ff);
	}
}
#endif

static inline
u_int32_t
ath_get_buffers_available(const struct ath_softc *sc)
{
	return ATH_TXBUF - atomic_read(&sc->sc_txbuf_counter);
}

#ifdef IEEE80211_DEBUG_REFCNT
/* NOTE: This function is valid in non-debug configurations, just not used. */
static inline
u_int32_t
ath_get_buffer_count(const struct ath_softc *sc)
{
	return atomic_read(&sc->sc_txbuf_counter);
}
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */

static 
struct ath_buf *
#ifdef IEEE80211_DEBUG_REFCNT
_take_txbuf_locked_debug(struct ath_softc *sc, int for_management, 
			const char *func, int line)
#else
_take_txbuf_locked(struct ath_softc *sc, int for_management)
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */
{
	struct ath_buf *bf = NULL;
	ATH_TXBUF_LOCK_ASSERT(sc);
	/* Reserve at least ATH_TXBUF_MGT_RESERVED buffers for management frames */
	if (ath_get_buffers_available(sc) <= ATH_TXBUF_MGT_RESERVED) {
		/* Stop the queue, we are full */
		DPRINTF(sc, ATH_DEBUG_XMIT, "Stopping queuing of additional "
					    "frames.  Insufficient free "
					    "buffers.\n");
		sc->sc_stats.ast_tx_qstop++;
		netif_stop_queue(sc->sc_dev);
	}

	/* Only let us go further if management frame, or there are enough */
	if (for_management || (ath_get_buffers_available(sc) > ATH_TXBUF_MGT_RESERVED)) {
		bf = STAILQ_FIRST(&sc->sc_txbuf);
		if (bf) {
			STAILQ_REMOVE_HEAD(&sc->sc_txbuf, bf_list);
			/* This should be redundant, unless someone illegally 
			 * accessed the buffer after returning it. */
			cleanup_ath_buf(sc, bf, BUS_DMA_TODEVICE);
			atomic_inc(&sc->sc_txbuf_counter);
#ifdef IEEE80211_DEBUG_REFCNT
			bf->bf_taken_at_func = func;
			bf->bf_taken_at_line = line;
#else /* IEEE80211_DEBUG_REFCNT */
			/* This isn't very useful, but it keeps the lost buffer scanning code simpler */
			bf->bf_taken_at_func = __func__;
			bf->bf_taken_at_line = __LINE__;
#endif /* IEEE80211_DEBUG_REFCNT */
#ifdef IEEE80211_DEBUG_REFCNT
			DPRINTF(sc, ATH_DEBUG_TXBUF, 
				"[TXBUF=%03d/%03d] (from %s:%d) took txbuf %p.\n", 
				ath_get_buffer_count(sc), ATH_TXBUF,
				func, line, bf);
#endif
		}
		else {
			DPRINTF(sc, ATH_DEBUG_ANY, 
				"Dropping %s; no xmit buffers available.\n", 
				for_management ? "management frame" : "frame");
			sc->sc_stats.ast_tx_nobuf++;
		}
	}

	return bf;
}

static
struct ath_buf *
#ifdef IEEE80211_DEBUG_REFCNT
_take_txbuf_debug(struct ath_softc *sc, int for_management,
		  const char *func, int line)
#else
_take_txbuf(struct ath_softc *sc, int for_management)
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */
{
	struct ath_buf *bf = NULL;
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
 * It must return either NETDEV_TX_OK or NETDEV_TX_BUSY
 */
static int
ath_hardstart(struct sk_buff *__skb, struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211_node *ni = NULL;
	struct ath_buf *bf = NULL;
	struct ether_header *eh;
	ath_bufhead bf_head;
	struct ath_buf *tbf;
	struct sk_buff *tskb;
	int framecnt;
	struct sk_buff *original_skb  = __skb; /* ALWAYS FREE THIS ONE!!! */
	struct ath_node *an;
	struct sk_buff *skb = NULL;
	/* We will use the requeue flag to denote when to stuff a skb back into
	 * the OS queues.  This should NOT be done under low memory conditions,
	 * such as skb allocation failure.  However, it should be done for the
	 * case where all the dma buffers are in use (take_txbuf returns null).
	*/
	int requeue = 0;
#ifdef ATH_SUPERG_FF
	unsigned int pktlen;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_txq *txq = NULL;
	/* NB: NEVER free __skb, leave it alone and use original_skb instead!
	 * IF original_skb is NULL it means the ownership was taken!
	 * *** ALWAYS *** free any skb != __skb when cleaning up - unless it was
	 * taken. */
	int ff_flush;
#endif
	ieee80211_skb_track(original_skb);
	if ((dev->flags & IFF_RUNNING) == 0 || sc->sc_invalid) {
		DPRINTF(sc, ATH_DEBUG_XMIT,
			"Dropping; invalid %d flags %x\n",
			sc->sc_invalid, dev->flags);
		sc->sc_stats.ast_tx_invalid++;
		ieee80211_dev_kfree_skb(&original_skb);
		return NETDEV_TX_OK;
	}

	/* NB: always passed down by 802.11 layer */
	if (NULL == (an = ATH_NODE(ni = SKB_NI(original_skb)))) {
		DPRINTF(sc, ATH_DEBUG_XMIT,
			"Dropping; No node in original_skb control block!\n");
		ieee80211_dev_kfree_skb(&original_skb);
		return NETDEV_TX_OK;
	}
	/* NOTE: This used to be done only for clones, but we are doing this
	 * here as a defensive measure.  XXX: Back off of this later. */
	if (!skb_cloned(original_skb)) {
		skb = original_skb;
		original_skb = NULL;
	}
	else {
		if (NULL == (skb = skb_copy(original_skb, GFP_ATOMIC))) {
			DPRINTF(sc, ATH_DEBUG_XMIT, "Dropping; skb_copy failure.\n");
			ieee80211_dev_kfree_skb(&original_skb);
			return NETDEV_TX_OK;
		}
		ieee80211_skb_copy_noderef(skb, original_skb);
	}

	STAILQ_INIT(&bf_head);
	eh = (struct ether_header *)skb->data;

	if (SKB_CB(skb)->flags & M_RAW) {
		bf = ath_take_txbuf(sc);
		if (bf == NULL) {
			/* All DMA buffers full, safe to try again. */
			requeue = 1;
			goto hardstart_fail;
		}
		ath_tx_startraw(dev, bf, skb);
		ieee80211_dev_kfree_skb(&original_skb);
		return NETDEV_TX_OK;
	}

#ifdef ATH_SUPERG_FF
	if (M_FLAG_GET(skb, M_UAPSD)) {
		/* bypass FF handling */
		bf = ath_take_txbuf(sc);
		if (bf == NULL) {
			/* All DMA buffers full, safe to try again. */
			requeue = 1;
			goto hardstart_fail;
		}
		goto ff_bypass;
	}

	/*
	 * Fast frames check.
	 */
	ATH_FF_MAGIC_CLR(skb);

	txq = sc->sc_ac2q[skb->priority];

	if (txq->axq_depth > ATH_QUEUE_DROP_COUNT) {
		/*
		 * WME queue too long, drop packet
		 *
		 * this is necessary to achieve prioritization of packets between
		 * different queues. otherwise the lower priority queue (BK)
		 * would consume all tx buffers, while higher priority queues
		 * (e.g. VO) would be empty
		 */
		requeue = 0;
		goto hardstart_fail;
	}

	/* NB: use this lock to protect an->an_tx_ffbuf (and txq->axq_stageq)
	 *     in athff_can_aggregate() call too. */
	ATH_TXQ_LOCK_IRQ(txq);
	if (athff_can_aggregate(sc, eh, an, skb, 
				ni->ni_vap->iv_fragthreshold, &ff_flush)) {
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
				"Aggregating fast-frame\n");
		} else {
			/* NB: Careful grabbing the TX_BUF lock since still 
			 *     holding the TXQ lock.  This could be avoided 
			 *     by always obtaining the TXBuf earlier, but 
			 *     the "if" portion of this "if/else" clause would 
			 *     then need to give the buffer back. */
			bf = ath_take_txbuf(sc);
			if (bf == NULL) {
				ATH_TXQ_UNLOCK_IRQ_EARLY(txq);
				/* All DMA buffers full, safe to try again. */
				goto hardstart_fail;
			}
			DPRINTF(sc, ATH_DEBUG_XMIT | ATH_DEBUG_FF,
				"Adding to fast-frame stage queue\n");

			bf->bf_skb = skb;
			bf->bf_queueage = txq->axq_totalqueued;
			an->an_tx_ffbuf[skb->priority] = bf;

			TAILQ_INSERT_HEAD(&txq->axq_stageq, bf, bf_stagelist);

			ATH_TXQ_UNLOCK_IRQ_EARLY(txq);

			ieee80211_dev_kfree_skb(&original_skb);
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
					"Dropping; fast-frame flush encap. "
					"failure\n");
				sc->sc_stats.ast_tx_encap++;
			} else {
				pktlen = bf_ff->bf_skb->len;	/* NB: don't reference skb below */
				if (!ath_tx_start(dev, ni, bf_ff, 
							bf_ff->bf_skb, 0))
					success = 1;
			}

			if (!success) {
				DPRINTF(sc, ATH_DEBUG_XMIT | ATH_DEBUG_FF,
					"Dropping; fast-frame stageq flush "
					"failure\n");
				ath_return_txbuf(sc, &bf_ff);
			}
			bf = ath_take_txbuf(sc);
			if (bf == NULL) {
				/* All DMA buffers full, safe to try again. */
				requeue = 1;
				goto hardstart_fail;
			}

			goto ff_flush_done;
		}
		/* XXX: out-of-order condition only occurs for AP mode and 
		 *      multicast.  But, there may be no valid way to get 
		 *      this condition. */
		else if (an->an_tx_ffbuf[skb->priority]) {
			DPRINTF(sc, ATH_DEBUG_XMIT | ATH_DEBUG_FF,
				"Discarding; out of sequence fast-frame\n");
		}

		bf = ath_take_txbuf(sc);
		if (bf == NULL) {
			ATH_TXQ_UNLOCK_IRQ_EARLY(txq);
			/* All DMA buffers full, safe to try again. */
			requeue = 1;
			goto hardstart_fail;
		}
	}

	ATH_TXQ_UNLOCK_IRQ(txq);

ff_flush_done:
ff_bypass:

#else /* ATH_SUPERG_FF */

	bf = ath_take_txbuf(sc);
	if (bf == NULL) {
		/* All DMA buffers full, safe to try again. */
		requeue = 1;
		goto hardstart_fail;
	}

#endif /* ATH_SUPERG_FF */

	/*
	 * Encapsulate the packet for transmission.
	 */
	skb = ieee80211_encap(ni, skb, &framecnt);
	if (skb == NULL) {
		DPRINTF(sc, ATH_DEBUG_XMIT,
			"Dropping; encapsulation failure\n");
		sc->sc_stats.ast_tx_encap++;
		goto hardstart_fail;
	}

	if (framecnt > 1) {
		unsigned int bfcnt;

		/*
		 *  Allocate 1 ath_buf for each frame given 1 was
		 *  already alloc'd
		 */
		bf->bf_skb = NULL;
		STAILQ_INSERT_TAIL(&bf_head, bf, bf_list);
		bf = NULL;

		for (bfcnt = 1; bfcnt < framecnt; ++bfcnt) {
			tbf = ath_take_txbuf_locked(sc);
			if (tbf == NULL)
				break;
			STAILQ_INSERT_TAIL(&bf_head, tbf, bf_list);
		}

		if (bfcnt != framecnt) {
			ath_return_txbuf_list_locked(sc, &bf_head);
			STAILQ_INIT(&bf_head);
			goto hardstart_fail;
		}

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
				bf->bf_skb = NULL; /* avoid duplicate free */
				goto hardstart_fail;
			} else {
				skb = NULL; /* consumed by ath_tx_start */
				bf  = NULL; /* consumed by ath_tx_start */
			}
			skb = tskb;
		}
	} else {
		if (ath_tx_start(dev, ni, bf, skb, 0) != 0) {
			STAILQ_INSERT_TAIL(&bf_head, bf, bf_list);
			bf->bf_skb = NULL; /* avoid duplicate free */
			goto hardstart_fail;
		} else {
			skb = NULL; /* consumed by ath_tx_start */
			bf  = NULL; /* consumed by ath_tx_start */
		}
	}

#ifdef ATH_SUPERG_FF
	/* flush out stale FF from staging Q for applicable operational modes. */
	/* XXX: ADHOC mode too? */
	if (txq && ic->ic_opmode == IEEE80211_M_HOSTAP)
		ath_ffstageq_flush(sc, txq, ath_ff_ageflushtestdone);
#endif

	KASSERT((skb == NULL),  ("(skb != NULL)"));
 	KASSERT((bf == NULL),  ("(bf != NULL)"));
	ieee80211_dev_kfree_skb(&original_skb);
	return NETDEV_TX_OK;

hardstart_fail:
	/* Release the buffers, now that skbs are disconnected */
	ath_return_txbuf_list(sc, &bf_head);
	/* Pass control of the skb to the caller (i.e., resources are their 
	 * problem). */
	if (requeue) {
		/* Queue is full, let the kernel backlog the skb */
		netif_stop_queue(dev);
		/* Take care to free clone, but not original when we are
		 * requeuing.  Untrack the original skb, so that it is good to
		 * go again. */
		if (skb != __skb)
			ieee80211_dev_kfree_skb(&skb);
		skb = NULL;
		ieee80211_skb_untrack(__skb);
		return NETDEV_TX_BUSY;
	}
	/* Avoid a duplicate free */
	KASSERT( ((!original_skb) || (skb != original_skb)), 
		("skb should never be the same as original_skb, without setting "
		 "original_skb to NULL"));
	ieee80211_dev_kfree_skb(&skb);
	ieee80211_dev_kfree_skb(&original_skb);
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
			"Dropping; invalid %d flags %x\n",
			sc->sc_invalid, dev->flags);
		sc->sc_stats.ast_tx_invalid++;
		error = -ENETDOWN;
		goto bad;
	}
	/*
	 * Grab a TX buffer and associated resources.
	 */
	bf = ath_take_txbuf_mgmt(sc);
	if (bf == NULL) {
		WPRINTF(sc, "Dropping; no available transmit buffers.\n");
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
	error = ath_tx_start(dev, SKB_NI(skb), bf, skb, 0);
	if (error)
		goto bad;

	sc->sc_stats.ast_tx_mgmt++;
	return 0;
bad:
	ieee80211_dev_kfree_skb(&skb);
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
	printk(" mac " MAC_FMT, MAC_ADDR(mac));
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
	static const u_int8_t zerobssid[IEEE80211_ADDR_LEN];
	struct ath_hal *ah = sc->sc_ah;

	KASSERT(k->wk_cipher->ic_cipher == IEEE80211_CIPHER_TKIP,
		("got a non-TKIP key, cipher %u", k->wk_cipher->ic_cipher));
	if ((k->wk_flags & IEEE80211_KEY_TXRX) == IEEE80211_KEY_TXRX) {
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
	} else if (k->wk_flags & IEEE80211_KEY_TXRX) {
		/* TX/RX key goes at first index.
		 * The HAL handles the MIC keys are index + 64. */
		memcpy(hk->kv_mic, k->wk_flags & IEEE80211_KEY_XMIT ?
			k->wk_txmic : k->wk_rxmic, sizeof(hk->kv_mic));
		KEYPRINTF(sc, k->wk_keyix, hk, mac);
		return ath_hal_keyset(ah, ATH_KEY(k->wk_keyix), hk, mac, 
				AH_FALSE);
	}
	return 0;
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
				"Key pair %u,%u %u,%u\n",
				keyix, keyix + 64,
				keyix + 32, keyix + 32 + 64);
			return keyix;
		}
	}
	EPRINTF(sc, "Out of pair space!\n");
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
				"Key pair %u,%u\n",
				keyix, keyix + 64);
			return keyix;
		}
	}
	EPRINTF(sc, "Out of pair space!\n");
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
			DPRINTF(sc, ATH_DEBUG_KEYCACHE, "Key %u\n", keyix);
			return keyix;
		}
	}
	EPRINTF(sc, "Out of space!\n");
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
				"Group key is invalid.\n");
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

	DPRINTF(sc, ATH_DEBUG_KEYCACHE, "Deleting key %u\n", keyix);

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

	DPRINTF(sc, ATH_DEBUG_KEYCACHE, "Begin\n");
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

	DPRINTF(sc, ATH_DEBUG_KEYCACHE, "End\n");
	netif_wake_queue(dev);
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
	struct ieee80211com *ic = &sc->sc_ic;
	struct net_device *dev = ic->ic_dev;
	struct ath_hal *ah = sc->sc_ah;
	u_int32_t rfilt;

	/* Preserve the current Phy. radar and err. filters. */
	rfilt = (ath_hal_getrxfilter(ah) &
			(HAL_RX_FILTER_PHYERR | HAL_RX_FILTER_PHYRADAR)) |
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
	if (sc->sc_curchan.privFlags & CHANNEL_DFS)
		rfilt |= (HAL_RX_FILTER_PHYERR | HAL_RX_FILTER_PHYRADAR);
	return rfilt;
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
			"Set RX filter: 0x%x, MC filter: %08x:%08x\n",
			rfilt, mfilt[0], mfilt[1]);
}

static inline int 
ath_slottime2timeout(struct ath_softc *sc, int slottime)
{
	/* IEEE 802.11 2007 9.2.8 says the ACK timeout shall be SIFSTime +
	 * slot time (+ PHY RX start delay). HAL seems to use a constant of 8
	 * for OFDM overhead and 18 for CCK overhead.
	 *
	 * XXX: Update based on emperical evidence (potentially save 15us per
	 * timeout). */
	if (((sc->sc_curchan.channelFlags & IEEE80211_CHAN_A) ==
				IEEE80211_CHAN_A) ||
	    (((sc->sc_curchan.channelFlags & IEEE80211_CHAN_108G) ==
	      			IEEE80211_CHAN_108G) && 
	     (sc->sc_ic.ic_flags & IEEE80211_F_SHSLOT)))
		/* Short slot time: 802.11a, and 802.11g turbo in turbo mode
		 * with short slot time. */
		return slottime + 8;
	else
		/* Constant for CCK MIB processing time. */
		return slottime + 18;
}

static inline int 
ath_default_ctsack_timeout(struct ath_softc *sc)
{
	struct ath_hal *ah = sc->sc_ah;

	int slottime = sc->sc_slottimeconf;
	if (slottime <= 0)
		slottime = ath_hal_getslottime(ah);

	return ath_slottime2timeout(sc, slottime);
}

static inline int
ath_getslottime(struct ath_softc *sc)
{
	return ath_hal_getslottime(sc->sc_ah);
}

static inline int
ath_getacktimeout(struct ath_softc *sc)
{
	return ath_hal_getacktimeout(sc->sc_ah);
}

static inline int
ath_getctstimeout(struct ath_softc *sc)
{
	return ath_hal_getctstimeout(sc->sc_ah);
}

/*
 * Set the slot time based on the current setting.
 */
static inline void
ath_setslottime(struct ath_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_hal *ah = sc->sc_ah;

	if (sc->sc_slottimeconf > 0) /* manual override */
		ath_hal_setslottime(ah, sc->sc_slottimeconf);
	else if (sc->sc_dturbo || (sc->sc_curchan.privFlags & CHANNEL_ST))
		ath_hal_setslottime(ah, HAL_SLOT_TIME_6);
	else if (ic->ic_flags & IEEE80211_F_SHSLOT)
		ath_hal_setslottime(ah, HAL_SLOT_TIME_9);
	else
		ath_hal_setslottime(ah, HAL_SLOT_TIME_20);
	sc->sc_updateslot = OK;
	ath_setacktimeout(sc);
	ath_setctstimeout(sc);
}

/*
 * Set the ACK timeout based on the current setting.
 */
static void
ath_setacktimeout(struct ath_softc *sc)
{
	struct ath_hal *ah = sc->sc_ah;

	if (sc->sc_acktimeoutconf > 0) /* manual override */
		ath_hal_setacktimeout(ah, sc->sc_acktimeoutconf);
	else
		ath_hal_setacktimeout(ah, ath_default_ctsack_timeout(sc));
}

/*
 * Set the CTS timeout based on the current setting.
 */
static void
ath_setctstimeout(struct ath_softc *sc)
{
	struct ath_hal *ah = sc->sc_ah;

	if (sc->sc_ctstimeoutconf > 0) /* manual override */
		ath_hal_setctstimeout(ah, sc->sc_ctstimeoutconf);
	else
		ath_hal_setctstimeout(ah, ath_default_ctsack_timeout(sc));
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
	struct ieee80211com *ic = vap->iv_ic;
	struct ath_softc *sc = ic->ic_dev->priv;

	/* Check VAP capability. */
	if ((ic->ic_opmode == IEEE80211_M_HOSTAP) && vap->iv_bss &&
			((vap->iv_bss->ni_ath_flags & IEEE80211_ATHC_TURBOP) == 
			 IEEE80211_ATHC_TURBOP)) {
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
		    (bss_traffic < sc->sc_dturbo_bw_base ||
		     !sc->sc_rate_recn_state)) {
			sc->sc_dturbo_hold = 1;
		} else {
			if (sc->sc_dturbo_hold &&
			    bss_traffic >= sc->sc_dturbo_bw_turbo &&
			    sc->sc_rate_recn_state) {
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
			DPRINTF(sc, ATH_DEBUG_TURBO, "Leaving turbo mode\n");
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
			DPRINTF(sc, ATH_DEBUG_TURBO, "Entering turbo mode.\n");
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
	for (slot = 0; slot < ath_maxvaps; slot++) {
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
	sc->sc_txqsetup |= 1 << qnum;
	return qnum;
}

/*
 * Configure IFS parameter for the beacon queue.
 */
static int
ath_beaconq_config(struct ath_softc *sc)
{
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
		qi.tqi_cwmin = 0;
		qi.tqi_cwmax = 2 * ((1 << wmep->wmep_logcwmin) - 1);
	}

	DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
		"Invoking ath_hal_settxqueueprops with tqi_aifs:%d tqi_cwmin:%d tqi_cwmax:%d\n",
		qi.tqi_aifs, qi.tqi_cwmin, qi.tqi_cwmax);
	if (!ath_hal_settxqueueprops(ah, sc->sc_bhalq, &qi)) {
		EPRINTF(sc, "Unable to update hardware beacon queue parameters.\n");
		return 0;
	} else {
		DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"Invoking ath_hal_resettxqueue with sc_bhalq:%d\n",
			sc->sc_bhalq);
		ath_hal_resettxqueue(ah, sc->sc_bhalq);	/* push to h/w */
		return 1;
	}
}

static int
ath_beacon_alloc(struct ath_softc *sc, struct ieee80211_node *ni)
{
	struct ath_vap * avp = ATH_VAP(ni->ni_vap);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)
	atomic_set(&avp->av_beacon_alloc, 1);
#else
	set_bit(0, &avp->av_beacon_alloc);
#endif

	return 0;
}

/* Allocate and setup an initial beacon frame.
 * Context: hwIRQ */
static int
ath_beacon_alloc_internal(struct ath_softc *sc, struct ieee80211_node *ni)
{
	struct ath_vap *avp = ATH_VAP(ni->ni_vap);
	struct ieee80211_frame *wh;
	struct ath_buf *bf;
	struct sk_buff *skb;

	DPRINTF(sc, ATH_DEBUG_BEACON, "Started.\n");

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
		DPRINTF(sc, ATH_DEBUG_BEACON, "Beacon allocation failed!\n");
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
		u_int64_t tuadjust;
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
		tuadjust = (ni->ni_intval * (ath_maxvaps - avp->av_bslot)) / ath_maxvaps;
		tsfadjust = cpu_to_le64(tuadjust << 10);	/* TU->TSF */

		DPRINTF(sc, ATH_DEBUG_BEACON,
			"%s beacons, bslot %d intval %u tsfadjust(Kus) %llu\n",
			sc->sc_stagbeacons ? "staggered" : "bursted",
			avp->av_bslot, ni->ni_intval, (unsigned long long) tuadjust);

		wh = (struct ieee80211_frame *)skb->data;
		memcpy(&wh[1], &tsfadjust, sizeof(tsfadjust));
	}

	bf->bf_skbaddr = bus_map_single(sc->sc_bdev,
		skb->data, skb->len, BUS_DMA_TODEVICE);
	bf->bf_skb = skb;

	DPRINTF(sc, ATH_DEBUG_BEACON, "Finished.\n");

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
	struct ieee80211com *ic = SKB_NI(bf->bf_skb)->ni_ic;
	struct sk_buff *skb = bf->bf_skb;
	struct ath_hal *ah = sc->sc_ah;
	struct ath_desc *ds = bf->bf_desc;
	unsigned int flags;
	int antenna = sc->sc_txantenna;
	const HAL_RATE_TABLE *rt;
	u_int8_t rix, rate;
	unsigned int ctsrate = 0, ctsduration = 0;

	DPRINTF(sc, ATH_DEBUG_BEACON_PROC, "skb=%p skb->len=%u\n",
		skb, skb->len);

	flags = HAL_TXDESC_NOACK;
#ifdef ATH_SUPERG_DYNTURBO
	if (sc->sc_dturbo_switch)
		flags |= HAL_TXDESC_INTREQ;
#endif
	
	/* Switch antenna every beacon if txantenna is not set
	 * Should only switch every beacon period, not for all
	 * SWBAs
	 * XXX: assumes two antennae */
	if (antenna == 0) {
		if (sc->sc_stagbeacons)
			antenna = ((sc->sc_stats.ast_be_xmit / 
						sc->sc_nbcnvaps) & 1 ? 2 : 1);
		else
			antenna = (sc->sc_stats.ast_be_xmit & 1 ? 2 : 1);
	}

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
	if (SKB_NI(bf->bf_skb)->ni_vap->iv_flags & IEEE80211_F_XR) {
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
	ds->ds_link = 0;
	ds->ds_data = bf->bf_skbaddr;
	ath_hal_setuptxdesc(ah, ds,
		skb->len + IEEE80211_CRC_LEN,	/* frame length */
		sizeof(struct ieee80211_frame), /* header length */
		HAL_PKT_TYPE_BEACON,		/* Atheros packet type */
		SKB_NI(bf->bf_skb)->ni_txpower, /* txpower XXX */
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

static int
txqactive(struct ath_hal *ah, int qnum)
{
	u_int32_t txqs = 1 << qnum;
	ath_hal_gettxintrtxqs(ah, &txqs);
	return (txqs & (1 << qnum));
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
	unsigned int curlen;
	u_int8_t tim_bitctl;
	int is_dtim = 0;

	if (vap->iv_state != IEEE80211_S_RUN) {
		DPRINTF(sc, ATH_DEBUG_BEACON_PROC, 
			"Skipping VAP in %s state\n",
			ieee80211_state_name[vap->iv_state]);
		return NULL;
	}

	if (ath_chan_unavail_dbgmsg(sc)) {
		DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"Skipping VAP when DFS requires radio silence\n");
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
	if (avp == NULL || avp->av_bcbuf == NULL) {
		DPRINTF(sc, ATH_DEBUG_ANY, "Returning NULL, one of these "
				"is NULL {avp=%p av_bcbuf=%p}\n", 
				avp, avp->av_bcbuf);
		return NULL;
	}
	bf = avp->av_bcbuf;

#ifdef ATH_SUPERG_DYNTURBO
	/*
	 * If we are using dynamic turbo, update the
	 * capability info and arrange for a mode change
	 * if needed.
	 */
	if (sc && avp && sc->sc_dturbo && NULL != avp->av_boff.bo_tim) {
		u_int8_t dtim;
		dtim = ((avp->av_boff.bo_tim[2] == 1) ||
			(avp->av_boff.bo_tim[3] == 1));
		ath_beacon_dturbo_update(vap, needmark, dtim);
	}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)
	if (atomic_add_unless(&avp->av_beacon_alloc, -1, 0))
#else
	if (test_and_clear_bit(0, &avp->av_beacon_alloc))
#endif
		ath_beacon_alloc_internal(sc, vap->iv_bss);

	/*
	 * Update dynamic beacon contents.  If this returns
	 * non-zero then we need to remap the memory because
	 * the beacon frame changed size (probably because
	 * of the TIM bitmap).
	 */
	skb = bf->bf_skb;
	curlen = skb->len;
	DPRINTF(sc, ATH_DEBUG_BEACON,
		"Updating beacon - mcast pending=%d.\n",
		avp->av_mcastq.axq_depth);
	if (ieee80211_beacon_update(SKB_NI(bf->bf_skb), &avp->av_boff, skb,
				!!STAILQ_FIRST(&avp->av_mcastq.axq_q),
				&is_dtim)) {
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
	if ((tim_bitctl & BITCTL_BUFD_MCAST) && 
	    STAILQ_FIRST(&avp->av_mcastq.axq_q) && 
	    STAILQ_FIRST(&sc->sc_cabq->axq_q) &&
	    (sc->sc_nvaps > 1) && 
	    (sc->sc_stagbeacons)) {
		ath_tx_draintxq(sc, sc->sc_cabq);
		DPRINTF(sc, ATH_DEBUG_BEACON,
			"Drained previous cabq transmit traffic to make room for next VAP.\n");
	} else if (STAILQ_FIRST(&sc->sc_cabq->axq_q) &&
			ATH_TXQ_SETUP(sc, sc->sc_cabq->axq_qnum) &&
			!txqactive(sc->sc_ah,  sc->sc_cabq->axq_qnum)) {
		DPRINTF(sc, 
			ATH_DEBUG_BEACON,
			"Draining %d stalled CABQ transmit buffers.\n",
			sc->sc_cabq->axq_depth);
		ath_tx_draintxq(sc, sc->sc_cabq);
	} else {
		DPRINTF(sc, ATH_DEBUG_BEACON,
			"No need to drain previous CABQ transmit traffic.\n");
		DPRINTF(sc, ATH_DEBUG_BEACON,"[dtim=%d mcast=%d cabq=%d nvaps=%d staggered=%d]\n", 
			!!(tim_bitctl & BITCTL_BUFD_MCAST),
			avp->av_mcastq.axq_depth,
			sc->sc_cabq->axq_depth,
			sc->sc_nvaps,
			sc->sc_stagbeacons);
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
	if (is_dtim) {
		struct ath_txq *cabq = sc->sc_cabq;
		struct ath_buf *bfmcast;
		DPRINTF(sc, ATH_DEBUG_BEACON,
			"Checking CABQ - It's DTIM.\n");
		/*
		 * Move everything from the VAPs mcast queue
		 * to the hardware cab queue.
		 */
		ATH_TXQ_LOCK_IRQ(&avp->av_mcastq);
		ATH_TXQ_LOCK_IRQ_INSIDE(cabq);
		bfmcast = STAILQ_FIRST(&avp->av_mcastq.axq_q);
		if (bfmcast != NULL) {
			struct ath_buf *tbf;
			DPRINTF(sc, ATH_DEBUG_BEACON,
				"Multicast pending.  "
				"Linking CABQ to avp->av_mcastq.axq_q.\n");

			/* Set the MORE_DATA bit for each packet except the
			 * last one */
			STAILQ_FOREACH(tbf, &avp->av_mcastq.axq_q, bf_list) {
				if (tbf != STAILQ_LAST(&avp->av_mcastq.axq_q, 
							ath_buf, bf_list))
					((struct ieee80211_frame *)
					 tbf->bf_skb->data)->i_fc[1] |= 
						IEEE80211_FC1_MORE_DATA;
			}

			/* Append the private VAP mcast list to the cabq. */
			ATH_TXQ_MOVE_Q(&avp->av_mcastq, cabq);
			if (!STAILQ_FIRST(&cabq->axq_q))
				ath_hw_puttxbuf(sc, cabq->axq_qnum,
					bfmcast->bf_daddr,
					__func__);
		}
		else {
			DPRINTF(sc, ATH_DEBUG_BEACON,
				"Not Linking CABQ to avp->av_mcastq.axq_q."
				"sc_stagbeacons=%d bfmcast=%d\n", 
				!!sc->sc_stagbeacons, !!bfmcast);
		}

		/* NB: gated by beacon so safe to start here */
		if (STAILQ_FIRST(&cabq->axq_q)) {
			DPRINTF(sc, ATH_DEBUG_BEACON,
				"Checking CABQ - CABQ being started.\n");
			if (!ath_hal_txstart(ah, cabq->axq_qnum)) {
				DPRINTF(sc, ATH_DEBUG_TX_PROC,
					"Failed to start CABQ\n");
			}
		}
		else {
			DPRINTF(sc, ATH_DEBUG_BEACON,
				"Checking CABQ - CABQ is empty!!\n");
		}
		ATH_TXQ_UNLOCK_IRQ_INSIDE(cabq);
		ATH_TXQ_UNLOCK_IRQ(&avp->av_mcastq);
	}
	else {
		DPRINTF(sc, ATH_DEBUG_BEACON,
			"Not checking CABQ - NOT DTIM.\n");
	}

	return bf;
}

/*
 * Transmit one or more beacon frames at SWBA.  Dynamic
 * updates to the frame contents are done as needed and
 * the slot time is also adjusted based on current state.
 */
static void
ath_beacon_send(struct ath_softc *sc, int *needmark, uint64_t hw_tsf)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211vap *vap;
	struct ath_buf *bf;
	unsigned int slot;
	u_int32_t bfaddr = 0;
	u_int32_t n_beacon;

	if (ath_chan_unavail_dbgmsg(sc))
		return;

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
			"Missed %u consecutive beacons (n_beacon=%u)\n",
			sc->sc_bmisscount, n_beacon);
		if (sc->sc_bmisscount > BSTUCK_THRESH)
			ATH_SCHEDULE_TQUEUE(&sc->sc_bstucktq, needmark);
		return;
	}
	if (sc->sc_bmisscount != 0) {
		DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"Resumed beacon xmit after %u misses\n",
			sc->sc_bmisscount);
		sc->sc_bmisscount = 0;
	}

	/*
	 * Generate beacon frames.  If we are sending frames
	 * staggered then calculate the slot for this frame based
	 * on the hw_tsf to safeguard against missing an swba.
	 * Otherwise we are bursting all frames together and need
	 * to generate a frame for each VAP that is up and running.
	 */
	if (sc->sc_stagbeacons) {		/* staggered beacons */
		struct ieee80211com *ic = &sc->sc_ic;
		u_int32_t tsftu;

		tsftu = IEEE80211_TSF_TO_TU(hw_tsf);
		slot = ((tsftu % ic->ic_lintval) * ath_maxvaps) / ic->ic_lintval;
		vap = sc->sc_bslot[(slot + 1) % ath_maxvaps];
		DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"Slot %d [tsf %llu tsftu %llu intval %u] vap %p\n",
			slot, (unsigned long long)hw_tsf, 
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
		for (slot = 0; slot < ath_maxvaps; slot++) {
			if ((vap = sc->sc_bslot[slot]) != NULL) {
				if ((bf = ath_beacon_generate(
						sc, vap, 
						needmark)) != NULL) {
					if (bflink != NULL)
						*bflink = ath_ds_link_swap(
								bf->bf_daddr);
					else
						/* For the first bf, save 
						 * bf_addr for later */
						bfaddr = bf->bf_daddr;

					bflink = &bf->bf_desc->ds_link;
				}
			}
		}
		if (bflink != NULL)
			*bflink = 0;		/* link of last frame */

		if (!bfaddr)
			DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
					"Bursted beacons failed to set "
					"bfaddr!\n");
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
	 *     set to ath_maxvaps so this check is a no-op.
	 */
	/* XXX locking */
	if (sc->sc_updateslot == UPDATE) {
		sc->sc_updateslot = COMMIT;	/* commit next beacon */
		sc->sc_slotupdate = slot;
	} else if ((sc->sc_updateslot == COMMIT) && (sc->sc_slotupdate == slot))
		ath_setslottime(sc);		/* commit change to hardware */

	/* If HW fast diversity is not enabled and there is not default RX
	 * antenna set, check recent per-antenna transmit statistics and flip
	 * the default RX antenna if noticeably more frames went out on the
	 * non-default antenna. */
	if ((!sc->sc_stagbeacons || slot == 0) &&
			!sc->sc_diversity && !sc->sc_rxantenna) {
		unsigned int otherant;
		/* XXX: assumes 2 antennae. */
		otherant = sc->sc_rxantenna & 1 ? 2 : 1;
		if (sc->sc_ant_tx[otherant] > sc->sc_ant_tx[sc->sc_rxantenna] + 
				ATH_ANTENNA_DIFF) {
			DPRINTF(sc, ATH_DEBUG_BEACON,
				"Flip default RX antenna to %u, %u > %u\n",
				otherant, sc->sc_ant_tx[otherant],
				sc->sc_ant_tx[sc->sc_rxantenna]);
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
			"Invoking ath_hal_stoptxdma with sc_bhalq:%d\n",
			sc->sc_bhalq);
		if (!ath_hal_stoptxdma(ah, sc->sc_bhalq)) {
			DPRINTF(sc, ATH_DEBUG_ANY,
				"Beacon queue %u did not stop?\n",
				sc->sc_bhalq);
			/* NB: the HAL still stops DMA, so proceed */
		}
		/* NB: cabq traffic should already be queued and primed */
		DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"Invoking ath_hw_puttxbuf with sc_bhalq: %d bfaddr: %x\n",
			sc->sc_bhalq, bfaddr);
		ath_hw_puttxbuf(sc, sc->sc_bhalq, bfaddr, __func__);

		DPRINTF(sc, ATH_DEBUG_BEACON_PROC,
			"Invoking ath_hal_txstart with sc_bhalq: %d\n",
			sc->sc_bhalq);
		ath_hal_txstart(ah, sc->sc_bhalq);
		if (sc->sc_beacon_cal && 
		    time_after(jiffies, (sc->sc_lastcal + (sc->sc_calinterval_sec * HZ))))
		{
			ath_calibrate((unsigned long)sc->sc_dev);
		}
#if 0
		/* This block was useful to detect jiffies rollover bug in the 
		 * timer conditional.  Too chatty to leave enabled, but may be 
		 * useful again when debugging per-radio calibration intervals */
		else if (sc->sc_beacon_cal) {
			printk("%s: %s: now=%lu lastcal=%lu expires=%lu remaining=%u ms\n", 
			       SC_DEV_NAME(sc),
			       __FUNCTION__,
			       jiffies
			       (sc->sc_lastcal)
			       (sc->sc_lastcal + (sc->sc_calinterval_sec * HZ))
			       jiffies_to_msecs(sc->sc_lastcal + (sc->sc_calinterval_sec * HZ) - jiffies));
		}
#endif

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
	EPRINTF(sc, "Stuck beacon; resetting (beacon miss count: %u)\n",
		sc->sc_bmisscount);
	ath_reset(dev);
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
 */
static void
ath_beacon_config(struct ath_softc *sc, struct ieee80211vap *vap)
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
	hw_tsftu = IEEE80211_TSF_TO_TU(hw_tsf);
	tsftu = IEEE80211_TSF_TO_TU(tsf);

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
			intval /= ath_maxvaps;	/* for staggered beacons */
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
		else if (ic->ic_opmode == IEEE80211_M_IBSS) {
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
	}

	if (ic->ic_opmode == IEEE80211_M_STA &&	!(sc->sc_nostabeacons)) {
		HAL_BEACON_STATE bs;
		int dtimperiod, dtimcount;
		int cfpperiod, cfpcount;
		unsigned int n;

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

		/* Pull nexttbtt forward to reflect the current TSF
		 * and calculate DTIM + CFP state for the result. */
		n = howmany(hw_tsftu + FUDGE - tsftu, intval);
		nexttbtt = tsftu + (n * intval);
		dtimcount = (dtimcount + n) % dtimperiod;
		cfpcount = (cfpcount + (n / dtimperiod)) % cfpperiod;

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
		/* Store the number of consecutive beacons to miss
		 * before taking a BMISS interrupt. */
		bs.bs_bmissthreshold = 
			IEEE80211_BMISSTHRESH_SANITISE(ic->ic_bmissthreshold);

		/* Calculate sleep duration.  The configuration is
		 * given in ms.  We ensure a multiple of the beacon
		 * period is used.  Also, if the sleep duration is
		 * greater than the DTIM period then it makes senses
		 * to make it a multiple of that.
		 *
		 * XXX: fixed at 100ms. */
		bs.bs_sleepduration =
			roundup(IEEE80211_MS_TO_TU(100), bs.bs_intval);
		if (bs.bs_sleepduration > bs.bs_dtimperiod)
			bs.bs_sleepduration = roundup(bs.bs_sleepduration, 
					bs.bs_dtimperiod);

		DPRINTF(sc, ATH_DEBUG_BEACON,
			"tsf %llu tsf:tu %u intval %u nexttbtt %u dtim %u "
			"nextdtim %u bmiss %u sleep %u cfp:period %u "
			"maxdur %u next %u timoffset %u\n",
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
		ath_set_beacon_cal(sc, 0);
	} else {
		ath_hal_intrset(ah, 0);
		if (reset_tsf)
			intval |= HAL_BEACON_RESET_TSF;
		if (IEEE80211_IS_MODE_BEACON(ic->ic_opmode)) {
			/* In AP/IBSS mode we enable the beacon timers and
			 * SWBA interrupts to prepare beacon frames. */
			intval |= HAL_BEACON_ENA;
			sc->sc_imask |= HAL_INT_SWBA;
			ath_set_beacon_cal(sc, 1);
			ath_beaconq_config(sc);
		} else
			ath_set_beacon_cal(sc, 0);

#ifdef ATH_SUPERG_DYNTURBO
		ath_beacon_dturbo_config(vap, intval &
				~(HAL_BEACON_RESET_TSF | HAL_BEACON_ENA));
#endif
		sc->sc_nexttbtt = nexttbtt;
		ath_hal_beaconinit(ah, nexttbtt, intval);
		if (intval & HAL_BEACON_RESET_TSF) {
			sc->sc_last_tsf = 0;
		}
		sc->sc_bmisscount = 0;
		ath_hal_intrset(ah, sc->sc_imask);
	}

ath_beacon_config_debug:
	/* We print all debug messages here, in order to preserve the
	 * time critical aspect of this function. */
	DPRINTF(sc, ATH_DEBUG_BEACON,
		"ni=%p tsf=%llu hw_tsf=%llu tsftu=%u hw_tsftu=%u\n",
		ni,
		(unsigned long long)tsf, (unsigned long long)hw_tsf,
		tsftu, hw_tsftu);

	if (reset_tsf)
		/* We just created the interface */
		DPRINTF(sc, ATH_DEBUG_BEACON, "%s: first beacon\n", __func__);
	else if (tsf == 1)
		/* We do not receive any beacons or probe response */
		DPRINTF(sc, ATH_DEBUG_BEACON,
				"%s: no beacon received...\n",__func__);
	else if (tsf > hw_tsf)
		/* We do receive a beacon and the hw TSF has not been updated */
		DPRINTF(sc, ATH_DEBUG_BEACON,
				"%s: beacon received, but TSF is incorrect\n", 
				__func__);
	else
		/* We do receive a beacon in the past, normal case */
		DPRINTF(sc, ATH_DEBUG_BEACON,
				"%s: beacon received, TSF is correct\n", 
				__func__);

	DPRINTF(sc, ATH_DEBUG_BEACON,
		"nexttbtt=%10x intval=%u%s%s imask=%s%s\n",
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

	DPRINTF(sc, ATH_DEBUG_RESET, "%s DMA: %u buffers %u desc/buf\n",
		name, nbuf, ndesc);

	dd->dd_name = name;
	dd->dd_desc_len = sizeof(struct ath_desc) * nbuf * ndesc;
	dd->dd_nbuf = nbuf;
	dd->dd_ndesc = ndesc;

	/* allocate descriptors */
	dd->dd_desc = bus_alloc_consistent(sc->sc_bdev,
		dd->dd_desc_len, &dd->dd_desc_paddr);
	if (dd->dd_desc == NULL) {
		error = -ENOMEM;
		goto fail;
	}
	ds = dd->dd_desc;
	DPRINTF(sc, ATH_DEBUG_RESET, "%s DMA map: %p (%lu) -> %08llx (%lu)\n",
		dd->dd_name, ds, (u_long) dd->dd_desc_len,
		(u_int64_t)dd->dd_desc_paddr, /*XXX*/ (u_long) dd->dd_desc_len);

	/* allocate buffers */
	bsize = sizeof(struct ath_buf) * nbuf;
	bf = kzalloc(bsize, GFP_KERNEL);
	if (bf == NULL) {
		error = -ENOMEM;
		goto fail2;
	}
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
			"beacon", ath_maxvaps, 1);
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
ath_node_alloc(struct ieee80211vap *vap)
{
	struct ath_softc *sc = vap->iv_ic->ic_dev->priv;
	const size_t space = sizeof(struct ath_node) + sc->sc_rc->arc_space;
	struct ath_node *an = kzalloc(space, GFP_ATOMIC);
	if (an != NULL) {
		an->an_decomp_index = INVALID_DECOMP_INDEX;
		an->an_avgrssi = ATH_RSSI_DUMMY_MARKER;
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
ath_node_cleanup(struct ieee80211_node *ni)
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
		ath_return_txbuf(sc, &bf);
		an->an_uapsd_qdepth--;
	}

	while (an->an_uapsd_overflowqdepth) {
		bf = STAILQ_FIRST(&an->an_uapsd_overflowq);
		STAILQ_REMOVE_HEAD(&an->an_uapsd_overflowq, bf_list);
		ath_return_txbuf(sc, &bf);
		an->an_uapsd_overflowqdepth--;
	}

	/* Clean up node-specific rate things - this currently appears to 
	 * always be a no-op */
	sc->sc_rc->ops->node_cleanup(sc, ATH_NODE(ni));

	ATH_NODE_UAPSD_LOCK_IRQ(an);
	sc->sc_node_cleanup(ni);
	ATH_NODE_UAPSD_UNLOCK_IRQ(an);
}

static void
ath_node_free(struct ieee80211_node *ni)
{
	struct ath_softc *sc = (struct ath_softc *)ni->ni_ic;

	sc->sc_node_free(ni);
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
 * XXX: Check for TXQ races
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

	if (ath_cac_running_dbgmsg(sc))
		return;

	if (ni->ni_vap->iv_flags & IEEE80211_F_XR) {
		struct ath_txq tmp_q;
		memset(&tmp_q, 0, sizeof(tmp_q));
		STAILQ_INIT(&tmp_q.axq_q);
		
		/* Collect all the data towards the node
		 * in to the tmp_q. */
		index = WME_AC_VO;
		while (index >= WME_AC_BE && txq != sc->sc_ac2q[index]) {
			txq = sc->sc_ac2q[index];

			ATH_TXQ_LOCK_IRQ(txq);
			ath_hal_stoptxdma(ah, txq->axq_qnum);
			bf = prev = STAILQ_FIRST(&txq->axq_q);
			
			/* Skip all the buffers that are done
			 * until the first one that is in progress. */
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

			/* Save the pointer to the last buf that's done. */
			if (prev == bf)
				bf_tmp = NULL;
			else
				bf_tmp = prev;
			while (bf) {
				if (ni == ATH_BUF_NI(bf)) {
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
						/* After deleting the node
						 * link the descriptors. */
#ifdef ATH_SUPERG_FF
						ds = &prev->bf_desc
							[prev->bf_numdescff];
#else
						/* NB: last descriptor. */
						ds = prev->bf_desc;
#endif
						ds->ds_link = ath_ds_link_swap(
								bf->bf_daddr);
					}
				} else {
					prev = bf;
					bf = STAILQ_NEXT(bf, bf_list);
				}
			}

			/* If the last buf. was deleted.
			 * set the pointer to the last descriptor. */
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
					}
				}
			}

			ATH_TXQ_UNLOCK_IRQ(txq);

			/* Restart the DMA from the first
			 * buffer that was not DMA'd. */
			if (bf_tmp)
				bf = STAILQ_NEXT(bf_tmp, bf_list);
			else
				bf = STAILQ_FIRST(&txq->axq_q);
			if (bf) {
				ath_hw_puttxbuf(sc, txq->axq_qnum,
						bf->bf_daddr, __func__);
				ath_hal_txstart(ah, txq->axq_qnum);
			}
		}
		
		/* Queue them on to the XR txqueue.
		 * Can not directly put them on to the XR txq, since the
		 * SKB data size may be greater than the XR fragmentation
		 * threshold size. */
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
		struct ath_txq wme_tmp_qs[WME_AC_VO + 1];
		struct ath_txq *wmeq = NULL, *prevq;
		struct ieee80211_frame *wh;
		struct ath_desc *ds = NULL;
		unsigned int count = 0;

		/* Move data from XR txq to Normal txqs. */
		DPRINTF(sc, ATH_DEBUG_XMIT_PROC, 
				"move buffers from XR to NORMAL\n");
		memset(&wme_tmp_qs, 0, sizeof(wme_tmp_qs));
		for (index = 0; index <= WME_AC_VO; index++)
			STAILQ_INIT(&wme_tmp_qs[index].axq_q);
		txq = sc->sc_xrtxq;

		ATH_TXQ_LOCK_IRQ(txq);
		ath_hal_stoptxdma(ah, txq->axq_qnum);
		bf = prev = STAILQ_FIRST(&txq->axq_q);
		/* Skip all the buffers that are done
		 * until the first one that is in progress. */
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
		
		/* Save the pointer to the last buf that's done. */
		if (prev == bf)
			bf_tmp1 = NULL;
		else
			bf_tmp1 = prev;

		/* Collect all the data in to four temp SW queues. */
		while (bf) {
			if (ni == ATH_BUF_NI(bf)) {
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
					/* XXX: Validate skb->priority, remove 
					 * mask */
					wmeq = &wme_tmp_qs[skb->priority & 0x3];
				} else
					wmeq = &wme_tmp_qs[WME_AC_BE];
				ATH_TXQ_LINK_DESC(wmeq, bf_tmp);
				STAILQ_INSERT_TAIL(&wmeq->axq_q, bf_tmp, bf_list);
				DPRINTF(sc, ATH_DEBUG_XMIT, 
						"link[%u]=%08llx (%p)\n",
						wmeq->axq_qnum, 
						(u_int64_t)bf_tmp->bf_daddr, 
						bf_tmp->bf_desc);

				/* Update the rate information. (?) */
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
				}
			}
		}
		ATH_TXQ_UNLOCK_IRQ(txq);

		/* Restart the DMA from the first buffer that was not DMA'd. */
		if (bf_tmp1)
			bf = STAILQ_NEXT(bf_tmp1, bf_list);
		else
			bf = STAILQ_FIRST(&txq->axq_q);

		if (bf) {
			ath_hw_puttxbuf(sc, txq->axq_qnum, bf->bf_daddr, __func__);
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
				if (bf)
					ATH_TXQ_MOVE_Q(wmeq, txq);
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
				ath_hw_puttxbuf(sc, txq->axq_qnum, bf->bf_daddr, __func__);
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
ath_alloc_skb(u_int size, u_int align)
{
	struct sk_buff *skb;
	u_int off;

	skb = ieee80211_dev_alloc_skb(size + align - 1);
	if (skb != NULL) {
		off = ((unsigned long) skb->data) % align;
		if (off != 0)
			skb_reserve(skb, align - off);
	}
	return skb;
}

static struct sk_buff *
ath_rxbuf_take_skb(struct ath_softc *sc, struct ath_buf *bf)
{
	struct sk_buff *skb = bf->bf_skb;
	bf->bf_skb = NULL;
	KASSERT(bf->bf_skbaddr, ("bf->bf_skbaddr is 0"));
	bus_unmap_single(sc->sc_bdev, bf->bf_skbaddr,
			sc->sc_rxbufsize, BUS_DMA_FROMDEVICE);
	bf->bf_skbaddr = 0;
	return skb;	
}

static int
ath_rxbuf_init(struct ath_softc *sc, struct ath_buf *bf)
{
	struct ath_hal *ah = sc->sc_ah;
	struct ath_desc *ds = NULL;
	struct sk_buff *skb;

	/* NB: I'm being cautious by unmapping and releasing the SKB every
	 * time.
	 * XXX: I could probably keep rolling, but the DMA map/unmap logic
	 * doesn't seem clean enough and cycling the skb through the free
	 * function and slab allocator seems to scrub any un-reset values. */
	if (bf->bf_skb != NULL) {
		skb = ath_rxbuf_take_skb(sc, bf);
		ieee80211_dev_kfree_skb(&skb);
	}

	if (!bf->bf_skb) {
		/* NB: Always use the same size for buffer allocations so that
		 * dynamically adding a monitor mode VAP to a running driver
		 * doesn't cause havoc. */
		int size = sc->sc_rxbufsize + IEEE80211_MON_MAXHDROOM +
			sc->sc_cachelsz - 1;
		int offset = 0;
		bf->bf_skb = ath_alloc_skb(size, 
					   sc->sc_cachelsz);
		if (!bf->bf_skb) {
			EPRINTF(sc, "%s: ERROR: skb initialization failed; size: %u!\n",
				__func__, size);
			return -ENOMEM;
		}

		/* Reserve space for the header. */
		skb_reserve(bf->bf_skb, IEEE80211_MON_MAXHDROOM);

		/* Cache-line-align.  This is important (for the
		 * 5210 at least) as not doing so causes bogus data
		 * in RX'd frames. */
		offset = ((unsigned long)bf->bf_skb->data) % sc->sc_cachelsz;
		if (offset != 0)
			skb_reserve(bf->bf_skb, sc->sc_cachelsz - offset);
	}
	bf->bf_skb->dev = sc->sc_dev;
	bf->bf_skbaddr = bus_map_single(sc->sc_bdev,
		bf->bf_skb->data, sc->sc_rxbufsize, BUS_DMA_FROMDEVICE);
	if (!bf->bf_skbaddr) {
		DPRINTF(sc, ATH_DEBUG_ANY,
			"%s: ERROR: skb dma bus mapping failed!\n",
			__func__);
		return -ENOMEM;
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
		skb_tailroom(bf->bf_skb),	/* buffer size */
		0);
	if (sc->sc_rxlink != NULL)
		*sc->sc_rxlink = bf->bf_daddr;
	sc->sc_rxlink = &ds->ds_link;
	bf->bf_status = 0;
	return 0;
}

/* This function calculates the presence of, and then removes any padding 
 * bytes between the frame header and frame body, and returns a modified 
 * SKB. */
static void
ath_skb_removepad(struct ieee80211com *ic, struct sk_buff *skb)
{
	struct ieee80211_frame *wh = (struct ieee80211_frame *)skb->data;
	unsigned int padbytes = 0, headersize = 0;
  	
	KASSERT(ic->ic_flags & IEEE80211_F_DATAPAD,
  		("data padding not enabled?"));

	/* Only non-control frames have bodies, and hence padding. */
	if (IEEE80211_FRM_HAS_BODY(wh)) {
		headersize = ieee80211_anyhdrsize(wh);
		padbytes = roundup(headersize, 4) - headersize;
		if (padbytes > 0) {
			memmove(skb->data + padbytes, skb->data, headersize);
			skb_pull(skb, padbytes);
		}
  	}
}

/*
 * Intercept management frames to collect beacon RSSI data and to do
 * ibss merges. This function is called for all management frames,
 * including those belonging to other BSS.
 */
static int
ath_recv_mgmt(struct ieee80211vap * vap, struct ieee80211_node *ni_or_null,
	struct sk_buff *skb, int subtype, int rssi, u_int64_t rtsf)
{
	struct ath_softc *sc = vap->iv_ic->ic_dev->priv;
	struct ieee80211_node * ni = ni_or_null;
	u_int64_t hw_tsf, beacon_tsf;
	u_int32_t hw_tu, beacon_tu, intval;
	int do_merge = 0;

	if (ni_or_null == NULL)
		ni = vap->iv_bss;
	DPRINTF(sc, ATH_DEBUG_BEACON,
		"vap:%p[" MAC_FMT "] ni:%p[" MAC_FMT "]\n",
		vap, MAC_ADDR(vap->iv_bssid),
		ni, MAC_ADDR(((struct ieee80211_frame *)skb->data)->i_addr2));

	/* Call up first so subsequent work can use information
	 * potentially stored in the node (e.g. for ibss merge). */
	if (sc->sc_recv_mgmt(vap, ni_or_null, skb, subtype, rssi, rtsf) == 0)
		return 0;

	/* Lookup the new node if any (this grabs a reference to it). */
	ni = ieee80211_find_rxnode(vap->iv_ic,
			(const struct ieee80211_frame_min *)skb->data);
	if (ni == NULL) {
		DPRINTF(sc, ATH_DEBUG_BEACON, "Dropping; node unknown.\n");
		return 0;
	}

	switch (subtype) {
	case IEEE80211_FC0_SUBTYPE_BEACON:
		/* Update beacon RSSI statistics, (apply to "pure" STA only)
		 * AND only for our AP's beacons */
		if (vap->iv_opmode == IEEE80211_M_STA &&
		    sc->sc_ic.ic_opmode == IEEE80211_M_STA &&
		    ni == vap->iv_bss)
			ATH_RSSI_LPF(sc->sc_halstats.ns_avgbrssi, rssi);
		if ((sc->sc_syncbeacon ||
		    (vap->iv_flags_ext & IEEE80211_FEXT_APPIE_UPDATE)) &&
		     ni == vap->iv_bss && vap->iv_state == IEEE80211_S_RUN) {
			/* Resync beacon timers using the TSF of the
			 * beacon frame we just received. */
			vap->iv_flags_ext &= ~IEEE80211_FEXT_APPIE_UPDATE;
			ath_beacon_config(sc, vap);
			DPRINTF(sc, ATH_DEBUG_BEACON, 
				"Updated beacon timers\n");
		}
		/* NB: Fall Through */
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
			hw_tu  = IEEE80211_TSF_TO_TU(hw_tsf);

			beacon_tsf = le64_to_cpu(ni->ni_tstamp.tsf);
			beacon_tu  = IEEE80211_TSF_TO_TU(beacon_tsf);

			DPRINTF(sc, ATH_DEBUG_BEACON,
					"Beacon transmitted at %10llx, "
					"received at %10llx(%lld), hw TSF "
					"%10llx(%lld)\n",
					beacon_tsf,
					rtsf, rtsf - beacon_tsf,
					hw_tsf, hw_tsf - beacon_tsf);

			if (beacon_tsf > rtsf) {
				DPRINTF(sc, ATH_DEBUG_BEACON,
						"IBSS merge: rtsf %10llx "
						"beacon's tsf %10llx\n",
						rtsf, beacon_tsf);
				do_merge = 1;
			}

			/* Check sc_nexttbtt */
			if (sc->sc_nexttbtt < hw_tu) {
				DPRINTF(sc, ATH_DEBUG_BEACON,
					"sc_nexttbtt (%8x TU) is in the past "
					"(tsf %8x TU), updating timers\n",
					sc->sc_nexttbtt, hw_tu);
				do_merge = 1;
			}

			intval = ni->ni_intval & HAL_BEACON_PERIOD;
			if (do_merge)
				ieee80211_ibss_merge(ni);
		}
		break;
	}

	ieee80211_unref_node(&ni);
	return 0;
}

static void
ath_setdefantenna(struct ath_softc *sc, u_int antenna)
{
	struct ath_hal *ah = sc->sc_ah;

	/* XXX: block beacon interrupts */
	ath_hal_setdefantenna(ah, antenna);
	if (sc->sc_rxantenna != antenna)
		sc->sc_stats.ast_ant_defswitch++;
	sc->sc_rxantenna = antenna;
	sc->sc_numrxotherant = 0;
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
	struct ath_hal *ah = sc ? sc->sc_ah : NULL;
	struct ath_desc *ds;
	struct ath_rx_status *rs;
	struct ieee80211_node *ni;
	struct sk_buff *skb = NULL;
	unsigned int len, phyerr, mic_fail = 0;
	int type = -1; /* undefined */
	int init_ret = 0;
	int bf_processed = 0;
	int skb_accepted = 0;
	int errors	 = 0;

	DPRINTF(sc, ATH_DEBUG_RX_PROC, "%s started...\n", __func__);
	do {
		/* Get next RX buffer pending processing by RX tasklet...
		 *  
		 * Descriptors are now processed at in the first-level interrupt
		 * handler to support U-APSD trigger search. This must also be
		 * done even when U-APSD is not active to support other error
		 * handling that requires immediate attention. We check
		 * bf_status to find out if the bf's descriptors have been
		 * processed by the interrupt handler and are ready for this
		 * tasklet to consume them.  We also never process/remove the
		 * self-linked entry at the end. */
		ATH_RXBUF_LOCK_IRQ(sc);
		bf = STAILQ_FIRST(&sc->sc_rxbuf);
		if (bf && (bf->bf_status & ATH_BUFSTATUS_RXDESC_DONE) &&
		    (bf->bf_desc->ds_link != bf->bf_daddr))
			STAILQ_REMOVE_HEAD(&sc->sc_rxbuf, bf_list);
		else {
			if (bf && (bf->bf_status & ATH_BUFSTATUS_RXDESC_DONE))
				EPRINTF(sc, "Warning: %s detected a non-empty "
						"skb that is self-linked. "
						"This may be a driver bug.\n",
						__func__);
			bf = NULL;
		}
		ATH_RXBUF_UNLOCK_IRQ(sc);
		if (!bf)
			break;

		bf_processed++;
		ds  = bf->bf_desc;

#ifdef AR_DEBUG
		if (sc->sc_debug & ATH_DEBUG_RECV_DESC)
			ath_printrxbuf(bf, 1);
#endif
		rs = &bf->bf_dsstatus.ds_rxstat;

		len = rs->rs_datalen;
		/* DMA sync. dies spectacularly if len == 0 */
		if (len == 0)
			goto rx_next;
		if (rs->rs_more) {
			/* Frame spans multiple descriptors; this
			 * cannot happen yet as we don't support
			 * jumbograms.  If not in monitor mode,
			 * discard the frame. */
#ifndef ERROR_FRAMES
			/* Enable this if you want to see
			 * error frames in Monitor mode. */
			if (ic->ic_opmode != IEEE80211_M_MONITOR) {
				sc->sc_stats.ast_rx_toobig++;
				errors++;
				goto rx_next;
			}
#endif
			/* Fall through for monitor mode handling... */
		} else if (rs->rs_status != 0) {
			errors++;
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
				/* Decrypt error.  If the error occurred
				 * because there was no hardware key, then
				 * let the frame through so the upper layers
				 * can process it.  This is necessary for 5210
				 * parts which have no way to setup a ``clear''
				 * key cache entry.
				 *
				 * XXX: Do key cache faulting. */
				if (rs->rs_keyix == HAL_RXKEYIX_INVALID)
					goto rx_accept;
				sc->sc_stats.ast_rx_badcrypt++;
			}
			if (rs->rs_status & HAL_RXERR_MIC) {
				sc->sc_stats.ast_rx_badmic++;
				mic_fail = 1;
				goto rx_accept;
			}

			/* Reject error frames if we have no vaps that
			 * are operating in monitor mode. */
			if (sc->sc_nmonvaps == 0)
				goto rx_next;
		}
rx_accept:
		skb_accepted++;
		
		/* Sync and unmap the frame.  At this point we're committed
		 * to passing the sk_buff somewhere so clear bf_skb; this means
		 * a new sk_buff must be allocated when the RX descriptor is
		 * setup again to receive another frame. 
		 * NB: Meta-data (rs, noise, tsf) in the ath_buf is still
		 * used. */
		bus_dma_sync_single(sc->sc_bdev,
			bf->bf_skbaddr, len, BUS_DMA_FROMDEVICE);
		skb = ath_rxbuf_take_skb(sc, bf);

		sc->sc_stats.ast_ant_rx[rs->rs_antenna]++;
		sc->sc_devstats.rx_packets++;
		sc->sc_devstats.rx_bytes += len;

		skb_put(skb, len);
		skb->protocol = __constant_htons(ETH_P_CONTROL);

		ath_skb_removepad(ic, skb);
		ieee80211_input_monitor(ic, skb, bf, 0 /* RX */, bf->bf_tsf, sc);

		/* Finished monitor mode handling, now reject error frames 
		 * before passing to other VAPs. Ignore MIC failures here, as 
		 * we need to recheck them. */
		if (rs->rs_status & ~(HAL_RXERR_MIC | HAL_RXERR_DECRYPT))
			goto rx_next;

		/* Remove the CRC. */
		skb_trim(skb, skb->len - IEEE80211_CRC_LEN);

		if (mic_fail) {
			struct ieee80211_frame_min *wh_m =
				(struct ieee80211_frame_min *)skb->data;
  			/* Ignore control frames which are reported with MIC 
  			 * error. */
			if ((wh_m->i_fc[0] & IEEE80211_FC0_TYPE_MASK) != 
						IEEE80211_FC0_TYPE_CTL) {
				ni = ieee80211_find_rxnode(ic, wh_m);
				if (ni) {
					if (ni->ni_table)
						ieee80211_check_mic(ni, skb);
					ieee80211_unref_node(&ni);
				}
			}

			mic_fail = 0;
			goto rx_next;
		}

		/* From this point on we assume the frame is at least
		 * as large as ieee80211_frame_min; verify that: */
		if (len < IEEE80211_MIN_LEN) {
			DPRINTF(sc, ATH_DEBUG_RECV, "Dropping short packet; length %d.\n",
				len);
			sc->sc_stats.ast_rx_tooshort++;
			errors++;
			goto rx_next;
		}

		/* Normal receive. */
		if (IFF_DUMPPKTS(sc, ATH_DEBUG_RECV))
			ieee80211_dump_pkt(ic, skb->data, skb->len,
					sc->sc_hwmap[rs->rs_rate].ieeerate,
					rs->rs_rssi, 0);

		{
			struct ieee80211_frame *wh =
				(struct ieee80211_frame *)skb->data;

			/* Only print beacons. */
			if ((skb->len >= sizeof(struct ieee80211_frame)) &&
			    ((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK)
			     == IEEE80211_FC0_TYPE_MGT) &&
			    ((wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK)
			     == IEEE80211_FC0_SUBTYPE_BEACON)) {

				DPRINTF(sc, ATH_DEBUG_BEACON,
					"RA:" MAC_FMT " TA:" MAC_FMT
					" BSSID:" MAC_FMT "\n",
					MAC_ADDR(wh->i_addr1),
					MAC_ADDR(wh->i_addr2),
					MAC_ADDR(wh->i_addr3));
			}
		}

		/* Update station stats/global stats for received frame if we 
		 * are on-channel. */
		if (!sc->sc_scanning && !(ic->ic_flags & IEEE80211_F_SCAN))
			ATH_RSSI_LPF(sc->sc_halstats.ns_avgrssi, rs->rs_rssi);

		/* Locate the node for sender, track state, and then pass the
		 * (referenced) node up to the 802.11 layer for its use.  If
		 * the sender is unknown spam the frame; it'll be dropped
		 * where it's not wanted. */
		if ((rs->rs_keyix != HAL_RXKEYIX_INVALID) &&
		    (ni = sc->sc_keyixmap[rs->rs_keyix]) != NULL) {
			/* Fast path: node is present in the key map;
			 * grab a reference for processing the frame. */
			ni = ieee80211_ref_node(ni);
		} else {
			/* No key index or no entry, do a lookup and
			 * add the node to the mapping table if possible. */
			ni = ieee80211_find_rxnode(ic,
				(const struct ieee80211_frame_min *)skb->data);
			if (ni != NULL) {
				/* If the station has a key cache slot assigned
				 * update the key->node mapping table. */
				ieee80211_keyix_t keyix =
					ni->ni_ucastkey.wk_keyix;
				if ((keyix != IEEE80211_KEYIX_NONE) &&
				    (sc->sc_keyixmap[keyix] == NULL))
					sc->sc_keyixmap[keyix] =
						ieee80211_ref_node(ni);
			}
		}

		/* If a node is found, dispatch, else, dispatch to all. */
		if (ni) {
			type = ieee80211_input(ni->ni_vap, ni, skb,
					rs->rs_rssi, bf->bf_tsf);
			ieee80211_unref_node(&ni);
		} else {
			type = ieee80211_input_all(ic, skb,
					rs->rs_rssi, bf->bf_tsf);
		}
		skb = NULL; /* SKB is no longer ours. */

		/* XXX: Why do this? */
		if (sc->sc_diversity) {
			/* When using hardware fast diversity, change the
			 * default RX antenna if RX diversity chooses the
			 * other antenna 3 times in a row. */
			if (sc->sc_rxantenna != rs->rs_antenna) {
				if (++sc->sc_numrxotherant >= 3)
					ath_setdefantenna(sc, rs->rs_antenna);
			} else
				sc->sc_numrxotherant = 0;
		}

		if (sc->sc_softled) {
			/* Blink for any data frame.  Otherwise do a
			 * heartbeat-style blink when idle.  The latter
			 * is mainly for station mode where we depend on
			 * periodic beacon frames to trigger the poll event. */
			if (type == IEEE80211_FC0_TYPE_DATA) {
				sc->sc_rxrate = rs->rs_rate;
				ath_led_event(sc, ATH_LED_RX);
			} else if (time_after(jiffies, sc->sc_ledevent + sc->sc_ledidle))
				ath_led_event(sc, ATH_LED_POLL);
		}
rx_next:
		/* SKBs that have not in a buf, and are not passed on. */
		ieee80211_dev_kfree_skb(&skb);

		KASSERT(bf != NULL, ("null bf"));

		if ((init_ret = ath_rxbuf_init(sc, bf)) != 0) {
			EPRINTF(sc, "Failed to reinitialize rxbuf: %d.  "
					"Lost an RX buffer!\n", init_ret);
			break;
		}

		/* Return the rx buffer to the queue... */
		ATH_RXBUF_LOCK_IRQ(sc);
		STAILQ_INSERT_TAIL(&sc->sc_rxbuf, bf, bf_list);
		ATH_RXBUF_UNLOCK_IRQ(sc);
	} while (1);

	if (sc->sc_useintmit) 
		ath_hal_rxmonitor(ah, &sc->sc_halstats, &sc->sc_curchan);
	if (!bf_processed)
		DPRINTF(sc, ATH_DEBUG_RX_PROC,
			"Warning: %s got scheduled when no receive "
			"buffers were ready. Were they cleared?\n",
			__func__);
	DPRINTF(sc, ATH_DEBUG_RX_PROC, "%s: cycle completed. "
		" %d rx buf processed. %d were errors. %d skb accepted.\n",
		__func__, bf_processed, errors, skb_accepted);
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
			EPRINTF(sc, "HAL hardware queue number, %u, is out of range."
				    "  The highest queue number is %u!\n",
				    qnum,
				    (unsigned)ARRAY_SIZE(sc->sc_txq));
			ath_hal_releasetxqueue(ah, qnum);
			return;
		}

		txq->axq_qnum = qnum;
	}
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
	struct ath_buf *bf = NULL, *head = NULL;
	struct ieee80211com *ic = vap->iv_ic;
	struct ath_softc *sc = ic->ic_dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	u_int8_t rate;
	unsigned int ctsrate = 0, ctsduration = 0;
	const HAL_RATE_TABLE *rt;
	u_int8_t cix, rtindex = 0;
	u_int type;
	struct ath_txq *txq = &sc->sc_grpplq;
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

	if (ath_cac_running_dbgmsg(sc))
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
		EPRINTF(sc, "grppoll buffer allocation failed\n");
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
						"sc_grppollbuf is empty!\n");
				ATH_TXBUF_UNLOCK_IRQ_EARLY(sc);
				return;
			}
			if (STAILQ_EMPTY(&sc->sc_grppollbuf)) {
				DPRINTF(sc, ATH_DEBUG_XMIT, 
						"sc_grppollbuf is empty!\n");
				ATH_TXBUF_UNLOCK_IRQ_EARLY(sc);
				return;
			}
			ATH_TXBUF_UNLOCK_IRQ(sc);

			bf->bf_skb = skb;
			bf->bf_desc->ds_data = bus_map_single(sc->sc_bdev,
				skb->data, skb->len, BUS_DMA_TODEVICE);;

			if ((i == pollcount) && (amode == (HAL_ANTENNA_MAX_MODE - 1))) {
				type = HAL_PKT_TYPE_NORMAL;
				flags |= (HAL_TXDESC_CLRDMASK | HAL_TXDESC_VEOL);
			} else {
				flags |= HAL_TXDESC_CTSENA;
				type = HAL_PKT_TYPE_GRP_POLL;
			}
			if ((i == 0) && (amode == HAL_ANTENNA_FIXED_A)) {
				flags |= HAL_TXDESC_CLRDMASK;
				head = bf;
			}

			ath_hal_setuptxdesc(ah, bf->bf_desc,
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
			ath_hal_filltxdesc(ah, bf->bf_desc,
				roundup(skb->len, 4),	/* buffer length */
				AH_TRUE,		/* first segment */
				AH_TRUE,		/* last segment */
				bf->bf_desc		/* first descriptor */
				);

			ath_desc_swap(bf->bf_desc);
			ATH_TXQ_LINK_DESC(txq, bf);
			ATH_TXQ_INSERT_TAIL(txq, bf, bf_list);

			pollsperrate++;
			if (pollsperrate > rates[rtindex]) {
				rtindex = (rtindex + 1) % MAX_GRPPOLL_RATE;
				pollsperrate = 0;
			}
		}
	}
	/* make it circular */
	bf->bf_desc->ds_link = ath_ds_link_swap(head->bf_daddr);
	/* start the queue */
	ath_hw_puttxbuf(sc, txq->axq_qnum, head->bf_daddr, __func__);
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
	if (((qtype == HAL_TX_QUEUE_DATA)|| (qtype == HAL_TX_QUEUE_UAPSD)) &&
	    ath_hal_compressionsupported(ah)) {
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
		EPRINTF(sc, "HAL hardware queue number, %u, is out of range."
			    "  The highest queue number is %u!\n",
			    qnum,
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
		EPRINTF(sc, "AC, %u, is out of range.  "
		        "The maximum AC is %u!\n",
		        ac, (unsigned)ARRAY_SIZE(sc->sc_ac2q));
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
	struct ieee80211com *ic = &sc->sc_ic;
	struct wmeParams *wmep = &ic->ic_wme.wme_chanParams.cap_wmeParams[ac];
	struct ath_hal *ah = sc->sc_ah;
	HAL_TXQ_INFO qi;

	ath_hal_gettxqueueprops(ah, txq->axq_qnum, &qi);
	qi.tqi_aifs = wmep->wmep_aifsn;
	qi.tqi_cwmin = (1 << wmep->wmep_logcwmin) - 1;
	qi.tqi_cwmax = (1 << wmep->wmep_logcwmax) - 1;
	qi.tqi_burstTime = wmep->wmep_txopLimit / 32; /* 32 us units. */

	if (!ath_hal_settxqueueprops(ah, txq->axq_qnum, &qi)) {
		EPRINTF(sc, "Unable to update hardware queue "
			"parameters for %s traffic!\n",
			ieee80211_wme_acnames[ac]);
		return 0;
	} else {
		ath_hal_resettxqueue(ah, txq->axq_qnum); /* push to h/w */
		return 1;
	}
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
		ath_tx_txqaddbuf(sc, ni, txq, bf, bf->bf_skb->len);
		an->an_uapsd_qdepth--;
	}

	while (an->an_uapsd_overflowqdepth) {
		bf = STAILQ_FIRST(&an->an_uapsd_overflowq);
		STAILQ_REMOVE_HEAD(&an->an_uapsd_overflowq, bf_list);
		bf->bf_desc->ds_link = 0;
		txq = sc->sc_ac2q[bf->bf_skb->priority & 0x3];
		ath_tx_txqaddbuf(sc, ni, txq, bf, bf->bf_skb->len);
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
	struct ath_buf *tbf;

	/* Case the delivery queue just sent and can move overflow q over. */
	if (an->an_uapsd_qdepth == 0 && an->an_uapsd_overflowqdepth != 0) {
		DPRINTF(sc, ATH_DEBUG_UAPSD,
			"Delivery queue empty, replacing with overflow queue\n");
		STAILQ_CONCAT(&an->an_uapsd_q, &an->an_uapsd_overflowq);
		an->an_uapsd_qdepth = an->an_uapsd_overflowqdepth;
		an->an_uapsd_overflowqdepth = 0;
	}

	/* Most common case - room on delivery q. */
	if (an->an_uapsd_qdepth < an->an_node.ni_uapsd_maxsp) {
		/* Add to delivery q. */
		ATH_STQ_LINK_DESC(&an->an_uapsd_q, bf);
		STAILQ_INSERT_TAIL(&an->an_uapsd_q, bf, bf_list);
		an->an_uapsd_qdepth++;

		DPRINTF(sc, ATH_DEBUG_UAPSD,
				"Added AC %d frame to delivery queue, "
				"new depth: %d\n", 
				bf->bf_skb->priority, an->an_uapsd_qdepth);
		return;
	}

	/* Check if need to make room on overflow queue. */
  	if (an->an_uapsd_overflowqdepth == an->an_node.ni_uapsd_maxsp) {
		/*  Pop oldest from delivery queue and cleanup. */
		tbf = STAILQ_FIRST(&an->an_uapsd_q);
  		STAILQ_REMOVE_HEAD(&an->an_uapsd_q, bf_list);
		ath_return_txbuf(sc, &tbf);
  
		/* Move oldest from overflow to delivery. */
		tbf = STAILQ_FIRST(&an->an_uapsd_overflowq);
  		STAILQ_REMOVE_HEAD(&an->an_uapsd_overflowq, bf_list);
  		an->an_uapsd_overflowqdepth--;
		STAILQ_INSERT_TAIL(&an->an_uapsd_q, tbf, bf_list);
  		DPRINTF(sc, ATH_DEBUG_UAPSD,
  			"Delivery and overflow queues full.  Dropped oldest.\n");
  	}
  
	/* Add to overflow q/ */
	ATH_STQ_LINK_DESC(&an->an_uapsd_overflowq, bf);
	STAILQ_INSERT_TAIL(&an->an_uapsd_overflowq, bf, bf_list);
	an->an_uapsd_overflowqdepth++;
	DPRINTF(sc, ATH_DEBUG_UAPSD, "Added AC %d to overflow queue, "
		"New depth: %d\n", 
		bf->bf_skb->priority, an->an_uapsd_overflowqdepth);

	return;
}

static int
ath_tx_start(struct net_device *dev, struct ieee80211_node *ni, 
		struct ath_buf *bf, struct sk_buff *skb, int nextfraglen)
{
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
	DPRINTF(sc, ATH_DEBUG_XMIT, "skb %p [data %p len %u] skbaddr %08llx\n",
		skb, skb->data, skb->len, (u_int64_t)bf->bf_skbaddr);
#else /* ATH_SUPERG_FF case */
	bf->bf_skbaddr = bus_map_single(sc->sc_bdev,
		skb->data, skb->len, BUS_DMA_TODEVICE);
	DPRINTF(sc, ATH_DEBUG_XMIT, "skb %p [data %p len %u] skbaddr %08llx\n",
		skb, skb->data, skb->len, (u_int64_t)bf->bf_skbaddr);
	/* NB: ensure skb->len had been updated for each skb so we don't need pktlen */
	{
		struct sk_buff *skbtmp = skb;
		unsigned int i = 0;

		while ((skbtmp = skbtmp->next)) {
			bf->bf_skbaddrff[i] = bus_map_single(sc->sc_bdev,
				skbtmp->data, skbtmp->len, BUS_DMA_TODEVICE);
			DPRINTF(sc, ATH_DEBUG_XMIT, "skb%d (FF) %p "
				"[data %p len %u] skbaddr %08llx\n", 
				i, skbtmp, skbtmp->data, skbtmp->len,
				(u_int64_t)bf->bf_skbaddrff[i]);
			i++;
		}
		bf->bf_numdescff = i;
	}
#endif /* ATH_SUPERG_FF */
	bf->bf_skb = skb;

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
		EPRINTF(sc, "Bogus frame type 0x%x\n", 
			wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK);
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
	if (ismcast && (vap->iv_ps_sta || STAILQ_FIRST(&avp->av_mcastq.axq_q))) {
		txq = &avp->av_mcastq;
		/* XXX: More bit in 802.11 frame header? */
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
			sc->sc_hwmap[txrate].ieeerate, -1, 1);

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
	    ((wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK) != IEEE80211_FC0_SUBTYPE_NULL)) {
		if (pktlen > ATH_COMP_THRESHOLD)
			comp = ATH_COMP_PROC_COMP_OPTIMAL;
		else
			comp = ATH_COMP_PROC_NO_COMP_ADD_CCS;
	}
#endif

	/* sc_txantenna == 0 means transmit diversity mode.
	 * sc_txantenna == 1 or sc_txantenna == 2 means the user has selected
	 * the first or second antenna port.
	 * If the user has set the txantenna, use it for multicast frames too. */
	if (ismcast && !sc->sc_txantenna) {
		/* Alternating antenna might be the wrong thing to do if we
		 * have one antenna that's significantly better than the other
		 *
		 * Use antenna in the ratio of the successfully sent unicast packets.
		 */
		if (sc->sc_mcastantenna > 0) {
			sc->sc_mcastantenna -= sc->sc_ant_tx[1] + 1;
			antenna = 2;
		} else {
			sc->sc_mcastantenna += sc->sc_ant_tx[2] + 1;
			antenna = 1;
		}
	} else
		antenna = sc->sc_txantenna;

	if (txrate == 0) {
		/* Drop frame, if the rate is 0.
		 * Otherwise this may lead to the continuous transmission of
		 * noise. */
		EPRINTF(sc, "Invalid transmission rate, %u.\n", txrate);
		return -EIO;
	}
	
	DPRINTF(sc, ATH_DEBUG_XMIT, "Set up txdesc: pktlen %d hdrlen %d "
		"atype %d txpower %d txrate %d try0 %d keyix %d ant %d flags %x "
		"ctsrate %d ctsdur %d icvlen %d ivlen %d comp %d\n",
		pktlen, hdrlen, atype, MIN(ni->ni_txpower, 60), txrate,
		try0, keyix, antenna, flags, ctsrate, ctsduration, icvlen, ivlen,
		comp);

	/* Formulate first tx descriptor with tx controls. */
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

		DPRINTF(sc, ATH_DEBUG_XMIT, "Set up multi rate/retry "
			"1:%d/%d 2:%d/%d 3:%d/%d\n", 
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

	DPRINTF(sc, ATH_DEBUG_XMIT, "Q%d: %08x %08x %08x %08x %08x %08x\n",
			M_FLAG_GET(skb, M_UAPSD) ? 0 : txq->axq_qnum,
			ds->ds_link, ds->ds_data,
			ds->ds_ctl0, ds->ds_ctl1,
			ds->ds_hw[0], ds->ds_hw[1]);
#else /* ATH_SUPERG_FF */
	{
		struct sk_buff *tskb;
		struct ath_desc *ds0 = ds;
		unsigned int i;

		for (i = 0, tskb = skb; 
				i < (bf->bf_numdescff + 1);
				i++, tskb = tskb->next, ds++) {
			/* Link to the next, (i + 1)th, desc. if it exists. */
			ds->ds_link = (tskb->next == NULL) ? 
				0 : bf->bf_daddr + ((i + 1) * sizeof(*ds));
			ds->ds_data = (i == 0) ?
				bf->bf_skbaddr : bf->bf_skbaddrff[i - 1];

			ath_hal_filltxdesc(ah, ds,
				tskb->len,		/* Segment length */
				(i == 0),		/* First segment? */
				(tskb->next == NULL),	/* Last segment? */
				ds0			/* First descriptor */
			);
			ath_desc_swap(ds);

			DPRINTF(sc, ATH_DEBUG_XMIT, "Q%d: (ds)%p (lk)%08x "
				"(d)%08x (c0)%08x (c1)%08x %08x %08x\n", 
				M_FLAG_GET(tskb, M_UAPSD) ? 0 : txq->axq_qnum,
				ds, ds->ds_link, ds->ds_data, ds->ds_ctl0,
				ds->ds_ctl1, ds->ds_hw[0], ds->ds_hw[1]);
		}
	}
#endif

	if (M_FLAG_GET(skb, M_UAPSD)) {
		/* must lock against interrupt-time processing (i.e., not just tasklet) */
		ATH_NODE_UAPSD_LOCK_IRQ(an);
		DPRINTF(sc, ATH_DEBUG_UAPSD, 
			"Queueing U-APSD data frame for node " MAC_FMT " \n",
			MAC_ADDR(an->an_node.ni_macaddr));
		ath_tx_uapsdqueue(sc, an, bf);
		if (IEEE80211_NODE_UAPSD_USETIM(ni) && (an->an_uapsd_qdepth == 1))
			vap->iv_set_tim(ni, 1);
		ATH_NODE_UAPSD_UNLOCK_IRQ(an);

		return 0;
	}

	ath_tx_txqaddbuf(sc, PASS_NODE(ni), txq, bf, pktlen);
	return 0;
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

	DPRINTF(sc, ATH_DEBUG_TX_PROC, "TX queue: %d (0x%x), link: %08x\n", 
		txq->axq_qnum, ath_hal_gettxbuf(sc->sc_ah, txq->axq_qnum),
		ath_get_last_ds_link(txq));

	if (txq == sc->sc_uapsdq) {
		DPRINTF(sc, ATH_DEBUG_UAPSD, "Reaping U-APSD txq\n");
		uapsdq = 1;
	}

	while (1) {
		ATH_TXQ_LOCK_IRQ(txq);

		txq->axq_intrcnt = 0; /* reset periodic desc intr count */
		bf = STAILQ_FIRST(&txq->axq_q);
		if (bf == NULL) {
			ATH_TXQ_UNLOCK_IRQ_EARLY(txq);
			goto bf_fail;
		}

#ifdef ATH_SUPERG_FF
		ds = &bf->bf_desc[bf->bf_numdescff];
		DPRINTF(sc, ATH_DEBUG_TX_PROC, "Frame's last desc: %p\n",
			ds);
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

		/* We make sure we don't remove the TX descriptor on
		 * which the HW is pointing since it contains the
		 * ds_link field, except if this is the last TX
		 * descriptor in the queue */
		if ((txq->axq_depth > 1) && (bf->bf_daddr == 
					ath_hal_gettxbuf(ah, txq->axq_qnum))) {
			ATH_TXQ_UNLOCK_IRQ_EARLY(txq);
			goto bf_fail;
		}

		ATH_TXQ_REMOVE_HEAD(txq, bf_list);
		ATH_TXQ_UNLOCK_IRQ(txq);

		ni = ATH_BUF_NI(bf);
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
				/* Update HAL stats for ANI, only when on-channel */
				if (!sc->sc_scanning && 
				    !(sc->sc_ic.ic_flags & IEEE80211_F_SCAN))
					ATH_RSSI_LPF(sc->sc_halstats.ns_avgtxrssi,
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
							"Frame in SP "
							"retried out, "
							"possible EOSP "
							"stranded!!!\n");
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
						"ath_buf with no node!"));
			if (qwh->i_qos[0] & IEEE80211_QOS_EOSP) {
				DPRINTF(sc, ATH_DEBUG_UAPSD, 
					"EOSP detected for node (" MAC_FMT ") on desc %p\n",
					MAC_ADDR(ni->ni_macaddr), ds);
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
					"Hardware assigned sequence number is "
					"not sane (%d), ignoring it\n", 
					ts->ts_seqnum);
			} else {
				DPRINTF(sc, ATH_DEBUG_TX_PROC, 
					"Updating frame's sequence number "
					"from %d to %d\n", 
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
#define TSTAMP_TX_MASK  ((2 ^ (27 - 1)) - 1)    /* First 27 bits. */ 

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

			/* HW is now finished with the SKB, so it is safe to
			 * remove padding. */
			ath_skb_removepad(&sc->sc_ic, skb);
			DPRINTF(sc, ATH_DEBUG_TX_PROC, "capture skb %p\n",
					bf->bf_skb);
			tskb = skb->next;
			ieee80211_input_monitor(&sc->sc_ic, skb, bf,
					1 /* TX */, bf->bf_tsf, sc);
			skb = tskb;

#ifdef ATH_SUPERG_FF
			/* Handle every skb after the first one - these are FF 
			 * extra buffers */
			for (i = 0; i < bf->bf_numdescff; i++) {
				ath_skb_removepad(&sc->sc_ic, skb); /* XXX: padding for FF? */
				DPRINTF(sc, ATH_DEBUG_TX_PROC, "capture skb %p\n",
					skb);
				tskb = skb->next;
				ieee80211_input_monitor(&sc->sc_ic, skb, bf,
						1 /* TX */, bf->bf_tsf, sc);
				skb = tskb;
			}
			bf->bf_numdescff = 0;
#endif
		}

		ni = NULL;
		ath_return_txbuf(sc, &bf);
	}
bf_fail:
#ifdef ATH_SUPERG_FF
	/* flush ff staging queue if buffer low */
	if (txq->axq_depth <= sc->sc_fftxqmin - 1)
		/* NB: consider only flushing a preset number based on age. */
		ath_ffstageq_flush(sc, txq, ath_ff_neverflushtestdone);

#else
	;
#endif /* ATH_SUPERG_FF */
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
	if (ATH_TXQ_SETUP(sc, sc->sc_cabq->axq_qnum) &&
	    STAILQ_FIRST(&sc->sc_cabq->axq_q)) {
		DPRINTF(sc, ATH_DEBUG_BEACON,
			"Processing CABQ... it is active in HAL.\n");
		ath_tx_processq(sc, sc->sc_cabq);
	}
	else {
		DPRINTF(sc, ATH_DEBUG_BEACON,
			"NOT processing CABQ... it is %s.\n",
			sc->sc_cabq->axq_depth ? "not setup" : "empty");
	}
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

	for (i = 0; i < HAL_NUM_TX_QUEUES; i++) {
		if (!ath_txq_check(sc, &sc->sc_txq[i], __func__))
			ath_txq_dump(sc, &sc->sc_txq[i]);
	}

	/* Process each active queue. This includes sc_cabq, sc_xrtq and
	 * sc_uapsdq */
	for (i = 0; i < HAL_NUM_TX_QUEUES; i++) {
		if (ATH_TXQ_SETUP(sc, i) && (txqactive(sc->sc_ah, i) ||
					(sc->sc_cabq->axq_qnum == i))) {
			if (sc->sc_cabq->axq_qnum == i)
				DPRINTF(sc, ATH_DEBUG_BEACON,
					"Processing CABQ... it is setup and active in HAL.\n");
			ath_tx_processq(sc, &sc->sc_txq[i]);
		}
		else if (sc->sc_cabq->axq_qnum == i) { /* is CABQ */
			DPRINTF(sc, ATH_DEBUG_BEACON,
				"NOT processing CABQ... it is %s.\n",
				STAILQ_FIRST(&sc->sc_cabq->axq_q) ? "not setup" : "empty");
		}
	}
	netif_wake_queue(dev);

	if (sc->sc_softled)
		ath_led_event(sc, ATH_LED_TX);
}

static void
ath_tx_timeout(struct net_device *dev)
{
	struct ath_softc *sc = dev->priv;
	int i;

	if (ath_chan_unavail(sc))
		return;

	DPRINTF(sc, ATH_DEBUG_WATCHDOG, "%sRUNNING.  sc is %svalid.\n",
		(dev->flags & IFF_RUNNING) ? "" : "NOT ",
		sc->sc_invalid ? "in" : "");

	for (i = 0; i < HAL_NUM_TX_QUEUES; i++) {
		if (ATH_TXQ_SETUP(sc, i)) {
			ath_txq_check(sc, &sc->sc_txq[i], __func__);
			ath_txq_dump(sc, &sc->sc_txq[i]);
		}
	}

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
	struct ath_buf *bf;
	/*
	 * NB: this assumes output has been stopped and
	 *     we do not need to block ath_tx_tasklet
	 */
	for (;;) {
		ATH_TXQ_LOCK_IRQ(txq);
		bf = STAILQ_FIRST(&txq->axq_q);
		if (bf == NULL) {
			ATH_TXQ_UNLOCK_IRQ_EARLY(txq);
			return;
		}
		ATH_TXQ_REMOVE_HEAD(txq, bf_list);
		ATH_TXQ_UNLOCK_IRQ(txq);
#ifdef AR_DEBUG
		if (sc->sc_debug & ATH_DEBUG_RESET)
			ath_printtxbuf(bf, ath_hal_txprocdesc(sc->sc_ah, bf->bf_desc, 
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
	DPRINTF(sc, ATH_DEBUG_RESET, "TX queue [%u] 0x%x, link %08x\n",
		txq->axq_qnum,
		ath_hal_gettxbuf(ah, txq->axq_qnum),
		ath_get_last_ds_link(txq));
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
			"Invoking ath_hal_stoptxdma with sc_bhalq:%d.\n",
			sc->sc_bhalq);
		(void) ath_hal_stoptxdma(ah, sc->sc_bhalq);
		DPRINTF(sc, ATH_DEBUG_RESET, "Beacon queue txbuf is 0x%x.\n",
			ath_hal_gettxbuf(ah, sc->sc_bhalq));
		for (i = 0; i < HAL_NUM_TX_QUEUES; i++)
			if (ATH_TXQ_SETUP(sc, i))
				ath_tx_stopdma(sc, &sc->sc_txq[i]);
	}
	sc->sc_dev->trans_start = jiffies;
	netif_wake_queue(sc->sc_dev);		/* XXX move to callers */
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

	ath_hal_stoppcurecv(ah);	/* disable PCU */
	ath_hal_setrxfilter(ah, 0);	/* clear recv filter */
	ath_hal_stopdmarecv(ah);	/* disable DMA engine */
	mdelay(3);			/* 3 ms is long enough for 1 frame */
#ifdef AR_DEBUG
	if (sc->sc_debug & (ATH_DEBUG_RESET | ATH_DEBUG_FATAL)) {
		struct ath_buf *bf;

		DPRINTF(sc, ATH_DEBUG_ANY, "receive queue buffer 0x%x, link %p\n",
			ath_hal_getrxbuf(ah), sc->sc_rxlink);
		ATH_RXBUF_LOCK_IRQ(sc);
		STAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list) {
			struct ath_desc *ds = bf->bf_desc;
			struct ath_rx_status *rs = &bf->bf_dsstatus.ds_rxstat;
			HAL_STATUS status = ath_hal_rxprocdesc(ah, ds,
				bf->bf_daddr, PA2DESC(sc, ds->ds_link), bf->bf_tsf, rs);
			if (status == HAL_OK || (sc->sc_debug & ATH_DEBUG_FATAL))
				ath_printrxbuf(bf, status == HAL_OK);
		}
		ATH_RXBUF_UNLOCK_IRQ(sc);
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
	DPRINTF(sc, ATH_DEBUG_RESET, "RX settings: mtu %u cachelsz %u rxbufsize %u\n",
		dev->mtu, sc->sc_cachelsz, sc->sc_rxbufsize);

	sc->sc_rxlink = NULL;
	ATH_RXBUF_LOCK_IRQ(sc);
	STAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list) {
		int error = ath_rxbuf_init(sc, bf);
		if (error < 0) {
			ATH_RXBUF_UNLOCK_IRQ_EARLY(sc);
			return error;
		}
	}
	ATH_RXBUF_UNLOCK_IRQ(sc);

	sc->sc_rxbufcur = NULL;

	bf = STAILQ_FIRST(&sc->sc_rxbuf);
	ath_hal_putrxbuf(ah, bf->bf_daddr);
	ath_hal_rxena(ah);		/* enable recv descriptors */
	ath_mode_init(dev);		/* set filters, etc. */
	ath_hal_startpcurecv(ah);	/* re-enable PCU/DMA engine */
	ath_override_intmit_if_disabled(sc);
	return 0;
}

/*
 * Flush skbs allocated for receiving.
 */
static void
ath_flushrecv(struct ath_softc *sc)
{
	struct ath_buf *bf;
	ATH_RXBUF_LOCK_IRQ(sc);
	STAILQ_FOREACH(bf, &sc->sc_rxbuf, bf_list)
		cleanup_ath_buf(sc, bf, BUS_DMA_FROMDEVICE);
	ATH_RXBUF_UNLOCK_IRQ(sc);
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
	/* Reset noise floor on channel change and let ieee layer know */
	ath_hal_process_noisefloor(sc->sc_ah);
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
	u_int8_t dfs_cac_needed = 0;
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
		"Changing channels from %3d (%4d MHz) to %3d (%4d MHz) -- Time: %ld.%06ld\n",
		ath_hal_mhz2ieee(ah, sc->sc_curchan.channel,
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


	/* Stop any pending channel calibrations or availability check if we
	 * are really changing channels.  maybe a turbo mode switch only. */
	if (hchan.channel != sc->sc_curchan.channel)
		if (!sc->sc_dfs_testmode && sc->sc_dfs_cac)
			ath_interrupt_dfs_cac(sc, 
					"Channel change interrupted DFS wait.");

	/* Need a DFS channel availability check?  We do if ... */
	dfs_cac_needed = IEEE80211_IS_MODE_DFS_MASTER(ic->ic_opmode) &&
		(hchan.channel != sc->sc_curchan.channel ||
		/* the scan wasn't already done */
		 (0 == (sc->sc_curchan.privFlags & CHANNEL_DFS_CLEAR))) &&
		/* the new channel requires DFS protection */
		ath_radar_is_dfs_required(sc, &hchan) &&
		(ic->ic_flags & IEEE80211_F_DOTH);

	channel_change_required = hchan.channel != sc->sc_curchan.channel ||
		hchan.channelFlags != sc->sc_curchan.channelFlags ||
		tswitch || dfs_cac_needed;

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
		if (!ath_hw_reset(sc, sc->sc_opmode, &hchan, AH_TRUE, &status)   ||
		    !ath_hw_reset(sc, sc->sc_opmode, &hchan, AH_FALSE, &status)) {
			EPRINTF(sc, "Unable to reset channel %u (%u MHz) "
				"flags 0x%x '%s' (HAL status %u)\n",
				ieee80211_chan2ieee(ic, chan), chan->ic_freq,
				hchan.channelFlags,
				ath_get_hal_status_desc(status), status);
			return -EIO;
		}

		/* Change channels and update the h/w rate map
		 * if we're switching; e.g. 11a to 11b/g. */
		ath_chan_change(sc, chan);
		 /* Re-enable rx framework. */
		if (ath_startrecv(sc) != 0) {
			EPRINTF(sc, "Unable to restart receive logic!\n");
			return -EIO;
		}

		do_gettimeofday(&tv);
		if (dfs_cac_needed && !(ic->ic_flags & IEEE80211_F_SCAN)) {
			DPRINTF(sc, ATH_DEBUG_STATE | ATH_DEBUG_DOTH, 
					"Starting DFS wait for "
					"channel %u -- Time: %ld.%06ld\n", 
					ieee80211_mhz2ieee(sc->sc_curchan.channel, 
						sc->sc_curchan.channelFlags), 
					tv.tv_sec, tv.tv_usec);
			/* set the timeout to normal */
			dev->watchdog_timeo = 120 * HZ;
			/* Disable beacons and beacon miss interrupts */
			sc->sc_dfs_cac = 1;
			sc->sc_beacons = 0;
			sc->sc_imask &= ~(HAL_INT_SWBA | HAL_INT_BMISS);
			ath_hal_intrset(ah, sc->sc_imask);

			/* Enter DFS wait period */
			mod_timer(&sc->sc_dfs_cac_timer,
				jiffies + (sc->sc_dfs_cac_period * HZ));
		}
		/*
		 * re configure beacons when it is a turbo mode switch.
		 * HW seems to turn off beacons during turbo mode switch.
		 */
		if (sc->sc_beacons && !sc->sc_dfs_cac)
			ath_beacon_config(sc, NULL);
		/*
		 * Re-enable interrupts.
		 */
		ath_hal_intrset(ah, sc->sc_imask);
	}
	else
		DPRINTF(sc, ATH_DEBUG_STATE | ATH_DEBUG_DOTH, 
				"Not performing channel change action: "
				"%d -- Time: %ld.%06ld\n", 
				ieee80211_mhz2ieee(sc->sc_curchan.channel, 
					sc->sc_curchan.channelFlags), 
				tv.tv_sec, tv.tv_usec);
	return 0;
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
			"Channel %u/%x - periodic recalibration\n", 
			sc->sc_curchan.channel, 
			sc->sc_curchan.channelFlags);

	ath_hal_process_noisefloor(ah);
	ic->ic_channoise = ath_hal_get_channel_noise(ah, &(sc->sc_curchan));

	if (ath_hal_getrfgain(ah) == HAL_RFGAIN_NEED_CHANGE) {
		/*
		 * Rfgain is out of bounds, reset the chip
		 * to load new gain values.
		 */
		int txcont_was_active = sc->sc_txcont;
		DPRINTF(sc, ATH_DEBUG_RESET | ATH_DEBUG_CALIBRATE | 
				ATH_DEBUG_DOTH,
			"Forcing reset() for (ath_hal_getrfgain(ah) == "
			"HAL_RFGAIN_NEED_CHANGE)\n");
		sc->sc_stats.ast_per_rfgain++;

		/* Even if beacons were not enabled presently,
		 * set sc->beacons if we might need to restart
                 * them after ath_reset. */
		if (!sc->sc_beacons &&
				(TAILQ_FIRST(&ic->ic_vaps)->iv_opmode != 
				 IEEE80211_M_WDS) &&
				!txcont_was_active &&
				!sc->sc_dfs_cac) {
			sc->sc_beacons = 1;
		}

		ath_reset(dev);
		/* Turn txcont back on as necessary */
		if (txcont_was_active)
			ath_set_txcont(ic, txcont_was_active);

	}
	else if (ath_hal_getrfgain(ah) == HAL_RFGAIN_READ_REQUESTED) {
		/* With current HAL, I've never seen this so I'm going to log it
		 * as an error and see if it ever shows up with newer HAL. */
#if 0
		EPRINTF(sc, "Calibration of channel %u skipped!  "
			    "HAL_RFGAIN_READ_REQUESTED pending!\n",
			sc->sc_curchan.channel);
#endif
	}
	else {
		if (!ath_hal_calibrate(ah, &sc->sc_curchan, &isIQdone)) {
			EPRINTF(sc, "Calibration of channel %u failed!\n",
				sc->sc_curchan.channel);
			sc->sc_stats.ast_per_calfail++;
		}

		/* Update calibration interval based on whether I gain and Q 
		 * gain adjustments completed. */
		sc->sc_calinterval_sec = (isIQdone == AH_TRUE) ? 
			ATH_LONG_CALINTERVAL_SECS : 
			ATH_SHORT_CALINTERVAL_SECS;
	}

	DPRINTF(sc, ATH_DEBUG_CALIBRATE, "Channel %u [flags=%04x] -- IQ %s.\n",
		sc->sc_curchan.channel, sc->sc_curchan.channelFlags,
		isIQdone ? "done" : "not done");
	sc->sc_lastcal = jiffies;
	if (!sc->sc_beacon_cal) {
		mod_timer(&sc->sc_cal_ch, jiffies + (sc->sc_calinterval_sec * HZ));
	}
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

	DPRINTF(sc, ATH_DEBUG_STATE, "RX filter set to 0x%x BSSID " MAC_FMT " AID 0\n",
		 rfilt, MAC_ADDR(dev->broadcast));
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

	DPRINTF(sc, ATH_DEBUG_STATE, "RX filter set to 0x%x BSSID " MAC_FMT " AID 0x%x\n",
		 rfilt, MAC_ADDR(sc->sc_curbssid),
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

	DPRINTF(sc, ATH_DEBUG_STATE, "%p[%s] %s -> %s\n",
		vap, vap->iv_nickname,
		ieee80211_state_name[vap->iv_state],
		ieee80211_state_name[nstate]);

	if (!sc->sc_beacon_cal)
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
			ath_hal_intrset(ah, sc->sc_imask & ~HAL_INT_GLOBAL);
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
		IEEE80211_ADDR_COPY(sc->sc_curbssid, vap->iv_bssid);
		DPRINTF(sc, ATH_DEBUG_BEACON,
			"sc_curbssid changed to " MAC_FMT " (Node " MAC_FMT ")\n",
			MAC_ADDR(vap->iv_bssid),
			MAC_ADDR(ni->ni_macaddr));
	} else
		sc->sc_curaid = 0;

	DPRINTF(sc, ATH_DEBUG_STATE, "Set RX filter to 0x%x for BSSID " MAC_FMT " AID 0x%x\n",
		 rfilt, MAC_ADDR(sc->sc_curbssid),
		 sc->sc_curaid);

	ath_hal_setrxfilter(ah, rfilt);
	if (stamode) {
		DPRINTF(sc, ATH_DEBUG_BEACON,
			"Set association ID for " MAC_FMT " to %d\n",
			MAC_ADDR(sc->sc_curbssid), sc->sc_curaid);
		ath_hal_setassocid(ah, sc->sc_curbssid, sc->sc_curaid);
	}

	if ((vap->iv_opmode != IEEE80211_M_STA) &&
			(vap->iv_flags & IEEE80211_F_PRIVACY)) {
		for (i = 0; i < IEEE80211_WEP_NKID; i++)
			if (ath_hal_keyisvalid(ah, i))
				ath_hal_keysetmac(ah, i, vap->iv_bssid);
	}

	/*
	 * Notify the rate control algorithm so rates
	 * are setup should ath_beacon_alloc be called.
	 */
	sc->sc_rc->ops->newstate(vap, nstate);

	if (vap->iv_opmode == IEEE80211_M_MONITOR) {
		/* nothing to do */;
	} else if (nstate == IEEE80211_S_RUN) {
		/*
		 * Reset RSSI stats (regardless of mode)...
		 */
		sc->sc_halstats.ns_avgbrssi = ATH_RSSI_DUMMY_MARKER;
		sc->sc_halstats.ns_avgrssi = ATH_RSSI_DUMMY_MARKER;
		sc->sc_halstats.ns_avgtxrssi = ATH_RSSI_DUMMY_MARKER;

		DPRINTF(sc, ATH_DEBUG_STATE,
			"%s->%s: ic_flags=0x%08x iv=%d BSSID=" MAC_FMT
			" capinfo=0x%04x chan=%d\n",
			 ieee80211_state_name[vap->iv_state],
			 ieee80211_state_name[nstate],
			 vap->iv_flags,
			 ni->ni_intval,
			 MAC_ADDR(vap->iv_bssid),
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
				"Invoking ath_hal_stoptxdma with sc_bhalq: %d\n",
				sc->sc_bhalq);
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
			if (!tmpvap || sc->sc_nostabeacons)
				sc->sc_beacons = 0;

			DPRINTF(sc, ATH_DEBUG_STATE | 
				ATH_DEBUG_BEACON | 
				ATH_DEBUG_BEACON_PROC, 
				"VAP %p[%s] transitioned to RUN state and "
				"we %s reconfiguring beacons.\n",
				vap, vap->iv_nickname, 
				sc->sc_beacons ? "ARE NOT" : "ARE");

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

			/* When we are in AP mode and we have a station,
			 * any time the station is in SCAN, AUTH, or ASSOC
			 * states - all AP VAPs are put down into INIT state
			 * and marked 'scan pending'.  So, every time the STA
			 * VAP returns to RUN state on an AP mode radio, we
			 * are almost certain to have a minimum of one AP VAP
			 * returning to RUN state, and needing beacons reset.
			 * Without this, a STA can come back up and leave the
			 * AP VAP in a state where it is not beaconing. */
			if (ic->ic_opmode == IEEE80211_M_HOSTAP || 
			    sc->sc_nostabeacons) 
				sc->sc_beacons = 0;

			DPRINTF(sc, ATH_DEBUG_STATE | 
				    ATH_DEBUG_BEACON | 
				    ATH_DEBUG_BEACON_PROC, 
				"VAP %p[%s] transitioned to RUN state and "
				"we %s reconfiguring beacons.\n",
				vap, vap->iv_nickname, 
				sc->sc_beacons ? "ARE NOT" : "ARE");

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

		/* if it is a DFS channel and has not been checked for radar
		 * do not let the 80211 state machine to go to RUN state. */
		if (sc->sc_dfs_cac &&
				IEEE80211_IS_MODE_DFS_MASTER(vap->iv_opmode)) {
			DPRINTF(sc, ATH_DEBUG_STATE | ATH_DEBUG_DOTH, 
				"VAP -> DFSWAIT_PENDING \n");
			/* start calibration timer with a really small value 
			 * 1/10 sec */
			if (!sc->sc_beacon_cal)
				mod_timer(&sc->sc_cal_ch, jiffies + (HZ / 10));
			/* wake the receiver */
			netif_wake_queue(dev);
			/* don't do the other usual stuff... */
			ieee80211_cancel_scan(vap);
			return 0;
		}

		/* Configure the beacon and sleep timers. */
		if (!sc->sc_beacons && (vap->iv_opmode != IEEE80211_M_WDS)) {
			DPRINTF(sc, ATH_DEBUG_STATE | ATH_DEBUG_BEACON | 
					ATH_DEBUG_BEACON_PROC, 
				"Beacons reconfigured by %p[%s]!\n",
				vap, vap->iv_nickname);
			ath_beacon_config(sc, vap);
			sc->sc_beacons = 1;
		}
	} else {
		if (sc->sc_dfs_cac &&
		    IEEE80211_IS_MODE_DFS_MASTER(vap->iv_opmode) &&
		    (sc->sc_dfs_cac_timer.data == (unsigned long)vap))
		{
			del_timer_sync(&sc->sc_dfs_cac_timer);
			sc->sc_dfs_cac = 0;
			DPRINTF(sc, ATH_DEBUG_STATE, 
					"VAP DFSWAIT_PENDING -> run\n");
		}

		/* XXX: if it is SCAN state, disable beacons. */
		if (nstate == IEEE80211_S_SCAN) {
			sc->sc_imask &= ~(HAL_INT_SWBA | HAL_INT_BMISS);
			ath_hal_intrset(ah, sc->sc_imask);
			/* need to reconfigure the beacons when it moves to 
			 * RUN */
			sc->sc_beacons = 0;
		}
	}

done:
	/* Invoke the parent method to complete the work. */
	error = avp->av_newstate(vap, nstate, arg);

	/* Reset halstats on state change (per freebsd sources) */
	sc->sc_halstats.ns_avgbrssi = ATH_RSSI_DUMMY_MARKER;
	sc->sc_halstats.ns_avgrssi = ATH_RSSI_DUMMY_MARKER;
	sc->sc_halstats.ns_avgtxrssi = ATH_RSSI_DUMMY_MARKER; 
  	/* Finally, start any timers. */
	if ((nstate == IEEE80211_S_RUN) && !sc->sc_beacon_cal) {
		/* start periodic recalibration timer */
		mod_timer(&sc->sc_cal_ch, jiffies + (sc->sc_calinterval_sec * HZ));
	}

#ifdef ATH_SUPERG_XR
	if (vap->iv_flags & IEEE80211_F_XR &&
		nstate == IEEE80211_S_RUN)
		ATH_SETUP_XR_VAP(sc, vap, rfilt);
	if (vap->iv_flags & IEEE80211_F_XR &&
		nstate == IEEE80211_S_INIT && sc->sc_xrgrppoll)
		ath_grppoll_stop(vap);
#endif
bad:
	netif_wake_queue(dev);
	dev->watchdog_timeo = (sc->sc_dfs_cac ? 120 : 5) * HZ;		/* set the timeout to normal */
	return error;
}

/* periodically checks for the HAL to set
 * CHANNEL_DFS_CLEAR flag on current channel.
 * if the flag is set and a VAP is waiting for it, push
 * transition the VAP to RUN state.
 *
 * Context: Timer (softIRQ) */
static void
ath_dfs_cac_completed(unsigned long data )
{
	struct ath_softc *sc = (struct ath_softc *)data;
	struct ieee80211com *ic = &sc->sc_ic;
	struct net_device *dev = sc->sc_dev;
	struct ieee80211vap *vap ;
	struct timeval tv;

	if (!sc->sc_dfs_cac) {
		DPRINTF(sc, ATH_DEBUG_DOTH, "DFS wait timer "
				"expired, but the driver didn't think we "
				"were in dfswait.  Somebody forgot to "
				"delete the DFS wait timer.\n");
		return;
	}

	if (!sc->sc_dfs_testmode) {
		do_gettimeofday(&tv);
		DPRINTF(sc, ATH_DEBUG_STATE | ATH_DEBUG_DOTH, 
				"DFS wait %s! - Channel: %u Time: "
				"%ld.%06ld\n", 
				(sc->sc_curchan.privFlags & CHANNEL_DFS) ?
					"completed" : "not applicable", 
					ieee80211_mhz2ieee(sc->sc_curchan.channel, 
						sc->sc_curchan.channelFlags), 
					tv.tv_sec, tv.tv_usec);
		sc->sc_dfs_cac = 0;
		if (sc->sc_curchan.privFlags & CHANNEL_INTERFERENCE) {
			DPRINTF(sc, ATH_DEBUG_DOTH, 
					"DFS wait timer expired "
					"but channel was already marked as "
					"having CHANNEL_INTERFERENCE.  "
					"Somebody forgot to delete the DFS "
					"wait timer.\n");
			return;
		}
		if (0 == (sc->sc_curchan.privFlags & CHANNEL_DFS)) {
			DPRINTF(sc, ATH_DEBUG_DOTH, "DFS wait "
					"timer expired but the current "
					"channel does not require DFS.  "
					"Maybe someone changed channels "
					"but forgot to cancel the DFS "
					"wait.\n");
			return;
		}
		DPRINTF(sc, ATH_DEBUG_DOTH, "Driver is now MARKING "
				"channel as CHANNEL_DFS_CLEAR.\n");
		sc->sc_curchan.privFlags |= CHANNEL_DFS_CLEAR;
		ath_chan_change(sc, ic->ic_curchan);
		/* restart each VAP that was pending... */
		TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) {
			struct ath_vap *avp = ATH_VAP(vap);
			if (IEEE80211_IS_MODE_DFS_MASTER(vap->iv_opmode)) {
				int error;
				DPRINTF(sc, ATH_DEBUG_STATE | ATH_DEBUG_DOTH, 
						"VAP DFSWAIT_PENDING "
						"-> RUN -- Time: %ld.%06ld\n", 
						tv.tv_sec, tv.tv_usec);
				/* re alloc beacons to update new channel info */
				error = ath_beacon_alloc(sc, vap->iv_bss);
				if (error < 0) {
					EPRINTF(sc, "Beacon allocation "
							"failed: %d\n", error);
					return;
				}
				if (!sc->sc_beacons &&
				    (vap->iv_opmode != IEEE80211_M_WDS))
					sc->sc_beacons = 1;
				avp->av_newstate(vap, IEEE80211_S_RUN, 0);
#ifdef ATH_SUPERG_XR
				if (vap->iv_flags & IEEE80211_F_XR) {
					u_int32_t rfilt = 0;
					rfilt = ath_calcrxfilter(sc);
					ATH_SETUP_XR_VAP(sc, vap, rfilt);
				}
#endif
			}
		}
		netif_wake_queue(dev);
		ath_reset(dev);
		if (sc->sc_beacons) {
			ath_beacon_config(sc, NULL);
		}
		dev->watchdog_timeo = 5 * HZ; /* restore normal timeout */
	} else {
		do_gettimeofday(&tv);
		if (sc->sc_dfs_testmode) {
			DPRINTF(sc, ATH_DEBUG_STATE | ATH_DEBUG_DOTH, 
					"VAP DFSWAIT_PENDING "
					"indefinitely.  dfs_testmode is "
					"enabled.  Waiting again. -- Time: "
					"%ld.%06ld\n",
					tv.tv_sec, tv.tv_usec);
			mod_timer(&sc->sc_dfs_cac_timer,
				  jiffies + (sc->sc_dfs_cac_period * HZ));
		} else {
			DPRINTF(sc, ATH_DEBUG_STATE | ATH_DEBUG_DOTH, 
					"VAP DFSWAIT_PENDING still.  "
					"Waiting again. -- Time: %ld.%06ld\n", 
					tv.tv_sec, tv.tv_usec);
			mod_timer(&sc->sc_dfs_cac_timer,
				  jiffies + (ATH_DFS_WAIT_SHORT_POLL_PERIOD * HZ));
		}
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
			if ((ni->ni_ucastkey.wk_flags & IEEE80211_KEY_TXRX)
							== IEEE80211_KEY_TXRX)
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
		EPRINTF(sc, "Insufficient memory for channel table!\n");
		return -ENOMEM;
	}
	if (!ath_hal_init_channels(ah, chans, IEEE80211_CHAN_MAX, &nchan,
	    ic->ic_regclassids, IEEE80211_REGCLASSIDS_MAX, &ic->ic_nregclass,
	    cc, HAL_MODE_ALL, outdoor, xchanmode)) {
		u_int32_t rd;

		ath_hal_getregdomain(ah, &rd);
		EPRINTF(sc, "Unable to collect channel list from HAL; "
			"regdomain likely %u country code %u\n", rd, cc);
		kfree(chans);
		return -EINVAL;
	}
	
	/* Convert HAL channels to ieee80211 ones. */
	DPRINTF(sc, ATH_DEBUG_RATE, "HAL returned %d channels.\n", nchan);
	for (i = 0; i < nchan; i++) {
		HAL_CHANNEL *c = &chans[i];
		struct ieee80211_channel *ichan = &ic->ic_channels[i];

		/* Correct the DFS flags to account for problems with DFS in
		 * older binary HALs returning the wrong answers for FCC... */
		ath_radar_correct_dfs_flags(sc, c);
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
		ic->ic_chan_non_occupy[i].tv_sec  = 0;
		ic->ic_chan_non_occupy[i].tv_usec = 0;

		DPRINTF(sc, ATH_DEBUG_RATE,
				"Channel %3d (%4d MHz) Max Tx Power %d dBm%s "
				"[%d hw %d reg] Flags%s%s%s%s%s%s%s%s%s%s%s%s%"
				"s%s%s%s%s%s%s%s%s%s%s%s\n",
				ichan->ic_ieee,
				c->channel,
				(c->maxRegTxPower > (c->maxTxPower / 2) ? 
				 (c->maxTxPower/2) : c->maxRegTxPower),
				(c->maxRegTxPower == (c->maxTxPower / 2) ? 
				 "" : 
				 ((c->maxRegTxPower > (c->maxTxPower / 2)) ? 
				  " (hw limited)" : " (reg limited)")),
				(c->maxTxPower / 2),
				c->maxRegTxPower,
				/* undocumented */
				(c->channelFlags & 0x0001 ? 
				 " CF & (1 << 0)" : ""),
				/* CW interference detected on channel */
				(c->channelFlags & CHANNEL_CW_INT ? 
				 " CF_CW_INTERFERENCE" : ""),
				/* undocumented */
				(c->channelFlags & 0x0004 ?
				 " CF & (1 << 2)" : ""),
				/* undocumented */
				(c->channelFlags & 0x0008 ?
				 " CF & (1 << 3)" : ""),
				/* Turbo channel */
				(c->channelFlags & CHANNEL_TURBO ? 
				 " CF_TURBO" : ""),
				/* CCK channel */
				(c->channelFlags & CHANNEL_CCK ? 
				 " CF_CCK" : ""),
				/* OFDM channel */
				(c->channelFlags & CHANNEL_OFDM ? 
				 " CF_OFDM" : ""),
				/* 2GHz spectrum channel. */
				(c->channelFlags & CHANNEL_2GHZ ? 
				 " CF_2GHZ" : ""),
				/* 5GHz spectrum channel */
				(c->channelFlags & CHANNEL_5GHZ ? 
				 " CF_5GHZ" : ""),
				/* Only passive scan allowed */
				(c->channelFlags & CHANNEL_PASSIVE ? 
				 " CF_PASSIVE_SCAN_ONLY" : ""),
				/* Dynamic CCK-OFDM channel */
				(c->channelFlags & CHANNEL_DYN ? 
				 " CF_DYNAMIC_TURBO" : ""),
				/* GFSK channel (FHSS  PHY) */
				(c->channelFlags & CHANNEL_XR ? 
				 " CF_FHSS" : ""),
				/* Radar found on channel */
				(c->channelFlags & IEEE80211_CHAN_RADAR ? 
				 " CF_RADAR_SEEN" : ""),
				/* 11a static turbo channel only */
				(c->channelFlags & CHANNEL_STURBO ? 
				 " CF_STATIC_TURBO"      :       ""),
				/* Half rate channel */
				(c->channelFlags & CHANNEL_HALF ? 
				 " CF_HALF_RATE" : ""),
				/* Quarter rate channel */
				(c->channelFlags & CHANNEL_QUARTER ? 
				 " CF_QUARTER_RATE" : ""),
				/* Software use: channel interference used 
				 * for as AR as well as RADAR interference 
				 * detection. */
				(c->privFlags & CHANNEL_INTERFERENCE ? 
				 " PF_INTERFERENCE" : ""),
				/* DFS required on channel */
				(c->privFlags & CHANNEL_DFS ? 
				 " PF_DFS_REQUIRED" : ""),
				/* 4msec packet limit on this channel */
				(c->privFlags & CHANNEL_4MS_LIMIT ? 
				 " PF_4MS_LIMIT" : ""),
				/* if channel has been checked for DFS */
				(c->privFlags & CHANNEL_DFS_CLEAR ? 
				 " PF_DFS_CLEAR" : ""),
				/* undocumented */
				(c->privFlags & 0x0010 ? 
				 " PF & (1 << 4)" : ""),
				/* undocumented */
				(c->privFlags & 0x0020 ? 
				 " PF & (1 << 5)" : ""),
				/* undocumented */
				(c->privFlags & 0x0040 ? 
				 " PF & (1 << 6)" : ""),
				/* undocumented */
				(c->privFlags & 0x0080 ? 
				 " PF & (1 << 7)" : "")
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
	mod_timer(&sc->sc_ledtimer, jiffies + sc->sc_ledoff);
}

/*
 * Blink the LED according to the specified on/off times.
 */
static void
ath_led_blink(struct ath_softc *sc, int on, int off)
{
	DPRINTF(sc, ATH_DEBUG_LED, "on %u off %u\n", on, off);
	ath_hal_gpioset(sc->sc_ah, sc->sc_ledpin, sc->sc_ledon);
	sc->sc_blinking = 1;
	sc->sc_ledoff = off;
	sc->sc_ledtimer.function = ath_led_off;
	mod_timer(&sc->sc_ledtimer, jiffies + on);
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
 * minimum of the regulatory constraint and the limits of the radio.
 * XXX: this function needs some locking to avoid being called 
 * twice/interrupted. Returns the value actually stored. */
static u_int32_t
ath_set_clamped_maxtxpower(struct ath_softc *sc, 
		u_int32_t new_clamped_maxtxpower)
{
	(void)ath_hal_settxpowlimit(sc->sc_ah, new_clamped_maxtxpower);
	return ath_get_clamped_maxtxpower(sc);
}

/* The HAL supports a maxtxpow which is something we can configure to be the
 * minimum of the regulatory constraint and the limits of the radio.
 * XXX: this function needs some locking to avoid being called 
 * twice/interrupted */
static u_int32_t
ath_get_clamped_maxtxpower(struct ath_softc *sc)
{
	u_int32_t clamped_maxtxpower;
	(void)ath_hal_getmaxtxpow(sc->sc_ah, &clamped_maxtxpower);
	return clamped_maxtxpower;
}

/* XXX: this function needs some locking to avoid being called 
 * twice/interrupted */
/* 1. Save the currently specified maximum txpower (as clamped by madwifi)
 * 2. Determine the real maximum txpower the card can support by
 *    setting a value that exceeds the maximum range (by one) and
 *    finding out what it limits us to.
 * 3. Restore the saved maxtxpower value we had previously specified */
static u_int32_t
ath_get_real_maxtxpower(struct ath_softc *sc)
{
	u_int32_t saved_clamped_maxtxpower;
	u_int32_t real_maxtxpower;

	saved_clamped_maxtxpower = ath_get_clamped_maxtxpower(sc);
	real_maxtxpower = 
		ath_set_clamped_maxtxpower(sc, IEEE80211_TXPOWER_MAX + 1);
	ath_set_clamped_maxtxpower(sc, saved_clamped_maxtxpower);
	return real_maxtxpower;
}


/* XXX: this function needs some locking to avoid being called 
 * twice/interrupted */
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
	/* Search for the VAP that needs a txpow change, if any */
	TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) {
		if (!hal_tpc || ic->ic_newtxpowlimit != vap->iv_bss->ni_txpower) {
			vap->iv_bss->ni_txpower = new_clamped_maxtxpower;
			ieee80211_iterate_nodes(&vap->iv_ic->ic_sta, 
					set_node_txpower, 
					&new_clamped_maxtxpower);
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
			"The rate table is too small (%u > %u)\n",
			rt->rateCount, IEEE80211_RATE_MAXSIZE);
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
				"The rate table is too small (%u > %u)\n",
				rt->rateCount, IEEE80211_RATE_MAXSIZE);
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
				"The rate table is too small (%u > %u)\n",
				rt->rateCount, IEEE80211_RATE_MAXSIZE);
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
		DPRINTF(sc, ATH_DEBUG_ANY, "Invalid mode %u\n", mode);
		return 0;
	}
	rt = sc->sc_rates[mode];
	if (rt == NULL)
		return 0;
	if (rt->rateCount > IEEE80211_RATE_MAXSIZE) {
		DPRINTF(sc, ATH_DEBUG_ANY,
			"The rate table is too small (%u > %u)\n",
			rt->rateCount, IEEE80211_RATE_MAXSIZE);
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

	/* Handle 4 ms channel limit. */
	if (sc->sc_curchan.privFlags & CHANNEL_4MS_LIMIT)
		txoplimit = MIN(txoplimit, 4000);

	if (txoplimit != 0 && athff_approx_txtime(sc, an, skb) > txoplimit) {
		DPRINTF(sc, ATH_DEBUG_XMIT | ATH_DEBUG_FF,
			"FF TxOp violation\n");
		if (ffbuf)
			*flushq = AH_TRUE;
		return AH_FALSE;
	}

	return AH_TRUE;
}
#endif

#ifdef AR_DEBUG

static void
ath_printrxbuf(const struct ath_buf *bf, int done)
{
	const struct ath_rx_status *rs = &bf->bf_dsstatus.ds_rxstat;
	const struct ath_desc *ds = bf->bf_desc;
	u_int8_t status = done ? rs->rs_status : 0;
	printk("R (%p %08llx) %08x %08x %08x %08x %08x %08x%s%s%s%s%s%s%s%s%s\n",
		ds, (u_int64_t)bf->bf_daddr,
		ds->ds_link, ds->ds_data,
		ds->ds_ctl0, ds->ds_ctl1,
		ds->ds_hw[0], ds->ds_hw[1],
		status				? ""			: " OK",
		status & HAL_RXERR_CRC		? " ERR_CRC"		: "",
		status & HAL_RXERR_PHY		? " ERR_PHY"		: "",
		status & HAL_RXERR_FIFO 	? " ERR_FIFO"		: "",
		status & HAL_RXERR_DECRYPT	? " ERR_DECRYPT"	: "",
		status & HAL_RXERR_MIC		? " ERR_MIC" 		: "",
		status & 0x20			? " (1<<5)" 		: "",
		status & 0x40			? " (1<<6)"		: "",
		status & 0x80			? " (1<<7)"		: "");
}

static void
ath_printtxbuf(const struct ath_buf *bf, int done)
{
	const struct ath_tx_status *ts = &bf->bf_dsstatus.ds_txstat;
	const struct ath_desc *ds = bf->bf_desc;
	struct ath_softc *sc = ATH_BUF_NI(bf)->ni_ic->ic_dev->priv;
	u_int8_t status = done ? ts->ts_status : 0;

	DPRINTF(sc, ATH_DEBUG_ANY, 
		"T (%p %08llx) %08x %08x %08x %08x %08x %08x %08x %08x%s%s%s%s%s%s%s%s%s\n",
		ds, (u_int64_t)bf->bf_daddr,
		ds->ds_link, ds->ds_data,
		ds->ds_ctl0, ds->ds_ctl1,
		ds->ds_hw[0], ds->ds_hw[1], ds->ds_hw[2], ds->ds_hw[3],
		status					? ""			: " OK",
		status & HAL_TXERR_XRETRY		? " ERR_XRETRY"		: "",
		status & HAL_TXERR_FILT			? " ERR_FILT"		: "",
		status & HAL_TXERR_FIFO			? " ERR_FIFO"		: "",
		status & HAL_TXERR_XTXOP		? " ERR_XTXOP" 		: "",
		status & HAL_TXERR_DESC_CFG_ERR		? " ERR_DESC_CFG_ERR" 	: "",
		status & HAL_TXERR_DATA_UNDERRUN	? " ERR_DATA_UNDERRUN"	: "",
		status & HAL_TXERR_DELIM_UNDERRUN	? " ERR_DELIM_UNDERRUN"	: "",
		status & 0x80				? " (1<<7)"		: "");
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
		EPRINTF(sc, 
			"Cannot set MAC address; device running!\n");
		return -EBUSY;
	}
	DPRINTF(sc, ATH_DEBUG_ANY, MAC_FMT, MAC_ADDR(mac->sa_data));

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
		DPRINTF(sc, ATH_DEBUG_ANY, "Invalid MTU, %d, valid range is {%u..%u}.\n",
			mtu, ATH_MIN_MTU, ATH_MAX_MTU);
		return -EINVAL;
	}
	DPRINTF(sc, ATH_DEBUG_ANY, "MTU set to %d\n", mtu);

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
	void *indata = NULL, *outdata = NULL;
	u_int32_t insize = ad->ad_in_size, outsize = ad->ad_out_size;
	int error = 0;

	if (ad->ad_id & ATH_DIAG_IN) {
		if (insize == 0) {
			error = -EINVAL;
			goto bad;
		}

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
		if (outsize == 0) {
			error = -EINVAL;
			goto bad;
		}

		/* Allocate a buffer for the results (otherwise the HAL
		 * returns a pointer to a buffer where we can read the
		 * results).  Note that we depend on the HAL leaving this
		 * pointer for us to use below in reclaiming the buffer;
		 * may want to be more defensive. */
		outdata = kmalloc(outsize, GFP_KERNEL);
		if (outdata == NULL) {
			error = -ENOMEM;
			goto bad;
		}
	}
	if (ath_hal_getdiagstate(ah, ad->ad_id & ATH_DIAG_ID, indata, insize, 
				&outdata, &outsize)) {
		ad->ad_out_size = outsize;
		if (outdata &&
		    copy_to_user(ad->ad_out_data, outdata, ad->ad_out_size))
			error = -EFAULT;
	} else
		error = -EINVAL;
bad:
	if (indata != NULL)
		kfree(indata);
	if (outdata != NULL)
		kfree(outdata);
	return error;
}

static int
ath_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct ath_softc *sc = dev->priv;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_diag ad;
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
		return -EOPNOTSUPP;
		break;
	case SIOCGATHHALDIAG:
		if (!capable(CAP_NET_ADMIN))
			error = -EPERM;
		else if (copy_from_user(&ad, ifr->ifr_data, sizeof(ad)))
			error = -EFAULT;
		else
			error = ath_ioctl_diag(sc, &ad);
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
		error = -EOPNOTSUPP;
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
	ATH_RP         		= 17,
	ATH_RP_PRINT   		= 18,
	ATH_RP_PRINT_ALL 	= 19,
	ATH_RP_PRINT_MEM 	= 20,
	ATH_RP_PRINT_MEM_ALL 	= 21,
	ATH_RP_FLUSH   		= 22,
	ATH_PANIC               = 23,
	ATH_RP_IGNORED 		= 24,
	ATH_RADAR_IGNORED       = 25,
	ATH_MAXVAPS  		= 26,
	ATH_INTMIT 		= 27,
	ATH_DISTANCE		= 28,
};

static inline int 
ath_ccatime(struct ath_softc *sc)
{
	if (((sc->sc_curchan.channelFlags & IEEE80211_CHAN_PUREG) == 
	    IEEE80211_CHAN_PUREG) && 
	    (sc->sc_ic.ic_flags & IEEE80211_F_SHSLOT))
		return CCA_PUREG;
	if ((sc->sc_curchan.channelFlags & IEEE80211_CHAN_A) == 
	    IEEE80211_CHAN_A)
		return CCA_A;

	return CCA_BG;
}

static inline int 
ath_estimate_max_distance(struct ath_softc *sc)
{
	/* Prefer override; ask HAL if not overridden. */
	int slottime = sc->sc_slottimeconf;
	if (slottime <= 0)
		slottime = ath_hal_getslottime(sc->sc_ah);
	/* NB: We ignore MAC overhead.  This function is the reverse operation
	 * of ath_distance2slottime, and assumes slottime is CCA + 2 * air
	 * propagation. */
	return (slottime - ath_ccatime(sc)) * 150;
}

static inline int 
ath_distance2slottime(struct ath_softc *sc, int distance)
{
	/* Allowance for air propagation (roundtrip time) should be at least 
	 * 5us per the standards.
	 * 
	 * So let's set a minimum distance to accomodate this: 
	 * roundtrip time = ((distance / speed_of_light) * 2)
	 * distance = ((time * 300 ) / 2) or ((5 * 300) / 2) = 750 m */
	int rtdist = distance * 2;
	int c = 299;	/* Speed of light in vacuum in m/us. */ 
	/* IEEE 802.11 2007 10l.4.3.2. In us. */
	int aAirPropagation = MAX(5, howmany(rtdist, c));
	/* XXX: RX/TX turnaround & MAC delay. */
	return ath_ccatime(sc) + aAirPropagation;
}

static inline int 
ath_distance2timeout(struct ath_softc *sc, int distance)
{
	/* HAL uses a constant of twice slot time plus 18us. The 18us covers
	 * RX/TX turnaround, MIB processing, etc., but the athctrl used to
	 * return (2 * slot) + 3, so the extra 15us of timeout is probably just
	 * being very careful or taking something into account that I can't
	 * find in the specs.
	 *
	 * XXX: Update based on emperical evidence (potentially save 15us per
	 * timeout). */
	return ath_slottime2timeout(sc, ath_distance2slottime(sc, distance));
}

static int
ATH_SYSCTL_DECL(ath_sysctl_halparam, ctl, write, filp, buffer, lenp, ppos)
{
	struct ath_softc *sc = ctl->extra1;
	struct ath_hal *ah = sc->sc_ah;
	u_int val;
	u_int tab_3_val[3];
	int ret = 0;

	ctl->data = &val;
	ctl->maxlen = sizeof(val);

	/* Special case for ATH_RP which expect 3 integers: TSF RSSI width. It
	 * should be noted that tsf is unsigned 64 bits but the sysctl API is
	 * only unsigned 32 bits. As a result, TSF might get truncated. */
	if (ctl->extra2 == (void *)ATH_RP) {
		ctl->data = &tab_3_val;
		ctl->maxlen = sizeof(tab_3_val);
	}

	ATH_LOCK(sc);
	if (write) {
		ret = ATH_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer, 
				lenp, ppos);
		if (ret == 0) {
			switch ((long)ctl->extra2) {
			case ATH_DISTANCE:
				if (val > 0) {
					sc->sc_slottimeconf    = ath_distance2slottime(sc, val);
					sc->sc_acktimeoutconf  = ath_distance2timeout(sc, val);
					sc->sc_ctstimeoutconf  = ath_distance2timeout(sc, val);
					ath_setslottime(sc);
				}
				else {
					/* disable manual overrides */
					sc->sc_slottimeconf   = 0;
					sc->sc_ctstimeoutconf = 0;
					sc->sc_acktimeoutconf = 0;
					ath_setslottime(sc);
				}
				/* Likely changed by the function */
				val = ath_estimate_max_distance(sc);
				break;
			case ATH_SLOTTIME:
				if (val > 0) {
					if (!ath_hal_setslottime(ah, val))
						ret = -EINVAL;
					else {
						int old = sc->sc_slottimeconf;
						sc->sc_slottimeconf = val;
						/* overridden slot time invalidates 
						 * previous overridden ack/cts 
						 * timeouts if it is longer! */
						if (old && old < sc->sc_slottimeconf) {
							sc->sc_ctstimeoutconf = 0;
							sc->sc_acktimeoutconf = 0;
							ath_setacktimeout(sc);
							ath_setctstimeout(sc);
						}
					}
				} else {
					/* disable manual override */
					sc->sc_slottimeconf = 0;
					ath_setslottime(sc);
				}
				/* Likely changed by the function */
				val = ath_getslottime(sc);
				break;
			case ATH_ACKTIMEOUT:
				if (val > 0) {
					if (!ath_hal_setacktimeout(ah, val))
						ret = -EINVAL;
					else 
						sc->sc_acktimeoutconf = val;
				} else {
					/* disable manual overrider */
					sc->sc_acktimeoutconf = 0;
					ath_setacktimeout(sc);
				}
				/* Likely changed by the function */
				val = ath_getacktimeout(sc);
				break;
			case ATH_CTSTIMEOUT:
				if (val > 0) {
					if (!ath_hal_setctstimeout(ah, val))
						ret = -EINVAL;
					else 
						sc->sc_ctstimeoutconf = val;
				} else {
					/* disable manual overrides */
					sc->sc_ctstimeoutconf = 0;
					ath_setctstimeout(sc);
				}
				/* Likely changed by the function */
				val = ath_getctstimeout(sc);
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
				IPRINTF(sc, "Ath. debug flags changed to "
						"0x%08x.\n", val);

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
			case ATH_RP:
				ath_rp_record(sc,
						tab_3_val[0],
						tab_3_val[1],
						tab_3_val[2],
						1 /* simulated */
						);
				/* we analyze pulses in a separate tasklet */
				ATH_SCHEDULE_TQUEUE(&sc->sc_rp_tq, NULL);
				break;
			case ATH_RP_PRINT:
				if (val)
					ath_rp_print(sc,1);
				break;
			case ATH_RP_PRINT_ALL:
				if (val)
					ath_rp_print(sc,0);
				break;
			case ATH_RP_PRINT_MEM:
				if (val)
					ath_rp_print_mem(sc,0);
				break;
			case ATH_RP_PRINT_MEM_ALL:
				if (val)
					ath_rp_print_mem(sc,0);
				break;
			case ATH_RP_FLUSH:
				if (val)
					ath_rp_flush(sc);
				break;
			case ATH_PANIC:
				if (val) {
					int * p = (int *)0xdeadbeef;
					*p = 0xcacadede;
				}
				break;
			case ATH_RP_IGNORED:
				sc->sc_rp_ignored = val;
				break;
			case ATH_RADAR_IGNORED:
				sc->sc_radar_ignored = val;
				break;
			case ATH_INTMIT:
				if (!sc->sc_hasintmit) {
					ret = -EOPNOTSUPP;
					break;
				}
				if (sc->sc_useintmit == val)
					break;
				sc->sc_useintmit = val;
				sc->sc_needmib = ath_hal_hwphycounters(ah) &&
					sc->sc_useintmit;
				/* Update the HAL and MIB interrupt mask bits */
				ath_hal_setintmit(ah, !!val);
				sc->sc_imask = (sc->sc_imask & ~HAL_INT_MIB) |
					(sc->sc_needmib ? HAL_INT_MIB : 0);
				ath_hal_intrset(sc->sc_ah, sc->sc_imask);
				/* Only do a reset if device is valid and UP
				 * and we just made a change to the settings. */
				if (sc->sc_dev && !sc->sc_invalid &&
				    (sc->sc_dev->flags & IFF_RUNNING))
					ath_reset(sc->sc_dev);
				/* NB: Run this step to cleanup if HAL doesn't
				 * obey capability flags and hangs onto ANI
				 * settings. */
				ath_override_intmit_if_disabled(sc);
				break;
			default:
				ret = -EINVAL;
				break;
			}
		}
	} else {
		switch ((long)ctl->extra2) {
		case ATH_DISTANCE:
			val = ath_estimate_max_distance(sc);
			break;
		case ATH_SLOTTIME:
			val = ath_getslottime(sc);
			break;
		case ATH_ACKTIMEOUT:
			val = ath_getacktimeout(sc);
			break;
		case ATH_CTSTIMEOUT:
			val = ath_getctstimeout(sc);
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
		case ATH_MAXVAPS:
			val = ath_maxvaps;
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
		case ATH_RP_IGNORED:
			val = sc->sc_rp_ignored;
			break;
		case ATH_RADAR_IGNORED:
			val = sc->sc_radar_ignored;
			break;
		case ATH_INTMIT:
			val = sc->sc_useintmit;
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

static const ctl_table ath_sysctl_template[] = {
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "distance",
	  .mode		= 0644,
	  .proc_handler	= ath_sysctl_halparam,
	  .extra2	= (void *)ATH_DISTANCE,
	},
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
	  .procname	= "maxvaps",
	  .mode		= 0444,
	  .proc_handler	= ath_sysctl_halparam,
	  .extra2	= (void *)ATH_MAXVAPS,
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
	{ .ctl_name	= CTL_AUTO,
	  .procname     = "rp",
	  .mode         = 0200,
	  .proc_handler = ath_sysctl_halparam,
	  .extra2	= (void *)ATH_RP,
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname     = "radar_print",
	  .mode         = 0200,
	  .proc_handler = ath_sysctl_halparam,
	  .extra2	= (void *)ATH_RP_PRINT,
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname     = "radar_print_all",
	  .mode         = 0200,
	  .proc_handler = ath_sysctl_halparam,
	  .extra2	= (void *)ATH_RP_PRINT_ALL,
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname     = "radar_dump",
	  .mode         = 0200,
	  .proc_handler = ath_sysctl_halparam,
	  .extra2	= (void *)ATH_RP_PRINT_MEM,
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname     = "radar_dump_all",
	  .mode         = 0200,
	  .proc_handler = ath_sysctl_halparam,
	  .extra2	= (void *)ATH_RP_PRINT_MEM_ALL,
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname     = "rp_flush",
	  .mode         = 0200,
	  .proc_handler = ath_sysctl_halparam,
	  .extra2	= (void *)ATH_RP_FLUSH,
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname     = "panic",
	  .mode         = 0200,
	  .proc_handler = ath_sysctl_halparam,
	  .extra2	= (void *)ATH_PANIC,
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname     = "rp_ignored",
	  .mode         = 0644,
	  .proc_handler = ath_sysctl_halparam,
	  .extra2	= (void *)ATH_RP_IGNORED,
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname     = "radar_ignored",
	  .mode         = 0644,
	  .proc_handler = ath_sysctl_halparam,
	  .extra2	= (void *)ATH_RADAR_IGNORED,
	},
	{ .ctl_name     = CTL_AUTO,
	  .procname     = "intmit",
	  .mode         = 0644,
	  .proc_handler = ath_sysctl_halparam,
	  .extra2       = (void *)ATH_INTMIT,
	},
	{ 0 }
};

static void
ath_dynamic_sysctl_register(struct ath_softc *sc)
{
	unsigned int i, space;
	char *dev_name = NULL;

	/* Prevent multiple registrations */
	if (sc->sc_sysctls)
		return;

	space = 5 * sizeof(struct ctl_table) + sizeof(ath_sysctl_template);
	sc->sc_sysctls = kzalloc(space, GFP_KERNEL);
	if (sc->sc_sysctls == NULL) {
		EPRINTF(sc, "Insufficient memory for sysctl table!\n");
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
		EPRINTF(sc, "Insufficient memory for device name storage!\n");
		kfree(sc->sc_sysctls);
		sc->sc_sysctls = NULL;
		return;
	}
	strncpy(dev_name, DEV_NAME(sc->sc_dev), strlen(DEV_NAME(sc->sc_dev)) + 1);

	/* setup the table */
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
		EPRINTF(sc, "Failed to register sysctls!\n");
		kfree(dev_name);
		kfree(sc->sc_sysctls);
		sc->sc_sysctls = NULL;
	}
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
	struct ath_softc *sc = dev->priv;
	struct ath_hal *ah = sc->sc_ah;
	u_int modes, cc;
#if 0
	unsigned int i;
#endif

	printk(KERN_INFO "%s: Atheros AR%s chip found (MAC %d.%d, ",
		DEV_NAME(dev),
		ath5k_chip_name(AR5K_VERSION_VER, ATH_SREV_FROM_AH(ah)),
		ah->ah_macVersion, ah->ah_macRev);
	printk("PHY %s %d.%d, ",
		ath5k_chip_name(AR5K_VERSION_RAD, ah->ah_phyRev),
		ah->ah_phyRev >> 4, ah->ah_phyRev & 0xf);

	/* Print radio revision(s).  We check the wireless modes to avoid 
	 * falsely printing revs for inoperable parts. Dual-band radio rev's
	 * are returned in the 5 GHz rev. number. */
	ath_hal_getcountrycode(ah, &cc);
	modes = ath_hal_getwirelessmodes(ah, cc);
	if ((modes & (HAL_MODE_11A | HAL_MODE_11B)) ==
			(HAL_MODE_11A | HAL_MODE_11B)) {
		if (ah->ah_analog5GhzRev && ah->ah_analog2GhzRev)
			printk("5 GHz Radio %d.%d 2 GHz Radio %d.%d)\n",
				ah->ah_analog5GhzRev >> 4,
				ah->ah_analog5GhzRev & 0xf,
				ah->ah_analog2GhzRev >> 4,
				ah->ah_analog2GhzRev & 0xf);
		else
			printk("Radio %d.%d)\n",
				ah->ah_analog5GhzRev >> 4,
				ah->ah_analog5GhzRev & 0xf);
	} else {
		printk("Radio %d.%d)\n",
			ah->ah_analog5GhzRev >> 4,
			ah->ah_analog5GhzRev & 0xf);
	}

/* Disabled - this information is not operationally useful. */
#if 0
	for (i = 0; i <= WME_AC_VO; i++) {
		struct ath_txq *txq = sc->sc_ac2q[i];
		printk(KERN_INFO "%s: Use H/W queue %u for %s traffic\n",
			DEV_NAME(dev),
			txq->axq_qnum,
			ieee80211_wme_acnames[i]);
	}
#ifdef ATH_SUPERG_XR
	printk(KERN_INFO "%s: Use hw queue %u for XR traffic\n",
		DEV_NAME(dev),
		sc->sc_xrtxq->axq_qnum);
#endif
	if (sc->sc_uapsdq != NULL) {
		printk(KERN_INFO "%s: Use hw queue %u for UAPSD traffic\n",
			DEV_NAME(dev),
			sc->sc_uapsdq->axq_qnum);
	}
	printk(KERN_INFO "%s: Use hw queue %u for CAB traffic\n",
		DEV_NAME(dev), sc->sc_cabq->axq_qnum);
	printk(KERN_INFO "%s: Use hw queue %u for beacons\n",
		DEV_NAME(dev), sc->sc_bhalq);
#endif
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
	  .procname	= "maxvaps",
	  .mode		= 0444,
	  .data		= &ath_maxvaps,
	  .maxlen	= sizeof(ath_maxvaps),
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

static const char *
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
		IPRINTF(sc, "Using default rate of %dM.\n",
				(rt->info[closest_rate_ix].rateKbps / 1000));
	} else if (sc->sc_txcont_rate == (rt->info[closest_rate_ix].rateKbps / 1000)) {
		IPRINTF(sc, "Using requested rate of %dM.\n",
				sc->sc_txcont_rate);
	} else {
		IPRINTF(sc, "Rounded down requested rate of %dM to %dM.\n",
				sc->sc_txcont_rate,
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
		EPRINTF(sc, "Cannot enable txcont when interface is"
			" not in running state.\n");
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
		sc->sc_txantenna = sc->sc_rxantenna = sc->sc_mcastantenna = 1;
		sc->sc_numrxotherant = 0;
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
		if (!ath_hw_reset(sc, sc->sc_opmode,
					&sc->sc_curchan, AH_TRUE, &status)) {
			EPRINTF(sc, "ath_hal_reset failed: '%s' "
					"(HAL status %u).\n",
					ath_get_hal_status_desc(status),
					status);
		}

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
					"Enabling dynamic turbo.\n");
				ic->ic_ath_cap |= IEEE80211_ATHC_BOOST;
				sc->sc_ignore_ar = 1;
				newflags |= IEEE80211_CHAN_TURBO;
			} else {
				DPRINTF(sc, ATH_DEBUG_TURBO, 
					"Disabling dynamic turbo.\n");
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
#define	AR5K_AR5212_QCU(_n, _a)				(((_n) << 2) + _a)
#define	AR5K_AR5212_DCU(_n, _a)				AR5K_AR5212_QCU(_n, _a)
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
txcont_queue_packet(struct ieee80211com *ic, struct ath_txq *txq)
{
	struct net_device *dev             = ic->ic_dev;
	struct ath_softc *sc               = dev->priv;
	struct ath_hal *ah                 = sc->sc_ah;
	struct ath_buf *bf                 = NULL;
	struct sk_buff *skb                = NULL;
	unsigned int i;
	/* maximum supported size, subtracting headers and required slack */
	unsigned int datasz                = 4028;
	struct ieee80211_frame *wh         = NULL;
	unsigned char *data                = NULL;
	unsigned char *crc                 = NULL;

	if (IFF_RUNNING != (ic->ic_dev->flags & IFF_RUNNING) || 
			(0 == sc->sc_txcont)) {
		EPRINTF(sc, "Refusing to queue self linked frame "
				"when txcont is not enabled.\n"); 
		return;
	}

	ath_hal_intrset(ah, 0);
	{
		bf  = ath_take_txbuf(sc);
		skb = alloc_skb(datasz + sizeof(struct ieee80211_frame) + 
				IEEE80211_CRC_LEN, GFP_ATOMIC);
		if (NULL == skb) {
			EPRINTF(sc, "alloc_skb(...) returned null!\n");
			BUG();
		}
		wh  = (struct ieee80211_frame *)skb_put(skb, 
				sizeof(struct ieee80211_frame));
		if (NULL == bf) {
			EPRINTF(sc, "ath_take_txbuf(sc) returned null!\n");
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
		bf->bf_desc->ds_link = bf->bf_daddr;
		bf->bf_desc->ds_data = bf->bf_skbaddr;

		/* in case of TPC, we need to set the maximum if we want < max */
		ath_hal_settxpowlimit(ah, sc->sc_txcont_power/2);
		ath_hal_settpc(ah, 0);
		ath_hal_setuptxdesc(ah,
			bf->bf_desc,			/* the descriptor */
			skb->len,			/* packet length */
			sizeof(struct ieee80211_frame),	/* header length */
			HAL_PKT_TYPE_NORMAL,   		/* Atheros packet type */
			sc->sc_txcont_power,	 	/* txpower in 0.5dBm 
							 * increments, range 0-n 
							 * depending upon card 
							 * typically 60-100 max */
			ath_get_txcont_adj_ratecode(sc),/* series 0 rate */
			0,				/* series 0 retries */
			HAL_TXKEYIX_INVALID,		/* key cache index */
			sc->sc_txantenna,		/* antenna mode */
			bf->bf_flags,			/* flags */
			0,				/* rts/cts rate */
			0,				/* rts/cts duration */
			0,				/* comp icv len */
			0,				/* comp iv len */
			ATH_COMP_PROC_NO_COMP_NO_CCS	/* comp scheme */
			);

		ath_hal_filltxdesc(ah,
				bf->bf_desc,	/* Descriptor to fill */
				skb->len,	/* buffer length */
				AH_TRUE,	/* is first segment */
				AH_TRUE,	/* is last segment */
				bf->bf_desc	/* first descriptor */
				);

		/*  Byteswap (as necessary) */
		ath_desc_swap(bf->bf_desc);
		/*  queue the self-linked frame */
		ath_tx_txqaddbuf(sc, NULL,	/* node */
				txq,		/* hardware queue */
				bf,		/* atheros buffer */
				bf->bf_skb->len	/* frame length */
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
		EPRINTF(sc, "Cannot enable txcont when"
				" interface is not in running state.\n");
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

/* See ath_set_dfs_testmode for details. */
static int
ath_get_dfs_testmode(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	return sc->sc_dfs_testmode;
}

/* In this mode:
 * "doth" flag can be enabled and radar detection will be logged to the syslog
 * (or console) with a timestamp. "markdfs" flag will be ignoredm and the
 * channel will never be marked as having radar interference. If the channel
 * was not already done with channel availability scan before this flag is set,
 * channel availability scan will run perpetually.
 *
 * Therefore you have two modes of usage:
 *
 * 1  Bring up an AP and then enable DFS test mode after the channel
 *    availability scan, it will report radar errors but will not stop the AP.
 *    (This is useful for demonstrating that even under high duty cycle, radar
 *    is still detected... but without changing channels -- for probability
 *    testing and debugging)
 * 2  Enable DFS test mode before the channel availability scan completes, it
 *    will report radar erors but will never begin transmitting beacons or
 *    acting as an AP. (This is useful for demonstrating channel availability
 *    check works during FCC and ETSI testing -- for probability testing and
 *    debugging) */
static void
ath_set_dfs_testmode(struct ieee80211com *ic, int value)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	sc->sc_dfs_testmode = !!value;
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

/* For testing, we will allow you to change the channel availability check
 * time. Do not use this in production, obviously. */
static void
ath_set_dfs_cac_time(struct ieee80211com *ic, unsigned int time_s)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	sc->sc_dfs_cac_period = time_s;
}

/* For testing, we will allow you to change the channel availability check
 * time. Do not use this in production, obviously. */
static unsigned int
ath_get_dfs_cac_time(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	return sc->sc_dfs_cac_period;
}

/* For testing, we will allow you to change the channel non-occupancy period.
 * Do not use this in production, obviously. This is very handy for
 * verifying and testing that the non-occupancy feature works, and that the
 * uniform channel spreading requirement is met.
 *
 * 1  Set a short non-occupancy period.
 * 2  Set the channel to a *fixed* channel requiring DFS.
 * 3  Wait for radar or use the ioctl to fake an event.
 * 4  Assumng you are not in dfstest mode, the radio will change to another
 *    channel... but it remembers you preferred fixed channel.
 * 5  Wait (a much shorter period than half an hour) to see that your original
 *    channel is returned to service AND that the AP switches back to it. */
static void
ath_set_dfs_excl_period(struct ieee80211com *ic, unsigned int time_s)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	sc->sc_dfs_excl_period = time_s;
}

/* See ath_set_dfs_excl_period for details. */
static unsigned int
ath_get_dfs_excl_period(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc = dev->priv;
	return sc->sc_dfs_excl_period;
}


/* This is called by a private ioctl (iwpriv) to simulate radar by directly
 * invoking the ath_radar_detected function even though we are outside of
 * interrupt context. */
static unsigned int
ath_test_radar(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc   = dev->priv;
	if ((ic->ic_flags & IEEE80211_F_DOTH) && (sc->sc_curchan.privFlags & CHANNEL_DFS))
		ath_radar_detected(sc, "ath_test_radar from user space");
	else
		DPRINTF(sc, ATH_DEBUG_DOTH, "Channel %u MHz is not marked "
				"for CHANNEL_DFS.  Radar test not performed!\n",
				 sc->sc_curchan.channel);
	return 0;
}

/* This is called by a private ioctl (iwpriv) to dump the HAL obfuscation table */
static unsigned int
ath_dump_hal_map(struct ieee80211com *ic)
{
	struct net_device *dev = ic->ic_dev;
	struct ath_softc *sc   = dev->priv;
	ath_hal_dump_map(sc->sc_ah);
	return 0;
}

/* If we are shutting down or blowing off the DFS channel availability check
 * then we call this to stop the behavior before we take the rest of the
 * necessary actions (such as a DFS reaction to radar). */
static void
ath_interrupt_dfs_cac(struct ath_softc *sc, const char *reason)
{
	struct timeval tv;

	del_timer_sync(&sc->sc_dfs_cac_timer);
	if (sc->sc_dfs_cac) {
		do_gettimeofday(&tv);
		DPRINTF(sc, ATH_DEBUG_STATE | ATH_DEBUG_DOTH,
				"%s - Channel: %u Time: %ld.%06ld\n",
				reason,
				ieee80211_mhz2ieee(sc->sc_curchan.channel,
					sc->sc_curchan.channelFlags),
				tv.tv_sec, tv.tv_usec);
	}
	sc->sc_dfs_cac = 0;
}

/* Invoked from interrupt context when radar is detected and positively
 * identified by historical event analysis. This guy must report the radar
 * event and perform the "DFS action" which can mean one of two things:
 *
 * If markdfs is enabled, it means mark the channel for non-occupancy with an
 * expiration (typically 30min by law), and then change channels for at least
 * that long.
 *
 * If markdfs is disabled or we are in dfstest mode we may just report the
 * radar or we may go to another channel, and sit quietly.  This 'go sit
 * quietly' (or mute test) behavior is an artifact of the previous DFS code in
 * trunk and it's left here because it may be used as the basis for
 * implementing AP requested mute tests in station mode later. */

void
ath_radar_detected(struct ath_softc *sc, const char *cause)
{
	struct ath_hal		*ah  = sc->sc_ah;
	struct ieee80211com	*ic  = &sc->sc_ic;
	struct ieee80211_channel ichan;
	struct timeval tv;

	DPRINTF(sc, ATH_DEBUG_DOTH, 
		"Radar detected on channel:%u cause: %s%s\n",
		sc->sc_curchan.channel,
		cause,
		sc->sc_radar_ignored ? " (ignored)" : "");

	if (sc->sc_radar_ignored) {
		return;
	}

	ath_rp_flush(sc);
	do_gettimeofday(&tv);

	/* Stop here if we are testing w/o channel switching */
	if (sc->sc_dfs_testmode) {
		/* ath_dump_phyerr_statistics(sc, cause); */
		if (sc->sc_dfs_cac)
			DPRINTF(sc, ATH_DEBUG_DOTH, 
					"dfs_testmode enabled -- "
					"staying in CAC mode!\n");
		else
			DPRINTF(sc, ATH_DEBUG_DOTH, "dfs_testmode "
					"enabled -- staying on channel!\n");
		return;
	}

	/*  Stop any pending channel availability check (if applicable) */
	ath_interrupt_dfs_cac(sc, "Radar detected.  Interrupting DFS wait.");

	/*  radar was found, initiate channel change */
	ichan.ic_ieee = ath_hal_mhz2ieee(ah, sc->sc_curchan.channel, 
			sc->sc_curchan.channelFlags);
	ichan.ic_freq = sc->sc_curchan.channel;
	ichan.ic_flags = sc->sc_curchan.channelFlags;

	if (IEEE80211_IS_MODE_DFS_MASTER(ic->ic_opmode)) {
		if (!(ic->ic_flags_ext & IEEE80211_FEXT_MARKDFS))
			DPRINTF(sc, ATH_DEBUG_DOTH, 
					"markdfs is disabled.  "
					"ichan=%3d (%4d MHz) ichan.icflags=0x%08X\n",
					ichan.ic_ieee, ichan.ic_freq, ichan.ic_flags);
		else {
			DPRINTF(sc, ATH_DEBUG_DOTH, "DFS marked!  "
				"ichan=%3d (%4d MHz), ichan.icflags=0x%08X "
				"-- Time: %ld.%06ld\n", 
				ichan.ic_ieee, ichan.ic_freq, ichan.ic_flags, 
				tv.tv_sec, tv.tv_usec);
			/* Mark the channel */
			sc->sc_curchan.privFlags &= ~CHANNEL_DFS_CLEAR;
			sc->sc_curchan.privFlags |= CHANNEL_INTERFERENCE;
			/* notify 80211 layer so it can change channels... */
			ieee80211_mark_dfs(ic, &ichan);
		}
	}
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

/* A filter for hiding the addresses we don't think are very interesting or
 * which have adverse side effects. Return AH_TRUE if the address should be
 * exlucded, and AH_FALSE otherwise. */
#ifdef ATH_REVERSE_ENGINEERING
static HAL_BOOL
ath_regdump_filter(struct ath_softc *sc, u_int32_t address)
{
#ifndef ATH_REVERSE_ENGINEERING_WITH_NO_FEAR
	char buf[MAX_REGISTER_NAME_LEN];
#endif
	if ((ar_device(sc->devid) != 5212) && (ar_device(sc->devid) != 5213)) 
		return AH_TRUE;
	/* Addresses with side effects are never dumped out by bulk debug 
	 * dump routines. */
	if ((address >= 0x00c0) && (address <= 0x00df)) return AH_TRUE;
	if ((address >= 0x143c) && (address <= 0x143f)) return AH_TRUE;
	/* PCI timing registers are not interesting */
	if ((address >= 0x4000) && (address <= 0x5000)) return AH_TRUE;
	/* Reading 0x0920-0x092c causes crashes in turbo A mode? */
	if ((address >= 0x0920) && (address <= 0x092c)) return AH_TRUE;

#ifndef ATH_REVERSE_ENGINEERING_WITH_NO_FEAR
	/* We are being conservative, and do not want to access addresses that
	 * may crash the system, so we will only consider addresses we know
	 * the names of from previous reverse engineering efforts (AKA
	 * openHAL). */
	return (AH_TRUE == ath_hal_lookup_register_name(sc->sc_ah, buf, 
				MAX_REGISTER_NAME_LEN, address)) ?
		AH_FALSE : AH_TRUE;
#else /* #ifndef ATH_REVERSE_ENGINEERING_WITH_NO_FEAR */

	return AH_FALSE;
#endif /* #ifndef ATH_REVERSE_ENGINEERING_WITH_NO_FEAR */
}
#endif /* #ifdef ATH_REVERSE_ENGINEERING */

/* Dump any Atheros registers we think might be interesting. */
#ifdef ATH_REVERSE_ENGINEERING
static void
ath_ar5212_registers_dump(struct ath_softc *sc)
{
	unsigned int address = MIN_REGISTER_ADDRESS;
	unsigned int value   = 0;

	do {
		if (ath_regdump_filter(sc, address))
			continue;
		value = ath_reg_read(sc, address);
		ath_hal_print_decoded_register(sc->sc_ah, SC_DEV_NAME(sc),
				address, value, value, 
				AH_FALSE);
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
	unsigned int *p_old  = 0;

	do {
		if (ath_regdump_filter(sc, address))
			continue;
		value = ath_reg_read(sc, address);
		p_old = (unsigned int *)&sc->register_snapshot[address];
		if (*p_old != value) {
			ath_hal_print_decoded_register(sc->sc_ah, SC_DEV_NAME(sc), 
					address, *p_old, value, AH_FALSE);
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
		*((unsigned int *)&sc->register_snapshot[address]) =
			ath_regdump_filter(sc, address) ?
			0x0 : ath_reg_read(sc, address);
	} while ((address += 4) < MAX_REGISTER_ADDRESS);
}
#endif /* #ifdef ATH_REVERSE_ENGINEERING */

/* Read an Atheros register...for reverse engineering. */
#ifdef ATH_REVERSE_ENGINEERING
static unsigned int
ath_read_register(struct ieee80211com *ic, unsigned int address, 
		unsigned int *value)
{
	struct ath_softc *sc = ic->ic_dev->priv;
	if (address >= MAX_REGISTER_ADDRESS) {
		IPRINTF(sc, "Illegal Atheros register access "
				"attempted: 0x%04x >= 0x%04x\n",
				address, MAX_REGISTER_ADDRESS);
		return 1;
	}
	if (address % 4) {
		IPRINTF(sc, "Illegal Atheros register access "
				"attempted: 0x%04x %% 4 != 0\n",
				address);
		return 1;
	}
	*value = ath_reg_read(sc, address);
	IPRINTF(sc, "*0x%04x -> 0x%08x\n", address, *value);
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
		IPRINTF(sc, "Illegal Atheros register access "
				"attempted: 0x%04x >= 0x%04x\n",
				address,
				MAX_REGISTER_ADDRESS);
		return 1;
	}
	if (address % 4) {
		IPRINTF(sc, "Illegal Atheros register access "
				"attempted: 0x%04x %% 4 != 0\n",
				address);
		return 1;
	}
	ath_reg_write(sc, address, value);
	IPRINTF(sc, "*0x%04x <- 0x%08x = 0x%08x\n", address, value,
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
ath_return_txbuf_locked(struct ath_softc *sc, struct ath_buf **bf) 
{
	struct ath_buf *bfaddr;
	ATH_TXBUF_LOCK_ASSERT(sc);

	if ((bf == NULL) || ((*bf) == NULL)) 
		return;
	bfaddr = *bf;
	cleanup_ath_buf(sc, (*bf), BUS_DMA_TODEVICE);
	STAILQ_INSERT_TAIL(&sc->sc_txbuf, (*bf), bf_list);
	*bf = NULL;
	atomic_dec(&sc->sc_txbuf_counter);
#ifdef IEEE80211_DEBUG_REFCNT
	DPRINTF(sc, ATH_DEBUG_TXBUF, 
		"[TXBUF=%03d/%03d] returned txbuf %p.\n", 
		ath_get_buffer_count(sc), ATH_TXBUF);
	if (DFLAG_ISSET(sc, ATH_DEBUG_TXBUF))
		dump_stack();
#endif /* #ifdef IEEE80211_DEBUG_REFCNT */
	if (netif_queue_stopped(sc->sc_dev) && 
	    (ath_get_buffers_available(sc) > ATH_TXBUF_MGT_RESERVED) && 
	    (!ath_chan_unavail(sc))) {
		DPRINTF(sc, ATH_DEBUG_TXBUF | ATH_DEBUG_RESET, 
			"Waking device queue with %d available buffers.\n", 
			ath_get_buffers_available(sc));
		netif_wake_queue(sc->sc_dev);
	}
#if 0
	else if (ath_chan_unavail(sc)) {
		DPRINTF(sc, (ATH_DEBUG_TXBUF | ATH_DEBUG_RESET | 
					ATH_DEBUG_DOTH), 
				"Not waking device queue.  Channel "
				"is not available.\n");
	}
#endif
}

/* Takes the TXBUF_LOCK */
static void 
ath_return_txbuf(struct ath_softc *sc, struct ath_buf **bf) 
{
	ATH_TXBUF_LOCK_IRQ(sc);
	ath_return_txbuf_locked(sc, bf);
	ATH_TXBUF_UNLOCK_IRQ(sc);
}

/* Takes the lock */
static void 
ath_return_txbuf_list(struct ath_softc *sc, ath_bufhead *bfhead) 
{
	if (!bfhead)
		return;
	ATH_TXBUF_LOCK_IRQ(sc);
	if (!STAILQ_EMPTY(bfhead)) {
		struct ath_buf *tbf, *nextbf;
		STAILQ_FOREACH_SAFE(tbf, bfhead, bf_list, nextbf) {
			ath_return_txbuf_locked(sc, &tbf);
		}
	}
	ATH_TXBUF_UNLOCK_IRQ(sc);
	STAILQ_INIT(bfhead);
}
/* Caller must have the lock */
static void 
ath_return_txbuf_list_locked(struct ath_softc *sc, ath_bufhead *bfhead)
{
	ATH_TXBUF_LOCK_ASSERT(sc);
	if (!bfhead)
		return;
	
	if (!STAILQ_EMPTY(bfhead)) {
		struct ath_buf *tbf, *nextbf;
		STAILQ_FOREACH_SAFE(tbf, bfhead, bf_list, nextbf) {
			ath_return_txbuf_locked(sc, &tbf);
		}
	}
	STAILQ_INIT(bfhead);
}

static struct ath_buf * 
cleanup_ath_buf(struct ath_softc *sc, struct ath_buf *bf, int direction) 
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
		struct sk_buff *next_ffskb = NULL;
		/* Start with the second skb for FF */
		struct sk_buff *ffskb = bf->bf_skb ? 
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

	bf->bf_flags = 0;
	if (bf->bf_desc) {
		bf->bf_desc->ds_link = 0;
		bf->bf_desc->ds_data = 0;
	}

	ieee80211_dev_kfree_skb_list(&bf->bf_skb);

	bf->bf_taken_at_line = 0;
	bf->bf_taken_at_func = NULL;
	return bf;
}

#define SCANTXBUF_NAMSIZ 64
static inline int
descdma_contains_buffer(struct ath_descdma *dd, struct ath_buf *bf)
{
	return (bf >= (dd->dd_bufptr)) && 
		bf <  (dd->dd_bufptr + dd->dd_nbuf);
}

static inline int
descdma_index_of_buffer(struct ath_descdma *dd, struct ath_buf *bf)
{
	if (!descdma_contains_buffer(dd, bf))
		return -1;
	return bf - dd->dd_bufptr;
}

static inline struct ath_buf *
descdma_get_buffer(struct ath_descdma *dd, int index)
{
	KASSERT((index >= 0 && index < dd->dd_nbuf), 
		("Invalid index, %d, requested for %s dma buffers", index, dd->dd_name));
	return dd->dd_bufptr + index;
}

static int ath_debug_iwpriv(struct ieee80211com *ic, 
		unsigned int param, unsigned int value)
{
	struct ath_softc *sc = ic->ic_dev->priv;
	switch (param) {
	case IEEE80211_PARAM_DRAINTXQ:
		printk("Draining tx queue...\n");
		ath_draintxq(sc);
		break;
	case IEEE80211_PARAM_STOP_QUEUE:
		printk("Stopping device queue...\n");
		netif_stop_queue(sc->sc_dev);
		break;
	case IEEE80211_PARAM_ATHRESET:
		printk("Forcing device to reset...\n");
		ath_reset(sc->sc_dev);
		break;
	case IEEE80211_PARAM_TXTIMEOUT:
		printk("Simulating tx watchdog timeout...\n");
		ath_tx_timeout(sc->sc_dev);
		break;
	case IEEE80211_PARAM_RESETTXBUFS:
		printk("Brute force reset of tx buffer pool...\n");
		ATH_TXBUF_LOCK_IRQ(sc);
		netif_stop_queue(sc->sc_dev);
		ath_draintxq(sc);
		ath_descdma_cleanup(sc, &sc->sc_txdma, &sc->sc_txbuf,
			BUS_DMA_TODEVICE);
		ath_descdma_setup(sc, &sc->sc_txdma, &sc->sc_txbuf,
				"tx", ATH_TXBUF, ATH_TXDESC);
		atomic_set(&sc->sc_txbuf_counter, ATH_TXBUF);
		ATH_TXBUF_UNLOCK_IRQ(sc);
		break;
	case IEEE80211_PARAM_LEAKTXBUFS:
		{
			int j;
			for (j = 0; j < value; j++) {
				if (!ath_take_txbuf_mgmt(sc)) {
					printk("Leaked %d tx buffers (of the %d tx buffers requested).\n", j, value);
					return 0;
				}
			}
			printk("Leaked %d tx buffers.\n", value);
		}
		break;
	case IEEE80211_PARAM_SCANBUFS:
		printk("Scanning DMA buffer heaps...\n");
		ath_scanbufs(sc);
		break;
	default:
		printk("%s: Unknown command: %d.\n", __func__, param);
		return -EINVAL;
	}

	return 0;
}

static void
ath_scanbufs_found_buf_locked(struct ath_softc *sc, struct ath_descdma *dd, 
				unsigned long *dd_bufs_found, struct ath_buf *tbf,
				const char *context)
{
	int index = descdma_index_of_buffer(dd, tbf);
	if (-1 != index) {
		printk("FOUND %s[%3d] (%p) in %s.  status: 0x%08x.\n", 
			dd->dd_name, index, tbf, context, tbf->bf_status);
		if (test_and_set_bit(index, dd_bufs_found)) {
			printk("FOUND %s[%3d] (%p) multiple times!!!\n", 
				dd->dd_name, index, tbf);
		}
	}
#if 0 
/* XXX: To make this work right, must know which hw queues are used 
 *      with each kind of buffer, and make a more selective call... */
	else {
		EPRINTF(sc, "Detected a martian %s (%p) in %s!!\n", 
			dd->dd_name, tbf, context);
	}
#endif
}

static void
ath_scanbufs_in_buflist_locked(struct ath_softc *sc, struct ath_descdma *dd, 
				 unsigned long *dd_bufs_found, ath_bufhead *bufs, 
				 const char *context)
{
	struct ath_buf *tbf;
	STAILQ_FOREACH(tbf, bufs, bf_list) {
		ath_scanbufs_found_buf_locked(sc, dd, dd_bufs_found, tbf, 
			context);
	}
}

static void 
ath_scanbufs_in_txq_locked(struct ath_softc *sc, struct ath_descdma *dd, 
		unsigned long *dd_bufs_found, struct ath_txq *txq, 
		const char *context)
{
	struct ath_buf *tbf;
	char sacontext[SCANTXBUF_NAMSIZ];

	STAILQ_FOREACH(tbf, &txq->axq_q, bf_list) {
		ath_scanbufs_found_buf_locked(sc, dd, dd_bufs_found, tbf, 
			context);
	}

	snprintf(sacontext, sizeof(sacontext), "%s staging area", context);

#ifdef ATH_SUPERG_FF
	TAILQ_FOREACH(tbf, &txq->axq_stageq, bf_stagelist) {
		ath_scanbufs_found_buf_locked(sc, dd, dd_bufs_found, tbf, 
			sacontext);
	}
#endif
}

static void
ath_scanbufs_in_vap_locked(struct ath_softc *sc, struct ath_descdma *dd, 
		unsigned long *dd_bufs_found, struct ath_vap *av)
{
	char context[SCANTXBUF_NAMSIZ];
	if (av->av_bcbuf && dd == &sc->sc_bdma) {
		snprintf(context, sizeof(context), "vap %s %p[" MAC_FMT 
			 "] bcbuf*", 
			 DEV_NAME(av->av_vap.iv_dev), av, 
			 MAC_ADDR(av->av_vap.iv_bssid));
		ath_scanbufs_found_buf_locked(sc, dd, dd_bufs_found, 
						av->av_bcbuf, context);
	}
	else if (dd == &sc->sc_txdma) {
		struct ath_buf *tbf = NULL;

		snprintf(context, sizeof(context), "vap %s %p[" MAC_FMT
			 "] mcast queue", 
			 DEV_NAME(av->av_vap.iv_dev), av, 
			 MAC_ADDR(av->av_vap.iv_bssid));
		STAILQ_FOREACH(tbf, &av->av_mcastq.axq_q, bf_list) {
			ath_scanbufs_found_buf_locked(sc, dd, dd_bufs_found, 
							tbf, context);
		}

		snprintf(context, sizeof(context), "vap %s %p[" MAC_FMT 
			 "] mcast queue staging area", 
			 DEV_NAME(av->av_vap.iv_dev), av, 
			 MAC_ADDR(av->av_vap.iv_bssid));

#ifdef ATH_SUPERG_FF
		TAILQ_FOREACH(tbf, &av->av_mcastq.axq_stageq, bf_stagelist) {
			ath_scanbufs_found_buf_locked(sc, dd, dd_bufs_found, tbf, 
				context);
		}
#endif
	}
}

static void
ath_scanbufs_in_all_vaps_locked(struct ath_softc *sc, struct ath_descdma *dd, 
		unsigned long *dd_bufs_found)
{
	struct ieee80211vap *vap;
	TAILQ_FOREACH(vap, &sc->sc_ic.ic_vaps, iv_next) {
		ath_scanbufs_in_vap_locked(sc, dd, dd_bufs_found, ATH_VAP(vap));
	}
}

static void
ath_scanbufs_in_all_nodetable_locked(struct ath_softc *sc, struct ath_descdma *dd, 
				 unsigned long *dd_bufs_found, 
				 struct ieee80211_node_table *nt)
{
	struct ieee80211_node *node = NULL;
	struct ath_node *athnode = NULL;
	char context[SCANTXBUF_NAMSIZ];
	TAILQ_FOREACH(node, &nt->nt_node, ni_list) {
		athnode = ATH_NODE(node);

		snprintf(context,  sizeof(context), 
			 "node %p[" MAC_FMT " on %s[" MAC_FMT "]] uapsd_q", 
			 athnode, 
			 MAC_ADDR(athnode->an_node.ni_macaddr), 
			 DEV_NAME(athnode->an_node.ni_vap->iv_dev),
			 MAC_ADDR(athnode->an_node.ni_bssid));
		ath_scanbufs_in_buflist_locked(sc, dd, dd_bufs_found, 
				&athnode->an_uapsd_q, context);

		snprintf(context,  sizeof(context), 
			 "node %p[" MAC_FMT " on %s[" MAC_FMT "]] uapsd_overflowq", 
			 athnode, 
			 MAC_ADDR(athnode->an_node.ni_macaddr), 
			 DEV_NAME(athnode->an_node.ni_vap->iv_dev),
			 MAC_ADDR(athnode->an_node.ni_bssid));
		ath_scanbufs_in_buflist_locked(sc, dd, dd_bufs_found, 
				&athnode->an_uapsd_overflowq, 
                                           context);
	}
}

static void
ath_scanbufs_in_all_hwtxq_locked(struct ath_softc *sc,
				 struct ath_descdma *dd, 
				 unsigned long *dd_bufs_found)
{
	int 		q = HAL_NUM_TX_QUEUES;
	char context[SCANTXBUF_NAMSIZ];
	while (--q >= 0)
		if (ATH_TXQ_SETUP(sc, q)) {
			snprintf(context, sizeof(context), "txq[%d]", q);
			ath_scanbufs_in_txq_locked(sc, dd,
				dd_bufs_found, &sc->sc_txq[q], context);
		}
}
static void
ath_scanbufs_print_leaks(struct ath_softc *sc,
		struct ath_descdma *dd, 
		unsigned long *dd_bufs_found)
{
	int index;
	struct ath_buf *lostbf;
	for (index = 0; index < dd->dd_nbuf; index++) {
		if (!test_bit(index, dd_bufs_found)) {
			lostbf = descdma_get_buffer(dd, index);
			/* XXX: Full alloc backtrace */
			EPRINTF(sc, "LOST %s[%3d] (%p) -- "
				"taken at line %s:%d!!\n", 
				dd->dd_name, index, lostbf, 
				(lostbf->bf_taken_at_func ? 
				 lostbf->bf_taken_at_func : "(null)"), 
				lostbf->bf_taken_at_line);
		}
	}
}

static void
ath_scanbufs(struct ath_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node_table *nt = &ic->ic_sta;
	int qnum = 0;
	struct ieee80211_node *ni;
	int i;
	/* Set up a list of dma areas to scan for.  Unfortunately the locks
	 * are all external to this, so they were specified above with the 
	 * standard lock macros... */
	struct ath_descdma *descdma[] = {
		&sc->sc_txdma,
		&sc->sc_rxdma,
		&sc->sc_bdma,
		&sc->sc_grppolldma
	};
	struct ath_descdma *dd;
	unsigned long *dd_bufs_found;

	/* NB: Locking sequence is critical to avoid deadlocks !!! */
	IEEE80211_LOCK_IRQ(&sc->sc_ic);
	IEEE80211_SCAN_LOCK_IRQ(nt);
	IEEE80211_NODE_TABLE_LOCK_IRQ(nt);
	ATH_LOCK(sc);
	ATH_HAL_LOCK_IRQ(sc);
	ATH_GBUF_LOCK_IRQ(sc);
	ATH_BBUF_LOCK_IRQ(sc);
	ATH_TXBUF_LOCK_IRQ(sc);
	ATH_RXBUF_LOCK_IRQ(sc);
	for (qnum = 0; qnum < HAL_NUM_TX_QUEUES; qnum++) {
		if (ATH_TXQ_SETUP(sc, qnum)) { 
			ATH_TXQ_LOCK_IRQ_INSIDE(&sc->sc_txq[qnum]);
		}
	}
	TAILQ_FOREACH(ni, &nt->nt_node, ni_list) {
		/* IEEE80211_NODE_LOCK_IRQ_INSIDE(ni); NB: ifdef'd out */
		IEEE80211_NODE_SAVEQ_LOCK_IRQ_INSIDE(ni);
		ATH_NODE_UAPSD_LOCK_IRQ_INSIDE(ATH_NODE(ni));
	}

	/* NB: We have all you base... */
	for (i = 0; i < ARRAY_SIZE(descdma); i++) {
		printk("\n");
		dd = descdma[i];
		if (dd->dd_bufptr) {
			printk("Analyzing %s DMA buffers...\n", dd->dd_name);
			dd_bufs_found = kzalloc(BITS_TO_LONGS(dd->dd_nbuf) * 
						sizeof(unsigned long), GFP_KERNEL);
			if (dd == &sc->sc_txdma) {
				ath_scanbufs_in_buflist_locked(sc, dd, dd_bufs_found, 
							       &sc->sc_txbuf, "free list");
				ath_scanbufs_in_all_hwtxq_locked(sc, dd, dd_bufs_found);
				ath_scanbufs_in_all_vaps_locked(sc, dd, dd_bufs_found);
				ath_scanbufs_in_all_nodetable_locked(sc, dd, dd_bufs_found, nt);
			}
			else if (dd == &sc->sc_rxdma) {
				ath_scanbufs_in_buflist_locked(sc, dd, dd_bufs_found, 
							       &sc->sc_rxbuf, "queue");
			}
			else if (dd == &sc->sc_bdma) {
				ath_scanbufs_in_buflist_locked(sc, dd, dd_bufs_found, 
							       &sc->sc_bbuf, "free list");
				ath_scanbufs_in_all_vaps_locked(sc, dd, dd_bufs_found);
				ath_scanbufs_in_all_hwtxq_locked(sc, dd, dd_bufs_found);
			}
			else if (dd == &sc->sc_grppolldma) {
				ath_scanbufs_in_buflist_locked(sc, dd, dd_bufs_found, 
							       &sc->sc_grppollbuf, "free list");
				ath_scanbufs_in_txq_locked(sc, dd, dd_bufs_found, 
							   &sc->sc_grpplq, "group poll queue");
			}

			/* print out what we found was missing and what line led to it */
			ath_scanbufs_print_leaks(sc, dd, dd_bufs_found);
			kfree(dd_bufs_found);
			dd_bufs_found = NULL;
		}
	}
	printk("\n");
	/* NB: Reversing above locking sequence is critical to avoid deadlocks !!! */
	TAILQ_FOREACH(ni, &nt->nt_node, ni_list) {
		/* IEEE80211_NODE_UNLOCK_IRQ_INSIDE(ni); NB: ifdef'd out */
		IEEE80211_NODE_SAVEQ_UNLOCK_IRQ_INSIDE(ni);
		ATH_NODE_UAPSD_UNLOCK_IRQ_INSIDE(ATH_NODE(ni));
	}
	for (qnum = HAL_NUM_TX_QUEUES - 1; qnum >= 0; qnum--) {
		if (ATH_TXQ_SETUP(sc, qnum))
			ATH_TXQ_UNLOCK_IRQ_INSIDE(&sc->sc_txq[qnum]);
	}
	ATH_RXBUF_UNLOCK_IRQ(sc);
	ATH_TXBUF_UNLOCK_IRQ(sc);
	ATH_BBUF_UNLOCK_IRQ(sc);
	ATH_GBUF_UNLOCK_IRQ(sc);
	ATH_HAL_UNLOCK_IRQ(sc);
	ATH_UNLOCK(sc);
	IEEE80211_NODE_TABLE_UNLOCK_IRQ(nt);
	IEEE80211_SCAN_UNLOCK_IRQ(nt);
	IEEE80211_UNLOCK_IRQ(ic);
}
