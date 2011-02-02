/*
 * This software is distributed under the terms of the
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
 * $Id: if_ath_radar.c 2464 2007-06-15 22:51:56Z mtaylor $
 */
#include "opt_ah.h"

#include "if_ath_debug.h"

#if !defined(AUTOCONF_INCLUDED) && !defined(CONFIG_LOCALVERSION)
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
#include <linux/param.h>

#include "if_ethersubr.h"	/* for ETHER_IS_MULTICAST */
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

#include "ah_devid.h"		/* XXX to identify chipset */

#ifdef ATH_PCI			/* PCI BUS */
#include "if_ath_pci.h"
#endif				/* PCI BUS */
#ifdef ATH_AHB			/* AHB BUS */
#include "if_ath_ahb.h"
#endif				/* AHB BUS */

#undef MAX
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))

#undef MIN
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))

#undef SQR
#define SQR(x)		((x)*(x))

#include "ah.h"
#include "if_ath_hal.h"

#ifdef ATH_TX99_DIAG
#include "ath_tx99.h"
#endif

#include "ah_os.h"
#include "if_ath_radar.h"

#define sizetab(t) (sizeof(t)/sizeof(t[0]))
#define nofloat_pct(_value, _pct) \
	( (_value * (1000 + _pct)) / 1000 )

/* convert units used for the rp_width into us. This conversion is based on
 * measure and might not be very accurate */
#define WIDTH_TO_TSF(x)	(((x) * 200) / 256)

struct radar_pattern_specification {
	/* The name of the rule/specification (i.e. what did we detect) */
	const char *name;
	/* Interval MIN = 1000000 / FREQ - 0.1%
	 * (a.k.a. Pulse/Burst Repetition Interval) */
	u_int32_t min_rep_int;
	/* Interval MAX = 1000000 / FREQ + 0.1%
	 * (a.k.a. Pulse/Burst Repetition Interval) */
	u_int32_t max_rep_int;
	/* AH_FALSE for ETSI radars, AH_TRUE for FCC radars. Used to adjust
	 * the timestamp of the pulse using the pulse width */
	HAL_BOOL is_fcc;
	/* Fuzz factor dynamic matching, as unsigned integer percentage 
	 * of variation (i.e. 2 for +/- 2% timing) */
	u_int32_t fuzz_pct;
	/* Match MIN (Minimum Pulse/Burst events required) */
	u_int32_t min_pulse;
	/* Match MIN duration (Minimum Pulse/Burst events 
	 * required including missed) */
	u_int32_t min_evts;
	/* Match MAX duration (Maximum Pulse/Burst events 
	 * required including missed) */
	u_int32_t max_evts;
	/* Maximum consecutive missing pulses */
	u_int32_t max_consecutive_missing;
	/* Maximum missing pulses */
	u_int32_t max_missing;
	/* Match on absolute distance to PRI/PRF midpoint */
	HAL_BOOL match_midpoint;
};

static struct radar_pattern_specification radar_patterns[] = {
#ifdef DFS_DOMAIN_ETSI
	/* ETSI - Type 2 - 1,2,5us - PRF 200 - BURST 10 or
	   ETSI - Type 3 - 10,15us - PRF 200 - BURST 15 */
	{"ETSI - PRF200",  4995, 5005, AH_FALSE, 20, 4, 4, 15, 4,  8, AH_TRUE},
	/* ETSI - Type 2 - 1,2,5us - PRF 300 - BURST 10 or
	   ETSI - Type 3 - 10,15us - PRF 300 - BURST 15 */
	{"ETSI - PRF300",  3329, 3337, AH_FALSE, 20, 4, 4, 15, 4,  6, AH_TRUE},
	/* ETSI - Type 2 - 1,2,5us - PRF 500 - BURST 10 or
	   ETSI - Type 3 - 10,15us - PRF 500 - BURST 15 */
	{"ETSI - PRF500",  1998, 2002, AH_FALSE, 20, 4, 4, 15, 4,  8, AH_TRUE},
	/* ETSI - Type 1 - 1us - PRF 750 - BURST 15 */
	{"ETSI - PRF750",  1331, 1335, AH_FALSE, 20, 5, 4, 15, 4, 13, AH_TRUE},
	/* ETSI - Type 2 - 1,2,5us - PRF 800 - BURST 10 or
	   ETSI - Type 3 - 10,15us - PRF 800 - BURST 15 */
	{"ETSI - PRF800",  1248, 1252, AH_FALSE, 20, 4, 4, 15, 4,  8, AH_TRUE},
	/* ETSI - Type 2 - 1,2,5us - PRF 1000 - BURST 10 or
	   ETSI - Type 3 - 10,15us - PRF 1000 - BURST 15 */
	{"ETSI - PRF1000",  999, 1001, AH_FALSE, 20, 4, 4, 15, 4,  8, AH_TRUE},
	/* ETSI - Type 4 - 1,2,5,10,15us - PRF 1200 - BURST 15 */
	{"ETSI - PRF1200",  832,  834, AH_FALSE, 20, 5, 4, 15, 4, 13, AH_TRUE},
	/* ETSI - Type 4 - 1,2,5,10,15us - PRF 1500 - BURST 15 */
	{"ETSI - PRF1500",  665,  667, AH_FALSE, 20, 5, 4, 15, 4,  6, AH_TRUE},
	/* ETSI - Type 4 - 1,2,5,10,15us - PRF 1600 - BURST 15 */
	{"ETSI - PRF1600",  624,  626, AH_FALSE, 20, 5, 4, 15, 4,  7, AH_TRUE},
	/* ETSI - Type 6 - 20,30us - PRF 2000 - BURST 20 */
	{"ETSI - PRF2000",  499,  501, AH_FALSE, 20, 6, 4, 20, 4, 10, AH_TRUE},
	/* ETSI - Type 5 - 1,2,5,10,15us - PRF 2300 - BURST 25 */
	{"ETSI - PRF2300",  432,  435, AH_FALSE, 20, 8, 4, 25, 6, 20, AH_TRUE},
	/* ETSI - Type 5 - 1,2,5,10,15us - PRF 3000 - BURST 25 or
	   ETSI - Type 6 - 20,30us - PRF 3000 - BURST 20 */
	{"ETSI - PRF3000",  332,  334, AH_FALSE, 20, 6, 4, 25, 5, 20, AH_TRUE},
	/* ETSI - Type 5 - 1,2,5,10,15us - PRF 3500 - BURST 25 */
	{"ETSI - PRF3500",  284,  286, AH_FALSE, 20, 8, 4, 25, 2, 20, AH_TRUE},
	/* ETSI - Type 5 - 1,2,5,10,15us - PRF 4000 - BURST 25 or
	   ETSI - Type 6 - 20,30us - PRF 4000 - BURST 20 */
	{"ETSI - PRF4000",  249,  251, AH_FALSE, 20, 6, 4, 25, 5, 20, AH_TRUE},
#endif
#ifdef DFS_DOMAIN_FCC
	/* FCC - Type 1 -     1us -    PRI 1428 - BURST 18 */
	{"FCC - Type 1",   1426, 1430, AH_TRUE,  10, 6, 6, 18, 4,  6, AH_FALSE},
	/* FCC - Type 2 -   1/5us - PRI 150/230 - BURST 23/29 */
	{"FCC - Type 2",    149,  231, AH_TRUE,  10, 8, 8, 29, 6, 12, AH_FALSE},
	/* FCC - Type 3 -  6/10us - PRI 200/500 - BURST 16/18 */
	{"FCC - Type 3",    199,  501, AH_TRUE,  10, 6, 6, 18, 6, 12, AH_FALSE},
	/* FCC - Type 4 - 11/20us - PRI 200/500 - BURST 12/16 */
	{"FCC - Type 4",    199,  501, AH_TRUE,  10, 5, 5, 16, 6, 12, AH_FALSE}
#endif
};

#ifdef AR_DEBUG
static u_int32_t interval_to_frequency(u_int32_t pri);
#endif /* AR_DEBUG */

/* Returns true if radar detection is enabled. */
int ath_radar_is_enabled(struct ath_softc *sc)
{
	struct ath_hal *ah = sc->sc_ah;
	if (ar_device(sc) >= 5211)
		return ((OS_REG_READ(ah, AR5K_AR5212_PHY_ERR_FIL) & 
			 AR5K_AR5212_PHY_ERR_FIL_RADAR) && 
			(sc->sc_imask & HAL_INT_RXPHY) && 
			(ath_hal_intrget(ah) & HAL_INT_RXPHY));
	else
		return ((sc->sc_imask & HAL_INT_RXPHY) && 
			(ath_hal_intrget(ah) & HAL_INT_RXPHY));
	return 0;
}

/* Read the radar pulse detection parameters. */
void ath_radar_get_params(struct ath_softc *sc, RADAR_PARAM *rp)
{
	u_int32_t radar = ath_reg_read(sc, AR5K_PHY_RADAR);
	rp->rp_fir_filter_output_power_thr = 
		(radar & AR5K_PHY_RADAR_FIRPWROUTTHR) >> 
		AR5K_PHY_RADAR_FIRPWROUTTHR_S;
	rp->rp_radar_rssi_thr = 
		(radar & AR5K_PHY_RADAR_PULSERSSITHR) >> 
		AR5K_PHY_RADAR_PULSERSSITHR_S;
	rp->rp_pulse_height_thr = 
		(radar & AR5K_PHY_RADAR_PULSEHEIGHTTHR) >> 
		AR5K_PHY_RADAR_PULSEHEIGHTTHR_S;
	rp->rp_pulse_rssi_thr = 
		(radar & AR5K_PHY_RADAR_RADARRSSITHR) >> 
		AR5K_PHY_RADAR_RADARRSSITHR_S;
	rp->rp_inband_thr = 
		(radar & AR5K_PHY_RADAR_INBANDTHR) >> 
		AR5K_PHY_RADAR_INBANDTHR_S;
}

/* Update the radar pulse detection parameters. 
 * If rp is NULL, defaults are used for all fields.
 * If any member of rp is set to RADAR_PARAM_USE_DEFAULT, the default
 * is used for that field. */
void ath_radar_set_params(struct ath_softc *sc, RADAR_PARAM *rp)
{
#define BUILD_PHY_RADAR_FIELD(_MASK,_SHIFT,_FIELD) \
	((NULL == rp || (rp->_FIELD == RADAR_PARAM_USE_DEFAULT)) ? \
		((AR5K_PHY_RADAR_ENABLED_AR5213 & (_MASK))) : \
		((rp->_FIELD << (_SHIFT)) & (_MASK)))
	ath_reg_write(sc, AR5K_PHY_RADAR,
	    BUILD_PHY_RADAR_FIELD(AR5K_PHY_RADAR_FIRPWROUTTHR,
		AR5K_PHY_RADAR_FIRPWROUTTHR_S,
		rp_fir_filter_output_power_thr) |
	    BUILD_PHY_RADAR_FIELD(AR5K_PHY_RADAR_RADARRSSITHR,
		AR5K_PHY_RADAR_RADARRSSITHR_S,
		rp_pulse_rssi_thr) |
	    BUILD_PHY_RADAR_FIELD(AR5K_PHY_RADAR_PULSEHEIGHTTHR,
		AR5K_PHY_RADAR_PULSEHEIGHTTHR_S,
		rp_pulse_height_thr) |
	    BUILD_PHY_RADAR_FIELD(AR5K_PHY_RADAR_PULSERSSITHR,
		AR5K_PHY_RADAR_PULSERSSITHR_S, rp_radar_rssi_thr) | 
		      BUILD_PHY_RADAR_FIELD(AR5K_PHY_RADAR_INBANDTHR, 
					    AR5K_PHY_RADAR_INBANDTHR_S, 
					    rp_inband_thr)
	    );
#undef BUILD_PHY_RADAR_FIELD
}

/* This is called on channel change to enable radar detection for 5211+ chips.  
 * NOTE: AR5210 doesn't have radar pulse detection support. */
int ath_radar_update(struct ath_softc *sc)
{

	struct ath_hal *ah = sc->sc_ah;
	struct ieee80211com *ic = &sc->sc_ic;
	int required = 0;

	/* Do not attempt to change radar state when bg scanning is
	 * the cause */
	if (ic->ic_flags & IEEE80211_F_SCAN)
		return 1;

	/* Update the DFS flags (as a sanity check) */
	if (ath_radar_correct_dfs_flags(sc, &sc->sc_curchan))
		DPRINTF(sc, ATH_DEBUG_DOTH, "channel required "
			"corrections to private flags.\n");
	required = ((sc->sc_curchan.privFlags & CHANNEL_DFS) &&
		    (ic->ic_flags & IEEE80211_F_DOTH));
	/* configure radar pulse detector register using default values, but do
	 * not toggle the enable bit.  XXX: allow tweaking?? */
	ath_radar_set_params(sc, NULL);
	if (ar_device(sc) >= 5211) {
		HAL_INT old_ier = ath_hal_intrget(ah);
		HAL_INT new_ier = old_ier;
		unsigned int old_radar = OS_REG_READ(ah, AR5K_PHY_RADAR);
		unsigned int old_filter = 
			OS_REG_READ(ah, AR5K_AR5212_PHY_ERR_FIL);
		unsigned int old_rxfilt = ath_hal_getrxfilter(ah);
		unsigned int old_mask = sc->sc_imask;
		unsigned int new_radar = old_radar;
		unsigned int new_filter = old_filter;
		unsigned int new_mask = old_mask;
		unsigned int new_rxfilt = old_rxfilt;

		ath_hal_intrset(ah, old_ier & ~HAL_INT_GLOBAL);
		if (required) {
			new_radar |= AR5K_PHY_RADAR_ENABLE;
			new_filter |= AR5K_AR5212_PHY_ERR_FIL_RADAR;
			new_rxfilt |= HAL_RX_FILTER_PHYRADAR;
			new_mask |= HAL_INT_RXPHY;
			new_ier |= HAL_INT_RXPHY;
		} else {
			new_radar &= ~AR5K_PHY_RADAR_ENABLE;
			new_filter &= ~AR5K_AR5212_PHY_ERR_FIL_RADAR;
			new_rxfilt &= ~HAL_RX_FILTER_PHYRADAR;
			new_mask &= ~HAL_INT_RXPHY;
			new_ier &= ~HAL_INT_RXPHY;
		}

		if (old_filter != new_filter)
			OS_REG_WRITE(ah, AR5K_AR5212_PHY_ERR_FIL, new_filter);
		if (old_radar != new_radar)
			OS_REG_WRITE(ah, AR5K_PHY_RADAR, new_radar);
		if (old_rxfilt != new_rxfilt)
			ath_hal_setrxfilter(ah, new_rxfilt);

		sc->sc_imask = new_mask;
		if (DFLAG_ISSET(sc, ATH_DEBUG_DOTH) && 
		    ((old_radar != new_radar) || 
		     (old_filter != new_filter) || 
		     (old_rxfilt != new_rxfilt) || 
		     (old_mask != new_mask) || 
		     (old_ier != new_ier))) {
			DPRINTF(sc, ATH_DEBUG_DOTH, 
				"%s: %s: Radar detection %s.\n", SC_DEV_NAME(sc),
				__func__, required ? "enabled" : "disabled");
		}
		ath_hal_intrset(ah, new_ier);
	}

	return (required == ath_radar_is_enabled(sc));
}

static
int ath_radar_is_indoor_channel(HAL_CHANNEL *hchan)
{
	/* Warning : we use hardcoded values here suited for France */
	if ((hchan->channel >= 2412) && (hchan->channel <= 2472))
		return 1;
	if ((hchan->channel >= 5150) && (hchan->channel <= 5350))
		return 1;
	if ((hchan->channel >= 5470) && (hchan->channel <= 5725))
		return 1;

	return 0;
}

static
int ath_radar_is_outdoor_channel(HAL_CHANNEL *hchan)
{
	/* Warning : we use hardcoded values here suited for France */
	if ((hchan->channel >= 2412) && (hchan->channel <= 2472))
		return 1;
	if ((hchan->channel >= 5470) && (hchan->channel <= 5725))
		return 1;

	return 0;
}

/* Update channel's DFS flags based upon whether DFS is required.  Return true
 * if the value was repaired. It also add flags to know if a channel can be
 * used indoor or outdoor or both. Those flags have been added and made
 * compatible with HAL flags (as defined in <hal/ah.h> */

#define CHANNEL_INDOOR  0x00004
#define CHANNEL_OUTDOOR 0x00008

int ath_radar_correct_dfs_flags(struct ath_softc *sc, HAL_CHANNEL *hchan)
{
	u_int32_t old_channelFlags = hchan->channelFlags;
	u_int8_t old_privFlags = hchan->privFlags;
	int changed;

	if (ath_radar_is_dfs_required(sc, hchan)) {
		hchan->channelFlags |= CHANNEL_PASSIVE;
		hchan->privFlags |= CHANNEL_DFS;
	} else {
		hchan->channelFlags &= ~CHANNEL_PASSIVE;
		hchan->privFlags &= ~CHANNEL_DFS;
	}

	changed = ((old_privFlags != hchan->privFlags) ||
		   (old_channelFlags != hchan->channelFlags));

	hchan->channelFlags &= ~(CHANNEL_INDOOR | CHANNEL_OUTDOOR);

	if (ath_radar_is_indoor_channel(hchan)) {
		hchan->channelFlags |= CHANNEL_INDOOR;
	}

	if (ath_radar_is_outdoor_channel(hchan)) {
		hchan->channelFlags |= CHANNEL_OUTDOOR;
	}

	return changed;
}

/* Returns true if DFS is required for the regulatory domain, country and 
 * combination in use. 
 * XXX: Need to add regulatory rules in here.  This is too conservative! */
int ath_radar_is_dfs_required(struct ath_softc *sc, HAL_CHANNEL *hchan)
{
	/* For FCC: 5250 to 5350MHz (channel 52 to 60) and for Europe added 
	 * 5470 to 5725 MHz (channel 100 to 140). 
	 * Being conservative, go with the entire band from 5250-5725 MHz. */
	return ((hchan->channel >= 5250) && (hchan->channel <= 5725)) ? 1 : 0;
}

static struct ath_rp *pulse_head(struct ath_softc *sc)
{
	return list_entry(sc->sc_rp_list.next, 
			  struct ath_rp, list);
}

static struct ath_rp *pulse_tail(struct ath_softc *sc)
{
	return list_entry(sc->sc_rp_list.prev, 
			  struct ath_rp, list);
}

static struct ath_rp *pulse_prev(struct ath_rp *pulse)
{
	return list_entry(pulse->list.prev, 
			  struct ath_rp, list);
}

#define CR_FALLTHROUGH		0
#define CR_NULL			1
#define CR_EXCESS_INTERVALS	2
#define CR_INTERVALS 		3
#define CR_EXCESS_DURATION	4
#define CR_DURATION		5
#define CR_PULSES		6
#define CR_MISSES		7
#define CR_MIDPOINT_A		8
#define CR_MIDPOINT_B		9
#define CR_MIDPOINT_C		10
#define CR_NOISE		11

#define MR_MATCH 		0
#define MR_FAIL_MIN_INTERVALS	1
#define MR_FAIL_REQD_MATCHES	2
#define MR_FAIL_MAX_MISSES	3
#define MR_FAIL_MIN_PERIOD	4
#define MR_FAIL_MAX_PERIOD	5

#ifdef AR_DEBUG
static const char *get_match_result_desc(u_int32_t code)
{
	switch (code) {
	case MR_MATCH:
		return "MATCH";
	case MR_FAIL_MIN_INTERVALS:
		return "TOO-SHORT";
	case MR_FAIL_REQD_MATCHES:
		return "TOO-FEW";
	case MR_FAIL_MAX_MISSES:
		return "TOO-LOSSY";
	case MR_FAIL_MIN_PERIOD:
		return "PRI<MIN";
	case MR_FAIL_MAX_PERIOD:
		return "PRI>MAX";
	default:
		return "unknown";
	}
}
#endif /* AR_DEBUG */

static int32_t match_radar(
	u_int32_t matched, 
	u_int32_t missed, 
	u_int32_t mean_period, 
	u_int32_t noise,
	u_int32_t min_evts,
	u_int32_t max_evts,
	u_int32_t min_rep_int,
	u_int32_t max_rep_int,
	u_int32_t min_pulse,
	u_int32_t max_misses)
{
	/* Not a match: insufficient overall burst length */
	if ( (matched + missed) < min_evts)
		return MR_FAIL_MIN_INTERVALS;

	/* Not a match: insufficient match count */
	if (matched < min_pulse)
		return MR_FAIL_REQD_MATCHES;

	/* Not a match: too many missies */
	if (missed > max_misses)
		return MR_FAIL_MAX_MISSES;

	/* Not a match, PRI out of range */
	if (mean_period < min_rep_int)
		return MR_FAIL_MIN_PERIOD;

	/* Not a match, PRI out of range */
	if (mean_period > max_rep_int) 
		return MR_FAIL_MAX_PERIOD;

	return MR_MATCH;
}

static int32_t compare_radar_matches(
	int32_t a_matched, 
	int32_t a_missed, 
	int32_t a_mean_period, 
	int32_t a_noise,
	int32_t a_min_evts,
	int32_t a_max_evts,
	int32_t a_min_rep_int,
	int32_t a_max_rep_int,
	int32_t a_min_pulse,
	int32_t a_max_misses,
	HAL_BOOL  a_match_midpoint,
	int32_t b_matched, 
	int32_t b_missed, 
	int32_t b_mean_period, 
	int32_t b_noise,
	int32_t b_min_evts,
	int32_t b_max_evts,
	int32_t b_min_rep_int,
	int32_t b_max_rep_int,
	int32_t b_min_pulse,
	int32_t b_max_misses,
	HAL_BOOL  b_match_midpoint
	)
{
	/* Intermediate calculations */
	int32_t   a_total            = a_matched + a_missed;
	int32_t   b_total            = b_matched + b_missed;
	int32_t   a_excess_total     = 
		MAX((int32_t)(a_total - (int32_t)a_max_evts), 0);
	int32_t   b_excess_total     = 
		MAX((int32_t)(b_total - (int32_t)b_max_evts), 0);
	u_int64_t a_duration 	     = a_total * a_mean_period;
	u_int64_t b_duration         = b_total * b_mean_period;
	u_int64_t a_excess_duration  = a_excess_total * a_mean_period;
	u_int64_t b_excess_duration  = b_excess_total * b_mean_period;
	u_int64_t a_dist_from_pri_mid = labs(a_mean_period - 
		     (a_min_rep_int + 
		      ((a_max_rep_int - a_min_rep_int) / 2)));
	u_int64_t b_dist_from_pri_mid = labs(b_mean_period - 
		     (b_min_rep_int + 
		      ((b_max_rep_int - b_min_rep_int) / 2)));
	/* Did one radar have fewer excess total pulse intervals than the 
	 * other? */
	if (a_excess_total != b_excess_total)
		return ((a_excess_total < b_excess_total) ? 1 : -1) * 
			CR_EXCESS_INTERVALS;
	/* Was one pulse longer chronologically, even though totals matched? */
	else if (a_excess_duration != b_excess_duration)
		return ((a_excess_duration < b_excess_duration) ? 1 : -1) * 
			CR_EXCESS_DURATION;
	/* Did one get more matches? */
	if (a_matched != b_matched)
		return (a_matched > b_matched ? 1 : -1) * CR_PULSES;
	/* Both waveforms are the same length, same total. 
	 * Did one get more misses? */
	if (a_missed != b_missed)
		return (a_missed < b_missed ? 1 : -1) * CR_MISSES;
	/* Did one get more noise? */
	if (a_noise != b_noise)
		return (a_noise < b_noise ? 1 : -1) * CR_NOISE;
	/* If both waveforms were not too long in terms of intervals */
	if (0 == (a_excess_total+b_excess_total)) {
		/* Did one waveform have to match more events than the other? */
		if (a_total != b_total)
			return ((a_total > b_total) ? 1 : -1) * CR_INTERVALS;
		/* Was one waveform longer than the other */
		if (a_duration != b_duration)
			return ((a_duration > b_duration) ? 1 : -1) * CR_DURATION;
	}
	/* both durations are legal, but one is closer to the original PRF/PRI */
	if (a_dist_from_pri_mid != b_dist_from_pri_mid) {
		if (a_match_midpoint && b_match_midpoint) {
			/* Which pattern is closer to midpoint? */
			return ((a_dist_from_pri_mid < b_dist_from_pri_mid) ? 1 : -1) *
				CR_MIDPOINT_A;
		}
		else if (a_match_midpoint) {
			/* If not within spitting distance of midpoint, reject */
			return ((a_dist_from_pri_mid < 3) ? 1 : -1) * 
				CR_MIDPOINT_B;

		}
		else if (b_match_midpoint) {
			/* If not within spitting distance of midpoint, reject */
			return ((b_dist_from_pri_mid >= 3) ? 1 : -1) * 
				CR_MIDPOINT_C;
		}
	}
	return -CR_FALLTHROUGH;
}

#ifdef ATH_RADAR_LONG_PULSE

struct lp_burst {
	u_int32_t lpb_num_pulses;
	u_int32_t lpb_num_noise;
	u_int32_t lpb_tsf_delta;
	u_int64_t lpb_tsf_rel;
	u_int64_t lpb_min_possible_tsf; /* noise vs real pulses */
	u_int64_t lpb_max_possible_tsf; /* noise vs real pulses */
};

static const u_int32_t LP_MIN_BC = 8;
static const u_int32_t LP_MAX_BC = 20;
static const u_int32_t LP_NUM_BC = 13; /* (LP_MAX_BC - LP_MIN_BC + 1); */
static const u_int64_t LP_TSF_FUZZ_US = 32768; /* (1<<15) because rs_tstamp 
						* rollover errors */
static const u_int32_t LP_MIN_PRI = 1000;
static const u_int32_t LP_MAX_PRI = 2000;

static void rp_analyze_long_pulse_bscan(
	struct ath_softc *sc, 
	struct ath_rp *last_pulse,
	u_int32_t *num_bursts,
	size_t bursts_buflen,
	struct lp_burst *bursts)
{
	int i = 0;
	struct ath_rp *newer = NULL;
	struct ath_rp *cur = last_pulse;
	struct ath_rp *older = pulse_prev(last_pulse);
	u_int32_t waveform_num_bursts = 0;

	if (num_bursts)
		*num_bursts = 0;

	for (;;) {
		/* check if we are at the end of the list */
		if (&cur->list == &sc->sc_rp_head) 
			break;
		if (!cur->rp_allocated)
			break;

		if (NULL != newer) {
			u_int64_t tsf_delta = 0;
			u_int64_t tsf_adjustment = 0;

			/* Figure out TSF delta, taking into account
			 * up to one multiple of (1<<15) of clock jitter 
			 * due to interrupt latency */
			tsf_delta = newer->rp_tsf - cur->rp_tsf;
			if ((tsf_delta - LP_TSF_FUZZ_US) >= LP_MIN_PRI && 
			    (tsf_delta - LP_TSF_FUZZ_US) <= LP_MAX_PRI) {
				tsf_adjustment = LP_TSF_FUZZ_US;
				tsf_delta -= tsf_adjustment;
			}

			/* If we are in range for pulse, assume it is a pulse. */
			if ((tsf_delta >= LP_MIN_PRI) && (tsf_delta <= LP_MAX_PRI)) {
				bursts[waveform_num_bursts].lpb_num_pulses++;
				bursts[waveform_num_bursts].lpb_min_possible_tsf = 
					cur->rp_tsf - tsf_adjustment;
			}
			else if (tsf_delta < LP_MIN_PRI) {
				bursts[waveform_num_bursts].lpb_num_noise++;
				/* It may have been THE pulse after all... */
				bursts[waveform_num_bursts].lpb_min_possible_tsf = 
					cur->rp_tsf - tsf_adjustment;
			}
			else /* tsf_delta > LP_MAX_PRI */ {
				bursts[waveform_num_bursts].lpb_num_pulses++;
				bursts[waveform_num_bursts].lpb_min_possible_tsf = 
					cur->rp_tsf;
				/* Do not overrun bursts_buflen */
				if ((waveform_num_bursts+1) >= bursts_buflen) {
					break;
				}
				waveform_num_bursts++;
				bursts[waveform_num_bursts].lpb_tsf_delta = tsf_delta;
				bursts[waveform_num_bursts].lpb_min_possible_tsf = 
					cur->rp_tsf;
				bursts[waveform_num_bursts].lpb_max_possible_tsf = 
					cur->rp_tsf;
			}
		}
		else {
			bursts[waveform_num_bursts].lpb_max_possible_tsf = 
				cur->rp_tsf;
		}
		
		/* advance to next pulse */
		newer   = cur;
		cur = pulse_prev(cur);
		older   = pulse_prev(cur);
	}
	if (num_bursts) {
		bursts[waveform_num_bursts].lpb_num_pulses++;
		waveform_num_bursts++;
		*num_bursts = waveform_num_bursts;
	}
	for (i = 0; i < waveform_num_bursts; i++)
                bursts[i].lpb_tsf_rel = 
			bursts[i].lpb_max_possible_tsf - 
			bursts[waveform_num_bursts-1].lpb_min_possible_tsf;
}

static HAL_BOOL rp_analyze_long_pulse(
	struct ath_softc *sc, struct ath_rp *last_pulse,
	u_int32_t *bc, 
	u_int32_t *matched, u_int32_t *missed, 
	u_int32_t *noise, u_int32_t *pulses) 
{
	int i;
	int32_t found_radar = 0;
	int32_t found_burst_count = 0;
	int32_t matching_burst_count = 0;
	u_int32_t best_bc = 0;
	u_int32_t best_matched = 0;
	u_int32_t best_missed = 0;
	u_int32_t best_noise = 0;
	u_int32_t best_pulses = 0;

	struct lp_burst bursts[LP_MAX_BC];
	memset(&bursts, 0, sizeof(bursts));

	if (bc)
		*bc = 0;
	if (matched)
		*matched = 0;
	if (missed)
		*missed = 0;
	if (noise)
		*noise = 0;

	rp_analyze_long_pulse_bscan(sc, last_pulse, 
					     &found_burst_count, 
					     LP_MAX_BC, &bursts[0]);
	/* Find the matches */
	for (matching_burst_count = LP_MAX_BC; 
			matching_burst_count >= LP_MIN_BC; 
			matching_burst_count--) {
		int32_t first_matched_index = -1;
		int32_t last_matched_index = -1;
		int32_t match_burst_index = 0;
		int32_t found_burst_index = 0;
		int32_t burst_period = (12000000 / matching_burst_count);
		int32_t waveform_offset = 0;
		int32_t total_big_gaps = 0;
		int32_t matched_span = 0;
		int32_t missed_bursts = 0;
		int32_t matched_bursts = 0;
		for (i = 0; i < matching_burst_count; i++) {
			int32_t d = bursts[i].lpb_tsf_delta;
			while (d >= burst_period) 
				d -= burst_period;
			total_big_gaps += d;
			waveform_offset = MAX(waveform_offset, d);
		}
		waveform_offset *= -1;

		found_burst_index = 0;
		for (match_burst_index = 0; 
				match_burst_index < matching_burst_count; 
				match_burst_index++) {
			int64_t limit_high = (burst_period * 
					 (matching_burst_count - 1 - 
					  match_burst_index + 1)) + 
					(2 * LP_TSF_FUZZ_US);
			int64_t limit_low  = (burst_period * 
					 (matching_burst_count - 1 - 
					  match_burst_index)) - 
					(2 * LP_TSF_FUZZ_US);
			/* If the burst is too old, skip it... it's noise too... */
                        while ((((int64_t)bursts[found_burst_index].lpb_tsf_rel + 
							waveform_offset) > 
						limit_high)) {
                                if (found_burst_index < (found_burst_count - 1))
                                        found_burst_index++;
                                else
                                        break;
                        }
			if ((((int64_t)bursts[found_burst_index].lpb_tsf_rel + 
							waveform_offset) <= 
						limit_high) &&
			   (((int64_t)bursts[found_burst_index].lpb_tsf_rel + 
			     waveform_offset) >= limit_low)) {
				if (-1 == first_matched_index) {
					first_matched_index = match_burst_index;
					matched_span = 1;
				}
				if (last_matched_index < match_burst_index) {
					last_matched_index = match_burst_index;
				}
				DPRINTF(sc, ATH_DEBUG_DOTHFILT, 
						"LP %2dp] [%2d/%2d] %10lld " 
						"in {%lld:%lld}] PASS\n", 
						matching_burst_count, 
						found_burst_index, 
						match_burst_index, 
						(int64_t)bursts[found_burst_index].lpb_tsf_rel - 
							waveform_offset, 
						limit_low, limit_high);
				matched_bursts++;
				found_burst_index++;
			}
			else {
				DPRINTF(sc, ATH_DEBUG_DOTHFILT, 
						"LP %2dp] [%2d/%2d] %10lld " 
						"in {%lld:%lld}] MISSED\n", 
						matching_burst_count, 
						match_burst_index, 
						found_burst_index, 
						(int64_t)bursts[found_burst_index].lpb_tsf_rel - 
							waveform_offset, 
						limit_low, limit_high);
				missed_bursts++;
			}
		}
		matched_span = last_matched_index - first_matched_index;
		DPRINTF(sc, ATH_DEBUG_DOTHFILT, "LP %2dp] burst_period=%10d, "
				"waveform_offset=%10d, matches=%2d/%2d, "
				"result=%s\n",
			matching_burst_count, burst_period, waveform_offset,
			matched_span, matching_burst_count,
			(matching_burst_count == matched_span) ? 
				"MATCH" : "MISMATCH"
			);
		/* XXX - Add comparison logic rather than taking first/last
		 * match based upon ATH_DEBUG_DOTHFILTNOSC? */
		if (matched_span >= (matching_burst_count - 4)) {
			found_radar++;
			best_bc	     = matching_burst_count;
			best_matched = matched_bursts;
			best_missed  = missed_bursts;
			best_noise   = 0;
			best_pulses  = 0;
			for (i = 0; i <= found_burst_index; i++) {
				best_noise  += bursts[match_burst_index].lpb_num_noise;
				best_pulses += bursts[match_burst_index].lpb_num_pulses;
			}
			if (!DFLAG_ISSET(sc, ATH_DEBUG_DOTHFILTNOSC))
				break;
		}
	}

	if (bc)
		*bc		= best_bc;
	if (matched)
		*matched 	= best_matched;
	if (missed)
		*missed 	= best_missed;
	if (noise)
		*noise 		= best_noise;
	if (pulses)
		*pulses		= best_pulses;

	return found_radar ? AH_TRUE : AH_FALSE;
}
#endif /* #ifdef ATH_RADAR_LONG_PULSE */

static HAL_BOOL rp_analyze_short_pulse(
	struct ath_softc *sc, struct ath_rp *last_pulse, 
	u_int32_t *index, u_int32_t *pri, u_int32_t *matching_pulses, 
	u_int32_t *missed_pulses, u_int32_t *noise_pulses)
{
	int i;
	int best_index = -1;
	unsigned int best_matched = 0;
	unsigned int best_noise = 0;
	unsigned int best_missed = 0;
	unsigned int best_pri = 0;
	unsigned int best_cr = 0;
	u_int64_t t0;

	struct ath_rp *pulse;
	u_int32_t pulse_count_minimum = 0;
	struct radar_pattern_specification *pattern = NULL;
	struct radar_pattern_specification *best_pattern = NULL;

	if (index)
		*index = 0;
	if (pri)
		*pri = 0;
	if (noise_pulses)
		*noise_pulses = 0;
	if (matching_pulses)
		*matching_pulses = 0;
	if (missed_pulses)
		*missed_pulses = 0;

	/* we need at least sc_rp_min (>2 pulses) */
	pulse_count_minimum = sc->sc_rp_min;
	if ((sc->sc_rp_num < pulse_count_minimum) || 
	    (sc->sc_rp_num < 2))
		return 0;

	/* Search algorithm:
	 *
	 * - since we have a limited and known number of radar patterns, we
	 *   loop on all possible radar pulse period
	 *
	 * - we start the search from the last timestamp (t0), going backward
	 *   in time, up to the point for the first possible radar pulse, ie
	 *   t0 - PERIOD_MAX * BURST_MAX
	 *
	 * - on this timescale, we matched the number of hit/missed using t0 -
	 *   PERIOD * n taking into account the 2% error margin (using
	 *   min_rep_int, max_rep_int)
	 *
	 * At the end, we have a number of pulse hit for each PRF
	 *
	 * TSF will roll over after just over 584,542 years of operation
	 * without restart.
	 * 
	 * This exceeds all known Atheros MTBF so, forget about TSF roll over.
	 */

	/* loop through all patterns */
	for (i = 0; i < sizetab(radar_patterns); i++) {
		int matched = 1, missed = 0, partial_miss = 0, noise = 0;
		int64_t t, last_t, t_min, t_avg, t_max, tn_max;
		int A, B, N, C, D, SUM_SQR;
		int a, a_min, a_avg, a_max;
		int b, b_min, b_avg, b_max;
		int count;
		int last_matched = 0;

		pattern = &radar_patterns[i];

		/* t0 is the timestamp of the beginning of the last radar
		 * pulse. We assume that the hardware reports the end of the
		 * pulse, so we compute the beginning based on the reported
		 * pulse width, which might larger than the real pulse.
		 *
		 * BIG WARNING : using FCC samples, it seems that this
		 * correction is not needed at all, so we are using the
		 * dyn_ints flag to avoid this correction */
		t0 = last_pulse->rp_tsf;
		if (!pattern->is_fcc)
			t0 -= WIDTH_TO_TSF(last_pulse->rp_width);

		/* initial values for a_min, a_avg, a_max and b_min, b_avg,
		 * b_max */
		a_min = -10;
		a_avg =   0;
		a_max = +10;

		b_min = pattern->min_rep_int;
		b_avg = (pattern->min_rep_int + pattern->max_rep_int)/2;
		b_max = pattern->max_rep_int;

		/* initial values for t_min, t_avg, t_max */
		t_min = a_min + b_min;
		t_avg = a_avg + b_avg;
		t_max = a_max + b_max;

		/* this max formula is to stop when we exceed maximum time
		 * period for the pattern.  It's the oldest possible value for
		 * t that could match. */
		tn_max = a_max + b_max*(pattern->max_evts-1);

		last_t = -1;

		A = 0;
		B = 0;
		N = 1;
		C = 0;
		D = 0;
		SUM_SQR = 0;

		/* we directly start with the timestamp before t0 */
		pulse = pulse_prev(last_pulse);

		for (;;) {

			/* check if we are at the end of the list */
			if (&pulse->list == &sc->sc_rp_list) 
				break;
			if (!pulse->rp_allocated)
				break;

			t = t0 - pulse->rp_tsf;
			if (!pattern->is_fcc)
				t += WIDTH_TO_TSF(pulse->rp_width);

			/* Do not go too far... this is an optimization to not
			 * keep checking after we hit maximum time span for
			 * the pattern. */
			if ((t < 0) || (t > tn_max)) {
				DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
					"%-17s [%2d] tsf: %10llu width: %3u "
					"t:%5lld [range: %5lld-%5lld] [%2d] "
					"%s\n",
					pattern->name,
					pulse->rp_index,
					pulse->rp_tsf,
					pulse->rp_width,
					t, t_min, t_max,
					matched+missed+partial_miss,
					"overflow");
				break;
			}

			/* we want to find the previous pulse in the range
			 * [t_min, t_max] which is closest to t_avg */

			if (t < t_min) {
				/* t < t_min : this pulse is not matching the
				 * current radar pattern, it is just
				 * noise. Ignore it by going to the next
				 * pulse */

				DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE, 
					"%-17s [%2d] tsf: %10llu width: %3u "
					"t:%5lld [range: %5lld-%5lld] [%2d] "
					"%s\n",
					pattern->name,
					pulse->rp_index,
					pulse->rp_tsf,
					pulse->rp_width,
					t, t_min, t_max,
					matched+missed+partial_miss,
					"noise");

				/* this event is noise, ignore it */
				pulse = pulse_prev(pulse);
				noise++;
			} else if (t <= t_max) {
				/* t_min <= t <= t_max : this pulse is
				 * matching the current radar pattern */

				if (last_t == -1) {
					/* this pulse is the first matching pulse */

					DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
						"%-17s [%2d] tsf: %10llu width: %3u "
						"t:%5lld [range: %5lld-%5lld] [%2d] "
						"%s\n",
						pattern->name,
						pulse->rp_index,
						pulse->rp_tsf,
						pulse->rp_width,
						t, t_min, t_max,
						matched+missed+partial_miss,
						"first match");


					matched ++;
					missed += partial_miss;
					partial_miss = 0;
					last_t = t;

					count = matched+missed+partial_miss-1;
					A += t;
					B += t*count;
					N += 1;
					C += count;
					D += count*count;
					SUM_SQR += SQR( t - (a_avg+b_avg*count));
				} else {
					/* this pulse is not the first
					 * matching pulse. Check if it is
					 * closest to t_avg than the existing
					 * pulse */
					if (labs(t-t_avg) < labs(last_t-t_avg)) {

						/* this pulse is closer to
						 * t_avg. Update computation
						 * and go to the next pulse */

						DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
							"%-17s [%2d] tsf: %10llu width: %3u "
							"t:%5lld [range: %5lld-%5lld] [%2d] "
							"%s\n",
							pattern->name,
							pulse->rp_index,
							pulse->rp_tsf,
							pulse->rp_width,
							t, t_min, t_max,
							matched+missed+partial_miss,
							"better match");

						count = matched+missed+partial_miss-1;
						A -= last_t;
						B -= last_t*count;
						SUM_SQR -= SQR( last_t - (a_avg+b_avg*count));

						A += t;
						B += t*count;
						SUM_SQR += SQR( last_t - (a_avg+b_avg*count));

						noise ++;
						last_t = t;
					} else {
						/* this pulse is further to t_avg. */

						DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
							"%-17s [%2d] tsf: %10llu width: %3u "
							"t:%5lld [range: %5lld-%5lld] [%2d] "
							"%s\n",
							pattern->name,
							pulse->rp_index,
							pulse->rp_tsf,
							pulse->rp_width,
							t, t_min, t_max,
							matched+missed+partial_miss,
							"worst match");

						noise ++;
					}
				}


				/* go to the previous pulse */
				pulse = pulse_prev(pulse);
			} else {
				/* t > t_max : this pulse is not matching the
				 * current radar pattern yet. Update range */

				DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
					"%-17s [%2d] tsf: %10llu width: %3u "
					"t:%5lld [range: %5lld-%5lld] [%2d] "
					"%s\n",
					pattern->name,
					pulse->rp_index,
					pulse->rp_tsf,
					pulse->rp_width,
					t, t_min, t_max,
					matched + missed + partial_miss,
					"range update");


				/* we do not found a pulse in the current
				 * range, we increase the number of miss */
				if (last_t == -1) {
					partial_miss++;

					/* if we missed more than the
					 * specified partial number of pulses,
					 * we stop searching */
					if (partial_miss > pattern->max_consecutive_missing) {
						DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
							"%-17s matching stopped (too many "
							"consecutive pulses missing). %d>%d "
							"matched=%d. missed=%d.\n",
							pattern->name,
							partial_miss,
							pattern->max_consecutive_missing,
							matched, missed);
						break;
					}

					/* if we missed more than the
					 * specified total number of pulses,
					 * we stop searching */
					if (missed + partial_miss > pattern->max_missing) {
						DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
							"%-17s matching stopped (too "
							"many total pulses missing). "
							"%d>%d  matched=%d. missed=%d."
							"\n",
							pattern->name,
							missed,
							pattern->max_missing,
							matched,
							missed);
						break;
					}
				}

				/* Quick update to b */
				if ((1 < matched) && (matched < 4) &&
				    (matched!=last_matched)) {
					int c = a_avg, d = b_avg;

					if (C*C-N*D != 0) {
						a = a_avg;
						b = (C*A-N*B)/(C*C-N*D);

						b_avg = b;

						DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
							"%-17s updating b_avg to %d\n",
							pattern->name, b_avg);

						SUM_SQR += (SQR(a)-SQR(c))*N+2*(a*b-c*d)*C+(SQR(b)-SQR(d))*D
							-2*(a-c)*A-2*(b-d)*B;

						last_matched = matched;
					}
				}

				/* Update (a,b) */
				if ((matched >= 4) &&
				    (matched != last_matched)) {
					int c = a_avg, d = b_avg;

					if (C*C-N*D != 0) {
						a=(C*B-A*D)/(C*C-N*D);
						b=(C*A-N*B)/(C*C-N*D);

						a_min = a-10;
						a_avg = a;
						a_max = a+10;

						b_min = b-1;
						b_avg = b;
						b_max = b+1;

						DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
							"%-17s updating a to (%d,%d,%d) and b to (%d,%d,%d)\n",
							pattern->name,
							a_min, a_avg, a_max,
							b_min, b_avg, b_max);

						SUM_SQR += (SQR(a)-SQR(c))*N+2*(a*b-c*d)*C+(SQR(b)-SQR(d))*D
							    -2*(a-c)*A-2*(b-d)*B;

						last_matched = matched;
					}
				}

				/* Update range */
				count = matched+missed+partial_miss;
				t_min = a_min + b_min*count;
				t_avg = a_avg + b_avg*count;
				t_max = a_max + b_max*count;

				/* Check if we intersect with previous/next range */
				/* t_min < t_max(count-1) */
				if ((count>=1) && (t_min < a_max + b_max*(count-1))) {

					t_min = a_avg + b_avg*count - b_avg/2;

					DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
						"%-17s [%2d] tsf: %10llu width: %3u "
						"t:%5lld [range: %5lld-%5lld] [%2d] "
						"%s\n",
						pattern->name,
						pulse->rp_index,
						pulse->rp_tsf,
						pulse->rp_width,
						t, t_min, t_max,
						matched + missed + partial_miss,
						"updated t_min");
				}
				/* t_max > t_min(count+1) */
				if (t_max > a_min + b_min*(count+1)) {

					t_max = a_avg + b_avg*count + b_avg/2;

					DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
						"%-17s [%2d] tsf: %10llu width: %3u "
						"t:%5lld [range: %5lld-%5lld] [%2d] "
						"%s\n",
						pattern->name,
						pulse->rp_index,
						pulse->rp_tsf,
						pulse->rp_width,
						t, t_min, t_max,
						matched + missed + partial_miss,
						"updated t_max");
				}
				last_t = -1;
			}
		}

		/* print counters for this PRF (we avoid case where the
		 * variance is greater than 100). 100 is based on the fact
		 * that most pulses are less than 10us from the ideal value
		 * and as such, variance should be less than 100 */
		if ((matched > 1) &&
		    (N != 0) && (SUM_SQR/N<100)) {
			int compare_result = CR_FALLTHROUGH;
			int match_result = MR_MATCH;

			DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
				"%-17s matched %d missed %d mean_period %d noise %d variance %d\n",
				pattern->name,
				matched,
				missed,
				b_avg,
				noise,
				SUM_SQR/N);

			/* check if PRF counters match a known radar, if we are
			 * confident enought */
			if (MR_MATCH == (match_result = match_radar(
					matched,
					missed,
					b_avg,
					noise,
					pattern->min_evts,
					pattern->max_evts,
					pattern->min_rep_int,
					pattern->max_rep_int,
					pattern->min_pulse,
					pattern->max_missing))) {
				compare_result = (NULL == best_pattern) ? CR_NULL : 
					compare_radar_matches(
						matched,
						missed, 
						b_avg,
						noise,
						pattern->min_evts,
						pattern->max_evts,
						pattern->min_rep_int,
						pattern->max_rep_int,
						pattern->min_pulse,
						pattern->max_missing,
						pattern->match_midpoint,
						best_matched, 
						best_missed, 
						best_pri, 
						best_noise,
						best_pattern->min_evts,
						best_pattern->max_evts,
						best_pattern->min_rep_int,
						best_pattern->max_rep_int,
						best_pattern->min_pulse,
						best_pattern->max_missing,
						best_pattern->match_midpoint);
			}
			if (DFLAG_ISSET(sc, ATH_DEBUG_DOTHFILT)) {
				DPRINTF(sc, ATH_DEBUG_DOTHFILT,
					"[%02d] %13s: %-17s [match=%2u {%2u"
					"..%2u},missed=%2u/%2u,dur=%2d {%2u.."
					"%2u},noise=%2u/%2u,cr:%d]\n",
					last_pulse->rp_index,
					compare_result > CR_FALLTHROUGH ? 
						"NEW-BEST" : 
						get_match_result_desc(match_result),
					pattern->name,
					matched, 
					pattern->min_pulse, 
					pattern->max_evts,
					missed, 
					pattern->max_missing,
					matched + missed, 
					pattern->min_evts, 
					pattern->max_evts, 
					noise, 
					matched + noise,
					compare_result);
			}

			if (compare_result > CR_FALLTHROUGH) {
				best_matched = matched;
				best_missed = missed;
				best_index = i;
				best_pattern = pattern;
				best_pri = b_avg;
				best_noise = noise;
				best_cr = compare_result;
			}
			else if (compare_result <= CR_FALLTHROUGH) {
				DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
					"%-17s match not better than best so "
					"far.  cr: %d matched: %d  missed: "
					"%d min_evts: %d\n",
					pattern->name, 
					compare_result,
					matched, 
					missed, 
					pattern->min_evts);
			}
		}
	}
	if (-1 != best_index) {
		DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
			"[%02d] %10s: %-17s [match=%2u {%2u..%2u},missed="
			"%2u/%2u,dur=%2d {%2u..%2u},noise=%2u/%2u,cr=%2d] "
			"RI=%-9u RF=%-4u\n",
			last_pulse->rp_index,
			"BEST/PULSE",
			best_pattern->name,
			best_matched, 
			best_pattern->min_pulse, 
			best_pattern->max_evts,
			best_missed, 
			best_pattern->max_missing,
			(best_matched + best_missed),
			best_pattern->min_evts,
			best_pattern->max_evts, 
			best_noise, 
			(best_matched + best_noise), 
			best_cr, 
			best_pri, 
			interval_to_frequency(best_pri));
		if (index)
			*index = best_index;
		if (pri)
			*pri = best_pri;
		if (matching_pulses)
			*matching_pulses = best_matched;
		if (noise_pulses)
			*noise_pulses = best_noise;
		if (missed_pulses)
			*missed_pulses = best_missed;
	}

	return (-1 != best_index) ? AH_TRUE : AH_FALSE;
}

#ifdef AR_DEBUG
static u_int32_t interval_to_frequency(u_int32_t interval)
{
	/* Calculate BRI from PRI */
	u_int32_t frequency = interval ? (1000000 / interval) : 0;
	/* Round to nearest multiple of 50 */
	return frequency + ((frequency % 50) >= 25 ? 50 : 0) - (frequency % 50);
}
#endif /* AR_DEBUG */

#ifdef ATH_RADAR_LONG_PULSE
static const char *get_longpulse_desc(int lp)
{
	switch (lp) {
	case  8:  return "FCC [5,  8 pulses]";
	case  9:  return "FCC [5,  9 pulses]";
	case 10:  return "FCC [5, 10 pulses]";
	case 11:  return "FCC [5, 11 pulses]";
	case 12:  return "FCC [5, 12 pulses]";
	case 13:  return "FCC [5, 13 pulses]";
	case 14:  return "FCC [5, 14 pulses]";
	case 15:  return "FCC [5, 15 pulses]";
	case 16:  return "FCC [5, 16 pulses]";
	case 17:  return "FCC [5, 17 pulses]";
	case 18:  return "FCC [5, 18 pulses]";
	case 19:  return "FCC [5, 19 pulses]";
	case 20:  return "FCC [5, 20 pulses]";
	default:  return "FCC [5, invalid pulses]";
	}
}
#endif /* #ifdef ATH_RADAR_LONG_PULSE */

static HAL_BOOL rp_analyze(struct ath_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	HAL_BOOL radar = 0;
	struct ath_rp *pulse;

	/* Best short pulse match */
	int32_t best_index = -1;
	u_int32_t best_pri = 0;
	u_int32_t best_matched = 0;
	u_int32_t best_missed = 0;
	u_int32_t best_noise = 0;
	int32_t best_cr = 0;

#ifdef ATH_RADAR_LONG_PULSE
	/* Best long pulse match */
	u_int32_t best_lp_bc	  = 0;
	u_int32_t best_lp_matched = 0;
	u_int32_t best_lp_missed  = 0;
	u_int32_t best_lp_noise   = 0;
	u_int32_t best_lp_pulses  = 0;
#endif /* #ifdef ATH_RADAR_LONG_PULSE */
	u_int32_t pass = 0;
	struct radar_pattern_specification *best_pattern = NULL;

	/* start the analysis by the last pulse since it might speed up
	 * things and then move backward for all non-analyzed pulses.
	 * For debugging ONLY - we continue to run this scan after radar is 
	 * detected, processing all pulses... even when they come in after an 
	 * iteration of all pulses that were present when this function was 
	 * invoked.  This can happen at some radar waveforms where we will 
	 * match the first few pulses and then the rest of the burst will come 
	 * in, but never be analyzed.
	 */

	while (pulse_tail(sc)->rp_allocated && 
	       !pulse_tail(sc)->rp_analyzed && 
	       (AH_FALSE == radar || 
		(DFLAG_ISSET(sc, ATH_DEBUG_DOTHFILT) && ++pass <= 3))) {

		list_for_each_entry_reverse(pulse, &sc->sc_rp_list, list) {
			if (!pulse->rp_allocated) 
				break;
			if (pulse->rp_analyzed)
				break;

			/* Skip pulse analysis after we have confirmed radar 
			 * presence unless we are debugging and have 
			 * disabled short-circuit logic.  In this case,
			 * we'll go through ALL the signatures and find 
			 * the best match to convince ourselves this code works.
			 */
			if (AH_FALSE == radar || 
			    DFLAG_ISSET(sc, ATH_DEBUG_DOTHFILTNOSC)) {
				/* short pulse match status */
				u_int32_t index	 	= 0;
				u_int32_t pri 		= 0;
				u_int32_t matched 	= 0;
				u_int32_t missed 	= 0;
				u_int32_t noise 	= 0;
#ifdef ATH_RADAR_LONG_PULSE

				/* long pulse match status */
				u_int32_t lp_bc	  	= 0;
				u_int32_t lp_matched 	= 0;
				u_int32_t lp_missed  	= 0;
				u_int32_t lp_noise   	= 0;
				u_int32_t lp_pulses  	= 0;
#endif /* #ifdef ATH_RADAR_LONG_PULSE */
				if (rp_analyze_short_pulse(sc, pulse, &index, 
							&pri, &matched, &missed, 
							&noise)) {
					int compare_result = (!radar || best_index == -1) ? 
						CR_NULL : 
						compare_radar_matches(
							matched, 
							missed, 
							pri,
							noise,
							radar_patterns[index].min_evts,
							radar_patterns[index].max_evts,
							radar_patterns[index].min_rep_int,
							radar_patterns[index].max_rep_int,
							radar_patterns[index].min_pulse,
							radar_patterns[index].max_missing,
							radar_patterns[index].match_midpoint,
							best_matched,
							best_missed,
							best_pri,
							best_noise,
							radar_patterns[best_index].min_evts,
							radar_patterns[best_index].max_evts,
							radar_patterns[best_index].min_rep_int,
							radar_patterns[best_index].max_rep_int,
							radar_patterns[best_index].min_pulse,
							radar_patterns[best_index].max_missing,
							radar_patterns[best_index].match_midpoint
						);
					if (compare_result > CR_FALLTHROUGH) {
						/* Update best match */
						best_matched 	= matched;
						best_missed 	= missed;
						best_index 	= index;
						best_pri 	= pri;
						best_noise 	= noise;
						radar 		= AH_TRUE;
						best_cr 	= compare_result;
					}
					DPRINTF(sc, ATH_DEBUG_DOTHFILT,
						"%10s: %-17s [match=%2u "
						"{%2u..%2u}, missed=%2u/%2u, "
						"dur=%2d {%2u..%2u}, "
						"noise=%2u/%2u, cr=%2d] "
						"RI=%-9u RF=%-4u\n",
						(compare_result > CR_FALLTHROUGH) ? 
							"BETTER" : "WORSE",
						radar_patterns[index].name,
						matched, 
						radar_patterns[index].min_pulse, 
						radar_patterns[index].max_evts,
						missed, 
						radar_patterns[index].max_missing,
						(matched + missed),
						radar_patterns[index].min_evts,
						radar_patterns[index].max_evts,
						noise, 
						(matched + noise), 
						compare_result, 
						pri, 
						interval_to_frequency(pri));
				}
#ifdef ATH_RADAR_LONG_PULSE
				if (rp_analyze_long_pulse(sc, pulse, 
								  &lp_bc, 
								  &lp_matched, 
								  &lp_missed, 
								  &lp_noise,
								  &lp_pulses)) {
					/* XXX: Do we care about best match?? */
					radar 		= AH_TRUE;
					best_lp_bc 	= lp_bc;
					best_lp_matched = lp_matched;
					best_lp_missed 	= lp_missed;
					best_lp_noise 	= lp_noise;
					best_lp_pulses 	= lp_pulses;
				}
#endif /* #ifdef ATH_RADAR_LONG_PULSE */
			}
			pulse->rp_analyzed = 1;
		}
	}
	if (AH_TRUE == radar) {
#ifdef ATH_RADAR_LONG_PULSE
		if (!best_lp_bc) {
#endif /* #ifdef ATH_RADAR_LONG_PULSE */
			best_pattern = 
				&radar_patterns[best_index];
			DPRINTF(sc, ATH_DEBUG_DOTHFILT,
				"%10s: %-17s [match=%2u {%2u..%2u},missed="
				"%2u/%2u,dur=%2d {%2u..%2u},noise=%2u/%2u,cr=%2d] "
				"RI=%-9u RF=%-4u\n",
				"BEST MATCH",
				best_pattern->name,
				best_matched, 
				best_pattern->min_pulse, 
				best_pattern->max_evts,
				best_missed, 
				best_pattern->max_missing,
				(best_matched + best_missed),
				best_pattern->min_evts,
				best_pattern->max_evts, 
				best_noise, 
				(best_matched + best_noise), 
				best_cr, 
				best_pri, 
				interval_to_frequency(best_pri));
#ifdef ATH_RADAR_LONG_PULSE
		}
		else {
			DPRINTF(sc, ATH_DEBUG_DOTHFILT,
				"%10s: %-17s [match=%2u {%2u..%2u},missed="
				"%2u/%2u,noise=%2u/%2u]\n",
				"BEST MATCH",
				get_longpulse_desc(best_lp_bc),
				best_lp_bc, 
				(best_lp_bc-4),
				best_lp_bc,
				best_lp_missed,
				(best_lp_bc-(best_lp_bc-4)),
				best_lp_noise, 
				(best_lp_pulses + best_lp_noise)
				);
		}
#endif /* #ifdef ATH_RADAR_LONG_PULSE */
		if (DFLAG_ISSET(sc, ATH_DEBUG_DOTHPULSES)) {
			DPRINTF(sc, ATH_DEBUG_DOTHPULSES,
				"========================================\n");
			DPRINTF(sc, ATH_DEBUG_DOTHPULSES,
				"==BEGIN RADAR SAMPLE====================\n");
			DPRINTF(sc, ATH_DEBUG_DOTHPULSES,
				"========================================\n");

#ifdef ATH_RADAR_LONG_PULSE
			if (!best_lp_bc) {
#endif /* #ifdef ATH_RADAR_LONG_PULSE */
				best_pattern = 
					&radar_patterns[best_index];
				DPRINTF(sc, ATH_DEBUG_DOTHPULSES,
					"Sample contains data matching %-17s "
					"[match=%2u {%2u..%2u}, "
					"missed=%2u/%2u, dur=%2d {%2u..%2u}, "
					"noise=%2u/%2u,cr=%d] RI=%-9u RF=%-4u\n",
					best_pattern->name,
					best_matched, 
					best_pattern->min_pulse, 
					best_pattern->max_evts,
					best_missed, 
					best_pattern->max_missing,
					best_matched + best_missed,
					best_pattern->min_evts,
					best_pattern->max_evts, 
					best_noise, 
					best_noise + best_matched, 
					best_cr,
					best_pri, 
					interval_to_frequency(best_pri));
#ifdef ATH_RADAR_LONG_PULSE
			} else {
				DPRINTF(sc, ATH_DEBUG_DOTHPULSES,
					"Sample contains data matching %s\n",
					get_longpulse_desc(best_lp_bc));
			}
#endif /* #ifdef ATH_RADAR_LONG_PULSE */

			ath_rp_print(sc, 0 /* analyzed pulses only */ );
			DPRINTF(sc, ATH_DEBUG_DOTHPULSES,
				"========================================\n");
			DPRINTF(sc, ATH_DEBUG_DOTHPULSES,
				"==END RADAR SAMPLE======================\n");
			DPRINTF(sc, ATH_DEBUG_DOTHPULSES,
				"========================================\n");
		}
#ifdef ATH_RADAR_LONG_PULSE
		if (!best_lp_bc)
#endif /* #ifdef ATH_RADAR_LONG_PULSE */
			ic->ic_radar_detected(ic,
					      radar_patterns[best_index].name,
					      0, 0);
#ifdef ATH_RADAR_LONG_PULSE
		else 
			ic->ic_radar_detected(ic,
					      get_longpulse_desc(best_lp_bc),
					      0, 0);
#endif /* #ifdef ATH_RADAR_LONG_PULSE */
	}
	return radar;
}

/* initialize ath_softc members so sensible values */
static void ath_rp_clear(struct ath_softc *sc)
{
	sc->sc_rp = NULL;
	INIT_LIST_HEAD(&sc->sc_rp_list);
	sc->sc_rp_num = 0;
	sc->sc_rp_analyze = NULL;
}

static void ath_rp_tasklet(TQUEUE_ARG data)
{
	struct net_device *dev = (struct net_device *)data;
	struct ath_softc *sc = netdev_priv(dev);

	if (sc->sc_rp_analyze != NULL)
		sc->sc_rp_analyze(sc);
}

void ath_rp_init(struct ath_softc *sc)
{
	struct net_device *dev = sc->sc_dev;
	int i;

	ath_rp_clear(sc);

	sc->sc_rp = (struct ath_rp *)kzalloc(
			sizeof(struct ath_rp) *
			ATH_RADAR_PULSE_NR, GFP_KERNEL);

	if (sc->sc_rp == NULL)
		return;

	/* initialize the circular list */
	INIT_LIST_HEAD(&sc->sc_rp_list);
	for (i = 0; i < ATH_RADAR_PULSE_NR; i++) {
		sc->sc_rp[i].rp_index = i;
		list_add_tail(&sc->sc_rp[i].list, 
			      &sc->sc_rp_list);
	}

	sc->sc_rp_num = 0;
	sc->sc_rp_analyze = rp_analyze;

	/* compute sc_rp_min */
	sc->sc_rp_min = 2;
	for (i = 0; i < sizetab(radar_patterns); i++)
		sc->sc_rp_min = 
			MIN(sc->sc_rp_min,
			    radar_patterns[i].min_pulse);

	/* default values is properly handle pulses and detected radars */
	sc->sc_rp_ignored = 0;
	sc->sc_radar_ignored = 0;

	ATH_INIT_TQUEUE(&sc->sc_rp_tq, ath_rp_tasklet, dev);
}

void ath_rp_done(struct ath_softc *sc)
{
	/* free what we allocated in ath_rp_init() */
	kfree(sc->sc_rp);

	ath_rp_clear(sc);
}

void ath_rp_record(struct ath_softc *sc, u_int64_t tsf, u_int8_t rssi, 
			    u_int8_t width, HAL_BOOL is_simulated)
{
	struct ath_rp *pulse;

	DPRINTF(sc, ATH_DEBUG_DOTHPULSES,
		"tsf=%10llu rssi=%3u width=%3u%s\n", 
		tsf, rssi, width,
		sc->sc_rp_ignored ? " (ignored)" : "");

	if (sc->sc_rp_ignored) {
		return;
	}

#if 0
	/* pulses width 255 seems to trigger false detection of radar. we
	 * ignored it then. */

	if (width == 255) {
		/* ignored */
		return ;
	}
#endif

	/* check if the new radar pulse is after the last one recorded, or
	 * else, we flush the history */
	pulse = pulse_tail(sc);
	if (tsf < pulse->rp_tsf) {
		if (is_simulated == AH_TRUE && 0 == tsf) {
			DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE, 
				"%s: %s: ath_rp_flush: simulated tsf "
				"reset.  tsf =%10llu, rptsf =%10llu\n", 
				SC_DEV_NAME(sc), __func__,
				(unsigned long long)tsf,
				(unsigned long long)pulse->rp_tsf);
			ath_rp_flush(sc);
		} else if ((pulse->rp_tsf - tsf) > (1 << 15)) {
			DPRINTF(sc, ATH_DEBUG_DOTHFILTVBSE,
				"%s: %s: ath_rp_flush: tsf reset.  "
				"(rp_tsf - tsf > 0x8000) tsf=%10llu, rptsf="
				"%10llu\n", 
				SC_DEV_NAME(sc), __func__,
				(unsigned long long)tsf,
				(unsigned long long)pulse->rp_tsf);
			ath_rp_flush(sc);
		} else {
			DPRINTF(sc, ATH_DEBUG_DOTHFILT,
				"%s: %s: tsf jitter/bug detected: tsf =%10llu, "
				"rptsf =%10llu, rp_tsf - tsf = %10llu\n", 
				SC_DEV_NAME(sc), __func__,
				(unsigned long long)tsf,
				(unsigned long long)pulse->rp_tsf,
				(unsigned long long)(pulse->rp_tsf - tsf));
		}
	}

	/* remove the head of the list */
	pulse = pulse_head(sc);
	list_del(&pulse->list);

	pulse->rp_tsf = tsf;
	pulse->rp_rssi = rssi;
	pulse->rp_width = width;
	pulse->rp_allocated = 1;
	pulse->rp_analyzed = 0;

	/* add at the tail of the list */
	list_add_tail(&pulse->list, &sc->sc_rp_list);
	if (ATH_RADAR_PULSE_NR > sc->sc_rp_num)
		sc->sc_rp_num++;
}

void ath_rp_print_mem(struct ath_softc *sc, int analyzed_pulses_only)
{
	struct ath_rp *pulse;
	u_int64_t oldest_tsf = ~0;
	int i;
	IPRINTF(sc, "Pulse dump of %spulses using sc_rp containing "
	       "%d allocated pulses.\n", 
	       analyzed_pulses_only ? "analyzed " : "", sc->sc_rp_num);

	/* Find oldest TSF value so we can print relative times */
	for (i = 0; i < ATH_RADAR_PULSE_NR; i++) {
		pulse = &sc->sc_rp[i];
		if (pulse->rp_allocated && pulse->rp_tsf < oldest_tsf)
			oldest_tsf = pulse->rp_tsf;
	}

	for (i = 0; i < ATH_RADAR_PULSE_NR; i++) {
		pulse = &sc->sc_rp[i];
		if (!pulse->rp_allocated)
			break;
		if ((!analyzed_pulses_only) || pulse->rp_analyzed)
			IPRINTF(sc, "Pulse [%3d, %p] : relative_tsf=%10llu "
			       "tsf=%10llu rssi=%3u width=%3u allocated=%d "
			       "analyzed=%d next=%p prev=%p\n",
			       pulse->rp_index,
			       pulse, 
			       (unsigned long long)(pulse->rp_tsf - oldest_tsf),
			       (unsigned long long)pulse->rp_tsf,
			       pulse->rp_rssi, 
			       pulse->rp_width, 
			       pulse->rp_allocated, 
			       pulse->rp_analyzed, 
			       pulse->list.next, 
			       pulse->list.prev);
	}
}

void ath_rp_print(struct ath_softc *sc, int analyzed_pulses_only)
{
	struct ath_rp *pulse;

	DPRINTF(sc, ATH_DEBUG_DOTHPULSES,
		"Pulse dump of %spulses from ring buffer containing %d "
		"pulses.\n",
		analyzed_pulses_only ? "analyzed " : "",
		sc->sc_rp_num);

	list_for_each_entry_reverse(pulse, &sc->sc_rp_list, list) {
		if (!pulse->rp_allocated)
			continue;
		if ((!analyzed_pulses_only) || pulse->rp_analyzed) {
			DPRINTF(sc, ATH_DEBUG_DOTHPULSES,
				"tsf=%10llu rssi=%3u width=%3u\n",
				pulse->rp_tsf, pulse->rp_rssi, pulse->rp_width);
		}
	}
}

void ath_rp_flush(struct ath_softc *sc)
{
	struct ath_rp *pulse;
	list_for_each_entry_reverse(pulse, &sc->sc_rp_list, list)
		pulse->rp_allocated = 0;
	sc->sc_rp_num = 0;
}
