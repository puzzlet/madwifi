/*
 * Copyright (c) 2008 Bruno Randolf <br1@einfach.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
	without modification.
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
 * $Id: br1 $
 */

#include "ath_hal/ah_devid.h"
#include "if_media.h"
#include <net80211/ieee80211_var.h>
#include "if_athvar.h"
#include "if_ath_hal.h"
#include "if_ath_hal_macros.h"
#include "if_ath_hal_wrappers.h"
#include "if_ath_hal_extensions.h"


/* Known SREVs */
static struct ath5k_srev_name srev_names[] = {
	{ "5210",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5210 },
	{ "5311",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5311 },
	{ "5311A",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5311A },
	{ "5311B",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5311B },
	{ "5211",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5211 },
	{ "5212",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5212 },
	{ "5213",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5213 },
	{ "5213A",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5213A },
	{ "2413",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR2413 },
	{ "2414",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR2414 },
	{ "2424",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR2424 },
	{ "5424",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5424 },
	{ "5413",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5413 },
	{ "5414",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5414 },
	{ "5416",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5416 },
	{ "5418",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR5418 },
	{ "2425",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR2425 },
	{ "9160",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR9160 },
	{ "9280",	AR5K_VERSION_VER,	AR5K_SREV_VER_AR9280 },
	{ "xxxxx",	AR5K_VERSION_VER,	AR5K_SREV_UNKNOWN },
	{ "5110",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5110 },
	{ "5111",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5111 },
	{ "2111",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2111 },
	{ "5112",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5112 },
	{ "5112A",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5112A },
	{ "2112",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2112 },
	{ "2112A",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_2112A },
	{ "SChip",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_SC0 },
	{ "SChip",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_SC1 },
	{ "SChip",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_SC2 },
	{ "5133",	AR5K_VERSION_RAD,	AR5K_SREV_RAD_5133 },
	{ "xxxxx",	AR5K_VERSION_RAD,	AR5K_SREV_UNKNOWN },
};

#define	AR5210_MAGIC	0x19980124
#define	AR5211_MAGIC	0x19570405
#define	AR5212_MAGIC	0x19541014
#define	AR5416_MAGIC	0x20065416

int
ar_device(struct ath_softc *sc)
{
	int magic = sc->sc_ah->ah_magic;

	switch (magic) {
	case AR5210_MAGIC:
		return 5210;
	case AR5211_MAGIC:
		return 5211;
	case AR5212_MAGIC:
		return 5212;
	case AR5416_MAGIC:
		return 5416;
	default:
		printk(KERN_WARNING "unknown HAL magic 0x%08x\n", magic);
		return 0;
	}
}

int
ath_set_ack_bitrate(struct ath_softc *sc, int high)
{
	if (ar_device(sc) == 5212) {
		/* set ack to be sent at low bit-rate */
		u_int32_t v = AR5K_STA_ID1_BASE_RATE_11B | AR5K_STA_ID1_ACKCTS_6MB;
		if (high)
			ath_reg_write(sc, AR5K_STA_ID1,
					ath_reg_read(sc, AR5K_STA_ID1) & ~v);
		else
			ath_reg_write(sc, AR5K_STA_ID1,
					ath_reg_read(sc, AR5K_STA_ID1) | v);
		return 0;
	}
	return 1;
}


const char *
ath5k_chip_name(enum ath5k_srev_type type, u_int16_t val)
{
	const char *name = "xxxxx";
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(srev_names); i++) {
		if (srev_names[i].sr_type != type)
			continue;
		if (val < srev_names[i + 1].sr_val) {
			name = srev_names[i].sr_name;
			break;
		}
	}

	return name;
}


void
ath_hw_beacon_stop(struct ath_softc *sc) {
	HAL_BEACON_TIMERS btimers;

	btimers.bt_intval = 0;
	btimers.bt_nexttbtt = 0;
	btimers.bt_nextdba = 0xffffffff;
	btimers.bt_nextswba = 0xffffffff;
	btimers.bt_nextatim = 0;

	ath_hal_setbeacontimers(sc->sc_ah, &btimers);
}


/*
 * IBSS mode: check the ATIM window size and fix it if necessary.
 *
 * the need for this function arises from the problem that due to unlucky timing
 * of beacon timer configuration (which we try to avoid) and due to unlucky
 * timing of local TSF updates (triggered by the reception of a beacon with the
 * same BSSID - something we can't avoid) the beacon timers can be up updated
 * seperately, leaving one of them in the past, not beeing updated until the
 * timers wrap around. due to the fact that the beacon interval does not fit
 * into the timer period (16 bit) a whole number of times the size of the ATIM
 * window can get bigger than desired.
 *
 * usually we have an ATIM window size of 1 but this function is written to
 * handle other window sizes as well.
 */
int
ath_hw_check_atim(struct ath_softc *sc, int window, int intval)
{
	struct ath_hal *ah = sc->sc_ah;
	unsigned int nbtt, atim, is5210 = 0;

	if (ATH_SREV_FROM_AH(ah) >= AR5K_SREV_VER_AR5416)
		return 0; /* AR5416+ doesn't do ATIM in HW */

	if (ATH_SREV_FROM_AH(ah) == AR5K_SREV_VER_AR5210) {
		nbtt = OS_REG_READ(ah, AR5K_TIMER0_5210);
		atim = OS_REG_READ(ah, AR5K_TIMER3_5210);
		is5210 = 1;
	}
	else {
		nbtt = OS_REG_READ(ah, AR5K_TIMER0_5211);
		atim = OS_REG_READ(ah, AR5K_TIMER3_5211);
	}

	/*
	 * check if the ATIM window is still correct:
	 *   1.) usually ATIM should be NBTT + window
	 *   2.) nbtt already updated
	 *   3.) nbtt already updated and has wrapped around
	 *   4.) atim has wrapped around
	 */
	if ((atim - nbtt != window) &&				/* 1.) */
	    (nbtt - atim != intval - window) &&			/* 2.) */
	    ((nbtt | 0x10000) - atim != intval - window) &&	/* 3.) */
	    ((atim | 0x10000) - nbtt != window)) {		/* 4.) */
		if (is5210)
			OS_REG_WRITE(ah, AR5K_TIMER3_5210, nbtt + window );
		else
			OS_REG_WRITE(ah, AR5K_TIMER3_5211, nbtt + window );
		return atim - nbtt;
	}
	return 0;
}
