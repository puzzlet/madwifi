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

#include "hal/ah_devid.h"
#include "if_media.h"
#include <net80211/ieee80211_var.h>
#include "if_athvar.h"
#include "if_ath_hal.h"
#include "if_ath_hal_macros.h"
#include "if_ath_hal_wrappers.h"
#include "if_ath_hal_extensions.h"


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


int
ath_set_ack_bitrate(struct ath_softc *sc, int high)
{
	if (ar_device(sc->devid) == 5212 || ar_device(sc->devid) == 5213) {
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


