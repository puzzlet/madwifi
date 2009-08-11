/*-
 * Copyright (c) 2007 Michael Taylor
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
 * $Id: foo mtaylor $
 */

/* This file provides some wrapper functions that invoke functions in 
 * if_ath_hal.h.  Since all the functions in the generated file, if_ath_hal.h
 * have locks to protect them... no further locking is required in these 
 * additional helper functions.  Mostly these just provide a series of nicknames
 * for specific sets of arguments to HAL functions that are commonly needed. */

#ifndef _IF_ATH_HAL_EXTENSIONS_H_
#define _IF_ATH_HAL_EXTENSIONS_H_

#include <linux/types.h>

#define AR5K_PHY_AGCSIZE			0x9850
#define AR5K_PHY_AGCSIZE_DESIRED		0x0ff00000
#define AR5K_PHY_AGCSIZE_DESIRED_S		20

#define AR5K_PHY_SIG				0x9858
#define AR5K_PHY_SIG_FIRSTEP			0x0003f000
#define AR5K_PHY_SIG_FIRSTEP_S			12
#define AR5K_PHY_SIG_FIRPWR			0x03fc0000
#define AR5K_PHY_SIG_FIRPWR_S			18

#define AR5K_PHY_AGCCOARSE			0x985c
#define AR5K_PHY_AGCCOARSE_LO			0x00007f80
#define AR5K_PHY_AGCCOARSE_LO_S			7
#define AR5K_PHY_AGCCOARSE_HI			0x003f8000
#define AR5K_PHY_AGCCOARSE_HI_S			15

#define AR5K_PHY_WEAK_OFDM_HIGH			0x9868
#define AR5K_PHY_WEAK_OFDM_HIGH_M1		0x00fe0000
#define AR5K_PHY_WEAK_OFDM_HIGH_M1_S		17
#define AR5K_PHY_WEAK_OFDM_HIGH_M2		0x7f000000
#define AR5K_PHY_WEAK_OFDM_HIGH_M2_S		24
#define AR5K_PHY_WEAK_OFDM_HIGH_M2_COUNT	0x0000001f
#define AR5K_PHY_WEAK_OFDM_HIGH_M2_COUNT_S	0

#define AR5K_PHY_WEAK_OFDM_HIGH_M1_OFF		127
#define AR5K_PHY_WEAK_OFDM_HIGH_M1_ON		77

#define AR5K_PHY_WEAK_OFDM_HIGH_M2_OFF		127
#define AR5K_PHY_WEAK_OFDM_HIGH_M2_ON		64

#define AR5K_PHY_WEAK_OFDM_HIGH_M2_COUNT_OFF	31
#define AR5K_PHY_WEAK_OFDM_HIGH_M2_COUNT_ON	16

/* NB: comparing defaults for these registers vs. patent, it appears this
 * is baseband processor stuff from table 2. */
#define AR5K_PHY_WEAK_OFDM_LOW 			0x986c
#define AR5K_PHY_WEAK_OFDM_LOW_M1		0x001fc000
#define AR5K_PHY_WEAK_OFDM_LOW_M1_S		14		
#define AR5K_PHY_WEAK_OFDM_LOW_M2		0x0fe00000
#define AR5K_PHY_WEAK_OFDM_LOW_M2_S		21
#define AR5K_PHY_WEAK_OFDM_LOW_M2_COUNT		0x00003f00
#define AR5K_PHY_WEAK_OFDM_LOW_M2_COUNT_S	8
#define AR5K_PHY_WEAK_OFDM_LOW_SELFCOR		0x00000001
#define AR5K_PHY_WEAK_OFDM_LOW_SELFCOR_S	0	

#define AR5K_PHY_WEAK_OFDM_LOW_M1_OFF		127
#define AR5K_PHY_WEAK_OFDM_LOW_M1_ON		50

#define AR5K_PHY_WEAK_OFDM_LOW_M2_OFF		127
#define AR5K_PHY_WEAK_OFDM_LOW_M2_ON		40

#define AR5K_PHY_WEAK_OFDM_LOW_M2_COUNT_OFF	63
#define AR5K_PHY_WEAK_OFDM_LOW_M2_COUNT_ON	48

#define AR5K_PHY_WEAK_OFDM_LOW_SELFCOR_OFF	0
#define AR5K_PHY_WEAK_OFDM_LOW_SELFCOR_ON	1

#define AR5K_PHY_SPUR				0x9924
#define AR5K_PHY_SPUR_THRESH			0x000000fe
#define AR5K_PHY_SPUR_THRESH_S			1

#define AR5K_PHY_WEAK_CCK			0xa208
#define AR5K_PHY_WEAK_CCK_THRESH		0x0000000f
#define AR5K_PHY_WEAK_CCK_THRESH_S		0

#define	AR5K_PHY_WEAK_CCK_THRESH_ON		6
#define	AR5K_PHY_WEAK_CCK_THRESH_OFF		8

#define DEFAULT_AR5K_PHY_AGCSIZE_DESIRED	-34
#define DEFAULT_AR5K_PHY_AGCCOARSE_HI		-18
#define DEFAULT_AR5K_PHY_AGCCOARSE_LO		-52
#define DEFAULT_AR5K_PHY_SIG_FIRPWR		-70

/* NB: I can never make up my mind which is right. */
#define DEFAULT_ENABLE_AR5K_PHY_WEAK_OFDM_11BG 	1
#define DEFAULT_ENABLE_AR5K_PHY_WEAK_OFDM_11A 	1

#define IS_CHAN_ANY(ah) \
	(((struct ieee80211com *)ah->ah_sc)->ic_bsschan == IEEE80211_CHAN_ANYC)

#define IS_BG_OR_ANY(ah) \
 (IS_CHAN_ANY(ah) || (!(ieee80211_chan2mode(((struct ieee80211com *)ah->ah_sc)->ic_bsschan) & \
		(IEEE80211_MODE_11A | IEEE80211_MODE_TURBO_A))))

#define DEFAULT_ENABLE_AR5K_PHY_WEAK_OFDM (IS_BG_OR_ANY(ah) ? \
		DEFAULT_ENABLE_AR5K_PHY_WEAK_OFDM_11BG : \
		DEFAULT_ENABLE_AR5K_PHY_WEAK_OFDM_11A \
		)

#define DEFAULT_ENABLE_AR5K_PHY_WEAK_OFDM_HIGH	DEFAULT_ENABLE_AR5K_PHY_WEAK_OFDM
#define DEFAULT_ENABLE_AR5K_PHY_WEAK_OFDM_LOW	DEFAULT_ENABLE_AR5K_PHY_WEAK_OFDM
#define DEFAULT_ENABLE_AR5K_PHY_WEAK_CCK	0

#define DEFAULT_AR5K_PHY_SPUR_THRESH		2
#define DEFAULT_AR5K_PHY_SIG_FIRSTEP		0

/*
 * Transmit configuration register
 */
#define AR5K_TXCFG		0x0030			/* Register Address */
#define AR5K_TXCFG_SDMAMR	0x00000007	/* DMA size */
#define AR5K_TXCFG_SDMAMR_S	0

/*
 * Receive configuration register
 */
#define AR5K_RXCFG		0x0034			/* Register Address */
#define AR5K_RXCFG_SDMAMW	0x00000007	/* DMA size */
#define AR5K_RXCFG_SDMAMW_S	0

/*
 * Second station id register (MAC address in upper 16 bits)
 */
#define AR5K_STA_ID1			0x8004			/* Register Address */
#define AR5K_STA_ID1_AP			0x00010000	/* Set AP mode */
#define AR5K_STA_ID1_ADHOC		0x00020000	/* Set Ad-Hoc mode */
#define AR5K_STA_ID1_PWR_SV		0x00040000	/* Power save reporting (?) */
#define AR5K_STA_ID1_NO_KEYSRCH		0x00080000	/* No key search */
#define AR5K_STA_ID1_NO_PSPOLL		0x00100000	/* No power save polling [5210] */
#define AR5K_STA_ID1_PCF_5211		0x00100000	/* Enable PCF on [5211+] */
#define AR5K_STA_ID1_PCF_5210		0x00200000	/* Enable PCF on [5210] */
#define	AR5K_STA_ID1_PCF		(ah->ah_version == AR5K_AR5210 ? \
					AR5K_STA_ID1_PCF_5210 : AR5K_STA_ID1_PCF_5211)
#define AR5K_STA_ID1_DEFAULT_ANTENNA	0x00200000	/* Use default antenna */
#define AR5K_STA_ID1_DESC_ANTENNA	0x00400000	/* Update antenna from descriptor */
#define AR5K_STA_ID1_RTS_DEF_ANTENNA	0x00800000	/* Use default antenna for RTS (?) */
#define AR5K_STA_ID1_ACKCTS_6MB		0x01000000	/* Use 6Mbit/s for ACK/CTS (?) */
#define AR5K_STA_ID1_BASE_RATE_11B	0x02000000	/* Use 11b base rate (for ACK/CTS ?) [5211+] */

/*
 * PCU beacon control register
 */
#define AR5K_BEACON_5210	0x8024
#define AR5K_BEACON_5211	0x8020

/*
 * Next beacon time register
 */
#define AR5K_TIMER0_5210	0x802c
#define AR5K_TIMER0_5211	0x8028
/*
 * Next DMA beacon alert register
 */
#define AR5K_TIMER1_5210	0x8030
#define AR5K_TIMER1_5211	0x802c

/*
 * Next software beacon alert register
 */
#define AR5K_TIMER2_5210	0x8034
#define AR5K_TIMER2_5211	0x8030

/*
 * Next ATIM window time register
 */
#define AR5K_TIMER3_5210	0x8038
#define AR5K_TIMER3_5211	0x8034


enum ath5k_srev_type {
	AR5K_VERSION_VER,
	AR5K_VERSION_RAD,
};

struct ath5k_srev_name {
	const char		*sr_name;
	enum ath5k_srev_type	sr_type;
	u_int			sr_val;
};

#define AR5K_SREV_UNKNOWN	0xffff

#define AR5K_SREV_VER_AR5210	0x00
#define AR5K_SREV_VER_AR5311	0x10
#define AR5K_SREV_VER_AR5311A	0x20
#define AR5K_SREV_VER_AR5311B	0x30
#define AR5K_SREV_VER_AR5211	0x40
#define AR5K_SREV_VER_AR5212	0x50
#define AR5K_SREV_VER_AR5213	0x55
#define AR5K_SREV_VER_AR5213A	0x59
#define AR5K_SREV_VER_AR2413	0x78
#define AR5K_SREV_VER_AR2414	0x79
#define AR5K_SREV_VER_AR2424	0xa0 /* PCI-E */
#define AR5K_SREV_VER_AR5424	0xa3 /* PCI-E */
#define AR5K_SREV_VER_AR5413	0xa4
#define AR5K_SREV_VER_AR5414	0xa5
#define AR5K_SREV_VER_AR5416	0xc0 /* PCI-E */
#define AR5K_SREV_VER_AR5418	0xca /* PCI-E */
#define AR5K_SREV_VER_AR2425	0xe2 /* PCI-E */
#define AR5K_SREV_VER_AR9160	0x400
#define AR5K_SREV_VER_AR9280	0x800

#define AR5K_SREV_RAD_5110	0x00
#define AR5K_SREV_RAD_5111	0x10
#define AR5K_SREV_RAD_5111A	0x15
#define AR5K_SREV_RAD_2111	0x20
#define AR5K_SREV_RAD_5112	0x30
#define AR5K_SREV_RAD_5112A	0x35
#define AR5K_SREV_RAD_2112	0x40
#define AR5K_SREV_RAD_2112A	0x45
#define AR5K_SREV_RAD_SC0	0x56	/* Found on 2413/2414 */
#define AR5K_SREV_RAD_SC1	0x63	/* Found on 5413/5414 */
#define AR5K_SREV_RAD_SC2	0xa2	/* Found on 2424-5/5424 */
#define AR5K_SREV_RAD_5133	0xc0	/* MIMO found on 5418 */

#define ATH_SREV_FROM_AH(_ah)	((_ah)->ah_macVersion << 4 | (_ah)->ah_macRev)

/*
 * DMA size definitions (2^(n+2))
 */
enum ath5k_dmasize {
	AR5K_DMASIZE_4B	= 0,
	AR5K_DMASIZE_8B,
	AR5K_DMASIZE_16B,
	AR5K_DMASIZE_32B,
	AR5K_DMASIZE_64B,
	AR5K_DMASIZE_128B,
	AR5K_DMASIZE_256B,
	AR5K_DMASIZE_512B
};


int ath_set_ack_bitrate(struct ath_softc *sc, int);
int ar_device(struct ath_softc *sc);
const char * ath5k_chip_name(enum ath5k_srev_type type, u_int16_t val);
void ath_hw_beacon_stop(struct ath_softc *sc);
int ath_hw_check_atim(struct ath_softc *sc, int window, int intval);

static inline unsigned long field_width(unsigned long mask, unsigned long shift)
{
	unsigned long r = 0;
	unsigned long x = mask >> shift;
	if ( 0 == mask )  return  0;
#if  BITS_PER_LONG >= 64
	if ( x & (~0UL<<32) )  { x >>= 32;  r += 32; }
#endif
	if ( x & 0xffff0000 )  { x >>= 16;  r += 16; }
	if ( x & 0x0000ff00 )  { x >>=  8;  r +=  8; }
	if ( x & 0x000000f0 )  { x >>=  4;  r +=  4; }
	if ( x & 0x0000000c )  { x >>=  2;  r +=  2; }
	if ( x & 0x00000002 )  {            r +=  1; }
	return r+1;
}

static inline u_int32_t get_field(struct ath_hal *ah, u_int32_t reg, u_int32_t mask, u_int32_t shift, int is_signed) {
	unsigned long x = ((OS_REG_READ(ah, reg) & mask) >> shift);
	if (is_signed) {
		unsigned long c =(-1) << (field_width(mask, shift)-1);
		return (x + c) ^ c;
	}
	return x;
}

static inline void set_field(struct ath_hal *ah, u_int32_t reg, u_int32_t mask, u_int32_t shift, u_int32_t value) {
	OS_REG_WRITE(ah, reg, 
			  (OS_REG_READ(ah, reg) & ~mask) | 
			  ((value << shift) & mask));
}

static inline u_int32_t field_eq(struct ath_hal *ah, u_int32_t reg, 
				 u_int32_t mask, u_int32_t shift, 
				 u_int32_t value, int is_signed) {
	return  (get_field(ah, reg, mask, shift, is_signed) & (mask >> shift)) == 
		(value & (mask >> shift));
}

static inline void override_warning(struct ath_hal *ah, const char *name,
				    u_int32_t reg, u_int32_t mask,
				    u_int32_t shift, u_int32_t expected, int is_signed) {

	if (!field_eq(ah, reg, mask, shift, expected, is_signed)) 
		printk("%s: Correcting 0x%04x[%s] from 0x%x (%d) to 0x%x (%d).\n", 
		       SC_DEV_NAME(ah->ah_sc),
		       reg,
		       name, 
		       (get_field(ah, reg, mask, shift, is_signed) & (mask >> shift)),
		       get_field(ah, reg, mask, shift, is_signed), 
		       (expected & (mask >> shift)), /* not sign extended */
		       expected);
#if 0 /* NB: For checking to see if HAL is fixed or not */
	else {
			printk("%s: Keeping 0x%04x[%s] - 0x%x (%d).\n",
			       SC_DEV_NAME(ah->ah_sc),
			       reg,
			       name, 
			       (get_field(ah, reg, mask, shift, is_signed) & (mask >> shift)),
			       get_field(ah, reg, mask, shift, is_signed));
	}
#endif
}

static inline void verification_warning(struct ath_hal *ah, const char *name,
    u_int32_t reg, u_int32_t mask, 
    u_int32_t shift, u_int32_t expected, int is_signed) {

	int ret = field_eq(ah, reg, mask, shift, expected, is_signed);
	if (!ret) {
		printk("%s: %s verification of %s default value "
		       "[found=0x%x (%d) expected=0x%x (%d)].\n", 
		       SC_DEV_NAME(ah->ah_sc),
		       (ret ? "PASSED" : "FAILED"),
			name, 
		       (get_field(ah, reg, mask, shift, is_signed) & (mask >> shift)), 
		       get_field(ah, reg, mask, shift, is_signed), 
		       (expected & (mask >> shift)), /* not sign extended */
		       expected);
		ath_hal_print_decoded_register(ah, NULL, reg, 
					       OS_REG_READ(ah, reg), OS_REG_READ(ah, reg), 0);
	}
}

#define GET_FIELD(ah, __reg, __mask, __signed) \
	get_field(ah, __reg, __mask, __mask ## _S, __signed)
#define SET_FIELD(ah, __reg, __mask, __value) \
	set_field(ah, __reg, __mask, __mask ## _S, __value);
#define FIELD_EQ(ah, __reg, __mask, __value, __signed) \
	field_eq(ah, __reg, __mask, __mask ## _S, __value, __signed)

#if 0 /* NB: These are working at this point, and HAL tweaks them a lot */
#define OVERRIDE_WARNING(ah, __reg, __mask, __expected, __signed) \
	override_warning(ah, #__mask, __reg, __mask, __mask ## _S, __expected, __signed)
#else
#define OVERRIDE_WARNING(ah, __reg, __mask, __expected, __signed) 
#endif
	
#define VERIFICATION_WARNING(ah, __reg, __mask, __signed) \
	verification_warning(ah, #__mask, __reg, __mask, __mask ## _S, DEFAULT_ ## __mask, __signed)
#define VERIFICATION_WARNING_SW(ah, __reg, __mask, __signed) \
	verification_warning(ah, #__mask, __reg, __mask, __mask ## _S, DEFAULT_ENABLE_ ## __reg ? __mask ## _ON : __mask ## _OFF, __signed)

static inline void ath_hal_set_noise_immunity(struct ath_hal *ah,
					      int agc_desired_size, 
					      int agc_coarse_hi,
					      int agc_coarse_lo, 
					      int sig_firpwr) 
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ath_hal_set_device(SC_DEV_NAME(ah->ah_sc));

#if 0 /* NB: These are working at this point, and HAL tweaks them a lot */
	OVERRIDE_WARNING(ah, AR5K_PHY_AGCSIZE, AR5K_PHY_AGCSIZE_DESIRED, agc_desired_size, 1);
	OVERRIDE_WARNING(ah, AR5K_PHY_AGCCOARSE, AR5K_PHY_AGCCOARSE_LO, agc_coarse_lo, 1);
	OVERRIDE_WARNING(ah, AR5K_PHY_AGCCOARSE, AR5K_PHY_AGCCOARSE_HI, agc_coarse_hi, 1);
	OVERRIDE_WARNING(ah, AR5K_PHY_SIG, AR5K_PHY_SIG_FIRPWR, sig_firpwr, 1);
#endif

	SET_FIELD(ah, AR5K_PHY_AGCSIZE, AR5K_PHY_AGCSIZE_DESIRED, agc_desired_size);
	SET_FIELD(ah, AR5K_PHY_AGCCOARSE, AR5K_PHY_AGCCOARSE_LO, agc_coarse_lo);
	SET_FIELD(ah, AR5K_PHY_AGCCOARSE, AR5K_PHY_AGCCOARSE_HI, agc_coarse_hi);
	SET_FIELD(ah, AR5K_PHY_SIG, AR5K_PHY_SIG_FIRPWR, sig_firpwr);

	ath_hal_set_function(NULL);
	ath_hal_set_device(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline void ath_hal_set_ofdm_weak_det(struct ath_hal *ah, 
	int low_m1, int low_m2, int low_m2_count, int low_self_corr,
	int high_m1, int high_m2, int high_m2_count)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ath_hal_set_device(SC_DEV_NAME(ah->ah_sc));

	OVERRIDE_WARNING(ah, AR5K_PHY_WEAK_OFDM_LOW, AR5K_PHY_WEAK_OFDM_LOW_M1, low_m1, 0);
	OVERRIDE_WARNING(ah, AR5K_PHY_WEAK_OFDM_LOW, AR5K_PHY_WEAK_OFDM_LOW_M2, low_m2, 0);
	OVERRIDE_WARNING(ah, AR5K_PHY_WEAK_OFDM_LOW, AR5K_PHY_WEAK_OFDM_LOW_M2_COUNT, low_m2_count, 0);
	OVERRIDE_WARNING(ah, AR5K_PHY_WEAK_OFDM_LOW, AR5K_PHY_WEAK_OFDM_LOW_SELFCOR, low_self_corr, 0);
	OVERRIDE_WARNING(ah, AR5K_PHY_WEAK_OFDM_HIGH, AR5K_PHY_WEAK_OFDM_HIGH_M1, high_m1, 0);
	OVERRIDE_WARNING(ah, AR5K_PHY_WEAK_OFDM_HIGH, AR5K_PHY_WEAK_OFDM_HIGH_M2, high_m2, 0);
	OVERRIDE_WARNING(ah, AR5K_PHY_WEAK_OFDM_HIGH, AR5K_PHY_WEAK_OFDM_HIGH_M2_COUNT, high_m2_count, 0);

	SET_FIELD(ah, AR5K_PHY_WEAK_OFDM_LOW, AR5K_PHY_WEAK_OFDM_LOW_M1, low_m1);
	SET_FIELD(ah, AR5K_PHY_WEAK_OFDM_LOW, AR5K_PHY_WEAK_OFDM_LOW_M2, low_m2);
	SET_FIELD(ah, AR5K_PHY_WEAK_OFDM_LOW, AR5K_PHY_WEAK_OFDM_LOW_M2_COUNT, low_m2_count);
	SET_FIELD(ah, AR5K_PHY_WEAK_OFDM_LOW, AR5K_PHY_WEAK_OFDM_LOW_SELFCOR, low_self_corr);
	SET_FIELD(ah, AR5K_PHY_WEAK_OFDM_HIGH, AR5K_PHY_WEAK_OFDM_HIGH_M1, high_m1);
	SET_FIELD(ah, AR5K_PHY_WEAK_OFDM_HIGH, AR5K_PHY_WEAK_OFDM_HIGH_M2, high_m2);
	SET_FIELD(ah, AR5K_PHY_WEAK_OFDM_HIGH, AR5K_PHY_WEAK_OFDM_HIGH_M2_COUNT, high_m2_count);

	ath_hal_set_function(NULL);
	ath_hal_set_device(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline void ath_hal_set_cck_weak_det(struct ath_hal *ah, int thresh)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ath_hal_set_device(SC_DEV_NAME(ah->ah_sc));

	OVERRIDE_WARNING(ah, AR5K_PHY_WEAK_CCK, AR5K_PHY_WEAK_CCK_THRESH, thresh, 0);

	SET_FIELD(ah, AR5K_PHY_WEAK_CCK, AR5K_PHY_WEAK_CCK_THRESH, thresh);

	ath_hal_set_function(NULL);
	ath_hal_set_device(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline void ath_hal_set_sig_firstep(struct ath_hal *ah, int firstep)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ath_hal_set_device(SC_DEV_NAME(ah->ah_sc));

	OVERRIDE_WARNING(ah, AR5K_PHY_SIG, AR5K_PHY_SIG_FIRSTEP, firstep, 0);

	SET_FIELD(ah, AR5K_PHY_SIG, AR5K_PHY_SIG_FIRSTEP, firstep);

	ath_hal_set_function(NULL);
	ath_hal_set_device(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline void ath_hal_set_spur_immunity(struct ath_hal *ah, int thresh)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ath_hal_set_device(SC_DEV_NAME(ah->ah_sc));

	OVERRIDE_WARNING(ah, AR5K_PHY_SPUR, AR5K_PHY_SPUR_THRESH, thresh, 0);

	SET_FIELD(ah, AR5K_PHY_SPUR, AR5K_PHY_SPUR_THRESH, thresh);

	ath_hal_set_function(NULL);
	ath_hal_set_device(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline void ath_hal_restore_default_noise_immunity(struct ath_hal *ah) {

	ath_hal_set_noise_immunity(ah, 
		DEFAULT_AR5K_PHY_AGCSIZE_DESIRED, 
		DEFAULT_AR5K_PHY_AGCCOARSE_HI,
		DEFAULT_AR5K_PHY_AGCCOARSE_LO,
		DEFAULT_AR5K_PHY_SIG_FIRPWR);
}

static inline void ath_hal_enable_ofdm_weak_det(struct ath_hal *ah, int enable) {
	if (enable)
		ath_hal_set_ofdm_weak_det(ah, 
			AR5K_PHY_WEAK_OFDM_LOW_M1_ON,
			AR5K_PHY_WEAK_OFDM_LOW_M2_ON,
			AR5K_PHY_WEAK_OFDM_LOW_M2_COUNT_ON,
		        AR5K_PHY_WEAK_OFDM_LOW_SELFCOR_ON,
			AR5K_PHY_WEAK_OFDM_HIGH_M1_ON,
			AR5K_PHY_WEAK_OFDM_HIGH_M2_ON,
			AR5K_PHY_WEAK_OFDM_HIGH_M2_COUNT_ON);
	else
		ath_hal_set_ofdm_weak_det(ah, 
			AR5K_PHY_WEAK_OFDM_LOW_M1_OFF,
			AR5K_PHY_WEAK_OFDM_LOW_M2_OFF,
			AR5K_PHY_WEAK_OFDM_LOW_M2_COUNT_OFF,
			AR5K_PHY_WEAK_OFDM_LOW_SELFCOR_OFF,
			AR5K_PHY_WEAK_OFDM_HIGH_M1_OFF,
			AR5K_PHY_WEAK_OFDM_HIGH_M2_OFF,
			AR5K_PHY_WEAK_OFDM_HIGH_M2_COUNT_OFF);
}

static inline void ath_hal_enable_cck_weak_det(struct ath_hal *ah, int enable) {
	ath_hal_set_cck_weak_det(ah, enable 
				 ? AR5K_PHY_WEAK_CCK_THRESH_ON 
				 : AR5K_PHY_WEAK_CCK_THRESH_OFF);
}

static inline void ath_hal_restore_default_ofdm_weak_det(struct ath_hal *ah) {
	ath_hal_enable_ofdm_weak_det(ah, DEFAULT_ENABLE_AR5K_PHY_WEAK_OFDM);
}

static inline void ath_hal_restore_default_cck_weak_det(struct ath_hal *ah) {
	ath_hal_enable_cck_weak_det(ah, DEFAULT_ENABLE_AR5K_PHY_WEAK_CCK);
}

static inline void ath_hal_restore_default_sig_firstep(struct ath_hal *ah) {

	ath_hal_set_sig_firstep(ah, 
		DEFAULT_AR5K_PHY_SIG_FIRSTEP);
}

static inline void ath_hal_restore_default_spur_immunity(struct ath_hal *ah) {

	ath_hal_set_spur_immunity(ah, 
		DEFAULT_AR5K_PHY_SPUR_THRESH);
}

static inline void ath_hal_restore_default_intmit(struct ath_hal *ah) {
	ath_hal_restore_default_noise_immunity(ah);
	ath_hal_restore_default_ofdm_weak_det(ah);
	ath_hal_restore_default_cck_weak_det(ah);
	ath_hal_restore_default_sig_firstep(ah);
	ath_hal_restore_default_spur_immunity(ah);

}

static inline void ath_hal_verify_default_intmit(struct ath_hal *ah) {
	/* Just a list of all the fields above, for sanity checks... */
	VERIFICATION_WARNING(ah, AR5K_PHY_AGCSIZE, AR5K_PHY_AGCSIZE_DESIRED, 1);
	VERIFICATION_WARNING(ah, AR5K_PHY_AGCCOARSE, AR5K_PHY_AGCCOARSE_LO, 1);
	VERIFICATION_WARNING(ah, AR5K_PHY_AGCCOARSE, AR5K_PHY_AGCCOARSE_HI, 1);
	VERIFICATION_WARNING(ah, AR5K_PHY_SIG, AR5K_PHY_SIG_FIRPWR, 1);
	VERIFICATION_WARNING_SW(ah, AR5K_PHY_WEAK_OFDM_LOW, AR5K_PHY_WEAK_OFDM_LOW_M1, 0);
	VERIFICATION_WARNING_SW(ah, AR5K_PHY_WEAK_OFDM_LOW, AR5K_PHY_WEAK_OFDM_LOW_M2, 0);
	VERIFICATION_WARNING_SW(ah, AR5K_PHY_WEAK_OFDM_LOW, AR5K_PHY_WEAK_OFDM_LOW_M2_COUNT, 0);
	VERIFICATION_WARNING_SW(ah, AR5K_PHY_WEAK_OFDM_LOW, AR5K_PHY_WEAK_OFDM_LOW_SELFCOR, 0);
	VERIFICATION_WARNING_SW(ah, AR5K_PHY_WEAK_OFDM_HIGH, AR5K_PHY_WEAK_OFDM_HIGH_M1, 0);
	VERIFICATION_WARNING_SW(ah, AR5K_PHY_WEAK_OFDM_HIGH, AR5K_PHY_WEAK_OFDM_HIGH_M2, 0);
	VERIFICATION_WARNING_SW(ah, AR5K_PHY_WEAK_OFDM_HIGH, AR5K_PHY_WEAK_OFDM_HIGH_M2_COUNT, 0);
	VERIFICATION_WARNING_SW(ah, AR5K_PHY_WEAK_CCK, AR5K_PHY_WEAK_CCK_THRESH, 0);
	VERIFICATION_WARNING(ah, AR5K_PHY_SIG, AR5K_PHY_SIG_FIRSTEP, 0);
	VERIFICATION_WARNING(ah, AR5K_PHY_SPUR, AR5K_PHY_SPUR_THRESH, 0);
}

static inline void ath_hal_set_dmasize_pcie(struct ath_hal *ah) {
	SET_FIELD(ah, AR5K_TXCFG, AR5K_TXCFG_SDMAMR, AR5K_DMASIZE_128B);
	SET_FIELD(ah, AR5K_RXCFG, AR5K_RXCFG_SDMAMW, AR5K_DMASIZE_128B);
}

#endif /* _IF_ATH_HAL_EXTENSIONS_H_ */
