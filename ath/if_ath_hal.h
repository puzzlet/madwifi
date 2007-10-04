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
 */
/* **************************************************************
 * *   WARNING: THIS IS A GENERATED FILE.  PLEASE DO NOT EDIT   *
 * ************************************************************** */

#include "if_ath_hal_macros.h"

#ifndef _IF_ATH_HAL_H_
#define _IF_ATH_HAL_H_

static inline void ath_hal_getmac(struct ath_hal *ah, u_int8_t *a1)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_getMacAddress(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline HAL_POWER_MODE ath_hal_getPowerMode(struct ath_hal *ah)
{
	HAL_POWER_MODE ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_getPowerMode(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_getdiagstate(struct ath_hal *ah, int request, 
		const void *args, u_int32_t argsize, void **result, 
		u_int32_t *resultsize)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_getDiagState(ah, request, args, argsize, *result, resultsize);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline void ath_hal_beaconreset(struct ath_hal *ah)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_resetStationBeaconTimers(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline void ath_hal_setcoverageclass(struct ath_hal *ah, u_int8_t a1, 
		int a2)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_setCoverageClass(ah, a1, a2);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline u_int64_t ath_hal_gettsf64(struct ath_hal *ah)
{
	u_int64_t ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_getTsf64(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline void ath_hal_rxena(struct ath_hal *ah)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_enableReceive(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline HAL_ANT_SETTING ath_hal_getantennaswitch(struct ath_hal *ah)
{
	HAL_ANT_SETTING ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_getAntennaSwitch(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_gpioset(struct ath_hal *ah, u_int32_t gpio, 
		u_int32_t val)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_gpioSet(ah, gpio, val);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_gpioCfgOutput(struct ath_hal *ah, u_int32_t gpio)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_gpioCfgOutput(ah, gpio);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_clearmcastfilter(struct ath_hal *ah, 
		u_int32_t index)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_clrMulticastFilterIndex(ah, index);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline void ath_hal_txreqintrdesc(struct ath_hal *ah, 
		struct ath_desc *a1)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_reqTxIntrDesc(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline void ath_hal_rxmonitor(struct ath_hal *ah, 
		const HAL_NODE_STATS *a1, HAL_CHANNEL *a2)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_rxMonitor(ah, a1, a2);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline HAL_BOOL ath_hal_puttxbuf(struct ath_hal *ah, u_int a1, 
		u_int32_t txdp)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setTxDP(ah, a1, txdp);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_keyset(struct ath_hal *ah, u_int16_t a1, 
		const HAL_KEYVAL *a2, const u_int8_t *a3, int a4)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setKeyCacheEntry(ah, a1, a2, a3, a4);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline void ath_hal_setopmode(struct ath_hal *ah)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_setPCUConfig(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline HAL_RFGAIN ath_hal_getrfgain(struct ath_hal *ah)
{
	HAL_RFGAIN ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_getRfGain(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline void ath_hal_setmcastfilter(struct ath_hal *ah, 
		u_int32_t filter0, u_int32_t filter1)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_setMulticastFilter(ah, filter0, filter1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline u_int ath_hal_getacktimeout(struct ath_hal *ah)
{
	u_int ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_getAckTimeout(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline void ath_hal_beacontimers(struct ath_hal *ah, 
		const HAL_BEACON_STATE *a1)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_setStationBeaconTimers(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline HAL_BOOL ath_hal_detectcardpresent(struct ath_hal *ah)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_detectCardPresent(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline u_int ath_hal_getslottime(struct ath_hal *ah)
{
	u_int ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_getSlotTime(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline void ath_hal_beaconinit(struct ath_hal *ah, u_int32_t nexttbtt, 
		u_int32_t intval)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_beaconInit(ah, nexttbtt, intval);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline void ath_hal_gpiosetintr(struct ath_hal *ah, u_int a1, 
		u_int32_t a2)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_gpioSetIntr(ah, a1, a2);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline HAL_BOOL ath_hal_releasetxqueue(struct ath_hal *ah, u_int q)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_releaseTxQueue(ah, q);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_keysetmac(struct ath_hal *ah, u_int16_t a1, 
		const u_int8_t *a2)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setKeyCacheEntryMac(ah, a1, a2);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_STATUS ath_hal_txprocdesc(struct ath_hal *ah, 
		struct ath_desc *a1, struct ath_tx_status *a2)
{
	HAL_STATUS ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_procTxDesc(ah, a1, a2);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_INT ath_hal_intrget(struct ath_hal *ah)
{
	HAL_INT ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_getInterrupts(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_setacktimeout(struct ath_hal *ah, u_int a1)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setAckTimeout(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_setbssidmask(struct ath_hal *ah, 
		const u_int8_t *a1)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setBssIdMask(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_setackctsrate(struct ath_hal *ah, u_int a1)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setAckCTSRate(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline u_int32_t ath_hal_getrxfilter(struct ath_hal *ah)
{
	u_int32_t ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_getRxFilter(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline int16_t ath_hal_get_channel_noise(struct ath_hal *ah, 
		HAL_CHANNEL *a1)
{
	int16_t ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_getChanNoise(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_keyreset(struct ath_hal *ah, u_int16_t a1)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_resetKeyCacheEntry(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_setantennaswitch(struct ath_hal *ah, 
		HAL_ANT_SETTING a1)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setAntennaSwitch(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_settxqueueprops(struct ath_hal *ah, int q, 
		const HAL_TXQ_INFO *qInfo)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setTxQueueProps(ah, q, qInfo);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline void ath_hal_putrxbuf(struct ath_hal *ah, u_int32_t rxdp)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_setRxDP(ah, rxdp);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline HAL_BOOL ath_hal_reset(struct ath_hal *ah, HAL_OPMODE a1, 
		HAL_CHANNEL *a2, HAL_BOOL bChannelChange, HAL_STATUS *status)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_reset(ah, a1, a2, bChannelChange, status);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_setdecompmask(struct ath_hal *ah, u_int16_t a1, 
		int a2)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setDecompMask(ah, a1, a2);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_gettxqueueprops(struct ath_hal *ah, int q, 
		HAL_TXQ_INFO *qInfo)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_getTxQueueProps(ah, q, qInfo);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_filltxdesc(struct ath_hal *ah, 
		struct ath_desc *a1, u_int segLen, HAL_BOOL firstSeg, 
		HAL_BOOL lastSeg, const struct ath_desc *a5)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_fillTxDesc(ah, a1, segLen, firstSeg, lastSeg, a5);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline u_int32_t ath_hal_numtxpending(struct ath_hal *ah, u_int q)
{
	u_int32_t ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_numTxPending(ah, q);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline void ath_hal_startpcurecv(struct ath_hal *ah)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_startPcuReceive(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline void ath_hal_setdefantenna(struct ath_hal *ah, u_int a1)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_setDefAntenna(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline HAL_BOOL ath_hal_setpower(struct ath_hal *ah, 
		HAL_POWER_MODE mode, int setChip)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setPowerMode(ah, mode, setChip);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_STATUS ath_hal_rxprocdesc(struct ath_hal *ah, 
		struct ath_desc *a1, u_int32_t phyAddr, struct ath_desc *next, 
		u_int64_t tsf, struct ath_rx_status *a5)
{
	HAL_STATUS ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_procRxDesc(ah, a1, phyAddr, next, tsf, a5);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline u_int ath_hal_getackctsrate(struct ath_hal *ah)
{
	u_int ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_getAckCTSRate(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline u_int32_t ath_hal_keycachesize(struct ath_hal *ah)
{
	u_int32_t ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_getKeyCacheSize(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_setupxtxdesc(struct ath_hal *ah, 
		struct ath_desc *a1, u_int txRate1, u_int txTries1, 
		u_int txRate2, u_int txTries2, u_int txRate3, u_int txTries3)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setupXTxDesc(ah, a1, txRate1, txTries1, txRate2, txTries2, 
			txRate3, txTries3);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_INT ath_hal_intrset(struct ath_hal *ah, HAL_INT a1)
{
	HAL_INT ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setInterrupts(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline u_int ath_hal_getctstimeout(struct ath_hal *ah)
{
	u_int ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_getCTSTimeout(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline void ath_hal_updatemibcounters(struct ath_hal *ah, 
		HAL_MIB_STATS *a1)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_updateMibCounters(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline HAL_BOOL ath_hal_calibrate(struct ath_hal *ah, HAL_CHANNEL *a1, 
		HAL_BOOL *a2)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_perCalibration(ah, a1, a2);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline u_int32_t ath_hal_getrxbuf(struct ath_hal *ah)
{
	u_int32_t ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_getRxDP(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_settxpowlimit(struct ath_hal *ah, u_int32_t a1)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setTxPowerLimit(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_getisr(struct ath_hal *ah, HAL_INT *a1)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_getPendingInterrupts(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_updatetxtriglevel(struct ath_hal *ah, 
		HAL_BOOL incTrigLevel)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_updateTxTrigLevel(ah, incTrigLevel);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_resettxqueue(struct ath_hal *ah, u_int q)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_resetTxQueue(ah, q);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_setmac(struct ath_hal *ah, const u_int8_t *a1)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setMacAddress(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_setctstimeout(struct ath_hal *ah, u_int a1)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setCTSTimeout(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline const HAL_RATE_TABLE *ath_hal_getratetable(struct ath_hal *ah, 
		u_int mode)
{
	const HAL_RATE_TABLE *ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_getRateTable(ah, mode);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline u_int32_t ath_hal_gettsf32(struct ath_hal *ah)
{
	u_int32_t ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_getTsf32(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline void ath_hal_mibevent(struct ath_hal *ah, 
		const HAL_NODE_STATS *a1)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_procMibEvent(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline void ath_hal_setbeacontimers(struct ath_hal *ah, 
		const HAL_BEACON_TIMERS *a1)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_setBeaconTimers(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline HAL_STATUS ath_hal_getcapability(struct ath_hal *ah, 
		HAL_CAPABILITY_TYPE a1, u_int32_t capability, 
		u_int32_t *result)
{
	HAL_STATUS ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_getCapability(ah, a1, capability, result);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_radar_wait(struct ath_hal *ah, HAL_CHANNEL *a1)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_radarWait(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_setmcastfilterindex(struct ath_hal *ah, 
		u_int32_t index)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setMulticastFilterIndex(ah, index);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline void ath_hal_getbssidmask(struct ath_hal *ah, u_int8_t *a1)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_getBssIdMask(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline HAL_BOOL ath_hal_intrpend(struct ath_hal *ah)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_isInterruptPending(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_txstart(struct ath_hal *ah, u_int a1)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_startTxDma(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline void ath_hal_gettxintrtxqs(struct ath_hal *ah, u_int32_t *a1)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_getTxIntrQueue(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline HAL_BOOL ath_hal_setslottime(struct ath_hal *ah, u_int a1)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setSlotTime(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline void ath_hal_setledstate(struct ath_hal *ah, HAL_LED_STATE a1)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_setLedState(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline void ath_hal_setassocid(struct ath_hal *ah, 
		const u_int8_t *bssid, u_int16_t assocId)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_writeAssocid(ah, bssid, assocId);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline void ath_hal_resettsf(struct ath_hal *ah)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_resetTsf(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline HAL_BOOL ath_hal_setuprxdesc(struct ath_hal *ah, 
		struct ath_desc *a1, u_int32_t size, u_int flags)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setupRxDesc(ah, a1, size, flags);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline void ath_hal_setrxfilter(struct ath_hal *ah, u_int32_t a1)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_setRxFilter(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline HAL_BOOL ath_hal_keyisvalid(struct ath_hal *ah, u_int16_t a1)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_isKeyCacheEntryValid(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline void ath_hal_stoppcurecv(struct ath_hal *ah)
{
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ah->ah_stopPcuReceive(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
}

static inline HAL_BOOL ath_hal_stoptxdma(struct ath_hal *ah, u_int a1)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_stopTxDma(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_setcapability(struct ath_hal *ah, 
		HAL_CAPABILITY_TYPE a1, u_int32_t capability, 
		u_int32_t setting, HAL_STATUS *a4)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setCapability(ah, a1, capability, setting, a4);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_stopdmarecv(struct ath_hal *ah)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_stopDmaReceive(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline u_int32_t ath_hal_gettxbuf(struct ath_hal *ah, u_int a1)
{
	u_int32_t ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_getTxDP(ah, a1);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline int ath_hal_setuptxqueue(struct ath_hal *ah, HAL_TX_QUEUE a1, 
		const HAL_TXQ_INFO *qInfo)
{
	int ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setupTxQueue(ah, a1, qInfo);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline u_int ath_hal_getdefantenna(struct ath_hal *ah)
{
	u_int ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_getDefAntenna(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_phydisable(struct ath_hal *ah)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_phyDisable(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_setregulatorydomain(struct ath_hal *ah, 
		u_int16_t a1, HAL_STATUS *a2)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setRegulatoryDomain(ah, a1, a2);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_setuptxdesc(struct ath_hal *ah, 
		struct ath_desc *a1, u_int pktLen, u_int hdrLen, 
		HAL_PKT_TYPE type, u_int txPower, u_int txRate0, 
		u_int txTries0, u_int keyIx, u_int antMode, u_int flags, 
		u_int rtsctsRate, u_int rtsctsDuration, u_int compicvLen, 
		u_int compivLen, u_int comp)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_setupTxDesc(ah, a1, pktLen, hdrLen, type, txPower, 
			txRate0, txTries0, keyIx, antMode, flags, rtsctsRate, 
			rtsctsDuration, compicvLen, compivLen, comp);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_gpiCfgInput(struct ath_hal *ah, u_int32_t gpio)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_gpioCfgInput(ah, gpio);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline u_int32_t ath_hal_gpioget(struct ath_hal *ah, u_int32_t gpio)
{
	u_int32_t ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_gpioGet(ah, gpio);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

static inline HAL_BOOL ath_hal_disable(struct ath_hal *ah)
{
	HAL_BOOL ret;
	ATH_HAL_LOCK_IRQ(ah->ah_sc);
	ath_hal_set_function(__func__);
	ret = ah->ah_disable(ah);
	ath_hal_set_function(NULL);
	ATH_HAL_UNLOCK_IRQ(ah->ah_sc);
	return ret;
}

#include "if_ath_hal_wrappers.h"

#endif				/* #ifndef _IF_ATH_HAL_H_ */
 /* *** THIS IS A GENERATED FILE -- DO NOT EDIT *** */
 /* *** THIS IS A GENERATED FILE -- DO NOT EDIT *** */
 /* *** THIS IS A GENERATED FILE -- DO NOT EDIT *** */
