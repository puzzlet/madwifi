/*-
 * Copyright (c) 2002-2006 Sam Leffler, Errno Consulting
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
#include "opt_ah.h"

#ifndef EXPORT_SYMTAB
#define	EXPORT_SYMTAB
#endif /* EXPORT_SYMTAB */

/* Don't use virtualized timer in Linux 2.6.20+ */
#define USE_REAL_TIME_DELAY

#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif /* AUTOCONF_INCLUDED */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/sched.h>

#include <linux/sysctl.h>
#include <linux/proc_fs.h>

#include <asm/io.h>

#include <ah.h>
#include <ah_os.h>

#ifdef AH_DEBUG
int ath_hal_debug = 0;
EXPORT_SYMBOL(ath_hal_debug);
#endif /* AH_DEBUG */

#define		MSG_MAXLEN	161

#define REGOP_NONE  0
#define REGOP_READ  1
#define REGOP_WRITE 2


int	ath_hal_dma_beacon_response_time = 2;	/* in TUs */
int	ath_hal_sw_beacon_response_time = 10;	/* in TUs */
int	ath_hal_additional_swba_backoff = 0;	/* in TUs */

struct ath_hal *
_ath_hal_attach(u_int16_t devid, HAL_SOFTC sc,
		HAL_BUS_TAG t, HAL_BUS_HANDLE h, HAL_STATUS *s)
{
	struct ath_hal *ah = ath_hal_attach(devid, sc, t, h, s);

	if (ah)
#ifndef __MOD_INC_USE_COUNT
		if (!try_module_get(THIS_MODULE)) {
			printk(KERN_WARNING "%s: try_module_get failed\n",
					__func__);
			_ath_hal_detach(ah);
			return NULL;
		}
#else /* __MOD_INC_USE_COUNT */
		MOD_INC_USE_COUNT;
#endif /* __MOD_INC_USE_COUNT */
	return ah;
}

void
_ath_hal_detach(struct ath_hal *ah)
{
	(*ah->ah_detach)(ah);
#ifndef __MOD_INC_USE_COUNT
	module_put(THIS_MODULE);
#else /* __MOD_INC_USE_COUNT */
	MOD_DEC_USE_COUNT;
#endif /* __MOD_INC_USE_COUNT */
}

/*
 * Print/log message support.
 */

#ifdef AH_ASSERT
void __ahdecl
ath_hal_assert_failed(const char *filename, int lineno, const char *msg)
{
	printk(KERN_ERR "Atheros HAL assertion failure: %s: line %u: %s\n",
		filename, lineno, msg);
	panic("ath_hal_assert");
}
#endif /* AH_ASSERT */

/* Store the current function name (should be called by wrapper functions)
 * useful for debugging and figuring out, which hal function sets which 
 * registers */
const char *ath_hal_func = NULL;
const char *ath_hal_device = NULL;
EXPORT_SYMBOL(ath_hal_func);
EXPORT_SYMBOL(ath_hal_device);

#ifdef AH_DEBUG_ALQ
/*
 * ALQ register tracing support.
 *
 * Setting hw.ath.hal.alq=1 enables tracing of all register reads and
 * writes to the file /var/log/ath_hal.log.  The file format is a simple
 * fixed-size array of records.  When done logging set hw.ath.hal.alq=0
 * and then decode the file with the ardecode program (that is part of the
 * HAL).  If you start+stop tracing the data will be appended to an
 * existing file.
 *
 * NB: doesn't handle multiple devices properly; only one DEVICE record
 *     is emitted and the different devices are not identified.
 */
#include "alq.h"

static	struct alq *ath_hal_alq = NULL;
static	u_int ath_hal_alq_lost;		/* count of lost records */
static	const char *ath_hal_logfile = "/var/log/ath_hal.log";
static	u_int ath_hal_alq_qsize = 8 * 1024;


static int
ath_hal_setlogging(int enable)
{
	int error;

	if (enable) {
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		error = alq_open(&ath_hal_alq, ath_hal_logfile,
				MSG_MAXLEN, ath_hal_alq_qsize, (enable == 1 ? 0x7fffffff : enable));
		ath_hal_alq_lost = 0;
		printk(KERN_INFO "ath_hal: logging to %s %s\n", ath_hal_logfile,
			error == 0 ? "enabled" : "could not be setup");
	} else {
		if (ath_hal_alq)
			alq_close(ath_hal_alq);
		ath_hal_alq = NULL;
		printk(KERN_INFO "ath_hal: logging disabled\n");
		error = 0;
	}
	return error;
}

/*
 * Deal with the sysctl handler api changing.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
#define	AH_SYSCTL_ARGS_DECL \
	ctl_table *ctl, int write, struct file *filp, void *buffer, \
		size_t *lenp
#define	AH_SYSCTL_ARGS		ctl, write, filp, buffer, lenp
#else /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,8) */
#define	AH_SYSCTL_ARGS_DECL \
	ctl_table *ctl, int write, struct file *filp, void *buffer,\
		size_t *lenp, loff_t *ppos
#define	AH_SYSCTL_ARGS		ctl, write, filp, buffer, lenp, ppos
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,8) */

static int
sysctl_hw_ath_hal_log(AH_SYSCTL_ARGS_DECL)
{
	int error, enable;

	ctl->data = &enable;
	ctl->maxlen = sizeof(enable);
	enable = (ath_hal_alq != NULL);
	error = proc_dointvec(AH_SYSCTL_ARGS);
	if (error || !write)
		return error;
	else
		return ath_hal_setlogging(enable);
}

static inline void
ath_hal_logvprintf(struct ath_hal *ah, const char *fmt, va_list ap)
{
	struct ale *ale;
	if (!ath_hal_alq) {
		ath_hal_alq_lost++;
		return;
	}

	ale = alq_get(ath_hal_alq, ALQ_NOWAIT);
	if (!ale) {
		ath_hal_alq_lost++;
		return;
	}

	memset(ale->ae_data, 0, MSG_MAXLEN);
	vsnprintf(ale->ae_data, MSG_MAXLEN, fmt, ap);
	alq_post(ath_hal_alq, ale);
}

void __ahdecl 
ath_hal_logprintf(struct ath_hal *ah, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	ath_hal_logvprintf(ah, fmt, ap);
	va_end(ap);
}
EXPORT_SYMBOL(ath_hal_logprintf);

#endif /* AH_DEBUG_ALQ */

static inline void
_hal_vprintf(struct ath_hal *ah, HAL_BOOL prefer_alq, const char *fmt, va_list ap)
{
	char buf[MSG_MAXLEN];
#ifdef AH_DEBUG_ALQ
	if (prefer_alq && ath_hal_alq) {
		ath_hal_logvprintf(ah, fmt, ap);
		return;
	}
#endif /* AH_DEBUG_ALQ */
	vsnprintf(buf, sizeof(buf), fmt, ap);
	printk("%s", buf);
}

void __ahdecl 
ath_hal_printf(struct ath_hal *ah, HAL_BOOL prefer_alq, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	_hal_vprintf(ah, prefer_alq, fmt, ap);
	va_end(ap);
}
EXPORT_SYMBOL(ath_hal_printf);

/* Lookup a friendly name for a register address (for any we have nicknames
 * for). Names were taken from openhal ar5212regs.h. Return AH_TRUE if the
 * name is a known ar5212 register, and AH_FALSE otherwise. */
HAL_BOOL
ath_hal_lookup_register_name(struct ath_hal *ah, char *buf, int buflen, 
		u_int32_t address) {
	const char *static_label = NULL;
	memset(buf, 0, buflen);
#if 0 /* Enable once for each new board/magic type */
	printk("MAGIC: %x\n", ah->ah_magic);
#endif
	if (ah->ah_magic == 0x19541014 /* AR5212 compatible HAL magic */ ) {
		/* Handle Static Register Labels (unique stuff we know about) */
		switch (address) {
		case 0x0008: static_label = "AR5K_CR"; break;
		case 0x000c: static_label = "AR5K_RXDP"; break;
		case 0x0014: static_label = "AR5K_CFG"; break;
		case 0x0024: static_label = "AR5K_IER"; break;
		case 0x0030: static_label = "AR5K_TXCFG"; break;
		case 0x0034: static_label = "AR5K_RXCFG"; break;
		case 0x0040: static_label = "AR5K_MIBC"; break;
		case 0x0044: static_label = "AR5K_TOPS"; break;
		case 0x0048: static_label = "AR5K_RXNOFRM"; break;
		case 0x004c: static_label = "AR5K_TXNOFRM"; break;
		case 0x0050: static_label = "AR5K_RPGTO"; break;
		case 0x0054: static_label = "AR5K_RFCNT"; break;
		case 0x0058: static_label = "AR5K_MISC"; break;
		case 0x0080: static_label = "AR5K_PISR"; break;
		case 0x0084: static_label = "AR5K_SISR0"; break;
		case 0x0088: static_label = "AR5K_SISR1"; break;
		case 0x008c: static_label = "AR5K_SISR2"; break;
		case 0x0090: static_label = "AR5K_SISR3"; break;
		case 0x0094: static_label = "AR5K_SISR4"; break;
		case 0x00a0: static_label = "AR5K_PIMR"; break;
		case 0x00a4: static_label = "AR5K_SIMR0"; break;
		case 0x00a8: static_label = "AR5K_SIMR1"; break;
		case 0x00ac: static_label = "AR5K_SIMR2"; break;
		case 0x00b0: static_label = "AR5K_SIMR3"; break;
		case 0x00b4: static_label = "AR5K_SIMR4"; break;
		case 0x00c0: static_label = "AR5K_RAC_PISR"; break;
		case 0x00c4: static_label = "AR5K_RAC_SISR0"; break;
		case 0x00c8: static_label = "AR5K_RAC_SISR1"; break;
		case 0x00cc: static_label = "AR5K_RAC_SISR2"; break;
		case 0x00d0: static_label = "AR5K_RAC_SISR3"; break;
		case 0x00d4: static_label = "AR5K_RAC_SISR4"; break;
		case 0x0400: static_label = "AR5K_DCM_ADDR"; break;
		case 0x0404: static_label = "AR5K_DCM_DATA"; break;
		case 0x0420: static_label = "AR5K_DCCFG"; break;
		case 0x0600: static_label = "AR5K_CCFG"; break;
		case 0x0604: static_label = "AR5K_CCFG_CUP"; break;
		case 0x0610: static_label = "AR5K_CPC0"; break;
		case 0x0614: static_label = "AR5K_CPC1"; break;
		case 0x0618: static_label = "AR5K_CPC2"; break;
		case 0x061c: static_label = "AR5K_CPC3"; break;
		case 0x0620: static_label = "AR5K_CPCORN"; break;
		case 0x0840: static_label = "AR5K_QCU_TXE"; break;
		case 0x0880: static_label = "AR5K_QCU_TXD"; break;
		case 0x0940: static_label = "AR5K_QCU_ONESHOTARM_SET"; break;
		case 0x0980: static_label = "AR5K_QCU_ONESHOTARM_CLEAR"; break;
		case 0xa200: static_label = "AR5K_PHY_MODE"; break;
		case 0x0a40: static_label = "AR5K_QCU_RDYTIMESHDN"; break;
		case 0x0b00: static_label = "AR5K_QCU_CBB_SELECT"; break;
		case 0x0b04: static_label = "AR5K_QCU_CBB_ADDR"; break;
		case 0x0b08: static_label = "AR5K_QCU_CBCFG"; break;
		case 0x1030: static_label = "AR5K_DCU_GBL_IFS_SIFS"; break;
		case 0x1038: static_label = "AR5K_DCU_TX_FILTER"; break;
		case 0x1070: static_label = "AR5K_DCU_GBL_IFS_SLOT"; break;
		case 0x10b0: static_label = "AR5K_DCU_GBL_IFS_EIFS"; break;
		case 0x10f0: static_label = "AR5K_DCU_GBL_IFS_MISC"; break;
		case 0x1230: static_label = "AR5K_DCU_FP"; break;
		case 0x1270: static_label = "AR5K_DCU_TXP"; break;
		case 0x143c: static_label = "AR5K_DCU_TX_FILTER_CLR"; break;
		case 0x147c: static_label = "AR5K_DCU_TX_FILTER_SET"; break;
		case 0x4000: static_label = "AR5K_RESET_CTL"; break;
		case 0x4004: static_label = "AR5K_SLEEP_CTL"; break;
		case 0x4008: static_label = "AR5K_INTPEND"; break;
		case 0x400c: static_label = "AR5K_SFR"; break;
		case 0x4010: static_label = "AR5K_PCICFG"; break;
		case 0x4014: static_label = "AR5K_GPIOCR"; break;
		case 0x4018: static_label = "AR5K_GPIODO"; break;
		case 0x401c: static_label = "AR5K_GPIODI"; break;
		case 0x4020: static_label = "AR5K_SREV"; break;
		case 0x6000: static_label = "AR5K_EEPROM_BASE"; break;
		case 0x6004: static_label = "AR5K_EEPROM_DATA_5211"; break;
		case 0x6008: static_label = "AR5K_EEPROM_CMD"; break;
		case 0x600c: static_label = "AR5K_EEPROM_STAT_5211"; break;
		case 0x6010: static_label = "AR5K_EEPROM_CFG"; break;
		case 0x8000: static_label = "AR5K_STA_ID0"; break;
		case 0x8004: static_label = "AR5K_STA_ID1"; break;
		case 0x8008: static_label = "AR5K_BSS_ID0"; break;
		case 0x800c: static_label = "AR5K_BSS_ID1"; break;
		case 0x8010: static_label = "AR5K_SLOT_TIME"; break;
		case 0x8014: static_label = "AR5K_TIME_OUT"; break;
		case 0x8018: static_label = "AR5K_RSSI_THR"; break;
		case 0x801c: static_label = "AR5K_USEC_5211"; break;
		case 0x8020: static_label = "AR5K_BEACON_5211"; break;
		case 0x8024: static_label = "AR5K_CFP_PERIOD_5211"; break;
		case 0x8028: static_label = "AR5K_TIMER0_5211"; break;
		case 0x802c: static_label = "AR5K_TIMER1_5211"; break;
		case 0x8030: static_label = "AR5K_TIMER2_5211"; break;
		case 0x8034: static_label = "AR5K_TIMER3_5211"; break;
		case 0x8038: static_label = "AR5K_CFP_DUR_5211"; break;
		case 0x803c: static_label = "AR5K_RX_FILTER_5211"; break;
		case 0x8040: static_label = "AR5K_MCAST_FILTER0_5211"; break;
		case 0x8044: static_label = "AR5K_MCAST_FILTER1_5211"; break;
		case 0x8048: static_label = "AR5K_DIAG_SW_5211"; break;
		case 0x804c: static_label = "AR5K_TSF_L32_5211"; break;
		case 0x8050: static_label = "AR5K_TSF_U32_5211"; break;
		case 0x8054: static_label = "AR5K_ADDAC_TEST"; break;
		case 0x8058: static_label = "AR5K_DEFAULT_ANTENNA"; break;
		case 0x8080: static_label = "AR5K_LAST_TSTP"; break;
		case 0x8084: static_label = "AR5K_NAV_5211"; break;
		case 0x8088: static_label = "AR5K_RTS_OK_5211"; break;
		case 0x808c: static_label = "AR5K_RTS_FAIL_5211"; break;
		case 0x8090: static_label = "AR5K_ACK_FAIL_5211"; break;
		case 0x8094: static_label = "AR5K_FCS_FAIL_5211"; break;
		case 0x8098: static_label = "AR5K_BEACON_CNT_5211"; break;
		case 0x80c0: static_label = "AR5K_XRMODE"; break;
		case 0x80c4: static_label = "AR5K_XRDELAY"; break;
		case 0x80c8: static_label = "AR5K_XRTIMEOUT"; break;
		case 0x80cc: static_label = "AR5K_XRCHIRP"; break;
		case 0x80d0: static_label = "AR5K_XRSTOMP"; break;
		case 0x80d4: static_label = "AR5K_SLEEP0"; break;
		case 0x80d8: static_label = "AR5K_SLEEP1"; break;
		case 0x80dc: static_label = "AR5K_SLEEP2"; break;
		case 0x80e0: static_label = "AR5K_BSS_IDM0"; break;
		case 0x80e4: static_label = "AR5K_BSS_IDM1"; break;
		case 0x80e8: static_label = "AR5K_TXPC"; break;
		case 0x80ec: static_label = "AR5K_PROFCNT_TX"; break;
		case 0x80f0: static_label = "AR5K_PROFCNT_RX"; break;
		case 0x80f4: static_label = "AR5K_PROFCNT_RXCLR"; break;
		case 0x80f8: static_label = "AR5K_PROFCNT_CYCLE"; break;
		case 0x8104: static_label = "AR5K_TSF_PARM"; break;
		case 0x810c: static_label = "AR5K_PHY_ERR_FIL"; break;
		case 0x9800: static_label = "AR5K_PHY"; break;
		case 0x9804: static_label = "AR5K_PHY_TURBO"; break;
		case 0x9808: static_label = "AR5K_PHY_AGC"; break;
		case 0x9814: static_label = "AR5K_PHY_TIMING_3"; break;
		case 0x9818: static_label = "AR5K_PHY_CHIP_ID"; break;
		case 0x981c: static_label = "AR5K_PHY_ACT"; break;
		case 0x9858: static_label = "AR5K_PHY_SIG"; break;
		case 0x985c: static_label = "AR5K_PHY_AGCCOARSE"; break;
		case 0x9860: static_label = "AR5K_PHY_AGCCTL"; break;
		case 0x9864: static_label = "AR5K_PHY_NF"; break;
		case 0x9870: static_label = "AR5K_PHY_SCR"; break;
		case 0x9874: static_label = "AR5K_PHY_SLMT"; break;
		case 0x9878: static_label = "AR5K_PHY_SCAL"; break;
		case 0x987c: static_label = "AR5K_PHY_PLL"; break;
		case 0x989c: static_label = "AR5K_RF_BUFFER"; break;
		case 0x98c0: static_label = "AR5K_RF_BUFFER_CONTROL_0"; break;
		case 0x98c4: static_label = "AR5K_RF_BUFFER_CONTROL_1"; break;
		case 0x98cc: static_label = "AR5K_RF_BUFFER_CONTROL_2"; break;
		case 0x98d0: static_label = "AR5K_RF_BUFFER_CONTROL_3"; break;
		case 0x98d4: static_label = "AR5K_RF_BUFFER_CONTROL_4"; break;
		case 0x98d8: static_label = "AR5K_RF_BUFFER_CONTROL_5"; break;
		case 0x98dc: static_label = "AR5K_RF_BUFFER_CONTROL_6"; break;
		case 0x9914: static_label = "AR5K_PHY_RX_DELAY"; break;
		case 0x9920: static_label = "AR5K_PHY_IQ"; break;
		case 0x9930: static_label = "AR5K_PHY_PAPD_PROBE"; break;
		case 0x9934: static_label = "AR5K_PHY_TXPOWER_RATE1"; break;
		case 0x9938: static_label = "AR5K_PHY_TXPOWER_RATE2"; break;
		case 0x993c: static_label = "AR5K_PHY_TXPOWER_RATE_MAX"; break;
		case 0x9944: static_label = "AR5K_PHY_FRAME_CTL_5211"; break;
		case 0x9954: static_label = "AR5K_PHY_RADAR"; break;
		case 0x99f0: static_label = "AR5K_PHY_SCLOCK"; break;
		case 0x99f4: static_label = "AR5K_PHY_SDELAY"; break;
		case 0x99f8: static_label = "AR5K_PHY_SPENDING"; break;
		case 0x9c10: static_label = "AR5K_PHY_IQRES_CAL_PWR_I"; break;
		case 0x9c14: static_label = "AR5K_PHY_IQRES_CAL_PWR_Q"; break;
		case 0x9c18: static_label = "AR5K_PHY_IQRES_CAL_CORR"; break;
		case 0x9c1c: static_label = "AR5K_PHY_CURRENT_RSSI"; break;
		case 0xa204: static_label = "AR5K_PHY_CCKTXCTL"; break;
		case 0xa20c: static_label = "AR5K_PHY_GAIN_2GHZ"; break;
		case 0xa234: static_label = "AR5K_PHY_TXPOWER_RATE3"; break;
		case 0xa238: static_label = "AR5K_PHY_TXPOWER_RATE4"; break;
		case 0x9850: static_label = "AR5K_PHY_AGCSIZE"; break;
		case 0x9924: static_label = "AR5K_PHY_SPUR"; break;
		default:
			break;
		}

		if (static_label) {
			if (strncmp(static_label, "ATH5K_", 6) == 0) 
				static_label += 6;
			snprintf(buf, buflen, "%s", static_label);
			return AH_TRUE;
		}

		/* Handle Key Table */
		if ((address >= 0x8800) && (address < 0x9800)) {
#define keytable_entry_reg_count (8)
#define keytable_entry_size      (keytable_entry_reg_count * sizeof(u_int32_t))
			int key = ((address - 0x8800) / keytable_entry_size);
			int reg = ((address - 0x8800) % keytable_entry_size) / 
				sizeof(u_int32_t);
			char *format = NULL;
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

		if (address >= 0x0800 && address <= 0x082c) {
			snprintf(buf, buflen, "AR5K_QUEUE_TXDP_%d",
				(u_int32_t)((address - 0x0800) / sizeof(u_int32_t)));
			return AH_TRUE;
		}
		if (address >= 0x08c0 && address <= 0x08ec) {
			snprintf(buf, buflen, "AR5K_QUEUE_CBRCFG_%d",
				(u_int32_t)((address - 0x08c0) / sizeof(u_int32_t)));
			return AH_TRUE;
		}
		if (address >= 0x0900 && address <= 0x092c) {
			snprintf(buf, buflen, "AR5K_QUEUE_RDYTIMECFG_%d",
				(u_int32_t)((address - 0x0900) / sizeof(u_int32_t)));
			return AH_TRUE;
		}
		if (address >= 0x09c0 && address <= 0x09ec) {
			snprintf(buf, buflen, "AR5K_QUEUE_MISC_%d",
				(u_int32_t)((address - 0x09c0) / sizeof(u_int32_t)));
			return AH_TRUE;
		}
		if (address >= 0x0a00 && address <= 0x0a2c) {
			snprintf(buf, buflen, "AR5K_QUEUE_STATUS_%d",
				(u_int32_t)((address - 0x0a00) / sizeof(u_int32_t)));
			return AH_TRUE;
		}
		if (address >= 0x1000 && address <= 0x102c) {
			snprintf(buf, buflen, "AR5K_QUEUE_QCUMASK_%d",
				(u_int32_t)((address - 0x1000) / sizeof(u_int32_t)));
			return AH_TRUE;
		}
		if (address >= 0x1040 && address <= 0x106c) {
			snprintf(buf, buflen, "AR5K_QUEUE_DFS_LOCAL_IFS_%d",
				(u_int32_t)((address - 0x1040) / sizeof(u_int32_t)));
			return AH_TRUE;
		}
		if (address >= 0x1080 && address <= 0x10ac) {
			snprintf(buf, buflen, "AR5K_QUEUE_DFS_RETRY_LIMIT_%d",
				(u_int32_t)((address - 0x1080) / sizeof(u_int32_t)));
			return AH_TRUE;
		}
		if (address >= 0x10c0 && address <= 0x10ec) {
			snprintf(buf, buflen, "AR5K_QUEUE_DFS_CHANNEL_TIME_%d",
				(u_int32_t)((address - 0x10c0) / sizeof(u_int32_t)));
			return AH_TRUE;
		}
		if (address >= 0x1100 && address <= 0x112c) {
			snprintf(buf, buflen, "AR5K_QUEUE_DFS_MISC_%d",
				(u_int32_t)((address - 0x1100) / sizeof(u_int32_t)));
			return AH_TRUE;
		}
		if (address >= 0x1140 && address <= 0x116c) {
			snprintf(buf, buflen, "AR5K_QUEUE_DFS_SEQNUM_%d",
				(u_int32_t)((address - 0x1140) / sizeof(u_int32_t)));
			return AH_TRUE;
		}
		if (address >= 0x4000 && address <= 0x5000) {
			snprintf(buf, buflen, "(PCI_TIMING)_%d",
				(u_int32_t)((address - 0x4000) / sizeof(u_int32_t)));
			return AH_TRUE;
		}
		if (address >= 0x8700 && address <= 0x877c) {
			snprintf(buf, buflen, "AR5K_RATE_DUR_%d",
				(u_int32_t)((address - 0x8700) / sizeof(u_int32_t)));
			return AH_TRUE;
		}
		if (address >= 0x8800 && address <= 0x9800) {
			snprintf(buf, buflen, "AR5K_KEYTABLE_0_5211_%d",
				(u_int32_t)((address - 0x8800) / sizeof(u_int32_t)));
			return AH_TRUE;
		}
		if (address >= 0x9a00 && address <= 0x9afc) {
			snprintf(buf, buflen, "AR5K_RF_GAIN_%d",
				(u_int32_t)((address - 0x9a00) / sizeof(u_int32_t)));
			return AH_TRUE;
		}
		if (address >= 0x9b00 && address <= 0x9bfc) {
			snprintf(buf, buflen, "AR5K_BB_GAIN_%d",
				(u_int32_t)((address - 0x9b00) / sizeof(u_int32_t)));
			return AH_TRUE;
		}
		if (address >= 0xa180 && address <= 0xa1fc) {
			snprintf(buf, buflen, "AR5K_PHY_PCDAC_TXPOWER_%d",
				(u_int32_t)((address - 0xa180) / sizeof(u_int32_t)));
			return AH_TRUE;
		}
	}

	/* Everything else... */
	snprintf(buf, buflen, "(unknown)");
	return AH_FALSE;
}
EXPORT_SYMBOL(ath_hal_lookup_register_name);

static void
_print_decoded_register_delta(struct ath_hal *ah, const char *device_name, 
			      HAL_BOOL prefer_alq, int regop,
			      u_int32_t address, u_int32_t v_old, u_int32_t v_new,
			      HAL_BOOL verbose)
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
	char name[64] = "";
	ath_hal_lookup_register_name(ah, name, sizeof(name), address);
	ath_hal_printf(ah, prefer_alq, 
		       "%s%s%s%23s:0x%04x:0x%08x:%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s"
			"%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s:%s\n",
		device_name ? device_name : "",
		device_name ? ":" : "",
	       (regop == REGOP_READ ? "R:" : 
		(regop == REGOP_WRITE ? "W:" : "")), 
		(0 < strlen(name)) ? name : "(unknown)",
		address,
		v_new,
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
 	        (ath_hal_func ?: "unknown")
		);
#undef BIT_UNCHANGED_ON
#undef BIT_UNCHANGED_OFF
#undef BIT_CHANGED_ON
#undef BIT_CHANGED_OFF
#undef NYBBLE_SEPARATOR
#undef BYTE_SEPARATOR
#undef BIT_STATUS
}

/* For any addresses we wish to get a symbolic representation of (i.e. flag
 * names) we can add it to this helper function and a subsequent line is
 * printed with the status in symbolic form. */
static void
_print_decoded_register_bitfields(struct ath_hal *ah, const char *device_name, 
				HAL_BOOL prefer_alq, int regop,
				u_int32_t address, u_int32_t old_v, 
				u_int32_t v, HAL_BOOL verbose)
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
	char name[128];
	ath_hal_lookup_register_name(ah, name, sizeof(name), address);

	if (address == AR5K_AR5212_PHY_ERR_FIL) {
		ath_hal_printf(ah, prefer_alq, 
			"%s%s%s%18s info:%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s"
			"%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s:%s\n",
		       device_name ? device_name : "",
		       device_name ? ":" : "",
		       (regop == REGOP_READ ? "R:" : 
			(regop == REGOP_WRITE ? "W:" : "")), 
			(0 < strlen(name)) ? name : "(unknown)",
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
		       (v & (1 <<  0)                ? " (1 <<  0)"     : ""),
		       (ath_hal_func ?: "unknown")
		      );
	}
	if (address == AR5K_AR5212_PISR || address == AR5K_AR5212_PIMR) {
		ath_hal_printf(ah, prefer_alq, 
			       "%s%s%s%18s info:%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s"
				"%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s:%s\n",
		       device_name ? device_name : "",
		       device_name ? ":" : "",
		       (regop == REGOP_READ ? "R:" : 
			(regop == REGOP_WRITE ? "W:" : "")), 
			(0 < strlen(name)) ? name : "(unknown)",
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
			(v & AR5K_AR5212_PIMR_QTRIG   ?  " QTRIG"          : ""),
 		        (ath_hal_func ?: "unknown")
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

/* Print out a single register name/address/value in hex and binary */
static inline void
_print_decoded_register(struct ath_hal *ah, const char *device_name, 
			HAL_BOOL prefer_alq, int regop,
			u_int32_t address, u_int32_t oldval, 
			u_int32_t newval, HAL_BOOL verbose)
{
	_print_decoded_register_delta(ah, device_name, prefer_alq, regop, 
				      address, oldval, newval, verbose);
	_print_decoded_register_bitfields(ah, device_name, prefer_alq, regop, 
					  address, oldval, newval, verbose);
}

/* Print out a single register name/address/value in hex and binary */
static inline void
_print_undecoded_register(struct ath_hal *ah, const char *device_name, 
			  HAL_BOOL prefer_alq, int regop,
			  u_int32_t address, u_int32_t newval)
{
	ath_hal_printf(ah, prefer_alq, "%s%s%s0x%05x = 0x%08x - %s\n", 
		device_name ? device_name : "",
		device_name ? ":" : "",
		(regop == REGOP_READ ? "R:" : 
		 (regop == REGOP_WRITE ? "W:" : "")), 
		address, 
		newval, 
		(ath_hal_func ?: "unknown")
		);
}

/* Print out a single register name/address/value in hex and binary */
void
ath_hal_print_decoded_register(struct ath_hal *ah, 
		       const char *device_name,
		       u_int32_t address, u_int32_t oldval, 
		       u_int32_t newval, HAL_BOOL bitfields)
{
	_print_decoded_register(ah, device_name, AH_FALSE /* don't use alq */, 
				REGOP_NONE, address, oldval, newval, bitfields);
}
EXPORT_SYMBOL(ath_hal_print_decoded_register);

/* Print out a single register in simple undecoded form */
void
ath_hal_print_register(struct ath_hal *ah, 
			       const char *device_name,
			       u_int32_t address, u_int32_t newval)
{
	_print_undecoded_register(ah, device_name, AH_FALSE /* don't use alq */, 
				REGOP_NONE, address, 
				newval);
}
EXPORT_SYMBOL(ath_hal_print_register);

static inline void 
_trace_regop(struct ath_hal *ah, int regop, u_int address, u_int32_t value)
{
#ifdef AH_DEBUG
	switch (ath_hal_debug) {
	case HAL_DEBUG_OFF:
		break;
	case HAL_DEBUG_REGOPS:
		/* XXX: Identify wifiX */
		_print_undecoded_register(ah, ath_hal_device, AH_TRUE /* prefer alq */,
					regop, address, 
					value);
		break;
	default:
		/* XXX: Identify wifiX */
		_print_decoded_register(ah, ath_hal_device, AH_TRUE /* prefer alq */,
					regop, address, 
					((regop == REGOP_WRITE && ath_hal_debug >= HAL_DEBUG_REGOPS_DELTAS) ? 
						_OS_REG_READ(ah, address) : 
						value), 
					value, 
					(ath_hal_debug >= HAL_DEBUG_REGOPS_BITFIELDS));
		break;
	}

#endif /* AH_DEBUG */
}

#if defined(AH_DEBUG) || defined(AH_REGOPS_FUNC) || defined(AH_DEBUG_ALQ)
/*
 * Memory-mapped device register read/write.  These are here
 * as routines when debugging support is enabled and/or when
 * explicitly configured to use function calls.  The latter is
 * for architectures that might need to do something before
 * referencing memory (e.g. remap an i/o window).
 *
 * This should only be called while holding the lock, sc->sc_hal_lock.
 *
 * NB: see the comments in ah_osdep.h about byte-swapping register
 *     reads and writes to understand what's going on below.
 */
void __ahdecl
ath_hal_reg_write(struct ath_hal *ah, u_int address, u_int32_t value)
{
	_trace_regop(ah, REGOP_WRITE, address, value);
	_OS_REG_WRITE(ah, address, value);
}
EXPORT_SYMBOL(ath_hal_reg_write);

/* This should only be called while holding the lock, sc->sc_hal_lock. */
u_int32_t __ahdecl
ath_hal_reg_read(struct ath_hal *ah, u_int address)
{
 	u_int32_t val = _OS_REG_READ(ah, address);
	_trace_regop(ah, REGOP_READ, address, val);
	return val;
}
EXPORT_SYMBOL(ath_hal_reg_read);
#endif

/*
 * Delay n microseconds.
 */
void __ahdecl
ath_hal_delay(int n)
{
	udelay(n);
}

/*
 * Allocate/free memory.
 */

void * __ahdecl
ath_hal_malloc(size_t size)
{
	return kzalloc(size, GFP_KERNEL);
}

void __ahdecl
ath_hal_free(void *p)
{
	kfree(p);
}

void __ahdecl
ath_hal_memzero(void *dst, size_t n)
{
	memset(dst, 0, n);
}
EXPORT_SYMBOL(ath_hal_memzero);

void * __ahdecl
ath_hal_memcpy(void *dst, const void *src, size_t n)
{
	return memcpy(dst, src, n);
}
EXPORT_SYMBOL(ath_hal_memcpy);

int __ahdecl
ath_hal_memcmp(const void *a, const void *b, size_t n)
{
	return memcmp(a, b, n);
}
EXPORT_SYMBOL(ath_hal_memcmp);

static ctl_table ath_hal_sysctls[] = {
#ifdef AH_DEBUG
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "debug",
	  .mode		= 0644,
	  .data		= &ath_hal_debug,
	  .maxlen	= sizeof(ath_hal_debug),
	  .proc_handler	= proc_dointvec
	},
#endif /* AH_DEBUG */
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "dma_beacon_response_time",
	  .data		= &ath_hal_dma_beacon_response_time,
	  .maxlen	= sizeof(ath_hal_dma_beacon_response_time),
	  .mode		= 0644,
	  .proc_handler	= proc_dointvec
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "sw_beacon_response_time",
	  .mode		= 0644,
	  .data		= &ath_hal_sw_beacon_response_time,
	  .maxlen	= sizeof(ath_hal_sw_beacon_response_time),
	  .proc_handler	= proc_dointvec
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "swba_backoff",
	  .mode		= 0644,
	  .data		= &ath_hal_additional_swba_backoff,
	  .maxlen	= sizeof(ath_hal_additional_swba_backoff),
	  .proc_handler	= proc_dointvec
	},
#ifdef AH_DEBUG_ALQ
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "alq",
	  .mode		= 0644,
	  .proc_handler	= sysctl_hw_ath_hal_log
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "alq_size",
	  .mode		= 0644,
	  .data		= &ath_hal_alq_qsize,
	  .maxlen	= sizeof(ath_hal_alq_qsize),
	  .proc_handler	= proc_dointvec
	},
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "alq_lost",
	  .mode		= 0644,
	  .data		= &ath_hal_alq_lost,
	  .maxlen	= sizeof(ath_hal_alq_lost),
	  .proc_handler	= proc_dointvec
	},
#endif /* AH_DEBUG_ALQ */
	{ 0 }
};
static ctl_table ath_hal_table[] = {
	{ .ctl_name	= CTL_AUTO,
	  .procname	= "hal",
	  .mode		= 0555,
	  .child	= ath_hal_sysctls
	}, { 0 }
};
static ctl_table ath_ath_table[] = {
	{ .ctl_name	= DEV_ATH,
	  .procname	= "ath",
	  .mode		= 0555,
	  .child	= ath_hal_table
	}, { 0 }
};
static ctl_table ath_root_table[] = {
	{ .ctl_name	= CTL_DEV,
	  .procname	= "dev",
	  .mode		= 0555,
	  .child	= ath_ath_table
	}, { 0 }
};
static struct ctl_table_header *ath_hal_sysctl_header;

static void
ath_hal_sysctl_register(void)
{
	static int initialized = 0;

	if (!initialized) {
		ath_hal_sysctl_header =
			ATH_REGISTER_SYSCTL_TABLE(ath_root_table);
		initialized = 1;
	}
}

static void
ath_hal_sysctl_unregister(void)
{
	if (ath_hal_sysctl_header)
		unregister_sysctl_table(ath_hal_sysctl_header);
}

/*
 * Module glue.
 */
#include "version.h"
#if 0
static char *dev_info = "ath_hal";
#endif

MODULE_AUTHOR("Errno Consulting, Sam Leffler");
MODULE_DESCRIPTION("Atheros Hardware Access Layer (HAL)");
MODULE_SUPPORTED_DEVICE("Atheros WLAN devices");
#ifdef MODULE_VERSION
MODULE_VERSION(TARGET ": " ATH_HAL_VERSION);
#endif
#ifdef MODULE_LICENSE
MODULE_LICENSE("Proprietary");
#endif

EXPORT_SYMBOL(ath_hal_probe);
EXPORT_SYMBOL(_ath_hal_attach);
EXPORT_SYMBOL(_ath_hal_detach);
EXPORT_SYMBOL(ath_hal_init_channels);
EXPORT_SYMBOL(ath_hal_getwirelessmodes);
EXPORT_SYMBOL(ath_hal_computetxtime);
EXPORT_SYMBOL(ath_hal_mhz2ieee);
EXPORT_SYMBOL(ath_hal_process_noisefloor);

#ifdef MMIOTRACE
extern void (*kmmio_logmsg)(struct ath_hal *ah, u8 write, u_int address, u_int32_t val);

void _trace_regop(struct ath_hal *ah, int regop, u_int address, u_int32_t newval);
static void _kmmio_logmsg(struct ath_hal *ah, u8 write, u_int address, u_int32_t val) {
	_trace_regop(ah, write ? REGOP_WRITE : REGOP_READ, address, val);
}
#endif /* MMIOTRACE */


static int __init
init_ath_hal(void)
{
	const char *sep;
	int i;
#ifdef MMIOTRACE
	kmmio_logmsg = _kmmio_logmsg;
#endif

	sep = "";
	for (i = 0; ath_hal_buildopts[i] != NULL; i++) {
		printk("%s%s", sep, ath_hal_buildopts[i]);
		sep = ", ";
	}
	printk(")\n");
	ath_hal_sysctl_register();
	return (0);
}
module_init(init_ath_hal);

static void __exit
exit_ath_hal(void)
{
#ifdef MMIOTRACE
	kmmio_logmsg = NULL;
#endif
	ath_hal_sysctl_unregister();
}
module_exit(exit_ath_hal);
