/* -*- linux-c -*- */
/*-
 * Copyright (c) 2007 Nick Kossifidis <mickflemm@gmail.com>
 * Copyright (c) 2007 Joerg Albert    <jal2 *at* gmx.de>
 *
 * This program is free software you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY, without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* So here is how it works:
 *
 * First compile...
 *
 * gcc ath_info.c -o ath_info
 *
 * then find card's physical address
 *
 * lspci -v
 *
 * 02:02.0 Ethernet controller: Atheros Communications, Inc. AR5212 802.11abg NIC (rev 01)
 *         Subsystem: Fujitsu Limited. Unknown device 1234
 *         Flags: bus master, medium devsel, latency 168, IRQ 23
 *         Memory at c2000000 (32-bit, non-prefetchable) [size=64K]
 *         Capabilities: [44] Power Management version 2
 *
 * address here is 0xc2000000
 *
 * load madwifi-ng or madwifi-old if not already loaded (be sure the
 * interface is down!)
 *
 * modprobe ath_pci
 *
 * OR
 *
 * call:
 * setpci -s 02:02.0 command=0x41f cache_line_size=0x10
 *
 * to enable access to the PCI device.
 *
 * and we run the thing...
 *
 * ./ath_info 0xc2000000
 *
 * In order to change the regdomain to 0, call:
 *
 * ./ath_info -w 0xc2000000 regdomain 0
 *
 * to change any PCI ID value, say:
 *
 * ./ath_info -w 0xc2000000 <name> X
 *
 * with <name> ::= pci_dev_id | pci_vendor_id | pci_class |
 *                 pci_subsys_dev_id | pci_subsys_vendor_id
 *
 * With newer chipsets (>= AR5004x, i.e. MAC >= AR5213), Atheros introduced
 * write protection on the EEPROM. On a GIGABYTE GN-WI01HT you can set GPIO 4
 * to low to be able to write the EEPROM. This depends highly on the PCB layout,
 * so there may be different GPIO used.
 * This program currently sets GPIO 4 to low for a MAC >= AR5213, but you can
 * override this with the -g option:
 *
 * ./ath_info -g 5:0 -w 0xc2000000 regdomain X
 *
 * would set GPIO 5 to low (and wouldn't touch GPIO 4). -g can be given several times.
 *
 * The write function is currently not tested with 5210 devices.
 *
 * Use at your own risk, entering a false device address will have really
 * nasty results!
 *
 * Writing wrong values to the PCI id fields may prevent the driver from
 * detecting the card!
 *
 * Transmitting on illegal frequencies may violate state laws. Stick to the local
 * regulations!
 *
 * DISCLAIMER:
 * The authors are in no case responsible for damaged hardware or violation of
 * local laws by operating modified hardware.
 *
 */

/* Try accepting 64-bit device address even with 32-bit userspace */
#define _FILE_OFFSET_BITS 64

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <endian.h>
#include <byteswap.h>

#undef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define dbg(fmt, __args__...) \
do { \
	if (verbose) \
		printf("#DBG %s: " fmt "\n", __FUNCTION__, ##__args__); \
 } while (0)

#define err(fmt, __args__...) \
fprintf(stderr, "#ERR %s: " fmt "\n", __FUNCTION__, ##__args__)

#define AR5K_PCI_MEM_SIZE 0x10000

#define AR5K_NUM_GPIO	6

#define AR5K_GPIOCR		0x4014	/* Register Address */
#define AR5K_GPIOCR_OUT(n)	(3 << ((n) * 2))	/* Mode 3 for pin n */
#define AR5K_GPIOCR_INT_SEL(n)	((n) << 12)	/* Interrupt for GPIO pin n */

/*
 * GPIO (General Purpose Input/Output) data output register
 */
#define AR5K_GPIODO	0x4018

/*
 * GPIO (General Purpose Input/Output) data input register
 */
#define AR5K_GPIODI	0x401c

/*
 * Common silicon revision/version values
 */
enum ath5k_srev_type {
	AR5K_VERSION_VER,
	AR5K_VERSION_REV,
	AR5K_VERSION_RAD,
};

struct ath5k_srev_name {
	const char *sr_name;
	enum ath5k_srev_type sr_type;
	u_int sr_val;
};

#define AR5K_SREV_UNKNOWN	0xffff

/* Known MAC revision numbers */
#define AR5K_SREV_VER_AR5210	0x00
#define AR5K_SREV_VER_AR5311	0x10
#define AR5K_SREV_VER_AR5311A	0x20
#define AR5K_SREV_VER_AR5311B	0x30
#define AR5K_SREV_VER_AR5211	0x40
#define AR5K_SREV_VER_AR5212	0x50
#define AR5K_SREV_VER_AR5213	0x55
#define AR5K_SREV_VER_AR5213A	0x59
#define	AR5K_SREV_VER_AR2424	0xa0
#define	AR5K_SREV_VER_AR5424	0xa3
#define	AR5K_SREV_VER_AR5413	0xa4
#define AR5K_SREV_VER_AR5414	0xa5
#define	AR5K_SREV_VER_AR5416	0xc0
#define	AR5K_SREV_VER_AR5418	0xca
#define	AR5K_SREV_VER_AR2425	0xe0

/* Known PHY revision nymbers */
#define AR5K_SREV_RAD_5110	0x00
#define AR5K_SREV_RAD_5111	0x10
#define AR5K_SREV_RAD_5111A	0x15
#define AR5K_SREV_RAD_2111	0x20
#define AR5K_SREV_RAD_5112	0x30
#define AR5K_SREV_RAD_5112A	0x35
#define AR5K_SREV_RAD_2112	0x40
#define AR5K_SREV_RAD_2112A	0x45
#define AR5K_SREV_RAD_SC1	0x63	/* Found on 5413/5414 */
#define	AR5K_SREV_RAD_SC2	0xa2	/* Found on 2424/5424 */
#define	AR5K_SREV_RAD_5133	0xc0	/* MIMO found on 5418 */

static const struct ath5k_srev_name ath5k_srev_names[] = {
	{"5210", AR5K_VERSION_VER, AR5K_SREV_VER_AR5210},
	{"5311", AR5K_VERSION_VER, AR5K_SREV_VER_AR5311},
	{"5311A", AR5K_VERSION_VER, AR5K_SREV_VER_AR5311A},
	{"5311B", AR5K_VERSION_VER, AR5K_SREV_VER_AR5311B},
	{"5211", AR5K_VERSION_VER, AR5K_SREV_VER_AR5211},
	{"5212", AR5K_VERSION_VER, AR5K_SREV_VER_AR5212},
	{"5213", AR5K_VERSION_VER, AR5K_SREV_VER_AR5213},
	{"5213A", AR5K_VERSION_VER, AR5K_SREV_VER_AR5213A},
	{"2424", AR5K_VERSION_VER, AR5K_SREV_VER_AR2424},
	{"5424", AR5K_VERSION_VER, AR5K_SREV_VER_AR5424},
	{"5413", AR5K_VERSION_VER, AR5K_SREV_VER_AR5413},
	{"5414", AR5K_VERSION_VER, AR5K_SREV_VER_AR5414},
	{"5416", AR5K_VERSION_VER, AR5K_SREV_VER_AR5416},
	{"5418", AR5K_VERSION_VER, AR5K_SREV_VER_AR5418},
	{"2425", AR5K_VERSION_VER, AR5K_SREV_VER_AR2425},
	{"xxxxx", AR5K_VERSION_VER, AR5K_SREV_UNKNOWN},
	{"5110", AR5K_VERSION_RAD, AR5K_SREV_RAD_5110},
	{"5111", AR5K_VERSION_RAD, AR5K_SREV_RAD_5111},
	{"2111", AR5K_VERSION_RAD, AR5K_SREV_RAD_2111},
	{"5112", AR5K_VERSION_RAD, AR5K_SREV_RAD_5112},
	{"5112a", AR5K_VERSION_RAD, AR5K_SREV_RAD_5112A},
	{"2112", AR5K_VERSION_RAD, AR5K_SREV_RAD_2112},
	{"2112a", AR5K_VERSION_RAD, AR5K_SREV_RAD_2112A},
	{"SChip", AR5K_VERSION_RAD, AR5K_SREV_RAD_SC1},
	{"SChip", AR5K_VERSION_RAD, AR5K_SREV_RAD_SC2},
	{"5133", AR5K_VERSION_RAD, AR5K_SREV_RAD_5133},
	{"xxxxx", AR5K_VERSION_RAD, AR5K_SREV_UNKNOWN},
};

/*
 * Silicon revision register
 */
#define AR5K_SREV		0x4020	/* Register Address */
#define AR5K_SREV_REV		0x0000000f	/* Mask for revision */
#define AR5K_SREV_REV_S		0
#define AR5K_SREV_VER		0x000000ff	/* Mask for version */
#define AR5K_SREV_VER_S		4

/*
 * PHY chip revision register
 */
#define	AR5K_PHY_CHIP_ID		0x9818

/*
 * PHY register
 */
#define	AR5K_PHY_BASE			0x9800
#define	AR5K_PHY(_n)			(AR5K_PHY_BASE + ((_n) << 2))
#define AR5K_PHY_SHIFT_2GHZ		0x00004007
#define AR5K_PHY_SHIFT_5GHZ		0x00000007

#define AR5K_RESET_CTL		0x4000	/* Register Address */
#define AR5K_RESET_CTL_PCU	0x00000001	/* Protocol Control Unit reset */
#define AR5K_RESET_CTL_DMA	0x00000002	/* DMA (Rx/Tx) reset -5210 only */
#define	AR5K_RESET_CTL_BASEBAND	0x00000002	/* Baseband reset (5211/5212) */
#define AR5K_RESET_CTL_MAC	0x00000004	/* MAC reset (PCU+Baseband?) -5210 only */
#define AR5K_RESET_CTL_PHY	0x00000008	/* PHY reset -5210 only */
#define AR5K_RESET_CTL_PCI	0x00000010	/* PCI Core reset (interrupts etc) */
#define AR5K_RESET_CTL_CHIP	(AR5K_RESET_CTL_PCU | AR5K_RESET_CTL_DMA |	\
				AR5K_RESET_CTL_MAC | AR5K_RESET_CTL_PHY)

/*
 * Sleep control register
 */
#define AR5K_SLEEP_CTL			0x4004	/* Register Address */
#define AR5K_SLEEP_CTL_SLDUR		0x0000ffff	/* Sleep duration mask */
#define AR5K_SLEEP_CTL_SLDUR_S		0
#define AR5K_SLEEP_CTL_SLE		0x00030000	/* Sleep enable mask */
#define AR5K_SLEEP_CTL_SLE_S		16
#define AR5K_SLEEP_CTL_SLE_WAKE		0x00000000	/* Force chip awake */
#define AR5K_SLEEP_CTL_SLE_SLP		0x00010000	/* Force chip sleep */
#define AR5K_SLEEP_CTL_SLE_ALLOW	0x00020000
#define AR5K_SLEEP_CTL_SLE_UNITS	0x00000008	/* not on 5210 */

#define AR5K_PCICFG			0x4010	/* Register Address */
#define AR5K_PCICFG_EEAE		0x00000001	/* Eeprom access enable [5210] */
#define AR5K_PCICFG_CLKRUNEN		0x00000004	/* CLKRUN enable [5211+] */
#define AR5K_PCICFG_EESIZE		0x00000018	/* Mask for EEPROM size [5211+] */
#define AR5K_PCICFG_EESIZE_S		3
#define AR5K_PCICFG_EESIZE_4K		0	/* 4K */
#define AR5K_PCICFG_EESIZE_8K		1	/* 8K */
#define AR5K_PCICFG_EESIZE_16K		2	/* 16K */
#define AR5K_PCICFG_EESIZE_FAIL		3	/* Failed to get size (?) [5211+] */

#define AR5K_PCICFG_SPWR_DN		0x00010000	/* Mask for power status (5210) */

#define AR5K_EEPROM_BASE		0x6000

#define AR5K_EEPROM_MAGIC		0x003d	/* Offset for EEPROM Magic number */
#define AR5K_EEPROM_MAGIC_VALUE		0x5aa5	/* Default - found on EEPROM */
#define AR5K_EEPROM_MAGIC_5212		0x0000145c	/* 5212 */
#define AR5K_EEPROM_MAGIC_5211		0x0000145b	/* 5211 */
#define AR5K_EEPROM_MAGIC_5210		0x0000145a	/* 5210 */

/*
 * EEPROM data register
 */
#define AR5K_EEPROM_DATA_5211	0x6004
#define AR5K_EEPROM_DATA_5210	0x6800
#define	AR5K_EEPROM_DATA	(mac_version == AR5K_SREV_VER_AR5210 ? \
				AR5K_EEPROM_DATA_5210 : AR5K_EEPROM_DATA_5211)

/*
 * EEPROM command register
 */
#define AR5K_EEPROM_CMD		0x6008	/* Register Addres */
#define AR5K_EEPROM_CMD_READ	0x00000001	/* EEPROM read */
#define AR5K_EEPROM_CMD_WRITE	0x00000002	/* EEPROM write */
#define AR5K_EEPROM_CMD_RESET	0x00000004	/* EEPROM reset */

/*
 * EEPROM status register
 */
#define AR5K_EEPROM_STAT_5210	0x6c00	/* Register Address [5210] */
#define AR5K_EEPROM_STAT_5211	0x600c	/* Register Address [5211+] */
#define	AR5K_EEPROM_STATUS	(mac_version == AR5K_SREV_VER_AR5210 ? \
				AR5K_EEPROM_STAT_5210 : AR5K_EEPROM_STAT_5211)
#define AR5K_EEPROM_STAT_RDERR	0x00000001	/* EEPROM read failed */
#define AR5K_EEPROM_STAT_RDDONE	0x00000002	/* EEPROM read successful */
#define AR5K_EEPROM_STAT_WRERR	0x00000004	/* EEPROM write failed */
#define AR5K_EEPROM_STAT_WRDONE	0x00000008	/* EEPROM write successful */

#define AR5K_EEPROM_REG_DOMAIN		0x00bf	/* Offset for EEPROM regulatory domain */
#define AR5K_EEPROM_INFO_BASE		0x00c0	/* Offset for EEPROM header */
#define AR5K_EEPROM_INFO_MAX		(0x400 - AR5K_EEPROM_INFO_BASE)
#define AR5K_EEPROM_INFO_CKSUM		0xffff
#define AR5K_EEPROM_INFO(_n)		(AR5K_EEPROM_INFO_BASE + (_n))
#define AR5K_EEPROM_MODE_11A		0
#define AR5K_EEPROM_MODE_11B		1
#define AR5K_EEPROM_MODE_11G		2

#define AR5K_EEPROM_VERSION		AR5K_EEPROM_INFO(1)

#define AR5K_EEPROM_HDR			AR5K_EEPROM_INFO(2)	/* Header that contains the device caps */
#define AR5K_EEPROM_HDR_11A(_v)		(((_v) >> AR5K_EEPROM_MODE_11A) & 0x1)	/* Device has a support */
#define AR5K_EEPROM_HDR_11B(_v)		(((_v) >> AR5K_EEPROM_MODE_11B) & 0x1)	/* Device has b support */
#define AR5K_EEPROM_HDR_11G(_v)		(((_v) >> AR5K_EEPROM_MODE_11G) & 0x1)	/* Device has g support */
#define AR5K_EEPROM_HDR_T_2GHZ_DIS(_v)	(((_v) >> 3) & 0x1)	/* Disable turbo for 2Ghz (?) */
#define AR5K_EEPROM_HDR_T_5GHZ_DBM(_v)	(((_v) >> 4) & 0x7f)	/* Max turbo power for a/XR mode (eeprom_init) */
#define AR5K_EEPROM_HDR_DEVICE(_v)	(((_v) >> 11) & 0x7)
#define AR5K_EEPROM_HDR_T_5GHZ_DIS(_v)	(((_v) >> 15) & 0x1)	/* Disable turbo for 5Ghz (?) */
#define AR5K_EEPROM_HDR_RFKILL(_v)	(((_v) >> 14) & 0x1)	/* Device has RFKill support */

/* Misc values available since EEPROM 4.0 */
#define AR5K_EEPROM_MISC0		0x00c4
#define AR5K_EEPROM_EARSTART(_v)	((_v) & 0xfff)
#define AR5K_EEPROM_EEMAP(_v)		(((_v) >> 14) & 0x3)
#define AR5K_EEPROM_MISC1		0x00c5
#define AR5K_EEPROM_TARGET_PWRSTART(_v)	((_v) & 0xfff)
#define AR5K_EEPROM_HAS32KHZCRYSTAL(_v)	(((_v) >> 14) & 0x1)

/*
 * Read data by masking
 */
#define AR5K_REG_MS(_val, _flags)	\
	(((_val) & (_flags)) >> _flags##_S)

/*
 * Access device registers
 */
#if __BYTE_ORDER == __BIG_ENDIAN
#define AR5K_REG_READ(_reg)		\
	__bswap_32(*((volatile u_int32_t *)(mem + (_reg))))
#define AR5K_REG_WRITE(_reg, _val)	\
	(*((volatile u_int32_t *)(mem + (_reg))) = __bswap_32(_val))
#else
#define AR5K_REG_READ(_reg)		\
	(*((volatile u_int32_t *)(mem + (_reg))))
#define AR5K_REG_WRITE(_reg, _val)	\
	(*((volatile u_int32_t *)(mem + (_reg))) = (_val))
#endif


#define AR5K_REG_ENABLE_BITS(_reg, _flags)	\
	AR5K_REG_WRITE(_reg, AR5K_REG_READ(_reg) | (_flags))

#define AR5K_REG_DISABLE_BITS(_reg, _flags)	\
	AR5K_REG_WRITE(_reg, AR5K_REG_READ(_reg) & ~(_flags))

#define AR5K_TUNE_REGISTER_TIMEOUT		20000

/* names for eeprom fields */
struct eeprom_entry {
	const char *name;
	int addr;
};

static const struct eeprom_entry eeprom_addr[] = {
	{"pci_dev_id", 0},
	{"pci_vendor_id", 1},
	{"pci_class", 2},
	{"pci_rev_id", 3},
	{"pci_subsys_dev_id", 7},
	{"pci_subsys_vendor_id", 8},
	{"regdomain", AR5K_EEPROM_REG_DOMAIN},
};

static const int eeprom_addr_len = sizeof(eeprom_addr) / sizeof(eeprom_addr[0]);

static int force_write = 0;
static int verbose = 0;

/* forward decl. */
static void usage(const char *n);

static u_int32_t ath5k_hw_bitswap(u_int32_t val, u_int bits)
{
	u_int32_t retval = 0, bit, i;

	for (i = 0; i < bits; i++) {
		bit = (val >> i) & 1;
		retval = (retval << 1) | bit;
	}

	return (retval);
}

/*
 * Get the PHY Chip revision
 */
static u_int16_t
ath5k_hw_radio_revision(u_int16_t mac_version, void *mem, u_int8_t chip)
{
	int i;
	u_int32_t srev;
	u_int16_t ret;

	/*
	 * Set the radio chip access register
	 */
	switch (chip) {
	case 0:
		AR5K_REG_WRITE(AR5K_PHY(0), AR5K_PHY_SHIFT_2GHZ);
		break;
	case 1:
		AR5K_REG_WRITE(AR5K_PHY(0), AR5K_PHY_SHIFT_5GHZ);
		break;
	default:
		return (0);
	}

	usleep(2000);

	/* ...wait until PHY is ready and read the selected radio revision */
	AR5K_REG_WRITE(AR5K_PHY(0x34), 0x00001c16);

	for (i = 0; i < 8; i++)
		AR5K_REG_WRITE(AR5K_PHY(0x20), 0x00010000);

	if (mac_version == AR5K_SREV_VER_AR5210) {
		srev = AR5K_REG_READ(AR5K_PHY(256) >> 28) & 0xf;

		ret = (u_int16_t)ath5k_hw_bitswap(srev, 4) + 1;
	} else {
		srev = (AR5K_REG_READ(AR5K_PHY(0x100)) >> 24) & 0xff;

		ret = (u_int16_t)ath5k_hw_bitswap(((srev & 0xf0) >> 4) |
						  ((srev & 0x0f) << 4), 8);
	}

	/* Reset to the 5GHz mode */
	AR5K_REG_WRITE(AR5K_PHY(0), AR5K_PHY_SHIFT_5GHZ);

	return (ret);
}

/*
 * Write to EEPROM
 */
static int
ath5k_hw_eeprom_write(void *mem, u_int32_t offset, u_int16_t data,
		      u_int8_t mac_version)
{
	u_int32_t status, timeout;

	/*
	 * Initialize EEPROM access
	 */

	if (mac_version == AR5K_SREV_VER_AR5210) {

		AR5K_REG_ENABLE_BITS(AR5K_PCICFG, AR5K_PCICFG_EEAE);

		/* data to write */
		(void)AR5K_REG_WRITE(AR5K_EEPROM_BASE + (4 * offset), data);

	} else {
		/* not 5210 */
		/* reset eeprom access */
		AR5K_REG_WRITE(AR5K_EEPROM_CMD, AR5K_EEPROM_CMD_RESET);
		usleep(5);

		AR5K_REG_WRITE(AR5K_EEPROM_DATA, data);

		/* set offset in EEPROM to write to */
		AR5K_REG_WRITE(AR5K_EEPROM_BASE, offset);
		usleep(5);

		/* issue write command */
		AR5K_REG_WRITE(AR5K_EEPROM_CMD, AR5K_EEPROM_CMD_WRITE);
	}

	for (timeout = AR5K_TUNE_REGISTER_TIMEOUT; timeout > 0; timeout--) {
		status = AR5K_REG_READ(AR5K_EEPROM_STATUS);
		if (status & AR5K_EEPROM_STAT_WRDONE) {
			if (status & AR5K_EEPROM_STAT_WRERR) {
				err("eeprom write access to 0x%04x failed",
				    offset);
				return 1;
			}
			return 0;
		}
		usleep(15);
	}

	return 1;
}

/*
 * Read from EEPROM
 */
static int
ath5k_hw_eeprom_read(void *mem, u_int32_t offset, u_int16_t *data,
		     u_int8_t mac_version)
{
	u_int32_t status, timeout;

	/*
	 * Initialize EEPROM access
	 */
	if (mac_version == AR5K_SREV_VER_AR5210) {
		AR5K_REG_ENABLE_BITS(AR5K_PCICFG, AR5K_PCICFG_EEAE);
		(void)AR5K_REG_READ(AR5K_EEPROM_BASE + (4 * offset));
	} else {
		AR5K_REG_WRITE(AR5K_EEPROM_BASE, offset);
		AR5K_REG_ENABLE_BITS(AR5K_EEPROM_CMD, AR5K_EEPROM_CMD_READ);
	}

	for (timeout = AR5K_TUNE_REGISTER_TIMEOUT; timeout > 0; timeout--) {
		status = AR5K_REG_READ(AR5K_EEPROM_STATUS);
		if (status & AR5K_EEPROM_STAT_RDDONE) {
			if (status & AR5K_EEPROM_STAT_RDERR)
				return 1;
			*data = (u_int16_t)
			    (AR5K_REG_READ(AR5K_EEPROM_DATA) & 0xffff);
			return (0);
		}
		usleep(15);
	}

	return 1;
}

static const char *ath5k_hw_get_part_name(enum ath5k_srev_type type,
					  u_int32_t val)
{
	const char *name = "xxxxx";
	int i;

	for (i = 0; i < ARRAY_SIZE(ath5k_srev_names); i++) {
		if (ath5k_srev_names[i].sr_type != type ||
		    ath5k_srev_names[i].sr_val == AR5K_SREV_UNKNOWN)
			continue;
		if ((val & 0xff) < ath5k_srev_names[i + 1].sr_val) {
			name = ath5k_srev_names[i].sr_name;
			break;
		}
	}

	return (name);
}

/* returns -1 on unknown name */
static int eeprom_name2addr(const char *name)
{
	int i;
	if (!name || !name[0])
		return -1;
	for (i = 0; i < eeprom_addr_len; i++)
		if (!strcmp(name, eeprom_addr[i].name))
			return eeprom_addr[i].addr;
	return -1;
}				/* eeprom_name2addr */

/* returns "<unknown>" on unknown address */
static const char *eeprom_addr2name(int addr)
{
	int i;
	for (i = 0; i < eeprom_addr_len; i++)
		if (eeprom_addr[i].addr == addr)
			return eeprom_addr[i].name;
	return "<unknown>";
}				/* eeprom_addr2name */

static int
do_write_pairs(int anr, int argc, char **argv, unsigned char *mem,
	       int mac_version)
{
#define MAX_NR_WRITES 16
	struct {
		int addr;
		unsigned int val;
	} wr_ops[MAX_NR_WRITES];
	int wr_ops_len = 0;
	int i;
	char *end;
	int errors = 0;		/* count errors during write/verify */

	if (anr >= argc) {
		err("missing values to write.");
		usage(argv[0]);
		return 1;
	}

	if ((argc - anr) % 2) {
		err("write spec. needs an even number of arguments.");
		usage(argv[0]);
		return 2;
	}

	if ((argc - anr) / 2 > MAX_NR_WRITES) {
		err("too many values to write (max. %d)", MAX_NR_WRITES);
		return 3;
	}

	/* get the (addr,val) pairs we have to write */
	i = 0;
	while (anr < (argc - 1)) {
		wr_ops[i].addr = strtoul(argv[anr], &end, 16);
		if (end == argv[anr]) {
			/* maybe a symbolic name for the address? */
			if ((wr_ops[i].addr =
			     eeprom_name2addr(argv[anr])) == -1) {
				err("pair %d: bad address %s", i, argv[anr]);
				return 4;
			}
		}

		if (wr_ops[i].addr >= AR5K_EEPROM_INFO_BASE) {
			err("offset 0x%04x in CRC protected area is "
			    "not supported", wr_ops[i].addr);
			return 5;
		}

		anr++;
		wr_ops[i].val = strtoul(argv[anr], &end, 16);
		if (end == argv[anr]) {
			err("pair %d: bad val %s", i, argv[anr]);
			return 5;
		}

		if (wr_ops[i].val > 0xffff) {
			err("pair %d: value %u too large", i, wr_ops[i].val);
			return 6;
		}
		anr++;
		i++;
	}			/* while (anr < (argc-1)) */

	if (!(wr_ops_len = i)) {
		err("no (addr,val) pairs given");
		return 7;
	}

	if (verbose || !force_write) {
		for (i = 0; i < wr_ops_len; i++)
			printf("%20s (0x%04x) := 0x%04x\n",
			       eeprom_addr2name(wr_ops[i].addr), wr_ops[i].addr,
			       wr_ops[i].val);
	}

	if (!force_write) {
		int c;
		printf
		    ("WARNING: The write function may easy brick your device or\n"
		     "violate state regulation on frequency usage.\n"
		     "Proceed on your own risk!\n"
		     "Shall I write the above value(s)? (y/n)\n");
		c = getchar();
		if (c != 'y' && c != 'Y') {
			printf("user abort\n");
			return 0;
		}
	}

	for (i = 0; i < wr_ops_len; i++) {
		u_int16_t oldval, u;

		if (ath5k_hw_eeprom_read
		    (mem, wr_ops[i].addr, &oldval, mac_version)) {
			err("failed to read old value from offset 0x%04x ",
			    wr_ops[i].addr);
			errors++;
		}

		if (oldval == wr_ops[i].val) {
			dbg("pair %d: skipped, value already there", i);
			continue;
		}

		dbg("writing *0x%04x := 0x%04x", wr_ops[i].addr, wr_ops[i].val);
		if (ath5k_hw_eeprom_write
		    (mem, wr_ops[i].addr, wr_ops[i].val, mac_version)) {
			err("failed to write 0x%04x to offset 0x%04x",
			    wr_ops[i].val, wr_ops[i].addr);
			errors++;
		} else {
			if (ath5k_hw_eeprom_read
			    (mem, wr_ops[i].addr, &u, mac_version)) {
				err("failed to read offset 0x%04x for "
				    "verification", wr_ops[i].addr);
				errors++;
			} else {
				if (u != wr_ops[i].val) {
					err("offset 0x%04x: wrote 0x%04x but "
					    "read 0x%04x", wr_ops[i].addr,
					    wr_ops[i].val, u);
					errors++;
				}
			}
		}
	}

	return errors ? 11 : 0;
}				/* do_write_pairs */

static void usage(const char *n)
{
	int i;

	fprintf(stderr, "%s [-w [-g N:M]] [-v] [-f] [-d] [-R addr] [-W addr val] <base_address> "
		"[<name1> <val1> [<name2> <val2> ...]]\n\n", n);
	fprintf(stderr,
		"-w      write values into EEPROM\n"
		"-g N:M  set GPIO N to level M (only used with -w)\n"
		"-v      verbose output\n"
		"-f      force; suppress question before writing\n"
		"-d      dump eeprom (file 'ath-eeprom-dump.bin' and screen)\n"
		"-R <addr>       read register at <addr> (hex)\n"
		"-W <addr> <val> write <val> (hex) into register at <addr> (hex)\n"
		"<base_address>  device base address (see lspci output)\n\n");

	fprintf(stderr,
		"- read info:\n"
		"  %s <base_address>\n\n"
		"- set regdomain to N:\n"
		"  %s -w <base_address> regdomain N\n\n"
		"- set a PCI id field to value N:\n"
		"  %s -w <base_address> <field> N\n"
		"  where <field> is on of:\n    ", n, n, n);
	for (i = 0; i < eeprom_addr_len; i++)
		fprintf(stderr, " %s", eeprom_addr[i].name);
	fprintf(stderr, "\n\n");
	fprintf(stderr,
		"You may need to set a GPIO to a certain value in order to enable\n"
		"writing to the EEPROM with newer chipsets, e.g. set GPIO 4 to low:\n"
		"  %s -g 4:0 -w <base_address> regdomain N\n", n);
	fprintf(stderr,
		"\nDISCLAIMER: The authors are not responsible for any damages caused by\n"
		"this program. Writing improper values may damage the card or cause\n"
		"unlawful radio transmissions!\n\n");
}

u_int32_t extend_tu(u_int32_t base_tu, u_int32_t val, u_int32_t mask)
{
	u_int32_t result;

	result = (base_tu & ~mask) | (val & mask);
	if ((base_tu & mask)  > (val & mask))
		result += mask+1;
	return result;
}

void dump_timers_register(void *mem, u_int16_t mac_version)
{
#define AR5K_TIMER0_5210		0x802c /* next TBTT */
#define AR5K_TIMER0_5211		0x8028
#define AR5K_TIMER0			(mac_version == AR5K_SREV_VER_AR5210? \
					AR5K_TIMER0_5210 : AR5K_TIMER0_5211)

#define AR5K_TIMER1_5210		0x8030 /* next DMA beacon */
#define AR5K_TIMER1_5211		0x802c
#define AR5K_TIMER1			(mac_version == AR5K_SREV_VER_AR5210? \
					AR5K_TIMER1_5210 : AR5K_TIMER1_5211)

#define AR5K_TIMER2_5210		0x8034 /* next SWBA interrupt */
#define AR5K_TIMER2_5211		0x8030
#define AR5K_TIMER2			(mac_version == AR5K_SREV_VER_AR5210? \
					AR5K_TIMER2_5210 : AR5K_TIMER2_5211)

#define AR5K_TIMER3_5210		0x8038 /* next ATIM window */
#define AR5K_TIMER3_5211		0x8034
#define AR5K_TIMER3			(mac_version == AR5K_SREV_VER_AR5210? \
					AR5K_TIMER3_5210 : AR5K_TIMER3_5211)

#define AR5K_TSF_L32_5210		0x806c /* TSF (lower 32 bits) */
#define AR5K_TSF_L32_5211		0x804c
#define AR5K_TSF_L32			(mac_version == AR5K_SREV_VER_AR5210? \
					AR5K_TSF_L32_5210 : AR5K_TSF_L32_5211)

#define AR5K_TSF_U32_5210		0x8070
#define AR5K_TSF_U32_5211		0x8050
#define AR5K_TSF_U32			(mac_version == AR5K_SREV_VER_AR5210? \
					AR5K_TSF_U32_5210 : AR5K_TSF_U32_5211)

#define AR5K_BEACON_5210		0x8024
#define AR5K_BEACON_5211		0x8020
#define AR5K_BEACON			(mac_version == AR5K_SREV_VER_AR5210? \
					AR5K_BEACON_5210 : AR5K_BEACON_5211)

#define AR5K_LAST_TSTP			0x8080
	
	const int timer_mask = 0xffff;

	u_int32_t timer0, timer1, timer2, timer3, now_tu;
	u_int32_t timer0_tu, timer1_tu, timer2_tu, timer3_tu;
	u_int64_t now_tsf;
	
	timer0 = AR5K_REG_READ(AR5K_TIMER0);		/* 0x0000ffff */
	timer1 = AR5K_REG_READ(AR5K_TIMER1_5211);	/* 0x0007ffff */
	timer2 = AR5K_REG_READ(AR5K_TIMER2_5211);	/* 0x?1ffffff */
	timer3 = AR5K_REG_READ(AR5K_TIMER3_5211);	/* 0x0000ffff */
	
	now_tsf = ((u_int64_t)AR5K_REG_READ(AR5K_TSF_U32_5211) << 32)
		| (u_int64_t)AR5K_REG_READ(AR5K_TSF_L32_5211);
	
	now_tu = now_tsf >> 10;
	
	timer0_tu = extend_tu(now_tu, timer0, 0xffff);
	printf("TIMER0 : 0x%8x TBTT : %5u TU:%8x\n", timer0,
	       timer0 & timer_mask, timer0_tu);
	timer1_tu = extend_tu(now_tu, timer1>>3, 0x7ffff>>3);
	printf("TIMER1 : 0x%8x DMAb : %5u TU:%8x(%d)\n", timer1,
	       (timer1>>3) & timer_mask, timer1_tu,
	       timer1_tu - timer0_tu);
	timer2_tu = extend_tu(now_tu, timer2>>3, 0x1ffffff>>3);
	printf("TIMER2 : 0x%8x SWBA : %5u TU:%8x(%d)\n", timer2,
	       (timer2>>3) & timer_mask, timer2_tu,
	       timer2_tu - timer0_tu);
	timer3_tu = extend_tu(now_tu, timer3, 0xffff);
	printf("TIMER3 : 0x%8x ATIM : %5u TU:%8x(%d)\n", timer3,
	       timer3 & timer_mask, timer3_tu,
	       timer3_tu - timer0_tu);
	printf("TSF    : 0x%8llx TSFTU: %5u TU:%8x\n",
	       (unsigned long long)now_tsf, now_tu & timer_mask, now_tu);

	printf("BEACON: %x\n", AR5K_REG_READ(AR5K_BEACON));
	printf("LAST_TSTP: %x\n", AR5K_REG_READ(AR5K_LAST_TSTP));
}

#define AR5K_KEYTABLE_0_5210		0x9000
#define AR5K_KEYTABLE_0_5211		0x8800
#define AR5K_KEYTABLE_0			(mac_version == AR5K_SREV_VER_AR5210? \
					AR5K_KEYTABLE_0_5210 : \
					AR5K_KEYTABLE_0_5211)

#define AR5K_KEYTABLE(_n)		(AR5K_KEYTABLE_0_5211 + ((_n) << 5))
#define AR5K_KEYTABLE_OFF(_n, x)	(AR5K_KEYTABLE(_n) + ((x) << 2))
#define AR5K_KEYTABLE_VALID		0x00008000

#define AR5K_KEYTABLE_SIZE_5210		64
#define AR5K_KEYTABLE_SIZE_5211		128
#define AR5K_KEYTABLE_SIZE		(mac_version == AR5K_SREV_VER_AR5210? \
					AR5K_KEYTABLE_SIZE_5210 : \
					AR5K_KEYTABLE_SIZE_5211)

void keycache_dump(void *mem, u_int16_t mac_version)
{
	int i, keylen;
	u_int32_t val0, val1, val2, val3, val4, keytype, ant, mac0, mac1;

	/* dump all 128 entries */
	printf("Dumping keycache entries...\n");
	for (i=0;i<AR5K_KEYTABLE_SIZE;i++) {
		mac1 = AR5K_REG_READ(AR5K_KEYTABLE_OFF(i,7));
		if (mac1 & AR5K_KEYTABLE_VALID) {
			val0    = AR5K_REG_READ(AR5K_KEYTABLE_OFF(i,0));
			val1    = AR5K_REG_READ(AR5K_KEYTABLE_OFF(i,1));
			val2    = AR5K_REG_READ(AR5K_KEYTABLE_OFF(i,2));
			val3    = AR5K_REG_READ(AR5K_KEYTABLE_OFF(i,3));
			val4    = AR5K_REG_READ(AR5K_KEYTABLE_OFF(i,4));
			keytype = AR5K_REG_READ(AR5K_KEYTABLE_OFF(i,5));
			ant = keytype & 8;
			keytype &= ~8;
			switch (keytype) {
			case 0: /* WEP40  */ keylen =  40 / 8; break;
			case 1: /* WEP104 */ keylen = 104 / 8; break;
			case 3: /* WEP128 */ keylen = 128 / 8; break;
			case 4: /* TKIP   */ keylen = 128 / 8; break;
			case 5: /* AES    */ keylen = 128 / 8; break;
			case 6: /* CCM    */ keylen = 128 / 8; break;
			default:             keylen = 0;       break;
			}
			mac0 = AR5K_REG_READ(AR5K_KEYTABLE_OFF(i,6));

			printf("[%3u] keytype %d [%s%s%s%s%s%s%s%s] mac %02x:%02x:%02x:%02x:%02x:%02x key:%08x-%08x-%08x-%08x-%08x\n",
			       i,
			       keytype,
			       keytype == 0 ? "WEP40 " : "",
			       keytype == 1 ? "WEP104" : "",
			       keytype == 3 ? "WEP128" : "",
			       keytype == 4 ? "TKIP  " : "",
			       keytype == 5 ? "AES   " : "",
			       keytype == 6 ? "CCM   " : "",
			       keytype == 7 ? "NULL  " : "",
			       ant     == 8 ? "+ANT"   : "",
			       ((mac0 <<  1) & 0xff),
			       ((mac0 >>  7) & 0xff),
			       ((mac0 >> 15) & 0xff),
			       ((mac0 >> 23) & 0xff),
			       ((mac1 <<  1) & 0xff) | (mac0 >> 31),
			       ((mac1 >>  7) & 0xff),
			       val0, val1, val2, val3, val4);
		}
	}
}

/* copy key index (0) to key index (idx) */

void keycache_copy(void *mem, u_int16_t mac_version, int idx)
{
	u_int32_t val0, val1, val2, val3, val4, keytype, mac0, mac1;
	
	printf("Copying keycache entry 0 to %d\n", idx);
	if (idx < 0 || idx >= AR5K_KEYTABLE_SIZE) {
		printf("invalid keycache index\n");
		return ;
	}

	val0    = AR5K_REG_READ(AR5K_KEYTABLE_OFF(0,0));
	val1    = AR5K_REG_READ(AR5K_KEYTABLE_OFF(0,1));
	val2    = AR5K_REG_READ(AR5K_KEYTABLE_OFF(0,2));
	val3    = AR5K_REG_READ(AR5K_KEYTABLE_OFF(0,3));
	val4    = AR5K_REG_READ(AR5K_KEYTABLE_OFF(0,4));
	keytype = AR5K_REG_READ(AR5K_KEYTABLE_OFF(0,5));
	mac0    = AR5K_REG_READ(AR5K_KEYTABLE_OFF(0,6));
	mac1    = AR5K_REG_READ(AR5K_KEYTABLE_OFF(0,7));

	AR5K_REG_WRITE(AR5K_KEYTABLE_OFF(idx,0), val0);
	AR5K_REG_WRITE(AR5K_KEYTABLE_OFF(idx,1), val1);
	AR5K_REG_WRITE(AR5K_KEYTABLE_OFF(idx,2), val2);
	AR5K_REG_WRITE(AR5K_KEYTABLE_OFF(idx,3), val3);
	AR5K_REG_WRITE(AR5K_KEYTABLE_OFF(idx,4), val4);
	AR5K_REG_WRITE(AR5K_KEYTABLE_OFF(idx,5), keytype);
	AR5K_REG_WRITE(AR5K_KEYTABLE_OFF(idx,6), mac0);
	AR5K_REG_WRITE(AR5K_KEYTABLE_OFF(idx,7), mac1);
}

void sta_id0_id1_dump(void *mem)
{
#define AR5K_STA_ID0			0x8000
#define AR5K_STA_ID1			0x8004
#define AR5K_STA_ID1_AP                 0x00010000
#define AR5K_STA_ID1_ADHOC              0x00020000
#define AR5K_STA_ID1_NO_KEYSRCH		0x00080000

	u_int32_t sta_id0, sta_id1;

	sta_id0 = AR5K_REG_READ(AR5K_STA_ID0);
	sta_id1 = AR5K_REG_READ(AR5K_STA_ID1);
	printf("STA_ID0 : %02x:%02x:%02x:%02x:%02x:%02x\n",
	       (sta_id0 >>  0) & 0xff,
	       (sta_id0 >>  8) & 0xff,
	       (sta_id0 >> 16) & 0xff,
	       (sta_id0 >> 24) & 0xff,
	       (sta_id1 >>  0) & 0xff,
	       (sta_id1 >>  8) & 0xff);
	printf("STA_ID1 : %08x AP:%d IBSS:%d "
	       "KeyCache Disable:%d\n",
	       sta_id1,
	       sta_id1 & AR5K_STA_ID1_AP ? 1 : 0,
	       sta_id1 & AR5K_STA_ID1_ADHOC ? 1 : 0,
	       sta_id1 & AR5K_STA_ID1_NO_KEYSRCH ? 1 : 0);
}

int main(int argc, char *argv[])
{
	unsigned long long dev_addr;
	u_int16_t eeprom_header, srev, phy_rev_5ghz, phy_rev_2ghz;
	u_int16_t eeprom_version, mac_version, regdomain, has_crystal, ee_magic;
	u_int8_t error, has_a, has_b, has_g, has_rfkill, eeprom_size;
	int byte_size = 0;
	void *mem;
	int fd;
	int i, anr = 1;
	int do_write = 0;	/* default: read only */
	int do_dump = 0;
	int timer_count = 1;
	int do_keycache_dump = 0;
	int keycache_copy_idx = 0;
	int reg_read = 0;
	int reg_write = 0;
	unsigned int reg_write_val = 0;

	struct {
		int valid;
		int value;
	} gpio_set[AR5K_NUM_GPIO];
	int nr_gpio_set = 0;

	for (i = 0; i < sizeof(gpio_set) / sizeof(gpio_set[0]); i++)
		gpio_set[i].valid = 0;

	if (argc < 2) {
		usage(argv[0]);
		return -1;
	}

	while (anr < argc && argv[anr][0] == '-') {
		switch (argv[anr][1]) {
		case 't':
			if (++anr < argc) {
				timer_count = atoi(argv[anr]);
				printf("timer_count:%d\n", timer_count);
			} else {
				usage(argv[0]);
				return 0;
			}
			break;
		case 'w':
			do_write = 1;
			break;
		case 'g':
			anr++;
			if (strlen(argv[anr]) != 3 || argv[anr][1] != ':' ||
			    argv[anr][0] < '0' || argv[anr][0] > '5' ||
			    (argv[anr][2] != '0' && argv[anr][2] != '1')) {
				err("invalid gpio spec. %s", argv[anr]);
				return 2;
			}
			gpio_set[argv[anr][0] - '0'].valid = 1;
			gpio_set[argv[anr][0] - '0'].value = argv[anr][2] - '0';
			nr_gpio_set++;
			break;

		case 'f':
			force_write = 1;
			break;

		case 'v':
			verbose = 1;
			break;

		case 'd':
			do_dump = 1;
			break;

		case 'k':
			do_keycache_dump = 1;
			break;

		case 'K':
			keycache_copy_idx = atoi(argv[++anr]);
			break;

		case 'R':
			anr++;
			reg_read = strtoul(argv[anr], NULL, 16);
			break;

		case 'W':
			anr++;
			reg_write = strtoul(argv[anr++], NULL, 16);
			reg_write_val = strtoul(argv[anr], NULL, 16);
			break;

		case 'h':
			usage(argv[0]);
			return 0;
			break;

		default:
			err("unknown option %s", argv[anr]);
			return 2;
		}		/* switch (argv[anr][1]) */

		anr++;
	}			/* while (anr < argc && ...) */

	if (anr >= argc) {
		err("missing device address");
		usage(argv[0]);
		return 3;
	}

	dev_addr = strtoull(argv[anr], NULL, 16);

	fd = open("/dev/mem", O_RDWR);
	if (fd < 0) {
		printf("Opening /dev/mem failed!\n");
		return -2;
	}

	mem = mmap(NULL, AR5K_PCI_MEM_SIZE, PROT_READ | PROT_WRITE,
		   MAP_SHARED | MAP_FILE, fd, dev_addr);

	if (mem == MAP_FAILED) {
		printf("Mmap of device at 0x%08llX for 0x%X bytes failed - "
		       "%s\n", dev_addr, AR5K_PCI_MEM_SIZE, strerror(errno));
		return -3;
	}

	/* wake from power-down and remove reset (in case the driver isn't running) */
	{
		u_int32_t
		    sleep_ctl = AR5K_REG_READ(AR5K_SLEEP_CTL),
		    reset_ctl = AR5K_REG_READ(AR5K_RESET_CTL);

		dbg("sleep_ctl reg %08x   reset_ctl reg %08x",
		    sleep_ctl, reset_ctl);
		if (sleep_ctl & AR5K_SLEEP_CTL_SLE_SLP) {
			dbg("waking up the chip");
			AR5K_REG_WRITE(AR5K_SLEEP_CTL,
				       (sleep_ctl & ~AR5K_SLEEP_CTL_SLE_SLP));
		}

		if (reset_ctl) {
			dbg("removing resets");
			AR5K_REG_WRITE(AR5K_RESET_CTL, 0);
		}
	}

	AR5K_REG_DISABLE_BITS(AR5K_PCICFG, AR5K_PCICFG_SPWR_DN);
	usleep(500);

	if (reg_read) {
		printf("READ %04x = %08x\n", reg_read, AR5K_REG_READ(reg_read));
		return 0;
	}

	if (reg_write) {
		printf("WRITE %04x = %08x\n", reg_write, reg_write_val);
		AR5K_REG_WRITE(reg_write, reg_write_val);
		return 0;
	}

	srev = AR5K_REG_READ(AR5K_SREV);
	mac_version = AR5K_REG_MS(srev, AR5K_SREV_VER) << 4;

	/* Verify eeprom magic value first */
	error = ath5k_hw_eeprom_read(mem, AR5K_EEPROM_MAGIC, &ee_magic,
				     mac_version);

	if (error) {
		printf("Unable to read EEPROM Magic value!\n");
		return -1;
	}

	if (ee_magic != AR5K_EEPROM_MAGIC_VALUE) {
		printf("Warning: Invalid EEPROM Magic number!\n");
	}

	error = ath5k_hw_eeprom_read(mem, AR5K_EEPROM_HDR, &eeprom_header,
				     mac_version);

	if (error) {
		printf("Unable to read EEPROM Header!\n");
		return -1;
	}

	error = ath5k_hw_eeprom_read(mem, AR5K_EEPROM_VERSION, &eeprom_version,
				     mac_version);

	if (error) {
		printf("Unable to read EEPROM version!\n");
		return -1;
	}

	error = ath5k_hw_eeprom_read(mem, AR5K_EEPROM_REG_DOMAIN, &regdomain,
				     mac_version);

	if (error) {
		printf("Unable to read Regdomain!\n");
		return -1;
	}

	if (eeprom_version >= 0x4000) {
		error = ath5k_hw_eeprom_read(mem, AR5K_EEPROM_MISC0,
					     &has_crystal, mac_version);

		if (error) {
			printf("Unable to read EEPROM Misc data!\n");
			return -1;
		}

		has_crystal = AR5K_EEPROM_HAS32KHZCRYSTAL(has_crystal);
	} else {
		has_crystal = 2;
	}

	eeprom_size = AR5K_REG_MS(AR5K_REG_READ(AR5K_PCICFG),
				  AR5K_PCICFG_EESIZE);

	has_a = AR5K_EEPROM_HDR_11A(eeprom_header);
	has_b = AR5K_EEPROM_HDR_11B(eeprom_header);
	has_g = AR5K_EEPROM_HDR_11G(eeprom_header);
	has_rfkill = AR5K_EEPROM_HDR_RFKILL(eeprom_header);

	if (has_a)
		phy_rev_5ghz = ath5k_hw_radio_revision(mac_version, mem, 1);
	else
		phy_rev_5ghz = 0;

	if (has_b)
		phy_rev_2ghz = ath5k_hw_radio_revision(mac_version, mem, 0);
	else
		phy_rev_2ghz = 0;

	printf(" -==Device Information==-\n");

	printf("MAC Version:  %-5s (0x%02x)\n",
	       ath5k_hw_get_part_name(AR5K_VERSION_VER, mac_version),
	       mac_version);

	printf("MAC Revision: %-5s (0x%02x)\n",
	       ath5k_hw_get_part_name(AR5K_VERSION_VER, srev), srev);

	/* Single-chip PHY with a/b/g support */
	if (has_b && !phy_rev_2ghz) {
		printf("PHY Revision: %-5s (0x%02x)\n",
		       ath5k_hw_get_part_name(AR5K_VERSION_RAD, phy_rev_5ghz),
		       phy_rev_5ghz);
		phy_rev_5ghz = 0;
	}

	/* Single-chip PHY with b/g support */
	if (!has_a) {
		printf("PHY Revision: %-5s (0x%02x)\n",
		       ath5k_hw_get_part_name(AR5K_VERSION_RAD, phy_rev_2ghz),
		       phy_rev_2ghz);
		phy_rev_2ghz = 0;
	}

	/* Different chip for 5Ghz and 2Ghz */
	if (phy_rev_5ghz) {
		printf("5Ghz PHY Revision: %-5s (0x%2x)\n",
		       ath5k_hw_get_part_name(AR5K_VERSION_RAD, phy_rev_5ghz),
		       phy_rev_5ghz);
	}
	if (phy_rev_2ghz) {
		printf("2Ghz PHY Revision: %-5s (0x%2x)\n",
		       ath5k_hw_get_part_name(AR5K_VERSION_RAD, phy_rev_2ghz),
		       phy_rev_2ghz);
	}

	printf(" -==EEPROM Information==-\n");

	printf("EEPROM Version:     %x.%x\n",
	       (eeprom_version & 0xF000) >> 12, eeprom_version & 0xFFF);

	printf("EEPROM Size: ");

	if (eeprom_size == 0) {
		printf("       4K\n");
		byte_size = 4096;
	} else if (eeprom_size == 1) {
		printf("       8K\n");
		byte_size = 8192;
	} else if (eeprom_size == 2) {
		printf("       16K\n");
		byte_size = 16384;
	} else
		printf("       ??\n");

	printf("Regulatory Domain:  0x%X\n", regdomain);

	printf(" -==== Capabilities ====-\n");

	printf("|  802.11a Support: ");
	if (has_a)
		printf("yes  |\n");
	else
		printf("no   |\n");

	printf("|  802.11b Support: ");
	if (has_b)
		printf("yes  |\n");
	else
		printf("no   |\n");

	printf("|  802.11g Support: ");
	if (has_g)
		printf("yes  |\n");
	else
		printf("no   |\n");

	printf("|  RFKill  Support: ");
	if (has_rfkill)
		printf("yes  |\n");
	else
		printf("no   |\n");

	if (has_crystal != 2) {
		printf("|  32KHz   Crystal: ");
		if (has_crystal)
			printf("yes  |\n");
		else
			printf("no   |\n");
	}
	printf(" ========================\n");

	/* print current GPIO settings */
	printf("GPIO registers: CR %08x DO %08x DI %08x\n",
	       AR5K_REG_READ(AR5K_GPIOCR), AR5K_REG_READ(AR5K_GPIODO),
	       AR5K_REG_READ(AR5K_GPIODI));

	if (do_dump) {
		u_int16_t data;
		FILE *dumpfile = fopen("ath-eeprom-dump.bin", "w");

		printf("\nEEPROM dump (%d byte)\n", byte_size);
		printf("==============================================");
		for (i = 0; i < byte_size / 2; i++) {
			error =
			    ath5k_hw_eeprom_read(mem, i, &data, mac_version);
			if (error) {
				printf("\nUnable to read at %04x\n", i);
				continue;
			}
			if (!(i % 8))
				printf("\n%04x:  ", i);
			printf("%04x ", data);
			fwrite(&data, 2, 1, dumpfile);
		}
		printf("\n==============================================\n");
		fclose(dumpfile);
	}

	if (do_write) {
		u_int32_t rcr = AR5K_REG_READ(AR5K_GPIOCR),
		    rdo = AR5K_REG_READ(AR5K_GPIODO);
		u_int32_t old_cr = rcr, old_do = rdo;
		int rc;

		if (mac_version >= AR5K_SREV_VER_AR5213 && !nr_gpio_set) {
			dbg("new MAC %x (>= AR5213) set gpio4 to low",
			    mac_version);
			gpio_set[4].valid = 1;
			gpio_set[4].value = 0;
		}

		/* set gpios */
		dbg("old GPIO CR %08x DO %08x DI %08x",
		    rcr, rdo, AR5K_REG_READ(AR5K_GPIODI));

		for (i = 0; i < sizeof(gpio_set) / sizeof(gpio_set[0]); i++) {
			if (gpio_set[i].valid) {
				rcr |= AR5K_GPIOCR_OUT(i);	/* we use mode 3 */
				rcr &= ~AR5K_GPIOCR_INT_SEL(i);
				rdo &= ~(1 << i);
				rdo |= (gpio_set[i].value << i);
			}
		}

		if (rcr != old_cr) {
			dbg("GPIO CR %x -> %x", old_cr, rcr);
			AR5K_REG_WRITE(AR5K_GPIOCR, rcr);
		}
		usleep(5);

		if (rdo != old_do) {
			dbg("GPIO CR %x -> %x", old_do, rdo);
			AR5K_REG_WRITE(AR5K_GPIODO, rdo);
		}

		/* dump current values again if we have written anything */
		if (rcr != old_cr || rdo != old_do)
			dbg("new GPIO CR %08x DO %08x DI %08x",
			    AR5K_REG_READ(AR5K_GPIOCR),
			    AR5K_REG_READ(AR5K_GPIODO),
			    AR5K_REG_READ(AR5K_GPIODI));

		/* let argv[anr] be the first write parameter */
		anr++;

		rc = do_write_pairs(anr, argc, argv, mem, mac_version);

		/* restore old GPIO settings */
		if (rcr != old_cr) {
			dbg("restoring GPIO CR %x -> %x", rcr, old_cr);
			AR5K_REG_WRITE(AR5K_GPIOCR, old_cr);
		}
		usleep(5);

		if (rdo != old_do) {
			dbg("restoring GPIO CR %x -> %x", rdo, old_do);
			AR5K_REG_WRITE(AR5K_GPIODO, old_do);
		}

		return rc;
	}

	sta_id0_id1_dump(mem);

	for (i=0; i<timer_count; i++)
		dump_timers_register(mem, mac_version);

	if (do_keycache_dump)
		keycache_dump(mem, mac_version);

	if (keycache_copy_idx > 0)
		keycache_copy(mem, mac_version, keycache_copy_idx);

	return 0;
}
