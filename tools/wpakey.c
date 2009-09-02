/** WEP/WPA key setting tool for MadWifi driver, version 0.5
 *
 * (C) 2008-03-28 Georg Lukas <georg@madwifi-project.org>
 *
 * This program can be used to debug the MadWifi Key Cache. Use with caution
 * and without warranty!
 *
 * Instructions:
 *
 *	gcc -I/usr/src/madwifi-ng -Wall wpakey.c -o wpakey
 *	./wpakey -h
 *
 * This code is published under the GNU General Public Licence v2.
 */

#include "wireless_copy.h"
#include <include/compat.h>
#include <net80211/ieee80211_ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include <unistd.h>
#include <sys/ioctl.h>

#define MACS "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx"
#define MACP(mac) (mac)[0], (mac)[1], (mac)[2], (mac)[3], (mac)[4], (mac)[5]

static char *dev = "ath0";
static int sock;
static int warn_wpa = 1;

static int parse_mac(uint8_t *mac, const char *str)
{
	if (sscanf(str, MACS,
			&mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]
			) < 6) {
		fprintf(stderr, "Invalid MAC address at \"%s\"\n", str);
		return 0;
	} else {
		return 1;
	}
}

static void hexdump(unsigned char *data, ssize_t dlen)
{
	//printf("%s: (%i) ", prefix, dlen);
	while (dlen-- > 0) {
		printf("%02hhx", *data++);
	}
}

static int set80211param(int op, int arg)
{
	struct iwreq iwr;

	memset(&iwr, 0, sizeof(iwr));

	strncpy(iwr.ifr_name, dev, IFNAMSIZ);
	iwr.u.mode = op;
	memcpy(iwr.u.name+4, &arg, 4);

	if (ioctl(sock, IEEE80211_IOCTL_SETPARAM, &iwr) < 0) {
		perror("ioctl(setparam)");
		return -1;
	}
	return 0;
}

static int get80211param(int op)
{
	struct iwreq iwr;

	memset(&iwr, 0, sizeof(iwr));

	strncpy(iwr.ifr_name, dev, IFNAMSIZ);
	iwr.u.mode = op;

	if (ioctl(sock, IEEE80211_IOCTL_GETPARAM, &iwr) < 0) {
		perror("ioctl(getparam)");
		return -1;
	}
	return iwr.u.mode;
}


static int set80211priv(int op, void *data, int len)
{
	struct iwreq iwr;

	memset(&iwr, 0, sizeof(iwr));

	strncpy(iwr.ifr_name, dev, IFNAMSIZ);
	iwr.u.data.pointer = data;
	iwr.u.data.length = len;

	if (ioctl(sock, op, &iwr) < 0) {
		perror("ioctl()");
		return -1;
	}
	return iwr.u.data.length;
}

static void prep_key(struct ieee80211req_key *wk, int keyidx, uint8_t *mac)
{
	memset(wk, 0, sizeof(struct ieee80211req_key));
	wk->ik_keyix = keyidx;

	if (keyidx == IEEE80211_KEYIX_NONE) {
		memcpy(wk->ik_macaddr, mac, 6);
	}
}

static char *cipherstrs[] = { "WEP", "TKIP", "OCB", "CCMP", "invalid",
			      "CKIP", "none" };

static char *strcipher(int c)
{

	if (c > IEEE80211_CIPHER_NONE) return "invalid";
	return cipherstrs[c];
}

static char *strflags(int f)
{
	static char buf[5];
	char *ff = buf;
	memset(buf, 0, sizeof(buf));

	if (f & IEEE80211_KEY_RECV) *ff++ = 'R';
	if (f & IEEE80211_KEY_XMIT) *ff++ = 'T';
	if (f & IEEE80211_KEY_DEFAULT) *ff++ = 'D';
	if (f & IEEE80211_KEY_GROUP) *ff++ = 'G';
	*ff = '\0';
	return buf;
}

static int getkey(int keyidx, uint8_t *mac, int verbose)
{
	struct ieee80211req_key wk;

	if (warn_wpa && get80211param(IEEE80211_PARAM_WPA) == 0) {
		printf("WARNING: WPA is disabled!\n");
		warn_wpa = 0;
	}
	prep_key(&wk, keyidx, mac);
	if (set80211priv(IEEE80211_IOCTL_GETKEY, &wk, sizeof(wk)) >= 0) {
		if (verbose == 0 && wk.ik_type == IEEE80211_CIPHER_NONE && wk.ik_keylen == 0)
			return 0;
		printf("Key %4x: <" MACS "> ", wk.ik_keyix, MACP(wk.ik_macaddr));
		if (wk.ik_type == IEEE80211_CIPHER_NONE && wk.ik_flags == 3 && wk.ik_keylen == 0) {
			printf("off\n");
		} else {
			printf("c=%-4s f=%-4s k<%-2i>=", strcipher(wk.ik_type),
				strflags(wk.ik_flags), wk.ik_keylen);
			hexdump(wk.ik_keydata, wk.ik_keylen);
			printf(" rs=%lld ts=%lld\n", (long long)wk.ik_keyrsc,
			       (long long)wk.ik_keytsc);
		}
		return 0;
	}
	return -1;
}

static int delkey(int keyidx, uint8_t *mac)
{
	struct ieee80211req_key wk;

	prep_key(&wk, keyidx, mac);
	return set80211priv(IEEE80211_IOCTL_DELKEY, &wk, sizeof(wk));
}

static int setkey(int keyidx, uint8_t *mac, int type, int flags, int keylen,
		  char *key)
{
	struct ieee80211req_key wk;

	prep_key(&wk, keyidx, mac);
	wk.ik_type = type;
	wk.ik_flags = flags;
	wk.ik_keylen = keylen;
	memcpy(wk.ik_keydata, key, keylen);

	return set80211priv(IEEE80211_IOCTL_SETKEY, &wk, sizeof(wk));
}


static void iter_sta(void)
{
	uint8_t buf[24*1024];
	uint8_t *bufpos;
	ssize_t len;

	if ((len = set80211priv(IEEE80211_IOCTL_STA_INFO, buf, sizeof(buf))) >= 0) {
		struct ieee80211req_sta_info *si;
		bufpos = buf;
		while (len > sizeof(struct ieee80211req_sta_info)) {
			si = (struct ieee80211req_sta_info*)bufpos;

			getkey(IEEE80211_KEYIX_NONE, si->isi_macaddr, 0);

			bufpos += si->isi_len;
			len -= si->isi_len;
		}

	}
}

static void set_wpa(int cipher, int wpa, int key)
{
	printf("Setting WPA: cipher=%s wpa=%i mgmt=%i\n",
		strcipher(cipher), wpa, key);
	set80211param(IEEE80211_PARAM_MCASTCIPHER, cipher);
	set80211param(IEEE80211_PARAM_UCASTCIPHERS, 1 << cipher);
	set80211param(IEEE80211_PARAM_KEYMGTALGS, key);
	set80211param(IEEE80211_PARAM_WPA, wpa);

	set80211param(IEEE80211_PARAM_PRIVACY, wpa > 0? 1 : 0);
}


static void init(void)
{
	sock = socket(PF_INET, SOCK_DGRAM, 0);
	if (sock < 0) {
		perror("socket()");
		exit(32);
	}
}


static void help(void)
{
	fprintf(stderr, "Possible options are:\n"
		"	-a		print all group keys\n"
		"	-A		print all keys (default option)\n"
		"	-i <if>		set interface (default: %s)\n"
		"	-w		WPA on\n"
		"	-n		no WPA\n"
		"	-k <idx|mac>	set / read specified key (default: 0)\n"
		"	-c <#>		set ciphers\n"
		"	-f <#|rtgd>	set key flags - integer or: Rx/Tx/Group/Default\n"
		"	[-|<key>]	ASCII text to be set as key or '-' to unset.\n"
		"\n"
		"Example:\n"
		"	# activate WPA on ath3, write an RX+TX key to slot #3\n"
		"	./wpakey -i ath3 -w -k 3 -f rt XXXXXXXXXXXXXXXX\n"
		"", dev);
}

int main(int argc, char** argv)
{
	int keyidx = 0;
	uint8_t mac[6];
	int cipher = IEEE80211_CIPHER_AES_CCM;
	int flags = IEEE80211_KEY_RECV | IEEE80211_KEY_XMIT;
	int c;
	int i;

	init();

	if (argc == 1) {
		for (i = 0; i <= 3; i++) {
			getkey(i, NULL, 1);
		}
		iter_sta();
		return 0;
	}
	while ((c = getopt(argc, argv, "aAhi:f:c:k:wn")) != -1) {
		//printf("%c - %s\n", c, optarg);
		switch (c) {
		case 'a':
		case 'A':
			for (i = 0; i <= 3; i++) {
				getkey(i, NULL, 1);
			}
			if (c == 'A')
				iter_sta();
			break;
		case 'h':
			help();
			break;
		case 'i':
			dev = optarg;
			break;
		case 'f':
			if (isalpha(optarg[0])) {
				flags = 0;
				while (*optarg) switch (*optarg++) {
				case 'g': flags |= IEEE80211_KEY_GROUP; break;
				case 'r': flags |= IEEE80211_KEY_RECV; break;
				case 't': flags |= IEEE80211_KEY_XMIT; break;
				case 'd': flags |= IEEE80211_KEY_DEFAULT; break;
				}
			} else
				flags = strtol(optarg, NULL, 0);
			printf("flags = 0x%02x\n", flags);
			break;
		case 'c':
			cipher = strtol(optarg, NULL, 0);
			printf("cipher = %x\n", cipher);
			break;
		case 'w':
			set_wpa(cipher, 3, WPA_ASE_8021X_PSK);
			break;
		case 'n':
			set_wpa(IEEE80211_CIPHER_NONE, 0, WPA_ASE_NONE);
			break;
		case 'k':
			if (strlen(optarg) < 17)
				keyidx = atoi(optarg);
			else {
				keyidx = IEEE80211_KEYIX_NONE;
				parse_mac(mac, optarg);
			}
			getkey(keyidx, mac, 1);
			break;
		}
	}

	if (optind < argc) {
		switch (argv[optind][0]) {
		case '-':
			delkey(keyidx, mac);
			setkey(keyidx, mac, IEEE80211_CIPHER_NONE, 0, 0, NULL);
			getkey(keyidx, mac, 1);
			return 0;
		case 'g':
			flags |= IEEE80211_KEY_GROUP;
			break;
		case 'd':
			flags |= IEEE80211_KEY_DEFAULT;
			break;
		}
		setkey(keyidx, mac, cipher,
			flags, 128/8, argv[optind]);
		getkey(keyidx, mac, 1);
	}

	return 0;
}
