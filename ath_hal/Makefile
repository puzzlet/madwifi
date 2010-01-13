#
# Copyright (c) 2002-2005 Sam Leffler, Errno Consulting
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer,
#    without modification.
# 2. Redistributions in binary form must reproduce at minimum a disclaimer
#    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
#    redistribution must be conditioned upon including a substantially
#    similar Disclaimer requirement for further binary redistribution.
# 3. Neither the names of the above-listed copyright holders nor the names
#    of any contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# Alternatively, this software may be distributed under the terms of the
# GNU General Public License ("GPL") version 2 as published by the Free
# Software Foundation.
#
# NO WARRANTY
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
# AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
# THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
# OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
# IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGES.
#
# $Id: //depot/sw/linuxsrc/src/802_11/madwifi/madwifi/ath/Makefile#12 $
#

#
# Makefile for the Atheros WLAN driver.
#

obj := $(firstword $(obj) $(SUBDIRS) .)
TOP = $(obj)/..

include $(TOP)/Makefile.inc

obj-m		+= ath_hal.o
ath_hal-objs	:= \
	ah.o \
	ah_eeprom_v1.o \
	ah_eeprom_v14.o \
	ah_eeprom_v3.o \
	ah_os.o \
	ah_regdomain.o \
	ar5210/ar5210_attach.o \
	ar5210/ar5210_beacon.o \
	ar5210/ar5210_interrupts.o \
	ar5210/ar5210_keycache.o \
	ar5210/ar5210_misc.o \
	ar5210/ar5210_phy.o \
	ar5210/ar5210_power.o \
	ar5210/ar5210_recv.o \
	ar5210/ar5210_reset.o \
	ar5210/ar5210_xmit.o \
	ar5211/ar5211_attach.o \
	ar5211/ar5211_beacon.o \
	ar5211/ar5211_interrupts.o \
	ar5211/ar5211_keycache.o \
	ar5211/ar5211_misc.o \
	ar5211/ar5211_phy.o \
	ar5211/ar5211_power.o \
	ar5211/ar5211_recv.o \
	ar5211/ar5211_reset.o \
	ar5211/ar5211_xmit.o \
	ar5212/ar2316.o \
	ar5212/ar2317.o \
	ar5212/ar2413.o \
	ar5212/ar2425.o \
	ar5212/ar5111.o \
	ar5212/ar5112.o \
	ar5212/ar5212_ani.o \
	ar5212/ar5212_attach.o \
	ar5212/ar5212_beacon.o \
	ar5212/ar5212_eeprom.o \
	ar5212/ar5212_gpio.o \
	ar5212/ar5212_interrupts.o \
	ar5212/ar5212_keycache.o \
	ar5212/ar5212_misc.o \
	ar5212/ar5212_phy.o \
	ar5212/ar5212_power.o \
	ar5212/ar5212_recv.o \
	ar5212/ar5212_reset.o \
	ar5212/ar5212_rfgain.o \
	ar5212/ar5212_xmit.o \
	ar5212/ar5413.o \
	ar5416/ar2133.o \
	ar5416/ar5416_ani.o \
	ar5416/ar5416_attach.o \
	ar5416/ar5416_beacon.o \
	ar5416/ar5416_cal.o \
	ar5416/ar5416_cal_adcdc.o \
	ar5416/ar5416_cal_adcgain.o \
	ar5416/ar5416_cal_iq.o \
	ar5416/ar5416_eeprom.o \
	ar5416/ar5416_gpio.o \
	ar5416/ar5416_interrupts.o \
	ar5416/ar5416_keycache.o \
	ar5416/ar5416_misc.o \
	ar5416/ar5416_phy.o \
	ar5416/ar5416_power.o \
	ar5416/ar5416_recv.o \
	ar5416/ar5416_reset.o \
	ar5416/ar5416_xmit.o \
	ar5416/ar9160_attach.o \
	ar5416/ar9280.o \
	ar5416/ar9280_attach.o

ifeq ($(strip $(BUS)),AHB)
ifeq ($(CONFIG_64BIT)),)
ath_hal-objs += \
	ar5312/ar5312_attach.o \
	ar5312/ar5312_eeprom.o \
	ar5312/ar5312_gpio.o \
	ar5312/ar5312_interrupts.o \
	ar5312/ar5312_misc.o \
	ar5312/ar5312_power.o \
	ar5312/ar5312_reset.o \
	ar5312/ar5315_gpio.o
endif
endif

ifdef LINUX24
export-objs	:= ah_os.o
endif

INCS += -I$(HAL)

EXTRA_CFLAGS+= $(INCS) $(COPTS)

-include $(TOPDIR)/Rules.make

.PHONY: all
all:
	$(MAKE) -C $(KERNELPATH) SUBDIRS=$(shell pwd) modules

.PHONY: install
install:
	test -d $(DESTDIR)/$(KMODPATH) || mkdir -p $(DESTDIR)/$(KMODPATH)
	install -m 0644 ath_hal.$(KMODSUF) $(DESTDIR)/$(KMODPATH)

.PHONY: clean
clean:
	rm -f *~ *.o *.ko *.mod.c .*.cmd
	rm -f modules.order .depend .version .*.o.flags .*.o.d
	rm -f ar*/*~ ar*/*.o ar*/.*.cmd
	rm -rf .tmp_versions

ath_hal.o: $(addprefix $(obj)/,$(ath_hal-objs))
	$(LD) $(LDOPTS) -o ath_hal.$(KMODSUF) -r $(ath_hal-objs)

