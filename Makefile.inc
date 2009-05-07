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
# $Id: //depot/sw/linuxsrc/src/802_11/madwifi/madwifi/Makefile.inc#11 $
#

# other locales sometimes cause trouble
export LC_ALL = POSIX

# BUS defines the bus type to which the wireless devices is attached.
# Currently, the valid BUS types are PCI and AHB.  If BUS is not
# defined, then, we assume the bus type is PCI
ifndef BUS
BUS=		PCI
endif

# DESTDIR is used as path prefix during installation.
DESTDIR ?=

# KERNELPATH is the path to the Linux kernel build tree.  Unless a
# separate build directory was used for the kernel build, it's the same
# as the kernel source tree.  KERNELPATH is used to access the kernel
# configuration, include files and the build system.  To build for
# another kernel, set KERNELPATH manually, for example with:
# make KERNELPATH=/path/to/kernel/source

# The default KERNELPATH points to the directory where the currently
# running kernel was compiled.  Note that the configuration and the
# version of the kernel tree might have changed since then.
ifeq ($(wildcard $(KERNELPATH)),)
KERNELPATH = /lib/modules/$(shell uname -r)/build
# sanity check: does KERNELPATH exist?
ifeq ($(shell cd $(KERNELPATH) && pwd),)
$(error $(KERNELPATH) is missing, please set KERNELPATH)
endif
export KERNELPATH
endif

# KERNELRELEASE is the target kernel's version.  It's always taken from
# the kernel build tree.  Kernel Makefile doesn't always know the exact
# kernel version (especially for vendor stock kernels), so we get it
# from <linux/version.h> instead. But simply grepping it from version.h
# doesn't work, since some distributions have multiple UTS_RELEASE
# in that file.
# This trick has been inspired by the lm_sensors project.
ifndef KERNELRELEASE
KERNELRELEASE := $(shell $(CC) -I $(KERNELPATH)/include -E $(TOP)/kernelversion.c | grep uts_release | cut -f2 -d'"')
ifeq (,$(KERNELRELEASE))
$(error Cannot detect kernel version - please check compiler and KERNELPATH)
endif
endif

# KERNELCONF is the name of the file that holds the configuration
# of the target kernel.
KERNELCONF ?=	$(KERNELPATH)/.config

# sanity check: does KERNELCONF exist?
ifeq ($(wildcard $(KERNELCONF)),)
$(error KERNELCONF: $(KERNELCONF) does not exist.)
endif
include $(KERNELCONF)

# KMODPATH nominates the directory where the modules will be
# installed to
KMODPATH :=	/lib/modules/$(KERNELRELEASE)/net

# Recognize 2.4.x kernels to support the old build system
ifeq ($(filter-out 2.4%,$(KERNELRELEASE)),)
LINUX24 = y
endif

ifndef LINUX24
KMODSUF		:= ko
else
export-objs	= $(foreach m, $(obj-m), $($(m:.o=-objs)))
list-multi	= $(obj-m)
KMODSUF		:= o
endif

NM=		nm

#
# Path to the HAL source code.
#
ifeq ($(HAL),)
HAL=	$(TOP)/ath_hal
endif
#
# Path to the 802.11 include files.
#
WLAN=	$(TOP)/net80211
#
# Path to the Atheros device driver.
#
ATH=	$(TOP)/ath
#
# Path to the rate control algorithms.
#
ATH_RATE=	$(TOP)/ath_rate
#
# Path to the userspace utilities. 
# 
TOOLS=  $(TOP)/tools

WARNINGS = -Werror -Wall
COPTS+= $(WARNINGS)
INCS=	-include $(TOP)/include/compat.h -I$(TOP)/include

ifndef ATH_DEBUG_SPINLOCKS
export ATH_DEBUG_SPINLOCKS=0
endif

ifndef IEEE80211_DEBUG_REFCNT
export IEEE80211_DEBUG_REFCNT=0
endif

ifndef HAL_DEBUG
export HAL_DEBUG=0
endif

ifndef ATH_DEBUG
export ATH_DEBUG=1
endif

ifeq ($(strip $(ATH_DEBUG_SPINLOCKS)),1)
COPTS+= -DATH_DEBUG_SPINLOCKS=1
endif

ifeq ($(strip $(IEEE80211_DEBUG_REFCNT)),1)
COPTS+= -DIEEE80211_DEBUG_REFCNT=1
endif

# HAL_DEBUG enables HAL debugging code
# At the moment this consists of wrappers around HAL functions so that 
# stack traces are more decipherable.
ifeq ($(strip $(HAL_DEBUG)),1)
COPTS+= -fno-inline -DAH_DEBUG=1
endif

include $(TOP)/BuildCaps.inc

ifeq ($(strip $(ATH_DEBUG)),1)
EXTRA_CFLAGS += -DAR_DEBUG -DIEEE80211_DEBUG 
endif

ifeq ($(strip $(ATH_DEBUG_SPINLOCKS)),1)
EXTRA_CFLAGS += -DATH_DEBUG_SPINLOCKS
endif

ifeq ($(strip $(IEEE80211_DEBUG_REFCNT)),1)
EXTRA_CFLAGS += -DIEEE80211_DEBUG_REFCNT
endif

# Ensure correct endianess
LDFLAGS += $(LDOPTS)

# Filter out compiler options that are not supported by all compilers
# and that are not needed to ensure compatible architecture and calling
# conventions.
COPTS := $(filter-out -mshort-load-bytes -mapcs-32,$(COPTS))


# The following variables will affect developers only, and are used in the
# make-release.bash script.
#
# RELEASE_TMP is used as a temporary store for the files needed to create the
# release tarball. Packaging will take place in TMPDIR/madwifi-release.
# Defaults to /tmp.
RELEASE_TMP=/tmp

# RELEASE_STORE is the (local) directory where the release tarball should be
# stored in. Defaults to the parent directory of the working copy used to
# create the release.
RELEASE_STORE=..

# expose these settings to make-release.bash
export RELEASE_TMP RELEASE_STORE
