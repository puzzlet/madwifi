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
# $Id: //depot/sw/linuxsrc/src/802_11/madwifi/madwifi/Makefile $
#

#
# Makefile for the HAL-based Atheros driver.
#

obj := $(firstword $(obj) $(SUBDIRS) .)
TOP = $(obj)

include $(TOP)/Makefile.inc

obj-y := ath/ ath_hal/ ath_rate/ net80211/

ifdef CRYPTO_TESTING
obj-y += regression/
endif

.PHONY: all
all: modules tools

.PHONY: modules
modules: configcheck $(TOP)/svnversion.h
ifdef LINUX24
	for i in $(obj-y); do \
		$(MAKE) -C $$i || exit 1; \
	done
else
	$(MAKE) -C $(KERNELPATH) SUBDIRS=$(shell pwd) modules
endif

$(addprefix $(obj)/, $(obj-y:/=)): $(TOP)/svnversion.h

$(TOP)/svnversion.h:
	@cd $(TOP) && \
	if [ -d .svn ]; then \
		ver=$$(svnversion -nc . | sed -e 's/^[^:]*://;s/[A-Za-z]//'); \
		echo "#define SVNVERSION \"svn r$$ver\"" > $@.tmp; \
	elif [ -d .git ]; then \
		ver=$$(git svn log | head -n2 | tail -n1 | cut -d\  -f1); \
		echo "#define SVNVERSION \"svn $$ver\"" > $@.tmp; \
	elif [ -s SNAPSHOT ]; then \
		ver=$$(sed -e '/^Revision: */!d;s///;q' SNAPSHOT); \
		echo "#define SVNVERSION \"svn r$$ver\"" > $@.tmp; \
	else \
		touch $@.tmp; \
	fi || exit 1; \
	diff $@ $@.tmp >/dev/null 2>&1 || cp -f $@.tmp $@; rm -f $@.tmp

# conflicts with the 'tools' subdirectory
.PHONY: tools
tools:
	$(MAKE) -C $(TOOLS) all || exit 1

.PHONY: install
install: install-modules install-tools

.PHONY: install-modules
install-modules: modules
	@# check if there are modules left from an old installation
	@# might cause make to abort the build
	sh scripts/find-madwifi-modules.sh -r $(KERNELRELEASE) $(DESTDIR)

	for i in $(obj-y); do \
		$(MAKE) -C $$i install || exit 1; \
	done
ifeq ($(DESTDIR),)
	(export KMODPATH=$(KMODPATH); /sbin/depmod -ae $(KERNELRELEASE))
endif

.PHONY: install-tools
install-tools: tools
	$(MAKE) -C $(TOOLS) install || exit 1

.PHONY: uninstall ininstall-modules
uninstall: uninstall-tools uninstall-modules
uninstall-modules:
	sh scripts/find-madwifi-modules.sh -r $(KERNELRELEASE) $(DESTDIR)

.PHONY: list-modules find-modules
list-modules: find-modules
find-modules:
	sh scripts/find-madwifi-modules.sh -l $(KERNELRELEASE)

.PHONY: uninstall-tools
uninstall-tools:
	$(MAKE) -C $(TOOLS) uninstall

.PHONY: reinstall reinstall-tools reinstall-modules
reinstall: uninstall install
reinstall-tools: uninstall-tools install-tools
reinstall-modules: uninstall-modules install-modules

.PHONY: clean
clean:
	for i in $(obj-y); do \
		$(MAKE) -C $$i clean; \
	done
	-$(MAKE) -C $(TOOLS) clean
	rm -rf .tmp_versions
	rm -f modules.order *.symvers Module.markers svnversion.h

.PHONY: info
info:
	@echo "The following settings will be used for compilation:"
	@echo "BUS          : $(BUS)"
	@echo "KERNELRELEASE: $(KERNELRELEASE)"
	@echo "KERNELPATH   : $(KERNELPATH)"
	@echo "KERNELCONF   : $(KERNELCONF)"
	@echo "KMODPATH     : $(KMODPATH)"
	@echo "KMODSUF      : $(KMODSUF)"

.PHONY: sanitycheck
sanitycheck:
	@echo -n "Checking requirements... "
	
	@# check if specified rate control is available
	@if [ ! -d $(ATH_RATE) ]; then \
	    echo "FAILED"; \
	    echo "Selected rate control $(ATH_RATE) not available."; \
	    exit 1; \
	fi
	
	@echo "ok."

.PHONY: release
release:
	sh scripts/make-release.bash

.PHONY: unload 
unload:
	bash scripts/madwifi-unload

.PHONY: configcheck
configcheck: sanitycheck
	@echo -n "Checking kernel configuration... "
	
	@# check version of kernel
	@echo $(KERNELRELEASE) | grep -q -i '^[2-9]\.[4-9]\.' || { \
	    echo "FAILED"; \
	    echo "Only kernel versions 2.4.x and above are supported."; \
	    echo "You have $(KERNELRELEASE)."; \
	    exit 1; \
	}
	
	@# check kernel configuration
	@if [ -z "$(CONFIG_SYSCTL)" ]; then \
	    echo "FAILED"; \
	    echo "Please enable sysctl support."; \
	    exit 1; \
	fi
	
ifeq ($(strip $(BUS)),PCI)
	@# check PCI support
	@if [ -z "$(CONFIG_PCI)" ]; then \
	    echo "FAILED"; \
	    echo "Please enable PCI support."; \
	    exit 1; \
	fi
endif
	
	@# check wireless extensions support is enabled
	@if [ -z "$(CONFIG_NET_RADIO)$(CONFIG_WIRELESS_EXT)" ]; then \
	    echo "FAILED"; \
	    echo "Please enable wireless extensions."; \
	    exit 1; \
	fi
	
	@# check crypto support is enabled
	@if [ -z "$(CONFIG_CRYPTO)" ]; then \
	    echo "FAILED"; \
	    echo "Please enable crypto API."; \
	    exit 1; \
	fi
	
	@echo "ok."
