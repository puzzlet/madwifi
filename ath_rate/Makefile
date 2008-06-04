obj := $(firstword $(obj) $(SUBDIRS) .)
TOP = $(obj)/..

obj-y := amrr/ onoe/ sample/ minstrel/

include $(TOP)/Makefile.inc

.PHONY: modules
modules:
ifdef LINUX24
	for i in $(obj-y); do \
		$(MAKE) -C $$i || exit 1; \
	done
else
	$(MAKE) -C $(KERNELPATH) SUBDIRS=$(shell pwd) modules
endif

.PHONY: install
install:
	for i in $(obj-y); do \
		$(MAKE) -C $$i install || exit 1; \
	done

.PHONY: clean
clean:
	for i in $(obj-y); do \
		$(MAKE) -C $$i clean; \
	done

	rm -f modules.order
