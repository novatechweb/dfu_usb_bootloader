PREFIX ?= arm-cortexm3-eabi
CROSS_COMPILE ?= $(PREFIX)-

all: libopencm3
	$(MAKE) CROSS_COMPILE=$(CROSS_COMPILE) -C src

distclean:
	-@$(RM) -rf libopencm3
	-@$(MAKE) -C src clean

%:
	$(MAKE) CROSS_COMPILE=$(CROSS_COMPILE) -C src $@

libopencm3:
	@git submodule init
	@git submodule update
	@$(MAKE) PREFIX=$(PREFIX) -C libopencm3 lib

