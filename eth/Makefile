ifneq ($(KERNELRELEASE),)

ccflags-y += \
	-I$(src)/../include \
	-DPACKAGE_VERSION=$(PACKAGE_VERSION)

obj-$(CONFIG_PON_ETH_DRV)  += mod_pon_eth.o
mod_pon_eth-objs += \
	pon_eth.o \
	pon_eth_iphost.o

else
check-style:
	@for f in *.h *.c Makefile; do \
		if [ "$$f" = *.mod.c ]; then \
			continue; \
		fi; \
		$(CHECK_SYNTAX) $$f; \
	done
endif
