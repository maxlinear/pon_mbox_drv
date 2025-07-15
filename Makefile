
export PACKAGE_VERSION=1.27.0
PACKAGE=pon_mbox_drv

ifneq ($(KERNELRELEASE),)

obj-y  += eth/
obj-y  += ptp/
obj-y  += src/
obj-y  += test/

else

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) clean

CHECK_SYNTAX=checkpatch.pl -f --no-tree --terse --show-types \
	--ignore LINUX_VERSION_CODE,PREFER_PACKED,SPLIT_STRING,LONG_LINE_STRING,FILE_PATH_CHANGES

check-style:
	@for dir in ./src ./eth ./ptp ./test ; do \
		(make -C $$dir check-style CHECK_SYNTAX="$(CHECK_SYNTAX)"); \
	done

distcheck: dist

PACKAGE_EXCLUDE=^scripts/|^unit_tests|^build_unit_tests|^build_win32

dist:
	git ls-tree -r HEAD --name-only | \
		grep -v -E "$(PACKAGE_EXCLUDE)" | \
		tar cfz $(PACKAGE)-$(PACKAGE_VERSION).tar.gz --transform='s/^/$(PACKAGE)-$(PACKAGE_VERSION)\//' -T -

endif
