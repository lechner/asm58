ASM58_KVER ?= $(shell uname -r)
ASM58_MAJMIN := $(shell echo $(ASM58_KVER) | cut -d . -f 1-2)

ifeq ($(ASM58_MAJMIN),2.6)

ifneq ($(PATCHLEVEL),)
#
# Make rules for use from within 2.6 kbuild system
#
obj-m	+= asm58.o

else  # We were called from command line

KDIR	?= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

install: all
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -a

clean:
	rm -rf .*.cmd  *.mod.c *.ko *.o .tmp_versions Module.symvers Module.markers modules.order *~ core *.i *.cmd
endif

endif
