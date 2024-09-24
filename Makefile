# SPDX-License-Identifier: GPL-2.0-only

obj-m += gud.o

# Default to running kernel's build directory if KDIR not set externally
KDIR ?= "/lib/modules/$(shell uname -r)/build"

all:
	$(MAKE) -C $(KDIR) M="$(CURDIR)" modules

debug:
	$(MAKE) -C $(KDIR) M="$(CURDIR)" modules EXTRA_CFLAGS="-g -DDEBUG"

install:
	$(MAKE) -C $(KDIR) M="$(CURDIR)" modules_install
	depmod -A

clean:
	$(MAKE) -C $(KDIR) M="$(CURDIR)" clean