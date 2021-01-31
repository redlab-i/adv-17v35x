#makefile for adv17v35x PCIe UARTs for Linux 2.6.9 and newer
KERNEL_SRC = /lib/modules/`uname -r`/build

DRV_NAME := adv17v35x

all: build

obj-m += adv17v35x.o

xrpci-objs :=	adv17v35x.o

EXTRA_CFLAGS += -DDEBUG=1

build:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

clean:
	@rm -f *.o
	@rm -f *.ko
	@rm -f *.mod.c modules.order  Module.symvers Module.markers
	@rm -f .*.cmd
	@rm -rf .tmp_versions
	@rm -rf *~

install:
	@echo "Intalling the advantech serial driver..."
	@echo "Please wait..."
	$(shell ! [ -d /lib/modules/$(shell uname -r)/kernel/drivers/adv17v35x ] && mkdir /lib/modules/$(shell uname -r)/kernel/drivers/adv17v35x)
	$(shell [ -f adv17v35x.ko ]  && cp ./adv17v35x.ko /lib/modules/$(shell uname -r)/kernel/drivers/adv17v35x/ && depmod && modprobe adv17v35x)
	@echo "Done"

uninstall:
	$(shell if grep $(DRV_NAME) /proc/modules > /dev/null ; then \
	 rmmod $(DRV_NAME) ; fi)
	$(shell rm -rf /lib/modules/$(shell uname -r)/kernel/drivers/$(DRV_NAME)/ && depmod)
