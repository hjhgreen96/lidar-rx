KDIR ?= $(HOME)/src/WSL2-Linux-Kernel-linux-msft-wsl-$(shell uname -r | cut -d- -f1)

obj-m := smoke.o

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) CC=gcc-13 LOCALVERSION= modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
