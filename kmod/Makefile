# lidar_mon 커널 모듈
#
#   make            실행 중인 WSL 커널용 (tools/wsl-kernel/build.sh로 만든 커널 트리 기준)
#   make clean
#
# 보드용 크로스 빌드는 5단계: make KDIR=<보드 커널 트리> ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- KCC=…
#
# LOCALVERSION= : 커널 트리가 git이면 버전 끝에 '+'가 붙어 실행 중인 커널과 달라지는 것을 막음

obj-m := lidar_mon.o

KDIR ?= $(HOME)/src/WSL2-Linux-Kernel-linux-msft-wsl-$(shell uname -r | cut -d- -f1)
# 컴파일러: WSL은 실행 중 커널을 만든 것과 같은 계열(gcc-13), 크로스 빌드면 $(CROSS_COMPILE)gcc
ifeq ($(CROSS_COMPILE),)
KCC ?= gcc-13
else
KCC ?= $(CROSS_COMPILE)gcc
endif

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) CC=$(KCC) LOCALVERSION= W=1 modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
