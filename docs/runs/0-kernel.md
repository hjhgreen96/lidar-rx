## 실행 중인 커널과 그 설정

모듈은 실행 중인 커널과 버전 문자열(vermagic)과 심볼 체크섬이 같아야 올라간다. 먼저 지금 커널과 그 설정을 본다.

```console
$ uname -r
6.18.54.1-microsoft-standard-WSL2
```

```console
$ zcat /proc/config.gz | grep -E "^CONFIG_(MODVERSIONS|GENKSYMS|DEBUG_INFO_BTF_MODULES|MODULE_ALLOW_BTF_MISMATCH|RANDSTRUCT_NONE|LOCALVERSION)="
CONFIG_LOCALVERSION="-microsoft-standard-WSL2"
CONFIG_MODVERSIONS=y
CONFIG_GENKSYMS=y
CONFIG_RANDSTRUCT_NONE=y
CONFIG_DEBUG_INFO_BTF_MODULES=y
CONFIG_MODULE_ALLOW_BTF_MISMATCH=y
```

MODVERSIONS=y: 심볼 체크섬 검사가 켜져 있어 커널을 한 번 전체 빌드해 Module.symvers를 얻어야 한다. MODULE_ALLOW_BTF_MISMATCH=y: 직접 빌드한 커널의 타입 정보(BTF)가 조금 달라도 적재는 허용.

## 같은 소스·같은 설정으로 커널 트리 준비

build.sh config: WSL 커널 소스(태그 linux-msft-wsl-6.18.54.1)에 /proc/config.gz를 적용하고, 달라진 설정 줄과 버전 문자열을 확인한다. 달라진 것은 컴파일러·어셈블러·링커 버전과 asm goto 플래그뿐이다 (구조체 같은 타입 정의는 그대로 → 심볼 체크섬에 영향 없음).

```console
$ tools/wsl-kernel/build.sh config 2>&1 | sed -n "/differ from the running kernel/,\$p"
== config lines that differ from the running kernel (< running, > this build)
212c212
< CONFIG_AS_VERSION=24100
---
> CONFIG_AS_VERSION=23800
353a354,355
> CONFIG_CC_HAS_ASM_GOTO_OUTPUT=y
> CONFIG_CC_HAS_ASM_GOTO_TIED_OUTPUT=y
376c378
< CONFIG_CC_VERSION_TEXT="gcc (GCC) 13.2.0"
---
> CONFIG_CC_VERSION_TEXT="gcc-13 (Ubuntu 13.4.0-6ubuntu1~22~ppa2) 13.4.0"
818d819
< CONFIG_GCC_ASM_GOTO_OUTPUT_BROKEN=y
821c822
< CONFIG_GCC_VERSION=130200
---
> CONFIG_GCC_VERSION=130400
1400c1401
< CONFIG_LD_VERSION=24100
---
> CONFIG_LD_VERSION=23800
== kernelrelease: 6.18.54.1-microsoft-standard-WSL2 (running: 6.18.54.1-microsoft-standard-WSL2)
```

전체 빌드(tools/wsl-kernel/build.sh build)는 한 번만 하면 되고 시간이 걸려 여기서는 다시 돌리지 않는다. 결과물:

```console
$ ls -l ~/src/WSL2-Linux-Kernel-linux-msft-wsl-6.18.54.1/Module.symvers ~/src/WSL2-Linux-Kernel-linux-msft-wsl-6.18.54.1/vmlinux
-rw-r--r-- 1 jihyeon jihyeon   1080434 Nov  8 23:48 /home/jihyeon/src/WSL2-Linux-Kernel-linux-msft-wsl-6.18.54.1/Module.symvers
-rwxr-xr-x 1 jihyeon jihyeon 397712792 Nov  8 23:52 /home/jihyeon/src/WSL2-Linux-Kernel-linux-msft-wsl-6.18.54.1/vmlinux
```

## 확인용 최소 모듈 적재 확인

```console
$ make -C tools/wsl-kernel/smoke 2>&1 | grep -E "CC \[M\]|LD \[M\]|BTF \[M\]"
  CC [M]  smoke.o
  CC [M]  smoke.mod.o
  CC [M]  .module-common.o
  LD [M]  smoke.ko
  BTF [M] smoke.ko
```

```console
$ /sbin/modinfo -F vermagic tools/wsl-kernel/smoke/smoke.ko
6.18.54.1-microsoft-standard-WSL2 SMP preempt mod_unload modversions 
```

```console
$ sudo insmod tools/wsl-kernel/smoke/smoke.ko && sudo rmmod smoke
```

```console
$ sudo dmesg | grep -E "smoke|disagrees|version magic" | tail -n 2
[49125.711162] smoke: loaded
[49125.744403] smoke: unloaded
```

vermagic이 uname -r과 같고, 체크섬 불일치(disagrees about version of symbol) 없이 올라가고 내려갔다.

_기록한 날: 2026-11-09 14:33, 커널 6.18.54.1-microsoft-standard-WSL2_
