#!/bin/bash
# 보드에서: lidar_mon을 지금 도는 보드 커널용으로 빌드한다
#
#   (WSL)  ssh xilinx@192.168.2.99 mkdir -p lidar_mon
#          scp kmod/lidar_mon.c kmod/Makefile tools/board/build_kmod.sh xilinx@192.168.2.99:lidar_mon/
#   (보드) cd lidar_mon && sudo bash build_kmod.sh
#   (WSL)  scp xilinx@192.168.2.99:lidar_mon/lidar_mon.ko kmod/board/     → tools/board/bundle.sh가 묶음에 넣음
#
# PYNQ 이미지에는 커널 헤더 트리(/lib/modules/<커널>/build)가 있지만 빌드 도구 실행 파일(fixdep, modpost)이 없다.
# 소스는 있으므로 처음 한 번 보드에서 만든다 (make scripts 약 2분, modules_prepare 약 30초, Cortex-A9).
# 보드의 gcc가 커널을 만든 컴파일러와 같은 판(11.2)이라 보드에서 빌드한다. 크로스 빌드라면 같은 커널 소스·설정과
# Module.symvers가 필요하다 (MODVERSIONS=y).
set -euo pipefail
cd "$(dirname "$0")"
[ "$(id -u)" = 0 ] || { echo "run with sudo (builds the tools inside the kernel header tree)" >&2; exit 1; }
K=/lib/modules/$(uname -r)/build
[ -f "$K/Makefile" ] || { echo "$K is missing: this image has no kernel headers" >&2; exit 1; }
if [ ! -x "$K/scripts/mod/modpost" ]; then
    echo "== $K: make scripts modules_prepare (first time only)"
    make -C "$K" scripts modules_prepare > /tmp/lidar_mon_prepare.log 2>&1 ||
        { tail -20 /tmp/lidar_mon_prepare.log >&2; exit 1; }
fi
echo "== build lidar_mon.ko"
# 빌드 결과는 저장소 주인 소유로 (sudo로 만든 파일이 남지 않게)
sudo -u "${SUDO_USER:-root}" make KDIR="$K" KCC=gcc
echo "== vermagic: this module, then a module that came with the image (must be the same)"
modinfo -F vermagic lidar_mon.ko
modinfo -F vermagic "$(find "/lib/modules/$(uname -r)" -name '*.ko' | head -n 1)"
