#!/bin/bash
# 실행 중인 WSL 커널과 같은 소스·같은 설정으로 커널을 빌드한다.
# 목적: 외부 모듈(kmod/lidar_mon)이 쓸 Module.symvers(심볼 체크섬)를 얻는 것.
#
#   ./build.sh config   소스 받기 + 설정 + 실행 중 커널 설정과 비교 + 버전 문자열 확인
#   ./build.sh build    전체 빌드 (로그: $SRC/build.log)
#
# 주의
# - 소스는 WSL 파일시스템(~/src)에 둔다. /mnt/* (NTFS)는 대소문자를 구분하지 않아 소스가 깨진다.
# - LOCALVERSION= : git 트리에서는 버전 끝에 '+'가 붙어 실행 중 커널과 달라진다(scripts/setlocalversion).
#   여기서는 압축 파일로 받아 git이 없지만, 같은 명령을 git 트리에서 써도 맞도록 항상 지정한다.
# - PATH를 고정해 conda 같은 다른 툴체인이 섞이지 않게 한다.
set -euo pipefail
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

RELEASE=$(uname -r)                       # 6.18.54.1-microsoft-standard-WSL2
TAG=linux-msft-wsl-${RELEASE%%-*}         # linux-msft-wsl-6.18.54.1
SRC=$HOME/src/WSL2-Linux-Kernel-$TAG
MAKE=(make -C "$SRC" CC=gcc-13 HOSTCC=gcc-13 LOCALVERSION= -j"$(nproc)")

config() {
    if [ ! -d "$SRC" ]; then
        mkdir -p "$HOME/src"
        curl -L --fail -o "$HOME/src/$TAG.tar.gz" \
            "https://github.com/microsoft/WSL2-Linux-Kernel/archive/refs/tags/$TAG.tar.gz"
        tar -C "$HOME/src" -xzf "$HOME/src/$TAG.tar.gz"
    fi
    zcat /proc/config.gz > "$SRC/.config"
    "${MAKE[@]}" olddefconfig

    echo "== config lines that differ from the running kernel (< running, > this build)"
    diff <(zcat /proc/config.gz | grep '^CONFIG_' | sort) \
         <(grep '^CONFIG_' "$SRC/.config" | sort) || true

    local built
    built=$("${MAKE[@]}" -s kernelrelease)
    echo "== kernelrelease: $built (running: $RELEASE)"
    [ "$built" = "$RELEASE" ] || { echo "version string differs: modules built from this tree would be refused" >&2; exit 1; }
}

build() {
    # JOBS=2: 마지막 링크의 BTF 생성(pahole)은 디버그 정보가 든 vmlinux 전체를 스레드마다 메모리에 올린다.
    # 기본값(-j와 같은 16)으로 두면 WSL VM(16 GB)의 메모리가 바닥나 VM 전체가 재시작됐다.
    "${MAKE[@]}" JOBS=2 all > "$SRC/build.log" 2>&1
    ls -l "$SRC/Module.symvers" "$SRC/vmlinux"
}

case "${1:-}" in
    config) config ;;
    build)  build ;;
    *) echo "usage: $0 config|build" >&2; exit 2 ;;
esac
