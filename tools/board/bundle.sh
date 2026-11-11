#!/bin/bash
# 보드(Arty Z7, ROS 2 없음)에 올릴 묶음 하나를 만든다 → target/board/lidar-rx-board.tar.gz
#
#   tools/board/bundle.sh                    (WSL, 일반 사용자로)
#
# 보드는 빌드하지 않고 받기만 한다: ARM용 lidar-rx를 여기서 크로스 빌드(.cargo/config.toml)해 넣고, 풀면
# deploy/install.sh가 기대하는 자리(저장소와 같은 상대 경로)에 놓이게 묶는다.
#
#   lidar-rx-board/target/release/lidar-rx   ARM 실행 파일 (install.sh는 ROS 2가 없으면 이 경로를 씀)
#   lidar-rx-board/deploy/                   서비스·설정·설치 스크립트 그대로
#   lidar-rx-board/kmod/lidar_mon.ko         kmod/board/lidar_mon.ko(보드 커널용으로 빌드한 것)가 있을 때만.
#                                            WSL용 kmod/lidar_mon.ko는 넣지 않는다 (다른 커널이라 올라가지 않음)
#   lidar-rx-board/tools/board/board_info.sh 보드 환경 수집
set -euo pipefail
cd "$(dirname "$0")/../.."
# shellcheck source=/dev/null
command -v cargo > /dev/null || source "$HOME/.cargo/env"

TARGET=armv7-unknown-linux-musleabihf
NAME=lidar-rx-board
OUT=target/board/$NAME.tar.gz
STAGE=target/board/stage

rustup target list --installed 2> /dev/null | grep -qx "$TARGET" ||
    { echo "rust target $TARGET is missing: rustup target add $TARGET" >&2; exit 1; }
command -v arm-linux-gnueabihf-gcc > /dev/null ||
    { echo "linker arm-linux-gnueabihf-gcc is missing: sudo apt install gcc-arm-linux-gnueabihf" >&2; exit 1; }

echo "== build lidar-rx for $TARGET (no ros2 feature)"
cargo build --release --target "$TARGET" -p rx
BIN=target/$TARGET/release/lidar-rx
file "$BIN" | grep -q 'ELF 32-bit LSB executable, ARM' || { echo "$BIN is not a 32-bit ARM executable" >&2; exit 1; }

echo "== stage $STAGE/$NAME"
rm -rf "$STAGE"
D=$STAGE/$NAME
mkdir -p "$D/target/release" "$D/tools/board"
install -m 755 "$BIN" "$D/target/release/lidar-rx"
cp -r deploy "$D/"
chmod 755 "$D"/deploy/*.sh   # Windows에서 고친 파일은 실행 비트를 잃을 수 있음
install -m 755 tools/board/board_info.sh "$D/tools/board/"
if [ -f kmod/board/lidar_mon.ko ]; then
    install -D -m 644 kmod/board/lidar_mon.ko "$D/kmod/lidar_mon.ko"
    echo "kernel module: kmod/board/lidar_mon.ko -> kmod/lidar_mon.ko"
else
    echo "NOTE: kmod/board/lidar_mon.ko not found: module not bundled (install.sh skips it with a warning)"
fi

echo "== $OUT"
# 소유자를 root로 적는다: 보드에서 sudo로 풀어도 WSL 사용자 번호가 따라가지 않게
tar -C "$STAGE" --owner=0 --group=0 --numeric-owner -czf "$OUT" "$NAME"
rm -rf "$STAGE"
tar -tvzf "$OUT"
echo
echo "bundle: $(pwd)/$OUT ($(du -h "$OUT" | cut -f1), $(stat -c %s "$OUT") bytes)"
echo "copy:   scp $OUT xilinx@192.168.2.99:"
echo "board:  tar xzf $NAME.tar.gz && cd $NAME && sudo deploy/install.sh && sudo systemctl enable --now lidar-rx"
echo "        (no ROS 2 there: install.sh drops --ros2 from lidar-rx.env and installs only lidar-rx.service)"
