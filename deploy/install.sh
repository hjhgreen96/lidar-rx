#!/bin/bash
# lidar-rx를 systemd 서비스로 설치한다 (docs/modules/deploy.md)
#
#   tools/build_rx_ros2.sh && make -C kmod        (먼저, 일반 사용자로)
#   sudo deploy/install.sh                         설치만 한다
#   sudo systemctl enable --now lidar-rx lidar-record     켜기 (부팅 때도)
#   sudo deploy/uninstall.sh [--purge]             되돌리기 (--purge: 사용자 lidar와 녹화 파일까지)
#   보드(ROS 2 없음): tools/board/bundle.sh로 묶어 보드에 풀고 같은 install.sh. --ros2를 빼고 녹화 서비스는 안 깖
#
#   설치하는 것                                    어디로
#   전용 사용자 lidar (홈 /var/lib/lidar)          ROS 2 로그 ~/.ros, 녹화 ~/bags
#   lidar-rx 실행 파일                             /usr/local/bin/lidar-rx
#   실행 인자, ROS 2 환경                          /etc/lidar-rx/lidar-rx.env, ros2.env
#   lidar_mon 모듈 + 부팅 때 적재·인자             /lib/modules/<커널>/updates/, /etc/modules-load.d/, /etc/modprobe.d/
#   소켓 버퍼 상한 (지금 값이 모자랄 때만)          /etc/sysctl.d/90-lidar-rx.conf
#   서비스 두 개                                   /etc/systemd/system/lidar-rx.service, lidar-record.service
set -euo pipefail
cd "$(dirname "$0")/.."
[ "$(id -u)" = 0 ] || { echo "run with sudo" >&2; exit 1; }
ROS_SETUP=${ROS_SETUP:-/opt/ros/humble/setup.bash}   # ROS_SETUP=/none으로 주면 ROS 2가 없는 보드처럼 설치 (WSL에서 시험용)
BIN=target/ros2/release/lidar-rx
[ -f "$ROS_SETUP" ] || BIN=target/release/lidar-rx   # ROS 2가 없는 보드: ROS 2 없이 빌드한 것 (tools/board/bundle.sh)
[ -e "$BIN" ] || { echo "$BIN is missing: build it first (see the top of this script)" >&2; exit 1; }

echo "== user lidar"
id lidar > /dev/null 2>&1 || useradd --system --home-dir /var/lib/lidar --create-home --shell /usr/sbin/nologin lidar
install -d -o lidar -g lidar /var/lib/lidar/bags

echo "== /usr/local/bin/lidar-rx ($BIN), /etc/lidar-rx/"
install -m 755 "$BIN" /usr/local/bin/lidar-rx
install -d /etc/lidar-rx
if [ ! -e /etc/lidar-rx/lidar-rx.env ]; then   # 고쳐 쓴 설정은 덮어쓰지 않음
    if [ -f "$ROS_SETUP" ]; then
        install -m 644 deploy/lidar-rx.env /etc/lidar-rx/
    else
        # ROS 2가 없는 보드: 같은 인자에서 --ros2만 뺀다. 수신부는 그대로, 소비자 구성만 바뀜
        sed 's/ --ros2//' deploy/lidar-rx.env > /etc/lidar-rx/lidar-rx.env
        chmod 644 /etc/lidar-rx/lidar-rx.env
    fi
fi
if [ -f "$ROS_SETUP" ]; then
    # ROS 2 환경을 셸의 source에 기대지 않고 파일로 준다: 빈 환경에서 setup.bash를 불러 필요한 변수만 남김
    env -i PATH=/usr/sbin:/usr/bin:/sbin:/bin bash -c "source $ROS_SETUP > /dev/null && env" |
        grep -E '^(PATH|LD_LIBRARY_PATH|PYTHONPATH|AMENT_PREFIX_PATH|CMAKE_PREFIX_PATH|ROS_[A-Z_]+|RMW_[A-Z_]+)=' \
            > /etc/lidar-rx/ros2.env
fi

echo "== kernel module lidar_mon (loaded at boot by systemd-modules-load)"
# 모듈은 관찰용이라 없어도 수신은 돈다 (서비스도 Requires를 쓰지 않음) → 여기서 실패해도 경고만 하고 설치를 계속한다.
# 이 커널용으로 빌드한 것만 올라간다 (vermagic). 커널이 바뀌면 다시 빌드해서 다시 설치
if [ ! -e kmod/lidar_mon.ko ]; then
    echo "WARNING: kmod/lidar_mon.ko is missing (make -C kmod): skipped; lidar-rx runs without it" >&2
else
    install -D -m 644 kmod/lidar_mon.ko "/lib/modules/$(uname -r)/updates/lidar_mon.ko"
    depmod -a
    install -m 644 deploy/modules-load.d/lidar_mon.conf /etc/modules-load.d/
    install -m 644 deploy/modprobe.d/lidar_mon.conf /etc/modprobe.d/
    # 부팅 때와 같은 경로로 지금 올림
    systemctl restart systemd-modules-load.service ||
        echo "WARNING: systemd-modules-load failed (journalctl -u systemd-modules-load)" >&2
    if ! grep -q '^lidar_mon ' /proc/modules; then
        if [ "$(systemctl show -p ConditionResult --value systemd-modules-load.service)" = no ]; then
            # WSL에서는 systemd가 자신을 컨테이너 안으로 보고(systemd-detect-virt: wsl) 이 서비스를 건너뛴다
            # (ConditionVirtualization=!container). 그래서 지금은 직접 올린다. 부팅 때 자동 적재는 보드에서 확인
            echo "systemd-modules-load skipped (virtualization: $(systemd-detect-virt)); loading lidar_mon with modprobe now"
        fi
        modprobe lidar_mon ||
            echo "WARNING: lidar_mon is not loaded (dmesg shows why); lidar-rx runs without it" >&2
    fi
fi

echo "== socket receive buffer limit"
# 지금 값으로 판단한다: WSL은 커널 기본값이 이미 4 MB. 지금 값이 sysctl -w로 잠깐 올린 것이면 재부팅 뒤 내려가니
# 그 경우 재설치 필요
NEED=1048576
NOW=$(sysctl -n net.core.rmem_max)
if [ -e /etc/sysctl.d/90-lidar-rx.conf ]; then
    echo "net.core.rmem_max is $NOW (/etc/sysctl.d/90-lidar-rx.conf already installed)"
elif [ "$NOW" -lt "$NEED" ]; then
    echo "$NOW" > /etc/lidar-rx/rmem_max.before   # uninstall이 되돌릴 값
    install -m 644 deploy/sysctl.d/90-lidar-rx.conf /etc/sysctl.d/
    sysctl -q -p /etc/sysctl.d/90-lidar-rx.conf
    echo "net.core.rmem_max $NOW -> $(sysctl -n net.core.rmem_max)"
else
    echo "net.core.rmem_max is already $NOW (>= $NEED): not lowering it, sysctl.d file not installed"
fi

echo "== services"
UNITS=(lidar-rx)
[ -f "$ROS_SETUP" ] && UNITS+=(lidar-record)   # 녹화(ros2 bag)는 ROS 2가 있어야 함 → 보드에서는 설치 안 함
for unit in "${UNITS[@]}"; do
    install -m 644 "deploy/$unit.service" /etc/systemd/system/
done
systemctl daemon-reload
echo "installed. start: sudo systemctl enable --now ${UNITS[*]}"
