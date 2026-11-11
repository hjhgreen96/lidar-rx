#!/bin/bash
# deploy/install.sh가 설치한 것을 되돌린다 (docs/modules/deploy.md)
#   sudo deploy/uninstall.sh            서비스·실행 파일·설정·모듈 자동 적재를 지움
#   sudo deploy/uninstall.sh --purge    사용자 lidar와 그 홈(/var/lib/lidar: 녹화 파일, ROS 2 로그)까지
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "run with sudo" >&2; exit 1; }

systemctl disable --now lidar-record.service lidar-rx.service 2> /dev/null || true
rm -f /etc/systemd/system/lidar-rx.service /etc/systemd/system/lidar-record.service
systemctl daemon-reload
systemctl reset-failed lidar-rx.service lidar-record.service 2> /dev/null || true
rm -f /usr/local/bin/lidar-rx

rm -f /etc/modules-load.d/lidar_mon.conf /etc/modprobe.d/lidar_mon.conf
# 설치한 뒤 커널이 바뀌었으면 옛 커널 쪽에 남은 사본도 지운다
for ko in /lib/modules/*/updates/lidar_mon.ko; do
    [ -e "$ko" ] || continue
    rm -f "$ko"
    rmdir "$(dirname "$ko")" 2> /dev/null || true
    depmod -a "$(basename "$(dirname "$(dirname "$ko")")")"
done
rmmod lidar_mon 2> /dev/null || true

if [ -f /etc/lidar-rx/rmem_max.before ]; then
    sysctl -q -w net.core.rmem_max="$(cat /etc/lidar-rx/rmem_max.before)"
    rm -f /etc/sysctl.d/90-lidar-rx.conf
fi
rm -rf /etc/lidar-rx

if [ "${1:-}" = --purge ]; then
    userdel lidar 2> /dev/null || true
    rm -rf /var/lib/lidar
fi
echo "uninstalled"
