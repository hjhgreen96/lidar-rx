#!/bin/bash
# 두 서비스의 지금 상태를 한눈에 (systemctl show에서 필요한 것만, sudo 없이 됨)
#   deploy/status.sh
show() { systemctl show "$1" -p "$2" --value; }
for unit in lidar-rx lidar-record; do
    printf '%-13s %s/%s  pid %s  restarts %s\n' "$unit" "$(show $unit ActiveState)" "$(show $unit SubState)" \
        "$(show $unit MainPID)" "$(show $unit NRestarts)"
done
# Type=notify: 실행 시작에서 "시작됨"(READY=1을 받은 때)까지
exec_at=$(show lidar-rx ExecMainStartTimestampMonotonic)
active_at=$(show lidar-rx ActiveEnterTimestampMonotonic)
[ "${active_at:-0}" -ge "${exec_at:-0}" ] && [ "${exec_at:-0}" -gt 0 ] &&
    echo "lidar-rx became active $(((active_at - exec_at) / 1000)) ms after exec (READY=1)"
status=$(show lidar-rx StatusText)
[ -z "$status" ] || echo "  status: $status"
