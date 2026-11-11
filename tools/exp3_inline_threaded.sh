#!/bin/bash
# 실험 3 - 버리는 위치 (inline vs threaded)
#
#   cargo build --release -p rx -p pcap-replay     (먼저)
#   tools/exp3_inline_threaded.sh
#
# lidar-rx에 프레임마다 D ms 동안 CPU를 쓰는 소비자(--work-ms D: 뒤따르는 인식 알고리즘 모사)를 붙이고,
# replay가 7,093 패킷(9.4 s)을 센서 간격으로 보내는 동안 받는다. 같은 D를 두 방식으로:
#
#   inline    수신 스레드가 소비자를 직접 부름 → 처리하는 동안 소켓을 비우지 못함
#             → 버퍼가 차면 커널이 아무 패킷이나 버림 → 구멍 나거나 손실 지점에서 끊긴 프레임
#   threaded  소비자는 자기 스레드 + 길이 N 채널. 수신 스레드는 계속 비우고,
#             채널이 차면 완성된 프레임 단위로 버림 (그 소비자 몫만)
#
# 프레임은 100 ms마다 하나. D가 100 ms를 넘으면 소비자는 따라갈 수 없다 → 무엇을 버리는지가 갈린다.
# 결과: 표(화면) + docs/media/exp3_inline_threaded.csv
set -euo pipefail
cd "$(dirname "$0")/.."
for f in target/release/lidar-rx target/release/pcap-replay data/vlp16.pcap; do
    [ -e "$f" ] || { echo "$f is missing: run the build commands above first" >&2; exit 1; }
done
PORT=2368
OUT=$(mktemp) ROWS=$(mktemp)   # 행을 모았다가 다 끝나면 CSV에 씀: 중간에 실패해도 이전 결과가 남게
trap 'rm -f "$OUT" "$ROWS"' EXIT

# lidar-rx 요약에서 값 하나를 꺼냄. 없으면 이유를 보이고 멈춤
need() {
    local v
    v=$(grep -oP "$1" "$OUT" | head -n 1 || true)
    if [ -z "$v" ]; then
        echo "'$2' not found in the lidar-rx output:" >&2
        cat "$OUT" >&2
        return 1
    fi
    echo "$v"
}

CSV=docs/media/exp3_inline_threaded.csv
mkdir -p docs/media
echo "mode,queue,work_ms,received_packets,missing_packets,kernel_drops,socket_wait_p99_us,work_frames,work_complete_frames,work_queue_drops,work_age_p99_us" > "$ROWS"
# received: lidar-rx가 실제로 꺼낸 패킷. missing은 받은 패킷 사이의 빈틈으로 판정하므로, 끝날 때 아직 큐에 남아
# 읽지 못한 패킷은 어느 쪽에도 안 들어간다 → 보낸 수(7,093) − received가 실제로 못 받은 수
# complete: 처리한 프레임 중 온전한 한 바퀴(0°에서 0°까지, 빠진 패킷 없음). 처음과 끝 조각은 한 바퀴가 아님
printf '%-9s %5s %7s | %8s %7s %6s %11s | %6s %8s %6s %10s\n' \
    mode queue work_ms received missing kdrops wait_p99_ms frames complete qdrops age_p99_ms

# 행: 방식 채널길이 처리시간(ms)
for row in "inline 0 0" "inline 0 50" "inline 0 90" "inline 0 150" \
           "threaded 4 0" "threaded 4 50" "threaded 4 90" "threaded 4 150" "threaded 1 150"; do
    read -r mode queue work <<< "$row"
    q_arg=()
    [ "$mode" = threaded ] && q_arg=(--queue "$queue")
    target/release/lidar-rx --udp 127.0.0.1:$PORT --duration 11 --status 100 \
        --mode "$mode" "${q_arg[@]}" --work-ms "$work" 2> "$OUT" &
    rx=$!
    sleep 0.5
    target/release/pcap-replay data/vlp16.pcap --to 127.0.0.1:$PORT --pace sensor > /dev/null 2>&1
    wait $rx || { echo "lidar-rx failed:" >&2; cat "$OUT" >&2; exit 1; }  # 포트 사용 중 등

    received=$(need '^decoder: \K[0-9]+' 'decoder packets')
    missing=$(need '^decoder: .*missing \K[0-9]+' 'decoder missing')
    kdrops=$(need 'kernel drops \(SO_RXQ_OVFL\) \K[0-9]+' 'kernel drops')
    wait_p99=$(need '^queue wait .*p99 \K[0-9]+' 'queue wait p99')
    qdrops=$(need '^consumer work: queue drops \K[0-9]+' 'work queue drops')
    frames=$(need '^consumer work: .*; \K[0-9]+(?= frames processed)' 'work frames')
    complete=$(need '^consumer work: .*\(\K[0-9]+(?= complete:)' 'work complete frames')
    age_p99=$(need '^consumer work: .*started p50 [0-9]+ us, p99 \K[0-9]+' 'work age p99')

    [ "$mode" = threaded ] || queue=-
    echo "$mode,$queue,$work,$received,$missing,$kdrops,$wait_p99,$frames,$complete,$qdrops,$age_p99" >> "$ROWS"
    awk -v m="$mode" -v q="$queue" -v w="$work" -v r="$received" -v mi="$missing" -v k="$kdrops" \
        -v wp="$wait_p99" -v f="$frames" -v c="$complete" -v qd="$qdrops" -v a="$age_p99" \
        'BEGIN { printf "%-9s %5s %7s | %8s %7s %6s %11.1f | %6s %8s %6s %10.1f\n",
                 m, q, w, r, mi, k, wp / 1000, f, c, qd, a / 1000 }'
done
cat "$ROWS" > "$CSV"
echo "saved: $CSV"
