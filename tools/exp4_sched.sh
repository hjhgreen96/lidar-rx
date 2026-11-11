#!/bin/bash
# 실험 4 - 부하 속 수신 지연 (스케줄링 정책)
#
#   cargo build --release -p rx -p pcap-replay     (먼저, 일반 사용자로)
#   sudo tools/exp4_sched.sh                        (SCHED_FIFO는 root 필요)
#
# 수신 스레드를 CPU 2에 고정하고(--cpu 2), replay는 CPU 6에서 보낸다. 조건마다 7,093 패킷을 받으며
# 큐 대기 시간(커널이 받은 시각 → lidar-rx가 꺼낸 시각)의 분포를 잰다.
#
#   부하  none   아무것도 안 돎
#         same   CPU 2(수신 스레드와 같은 CPU)에 CPU를 쉬지 않고 쓰는 프로세스 4개
#         other  CPU 3(다른 CPU)에 같은 것 4개 - 부하 자체가 아니라 "같은 CPU"가 문제인지 확인
#   정책  SCHED_OTHER  일반 (나머지 프로세스와 CPU 시간을 나눔)
#         SCHED_FIFO   --rt-prio 50, 수신 스레드 하나에만. 깨어나면 일반 작업에게서 바로 CPU를 빼앗음
#
# 부하는 bash의 무한 루프 (stress-ng 없이). 결과: 표(화면) + docs/media/exp4_sched.csv
set -euo pipefail
cd "$(dirname "$0")/.."
[ "$(id -u)" = 0 ] || { echo "run with sudo (SCHED_FIFO needs root)" >&2; exit 1; }
for f in target/release/lidar-rx target/release/pcap-replay data/vlp16.pcap; do
    [ -e "$f" ] || { echo "$f is missing: run the build commands above first" >&2; exit 1; }
done
PORT=2368 RX_CPU=2 TX_CPU=6 OTHER_CPU=3 HOGS=4
OUT=$(mktemp) ROWS=$(mktemp)   # 행을 모았다가 다 끝나면 CSV에 씀: 중간에 실패해도 이전 결과가 남게
HOG_PIDS=()
stop_hogs() { [ ${#HOG_PIDS[@]} -eq 0 ] || kill "${HOG_PIDS[@]}" 2>/dev/null || true; wait 2>/dev/null || true; HOG_PIDS=(); }
trap 'stop_hogs; rm -f "$OUT" "$ROWS"' EXIT

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

CSV=docs/media/exp4_sched.csv
mkdir -p docs/media
echo "load,policy,wait_p50_us,wait_p99_us,wait_max_us,missing_packets,kernel_drops,receive_thread" > "$ROWS"
printf '%-5s %-11s | %8s %8s %8s | %7s %6s | %s\n' \
    load policy p50_us p99_us max_us missing kdrops "receive thread (read back)"

for row in "none other" "none fifo" "same other" "same fifo" "other other"; do
    read -r load policy <<< "$row"
    case $load in
        same) hog_cpu=$RX_CPU ;;
        other) hog_cpu=$OTHER_CPU ;;
        *) hog_cpu= ;;
    esac
    if [ -n "$hog_cpu" ]; then
        for _ in $(seq $HOGS); do
            taskset -c "$hog_cpu" bash -c 'while :; do :; done' & HOG_PIDS+=($!)
        done
        sleep 0.5
    fi
    prio=()
    [ "$policy" = fifo ] && prio=(--rt-prio 50)
    target/release/lidar-rx --udp 127.0.0.1:$PORT --duration 11 --status 100 \
        --cpu $RX_CPU "${prio[@]}" 2> "$OUT" &
    rx=$!
    sleep 0.5
    taskset -c $TX_CPU target/release/pcap-replay data/vlp16.pcap --to 127.0.0.1:$PORT --pace sensor > /dev/null 2>&1
    wait $rx || { echo "lidar-rx failed:" >&2; cat "$OUT" >&2; exit 1; }  # 포트 사용 중 등
    stop_hogs

    p50=$(need '^queue wait .*p50 \K[0-9]+' 'queue wait p50')
    p99=$(need '^queue wait .*p99 \K[0-9]+' 'queue wait p99')
    max=$(need '^queue wait .*max \K[0-9]+' 'queue wait max')
    missing=$(need '^decoder: .*missing \K[0-9]+' 'decoder missing')
    kdrops=$(need 'kernel drops \(SO_RXQ_OVFL\) \K[0-9]+' 'kernel drops')
    thread=$(need '^receive thread: \K.*(?= \(read back)' 'receive thread')

    echo "$load,$policy,$p50,$p99,$max,$missing,$kdrops,\"$thread\"" >> "$ROWS"  # 값 안에 쉼표가 있어 따옴표로
    printf '%-5s %-11s | %8s %8s %8s | %7s %6s | %s\n' "$load" "$policy" "$p50" "$p99" "$max" \
        "$missing" "$kdrops" "$thread"
done
cat "$ROWS" > "$CSV"
chown "$(stat -c %U .):" "$CSV"
echo "saved: $CSV"
