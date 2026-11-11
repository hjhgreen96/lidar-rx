#!/bin/bash
# 실험 1 - 소켓 수신 버퍼 용량 (+ 실험 2: 버퍼는 곧 지연이다)
#
#   cargo build --release -p rx -p pcap-replay     (먼저, 일반 사용자로)
#   make -C kmod                                    (먼저)
#   sudo tools/exp1_rcvbuf.sh [쉬는 ms=1000]
#
# 수신 버퍼 크기(SO_RCVBUF 요청값)마다:
#   lidar-rx가 패킷 1500개를 받은 뒤 한 번 [쉬는 ms] 동안 읽지 않음 → 그동안 도착한 약 754 × 쉬는 s 개 중
#   버퍼에 들어간 만큼만 남고 나머지는 커널이 버림. 다시 읽기 시작할 때 큐에 있던 수 = 버퍼 용량(패킷 수).
#
# 버린 수를 네 군데서 세어 맞는지 본다:
#   ① 커널 모듈(IP 계층)이 본 수 − lidar-rx가 받은 수     ② SO_RXQ_OVFL (소켓 누적 드롭)
#   ③ 트레이스포인트 skb:kfree_skb, reason SOCKET_RCVBUFF  ④ 트레이스포인트 udp:udp_fail_queue_rcv_skb
#
# 결과: 표(화면) + docs/media/exp1_rcvbuf.csv
set -euo pipefail
cd "$(dirname "$0")/.."
[ "$(id -u)" = 0 ] || { echo "run with sudo (loads the kernel module, uses tracing)" >&2; exit 1; }
for f in target/release/lidar-rx target/release/pcap-replay kmod/lidar_mon.ko data/vlp16.pcap; do
    [ -e "$f" ] || { echo "$f is missing: run the build commands above first" >&2; exit 1; }
done
STALL_MS=${1:-1000}
PORT=2368
T=/sys/kernel/tracing
[ -e $T/trace ] || mount -t tracefs nodev $T
KFREE=$T/events/skb/kfree_skb UDPFAIL=$T/events/udp/udp_fail_queue_rcv_skb
# ③ 버린 이유(drop reason)는 5.17부터 있다. 번호는 커널마다 다를 수 있어 format에서 SOCKET_RCVBUFF의 번호를 읽음
RCVBUFF_NO=$(grep -oP '\{ \K[0-9]+(?=, "SOCKET_RCVBUFF" \})' $KFREE/format 2>/dev/null || true)
[ -d $UDPFAIL ] && HAVE_UDPFAIL=1 || HAVE_UDPFAIL=0
LOADED_HERE=0
if [ ! -e /proc/lidar_mon ]; then insmod kmod/lidar_mon.ko port=$PORT; LOADED_HERE=1; fi
OUT=$(mktemp) ROWS=$(mktemp)   # 행을 모았다가 다 끝나면 CSV에 씀: 중간에 실패해도 이전 결과가 남게
cleanup() {
    rm -f "$OUT" "$ROWS"
    echo 0 > $T/tracing_on
    if [ -n "$RCVBUFF_NO" ]; then echo 0 > $KFREE/enable; echo 0 > $KFREE/filter; fi
    [ $HAVE_UDPFAIL = 0 ] || echo 0 > $UDPFAIL/enable
    [ $LOADED_HERE = 0 ] || rmmod lidar_mon
}
trap cleanup EXIT

trace_start() {
    echo 0 > $T/tracing_on
    echo > $T/trace
    if [ -n "$RCVBUFF_NO" ]; then
        echo "reason == $RCVBUFF_NO" > $KFREE/filter
        echo 1 > $KFREE/enable
    fi
    [ $HAVE_UDPFAIL = 0 ] || echo 1 > $UDPFAIL/enable
    echo 1 > $T/tracing_on
}
trace_count() {   # 실험 중 다른 UDP 트래픽은 없다고 보고 해당 사건을 모두 셈. 없는 트레이스포인트는 n/a
    echo 0 > $T/tracing_on
    TRACE_RCVBUFF=n/a TRACE_UDPFAIL=n/a
    [ -z "$RCVBUFF_NO" ] || TRACE_RCVBUFF=$(grep -c 'reason: SOCKET_RCVBUFF' $T/trace || true)
    [ $HAVE_UDPFAIL = 0 ] || TRACE_UDPFAIL=$(grep -c 'udp_fail_queue_rcv_skb:' $T/trace || true)
}

# lidar-rx 요약에서 값 하나를 꺼냄. 없으면 (모듈 카운터를 못 읽음 등) 이유를 보이고 멈춤
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

CSV=docs/media/exp1_rcvbuf.csv
# 이론 용량: 커널 6.18 판정식 ⌊실제 버퍼 ÷ truesize⌋, 쉬는 동안 도착한 수보다 많을 수는 없음.
# truesize 2304 B = 2 KB 데이터 칸(헤더 + 페이로드 1206 B + skb_shared_info 320 B) + struct sk_buff 256 B
TRUESIZE=2304
echo "requested_B,actual_B,queued_at_resume,theory_capacity,oldest_wait_ms,arrived_during_stall,kmod_seen_minus_received,so_rxq_ovfl,trace_kfree_rcvbuff,trace_udp_fail" > "$ROWS"
# 열: 요청한 버퍼, 커널이 잡은 버퍼, 다시 읽을 때 큐에 있던 수(측정), 이론 용량, 가장 오래 기다린 ms, 쉬는 동안 도착한 수
#     | 버린 수 ① 모듈이 본 수 − 받은 수 ② SO_RXQ_OVFL ③ kfree_skb(SOCKET_RCVBUFF) ④ udp_fail_queue_rcv_skb
# (한글 머리글은 printf가 바이트 수로 칸을 맞춰 줄이 어긋나므로 영어로)
printf '%-10s %-10s %8s %8s %10s %8s | %10s %10s %10s %10s\n' \
    "request_B" "actual_B" "queued" "theory" "oldest_ms" "arrived" "1:kmod" "2:ovfl" "3:kfree" "4:udpfail"

for REQ in 4096 16384 65536 131072 262144 524288 1048576; do
    trace_start
    target/release/lidar-rx --udp 127.0.0.1:$PORT --rcvbuf $REQ --stall-ms "$STALL_MS" --stall-after 1500 \
        --duration 11 --status 100 2> "$OUT" &
    RX=$!
    sleep 0.5
    target/release/pcap-replay data/vlp16.pcap --to 127.0.0.1:$PORT --pace sensor > /dev/null 2>&1
    wait $RX || { echo "lidar-rx failed:" >&2; cat "$OUT" >&2; exit 1; }  # 포트 사용 중 등
    trace_count

    ACTUAL=$(need 'socket receive buffer \K[0-9]+' 'socket receive buffer')
    QUEUED=$(need '\K[0-9]+(?= packets were queued)' 'stall: … packets were queued')
    OLDEST_US=$(need 'oldest waited \K[0-9]+' 'oldest waited')
    KMOD_DIFF=$(need 'seen − received here = \K-?[0-9]+' 'kernel module … seen − received here')
    OVFL=$(need 'SO_RXQ_OVFL\) \K[0-9]+' 'SO_RXQ_OVFL')
    ARRIVED=$((QUEUED + KMOD_DIFF))
    THEORY=$((ACTUAL / TRUESIZE))
    [ $THEORY -gt $ARRIVED ] && THEORY=$ARRIVED
    printf '%-10s %-10s %8s %8s %10s %8s | %10s %10s %10s %10s\n' \
        "$REQ" "$ACTUAL" "$QUEUED" "$THEORY" "$((OLDEST_US / 1000))" "$ARRIVED" "$KMOD_DIFF" "$OVFL" "$TRACE_RCVBUFF" "$TRACE_UDPFAIL"
    echo "$REQ,$ACTUAL,$QUEUED,$THEORY,$((OLDEST_US / 1000)),$ARRIVED,$KMOD_DIFF,$OVFL,$TRACE_RCVBUFF,$TRACE_UDPFAIL" >> "$ROWS"
done
cat "$ROWS" > "$CSV"
chown "$(stat -c %U .):" "$CSV"
echo "saved: $CSV"
