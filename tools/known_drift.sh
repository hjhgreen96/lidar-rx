#!/bin/bash
# 정답을 아는 시계 테스트 (WSL loopback: 보내는 쪽과 받는 쪽이 같은 시계).
#
#   tools/known_drift.sh [드리프트 ppm=100] [반복=3]
#
# 1) --pace sensor --drift-ppm X : 센서 시각 간격 × (1 + X ppm)으로 보냄 → 받는 쪽 추정 기울기 ≈ X
# 2) --pace capture (반복 없이)   : 원래 캡처 간격대로 보냄 → 원래 캡처 PC와 센서 사이의 관계(약 −53 ppm)가 재현됨
#    캡처 간격으로 반복 재생하면 반복이 이어지는 곳마다 두 시계의 길이 차이(약 0.5 ms)만큼 틈이 생겨 기울기가 흐려짐
set -euo pipefail
cd "$(dirname "$0")/.."
DRIFT=${1:-100}
LOOPS=${2:-3}
cargo build -q --release -p rx -p pcap-replay

run() {
    local label=$1 loops=$2; shift 2
    echo "=== $label"
    ./target/release/lidar-rx --udp 127.0.0.1:2368 --duration "$(( loops * 10 + 1 ))" --status 5 &
    local rx=$!
    sleep 0.5
    ./target/release/pcap-replay data/vlp16.pcap --to 127.0.0.1:2368 --loops "$loops" "$@"
    wait "$rx"
}

run "sensor pace, drift +$DRIFT ppm, $LOOPS loops" "$LOOPS" --pace sensor --drift-ppm "$DRIFT"
run "capture pace, 1 loop" 1 --pace capture
