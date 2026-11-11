#!/bin/bash
# 보드 실시간 데모 (녹화 없이): Windows replay → 실제 Ethernet → 보드 lidar-rx 서비스 → PC의 RViz. RViz 창을 닫으면 모두 끝남.
#
#   tools/board_demo.sh                   100바퀴(약 16분) 보냄
#   tools/board_demo.sh --gap 12:0.8      보내기 시작 12 s 뒤 0.8 s 끊기 (선을 뽑은 것처럼 - 보드 로그의 missing이 603 늚)
#   tools/board_demo.sh --loops 300       더 오래 (약 47분)
#   (뒤 인자는 pcap-replay.exe에 그대로. 같은 인자를 또 주면 뒤의 것이 이김)
#
#   Windows  pcap-replay.exe ──Ethernet──► 보드 192.168.2.99:2368 (lidar-rx 서비스, --forward 192.168.2.1:2369)
#              └ --echo :2369 (보드가 넘긴 것을 받아) --echo-relay ──► WSL lidar-rx --ros2 ──► /velodyne_points ──► RViz
#   이 터미널: 보드 서비스 로그 (journalctl -f, 2초 간격)
#
# 전달을 PowerShell 스크립트가 아니라 pcap-replay.exe가 맡는 이유: 방화벽 첫 확인 창에서 "개인 네트워크"만 고르면
# powershell.exe의 "공용 네트워크" 수신 차단 규칙이 생기고(차단이 허용보다 우선), 보드와 연결한 USB 어댑터는 공용.
# --echo는 패킷마다 기록을 남기므로 바퀴 수 지정 (--loops N).
#
# 사전 준비: README "보드에서 실행"
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
BOARD=${BOARD:-xilinx@192.168.2.99}
BOARD_IP=${BOARD#*@}
PORT=2369
RX=target/ros2/release/lidar-rx

tools/build_rx_ros2.sh -q
cargo build -q --release --target x86_64-pc-windows-gnu -p pcap-replay
[ -f data/vlp16.pcap ] || tools/fetch_data.sh
# Windows 쪽 파일: WSL 경로(\\wsl.localhost)에서 바로 실행하면 시작이 수 초 늦어 Windows 로컬 디스크로 복사해 씀
WIN_DIR=$(wslpath "$(cmd.exe /c 'echo %USERPROFILE%' 2> /dev/null | tr -d '\r')")/lidar-demo
mkdir -p "$WIN_DIR"
cp target/x86_64-pc-windows-gnu/release/pcap-replay.exe data/vlp16.pcap "$WIN_DIR/"

if ! ssh -o ConnectTimeout=5 "$BOARD" 'systemctl is-active --quiet lidar-rx'; then
    echo "board not reachable or lidar-rx not running ($BOARD)." >&2
    echo "  - board powered for ~1.5 min after plugging in? (the service starts at boot)" >&2
    echo "  - PC USB Ethernet adapter still has 192.168.2.1/24? (README: 보드에서 실행)" >&2
    exit 1
fi

PIDS=()
cleanup() {
    kill "${PIDS[@]}" 2> /dev/null || true
    taskkill.exe /IM pcap-replay.exe /F > /dev/null 2>&1 || true
    wait 2> /dev/null || true
}
trap cleanup EXIT INT TERM

WSL_IP=$(hostname -I | awk '{print $1}')
"$RX" --udp 0.0.0.0:$PORT --ros2 --status 10 > /dev/null 2>&1 & PIDS+=($!)
ssh "$BOARD" 'journalctl -u lidar-rx -f -o cat -n 3' & PIDS+=($!)
sleep 1
(cd "$WIN_DIR" && ./pcap-replay.exe vlp16.pcap --to "$BOARD_IP:2368" --pace sensor --loops 100 \
    --echo 0.0.0.0:$PORT --echo-relay "$WSL_IP:$PORT" "$@" > /dev/null) & PIDS+=($!)
rviz2 -d tools/rviz/lidar.rviz 2> >(grep -v 'Stereo is NOT SUPPORTED' >&2)
