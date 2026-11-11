#!/bin/bash
# 데모: 녹화된 VLP-16 패킷 → lidar-rx → ROS 2 → RViz. 세 프로그램을 한 번에 띄운다.
#
#   pcap-replay (센서 대신, 끝없이 반복) ─UDP 127.0.0.1:2368─► lidar-rx --ros2 ─/velodyne_points─► rviz2
#
#   tools/rviz_demo.sh                    원래 센서 간격대로
#   tools/rviz_demo.sh --drift-ppm 100    센서 시계를 100 ppm 느리게 → lidar-rx 상태 줄의 clock이 +100 ppm으로 따라감
#
# 뒤에 준 인자는 pcap-replay에 그대로 넘긴다. RViz 창을 닫으면 나머지 둘도 같이 끝난다.
# RViz 설정(고정 좌표계 velodyne, 토픽, 색)은 tools/rviz/lidar.rviz.
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash

RX=target/ros2/release/lidar-rx
# 항상 빌드를 거침 (바뀐 게 없으면 바로 끝남). "없을 때만 빌드"로 두면 코드를 고친 뒤에도 옛 실행 파일로 돈다
tools/build_rx_ros2.sh -q
cargo build -q --release -p pcap-replay
[ -f data/vlp16.pcap ] || tools/fetch_data.sh

"$RX" --udp 127.0.0.1:2368 --ros2 --status 2 &
RX_PID=$!
sleep 1
# --pace sensor: 반복할 때 타임스탬프가 끊김 없이 이어짐 (수신 쪽 시계 추정이 매번 다시 시작하지 않음)
target/release/pcap-replay data/vlp16.pcap --pace sensor --loops 0 "$@" > /dev/null 2>&1 &
RP_PID=$!
trap 'kill $RP_PID $RX_PID 2>/dev/null; wait 2>/dev/null' EXIT

rviz2 -d tools/rviz/lidar.rviz 2> >(grep -v 'Stereo is NOT SUPPORTED' >&2)
